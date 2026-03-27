"""
REV .revlog analyzer.

Detects per-motor issues from SPARK MAX / SPARK FLEX status frame data:
  - Over-temperature
  - Stall conditions
  - Active hardware faults
  - Sticky faults present at startup
  - Per-motor bus voltage sag
  - Has-reset (motor rebooted mid-session)
"""

from signals import find_threshold_spans, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO
from revlog_parser import get_spark_device_ids, decode_fault_names, HARDWARE_FAULT_MASK
from device_config import DeviceConfig

DEFAULT_SUBSYSTEM = "REVLOG"

# Thresholds
_TEMP_ERR_C       = 80.0
_TEMP_MIN_DUR_S   = 2.0
_VOLTAGE_WARN_V   = 10.0
_VOLTAGE_MIN_DUR  = 0.5
_VOLTAGE_STARVED_V = 7.0
_VOLTAGE_STARVED_DUR = 5.0
_STALL_CURRENT_A  = 0.3   # applied output fraction (not amps) used as proxy
_STALL_VEL_RPM    = 50.0
_STALL_MIN_DUR    = 0.5


def analyze_revlog(channels: dict, can_map: dict = None,
                   config: DeviceConfig = None) -> list[Issue]:
    issues = []
    can_map = can_map or {}

    device_ids = get_spark_device_ids(channels)

    for dev in device_ids:
        prefix = f"/REV/SPARK/{dev}"
        key = f"SPARK:{dev}"

        if config:
            label = config.label(key, f"SPARK #{dev}")
            subsystem = config.subsystem(key, DEFAULT_SUBSYSTEM)
        else:
            label = can_map.get(key, f"SPARK #{dev}")
            subsystem = DEFAULT_SUBSYSTEM

        # --- Over-temperature ---
        temp_series = channels.get(f"{prefix}/MotorTemp", [])
        if temp_series:
            hot_spans = find_threshold_spans(temp_series, threshold=_TEMP_ERR_C,
                                             min_duration=_TEMP_MIN_DUR_S, above=True)
            for start, end, peak in hot_spans:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem=subsystem,
                    message=(f"{label} over-temperature "
                             f"{fmt_time(start)}–{fmt_time(end)} "
                             f"({end - start:.1f}s), peak {peak:.0f}°C"),
                    time_start=start,
                    time_end=end,
                    detail=label,
                ))

        # --- Power starved: sustained very low voltage ---
        voltage_series = channels.get(f"{prefix}/BusVoltage", [])
        if voltage_series:
            starved_spans = find_threshold_spans(voltage_series, threshold=_VOLTAGE_STARVED_V,
                                                 min_duration=_VOLTAGE_STARVED_DUR, above=False)
            for start, end, peak in starved_spans:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem=subsystem,
                    message=(f"{label} POWER STARVED "
                             f"{fmt_time(start)}–{fmt_time(end)} "
                             f"({end - start:.1f}s), min {peak:.2f} V — "
                             f"check power wiring to this motor"),
                    time_start=start,
                    time_end=end,
                    detail=label,
                ))

            # --- Bus voltage sag (skip spans already covered by starved) ---
            sag_spans = find_threshold_spans(voltage_series, threshold=_VOLTAGE_WARN_V,
                                             min_duration=_VOLTAGE_MIN_DUR, above=False)
            for start, end, peak in sag_spans:
                already = any(s <= start and end <= e + 1.0
                              for s, e, _ in starved_spans)
                if not already:
                    if peak < 7.0:
                        sev = SEVERITY_ERR
                    elif peak < 9.0:
                        sev = SEVERITY_WARN
                    else:
                        sev = SEVERITY_INFO
                    issues.append(Issue(
                        severity=sev,
                        subsystem=subsystem,
                        message=(f"{label} bus voltage sag "
                                 f"{fmt_time(start)}–{fmt_time(end)} "
                                 f"({end - start:.1f}s), min {peak:.2f} V"),
                        time_start=start,
                        time_end=end,
                        detail=label,
                    ))

        # --- Stall detection: high applied output, near-zero velocity ---
        applied_series = channels.get(f"{prefix}/AppliedOutput", [])
        vel_series     = channels.get(f"{prefix}/Velocity", [])
        if applied_series and vel_series:
            import bisect
            vel_times = [t for t, _ in vel_series]
            vel_vals = [v for _, v in vel_series]

            # Build a "stall proxy" series: 1.0 if stalling, 0.0 otherwise
            stall_proxy = []
            for t, app in applied_series:
                # Binary search for nearest velocity sample
                idx = bisect.bisect_left(vel_times, t)
                if idx == 0:
                    vel = vel_vals[0]
                elif idx >= len(vel_times):
                    vel = vel_vals[-1]
                elif t - vel_times[idx - 1] <= vel_times[idx] - t:
                    vel = vel_vals[idx - 1]
                else:
                    vel = vel_vals[idx]
                stalling = abs(app) > _STALL_CURRENT_A and abs(vel) < _STALL_VEL_RPM
                stall_proxy.append((t, 1.0 if stalling else 0.0))

            stall_spans = find_threshold_spans(stall_proxy, threshold=0.5,
                                               min_duration=_STALL_MIN_DUR, above=True)
            for start, end, _ in stall_spans:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=(f"{label} stall detected "
                             f"{fmt_time(start)}–{fmt_time(end)} "
                             f"({end - start:.1f}s)"),
                    time_start=start,
                    time_end=end,
                    detail=label,
                ))

        # --- Fault analysis ---
        faults_series  = channels.get(f"{prefix}/Faults", [])
        sticky_series  = channels.get(f"{prefix}/StickyFaults", [])

        # Active hardware faults (any sample)
        if faults_series:
            hw_events = [(t, v) for t, v in faults_series if v & HARDWARE_FAULT_MASK]
            if hw_events:
                first_t, first_mask = hw_events[0]
                names = decode_fault_names(first_mask & HARDWARE_FAULT_MASK)
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem=subsystem,
                    message=(f"{label} hardware fault at {fmt_time(first_t)}: "
                             f"{', '.join(names)}"),
                    time_start=first_t,
                    detail=label,
                ))

        # Sticky faults at startup + has-reset
        if sticky_series:
            first_t, first_mask = sticky_series[0]
            if first_mask != 0:
                names = decode_fault_names(first_mask)
                issues.append(Issue(
                    severity=SEVERITY_INFO,
                    subsystem=subsystem,
                    message=(f"{label} sticky faults at startup: "
                             f"{', '.join(names)}"),
                    time_start=first_t,
                    detail=label,
                ))
            else:
                # Check if has-reset appears later during session
                resets = [(t, v) for t, v in sticky_series if v & (1 << 17)]
                if resets:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=(f"{label} HasReset mid-session at "
                                 f"{fmt_time(resets[0][0])} "
                                 f"(motor rebooted)"),
                        time_start=resets[0][0],
                        detail=label,
                    ))

    return issues
