"""
CTRE Phoenix 6 hoot log analyzer.

Detects issues from CTRE device telemetry (TalonFX, CANcoder, Pigeon2, CANdle):
  - TalonFX: hardware faults, undervoltage, over-temperature, stator current spikes,
    boot-during-enable, stall current limiting
  - CANcoder: bad magnet, hardware faults
  - Pigeon2: hardware faults, sensor saturation
  - CANdle: short circuit, thermal faults
  - All devices: sticky fault summary at startup
"""

from signals import get, find_channels, find_threshold_spans, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO
from device_config import DeviceConfig
from parser import get_game_mode_at, MODE_DISABLED

DEFAULT_SUBSYSTEM = "HOOT"

# Critical fault signal names (value > 0.5 means fault active)
_CRITICAL_FAULTS = [
    "Fault_Hardware",
    "Fault_BridgeBrownout",
    "Fault_BootDuringEnable",
]

_WARN_FAULTS = [
    "Fault_Undervoltage",
    "Fault_DeviceTemp",
    "Fault_ProcTemp",
    "Fault_OverSupplyV",
    "Fault_UnstableSupplyV",
    "Fault_BadMagnet",
    "Fault_ShortCircuit",
    "Fault_Thermal",
    "Fault_SaturatedAccelerometer",
    "Fault_SaturatedGyroscope",
    "Fault_SaturatedMagnetometer",
]

# Faults that are only meaningful when the robot is enabled (not DISABLED)
_ENABLED_ONLY_FAULTS = [
    "Fault_StatorCurrLimit",
    "Fault_SupplyCurrLimit",
]

# Thresholds
_TEMP_WARN_C = 80.0
_TEMP_MIN_DUR = 2.0
_SUPPLY_VOLTAGE_WARN = 10.0
_SUPPLY_VOLTAGE_DUR = 0.5
_SUPPLY_VOLTAGE_STARVED = 7.0
_SUPPLY_VOLTAGE_STARVED_DUR = 5.0
_STATOR_CURRENT_WARN = 80.0
_STATOR_CURRENT_DUR = 0.5


def _get_devices(channels: dict) -> dict[str, list[str]]:
    """Group Phoenix6 channels by device. Returns {device_prefix: [signal_names]}."""
    devices = {}
    for name in channels:
        if not name.startswith("Phoenix6/"):
            continue
        parts = name.split("/")
        if len(parts) < 3:
            continue
        prefix = f"Phoenix6/{parts[1]}"
        signal = parts[2]
        if prefix not in devices:
            devices[prefix] = []
        devices[prefix].append(signal)
    return devices


def _device_key(device_prefix: str) -> str:
    """Convert Phoenix6 prefix to device config key."""
    return f"CTRE:{device_prefix.replace('Phoenix6/', '')}"


def analyze_hoot(channels: dict, can_map: dict = None, bus_name: str = "",
                 config: DeviceConfig = None, game_timeline: list = None) -> list[Issue]:
    issues = []
    can_map = can_map or {}
    config = config
    devices = _get_devices(channels)
    bus_tag = f" [{bus_name}]" if bus_name else ""

    for prefix, signals in sorted(devices.items()):
        device_type = prefix.split("/")[1].split("-")[0] if "/" in prefix else ""
        key = _device_key(prefix)
        short = prefix.replace("Phoenix6/", "")

        # Resolve label and subsystem from config
        if config:
            motor_name = config.label(key, short)
            subsystem = config.subsystem(key, DEFAULT_SUBSYSTEM)
        else:
            motor_name = can_map.get(key, short)
            subsystem = DEFAULT_SUBSYSTEM
        label = motor_name + bus_tag  # full label for messages
        # Only actual motors appear in MOTOR STATUS table
        detail = motor_name if device_type == "TalonFX" else None

        # --- Active critical faults ---
        for fault_name in _CRITICAL_FAULTS:
            if fault_name not in signals:
                continue
            series = get(channels, f"{prefix}/{fault_name}")
            active = [(t, v) for t, v in series if v > 0.5]
            if active:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem=subsystem,
                    message=f"{label} {fault_name.replace('Fault_', '')}"
                            f" at {fmt_time(active[0][0])}",
                    time_start=active[0][0],
                    detail=detail,
                ))

        # --- Active warning faults ---
        for fault_name in _WARN_FAULTS:
            if fault_name not in signals:
                continue
            series = get(channels, f"{prefix}/{fault_name}")
            active = [(t, v) for t, v in series if v > 0.5]
            if active:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=f"{label} {fault_name.replace('Fault_', '')}"
                            f" at {fmt_time(active[0][0])}",
                    time_start=active[0][0],
                    detail=detail,
                ))

        # --- Enabled-only faults (ignore when DISABLED) ---
        for fault_name in _ENABLED_ONLY_FAULTS:
            if fault_name not in signals:
                continue
            series = get(channels, f"{prefix}/{fault_name}")
            if game_timeline:
                active = [(t, v) for t, v in series
                          if v > 0.5 and get_game_mode_at(game_timeline, t) != MODE_DISABLED]
            else:
                active = [(t, v) for t, v in series if v > 0.5]
            if active:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=f"{label} {fault_name.replace('Fault_', '')}"
                            f" at {fmt_time(active[0][0])}",
                    time_start=active[0][0],
                    detail=detail,
                ))

        # --- Sticky faults at startup ---
        sticky_names = [s for s in signals if s.startswith("StickyFault_") and s != "StickyFaultField"]
        active_sticky = []
        for sname in sticky_names:
            series = get(channels, f"{prefix}/{sname}")
            if series and series[0][1] > 0.5:
                active_sticky.append(sname.replace("StickyFault_", ""))
        if active_sticky:
            issues.append(Issue(
                severity=SEVERITY_INFO,
                subsystem=subsystem,
                message=f"{label} sticky faults at startup: {', '.join(active_sticky)}",
                time_start=0.0,
                detail=detail,
            ))

        # --- TalonFX-specific checks ---
        if device_type == "TalonFX":
            # Over-temperature
            temp_series = get(channels, f"{prefix}/DeviceTemp")
            if temp_series:
                hot = find_threshold_spans(temp_series, _TEMP_WARN_C, _TEMP_MIN_DUR, above=True)
                for start, end, peak in hot:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} over-temperature "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.0f}°C",
                        time_start=start, time_end=end, detail=detail,
                    ))

            # Power starved: sustained very low voltage (< 7V for > 5s)
            supply_series = get(channels, f"{prefix}/SupplyVoltage")
            if supply_series:
                starved = find_threshold_spans(supply_series, _SUPPLY_VOLTAGE_STARVED,
                                               _SUPPLY_VOLTAGE_STARVED_DUR, above=False)
                for start, end, peak in starved:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=f"{label} POWER STARVED "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), min {peak:.2f} V — "
                                f"check power wiring to this motor",
                        time_start=start, time_end=end, detail=detail,
                    ))

                # Supply voltage sag (only report sags not already covered by starved)
                sags = find_threshold_spans(supply_series, _SUPPLY_VOLTAGE_WARN,
                                            _SUPPLY_VOLTAGE_DUR, above=False)
                for start, end, peak in sags:
                    # Skip if this sag overlaps a power-starved span
                    already = any(s <= start and end <= e + 1.0
                                  for s, e, _ in starved)
                    if not already:
                        duration = end - start
                        if duration > 1.0 or peak < 7.0:
                            sev = SEVERITY_ERR
                        elif peak < 9.0:
                            sev = SEVERITY_WARN
                        else:
                            sev = SEVERITY_INFO
                        issues.append(Issue(
                            severity=sev,
                            subsystem=subsystem,
                            message=f"{label} supply voltage sag "
                                    f"{fmt_time(start)}–{fmt_time(end)} "
                                    f"({end - start:.1f}s), min {peak:.2f} V",
                            time_start=start, time_end=end, detail=detail,
                        ))

            # Motor reboot detection via BootDuringEnable
            boot_series = get(channels, f"{prefix}/Fault_BootDuringEnable")
            if boot_series:
                reboots = [(t, v) for t, v in boot_series if v > 0.5]
                if reboots:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=f"{label} MOTOR REBOOTED at {fmt_time(reboots[0][0])} — "
                                f"device reset while robot was enabled",
                        time_start=reboots[0][0], detail=detail,
                    ))

            # Stator current spikes
            stator_series = get(channels, f"{prefix}/StatorCurrent")
            if stator_series:
                spikes = find_threshold_spans(stator_series, _STATOR_CURRENT_WARN,
                                              _STATOR_CURRENT_DUR, above=True)
                for start, end, peak in spikes:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} stator current spike "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.0f} A",
                        time_start=start, time_end=end, detail=detail,
                    ))

            # Hardware fault detection
            hw_series = get(channels, f"{prefix}/Fault_Hardware")
            if hw_series:
                hw_active = [(t, v) for t, v in hw_series if v > 0.5]
                if hw_active:
                    first_t = hw_active[0][0]
                    cleared = any(v < 0.5 for t, v in hw_series if t > first_t)
                    if cleared:
                        msg = f"{label} HARDWARE FAULT at {fmt_time(first_t)}"
                    else:
                        msg = (f"{label} HARDWARE FAULT at {fmt_time(first_t)} — "
                               f"active for remainder of log, never cleared")
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=msg,
                        time_start=first_t, detail=detail,
                    ))

            # Motor output loss: motor voltage drops from sustained drive to 0
            # while supply is healthy. Must distinguish from normal PWM switching
            # by requiring: high prior voltage (>5V) AND output stays near 0 for
            # multiple consecutive samples (>50ms).
            motor_v_series = get(channels, f"{prefix}/MotorVoltage")
            if motor_v_series and supply_series:
                supply_map = dict(supply_series)
                supply_times = sorted(supply_map)
                _MV_PRIOR_THRESH = 5.0   # prior voltage must be sustained drive
                _MV_ZERO_THRESH = 0.5    # consider "off" below this
                _MV_ZERO_SAMPLES = 5     # must stay off for this many consecutive samples

                i = 0
                while i < len(motor_v_series) - _MV_ZERO_SAMPLES:
                    t_cur, mv_cur = motor_v_series[i]

                    # Look for transition from high voltage to zero
                    if abs(mv_cur) > _MV_PRIOR_THRESH:
                        # Check if the next N samples are all near zero
                        all_zero = True
                        for j in range(1, _MV_ZERO_SAMPLES + 1):
                            if i + j >= len(motor_v_series):
                                all_zero = False
                                break
                            if abs(motor_v_series[i + j][1]) > _MV_ZERO_THRESH:
                                all_zero = False
                                break

                        if all_zero:
                            drop_t = motor_v_series[i + 1][0]
                            sv = None
                            for st in supply_times:
                                if st >= drop_t:
                                    sv = supply_map[st]
                                    break
                            if sv is not None and sv > 8.0:
                                issues.append(Issue(
                                    severity=SEVERITY_ERR,
                                    subsystem=subsystem,
                                    message=(f"{label} MOTOR OUTPUT LOST at {fmt_time(drop_t)} — "
                                             f"voltage dropped from {mv_cur:.1f}V to 0V "
                                             f"while supply was {sv:.1f}V"),
                                    time_start=drop_t, detail=detail,
                                ))
                                break  # only report first occurrence
                    i += 1

            # Static brake disabled (controller cannot hold motor)
            brake_series = get(channels, f"{prefix}/Fault_StaticBrakeDisabled")
            if brake_series:
                brake_active = [(t, v) for t, v in brake_series if v > 0.5]
                if brake_active:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=(f"{label} STATIC BRAKE DISABLED at "
                                 f"{fmt_time(brake_active[0][0])} — "
                                 f"controller cannot hold motor position"),
                        time_start=brake_active[0][0], detail=detail,
                    ))

        # --- CANcoder-specific checks ---
        if device_type == "CANcoder":
            magnet_series = channels.get(f"{prefix}/MagnetHealth", [])
            bad = [(t, v) for t, v in magnet_series if isinstance(v, str) and v != "Magnet_Green"]
            if bad:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=f"{label} magnet health: {bad[0][1]} at {fmt_time(bad[0][0])}",
                    time_start=bad[0][0], detail=detail,
                ))

    return issues
