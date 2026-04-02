"""
CTRE Phoenix 6 hoot log analyzer.

Detects issues from CTRE device telemetry (TalonFX, CANcoder, Pigeon2, CANdle):
  - TalonFX: hardware faults, undervoltage, over-temperature, stator current spikes,
    boot-during-enable, stall current limiting
  - CANcoder: bad magnet, hardware faults
  - Pigeon2: hardware faults, sensor saturation, gyro drift, collisions/impacts,
    tilt/tip detection, thermal drift, magnetometer interference, supply voltage,
    no-motion validation
  - CANdle: short circuit, thermal faults
  - All devices: sticky fault summary at startup
"""

from signals import get, find_channels, find_threshold_spans, find_drops, derivative, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO
from device_config import DeviceConfig, CONTROL_MODE_POSITION, CONTROL_MODE_MOTION_PROFILE
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

# Pigeon2 thresholds
_PIGEON_YAW_DRIFT_DEG_S = 0.5        # yaw rate while stationary (deg/s)
_PIGEON_YAW_DRIFT_MIN_DUR = 3.0      # must persist to filter noise
_PIGEON_ACCEL_IMPACT_G = 1.99        # collision detection (g)
_PIGEON_ACCEL_WARN_G = 1.5           # hard-stop warning (g)
_PIGEON_TILT_ERR_DEG = 15.0          # severe tilt
_PIGEON_TILT_WARN_DEG = 10.0         # moderate tilt
_PIGEON_TILT_MIN_DUR = 0.5           # tilt must persist
_PIGEON_TEMP_WARN_C = 50.0           # IMU temperature warning
_PIGEON_TEMP_MIN_DUR = 5.0
_PIGEON_MAG_FIELD_WARN_UT = 100.0    # magnetic interference threshold (µT)
_PIGEON_MAG_FIELD_MIN_DUR = 1.0
_PIGEON_SUPPLY_VOLTAGE_WARN = 10.0   # Pigeon supply voltage sag
_PIGEON_SUPPLY_VOLTAGE_DUR = 0.5


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
                        if duration > 2.0 or peak < 7.0:
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

            # Limit switch engagement
            for direction in ("Forward", "Reverse"):
                limit_series = channels.get(f"{prefix}/{direction}Limit", [])
                if not limit_series:
                    continue
                closed_events = [(t, v) for t, v in limit_series
                                 if isinstance(v, str) and v == "Closed"]
                if closed_events:
                    # Group into spans (gap > 1s = new event)
                    spans = []
                    span_start = closed_events[0][0]
                    prev_t = span_start
                    for t, _ in closed_events[1:]:
                        if t - prev_t > 1.0:
                            spans.append((span_start, prev_t))
                            span_start = t
                        prev_t = t
                    spans.append((span_start, prev_t))

                    for start, end in spans:
                        dur = end - start
                        if dur > 0.1:
                            issues.append(Issue(
                                severity=SEVERITY_INFO,
                                subsystem=subsystem,
                                message=f"{label} {direction.lower()} limit switch engaged "
                                        f"{fmt_time(start)}–{fmt_time(end)} ({dur:.1f}s)",
                                time_start=start, time_end=end, detail=detail,
                            ))
                        else:
                            issues.append(Issue(
                                severity=SEVERITY_INFO,
                                subsystem=subsystem,
                                message=f"{label} {direction.lower()} limit switch hit "
                                        f"at {fmt_time(start)}",
                                time_start=start, detail=detail,
                            ))

            # Duty cycle saturation: motor maxed out at ±1.0
            _DUTY_SAT_THRESH = 0.98
            _DUTY_SAT_MIN_DUR = 1.0
            duty_series = get(channels, f"{prefix}/DutyCycle")
            if duty_series:
                abs_duty = [(t, abs(v)) for t, v in duty_series]
                sat_spans = find_threshold_spans(
                    abs_duty, _DUTY_SAT_THRESH, _DUTY_SAT_MIN_DUR, above=True)
                for start, end, peak in sat_spans:
                    # Only flag if the motor was actually enabled (ignore disabled periods)
                    if game_timeline:
                        mode = get_game_mode_at(game_timeline, start)
                        if mode == MODE_DISABLED:
                            continue
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} duty cycle saturated "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s) — motor at full output",
                        time_start=start, time_end=end, detail=detail,
                    ))

            # Motor position following error (closed-loop)
            # Only check for devices configured as position or motion_profile control.
            # Velocity-controlled motors report meaningless position errors.
            _POS_ERROR_WARN = 0.5     # rotations
            _POS_ERROR_ERR = 2.0      # rotations
            _POS_ERROR_MIN_DUR = 0.5
            device_control_mode = config.control_mode(key) if config else ""
            if device_control_mode in (CONTROL_MODE_POSITION, CONTROL_MODE_MOTION_PROFILE):
                pos_error = get(channels, f"{prefix}/PIDPosition_ClosedLoopError")
                if pos_error:
                    abs_error = [(t, abs(v)) for t, v in pos_error]
                    active_error = [(t, v) for t, v in abs_error if v > 0.01]
                    if active_error:
                        err_spans = find_threshold_spans(
                            active_error, _POS_ERROR_ERR,
                            _POS_ERROR_MIN_DUR, above=True)
                        for start, end, peak in err_spans:
                            issues.append(Issue(
                                severity=SEVERITY_ERR,
                                subsystem=subsystem,
                                message=f"{label} position following error "
                                        f"{fmt_time(start)}–{fmt_time(end)} "
                                        f"({end - start:.1f}s), peak {peak:.2f} rot — "
                                        f"mechanism may be jammed or obstructed",
                                time_start=start, time_end=end, detail=detail,
                            ))

                        warn_spans = find_threshold_spans(
                            active_error, _POS_ERROR_WARN,
                            _POS_ERROR_MIN_DUR, above=True)
                        for start, end, peak in warn_spans:
                            already = any(s <= start and end <= e + 0.5
                                          for s, e, _ in err_spans)
                            if not already:
                                issues.append(Issue(
                                    severity=SEVERITY_WARN,
                                    subsystem=subsystem,
                                    message=f"{label} position following error "
                                            f"{fmt_time(start)}–{fmt_time(end)} "
                                            f"({end - start:.1f}s), peak {peak:.2f} rot",
                                    time_start=start, time_end=end, detail=detail,
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

        # --- Pigeon2-specific checks ---
        if device_type == "Pigeon2":

            # -- Gyro drift detection --
            # Look for yaw changing while the Pigeon reports no motion
            yaw_series = get(channels, f"{prefix}/Yaw")
            no_motion_series = get(channels, f"{prefix}/NoMotionCount")
            if yaw_series and no_motion_series:
                yaw_rate = derivative(yaw_series)
                # Build a set of times when NoMotionCount > 0 (robot stationary)
                stationary_times = {round(t, 2) for t, v in no_motion_series if v > 0}
                # Yaw rate while stationary
                drift_series = [(t, abs(v)) for t, v in yaw_rate
                                if round(t, 2) in stationary_times]
                drift_spans = find_threshold_spans(
                    drift_series, _PIGEON_YAW_DRIFT_DEG_S,
                    _PIGEON_YAW_DRIFT_MIN_DUR, above=True)
                for start, end, peak in drift_spans:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} gyro drift while stationary "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.2f}°/s — "
                                f"check calibration or thermal stability",
                        time_start=start, time_end=end,
                    ))

            # -- Accumulated gyro drift --
            # Large accumulated gyro Z change while stationary indicates integration error
            accum_z = get(channels, f"{prefix}/AccumGyroZ")
            if accum_z and len(accum_z) > 1:
                total_drift = abs(accum_z[-1][1] - accum_z[0][1])
                log_dur = accum_z[-1][0] - accum_z[0][0]
                if log_dur > 10.0:
                    drift_rate = total_drift / log_dur
                    if drift_rate > _PIGEON_YAW_DRIFT_DEG_S:
                        issues.append(Issue(
                            severity=SEVERITY_WARN,
                            subsystem=subsystem,
                            message=f"{label} accumulated gyro Z drift: "
                                    f"{total_drift:.1f}° over {log_dur:.0f}s "
                                    f"({drift_rate:.2f}°/s avg)",
                            time_start=accum_z[0][0], time_end=accum_z[-1][0],
                        ))

            # -- Collision / impact detection --
            # Collect all impact/bump events across axes, then group into bursts
            all_impacts = []   # (time, peak_g, axis, severity)
            for axis in ("X", "Y", "Z"):
                accel_series = get(channels, f"{prefix}/Acceleration{axis}")
                if not accel_series:
                    continue
                # Z axis has ~1g from gravity, so subtract it
                if axis == "Z":
                    accel_series = [(t, abs(v) - 1.0) for t, v in accel_series]
                else:
                    accel_series = [(t, abs(v)) for t, v in accel_series]

                severe = find_threshold_spans(
                    accel_series, _PIGEON_ACCEL_IMPACT_G, 0.0, above=True)
                for start, end, peak in severe:
                    all_impacts.append((start, peak, axis, SEVERITY_ERR))

                moderate = find_threshold_spans(
                    accel_series, _PIGEON_ACCEL_WARN_G, 0.0, above=True)
                for start, end, peak in moderate:
                    # Skip if already captured as severe
                    already_severe = any(abs(start - s) < 0.05
                                         for s, _, _, sev in all_impacts
                                         if sev == SEVERITY_ERR)
                    if already_severe:
                        continue
                    all_impacts.append((start, peak, axis, SEVERITY_WARN))

            # Group impacts into bursts (gap > 2s = new burst)
            if all_impacts:
                all_impacts.sort(key=lambda x: x[0])
                bursts = []
                cur_burst = [all_impacts[0]]
                for evt in all_impacts[1:]:
                    if evt[0] - cur_burst[-1][0] > 2.0:
                        bursts.append(cur_burst)
                        cur_burst = [evt]
                    else:
                        cur_burst.append(evt)
                bursts.append(cur_burst)

                for burst in bursts:
                    t_start = burst[0][0]
                    t_end = burst[-1][0]
                    peak_g = max(e[1] for e in burst)
                    axes = sorted(set(e[2] for e in burst))
                    has_severe = any(e[3] == SEVERITY_ERR for e in burst)
                    sev = SEVERITY_ERR if has_severe else SEVERITY_WARN
                    count = len(burst)
                    ax_str = "/".join(axes)
                    if count == 1:
                        msg = (f"{label} {'IMPACT' if has_severe else 'hard stop/bump'} "
                               f"({ax_str}) at {fmt_time(t_start)}, "
                               f"peak {peak_g:.2f}g")
                    else:
                        msg = (f"{label} {'IMPACTS' if has_severe else 'hard stops/bumps'} "
                               f"({ax_str}) {fmt_time(t_start)}–{fmt_time(t_end)}, "
                               f"×{count} events, peak {peak_g:.2f}g")
                    issues.append(Issue(
                        severity=sev,
                        subsystem=subsystem,
                        message=msg,
                        time_start=t_start, time_end=t_end,
                    ))

            # -- Tilt / tip detection --
            pitch_series = get(channels, f"{prefix}/Pitch")
            roll_series = get(channels, f"{prefix}/Roll")
            for angle_name, angle_series in [("pitch", pitch_series), ("roll", roll_series)]:
                if not angle_series:
                    continue
                abs_angle = [(t, abs(v)) for t, v in angle_series]

                # ERR: severe tilt
                severe = find_threshold_spans(
                    abs_angle, _PIGEON_TILT_ERR_DEG,
                    _PIGEON_TILT_MIN_DUR, above=True)
                for start, end, peak in severe:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=f"{label} severe {angle_name} tilt "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.1f}°",
                        time_start=start, time_end=end,
                    ))

                # WARN: moderate tilt (skip spans already covered by severe)
                moderate = find_threshold_spans(
                    abs_angle, _PIGEON_TILT_WARN_DEG,
                    _PIGEON_TILT_MIN_DUR, above=True)
                for start, end, peak in moderate:
                    already = any(s <= start and end <= e + 0.5
                                  for s, e, _ in severe)
                    if not already:
                        issues.append(Issue(
                            severity=SEVERITY_WARN,
                            subsystem=subsystem,
                            message=f"{label} {angle_name} tilt "
                                    f"{fmt_time(start)}–{fmt_time(end)} "
                                    f"({end - start:.1f}s), peak {peak:.1f}°",
                            time_start=start, time_end=end,
                        ))

            # -- Temperature monitoring --
            temp_series = get(channels, f"{prefix}/Temperature")
            if temp_series:
                hot = find_threshold_spans(
                    temp_series, _PIGEON_TEMP_WARN_C,
                    _PIGEON_TEMP_MIN_DUR, above=True)
                for start, end, peak in hot:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} IMU over-temperature "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.0f}°C — "
                                f"gyro accuracy degrades with heat",
                        time_start=start, time_end=end,
                    ))

            # -- Magnetometer interference --
            for axis in ("X", "Y", "Z"):
                mag_series = get(channels, f"{prefix}/RawMagneticField{axis}")
                if not mag_series:
                    continue
                abs_mag = [(t, abs(v)) for t, v in mag_series]
                interference = find_threshold_spans(
                    abs_mag, _PIGEON_MAG_FIELD_WARN_UT,
                    _PIGEON_MAG_FIELD_MIN_DUR, above=True)
                for start, end, peak in interference:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} magnetic interference ({axis}) "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), peak {peak:.0f} µT — "
                                f"heading accuracy may be degraded",
                        time_start=start, time_end=end,
                    ))

            # -- Supply voltage monitoring --
            pigeon_supply = get(channels, f"{prefix}/SupplyVoltage")
            if pigeon_supply:
                sags = find_threshold_spans(
                    pigeon_supply, _PIGEON_SUPPLY_VOLTAGE_WARN,
                    _PIGEON_SUPPLY_VOLTAGE_DUR, above=False)
                for start, end, peak in sags:
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem=subsystem,
                        message=f"{label} supply voltage sag "
                                f"{fmt_time(start)}–{fmt_time(end)} "
                                f"({end - start:.1f}s), min {peak:.2f}V",
                        time_start=start, time_end=end,
                    ))

            # -- No-motion validation --
            # Flag if NoMotionCount stays at 0 for entire log (sensor may be misconfigured)
            if no_motion_series:
                ever_stationary = any(v > 0 for _, v in no_motion_series)
                if not ever_stationary and len(no_motion_series) > 10:
                    issues.append(Issue(
                        severity=SEVERITY_INFO,
                        subsystem=subsystem,
                        message=f"{label} NoMotionCount never incremented — "
                                f"no-motion detection may be disabled or "
                                f"sensor is experiencing constant vibration",
                        time_start=no_motion_series[0][0],
                    ))

    return issues
