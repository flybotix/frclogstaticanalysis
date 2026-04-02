"""
Mechanical issue detection:
  - Shooter velocity error (flywheel / backwheel)
  - Shooter tracking error (firing while off-target)
  - Intake motor stall
  - Intake roller imbalance (left vs right divergence)
  - Turret position drift
  - Swerve heading error, translation drift, pose jumps
  - Indexer jam detection
  - Vision system dropout
"""

import math
from signals import (
    get, find_channels, find_threshold_spans, find_true_spans,
    subtract, abs_series, derivative, is_near_zero, find_pose_jumps,
    fmt_time, fmt_duration
)
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO


def _motor_label(channel_name: str, can_map: dict) -> str:
    """
    Extract SUBSYSTEM/MotorName from a channel path and optionally append CAN ID.
    e.g. '/RealOutputs/CustomLogs/SHOOTER/Flywheel Real Vel' -> 'SHOOTER/Flywheel'
    """
    parts = channel_name.split("/")
    # Find 'CustomLogs' index and take the next two segments
    try:
        idx = parts.index("CustomLogs")
        subsys = parts[idx + 1]
        motor = parts[idx + 2] if len(parts) > idx + 2 else ""
        # Strip trailing signal name words to get motor name only
        label = f"{subsys}/{motor}" if motor else subsys
    except (ValueError, IndexError):
        label = channel_name.split("/")[-2] if "/" in channel_name else channel_name

    can_id = can_map.get(label)
    if can_id is not None:
        return f"{label} (CAN {can_id})"
    return label


def _strip_motor_name(channel_name: str) -> str:
    """Extract just the motor name component (e.g. 'Flywheel', 'Backwheel')."""
    parts = channel_name.split("/")
    try:
        idx = parts.index("CustomLogs")
        return parts[idx + 2] if len(parts) > idx + 2 else parts[-1]
    except (ValueError, IndexError):
        return parts[-1]


def analyze_mechanical(channels: dict, can_map: dict = None) -> list[Issue]:
    issues = []
    can_map = can_map or {}

    # ------------------------------------------------------------------ SHOOTER
    # Pair up Set Vel / Real Vel channels by motor name
    set_vel_chs = find_channels(channels, "SHOOTER", "Set Vel")
    real_vel_chs = find_channels(channels, "SHOOTER", "Real Vel")

    for set_ch in set_vel_chs:
        motor = _strip_motor_name(set_ch)
        # Find matching Real Vel channel
        real_ch = next((c for c in real_vel_chs if _strip_motor_name(c) == motor), None)
        if real_ch is None:
            continue

        set_v = get(channels, set_ch)
        real_v = get(channels, real_ch)
        label = _motor_label(set_ch, can_map)

        # Only analyze when setpoint is active (> 50 RPM)
        active_set = [(t, v) for t, v in set_v if abs(v) > 50]
        if not active_set:
            continue

        error = abs_series(subtract(real_v, set_v))
        # Only look at times when setpoint is nonzero
        active_times = {round(t, 3) for t, _ in active_set}
        error_active = [(t, v) for t, v in error
                        if any(abs(t - at) < 0.05 for at in active_times)]

        spans = find_threshold_spans(error_active, threshold=200.0, min_duration=0.5, above=True)
        for start, end, peak in spans:
            dur = end - start
            sev = SEVERITY_ERR if dur > 2.0 else SEVERITY_WARN
            issues.append(Issue(
                severity=sev,
                subsystem="SHOOTER",
                message=(
                    f"{label}  velocity error: "
                    f"{fmt_time(start)} – {fmt_time(end)} ({dur:.1f}s), peak Δ {peak:.0f} RPM"
                ),
                time_start=start,
                time_end=end,
                detail=label,
            ))

    # ------------------------------------------------------------------ INTAKE
    voltage_chs = find_channels(channels, "Intake", "Voltage") + \
                  find_channels(channels, "INTAKE", "Voltage") + \
                  find_channels(channels, "Intake", "Volt") + \
                  find_channels(channels, "INTAKE", "Volt")
    rotat_chs = find_channels(channels, "Intake", "Rotat") + \
                find_channels(channels, "INTAKE", "Rotat") + \
                find_channels(channels, "Intake", "RPM") + \
                find_channels(channels, "INTAKE", "RPM")

    for volt_ch in set(voltage_chs):
        motor = _strip_motor_name(volt_ch)
        rotat_ch = next((c for c in set(rotat_chs) if _strip_motor_name(c) == motor), None)
        if rotat_ch is None:
            # Try any rotation channel for this subsystem
            rotat_ch = rotat_chs[0] if rotat_chs else None
        if rotat_ch is None:
            continue

        volt = get(channels, volt_ch)
        rotat = get(channels, rotat_ch)
        label = _motor_label(volt_ch, can_map)

        # Active: voltage > 4 V
        high_volt = [(t, v) for t, v in volt if v > 4.0]
        if not high_volt:
            continue

        rotat_deriv = derivative(rotat)
        near_zero = is_near_zero(rotat_deriv, threshold=0.5)
        # Build bool series aligned to high_volt times
        stall_series = []
        nz_dict = {round(t, 2): v for t, v in near_zero}
        for t, _ in high_volt:
            key = round(t, 2)
            if key in nz_dict and nz_dict[key]:
                stall_series.append((t, True))
            else:
                stall_series.append((t, False))

        stall_spans = find_true_spans(stall_series, min_duration=0.5)
        for start, end in stall_spans:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="INTAKE",
                message=(
                    f"{label}  stall: "
                    f"{fmt_time(start)} – {fmt_time(end)} ({end - start:.1f}s), "
                    f"voltage applied, Δrotation ≈ 0"
                ),
                time_start=start,
                time_end=end,
                detail=label,
            ))

    # ------------------------------------------------------------------ TURRET
    # Only look at actual motor angle channels, not target/commanded angle channels
    angle_chs = [c for c in
                 find_channels(channels, "TURRET", "Angle") +
                 find_channels(channels, "Turret", "Angle")
                 if not any(x in c.lower() for x in ("target", "setpoint", "cmd", "sotm", "desired"))]
    volt_chs = find_channels(channels, "TURRET", "Voltage") + \
               find_channels(channels, "Turret", "Voltage") + \
               find_channels(channels, "TURRET", "Volt") + \
               find_channels(channels, "Turret", "Volt")

    for angle_ch in set(angle_chs):
        motor = _strip_motor_name(angle_ch)
        volt_ch = next((c for c in set(volt_chs) if _strip_motor_name(c) == motor), None)
        if volt_ch is None:
            volt_ch = volt_chs[0] if volt_chs else None

        label = _motor_label(angle_ch, can_map)
        angle = get(channels, angle_ch)
        angle_deriv = derivative(angle)

        # Drift: angle changing unexpectedly while no voltage commanded
        if volt_ch:
            volt = get(channels, volt_ch)
            low_volt_times = {round(t, 2) for t, v in volt if abs(v) < 0.5}
            drifting = [(t, abs(v)) for t, v in angle_deriv
                        if round(t, 2) in low_volt_times and abs(v) > 5.0]  # >5 deg/s with no voltage
            spans = find_threshold_spans(drifting, threshold=5.0, min_duration=1.0, above=True)
            for start, end, peak in spans:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="TURRET",
                    message=(
                        f"{label}  position drift: "
                        f"{fmt_time(start)} – {fmt_time(end)}, "
                        f"peak {peak:.1f}°/s while voltage ≈ 0"
                    ),
                    time_start=start,
                    time_end=end,
                    detail=label,
                ))

    # ------------------------------------------------------------------ SWERVE
    # Heading error: driveRotate vs TargetSpeeds.omega
    drive_rotate = get(channels, "/RealOutputs/CustomLogs/SWERVE/driveRotate")
    target_speeds_raw = channels.get("/RealOutputs/CustomLogs/SWERVE/TargetSpeeds", [])

    if drive_rotate and target_speeds_raw:
        omega_series = []
        for t, v in target_speeds_raw:
            if isinstance(v, dict) and "omega" in v:
                omega_series.append((t, math.degrees(v["omega"])))

        if omega_series and drive_rotate:
            heading_error = abs_series(subtract(drive_rotate, omega_series))
            spans = find_threshold_spans(heading_error, threshold=10.0, min_duration=0.5, above=True)

            if spans:
                # Group spans into sessions (gap > 30s = new session)
                sessions: list[list] = []
                cur: list = [spans[0]]
                for span in spans[1:]:
                    if span[0] - cur[-1][1] > 30.0:
                        sessions.append(cur)
                        cur = [span]
                    else:
                        cur.append(span)
                sessions.append(cur)

                for session in sessions:
                    t_start = session[0][0]
                    t_end = session[-1][1]
                    peak = max(s[2] for s in session)
                    count = len(session)
                    total_dur = sum(s[1] - s[0] for s in session)
                    issues.append(Issue(
                        severity=SEVERITY_WARN,
                        subsystem="SWERVE",
                        message=(
                            f"Heading error: {fmt_time(t_start)} – {fmt_time(t_end)}, "
                            f"{count} event(s) / {total_dur:.1f}s total, "
                            f"peak {peak:.1f}°/s  [module unknown — inspect all pods]"
                        ),
                        time_start=t_start,
                        time_end=t_end,
                        detail="SWERVE",
                    ))

    # Pose jumps — collapse into bursts
    pose_raw = channels.get("/RealOutputs/CustomLogs/SWERVE/Current Pose", [])
    if pose_raw:
        jumps = find_pose_jumps(pose_raw, jump_m=0.3)
        if jumps:
            # Group into bursts: new burst if gap > 5 seconds
            bursts: list[list] = []
            current: list = [jumps[0]]
            for j in jumps[1:]:
                if j[0] - current[-1][0] > 5.0:
                    bursts.append(current)
                    current = [j]
                else:
                    current.append(j)
            bursts.append(current)

            for burst in bursts:
                count = len(burst)
                t_start = burst[0][0]
                t_end = burst[-1][0]
                avg_dist = sum(d for _, d in burst) / count
                if count == 1:
                    msg = (
                        f"Pose jump at {fmt_time(t_start)}: {avg_dist:.2f} m  "
                        f"[module unknown — inspect all pods]"
                    )
                else:
                    msg = (
                        f"Pose jumps ×{count}: {fmt_time(t_start)} – {fmt_time(t_end)}, "
                        f"avg {avg_dist:.2f} m/jump  "
                        f"[module unknown — inspect all pods]"
                    )
                sev = SEVERITY_ERR if count > 10 else SEVERITY_WARN
                issues.append(Issue(
                    severity=sev,
                    subsystem="SWERVE",
                    message=msg,
                    time_start=t_start,
                    time_end=t_end,
                    detail="SWERVE",
                ))

    # Translation drift: large TranslateX/Y vs commanded speeds
    trans_x = get(channels, "/RealOutputs/CustomLogs/SWERVE/TranslateX")
    trans_y = get(channels, "/RealOutputs/CustomLogs/SWERVE/TranslateY")
    if trans_x and target_speeds_raw:
        vx_cmd = [(t, v["vx"]) for t, v in target_speeds_raw if isinstance(v, dict)]
        vy_cmd = [(t, v["vy"]) for t, v in target_speeds_raw if isinstance(v, dict)]

        x_drift = abs_series(subtract(trans_x, vx_cmd))
        y_drift = abs_series(subtract(trans_y, vy_cmd))

        x_spans = find_threshold_spans(x_drift, threshold=0.3, min_duration=0.5, above=True)
        y_spans = find_threshold_spans(y_drift, threshold=0.3, min_duration=0.5, above=True)

        # Merge X and Y spans that overlap into single events
        all_spans = sorted(
            [(s, e, p, "X") for s, e, p in x_spans] +
            [(s, e, p, "Y") for s, e, p in y_spans],
            key=lambda x: x[0]
        )
        merged = []
        for span in all_spans:
            s, e, p, ax = span
            if merged and s <= merged[-1][1] + 0.5:
                ps, pe, pp, pax = merged[-1]
                axes_set = set(pax.split("+")) | {ax}
                new_ax = "+".join(sorted(axes_set))
                merged[-1] = [ps, max(pe, e), max(pp, p), new_ax]
            else:
                merged.append(list(span))

        # Group merged spans into sessions (gap > 30s = new session)
        sessions: list[list] = []
        cur: list = []
        for span in merged:
            if cur and span[0] - cur[-1][1] > 30.0:
                sessions.append(cur)
                cur = [span]
            else:
                cur.append(span)
        if cur:
            sessions.append(cur)

        for session in sessions:
            t_start = session[0][0]
            t_end = session[-1][1]
            peak = max(s[2] for s in session)
            count = len(session)
            axes: set[str] = set()
            for s in session:
                axes.update(s[3].split("+"))
            ax_str = "+".join(sorted(axes))
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="SWERVE",
                message=(
                    f"Translation drift ({ax_str}): {fmt_time(t_start)} – {fmt_time(t_end)}, "
                    f"{count} span(s), peak {peak:.2f} m/s off-axis  "
                    f"[module unknown — inspect all pods]"
                ),
                time_start=t_start,
                time_end=t_end,
                detail="SWERVE",
            ))

    # Console messages for swerve errors — skip generic loop overrun noise
    console_raw = channels.get("/RealOutputs/Console", [])
    # Only flag console messages that indicate actual swerve faults, not timing/periodic output
    _swerve_fault_kw = ("swerve error", "swerve fault", "swerve exception", "module fault",
                        "steer fault", "azimuth", "drive fault", "can timeout", "motor fault")
    for t, msg in console_raw:
        if not isinstance(msg, str):
            continue
        ml = msg.lower()
        if any(kw in ml for kw in _swerve_fault_kw):
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="SWERVE",
                message=f"Console @ {fmt_time(t)}: {msg.strip()[:120]}",
                time_start=t,
                detail="SWERVE",
            ))

    # ------------------------------------------------------------------ VISION
    # Detect vision system dropouts — gaps in VisionPose data while robot is enabled
    vision_pose_raw = channels.get("/RealOutputs/CustomLogs/Vision/VisionPose", [])
    if vision_pose_raw and len(vision_pose_raw) > 10:
        # Find gaps > 1 second in vision data
        _VISION_GAP_WARN = 1.0
        _VISION_GAP_ERR = 3.0
        for i in range(1, len(vision_pose_raw)):
            t_prev = vision_pose_raw[i - 1][0]
            t_cur = vision_pose_raw[i][0]
            gap = t_cur - t_prev
            if gap >= _VISION_GAP_ERR:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem="SWERVE",
                    message=f"Vision dropout {fmt_time(t_prev)}–{fmt_time(t_cur)} "
                            f"({gap:.1f}s) — no AprilTag detections",
                    time_start=t_prev, time_end=t_cur,
                ))
            elif gap >= _VISION_GAP_WARN:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="SWERVE",
                    message=f"Vision gap {fmt_time(t_prev)}–{fmt_time(t_cur)} "
                            f"({gap:.1f}s)",
                    time_start=t_prev, time_end=t_cur,
                ))

    # Vision ambiguity — high ambiguity means pose estimate is unreliable
    ambiguity = get(channels, "/RealOutputs/CustomLogs/Vision/AmbiguityRatio")
    if ambiguity:
        _AMBIGUITY_WARN = 0.15
        _AMBIGUITY_MIN_DUR = 0.5
        high_ambiguity = find_threshold_spans(
            ambiguity, _AMBIGUITY_WARN, _AMBIGUITY_MIN_DUR, above=True)
        for start, end, peak in high_ambiguity:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="SWERVE",
                message=f"Vision high ambiguity "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s), peak {peak:.2f} — "
                        f"pose estimate unreliable",
                time_start=start, time_end=end,
            ))

    # ------------------------------------------------------------------ INDEXER
    # Detect indexer jams: voltage/RPM commanded but actual RPM near zero
    indexer_output = get(channels, "/RealOutputs/CustomLogs/Indexer/Output RPM")
    indexer_spinner = get(channels, "/RealOutputs/CustomLogs/Indexer/Spinner RPM")
    if indexer_output and indexer_spinner:
        # Look for commanded RPM high but actual RPM near zero
        _INDEXER_CMD_THRESH = 100.0    # commanded RPM threshold
        _INDEXER_ACTUAL_THRESH = 20.0  # actual RPM below this = stalled
        _INDEXER_STALL_DUR = 0.5

        # Build stall series: True when output is commanding but spinner isn't moving
        output_dict = {round(t, 3): v for t, v in indexer_output}
        stall_series = []
        for t, spinner_v in indexer_spinner:
            key = round(t, 3)
            cmd = output_dict.get(key)
            if cmd is not None and abs(cmd) > _INDEXER_CMD_THRESH and abs(spinner_v) < _INDEXER_ACTUAL_THRESH:
                stall_series.append((t, True))
            else:
                stall_series.append((t, False))

        stall_spans = find_true_spans(stall_series, min_duration=_INDEXER_STALL_DUR)
        for start, end in stall_spans:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="INTAKE",
                message=f"Indexer stall/jam "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s) — spinner not moving while commanded",
                time_start=start, time_end=end,
            ))

    # --------------------------------------------------------- INTAKE IMBALANCE
    # Detect left/right intake roller voltage divergence
    left_volt = get(channels, "/RealOutputs/CustomLogs/Intake/Left Roller Motor Voltage")
    right_volt = get(channels, "/RealOutputs/CustomLogs/Intake/Right Roller Motor Voltage")
    if left_volt and right_volt:
        _IMBALANCE_THRESH = 3.0   # voltage difference threshold
        _IMBALANCE_MIN_DUR = 1.0
        diff = abs_series(subtract(left_volt, right_volt))
        # Only check when at least one roller is active (> 2V)
        active_diff = []
        right_dict = {round(t, 3): v for t, v in right_volt}
        for t, d in diff:
            key = round(t, 3)
            left_v = next((v for ts, v in left_volt if round(ts, 3) == key), 0)
            right_v = right_dict.get(key, 0)
            if abs(left_v) > 2.0 or abs(right_v) > 2.0:
                active_diff.append((t, d))

        imbalance_spans = find_threshold_spans(
            active_diff, _IMBALANCE_THRESH, _IMBALANCE_MIN_DUR, above=True)
        for start, end, peak in imbalance_spans:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="INTAKE",
                message=f"Intake roller imbalance "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s), peak ΔV {peak:.1f}V — "
                        f"left/right rollers diverging",
                time_start=start, time_end=end,
            ))

    # -------------------------------------------------------- SHOOTER TRACKING
    # Detect shooter firing while not on target
    tracking_raw = channels.get("/RealOutputs/CustomLogs/SCORING/Tracking", [])
    shooter_speed = get(channels, "/RealOutputs/CustomLogs/SCORING/Shooter Speed")
    if tracking_raw and shooter_speed:
        _SHOOTER_ACTIVE_RPM = 500.0
        # Find times when shooter is spinning but tracking is False
        tracking_dict = {}
        for t, v in tracking_raw:
            tracking_dict[t] = v
        # Build interpolated tracking state
        tracking_times = sorted(tracking_dict.keys())

        def get_tracking_at(t):
            # Find most recent tracking state
            best_t = None
            for tt in tracking_times:
                if tt <= t:
                    best_t = tt
                else:
                    break
            if best_t is not None:
                return tracking_dict[best_t]
            return None

        off_target_shots = []
        for t, rpm in shooter_speed:
            if rpm > _SHOOTER_ACTIVE_RPM:
                tracking = get_tracking_at(t)
                if tracking is False:
                    off_target_shots.append((t, rpm))

        if off_target_shots:
            # Group into bursts (gap > 2s)
            bursts = []
            cur = [off_target_shots[0]]
            for evt in off_target_shots[1:]:
                if evt[0] - cur[-1][0] > 2.0:
                    bursts.append(cur)
                    cur = [evt]
                else:
                    cur.append(evt)
            bursts.append(cur)

            for burst in bursts:
                t_start = burst[0][0]
                t_end = burst[-1][0]
                dur = t_end - t_start
                if dur < 0.3:
                    continue  # skip very brief transients during tracking acquisition
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="SHOOTER",
                    message=f"Shooter active while off-target "
                            f"{fmt_time(t_start)}–{fmt_time(t_end)} "
                            f"({dur:.1f}s)",
                    time_start=t_start, time_end=t_end,
                ))

    return issues
