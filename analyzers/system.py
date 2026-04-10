"""
System health detection:
  - CAN bus utilization (high bus load causing dropped frames)
  - CPU temperature (roboRIO overheating)
  - Code loop overruns (robot code can't keep up with cycle time)
  - GC pauses (Java garbage collection stalls)
  - PhotonVision coprocessor disconnect (NT client drops)
  - PhotonAlerts/warnings (camera errors reported by PhotonVision)
"""

from signals import get, get_bool, find_channels, find_threshold_spans, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO
from parser import get_game_mode_at, MODE_DISABLED

# CAN bus utilization thresholds (fraction 0-1)
_CAN_UTIL_ERR = 0.75
_CAN_UTIL_WARN = 0.60
_CAN_UTIL_MIN_DUR = 1.0

# CPU temperature thresholds (Celsius)
_CPU_TEMP_ERR = 75.0
_CPU_TEMP_WARN = 65.0
_CPU_TEMP_MIN_DUR = 5.0

# Loop timing thresholds (milliseconds)
# Standard FRC loop is 20ms; AdvantageKit logs at 20ms cycle
_LOOP_OVERRUN_WARN_MS = 40.0    # 2x normal cycle
_LOOP_OVERRUN_ERR_MS = 100.0    # 5x normal cycle
_LOOP_OVERRUN_MIN_DUR = 1.0

# GC pause thresholds (milliseconds per cycle)
_GC_PAUSE_WARN_MS = 20.0
_GC_PAUSE_ERR_MS = 50.0


def analyze_system(channels: dict, game_timeline: list = None) -> list[Issue]:
    issues = []

    # --- CAN bus utilization ---
    can_util = get(channels, "/SystemStats/CANBus/Utilization")
    if can_util:
        err_spans = find_threshold_spans(
            can_util, _CAN_UTIL_ERR, _CAN_UTIL_MIN_DUR, above=True)
        for start, end, peak in err_spans:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem="CAN",
                message=f"CAN bus overloaded "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s), peak {peak:.0%} utilization — "
                        f"frames are likely being dropped",
                time_start=start, time_end=end,
            ))

        warn_spans = find_threshold_spans(
            can_util, _CAN_UTIL_WARN, _CAN_UTIL_MIN_DUR, above=True)
        for start, end, peak in warn_spans:
            already = any(s <= start and end <= e + 0.5
                          for s, e, _ in err_spans)
            if not already:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="CAN",
                    message=f"CAN bus high utilization "
                            f"{fmt_time(start)}–{fmt_time(end)} "
                            f"({end - start:.1f}s), peak {peak:.0%}",
                    time_start=start, time_end=end,
                ))

    # --- CPU temperature ---
    cpu_temp = get(channels, "/SystemStats/CPUTempCelsius")
    if cpu_temp:
        err_spans = find_threshold_spans(
            cpu_temp, _CPU_TEMP_ERR, _CPU_TEMP_MIN_DUR, above=True)
        for start, end, peak in err_spans:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem="ELECTRICAL",
                message=f"roboRIO CPU overheating "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s), peak {peak:.0f}°C — "
                        f"may cause loop overruns or shutdown",
                time_start=start, time_end=end,
            ))

        warn_spans = find_threshold_spans(
            cpu_temp, _CPU_TEMP_WARN, _CPU_TEMP_MIN_DUR, above=True)
        for start, end, peak in warn_spans:
            already = any(s <= start and end <= e + 0.5
                          for s, e, _ in err_spans)
            if not already:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem="ELECTRICAL",
                    message=f"roboRIO CPU temperature elevated "
                            f"{fmt_time(start)}–{fmt_time(end)} "
                            f"({end - start:.1f}s), peak {peak:.0f}°C",
                    time_start=start, time_end=end,
                ))

    # --- Loop timing overruns ---
    full_cycle = get(channels, "/RealOutputs/LoggedRobot/FullCycleMS")
    if full_cycle:
        # Skip the first few samples — startup is always slow
        if len(full_cycle) > 10:
            full_cycle = full_cycle[5:]

        err_spans = find_threshold_spans(
            full_cycle, _LOOP_OVERRUN_ERR_MS, _LOOP_OVERRUN_MIN_DUR, above=True)
        for start, end, peak in err_spans:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem="ELECTRICAL",
                message=f"Severe loop overrun "
                        f"{fmt_time(start)}–{fmt_time(end)} "
                        f"({end - start:.1f}s), peak {peak:.0f}ms cycle — "
                        f"robot control degraded",
                time_start=start, time_end=end,
            ))

        warn_spans = find_threshold_spans(
            full_cycle, _LOOP_OVERRUN_WARN_MS, _LOOP_OVERRUN_MIN_DUR, above=True)
        for start, end, peak in warn_spans:
            already = any(s <= start and end <= e + 0.5
                          for s, e, _ in err_spans)
            if not already:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="ELECTRICAL",
                    message=f"Loop overrun "
                            f"{fmt_time(start)}–{fmt_time(end)} "
                            f"({end - start:.1f}s), peak {peak:.0f}ms cycle",
                    time_start=start, time_end=end,
                ))

    # --- GC pauses ---
    gc_time = get(channels, "/RealOutputs/LoggedRobot/GCTimeMS")
    if gc_time:
        err_spans = find_threshold_spans(
            gc_time, _GC_PAUSE_ERR_MS, 0.0, above=True)
        for start, end, peak in err_spans:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem="ELECTRICAL",
                message=f"GC pause {fmt_time(start)}, {peak:.0f}ms — "
                        f"control loop stalled",
                time_start=start, time_end=end,
            ))

        warn_events = find_threshold_spans(
            gc_time, _GC_PAUSE_WARN_MS, 0.0, above=True)
        # Only report warn-level GC if there are many
        warn_only = [e for e in warn_events
                     if not any(s <= e[0] and e[1] <= s2 + 0.5
                                for s, s2, _ in err_spans)]
        if len(warn_only) >= 3:
            peak = max(e[2] for e in warn_only)
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem="ELECTRICAL",
                message=f"Frequent GC pauses: {len(warn_only)} events "
                        f">{_GC_PAUSE_WARN_MS:.0f}ms, peak {peak:.0f}ms",
                time_start=warn_only[0][0],
                time_end=warn_only[-1][0],
            ))

    # --- Vision issue filtering helper ---
    # When game_timeline is available, only report vision issues that overlap
    # with AUTO, TELEOP, or TEST modes (skip DISABLED-only events).
    def _overlaps_enabled(start: float, end: float) -> bool:
        """Return True if [start, end] overlaps any non-DISABLED mode."""
        if not game_timeline:
            return True  # no timeline info — report everything
        # Walk the timeline to find modes that overlap [start, end]
        for i, (trans_t, mode) in enumerate(game_timeline):
            # Determine the end of this mode segment
            seg_end = game_timeline[i + 1][0] if i + 1 < len(game_timeline) else float("inf")
            # Check overlap with [start, end]
            if seg_end <= start:
                continue
            if trans_t >= end:
                break
            if mode != MODE_DISABLED:
                return True
        return False

    def _point_is_enabled(t: float) -> bool:
        """Return True if timestamp t falls in a non-DISABLED mode."""
        if not game_timeline:
            return True
        return get_game_mode_at(game_timeline, t) != MODE_DISABLED

    # --- PhotonVision coprocessor disconnect ---
    # Detect when PhotonVision NT client disconnects (camera/coprocessor dropped out)
    pv_connected_keys = find_channels(channels, "NTClients", "photonvision", "Connected")
    for pv_key in pv_connected_keys:
        pv_conn = channels.get(pv_key, [])
        if len(pv_conn) < 2:
            continue
        # Extract the client name from the key (e.g. "photonvision@2")
        parts = pv_key.split("/")
        client_name = "PhotonVision"
        for p in parts:
            if "photonvision" in p.lower():
                client_name = p
                break

        # Find disconnect events (transition to False)
        for i in range(1, len(pv_conn)):
            t_prev, v_prev = pv_conn[i - 1]
            t_cur, v_cur = pv_conn[i]
            if v_prev is True and v_cur is False:
                # Find when it reconnects (if ever)
                reconnect_t = None
                for j in range(i + 1, len(pv_conn)):
                    if pv_conn[j][1] is True:
                        reconnect_t = pv_conn[j][0]
                        break
                end_t = reconnect_t if reconnect_t else pv_conn[-1][0]
                if not _overlaps_enabled(t_cur, end_t):
                    continue
                dur = end_t - t_cur
                reconn_str = (f", reconnected at {fmt_time(reconnect_t)}"
                              if reconnect_t else ", did not reconnect")
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem="VISION",
                    message=f"{client_name} disconnected at {fmt_time(t_cur)} "
                            f"({dur:.1f}s){reconn_str} — "
                            f"coprocessor or camera dropout",
                    time_start=t_cur,
                    time_end=end_t,
                ))

    # --- PhotonAlerts: per-camera disconnect tracking ---
    # PhotonVision warnings include messages like:
    #   "PhotonCamera 'Back_Arducam_OV9782_F' is disconnected."
    #   "PhotonVision coprocessor at path /photonvision/Left_... is not connected to the TimeSyncServer?..."
    # Track disconnect spans per camera name so the ERR output is specific.
    import re
    _CAM_DISCONNECT_RE = re.compile(r"PhotonCamera '([^']+)' is disconnected")
    _COPRO_DISCONNECT_RE = re.compile(
        r"coprocessor at path /photonvision/(\S+) is not connected to the TimeSyncServer")

    for alert_key, label in [
        ("/RealOutputs/PhotonAlerts/warnings", "warning"),
        ("/RealOutputs/PhotonAlerts/errors", "error"),
    ]:
        pv_alerts = channels.get(alert_key, [])
        if not pv_alerts:
            continue

        # Build per-camera timeline: for each timestamp, which cameras are alerting
        # cam_name -> list of (timestamp, present_bool)
        cam_events: dict[str, list[tuple[float, bool]]] = {}
        # Track unrecognized alert messages separately
        unrecognized: list[tuple[float, str]] = []

        prev_cameras: set[str] = set()
        for ts, val in pv_alerts:
            if not isinstance(val, list):
                continue
            current_cameras: set[str] = set()
            for s in val:
                if not isinstance(s, str) or not s.strip():
                    continue
                m = _CAM_DISCONNECT_RE.search(s)
                if m:
                    current_cameras.add(m.group(1))
                    continue
                m = _COPRO_DISCONNECT_RE.search(s)
                if m:
                    current_cameras.add(m.group(1))
                    continue
                # Unrecognized but non-trivial message
                clean = "".join(c for c in s if c.isprintable()).strip()
                if len(clean) >= 5:
                    unrecognized.append((ts, clean))

            # Record state transitions for each camera
            appeared = current_cameras - prev_cameras
            disappeared = prev_cameras - current_cameras
            for cam in appeared:
                cam_events.setdefault(cam, []).append((ts, True))
            for cam in disappeared:
                cam_events.setdefault(cam, []).append((ts, False))
            prev_cameras = current_cameras

        # Convert per-camera events into disconnect spans
        for cam, events in sorted(cam_events.items()):
            disconnect_start = None
            for ts, is_disconnected in events:
                if is_disconnected and disconnect_start is None:
                    disconnect_start = ts
                elif not is_disconnected and disconnect_start is not None:
                    if _overlaps_enabled(disconnect_start, ts):
                        dur = ts - disconnect_start
                        issues.append(Issue(
                            severity=SEVERITY_ERR,
                            subsystem="VISION",
                            message=f"Camera '{cam}' disconnected "
                                    f"{fmt_time(disconnect_start)}–{fmt_time(ts)} "
                                    f"({dur:.1f}s)",
                            time_start=disconnect_start,
                            time_end=ts,
                        ))
                    disconnect_start = None
            # Still disconnected at end of log
            if disconnect_start is not None:
                end_t = pv_alerts[-1][0]
                if _overlaps_enabled(disconnect_start, end_t):
                    dur = end_t - disconnect_start
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem="VISION",
                        message=f"Camera '{cam}' disconnected at "
                                f"{fmt_time(disconnect_start)} ({dur:.1f}s) "
                                f"— did not reconnect",
                        time_start=disconnect_start,
                        time_end=end_t,
                    ))

        # Report any unrecognized alerts (only during enabled modes)
        for ts, msg in unrecognized:
            if _point_is_enabled(ts):
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem="VISION",
                    message=f"PhotonAlert {label} @ {fmt_time(ts)}: {msg[:120]}",
                    time_start=ts,
                ))

    return issues
