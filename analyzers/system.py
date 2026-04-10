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


def analyze_system(channels: dict) -> list[Issue]:
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

    # --- PhotonAlerts/warnings and errors ---
    # Detect non-empty alert strings from PhotonVision.
    # WPILib Alerts are string arrays; short garbled entries (from binary
    # mis-decode) are filtered out — only report messages ≥ 5 chars.
    _MIN_ALERT_LEN = 5
    for alert_key, label in [
        ("/RealOutputs/PhotonAlerts/warnings", "warning"),
        ("/RealOutputs/PhotonAlerts/errors", "error"),
    ]:
        pv_alerts = channels.get(alert_key, [])
        for ts, val in pv_alerts:
            if not isinstance(val, list):
                continue
            for s in val:
                if not isinstance(s, str):
                    continue
                clean = "".join(c for c in s if c.isprintable()).strip()
                if len(clean) >= _MIN_ALERT_LEN:
                    sev = SEVERITY_ERR if label == "error" else SEVERITY_ERR
                    issues.append(Issue(
                        severity=sev,
                        subsystem="VISION",
                        message=f"PhotonAlert {label} @ {fmt_time(ts)}: "
                                f"{clean[:120]}",
                        time_start=ts,
                    ))

    # Count PhotonAlert state changes as camera dropout indicators.
    # When the warnings array goes from empty to non-empty, a camera issue
    # started; when it goes back to empty, it recovered.
    pv_warn_raw = channels.get("/RealOutputs/PhotonAlerts/warnings", [])
    if len(pv_warn_raw) >= 2:
        dropout_start = None
        for i, (ts, val) in enumerate(pv_warn_raw):
            has_alert = (isinstance(val, list) and
                         any(s.strip() for s in val if isinstance(s, str)))
            if has_alert and dropout_start is None:
                dropout_start = ts
            elif not has_alert and dropout_start is not None:
                dur = ts - dropout_start
                if dur >= 0.1:  # ignore sub-100ms flickers
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem="VISION",
                        message=f"PhotonVision camera alert active "
                                f"{fmt_time(dropout_start)}–{fmt_time(ts)} "
                                f"({dur:.1f}s)",
                        time_start=dropout_start,
                        time_end=ts,
                    ))
                dropout_start = None
        # If alert was still active at end of log
        if dropout_start is not None:
            end_t = pv_warn_raw[-1][0]
            dur = end_t - dropout_start
            if dur >= 0.1:
                issues.append(Issue(
                    severity=SEVERITY_ERR,
                    subsystem="VISION",
                    message=f"PhotonVision camera alert active "
                            f"{fmt_time(dropout_start)}–{fmt_time(end_t)} "
                            f"({dur:.1f}s) — did not clear",
                    time_start=dropout_start,
                    time_end=end_t,
                ))

    return issues
