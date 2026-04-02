"""
System health detection:
  - CAN bus utilization (high bus load causing dropped frames)
  - CPU temperature (roboRIO overheating)
  - Code loop overruns (robot code can't keep up with cycle time)
  - GC pauses (Java garbage collection stalls)
"""

from signals import get, find_threshold_spans, fmt_time
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
                    severity=SEVERITY_WARN,
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

    return issues
