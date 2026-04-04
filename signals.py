"""
Signal extraction and analysis helpers.
Works without numpy; numpy-accelerated paths used when available.
"""

from typing import Any

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


TimeSeries = list[tuple[float, float]]


def get(channels: dict, name: str) -> TimeSeries:
    """Return (t_sec, value) pairs for a numeric channel, empty list if missing."""
    raw = channels.get(name, [])
    result = []
    for t, v in raw:
        try:
            result.append((t, float(v)))
        except (TypeError, ValueError):
            pass
    return result


def get_bool(channels: dict, name: str) -> list[tuple[float, bool]]:
    return channels.get(name, [])


def find_channels(channels: dict, *fragments: str, case_insensitive=True) -> list[str]:
    """Return channel names containing all given fragments."""
    matches = []
    for name in channels:
        n = name.lower() if case_insensitive else name
        if all((f.lower() if case_insensitive else f) in n for f in fragments):
            matches.append(name)
    return sorted(matches)


def duration(channels: dict) -> float:
    """Return total log duration in seconds."""
    max_t = 0.0
    for series in channels.values():
        if series:
            max_t = max(max_t, series[-1][0])
    return max_t


def find_true_spans(bool_series: list[tuple[float, bool]], min_duration=0.0) -> list[tuple[float, float]]:
    """Find contiguous spans where bool channel is True."""
    spans = []
    start = None
    for t, val in bool_series:
        if val and start is None:
            start = t
        elif not val and start is not None:
            if t - start >= min_duration:
                spans.append((start, t))
            start = None
    if start is not None:
        end = bool_series[-1][0]
        if end - start >= min_duration:
            spans.append((start, end))
    return spans


def find_threshold_spans(series: TimeSeries, threshold: float, min_duration=0.0,
                         above=True) -> list[tuple[float, float, float]]:
    """
    Find contiguous spans where value is above (or below) threshold.
    Returns list of (start_sec, end_sec, peak_value).
    """
    if not series:
        return []
    spans = []
    start = None
    peak = None
    for t, v in series:
        cond = v >= threshold if above else v <= threshold
        if cond and start is None:
            start = t
            peak = v
        elif cond and start is not None:
            peak = max(peak, v) if above else min(peak, v)
        elif not cond and start is not None:
            if t - start >= min_duration:
                spans.append((start, t, peak))
            start = None
            peak = None
    if start is not None:
        end = series[-1][0]
        if end - start >= min_duration:
            spans.append((start, end, peak))
    return spans


def find_drops(series: TimeSeries, drop_amount: float, window_sec=0.5) -> list[tuple[float, float, float]]:
    """
    Find sudden drops of >= drop_amount within window_sec.
    Returns list of (time_sec, value_before, value_after).
    """
    if len(series) < 2:
        return []
    events = []
    seen = set()
    for i, (t1, v1) in enumerate(series):
        for j in range(i + 1, len(series)):
            t2, v2 = series[j]
            if t2 - t1 > window_sec:
                break
            drop = v1 - v2
            if drop >= drop_amount:
                key = round(t1, 2)
                if key not in seen:
                    seen.add(key)
                    events.append((t1, v1, v2))
                break
    return events


def find_pose_jumps(pose_series: list[tuple[float, dict]], jump_m=0.3) -> list[tuple[float, float]]:
    """Find sudden discontinuities in Pose2d. Returns (time_sec, jump_distance_m)."""
    jumps = []
    for i in range(1, len(pose_series)):
        t_prev, p_prev = pose_series[i - 1]
        t_cur, p_cur = pose_series[i]
        if p_prev is None or p_cur is None:
            continue
        dx = p_cur["x"] - p_prev["x"]
        dy = p_cur["y"] - p_prev["y"]
        dist = (dx * dx + dy * dy) ** 0.5
        dt = t_cur - t_prev
        # Only flag if jump happens in a short time (not just slow movement)
        if dt < 0.1 and dist >= jump_m:
            jumps.append((t_cur, dist))
    return jumps


def derivative(series: TimeSeries) -> TimeSeries:
    """Compute numerical derivative (rate of change per second)."""
    if len(series) < 2:
        return []
    result = []
    for i in range(1, len(series)):
        t0, v0 = series[i - 1]
        t1, v1 = series[i]
        dt = t1 - t0
        if dt > 0:
            result.append(((t0 + t1) / 2, (v1 - v0) / dt))
    return result


def is_near_zero(series: TimeSeries, threshold=0.5) -> list[tuple[float, bool]]:
    """Return bool series: True when abs(value) < threshold."""
    return [(t, abs(v) < threshold) for t, v in series]


def subtract(a: TimeSeries, b: TimeSeries) -> TimeSeries:
    """
    Compute a - b, aligning by nearest timestamp.
    Simple approach: for each point in a, find closest point in b.
    """
    if not a or not b:
        return []
    b_times = [t for t, _ in b]
    b_vals = [v for _, v in b]
    result = []
    j = 0
    for t, va in a:
        # Advance j to nearest
        while j + 1 < len(b_times) and abs(b_times[j + 1] - t) < abs(b_times[j] - t):
            j += 1
        result.append((t, va - b_vals[j]))
    return result


def abs_series(series: TimeSeries) -> TimeSeries:
    return [(t, abs(v)) for t, v in series]


def interp(series: TimeSeries, t: float) -> float:
    """Linearly interpolate a time series at timestamp t."""
    if not series:
        return 0.0
    if t <= series[0][0]:
        return series[0][1]
    if t >= series[-1][0]:
        return series[-1][1]
    lo, hi = 0, len(series) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, v0 = series[lo]
    t1, v1 = series[hi]
    if t1 == t0:
        return v0
    return v0 + (t - t0) / (t1 - t0) * (v1 - v0)


def fmt_time(t_sec: float) -> str:
    """Format seconds as M:SS.t"""
    m = int(t_sec) // 60
    s = t_sec - m * 60
    return f"T+{m}:{s:05.2f}"


def fmt_duration(t_sec: float) -> str:
    """Format seconds as human-readable duration."""
    m = int(t_sec) // 60
    s = int(t_sec) % 60
    if m > 0:
        return f"{m}m {s:02d}s"
    return f"{t_sec:.1f}s"
