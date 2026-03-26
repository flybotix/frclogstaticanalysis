"""
Electrical issue detection:
  - Brownouts
  - Low battery voltage
  - Voltage sags
  - PDH current spikes (sustained and instantaneous)
  - CAN bus errors
  - Voltage rail faults
"""

from dataclasses import dataclass, field
from signals import (
    get, get_bool, find_true_spans, find_threshold_spans, find_drops, fmt_time
)

SEVERITY_ERR = "ERR"
SEVERITY_WARN = "WARN"
SEVERITY_INFO = "INFO"
SEVERITY_OK = "OK"


@dataclass
class Issue:
    severity: str
    subsystem: str
    message: str
    time_start: float = 0.0
    time_end: float = 0.0
    detail: str = ""


def analyze_electrical(channels: dict, can_map: dict = None) -> list[Issue]:
    issues = []
    can_map = can_map or {}

    # --- Brownouts ---
    brownout = get_bool(channels, "/SystemStats/BrownedOut")
    spans = find_true_spans(brownout, min_duration=0.0)
    for start, end in spans:
        issues.append(Issue(
            severity=SEVERITY_ERR,
            subsystem="ELECTRICAL",
            message=f"Brownout at {fmt_time(start)}",
            time_start=start,
            time_end=end,
        ))

    # --- Battery voltage low ---
    batt = get(channels, "/SystemStats/BatteryVoltage")
    if not batt:
        batt = get(channels, "/PowerDistribution/Voltage")

    if batt:
        low_spans = find_threshold_spans(batt, threshold=11.0, min_duration=1.0, above=False)
        for start, end, peak in low_spans:
            if peak < 7.0:
                sev = SEVERITY_ERR
            elif peak < 9.0:
                sev = SEVERITY_WARN
            else:
                sev = SEVERITY_INFO
            issues.append(Issue(
                severity=sev,
                subsystem="ELECTRICAL",
                message=f"Low battery {fmt_time(start)} – {fmt_time(end)} ({end - start:.1f}s), min {peak:.2f} V",
                time_start=start,
                time_end=end,
            ))

        # --- Voltage sags (require 2V drop in 0.5s to reduce normal-operation noise) ---
        drops = find_drops(batt, drop_amount=2.0, window_sec=0.5)
        if drops:
            min_v = min(v2 for _, _, v2 in drops)
            if min_v < 7.0:
                sev = SEVERITY_ERR
            elif min_v < 9.0:
                sev = SEVERITY_WARN
            else:
                sev = SEVERITY_INFO
            issues.append(Issue(
                severity=sev,
                subsystem="ELECTRICAL",
                message=(
                    f"Voltage sag ×{len(drops)}: "
                    f"min {min_v:.2f} V, "
                    f"avg sag {sum(v1 - v2 for _, v1, v2 in drops) / len(drops):.1f} V"
                ),
                time_start=drops[0][0],
            ))

    # --- PDH channel currents ---
    pdh_ch = get(channels, "/PowerDistribution/ChannelCurrent")
    # ChannelCurrent is a float[] — each sample is a list
    pdh_raw = channels.get("/PowerDistribution/ChannelCurrent", [])

    # Reconstruct per-channel time series
    if pdh_raw and isinstance(pdh_raw[0][1], list):
        num_channels = max(len(v) for _, v in pdh_raw)
        for ch_idx in range(num_channels):
            ch_series = [(t, v[ch_idx]) for t, v in pdh_raw if ch_idx < len(v)]

            motor_label = can_map.get(f"PDH:{ch_idx}", f"PDH ch{ch_idx}")

            # Sustained spike: > 40 A for > 500 ms
            spikes = find_threshold_spans(ch_series, threshold=40.0, min_duration=0.5, above=True)
            for start, end, peak in spikes:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem="ELECTRICAL",
                    message=(
                        f"Current spike on {motor_label}: "
                        f"{fmt_time(start)} – {fmt_time(end)} ({end - start:.1f}s), peak {peak:.0f} A"
                    ),
                    time_start=start,
                    time_end=end,
                    detail=motor_label,
                ))

            # Instantaneous spike: > 60 A (single sample)
            for t, v in ch_series:
                if v >= 60.0:
                    # Check it's not already in a sustained span
                    already = any(s <= t <= e for s, e, _ in spikes)
                    if not already:
                        issues.append(Issue(
                            severity=SEVERITY_WARN,
                            subsystem="ELECTRICAL",
                            message=(
                                f"Instantaneous current spike on {motor_label}: "
                                f"{fmt_time(t)}, {v:.0f} A"
                            ),
                            time_start=t,
                            detail=motor_label,
                        ))
                    break  # only report first per channel

    # --- CAN bus errors ---
    tx_errs = get(channels, "/SystemStats/CANBus/TransmitErrorCount")
    rx_errs = get(channels, "/SystemStats/CANBus/ReceiveErrorCount")

    def count_increases(series):
        if len(series) < 2:
            return 0, []
        events = []
        total = 0
        for i in range(1, len(series)):
            delta = series[i][1] - series[i - 1][1]
            if delta > 0:
                total += delta
                events.append((series[i][0], int(delta)))
        return total, events

    tx_total, tx_events = count_increases(tx_errs)
    rx_total, rx_events = count_increases(rx_errs)

    if tx_total > 0 or rx_total > 0:
        parts = []
        if tx_total:
            parts.append(f"{int(tx_total)} TX error{'s' if tx_total != 1 else ''}")
        if rx_total:
            parts.append(f"{int(rx_total)} RX error{'s' if rx_total != 1 else ''}")
        first_t = min(
            (e[0] for e in tx_events + rx_events), default=0.0
        )
        issues.append(Issue(
            severity=SEVERITY_WARN,
            subsystem="CAN",
            message=f"CAN bus errors: {', '.join(parts)} starting {fmt_time(first_t)}",
            time_start=first_t,
        ))

    # --- Voltage rail faults ---
    for rail in ("3v3", "5v", "6v"):
        fault_ch = f"/SystemStats/{rail.capitalize()}Rail/CurrentFaults"
        # Try alternate casing
        for candidate in [fault_ch, f"/SystemStats/{rail}Rail/CurrentFaults",
                          f"/SystemStats/{rail.upper()}Rail/CurrentFaults"]:
            faults = get(channels, candidate)
            if faults:
                nonzero = [(t, v) for t, v in faults if v > 0]
                if nonzero:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem="ELECTRICAL",
                        message=f"{rail} rail fault at {fmt_time(nonzero[0][0])}",
                        time_start=nonzero[0][0],
                    ))
                break

    return issues
