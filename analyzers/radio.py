"""
Radio and communications analyzer.

Detects issues from DriverStation, SystemStats, and RadioStatus channels:
  - Radio disconnects (RadioStatus/Connected going False)
  - Comms disable events (SystemStats/CommsDisableCount increments)
  - Joystick data gaps during enabled periods (proxy for radio latency/dropouts)
  - Signal strength, SNR, and link rate from RadioStatus/Status JSON
"""

from signals import get, get_bool, find_threshold_spans, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO
from parser import get_game_mode_timeline, get_game_mode_at, MODE_DISABLED, MODE_TELEOP

SUBSYSTEM = "RADIO"

# Joystick gap thresholds
_GAP_ERR_MS = 1000      # > 1s gap = loss of control
_GAP_WARN_MS = 500       # > 500ms = noticeable
_GAP_INFO_MS = 250       # > 250ms = minor latency

# Signal strength thresholds (dBm) — WiFi RSSI for real-time control
_SIGNAL_ERR_DBM = -70    # likely dropouts
_SIGNAL_WARN_DBM = -60   # marginal
_SIGNAL_INFO_DBM = -50   # acceptable but not ideal

# Signal-to-noise ratio thresholds (dB)
_SNR_ERR = 20            # severe, expect packet loss
_SNR_WARN = 30           # marginal
_SNR_INFO = 40           # ok but could be better

# Link rate thresholds (Mbps) — low rate = radio negotiating down
_LINKRATE_ERR = 50       # severe degradation
_LINKRATE_WARN = 200     # significant degradation


def _extract_radio_signals(channels: dict) -> list[tuple[float, dict]]:
    """Extract networkStatus6 fields from RadioStatus/Status JSON samples."""
    raw = channels.get("/RadioStatus/Status", [])
    results = []
    for t, j in raw:
        if not isinstance(j, dict):
            continue
        # Prefer 6GHz, fall back to 2.4GHz
        ns = j.get("networkStatus6") or j.get("networkStatus24") or {}
        if not ns.get("isLinked"):
            results.append((t, None))
            continue
        results.append((t, {
            "signalDbm": ns.get("signalDbm", 0),
            "noiseDbm": ns.get("noiseDbm", 0),
            "snr": ns.get("signalNoiseRatio", -999),
            "rxRate": ns.get("rxRateMbps", -999),
            "txRate": ns.get("txRateMbps", -999),
            "bandwidth": ns.get("bandwidthUsedMbps", -999),
            "quality": ns.get("connectionQuality", ""),
        }))
    return results


def analyze_radio(channels: dict) -> list[Issue]:
    issues = []
    transitions, _ = get_game_mode_timeline(channels)

    # --- Radio disconnects ---
    radio_connected = get_bool(channels, "/RadioStatus/Connected")
    if radio_connected:
        for i in range(1, len(radio_connected)):
            t_prev, v_prev = radio_connected[i - 1]
            t_cur, v_cur = radio_connected[i]
            if v_prev and not v_cur:
                mode = get_game_mode_at(transitions, t_cur)
                sev = SEVERITY_ERR if mode != MODE_DISABLED else SEVERITY_WARN
                issues.append(Issue(
                    severity=sev,
                    subsystem=SUBSYSTEM,
                    message=f"Radio disconnected at {fmt_time(t_cur)}",
                    time_start=t_cur,
                ))

    # --- CommsDisableCount increments ---
    comms = get(channels, "/SystemStats/CommsDisableCount")
    if len(comms) >= 2:
        bursts = []
        burst_start = None
        burst_count = 0

        for i in range(1, len(comms)):
            t_prev, v_prev = comms[i - 1]
            t_cur, v_cur = comms[i]
            delta = v_cur - v_prev
            if delta > 0:
                if burst_start is None:
                    burst_start = t_cur
                    burst_count = int(delta)
                else:
                    burst_count += int(delta)
            else:
                if burst_start is not None:
                    bursts.append((burst_start, t_prev, burst_count))
                    burst_start = None
                    burst_count = 0

        if burst_start is not None:
            bursts.append((burst_start, comms[-1][0], burst_count))

        for start, end, count in bursts:
            mode = get_game_mode_at(transitions, start)
            dur = end - start
            if count >= 5:
                sev = SEVERITY_ERR
            elif count >= 2:
                sev = SEVERITY_WARN
            else:
                sev = SEVERITY_INFO

            if mode != MODE_DISABLED or sev != SEVERITY_INFO:
                issues.append(Issue(
                    severity=sev,
                    subsystem=SUBSYSTEM,
                    message=(f"Comms dropout ×{count} at {fmt_time(start)}"
                             f" ({dur:.1f}s burst)"),
                    time_start=start,
                    time_end=end,
                ))

    # --- Radio signal quality from RadioStatus/Status JSON ---
    radio_signals = _extract_radio_signals(channels)
    if radio_signals:
        _analyze_signal_quality(radio_signals, transitions, issues)

    # --- Joystick data gaps during TELEOP only ---
    # Disabled: produces too many false positives from normal driver idle periods.
    # The analysis code is preserved below but not called.
    # To re-enable, uncomment the following line:
    # _analyze_joystick_gaps(channels, transitions, issues)

    return issues


def _analyze_joystick_gaps(channels, transitions, issues):
    """Detect joystick data gaps during TELEOP as a proxy for radio dropouts."""
    joy = channels.get("/DriverStation/Joystick0/AxisValues", [])
    if len(joy) >= 2:
        err_gaps = []
        warn_gaps = []
        info_gaps = []

        for i in range(1, len(joy)):
            t_prev = joy[i - 1][0]
            t_cur = joy[i][0]
            dt_ms = (t_cur - t_prev) * 1000

            if dt_ms < _GAP_INFO_MS:
                continue

            mode = get_game_mode_at(transitions, t_prev)
            if mode != MODE_TELEOP:
                continue

            if dt_ms >= _GAP_ERR_MS:
                err_gaps.append((t_prev, t_cur, dt_ms))
            elif dt_ms >= _GAP_WARN_MS:
                warn_gaps.append((t_prev, t_cur, dt_ms))
            else:
                info_gaps.append((t_prev, t_cur, dt_ms))

        # Report ERR gaps individually (these are serious)
        for t1, t2, dt_ms in err_gaps:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem=SUBSYSTEM,
                message=(f"Joystick data gap {fmt_time(t1)}–{fmt_time(t2)} "
                         f"({dt_ms:.0f}ms) — loss of driver control"),
                time_start=t1,
                time_end=t2,
            ))

        # Summarize WARN gaps
        if warn_gaps:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem=SUBSYSTEM,
                message=(f"Joystick data gaps ×{len(warn_gaps)} "
                         f"(500–1000ms) while enabled — "
                         f"max {max(dt for _,_,dt in warn_gaps):.0f}ms"),
                time_start=warn_gaps[0][0],
            ))

        # Summarize INFO gaps
        if info_gaps:
            issues.append(Issue(
                severity=SEVERITY_INFO,
                subsystem=SUBSYSTEM,
                message=(f"Joystick data gaps ×{len(info_gaps)} "
                         f"(250–500ms) while enabled — "
                         f"max {max(dt for _,_,dt in info_gaps):.0f}ms"),
                time_start=info_gaps[0][0],
            ))


def _analyze_signal_quality(radio_signals, transitions, issues):
    """Analyze signal strength, SNR, and link rates from radio JSON."""

    # Separate linked vs unlinked samples
    linked = [(t, s) for t, s in radio_signals if s is not None]
    unlinked = [(t, s) for t, s in radio_signals if s is None]

    if not linked:
        if unlinked:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem=SUBSYSTEM,
                message=f"Radio never linked during session ({len(unlinked)} samples)",
                time_start=unlinked[0][0],
            ))
        return

    # --- Signal strength (dBm) ---
    signal_series = [(t, s["signalDbm"]) for t, s in linked if s["signalDbm"] != 0]
    if signal_series:
        worst_signal = min(v for _, v in signal_series)
        avg_signal = sum(v for _, v in signal_series) / len(signal_series)

        err_spans = [(t, v) for t, v in signal_series if v <= _SIGNAL_ERR_DBM]
        warn_spans = [(t, v) for t, v in signal_series if _SIGNAL_ERR_DBM < v <= _SIGNAL_WARN_DBM]
        info_spans = [(t, v) for t, v in signal_series if _SIGNAL_WARN_DBM < v <= _SIGNAL_INFO_DBM]

        if err_spans:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem=SUBSYSTEM,
                message=(f"Weak signal ×{len(err_spans)} samples "
                         f"(≤{_SIGNAL_ERR_DBM} dBm), "
                         f"worst {worst_signal} dBm at {fmt_time(err_spans[0][0])}"),
                time_start=err_spans[0][0],
            ))
        elif warn_spans:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem=SUBSYSTEM,
                message=(f"Marginal signal ×{len(warn_spans)} samples "
                         f"({_SIGNAL_ERR_DBM} to {_SIGNAL_WARN_DBM} dBm), "
                         f"worst {worst_signal} dBm"),
                time_start=warn_spans[0][0],
            ))
        elif info_spans:
            issues.append(Issue(
                severity=SEVERITY_INFO,
                subsystem=SUBSYSTEM,
                message=(f"Signal {avg_signal:.0f} dBm avg, "
                         f"worst {worst_signal} dBm "
                         f"({len(info_spans)} samples below {_SIGNAL_INFO_DBM} dBm)"),
                time_start=info_spans[0][0],
            ))

    # --- SNR ---
    snr_series = [(t, s["snr"]) for t, s in linked if s["snr"] != -999]
    if snr_series:
        worst_snr = min(v for _, v in snr_series)

        err_snr = [(t, v) for t, v in snr_series if v <= _SNR_ERR]
        warn_snr = [(t, v) for t, v in snr_series if _SNR_ERR < v <= _SNR_WARN]

        if err_snr:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem=SUBSYSTEM,
                message=(f"Low SNR ×{len(err_snr)} samples "
                         f"(≤{_SNR_ERR} dB), worst {worst_snr} dB "
                         f"at {fmt_time(err_snr[0][0])} — expect packet loss"),
                time_start=err_snr[0][0],
            ))
        elif warn_snr:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem=SUBSYSTEM,
                message=(f"Marginal SNR ×{len(warn_snr)} samples "
                         f"({_SNR_ERR}–{_SNR_WARN} dB), worst {worst_snr} dB"),
                time_start=warn_snr[0][0],
            ))

    # --- Link rate ---
    rx_series = [(t, s["rxRate"]) for t, s in linked if s["rxRate"] > 0]
    tx_series = [(t, s["txRate"]) for t, s in linked if s["txRate"] > 0]

    for label, series in [("RX", rx_series), ("TX", tx_series)]:
        if not series:
            continue
        worst_rate = min(v for _, v in series)

        err_rate = [(t, v) for t, v in series if v <= _LINKRATE_ERR]
        warn_rate = [(t, v) for t, v in series if _LINKRATE_ERR < v <= _LINKRATE_WARN]

        if err_rate:
            issues.append(Issue(
                severity=SEVERITY_ERR,
                subsystem=SUBSYSTEM,
                message=(f"Low {label} link rate ×{len(err_rate)} samples "
                         f"(≤{_LINKRATE_ERR} Mbps), "
                         f"worst {worst_rate:.0f} Mbps at {fmt_time(err_rate[0][0])}"),
                time_start=err_rate[0][0],
            ))
        elif warn_rate:
            issues.append(Issue(
                severity=SEVERITY_WARN,
                subsystem=SUBSYSTEM,
                message=(f"Degraded {label} link rate ×{len(warn_rate)} samples "
                         f"({_LINKRATE_ERR}–{_LINKRATE_WARN} Mbps), "
                         f"worst {worst_rate:.0f} Mbps"),
                time_start=warn_rate[0][0],
            ))

    # --- Connection quality changes ---
    qualities = [(t, s["quality"]) for t, s in linked if s["quality"]]
    bad_quality = [(t, q) for t, q in qualities if q not in ("excellent", "good")]
    if bad_quality:
        worst_q = bad_quality[0][1]
        issues.append(Issue(
            severity=SEVERITY_WARN,
            subsystem=SUBSYSTEM,
            message=(f"Connection quality degraded to \"{worst_q}\" "
                     f"at {fmt_time(bad_quality[0][0])} "
                     f"({len(bad_quality)} samples below \"good\")"),
            time_start=bad_quality[0][0],
        ))
