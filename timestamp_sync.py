"""
Cross-format timestamp synchronization.

Computes time offsets between log sources by correlating enable/disable
transitions visible in multiple formats:

  - wpilog: DriverStation/Enabled + Autonomous channels
  - hoot: Phoenix6/*/RobotEnable signal on any TalonFX

The offset is computed from the first enable transition after a long
disabled period, then verified against subsequent transitions. If the
verification fails (> 1s error), no offset is applied.
"""

import sys

_MIN_DISABLED_GAP = 3.0   # minimum disabled duration to count as a "real" transition
_MAX_SYNC_ERROR = 1.0      # maximum acceptable error (seconds) for verification


def compute_hoot_offset(hoot_channels: dict, wpi_transitions: list) -> float | None:
    """
    Compute the time offset to add to hoot timestamps to align with wpilog.

    Args:
        hoot_channels: parsed hoot channel data (session-relative timestamps)
        wpi_transitions: game mode timeline from get_game_mode_timeline()

    Returns:
        Offset in seconds (add to hoot timestamps), or None if sync failed.
    """
    from parser import MODE_DISABLED, MODE_AUTO, MODE_TELEOP

    # Find enable transitions in wpilog (DISABLED → enabled, after gap)
    wpi_enables = _find_enable_transitions(wpi_transitions, MODE_DISABLED,
                                           {MODE_AUTO, MODE_TELEOP})
    if not wpi_enables:
        return None

    # Find RobotEnable transitions in hoot data (any TalonFX)
    hoot_enables = _find_hoot_enables(hoot_channels)
    if not hoot_enables:
        return None

    # Compute offset from first pair
    offset = wpi_enables[0] - hoot_enables[0]

    # Verify with subsequent transitions
    matched = 0
    total_error = 0.0
    for wt in wpi_enables[1:]:
        expected_hoot = wt - offset
        closest = min(hoot_enables, key=lambda h: abs(h - expected_hoot))
        error = abs(closest - expected_hoot)
        if error < _MAX_SYNC_ERROR:
            matched += 1
            total_error += error

    if len(wpi_enables) > 1 and matched == 0:
        # None of the subsequent transitions matched — offset is unreliable
        return None

    avg_error = total_error / matched if matched else 0.0
    print(f"Timestamp sync: hoot offset = {offset:+.2f}s "
          f"(verified {matched}/{len(wpi_enables)-1} transitions, "
          f"avg error {avg_error:.3f}s)", file=sys.stderr)

    return offset


def _find_enable_transitions(transitions, disabled_mode, enabled_modes) -> list[float]:
    """Find enable transitions preceded by a long disabled period."""
    enables = []
    last_disable_t = 0.0
    prev_mode = disabled_mode

    for t, mode in transitions:
        if mode == disabled_mode:
            last_disable_t = t
        elif mode in enabled_modes and prev_mode == disabled_mode:
            gap = t - last_disable_t
            if gap >= _MIN_DISABLED_GAP or not enables:
                enables.append(t)
        prev_mode = mode

    return enables


def _find_hoot_enables(channels: dict) -> list[float]:
    """Find RobotEnable transitions across all TalonFX devices in hoot data."""
    # Find any TalonFX RobotEnable channel
    re_channel = None
    for name in channels:
        if "RobotEnable" in name and "TalonFX" in name:
            re_channel = name
            break

    if not re_channel:
        return []

    series = channels[re_channel]
    if not series:
        return []

    enables = []
    last_disable_t = 0.0
    was_enabled = False

    for t, v in series:
        is_enabled = (str(v) == "Enabled") if isinstance(v, str) else (v > 0.5)
        if is_enabled and not was_enabled:
            gap = t - last_disable_t
            if gap >= _MIN_DISABLED_GAP or not enables:
                enables.append(t)
        elif not is_enabled and was_enabled:
            last_disable_t = t
        was_enabled = is_enabled

    return enables


def apply_offset(channels: dict, offset: float) -> dict:
    """Apply a time offset to all channels. Returns a new dict."""
    result = {}
    for name, series in channels.items():
        result[name] = [(t + offset, v) for t, v in series]
    return result
