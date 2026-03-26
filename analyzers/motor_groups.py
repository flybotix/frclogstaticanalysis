"""
Motor group analyzer.

Detects issues by comparing linked motors that should behave similarly:
  - Output divergence: one motor driving while the other is off
  - Velocity divergence: motors running at different speeds
  - Current imbalance: one motor drawing significantly more current
"""

from signals import get, find_threshold_spans, fmt_time
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN
from device_config import DeviceConfig, MotorGroup


# Thresholds
_VEL_DIVERGE_FRAC = 0.20    # 20% velocity difference
_VEL_DIVERGE_MIN_DUR = 0.5  # seconds
_VEL_MIN_ACTIVE = 5.0       # minimum velocity (rps) to consider motor active
_CURRENT_IMBALANCE = 1.5    # one motor > 1.5x the other
_CURRENT_MIN = 5.0          # minimum current (A) to consider


def _find_channels(channels: dict, device_key: str) -> dict[str, str]:
    """Map signal names to channel keys for a device."""
    # CTRE devices: "CTRE:TalonFX-29" -> channels like "Phoenix6/TalonFX-29/..."
    # SPARK devices: "SPARK:16" -> channels like "/REV/SPARK/16/..."
    result = {}
    if device_key.startswith("CTRE:"):
        ctre_name = device_key.replace("CTRE:", "")
        prefix = f"Phoenix6/{ctre_name}"
        for ch_name in channels:
            if ch_name.startswith(prefix + "/"):
                signal = ch_name[len(prefix) + 1:]
                result[signal] = ch_name
    elif device_key.startswith("SPARK:"):
        dev_num = device_key.replace("SPARK:", "")
        prefix = f"/REV/SPARK/{dev_num}"
        for ch_name in channels:
            if ch_name.startswith(prefix + "/"):
                signal = ch_name[len(prefix) + 1:]
                result[signal] = ch_name
    return result


def _get_velocity_channel(signal_map: dict, device_key: str) -> str | None:
    """Find the velocity channel for a device."""
    if device_key.startswith("CTRE:"):
        return signal_map.get("Velocity") or signal_map.get("RotorVelocity")
    return signal_map.get("Velocity")


def _get_current_channel(signal_map: dict, device_key: str) -> str | None:
    """Find the current channel for a device."""
    if device_key.startswith("CTRE:"):
        return signal_map.get("StatorCurrent") or signal_map.get("SupplyCurrent")
    return signal_map.get("OutputCurrent")


def _get_output_channel(signal_map: dict, device_key: str) -> str | None:
    """Find the motor output/voltage channel."""
    if device_key.startswith("CTRE:"):
        return signal_map.get("MotorVoltage") or signal_map.get("DutyCycle")
    return signal_map.get("AppliedOutput")


def analyze_motor_groups(channels: dict, config: DeviceConfig) -> list[Issue]:
    """Run comparison checks across configured motor groups."""
    if not config or not config.groups:
        return []

    issues = []

    for group in config.groups:
        if len(group.motors) < 2:
            continue

        subsystem = group.subsystem or "HOOT"

        # Get channel maps for each motor
        motor_channels = {}
        for key in group.motors:
            motor_channels[key] = _find_channels(channels, key)

        # Only analyze if we have data for at least 2 motors
        motors_with_data = [k for k, v in motor_channels.items() if v]
        if len(motors_with_data) < 2:
            continue

        # --- Output divergence ---
        issues.extend(_check_output_divergence(
            channels, group, motor_channels, config, subsystem))

        # --- Velocity divergence ---
        issues.extend(_check_velocity_divergence(
            channels, group, motor_channels, config, subsystem))

        # --- Current imbalance ---
        issues.extend(_check_current_imbalance(
            channels, group, motor_channels, config, subsystem))

    return issues


def _check_output_divergence(channels, group, motor_channels, config, subsystem):
    """Detect when one motor is driving while another in the group is off."""
    issues = []

    for i, key_a in enumerate(group.motors):
        for key_b in group.motors[i + 1:]:
            ch_a = _get_output_channel(motor_channels.get(key_a, {}), key_a)
            ch_b = _get_output_channel(motor_channels.get(key_b, {}), key_b)
            if not ch_a or not ch_b:
                continue

            series_a = get(channels, ch_a)
            series_b = get(channels, ch_b)
            if not series_a or not series_b:
                continue

            # Build time-aligned divergence series
            b_map = dict(series_b)
            b_times = sorted(b_map)
            if not b_times:
                continue

            label_a = config.label(key_a, key_a)
            label_b = config.label(key_b, key_b)

            j = 0
            diverge_start = None
            for t, va in series_a:
                while j + 1 < len(b_times) and abs(b_times[j + 1] - t) < abs(b_times[j] - t):
                    j += 1
                vb = b_map[b_times[j]]

                a_active = abs(va) > 0.5
                b_active = abs(vb) > 0.5

                if a_active != b_active:
                    if diverge_start is None:
                        diverge_start = t
                else:
                    if diverge_start is not None and t - diverge_start >= 0.5:
                        issues.append(Issue(
                            severity=SEVERITY_ERR,
                            subsystem=subsystem,
                            message=(f"{group.name}: OUTPUT DIVERGENCE "
                                     f"{fmt_time(diverge_start)}–{fmt_time(t)} "
                                     f"({t - diverge_start:.1f}s) — "
                                     f"{label_a} vs {label_b}"),
                            time_start=diverge_start,
                            time_end=t,
                            detail=group.name,
                        ))
                    diverge_start = None

            # Check trailing span
            if diverge_start is not None and series_a:
                end_t = series_a[-1][0]
                if end_t - diverge_start >= 0.5:
                    issues.append(Issue(
                        severity=SEVERITY_ERR,
                        subsystem=subsystem,
                        message=(f"{group.name}: OUTPUT DIVERGENCE "
                                 f"{fmt_time(diverge_start)}–{fmt_time(end_t)} "
                                 f"({end_t - diverge_start:.1f}s) — "
                                 f"{label_a} vs {label_b}"),
                        time_start=diverge_start,
                        time_end=end_t,
                        detail=group.name,
                    ))

    return issues


def _check_velocity_divergence(channels, group, motor_channels, config, subsystem):
    """Detect when linked motors are running at significantly different speeds."""
    issues = []

    for i, key_a in enumerate(group.motors):
        for key_b in group.motors[i + 1:]:
            ch_a = _get_velocity_channel(motor_channels.get(key_a, {}), key_a)
            ch_b = _get_velocity_channel(motor_channels.get(key_b, {}), key_b)
            if not ch_a or not ch_b:
                continue

            series_a = get(channels, ch_a)
            series_b = get(channels, ch_b)
            if not series_a or not series_b:
                continue

            ratio = group.ratio
            sign = -1.0 if group.relationship == "opposite_direction" else 1.0

            b_map = dict(series_b)
            b_times = sorted(b_map)
            if not b_times:
                continue

            label_a = config.label(key_a, key_a)
            label_b = config.label(key_b, key_b)

            # Build divergence proxy: 1.0 if diverged, 0.0 if ok
            proxy = []
            j = 0
            for t, va in series_a:
                while j + 1 < len(b_times) and abs(b_times[j + 1] - t) < abs(b_times[j] - t):
                    j += 1
                vb = b_map[b_times[j]] * sign * ratio

                # Only check when both are moving
                if abs(va) < _VEL_MIN_ACTIVE and abs(vb) < _VEL_MIN_ACTIVE:
                    proxy.append((t, 0.0))
                    continue

                faster = max(abs(va), abs(vb))
                diff = abs(va - vb)
                diverged = diff > faster * _VEL_DIVERGE_FRAC
                proxy.append((t, 1.0 if diverged else 0.0))

            spans = find_threshold_spans(proxy, 0.5, _VEL_DIVERGE_MIN_DUR, above=True)
            for start, end, _ in spans[:5]:  # limit to 5 reports
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=(f"{group.name}: velocity divergence "
                             f"{fmt_time(start)}–{fmt_time(end)} "
                             f"({end - start:.1f}s) — "
                             f"{label_a} vs {label_b}"),
                    time_start=start,
                    time_end=end,
                    detail=group.name,
                ))

    return issues


def _check_current_imbalance(channels, group, motor_channels, config, subsystem):
    """Detect sustained current imbalance between linked motors."""
    issues = []

    for i, key_a in enumerate(group.motors):
        for key_b in group.motors[i + 1:]:
            ch_a = _get_current_channel(motor_channels.get(key_a, {}), key_a)
            ch_b = _get_current_channel(motor_channels.get(key_b, {}), key_b)
            if not ch_a or not ch_b:
                continue

            series_a = get(channels, ch_a)
            series_b = get(channels, ch_b)
            if not series_a or not series_b:
                continue

            label_a = config.label(key_a, key_a)
            label_b = config.label(key_b, key_b)

            b_map = dict(series_b)
            b_times = sorted(b_map)
            if not b_times:
                continue

            proxy = []
            j = 0
            for t, ca in series_a:
                while j + 1 < len(b_times) and abs(b_times[j + 1] - t) < abs(b_times[j] - t):
                    j += 1
                cb = b_map[b_times[j]]

                if ca < _CURRENT_MIN and cb < _CURRENT_MIN:
                    proxy.append((t, 0.0))
                    continue

                if cb > 0.1 and ca / cb > _CURRENT_IMBALANCE:
                    proxy.append((t, 1.0))
                elif ca > 0.1 and cb / ca > _CURRENT_IMBALANCE:
                    proxy.append((t, 1.0))
                else:
                    proxy.append((t, 0.0))

            spans = find_threshold_spans(proxy, 0.5, 2.0, above=True)
            for start, end, _ in spans[:3]:
                issues.append(Issue(
                    severity=SEVERITY_WARN,
                    subsystem=subsystem,
                    message=(f"{group.name}: current imbalance "
                             f"{fmt_time(start)}–{fmt_time(end)} "
                             f"({end - start:.1f}s) — "
                             f"{label_a} vs {label_b}"),
                    time_start=start,
                    time_end=end,
                    detail=group.name,
                ))

    return issues
