"""
Subsystem roll-up: combine electrical + mechanical findings and
produce a per-subsystem summary status.
"""

from dataclasses import dataclass
from analyzers.electrical import Issue, SEVERITY_ERR, SEVERITY_WARN, SEVERITY_INFO, SEVERITY_OK


DEFAULT_SUBSYSTEMS = ["ELECTRICAL", "RADIO", "SHOOTER", "INTAKE", "TURRET", "SWERVE", "CAN", "MOTORS"]

# HOOT and REVLOG issues are merged into MOTORS for display
_MOTOR_SUBSYSTEMS = {"HOOT", "REVLOG"}


@dataclass
class SubsystemStatus:
    name: str
    status: str       # ERR / WARN / OK
    summary: str      # one-line summary for the table
    issues: list      # list[Issue]


def roll_up(all_issues: list[Issue], extra_subsystems: list = None) -> list[SubsystemStatus]:
    """Group issues by subsystem and compute overall status."""
    # Build ordered subsystem list: defaults + user-defined + any seen in issues
    subsystems = list(DEFAULT_SUBSYSTEMS)
    for s in (extra_subsystems or []):
        if s not in subsystems:
            subsystems.append(s)
    for issue in all_issues:
        target = "MOTORS" if issue.subsystem in _MOTOR_SUBSYSTEMS else issue.subsystem
        if target not in subsystems:
            subsystems.append(target)

    by_subsystem: dict[str, list[Issue]] = {s: [] for s in subsystems}

    for issue in all_issues:
        sub = "MOTORS" if issue.subsystem in _MOTOR_SUBSYSTEMS else issue.subsystem
        if sub not in by_subsystem:
            by_subsystem[sub] = []
        by_subsystem[sub].append(issue)

    results = []
    for sub in subsystems:
        issues = by_subsystem.get(sub, [])
        if not issues:
            results.append(SubsystemStatus(sub, SEVERITY_OK, "—", []))
            continue

        has_err = any(i.severity == SEVERITY_ERR for i in issues)
        has_warn = any(i.severity == SEVERITY_WARN for i in issues)
        if has_err:
            status = SEVERITY_ERR
        elif has_warn:
            status = SEVERITY_WARN
        else:
            status = SEVERITY_INFO

        # Build a short summary line
        summary = _summarize(sub, issues)
        results.append(SubsystemStatus(sub, status, summary, issues))

    return results


def _summarize(subsystem: str, issues: list[Issue]) -> str:
    errs = [i for i in issues if i.severity == SEVERITY_ERR]
    warns = [i for i in issues if i.severity == SEVERITY_WARN]

    if subsystem == "ELECTRICAL":
        parts = []
        brownouts = [i for i in issues if "Brownout" in i.message]
        sags = [i for i in issues if "sag" in i.message]
        spikes = [i for i in issues if "spike" in i.message]
        low = [i for i in issues if "Low battery" in i.message]
        if brownouts:
            parts.append(f"{len(brownouts)} brownout{'s' if len(brownouts) != 1 else ''}")
        if sags:
            parts.append("voltage sag")
        if low:
            parts.append("low battery")
        if spikes:
            parts.append(f"current spike ×{len(spikes)}")
        return ", ".join(parts) if parts else f"{len(issues)} issue(s)"

    if subsystem == "RADIO":
        parts = []
        disconnects = [i for i in issues if "disconnected" in i.message]
        dropouts = [i for i in issues if "dropout" in i.message.lower()]
        err_gaps = [i for i in issues if "loss of driver" in i.message]
        warn_gaps = [i for i in issues if "500–1000ms" in i.message]
        weak_signal = [i for i in issues if "Weak signal" in i.message or "Marginal signal" in i.message]
        low_snr = [i for i in issues if "SNR" in i.message]
        low_rate = [i for i in issues if "link rate" in i.message]
        if disconnects:
            parts.append(f"radio disconnect ×{len(disconnects)}")
        if dropouts:
            total = sum(int(i.message.split("×")[1].split()[0]) for i in dropouts)
            parts.append(f"comms dropout ×{total}")
        if err_gaps:
            parts.append(f"data loss ×{len(err_gaps)}")
        if warn_gaps:
            parts.append("latency spikes")
        if weak_signal:
            parts.append("weak signal")
        if low_snr:
            parts.append("low SNR")
        if low_rate:
            parts.append("degraded link rate")
        return ", ".join(parts) if parts else f"{len(issues)} issue(s)"

    if subsystem == "CAN":
        return issues[0].message.replace("CAN bus errors: ", "") if issues else "—"

    if subsystem == "SHOOTER":
        vel_errs = [i for i in issues if "velocity error" in i.message]
        if vel_errs:
            total_dur = sum(i.time_end - i.time_start for i in vel_errs if i.time_end > 0)
            labels = list({i.detail for i in vel_errs if i.detail})
            label_str = f"  [{', '.join(labels)}]" if labels else ""
            return f"velocity error {total_dur:.1f}s{label_str}"

    if subsystem == "INTAKE":
        stalls = [i for i in issues if "stall" in i.message]
        if stalls:
            labels = list({i.detail for i in stalls if i.detail})
            label_str = f"  [{', '.join(labels)}]" if labels else ""
            return f"stall ×{len(stalls)}{label_str}"

    if subsystem == "TURRET":
        drifts = [i for i in issues if "drift" in i.message]
        if drifts:
            labels = list({i.detail for i in drifts if i.detail})
            label_str = f"  [{', '.join(labels)}]" if labels else ""
            return f"position drift ×{len(drifts)}{label_str}"

    if subsystem == "SWERVE":
        parts = []
        heading = [i for i in issues if "Heading" in i.message]
        jumps = [i for i in issues if "jump" in i.message]
        drift = [i for i in issues if "drift" in i.message.lower() and "Translation" in i.message]
        console = [i for i in issues if "Console" in i.message]
        if heading:
            parts.append(f"heading error ×{len(heading)}")
        if jumps:
            parts.append(f"pose jump ×{len(jumps)}")
        if drift:
            parts.append(f"translation drift ×{len(drift)}")
        if console:
            parts.append(f"{len(console)} console msg")
        return ", ".join(parts) if parts else f"{len(issues)} issue(s)"

    # Generic motor/device summary — used for REVLOG, HOOT, and user-defined subsystems
    return _summarize_device_issues(issues, errs, warns)


def _summarize_device_issues(issues, errs, warns):
    """Categorized summary for any subsystem with motor/device issues."""
    parts = []
    starved = [i for i in issues if "POWER STARVED" in i.message]
    rebooted = [i for i in issues if "REBOOTED" in i.message or "BootDuringEnable" in i.message]
    brownout = [i for i in issues if "BridgeBrownout" in i.message and "sticky" not in i.message]
    hw_fault = [i for i in issues if "HARDWARE FAULT" in i.message or "hardware fault" in i.message.lower()]
    output_lost = [i for i in issues if "OUTPUT LOST" in i.message]
    brake = [i for i in issues if "BRAKE DISABLED" in i.message]
    divergence = [i for i in issues if "DIVERGENCE" in i.message or "diverge" in i.message.lower()]
    sags = [i for i in issues if "voltage sag" in i.message]
    sticky = [i for i in issues if "sticky fault" in i.message.lower()]
    current = [i for i in issues if "current spike" in i.message or "stator current" in i.message]
    temp = [i for i in issues if "temperature" in i.message.lower()]
    stall = [i for i in issues if "stall" in i.message.lower() and "sticky" not in i.message]
    if starved:
        parts.append(f"power starved ×{len(starved)}")
    if rebooted:
        parts.append(f"rebooted ×{len(rebooted)}")
    if hw_fault:
        parts.append(f"hw fault ×{len(hw_fault)}")
    if output_lost:
        parts.append(f"output lost ×{len(output_lost)}")
    if brake:
        parts.append(f"brake disabled ×{len(brake)}")
    if brownout:
        parts.append(f"brownout ×{len(brownout)}")
    if divergence:
        parts.append(f"group divergence ×{len(divergence)}")
    if temp:
        parts.append(f"over-temp ×{len(temp)}")
    if current:
        parts.append(f"current spike ×{len(current)}")
    if stall:
        parts.append(f"stall ×{len(stall)}")
    if sags:
        parts.append(f"voltage sag ×{len(sags)}")
    # Sticky faults at startup are INFO-level and excluded from summary
    if parts:
        return ", ".join(parts)

    # Fallback for issues that don't match any category
    infos = [i for i in issues if i.severity == SEVERITY_INFO]
    if errs:
        return f"{len(errs)} error(s), {len(warns)} warning(s)"
    if warns:
        return f"{len(warns)} warning(s)"
    return f"{len(infos)} info"


@dataclass
class MotorStatus:
    name: str
    status: str       # ERR / WARN / OK
    issues: list      # list[Issue]


def motor_roll_up(all_issues: list[Issue]) -> list[MotorStatus]:
    """
    Build per-motor status from issues that have a `detail` field set.
    Groups issues by the `detail` field (motor/device label) and computes status.
    Only includes actual motors/devices — skips subsystem-level issues.
    """
    # Skip subsystems that are purely software/system-level
    skip_subsystems = {"ELECTRICAL", "CAN", "RADIO"}
    # detail values that are subsystem names, not motor names
    skip_details = set(DEFAULT_SUBSYSTEMS) | _MOTOR_SUBSYSTEMS
    by_motor: dict[str, list[Issue]] = {}

    for issue in all_issues:
        if issue.subsystem in skip_subsystems:
            continue
        if not issue.detail:
            continue
        if issue.detail in skip_details:
            continue
        label = issue.detail
        if label not in by_motor:
            by_motor[label] = []
        by_motor[label].append(issue)

    results = []
    for label in sorted(by_motor.keys()):
        issues = by_motor[label]
        has_err = any(i.severity == SEVERITY_ERR for i in issues)
        has_warn = any(i.severity == SEVERITY_WARN for i in issues)
        if has_err:
            status = SEVERITY_ERR
        elif has_warn:
            status = SEVERITY_WARN
        else:
            # Only INFO-level issues (e.g., sticky faults at startup) — skip
            continue
        results.append(MotorStatus(label, status, issues))

    return results
