"""
Terminal report formatter.
Uses rich if available, falls back to ANSI escape codes.
"""

import os
import sys

try:
    from rich.console import Console
    from rich.table import Table
    from rich import box
    from rich.text import Text
    _HAS_RICH = True
    _console = Console()
except ImportError:
    _HAS_RICH = False
    _console = None


# ANSI fallback colors
_RESET = "\033[0m"
_RED = "\033[91m"
_YELLOW = "\033[93m"
_GREEN = "\033[92m"
_CYAN = "\033[96m"
_BOLD = "\033[1m"
_DIM = "\033[2m"


def _ansi(text, *codes):
    if not sys.stdout.isatty():
        return text
    return "".join(codes) + text + _RESET


from parser import fmt_match_time as _fmt_match_time

DETAIL_LEVELS = {"ERR": 0, "WARN": 1, "INFO": 2}


def _issue_prefix(issue, game_timeline, match_start):
    """Format the game mode + match time prefix for an issue line."""
    if not game_timeline or issue.time_start <= 0:
        return ""
    return f"[{_fmt_match_time(issue.time_start, match_start, game_timeline)}] "


def _status_color_rich(status):
    if status == "ERR":
        return "bold red"
    if status == "WARN":
        return "bold yellow"
    if status == "INFO":
        return "cyan"
    return "green"


def _status_icon(status):
    if status == "ERR":
        return "✗"
    if status == "WARN":
        return "⚠"
    if status == "INFO":
        return "ℹ"
    return "✓"


def _severity_prefix_ansi(severity):
    if severity == "ERR":
        return _ansi("  ✗", _RED, _BOLD)
    if severity == "WARN":
        return _ansi("  ⚠", _YELLOW)
    if severity == "INFO":
        return _ansi("  ℹ", _CYAN)
    return _ansi("  ✓", _GREEN)


def print_report(filepath: str, match_info: dict, duration_sec: float,
                 enabled_sec: float, statuses: list, filter_subsystem: str = None,
                 motor_statuses: list = None, detail_level: str = "ERR",
                 game_timeline: list = None, match_start: float = 0.0):
    """Print the full match analysis report.

    detail_level controls which issues appear in the detailed output:
        "ERR"  — only errors (default)
        "WARN" — errors and warnings
        "INFO" — errors, warnings, and info
    """
    if _HAS_RICH:
        _print_rich(filepath, match_info, duration_sec, enabled_sec, statuses, filter_subsystem,
                    motor_statuses, detail_level, game_timeline, match_start)
    else:
        _print_ansi(filepath, match_info, duration_sec, enabled_sec, statuses, filter_subsystem,
                    motor_statuses, detail_level, game_timeline, match_start)


# ── Rich renderer ─────────────────────────────────────────────────────────────

def _should_show(severity: str, detail_level: str) -> bool:
    """Return True if this severity should be shown at the given detail level."""
    return DETAIL_LEVELS.get(severity, -1) <= DETAIL_LEVELS.get(detail_level, 0)


def _print_rich(filepath, match_info, duration_sec, enabled_sec, statuses, filter_subsystem,
                motor_statuses=None, detail_level="ERR", game_timeline=None, match_start=0.0):
    c = _console
    filename = os.path.basename(filepath) if filepath else "unknown"

    c.print()
    c.print(f"[bold]Match log:[/bold]  {filename}")
    c.print(f"[bold]Match:    [/bold]  {match_info.get('match', 'Unknown')}")

    dur_str = _fmt_dur(duration_sec)
    ena_str = _fmt_dur(enabled_sec)
    c.print(f"[bold]Duration: [/bold]  {dur_str}  |  Enabled: {ena_str}")
    c.print()

    # Summary table
    table = Table(box=box.SIMPLE_HEAD, show_header=True, header_style="bold")
    table.add_column("SUBSYSTEM", style="bold", width=14)
    table.add_column("STATUS", width=8)
    table.add_column("ISSUES")

    for ss in statuses:
        if filter_subsystem and ss.name != filter_subsystem.upper():
            continue
        style = _status_color_rich(ss.status)
        icon = _status_icon(ss.status)
        table.add_row(
            ss.name,
            Text(f"{icon} {ss.status}", style=style),
            ss.summary,
        )

    c.print(table)

    # Motor summary table
    if motor_statuses and not filter_subsystem:
        _print_motor_table_rich(motor_statuses)

    # Detailed sections (filtered by detail_level)
    for ss in statuses:
        if filter_subsystem and ss.name != filter_subsystem.upper():
            continue
        visible = [i for i in ss.issues if _should_show(i.severity, detail_level)]
        if not visible:
            continue
        visible.sort(key=lambda i: i.time_start)

        c.print(f"[bold cyan]─── {ss.name} {'─' * max(1, 60 - len(ss.name))}[/bold cyan]")
        for issue in visible:
            icon = _status_icon(issue.severity)
            style = _status_color_rich(issue.severity)
            gm = _issue_prefix(issue, game_timeline, match_start)
            c.print(f"  [{style}]{icon}[/{style}] {gm}{issue.message}")
        c.print()


def _print_motor_table_rich(motor_statuses):
    c = _console
    c.print(f"[bold cyan]─── MOTOR STATUS {'─' * 47}[/bold cyan]")

    table = Table(box=box.SIMPLE_HEAD, show_header=True, header_style="bold")
    table.add_column("MOTOR", style="bold", width=32)
    table.add_column("STATUS", width=8)
    table.add_column("ISSUES")

    for ms in motor_statuses:
        style = _status_color_rich(ms.status)
        icon = _status_icon(ms.status)
        # Build compact issue summary
        summary = _motor_issue_summary(ms.issues)
        table.add_row(
            ms.name,
            Text(f"{icon} {ms.status}", style=style),
            summary,
        )

    c.print(table)
    c.print()


def _motor_issue_summary(issues: list) -> str:
    """One-line summary of a motor's issues."""
    parts = []
    starved = [i for i in issues if "POWER STARVED" in i.message]
    rebooted = [i for i in issues if "REBOOTED" in i.message or "BootDuringEnable" in i.message]
    brownout = [i for i in issues if "BridgeBrownout" in i.message and "sticky" not in i.message]
    hw_fault = [i for i in issues if "hardware fault" in i.message.lower()
                or "Hardware" in i.message and "Fault_" in i.message and "sticky" not in i.message]
    sags = [i for i in issues if "voltage sag" in i.message or "bus voltage sag" in i.message]
    current = [i for i in issues if "current spike" in i.message or "stator current" in i.message]
    temp = [i for i in issues if "temperature" in i.message.lower()]
    stall = [i for i in issues if "stall" in i.message.lower() and "sticky" not in i.message]
    output_lost = [i for i in issues if "OUTPUT LOST" in i.message]
    brake = [i for i in issues if "BRAKE DISABLED" in i.message]
    drift = [i for i in issues if "drift" in i.message.lower()]
    divergence = [i for i in issues if "diverge" in i.message.lower()]
    has_reset = [i for i in issues if "HasReset" in i.message]
    vel_error = [i for i in issues if "velocity error" in i.message]
    curr_limit = [i for i in issues if "CurrLimit" in i.message]
    supply_fault = [i for i in issues if "OverSupplyV" in i.message or "UnstableSupplyV" in i.message]
    # Catch-all for any remaining non-INFO, non-sticky issues
    known_msgs = (starved + rebooted + brownout + hw_fault + sags + current +
                  temp + stall + output_lost + brake + drift + divergence +
                  has_reset + vel_error + curr_limit + supply_fault)
    known_set = set(id(i) for i in known_msgs)
    # Exclude sticky faults (INFO) from uncategorized
    other = [i for i in issues if id(i) not in known_set
             and i.severity != "INFO" and "sticky" not in i.message.lower()]

    if starved:
        dur = sum(i.time_end - i.time_start for i in starved if i.time_end > 0)
        parts.append(f"POWER STARVED {dur:.0f}s")
    if rebooted:
        parts.append("REBOOTED")
    if brownout:
        parts.append("brownout")
    if hw_fault:
        parts.append("hw fault")
    if temp:
        parts.append("over-temp")
    if current:
        parts.append(f"current spike ×{len(current)}")
    if stall:
        parts.append(f"stall ×{len(stall)}")
    if output_lost:
        parts.append(f"output lost ×{len(output_lost)}")
    if brake:
        parts.append("brake disabled")
    if drift:
        parts.append(f"drift ×{len(drift)}")
    if divergence:
        parts.append(f"divergence ×{len(divergence)}")
    if has_reset:
        parts.append("HasReset")
    if vel_error:
        parts.append(f"velocity error ×{len(vel_error)}")
    if curr_limit:
        parts.append(f"current limited ×{len(curr_limit)}")
    if supply_fault:
        parts.append(f"supply fault ×{len(supply_fault)}")
    if sags:
        parts.append(f"voltage sag ×{len(sags)}")
    # Catch-all for any non-INFO issue not matched above
    if other:
        parts.append(f"{len(other)} other issue(s)")
    # Sticky faults at startup are INFO-level; excluded from motor summary

    return ", ".join(parts) if parts else "ok"


# ── ANSI fallback renderer ────────────────────────────────────────────────────

def _print_ansi(filepath, match_info, duration_sec, enabled_sec, statuses, filter_subsystem,
                motor_statuses=None, detail_level="ERR", game_timeline=None, match_start=0.0):
    filename = os.path.basename(filepath) if filepath else "unknown"
    print()
    print(f"{_BOLD}Match log:{_RESET}  {filename}")
    print(f"{_BOLD}Match:    {_RESET}  {match_info.get('match', 'Unknown')}")
    print(f"{_BOLD}Duration: {_RESET}  {_fmt_dur(duration_sec)}  |  Enabled: {_fmt_dur(enabled_sec)}")
    print()

    # Summary table header
    print(_ansi(f"{'SUBSYSTEM':<14}  {'STATUS':<8}  ISSUES", _BOLD))
    print("─" * 70)

    for ss in statuses:
        if filter_subsystem and ss.name != filter_subsystem.upper():
            continue
        icon = _status_icon(ss.status)
        if ss.status == "ERR":
            status_str = _ansi(f"{icon} {ss.status}", _RED, _BOLD)
        elif ss.status == "WARN":
            status_str = _ansi(f"{icon} {ss.status}", _YELLOW, _BOLD)
        else:
            status_str = _ansi(f"{icon} {ss.status}", _GREEN)
        # Pad for alignment (ANSI codes add invisible chars)
        print(f"{ss.name:<14}  {status_str:<8}  {ss.summary}")

    print()

    # Motor summary table
    if motor_statuses and not filter_subsystem:
        _print_motor_table_ansi(motor_statuses)

    for ss in statuses:
        if filter_subsystem and ss.name != filter_subsystem.upper():
            continue
        visible = [i for i in ss.issues if _should_show(i.severity, detail_level)]
        if not visible:
            continue
        visible.sort(key=lambda i: i.time_start)

        header = f"─── {ss.name} {'─' * max(1, 60 - len(ss.name))}"
        print(_ansi(header, _CYAN, _BOLD))
        for issue in visible:
            prefix = _severity_prefix_ansi(issue.severity)
            gm = _issue_prefix(issue, game_timeline, match_start)
            print(f"{prefix} {gm}{issue.message}")
        print()


def _print_motor_table_ansi(motor_statuses):
    header = f"─── MOTOR STATUS {'─' * 47}"
    print(_ansi(header, _CYAN, _BOLD))
    print(_ansi(f"{'MOTOR':<32}  {'STATUS':<8}  ISSUES", _BOLD))
    print("─" * 70)

    for ms in motor_statuses:
        icon = _status_icon(ms.status)
        if ms.status == "ERR":
            status_str = _ansi(f"{icon} {ms.status}", _RED, _BOLD)
        elif ms.status == "WARN":
            status_str = _ansi(f"{icon} {ms.status}", _YELLOW, _BOLD)
        else:
            status_str = _ansi(f"{icon} {ms.status}", _GREEN)
        summary = _motor_issue_summary(ms.issues)
        print(f"{ms.name:<32}  {status_str:<8}  {summary}")

    print()


def print_batch_row(filename: str, duration_sec: float, statuses: list):
    """Print a single-line summary row for batch mode."""
    has_err = any(ss.status == "ERR" for ss in statuses)
    has_warn = any(ss.status == "WARN" for ss in statuses)
    overall = "ERR" if has_err else ("WARN" if has_warn else "OK")
    icon = _status_icon(overall)

    top_issues = [f"{ss.name}:{ss.status}" for ss in statuses if ss.status != "OK"]
    issues_str = "  ".join(top_issues[:3]) if top_issues else "—"

    if _HAS_RICH:
        style = _status_color_rich(overall)
        _console.print(
            f"{filename:<35}  {_fmt_dur(duration_sec):>6}  "
            f"[{style}]{icon} {overall:<4}[/{style}]  {issues_str}"
        )
    else:
        if overall == "ERR":
            icon_str = _ansi(f"{icon} {overall}", _RED, _BOLD)
        elif overall == "WARN":
            icon_str = _ansi(f"{icon} {overall}", _YELLOW)
        else:
            icon_str = _ansi(f"{icon} {overall}", _GREEN)
        print(f"{filename:<35}  {_fmt_dur(duration_sec):>6}  {icon_str}  {issues_str}")


def print_batch_header():
    header = f"{'FILE':<35}  {'DUR':>6}  {'STATUS':<8}  ISSUES"
    if _HAS_RICH:
        _console.print(f"[bold]{header}[/bold]")
        _console.print("─" * 80)
    else:
        print(_ansi(header, _BOLD))
        print("─" * 80)


def _fmt_dur(sec: float) -> str:
    if sec <= 0:
        return "0:00"
    m = int(sec) // 60
    s = int(sec) % 60
    return f"{m}:{s:02d}"
