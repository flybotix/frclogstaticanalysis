#!/usr/bin/env python3
"""
AdvantageKit Log Analyzer
Usage:
  python analyze.py <logfile.wpilog>
  python analyze.py <logfile.wpilog> --revlog <file> --hoot <file> [<file>]
  python analyze.py --dir <directory> --time 2026-03-24_23-38
  python analyze.py --batch <directory>
"""

import argparse
import glob
import json
import os
import re
import sys

from parser import parse_wpilog, get_enabled_intervals, get_match_info, get_game_mode_timeline
from revlog_parser import parse_revlog
from hoot_converter import parse_hoot
from device_config import load_device_config, DeviceConfig
from signals import duration, get
from analyzers import (analyze_electrical, analyze_mechanical, analyze_revlog,
                       analyze_hoot, analyze_motor_groups, roll_up)
from analyzers.subsystems import motor_roll_up
from report import print_report, print_batch_header, print_batch_row


CAN_MAP_FILE = os.path.join(os.path.dirname(__file__), "can_map.json")


def load_can_map() -> tuple[dict, DeviceConfig]:
    """Load can_map.json and return both raw dict and parsed DeviceConfig."""
    raw = {}
    if os.path.exists(CAN_MAP_FILE):
        try:
            with open(CAN_MAP_FILE) as f:
                raw = json.load(f)
        except Exception:
            pass
    return raw, load_device_config(raw)


def discover_files(directory: str, timestamp: str) -> dict:
    """
    Auto-discover log files in a directory and all subdirectories matching
    a timestamp prefix.

    Searches recursively because FRC vendors place logs in different directory
    structures (e.g., REV logs in one folder, CTRE hoot files in another,
    AdvantageKit wpilog files in yet another). All that's required is that
    the timestamp appears somewhere in the filename.

    Args:
        directory: Root directory to search (recurses into subdirectories)
        timestamp: Timestamp prefix to match (e.g., "2026-03-24_23-38")

    Returns:
        dict with keys: wpilog, revlog, hoot (list of hoot file paths)
    """
    result = {"wpilog": None, "revlog": None, "hoot": []}

    # Normalize timestamp for matching: allow partial timestamps
    ts = timestamp.replace(":", "-")
    # YY variant for wpilog files (2026-03-24 → 26-03-24)
    ts_short = ts[2:] if len(ts) >= 4 and ts[:4].isdigit() else ts
    # Compact variant for revlog files (2026-03-24_23-38 → 2026032423-38 → 20260324)
    ts_compact = re.sub(r"[-_:]", "", ts)

    for dirpath, _dirnames, filenames in os.walk(directory):
        for f in filenames:
            full = os.path.join(dirpath, f)

            if f.endswith(".wpilog") and not f.endswith("_converted.wpilog"):
                if ts_short in f or ts in f:
                    result["wpilog"] = full

            elif f.endswith(".revlog"):
                if ts_compact[:8] in f:
                    result["revlog"] = full

            elif f.endswith(".hoot"):
                if ts in f:
                    result["hoot"].append(full)

    return result


def analyze_file(filepath: str, can_map: dict, subsystem_filter: str = None,
                 verbose: bool = False, revlog_path: str = None,
                 hoot_paths: list = None, detail_level: str = "ERR",
                 config: DeviceConfig = None) -> list:
    """Parse and analyze a single log file. Returns list of SubsystemStatus."""
    channels, types = {}, {}
    if filepath:
        try:
            channels, types = parse_wpilog(filepath)
        except Exception as e:
            print(f"Error reading {filepath}: {e}", file=sys.stderr)
            return []

    match_info = get_match_info(channels)
    total_dur = duration(channels)

    enabled_intervals = get_enabled_intervals(channels)
    enabled_sec = sum(end - start for start, end in enabled_intervals)

    game_timeline, match_start = get_game_mode_timeline(channels)

    elec_issues = analyze_electrical(channels, can_map)
    mech_issues = analyze_mechanical(channels, can_map)

    # Collect all device channels for motor group analysis
    all_device_channels = dict(channels)

    rev_issues = []
    if revlog_path:
        try:
            rev_channels, rev_types = parse_revlog(revlog_path)
            rev_issues = analyze_revlog(rev_channels, can_map, config=config)
            all_device_channels.update(rev_channels)
            if verbose:
                types.update(rev_types)
                channels.update(rev_channels)
        except Exception as e:
            print(f"Error reading {revlog_path}: {e}", file=sys.stderr)

    hoot_issues = []
    for hoot_path in (hoot_paths or []):
        try:
            hoot_channels, hoot_types, header = parse_hoot(hoot_path)
            bus_name = header.get("bus_name", "")
            hoot_issues.extend(analyze_hoot(hoot_channels, can_map,
                                            bus_name=bus_name, config=config))
            all_device_channels.update(hoot_channels)
            if verbose:
                for name in list(hoot_channels.keys()):
                    prefixed = f"[{bus_name}] {name}" if bus_name else name
                    channels[prefixed] = hoot_channels[name]
                    types[prefixed] = hoot_types.get(name, "?")
            if not total_dur:
                total_dur = duration(hoot_channels)
        except Exception as e:
            print(f"Error reading {hoot_path}: {e}", file=sys.stderr)

    # Motor group comparison analysis
    group_issues = []
    if config and config.groups:
        group_issues = analyze_motor_groups(all_device_channels, config)

    all_issues = elec_issues + mech_issues + rev_issues + hoot_issues + group_issues

    extra_subs = config.extra_subsystems if config else []
    statuses = roll_up(all_issues, extra_subsystems=extra_subs)
    motor_statuses = motor_roll_up(all_issues)

    display_path = filepath or (hoot_paths[0] if hoot_paths else "unknown")
    print_report(
        filepath=display_path,
        match_info=match_info,
        duration_sec=total_dur,
        enabled_sec=enabled_sec,
        statuses=statuses,
        filter_subsystem=subsystem_filter,
        motor_statuses=motor_statuses,
        detail_level=detail_level,
        game_timeline=game_timeline,
        match_start=match_start,
    )

    if verbose:
        _print_verbose(channels, types)

    return statuses


def _print_verbose(channels: dict, types: dict):
    print("\n── Available channels ──────────────────────────────────────")
    for name in sorted(channels.keys()):
        count = len(channels[name])
        print(f"  {name}  [{types.get(name, '?')}]  ({count} samples)")
    print()


def batch_mode(directory: str, can_map: dict):
    pattern = os.path.join(directory, "*.wpilog")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No .wpilog files found in {directory}", file=sys.stderr)
        sys.exit(1)

    print(f"\nAnalyzing {len(files)} log files in {directory}\n")
    print_batch_header()

    for filepath in files:
        try:
            channels, types = parse_wpilog(filepath)
        except Exception as e:
            print(f"  [error] {os.path.basename(filepath)}: {e}", file=sys.stderr)
            continue

        total_dur = duration(channels)
        elec_issues = analyze_electrical(channels, can_map)
        mech_issues = analyze_mechanical(channels, can_map)
        statuses = roll_up(elec_issues + mech_issues)
        print_batch_row(os.path.basename(filepath), total_dur, statuses)


def main():
    parser = argparse.ArgumentParser(
        description="FRC match log analyzer — surfaces mechanical, electrical, and motor controller issues."
    )
    parser.add_argument("logfile", nargs="?", help="Path to .wpilog file")
    parser.add_argument("--revlog", "-r", metavar="FILE",
                        help="Path to .revlog file (REV Robotics motor data)")
    parser.add_argument("--hoot", metavar="FILE", nargs="+",
                        help="Path to .hoot file(s) (CTRE Phoenix 6 signal logs)")
    parser.add_argument("--dir", "-d", metavar="DIR",
                        help="Directory containing log files (use with --time)")
    parser.add_argument("--time", "-t", metavar="TIMESTAMP",
                        help="Timestamp prefix for auto-discovery (e.g., 2026-03-24_23-38)")
    parser.add_argument("--subsystem", "-s", metavar="NAME",
                        help="Filter output to one subsystem (e.g. SHOOTER)")
    parser.add_argument("--warn", "-w", action="store_true",
                        help="Show warnings and errors in detailed output (default: errors only)")
    parser.add_argument("--info", "-i", action="store_true",
                        help="Show all issues including info-level in detailed output")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Show all available channels")
    parser.add_argument("--batch", "-b", metavar="DIR",
                        help="Batch-analyze all .wpilog files in directory")

    args = parser.parse_args()

    if args.info:
        detail_level = "INFO"
    elif args.warn:
        detail_level = "WARN"
    else:
        detail_level = "ERR"

    can_map, config = load_can_map()

    if args.batch:
        batch_mode(args.batch, can_map)
        return

    # Auto-discovery mode
    if args.dir and args.time:
        if not os.path.isdir(args.dir):
            print(f"Directory not found: {args.dir}", file=sys.stderr)
            sys.exit(1)

        found = discover_files(args.dir, args.time)
        sources = []
        if found["wpilog"]:
            sources.append(f"wpilog: {os.path.basename(found['wpilog'])}")
        if found["revlog"]:
            sources.append(f"revlog: {os.path.basename(found['revlog'])}")
        for h in found["hoot"]:
            sources.append(f"hoot: {os.path.basename(h)}")

        if not any([found["wpilog"], found["revlog"], found["hoot"]]):
            print(f"No log files found matching timestamp '{args.time}' in {args.dir}",
                  file=sys.stderr)
            sys.exit(1)

        print(f"Auto-discovered {len(sources)} file(s):")
        for s in sources:
            print(f"  {s}")
        print()

        analyze_file(
            filepath=found["wpilog"],
            can_map=can_map,
            subsystem_filter=args.subsystem,
            verbose=args.verbose,
            revlog_path=found["revlog"],
            hoot_paths=found["hoot"],
            detail_level=detail_level,
            config=config,
        )
        return

    # Manual mode
    if not args.logfile and not args.revlog and not args.hoot:
        parser.print_help()
        sys.exit(1)

    if args.logfile and not os.path.exists(args.logfile):
        print(f"File not found: {args.logfile}", file=sys.stderr)
        sys.exit(1)

    if args.revlog and not os.path.exists(args.revlog):
        print(f"File not found: {args.revlog}", file=sys.stderr)
        sys.exit(1)

    for h in (args.hoot or []):
        if not os.path.exists(h):
            print(f"File not found: {h}", file=sys.stderr)
            sys.exit(1)

    analyze_file(
        filepath=args.logfile,
        can_map=can_map,
        subsystem_filter=args.subsystem,
        verbose=args.verbose,
        revlog_path=args.revlog,
        hoot_paths=args.hoot,
        detail_level=detail_level,
        config=config,
    )


if __name__ == "__main__":
    main()
