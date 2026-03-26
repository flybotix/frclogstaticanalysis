"""
WPILog binary file parser.
Supports WPILog format version 1.0 (AdvantageKit logs).
"""

import struct
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Entry:
    entry_id: int
    name: str
    type_str: str
    metadata: str


def _read_int(data: bytes, pos: int, size: int) -> tuple[int, int]:
    val = int.from_bytes(data[pos:pos + size], "little")
    return val, pos + size


def _read_str(data: bytes, pos: int) -> tuple[str, int]:
    length, pos = _read_int(data, pos, 4)
    s = data[pos:pos + length].decode("utf-8", errors="replace")
    return s, pos + length


def _decode_value(payload: bytes, type_str: str) -> Any:
    if type_str == "boolean":
        return bool(payload[0]) if payload else None
    if type_str == "int64":
        return struct.unpack_from("<q", payload)[0] if len(payload) >= 8 else None
    if type_str == "float":
        return struct.unpack_from("<f", payload)[0] if len(payload) >= 4 else None
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0] if len(payload) >= 8 else None
    if type_str == "string":
        return payload.decode("utf-8", errors="replace")
    if type_str == "boolean[]":
        return [bool(b) for b in payload]
    if type_str == "int64[]":
        n = len(payload) // 8
        return list(struct.unpack_from(f"<{n}q", payload))
    if type_str == "float[]":
        n = len(payload) // 4
        return list(struct.unpack_from(f"<{n}f", payload))
    if type_str == "double[]":
        n = len(payload) // 8
        return list(struct.unpack_from(f"<{n}d", payload))
    if type_str == "string[]":
        strings = []
        pos = 0
        while pos + 4 <= len(payload):
            length = int.from_bytes(payload[pos:pos + 4], "little")
            pos += 4
            if pos + length <= len(payload):
                strings.append(payload[pos:pos + length].decode("utf-8", errors="replace"))
                pos += length
        return strings
    if type_str == "struct:Pose2d":
        if len(payload) >= 24:
            x, y, rot = struct.unpack_from("<ddd", payload)
            return {"x": x, "y": y, "rotation_rad": rot}
        return None
    if type_str == "struct:ChassisSpeeds":
        if len(payload) >= 24:
            vx, vy, omega = struct.unpack_from("<ddd", payload)
            return {"vx": vx, "vy": vy, "omega": omega}
        return None
    # Unknown type — return raw bytes length as a hint
    return None


def parse_wpilog(filepath: str) -> tuple[dict[str, list[tuple[float, Any]]], dict[str, str]]:
    """
    Parse a .wpilog file.

    Returns:
        channels: dict mapping channel name -> list of (timestamp_sec, value)
        types:    dict mapping channel name -> type string
    """
    with open(filepath, "rb") as f:
        data = f.read()

    if not data.startswith(b"WPILOG"):
        raise ValueError(f"Not a valid WPILog file: {filepath}")

    pos = 6  # skip "WPILOG"
    if len(data) < pos + 2:
        raise ValueError("File too short")

    version = int.from_bytes(data[pos:pos + 2], "little")
    pos += 2  # version (e.g. 0x0100 = 1.0)

    # Extra header string
    if pos + 4 > len(data):
        return {}, {}
    extra_len = int.from_bytes(data[pos:pos + 4], "little")
    pos += 4 + extra_len  # skip extra metadata string

    # Build entry registry and channel data
    entries: dict[int, Entry] = {}
    channels: dict[str, list[tuple[float, Any]]] = {}
    types: dict[str, str] = {}

    while pos < len(data):
        if pos >= len(data):
            break

        # Bit field
        if pos + 1 > len(data):
            break
        bit_field = data[pos]
        pos += 1

        entry_id_size = (bit_field & 0x03) + 1
        payload_size_size = ((bit_field >> 2) & 0x03) + 1
        timestamp_size = ((bit_field >> 4) & 0x07) + 1

        needed = entry_id_size + payload_size_size + timestamp_size
        if pos + needed > len(data):
            break

        entry_id, pos = _read_int(data, pos, entry_id_size)
        payload_size, pos = _read_int(data, pos, payload_size_size)
        timestamp_us, pos = _read_int(data, pos, timestamp_size)

        if pos + payload_size > len(data):
            break
        payload = data[pos:pos + payload_size]
        pos += payload_size

        timestamp_sec = timestamp_us / 1_000_000.0

        if entry_id == 0:
            # Control record
            if len(payload) < 1:
                continue
            ctrl_type = payload[0]

            if ctrl_type == 0:  # Start
                if len(payload) < 5:
                    continue
                new_id = int.from_bytes(payload[1:5], "little")
                p = 5
                name, p = _read_str(payload, p)
                type_str, p = _read_str(payload, p)
                metadata, p = _read_str(payload, p)
                entries[new_id] = Entry(new_id, name, type_str, metadata)
                if name not in channels:
                    channels[name] = []
                    types[name] = type_str

            elif ctrl_type == 1:  # Finish
                pass  # entry retired, keep its data

            elif ctrl_type == 2:  # SetMetadata
                pass

        else:
            # Data record
            entry = entries.get(entry_id)
            if entry is None:
                continue
            value = _decode_value(payload, entry.type_str)
            if value is not None and entry.name in channels:
                channels[entry.name].append((timestamp_sec, value))

    return channels, types


def get_enabled_intervals(channels: dict) -> list[tuple[float, float]]:
    """Return list of (start_sec, end_sec) when robot was enabled."""
    enabled = channels.get("/DriverStation/Enabled", [])
    if not enabled:
        return []
    intervals = []
    start = None
    for t, val in enabled:
        if val and start is None:
            start = t
        elif not val and start is not None:
            intervals.append((start, t))
            start = None
    if start is not None:
        intervals.append((start, enabled[-1][0]))
    return intervals


def get_match_info(channels: dict) -> dict:
    """Extract match metadata from DriverStation channels."""
    def first(ch):
        vals = channels.get(ch, [])
        return vals[0][1] if vals else None

    match_type = first("/DriverStation/MatchType")
    match_num = first("/DriverStation/MatchNumber")
    event = first("/DriverStation/EventName")
    alliance = first("/DriverStation/AllianceStation")

    type_names = {0: "Practice", 1: "Practice", 2: "Qual", 3: "Elim", 4: "Elim"}
    alliance_names = {
        0: "Red 1", 1: "Red 2", 2: "Red 3",
        3: "Blue 1", 4: "Blue 2", 5: "Blue 3",
    }

    if match_type in (None, 0) or not match_num:
        match_str = "No FMS / Practice"
    else:
        type_str = type_names.get(match_type, f"Type{match_type}")
        match_str = f"{type_str} {match_num}"
        if event:
            match_str += f"  |  Event: {event}"
        if alliance is not None:
            match_str += f"  |  Alliance: {alliance_names.get(alliance, str(alliance))}"

    return {"match": match_str}


# Game mode constants
MODE_DISABLED = "DISABLED"
MODE_AUTO = "AUTO"
MODE_TELEOP = "TELEOP"


def get_game_mode_timeline(channels: dict) -> tuple[list[tuple[float, str]], float]:
    """
    Build a timeline of game mode transitions from DriverStation channels.

    Returns:
        (transitions, match_start_time)
        transitions: list of (time_sec, mode) where mode is DISABLED/AUTO/TELEOP
        match_start_time: time of first DISABLED→AUTO transition that is followed
                         by AUTO→TELEOP (standard match sequence), or 0.0 if none
    """
    enabled_raw = channels.get("/DriverStation/Enabled", [])
    auto_raw = channels.get("/DriverStation/Autonomous", [])

    if not enabled_raw:
        return [], 0.0

    # Merge both channels into a single sorted event stream
    events = []
    for t, v in enabled_raw:
        events.append((t, "enabled", bool(v)))
    for t, v in auto_raw:
        events.append((t, "auto", bool(v)))
    events.sort(key=lambda e: e[0])

    # Walk through events and track state
    is_enabled = False
    is_auto = False
    transitions = []

    def _mode():
        if not is_enabled:
            return MODE_DISABLED
        return MODE_AUTO if is_auto else MODE_TELEOP

    current_mode = MODE_DISABLED
    for t, kind, value in events:
        if kind == "enabled":
            is_enabled = value
        elif kind == "auto":
            is_auto = value

        new_mode = _mode()
        if new_mode != current_mode:
            transitions.append((t, new_mode))
            current_mode = new_mode

    if not transitions:
        transitions.append((0.0, MODE_DISABLED))

    # Detect match start: the first enabled mode must be AUTO (standard FMS match).
    # If the first enabled mode is TELEOP, this is a practice session — no match start.
    match_start = 0.0
    for t, mode in transitions:
        if mode == MODE_AUTO:
            match_start = t
            break
        if mode == MODE_TELEOP:
            # First enabled mode was teleop — practice session, no match start
            break

    return transitions, match_start


def get_game_mode_at(transitions: list[tuple[float, str]], t: float) -> str:
    """Look up the game mode at a given timestamp."""
    mode = MODE_DISABLED
    for trans_t, trans_mode in transitions:
        if trans_t > t:
            break
        mode = trans_mode
    return mode


def fmt_match_time(t: float, match_start: float, transitions: list[tuple[float, str]]) -> str:
    """
    Format a timestamp as game mode + match time.

    With match start: "AUTO 0:12" or "TELEOP 2:05"
    Without match start (practice): "TELEOP T+0:40"
    """
    mode = get_game_mode_at(transitions, t)

    if match_start > 0.0:
        mt = t - match_start
        sign = "-" if mt < 0 else ""
        mt = abs(mt)
        m = int(mt) // 60
        s = int(mt) % 60
        return f"{mode} {sign}{m}:{s:02d}"
    else:
        m = int(t) // 60
        s = int(t) % 60
        return f"{mode} T+{m}:{s:02d}"
