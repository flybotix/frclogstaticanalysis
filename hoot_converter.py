"""
CTRE hoot file converter.

The .hoot format is proprietary and compressed. This module wraps CTRE's
closed-source `owlet` CLI to convert .hoot → .wpilog, then parses the
result with the existing wpilog parser.

Owlet binaries are downloaded from redist.ctr-electronics.com on first use
and cached in tools/.
"""

import json
import os
import platform
import stat
import struct
import subprocess
import sys
import tempfile
from urllib.request import urlopen, Request

from parser import parse_wpilog

TOOLS_DIR = os.path.join(os.path.dirname(__file__), "tools")
OWLET_PATH = os.path.join(TOOLS_DIR, "owlet")
REDIST_URL = "https://redist.ctr-electronics.com/index.json"

# Map compliancy version to minimum owlet version that supports it
_COMPLIANCY_MAP = {
    6: "1.0.1.1",
    9: "25.0.0",
    10: "25.0.1",
    11: "25.0.2",
    13: "25.1.0",
    18: "26.0.0",
    19: "26.1.0",
    20: "25.90.0",
}


def read_hoot_header(filepath: str) -> dict:
    """Read the 80-byte hoot file header."""
    with open(filepath, "rb") as f:
        data = f.read(0x50)
    bus_name = data[0x00:0x40].rstrip(b"\x00").decode("ascii", errors="replace")
    firmware = data[0x40:0x46].rstrip(b"\x00").decode("ascii", errors="replace")
    compliancy = data[0x46]
    timestamp = struct.unpack_from("<I", data, 0x48)[0]
    return {
        "bus_name": bus_name,
        "firmware": firmware,
        "compliancy": compliancy,
        "timestamp": timestamp,
    }


def _platform_suffix() -> str:
    """Determine the owlet binary suffix for this platform."""
    system = platform.system().lower()
    machine = platform.machine().lower()
    if system == "darwin":
        return "macosuniversal"
    if system == "linux":
        if "aarch64" in machine or "arm64" in machine:
            return "linuxarm64"
        if "arm" in machine:
            return "linuxarm32"
        return "linuxx86-64"
    if system == "windows":
        return "windowsx86-64.exe"
    raise RuntimeError(f"Unsupported platform: {system} {machine}")


def _find_owlet_url(min_compliancy: int) -> str | None:
    """Query the CTRE redist index for a compatible owlet download URL."""
    req = Request(REDIST_URL, headers={"User-Agent": "AdvantageKitAnalysis/1.0"})
    with urlopen(req, timeout=30) as resp:
        index = json.loads(resp.read())

    suffix = _platform_suffix()
    best_url = None
    best_version = None

    for entry in index:
        path = entry.get("path", "")
        if "owlet" not in path.lower():
            continue
        if suffix not in path:
            continue
        # Extract version from path like tools/owlet/26.1.0/owlet-26.1.0-macosuniversal
        parts = path.split("/")
        for p in parts:
            if p and p[0].isdigit() and "." in p:
                version = p
                break
        else:
            continue

        # Check if this version supports the required compliancy
        for comp, min_ver in sorted(_COMPLIANCY_MAP.items()):
            if comp >= min_compliancy and version >= min_ver:
                if best_version is None or version > best_version:
                    best_version = version
                    best_url = f"https://redist.ctr-electronics.com/{path}"
                break

    return best_url


def ensure_owlet(compliancy: int = 19) -> str:
    """Download owlet if not already present. Returns path to owlet binary."""
    if os.path.isfile(OWLET_PATH) and os.access(OWLET_PATH, os.X_OK):
        return OWLET_PATH

    os.makedirs(TOOLS_DIR, exist_ok=True)

    print(f"Downloading owlet (compliancy C{compliancy})...", file=sys.stderr)
    url = _find_owlet_url(compliancy)
    if not url:
        raise RuntimeError(
            f"Could not find owlet binary for compliancy C{compliancy}. "
            f"Download manually from https://redist.ctr-electronics.com"
        )

    req = Request(url, headers={"User-Agent": "AdvantageKitAnalysis/1.0"})
    with urlopen(req, timeout=120) as resp:
        data = resp.read()

    with open(OWLET_PATH, "wb") as f:
        f.write(data)
    os.chmod(OWLET_PATH, os.stat(OWLET_PATH).st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)

    print(f"Downloaded owlet to {OWLET_PATH}", file=sys.stderr)
    return OWLET_PATH


def convert_hoot(hoot_path: str, output_path: str | None = None) -> str:
    """
    Convert a .hoot file to .wpilog using owlet.

    Args:
        hoot_path: Path to the .hoot file
        output_path: Optional output path. Defaults to same directory with .wpilog extension.

    Returns:
        Path to the converted .wpilog file.
    """
    header = read_hoot_header(hoot_path)
    owlet = ensure_owlet(header["compliancy"])

    if output_path is None:
        base = os.path.splitext(hoot_path)[0]
        output_path = base + "_converted.wpilog"

    result = subprocess.run(
        [owlet, "-f", "wpilog", hoot_path, output_path],
        capture_output=True, text=True, timeout=300,
    )

    if not os.path.isfile(output_path):
        raise RuntimeError(
            f"owlet conversion failed for {hoot_path}:\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}"
        )

    return output_path


def parse_hoot(hoot_path: str) -> tuple[dict, dict, dict]:
    """
    Convert and parse a .hoot file.

    Returns:
        (channels, types, header) where header contains bus_name, firmware, etc.
    """
    header = read_hoot_header(hoot_path)

    # Use a deterministic converted path to cache results
    base = os.path.splitext(hoot_path)[0]
    converted_path = base + "_converted.wpilog"

    if not os.path.isfile(converted_path):
        convert_hoot(hoot_path, converted_path)

    channels, types = parse_wpilog(converted_path)

    # Normalize timestamps: owlet uses Unix epoch microseconds,
    # convert to session-relative seconds if timestamps are large
    if channels:
        first_ts = None
        for series in channels.values():
            if series:
                if first_ts is None or series[0][0] < first_ts:
                    first_ts = series[0][0]
        if first_ts and first_ts > 1e9:  # Unix epoch in seconds (> year 2001)
            for name in channels:
                channels[name] = [(t - first_ts, v) for t, v in channels[name]]

    return channels, types, header
