"""
Device configuration loader.

Supports two can_map.json formats:

Legacy (flat):
  {"PDH:0": "Front Left Drive", "SPARK:16": "FL SPARK"}

Extended (with subsystems, motor groups, and control modes):
  {
    "subsystems": ["ELEVATOR", "INTAKE"],
    "devices": {
      "CTRE:TalonFX-29": {"name": "Elevator Left", "subsystem": "ELEVATOR",
                           "control_mode": "position"},
      "CTRE:TalonFX-3":  {"name": "FL Drive",      "subsystem": "SWERVE",
                           "control_mode": "velocity"},
      "SPARK:16":         {"name": "Indexer",        "subsystem": "SHOOTER"}
    },
    "motor_groups": [
      {
        "name": "Elevator",
        "subsystem": "ELEVATOR",
        "motors": ["CTRE:TalonFX-29", "CTRE:TalonFX-34"],
        "relationship": "same_direction",
        "ratio": 1.0
      }
    ]
  }

Valid control_mode values:
  - "position"          Position closed-loop (PID following error is monitored)
  - "velocity"          Velocity closed-loop (position error is ignored)
  - "motion_profile"    Motion profiling (position error is monitored)

If control_mode is omitted, position following error checks are skipped for
that device (conservative default to avoid false positives).

Both formats can coexist — flat keys are treated as legacy device names
with no subsystem assignment.
"""

from dataclasses import dataclass, field


CONTROL_MODE_POSITION = "position"
CONTROL_MODE_VELOCITY = "velocity"
CONTROL_MODE_MOTION_PROFILE = "motion_profile"
_VALID_CONTROL_MODES = {CONTROL_MODE_POSITION, CONTROL_MODE_VELOCITY, CONTROL_MODE_MOTION_PROFILE}


@dataclass
class DeviceInfo:
    key: str            # e.g. "CTRE:TalonFX-29", "SPARK:16", "PDH:0"
    name: str           # human-readable label
    subsystem: str      # subsystem to route issues to, or ""
    control_mode: str = ""  # "position", "velocity", "motion_profile", or "" (unknown)


@dataclass
class MotorGroup:
    name: str
    subsystem: str
    motors: list        # list of device keys
    relationship: str   # "same_direction" or "opposite_direction"
    ratio: float        # gear ratio between motors


@dataclass
class DeviceConfig:
    devices: dict       # key -> DeviceInfo
    groups: list        # list[MotorGroup]
    extra_subsystems: list  # user-declared subsystem names

    def label(self, device_key: str, fallback: str = "") -> str:
        """Get the human-readable name for a device."""
        info = self.devices.get(device_key)
        if info:
            return info.name
        return fallback or device_key

    def subsystem(self, device_key: str, fallback: str = "") -> str:
        """Get the subsystem for a device, or fallback."""
        info = self.devices.get(device_key)
        if info and info.subsystem:
            return info.subsystem
        return fallback

    def control_mode(self, device_key: str) -> str:
        """Get the control mode for a device, or '' if unknown."""
        info = self.devices.get(device_key)
        if info:
            return info.control_mode
        return ""


def load_device_config(raw: dict) -> DeviceConfig:
    """Parse a can_map.json dict into a DeviceConfig."""
    devices = {}
    groups = []
    extra_subsystems = []

    # Extended format: "subsystems" key
    if "subsystems" in raw:
        extra_subsystems = [s.upper() for s in raw["subsystems"]]

    # Extended format: "devices" key
    if "devices" in raw:
        for key, val in raw["devices"].items():
            if isinstance(val, dict):
                cm = val.get("control_mode", "").lower()
                if cm and cm not in _VALID_CONTROL_MODES:
                    cm = ""
                devices[key] = DeviceInfo(
                    key=key,
                    name=val.get("name", key),
                    subsystem=val.get("subsystem", "").upper(),
                    control_mode=cm,
                )
            elif isinstance(val, str):
                devices[key] = DeviceInfo(key=key, name=val, subsystem="")

    # Extended format: "motor_groups" key
    if "motor_groups" in raw:
        for g in raw["motor_groups"]:
            groups.append(MotorGroup(
                name=g.get("name", ""),
                subsystem=g.get("subsystem", "").upper(),
                motors=g.get("motors", []),
                relationship=g.get("relationship", "same_direction"),
                ratio=g.get("ratio", 1.0),
            ))

    # Legacy format: flat key-value pairs (skip known extended keys)
    extended_keys = {"subsystems", "devices", "motor_groups"}
    for key, val in raw.items():
        if key in extended_keys:
            continue
        if key not in devices:
            if isinstance(val, str):
                devices[key] = DeviceInfo(key=key, name=val, subsystem="")
            elif isinstance(val, (int, float)):
                # Legacy: "SHOOTER/Flywheel": 10 — CAN ID mapping
                devices[key] = DeviceInfo(key=key, name=key, subsystem="")

    return DeviceConfig(
        devices=devices,
        groups=groups,
        extra_subsystems=extra_subsystems,
    )
