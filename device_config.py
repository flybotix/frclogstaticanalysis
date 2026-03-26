"""
Device configuration loader.

Supports two can_map.json formats:

Legacy (flat):
  {"PDH:0": "Front Left Drive", "SPARK:16": "FL SPARK"}

Extended (with subsystems and motor groups):
  {
    "subsystems": ["ELEVATOR", "INTAKE"],
    "devices": {
      "CTRE:TalonFX-29": {"name": "Elevator Left", "subsystem": "ELEVATOR"},
      "SPARK:16":         {"name": "Indexer",        "subsystem": "SHOOTER"},
      "PDH:0":            {"name": "FL Drive",       "subsystem": "SWERVE"}
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

Both formats can coexist — flat keys are treated as legacy device names
with no subsystem assignment.
"""

from dataclasses import dataclass, field


@dataclass
class DeviceInfo:
    key: str            # e.g. "CTRE:TalonFX-29", "SPARK:16", "PDH:0"
    name: str           # human-readable label
    subsystem: str      # subsystem to route issues to, or ""


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
                devices[key] = DeviceInfo(
                    key=key,
                    name=val.get("name", key),
                    subsystem=val.get("subsystem", "").upper(),
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
