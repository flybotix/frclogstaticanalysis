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
    ],
    "swerve": {
      "max_rotation": "180 deg/s",
      "translate_input": {"controller": 0, "x_axis": 0, "y_axis": 1},
      "rotate_input":    {"controller": 0, "axis": 4}
    }
  }

Valid control_mode values:
  - "position"          Position closed-loop (PID following error is monitored)
  - "velocity"          Velocity closed-loop (position error is ignored)
  - "motion_profile"    Motion profiling (position error is monitored)

If control_mode is omitted, position following error checks are skipped for
that device (conservative default to avoid false positives).

swerve.max_rotation accepts:
  - A number (treated as deg/s): 180
  - A string with units: "180 deg/s", "3.14 rad/s"

swerve.translate_input: identifies the controller and axes used for swerve
  translation.  {"controller": 0, "x_axis": 0, "y_axis": 1}
  controller is the DriverStation joystick slot (0-5), x_axis / y_axis are
  the zero-based axis indices on that controller.

swerve.rotate_input: identifies the controller and axis used for swerve
  rotation.  {"controller": 0, "axis": 4}
  May reference a different controller than translate_input.

If either is omitted, the corresponding raw joystick values are not
included in yaw diagnostic messages.

Both formats can coexist — flat keys are treated as legacy device names
with no subsystem assignment.
"""

import math
import re
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
class SwerveTranslateInput:
    controller: int     # DriverStation joystick slot (0-5)
    x_axis: int         # axis index for strafe
    y_axis: int         # axis index for forward/back


@dataclass
class SwerveRotateInput:
    controller: int     # DriverStation joystick slot (0-5)
    axis: int           # axis index for rotation


@dataclass
class DeviceConfig:
    devices: dict       # key -> DeviceInfo
    groups: list        # list[MotorGroup]
    extra_subsystems: list  # user-declared subsystem names
    swerve_max_omega_rad: float = 0.0  # max swerve rotation in rad/s (0 = not configured)
    swerve_translate_input: SwerveTranslateInput = None  # translation joystick mapping
    swerve_rotate_input: SwerveRotateInput = None        # rotation joystick mapping

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


def _parse_rotation_rate(value) -> float:
    """
    Parse a rotation rate into rad/s.

    Accepts:
      - A number (treated as deg/s): 180 -> pi rad/s
      - A string with units: "180 deg/s", "3.14 rad/s"
    Returns rad/s, or 0.0 if unparseable.
    """
    if isinstance(value, (int, float)):
        return math.radians(float(value))
    if isinstance(value, str):
        m = re.match(r"^\s*([\d.]+)\s*(deg/?s|rad/?s)\s*$", value.strip(), re.IGNORECASE)
        if m:
            num = float(m.group(1))
            unit = m.group(2).lower().replace("/", "")
            if unit == "degs":
                return math.radians(num)
            elif unit == "rads":
                return num
    return 0.0


def load_device_config(raw: dict) -> DeviceConfig:
    """Parse a can_map.json dict into a DeviceConfig."""
    devices = {}
    groups = []
    extra_subsystems = []
    swerve_max_omega_rad = 0.0

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

    # Swerve configuration
    swerve_translate_input = None
    swerve_rotate_input = None
    if "swerve" in raw and isinstance(raw["swerve"], dict):
        swerve = raw["swerve"]
        mr = swerve.get("max_rotation")
        if mr is not None:
            swerve_max_omega_rad = _parse_rotation_rate(mr)
        ti = swerve.get("translate_input")
        if isinstance(ti, dict):
            try:
                swerve_translate_input = SwerveTranslateInput(
                    controller=int(ti["controller"]),
                    x_axis=int(ti["x_axis"]),
                    y_axis=int(ti["y_axis"]),
                )
            except (KeyError, TypeError, ValueError):
                pass
        ri = swerve.get("rotate_input")
        if isinstance(ri, dict):
            try:
                swerve_rotate_input = SwerveRotateInput(
                    controller=int(ri["controller"]),
                    axis=int(ri["axis"]),
                )
            except (KeyError, TypeError, ValueError):
                pass

    # Legacy format: flat key-value pairs (skip known extended keys)
    extended_keys = {"subsystems", "devices", "motor_groups", "swerve"}
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
        swerve_max_omega_rad=swerve_max_omega_rad,
        swerve_translate_input=swerve_translate_input,
        swerve_rotate_input=swerve_rotate_input,
    )
