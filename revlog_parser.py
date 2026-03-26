"""
REV Robotics .revlog binary parser.

Format: flat sequence of variable-length records.
  [1 byte bitfield] [1-4 bytes entry_id] [1-4 bytes payload_size] [N bytes payload]

Entry ID 1 = Firmware record  (N × 10-byte chunks)
Entry ID 2 = Periodic record  (N × 16-byte chunks)

Decodes SPARK MAX / SPARK FLEX status frames into named channels
using the same (channels, types) shape as parse_wpilog().
"""

import struct
from typing import Any


# FRC CAN device type for motor controllers (SPARK MAX / FLEX)
_SPARK_DEVICE_TYPE = 2
_REV_MANUFACTURER = 5

# API class for SPARK periodic status frames (0x2E = 46 in FRC extended CAN)
_SPARK_STATUS_API_CLASS = 46

# API indices for SPARK status frames
_STATUS_0 = 0   # applied output, voltage, current, temp
_STATUS_1 = 1   # faults, sticky faults, warnings
_STATUS_2 = 2   # primary encoder velocity, position
_STATUS_3 = 3   # analog sensor voltage, velocity, position
_STATUS_4 = 4   # external/alternate encoder velocity, position
_STATUS_5 = 5   # duty cycle encoder velocity, position
_STATUS_6 = 6   # duty cycle raw, period
_STATUS_7 = 7   # I-accumulation
_STATUS_8 = 8   # closed-loop setpoint
_STATUS_9 = 9   # MAXMotion setpoint

# SPARK fault bit definitions (active faults and sticky faults share the same layout)
FAULT_BITS = {
    0:  "Brownout",
    1:  "Overcurrent",
    2:  "iAccumulation",
    3:  "AllSensorsOut",
    4:  "GateDriver",
    5:  "EscEeprom",
    6:  "MotorType",
    8:  "SoftLimitFwd",
    9:  "SoftLimitRev",
    10: "HardLimitFwd",
    11: "HardLimitRev",
    14: "Stall",
    17: "HasReset",
}

HARDWARE_FAULT_MASK = (1 << 0) | (1 << 1) | (1 << 4)  # Brownout | Overcurrent | GateDriver


def decode_fault_names(mask: int) -> list[str]:
    """Return list of fault name strings set in the bitmask."""
    return [name for bit, name in FAULT_BITS.items() if mask & (1 << bit)]


def _decode_can_id(msg_id: int) -> tuple[int, int, int, int, int]:
    device_type  = (msg_id >> 24) & 0x1F
    manufacturer = (msg_id >> 16) & 0xFF
    api_class    = (msg_id >> 10) & 0x3F
    api_index    = (msg_id >> 6)  & 0x0F
    device_num   =  msg_id        & 0x3F
    return device_type, manufacturer, api_class, api_index, device_num


def _decode_status0(frame: bytes) -> dict[str, float] | None:
    """Status 0: applied output (16-bit signed LE), bus voltage (12-bit), current (12-bit), temp (8-bit)."""
    if len(frame) < 8:
        return None
    applied_raw = struct.unpack_from("<h", frame, 0)[0]   # signed 16-bit LE
    applied = applied_raw * 3.082369457075716e-5           # DBC: ≈ 1/32447

    # Bytes 2–4: voltage 12-bit (bits 16-27) + current 12-bit (bits 28-39) packed
    raw24 = frame[2] | (frame[3] << 8) | (frame[4] << 16)
    voltage_raw = raw24 & 0xFFF
    current_raw = (raw24 >> 12) & 0xFFF
    voltage = voltage_raw * 7.3260073260073e-3             # DBC: ≈ 1/136.5
    current = current_raw * 3.66300366300366e-2            # DBC: ≈ 1/27.3

    temp = frame[5]  # byte 5: motor temperature °C

    return {
        "AppliedOutput": applied,
        "BusVoltage":    voltage,
        "OutputCurrent": current,
        "MotorTemp":     float(temp),
    }


def _decode_status1(frame: bytes) -> dict[str, int] | None:
    """Status 1: active faults (32-bit LE) and sticky faults (32-bit LE)."""
    if len(frame) < 8:
        return None
    faults        = struct.unpack_from("<I", frame, 0)[0]
    sticky_faults = struct.unpack_from("<I", frame, 4)[0]
    return {"Faults": faults, "StickyFaults": sticky_faults}


def _decode_status2(frame: bytes) -> dict[str, float] | None:
    """Status 2: velocity (float32 RPM) and position (float32 rotations)."""
    if len(frame) < 8:
        return None
    velocity = struct.unpack_from("<f", frame, 0)[0]
    position = struct.unpack_from("<f", frame, 4)[0]
    return {"Velocity": velocity, "Position": position}


def _decode_status3(frame: bytes) -> dict[str, float] | None:
    """Status 3: analog sensor voltage (10-bit), velocity (22-bit signed), position (float32)."""
    if len(frame) < 8:
        return None
    raw32 = frame[0] | (frame[1] << 8) | (frame[2] << 16) | (frame[3] << 24)
    analog_voltage_raw = raw32 & 0x3FF
    analog_velocity_raw = (raw32 >> 10) & 0x3FFFFF
    # Sign-extend 22-bit
    if analog_velocity_raw & 0x200000:
        analog_velocity_raw -= 0x400000
    analog_voltage = analog_voltage_raw * 4.8973607038123e-3  # DBC factor
    analog_velocity = analog_velocity_raw * 7.812026887906498e-3  # DBC factor, RPM
    analog_position = struct.unpack_from("<f", frame, 4)[0]
    return {
        "AnalogVoltage": analog_voltage,
        "AnalogVelocity": analog_velocity,
        "AnalogPosition": analog_position,
    }


def _decode_status4(frame: bytes) -> dict[str, float] | None:
    """Status 4: external/alternate encoder velocity (float32) and position (float32)."""
    if len(frame) < 8:
        return None
    velocity = struct.unpack_from("<f", frame, 0)[0]
    position = struct.unpack_from("<f", frame, 4)[0]
    return {"AltEncoderVelocity": velocity, "AltEncoderPosition": position}


def _decode_status5(frame: bytes) -> dict[str, float] | None:
    """Status 5: duty cycle encoder velocity (float32) and position (float32)."""
    if len(frame) < 8:
        return None
    velocity = struct.unpack_from("<f", frame, 0)[0]
    position = struct.unpack_from("<f", frame, 4)[0]
    return {"DutyCycleVelocity": velocity, "DutyCyclePosition": position}


def _decode_status6(frame: bytes) -> dict[str, float] | None:
    """Status 6: raw duty cycle (16-bit), period in microseconds (16-bit), no-signal flag."""
    if len(frame) < 5:
        return None
    duty_raw = struct.unpack_from("<H", frame, 0)[0]
    period_us = struct.unpack_from("<H", frame, 2)[0]
    no_signal = frame[4] & 0x01
    return {
        "DutyCycleRaw": duty_raw * 1.541161211566339e-5,  # DBC factor, 0-1
        "DutyCyclePeriod": float(period_us),  # microseconds
        "DutyCycleNoSignal": float(no_signal),
    }


def _decode_status7(frame: bytes) -> dict[str, float] | None:
    """Status 7: I-accumulation (float32)."""
    if len(frame) < 4:
        return None
    i_accum = struct.unpack_from("<f", frame, 0)[0]
    return {"IAccumulation": i_accum}


def _decode_firmware(frame: bytes) -> str | None:
    """Firmware version from 6-byte CAN frame: [major, minor, build_lo, build_hi, debug, hw_rev]."""
    if len(frame) < 6:
        return None
    major    = frame[0]
    minor    = frame[1]
    build    = frame[2] | (frame[3] << 8)
    return f"{major}.{minor}.{build}"


def parse_revlog(filepath: str) -> tuple[dict[str, list[tuple[float, Any]]], dict[str, str]]:
    """
    Parse a .revlog file.

    Returns:
        channels: dict mapping channel name -> list of (timestamp_sec, value)
        types:    dict mapping channel name -> type string
    """
    with open(filepath, "rb") as f:
        data = f.read()

    channels: dict[str, list[tuple[float, Any]]] = {}
    types: dict[str, str] = {}
    firmware_seen: set[int] = set()  # track which devices already have firmware recorded

    def _ensure(name: str, type_str: str):
        if name not in channels:
            channels[name] = []
            types[name] = type_str

    pos = 0
    n = len(data)

    while pos < n:
        if pos + 1 > n:
            break
        bitfield = data[pos]
        pos += 1

        entry_id_size  = (bitfield & 0x03) + 1
        payload_size_size = ((bitfield >> 2) & 0x03) + 1

        if pos + entry_id_size + payload_size_size > n:
            break

        entry_id     = int.from_bytes(data[pos: pos + entry_id_size],     "little")
        pos += entry_id_size
        payload_size = int.from_bytes(data[pos: pos + payload_size_size], "little")
        pos += payload_size_size

        if pos + payload_size > n:
            break
        payload = data[pos: pos + payload_size]
        pos += payload_size

        if entry_id == 1:
            # Firmware record: N × 10-byte chunks
            for i in range(0, len(payload) - 9, 10):
                msg_id = struct.unpack_from("<I", payload, i)[0]
                frame  = payload[i + 4: i + 10]
                device_type, manufacturer, api_class, api_index, device_num = _decode_can_id(msg_id)
                if device_type != _SPARK_DEVICE_TYPE or manufacturer != _REV_MANUFACTURER:
                    continue
                if device_num in firmware_seen:
                    continue
                version = _decode_firmware(frame)
                if version:
                    firmware_seen.add(device_num)
                    ch = f"/REV/Firmware/{device_num}"
                    _ensure(ch, "string")
                    channels[ch].append((0.0, version))

        elif entry_id == 2:
            # Periodic record: N × 16-byte chunks
            for i in range(0, len(payload) - 15, 16):
                ts_ms  = struct.unpack_from("<I", payload, i)[0]
                msg_id = struct.unpack_from("<I", payload, i + 4)[0]
                frame  = payload[i + 8: i + 16]

                device_type, manufacturer, api_class, api_index, device_num = _decode_can_id(msg_id)
                if device_type != _SPARK_DEVICE_TYPE or manufacturer != _REV_MANUFACTURER:
                    continue
                if api_class != _SPARK_STATUS_API_CLASS:
                    continue

                ts = ts_ms / 1000.0
                prefix = f"/REV/SPARK/{device_num}"

                if api_index == _STATUS_0:
                    decoded = _decode_status0(frame)
                    if decoded:
                        for signal, value in decoded.items():
                            ch = f"{prefix}/{signal}"
                            _ensure(ch, "float")
                            channels[ch].append((ts, value))

                elif api_index == _STATUS_1:
                    decoded = _decode_status1(frame)
                    if decoded:
                        for signal, value in decoded.items():
                            ch = f"{prefix}/{signal}"
                            _ensure(ch, "int64")
                            channels[ch].append((ts, value))

                elif api_index == _STATUS_2:
                    decoded = _decode_status2(frame)
                    if decoded:
                        for signal, value in decoded.items():
                            ch = f"{prefix}/{signal}"
                            _ensure(ch, "float")
                            channels[ch].append((ts, value))

                elif api_index in (_STATUS_3, _STATUS_4, _STATUS_5,
                                   _STATUS_6, _STATUS_7):
                    decoder = {
                        _STATUS_3: _decode_status3,
                        _STATUS_4: _decode_status4,
                        _STATUS_5: _decode_status5,
                        _STATUS_6: _decode_status6,
                        _STATUS_7: _decode_status7,
                    }[api_index]
                    decoded = decoder(frame)
                    if decoded:
                        for signal, value in decoded.items():
                            ch = f"{prefix}/{signal}"
                            _ensure(ch, "float")
                            channels[ch].append((ts, value))

    return channels, types


def get_spark_device_ids(channels: dict) -> list[int]:
    """Return sorted list of SPARK device numbers seen in the channels dict."""
    ids = set()
    for name in channels:
        if name.startswith("/REV/SPARK/"):
            parts = name.split("/")
            if len(parts) >= 4:
                try:
                    ids.add(int(parts[3]))
                except ValueError:
                    pass
    return sorted(ids)
