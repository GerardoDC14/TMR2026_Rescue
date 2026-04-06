from __future__ import annotations

from dataclasses import dataclass


ZE300_DIRECT_REPLY_MASK = 0x7FF
ZE300_REQUEST_TAG_MASK = 0x100
ZE300_BROADCAST_ADDRESS = 0x00
ZE300_COMMON_ADDRESS = 0xFF
ZE300_DEFAULT_BITRATE_KBPS = 1000
ZE300_COUNTS_PER_REV = 16384

CMD_READ_VERSIONS = 0xA0
CMD_READ_Q_CURRENT = 0xA1
CMD_READ_SPEED = 0xA2
CMD_READ_ABSOLUTE_ANGLES = 0xA3
CMD_READ_REALTIME_STATE = 0xA4
CMD_READ_FAULT_STATE = 0xAE


class ZE300ProtocolError(ValueError):
    """Raised when a ZE300 CAN frame does not match the documented layout."""


@dataclass(frozen=True)
class ZE300Request:
    arbitration_id: int
    data: bytes
    expected_reply_id: int | None
    command: int


@dataclass(frozen=True)
class ZE300VersionInfo:
    boot_version_raw: int
    app_version_raw: int
    hardware_version_raw: int
    can_protocol_version_raw: int


@dataclass(frozen=True)
class ZE300AngleState:
    single_turn_counts: int
    multi_turn_counts: int
    single_turn_deg: float
    multi_turn_deg: float


@dataclass(frozen=True)
class ZE300RealtimeState:
    temperature_c: int
    q_axis_current_a: float
    speed_rpm: float
    single_turn_counts: int
    single_turn_deg: float


@dataclass(frozen=True)
class ZE300FaultState:
    bus_voltage_v: float
    bus_current_a: float
    temperature_c: int
    run_mode: int
    run_mode_label: str
    fault_code: int
    fault_labels: tuple[str, ...]


def validate_device_address(device_address: int) -> None:
    if not 0 <= device_address <= 0xFF:
        raise ZE300ProtocolError("device_address must be in the range 0..255.")


def counts_to_deg(counts: int) -> float:
    return counts * (360.0 / ZE300_COUNTS_PER_REV)


def build_request_arbitration_id(device_address: int, *, tagged_request: bool = True) -> int:
    validate_device_address(device_address)
    if tagged_request and device_address not in (ZE300_BROADCAST_ADDRESS, ZE300_COMMON_ADDRESS):
        return ZE300_REQUEST_TAG_MASK | device_address
    return device_address


def build_reply_arbitration_id(device_address: int) -> int | None:
    validate_device_address(device_address)
    if device_address == ZE300_BROADCAST_ADDRESS:
        return None
    return device_address


def build_simple_read_request(
    device_address: int,
    command: int,
    *,
    tagged_request: bool = True,
) -> ZE300Request:
    arbitration_id = build_request_arbitration_id(device_address, tagged_request=tagged_request)
    expected_reply_id = build_reply_arbitration_id(device_address)
    return ZE300Request(
        arbitration_id=arbitration_id,
        data=bytes([command]),
        expected_reply_id=expected_reply_id,
        command=command,
    )


def build_read_versions_request(device_address: int, *, tagged_request: bool = True) -> ZE300Request:
    return build_simple_read_request(device_address, CMD_READ_VERSIONS, tagged_request=tagged_request)


def build_read_angles_request(device_address: int, *, tagged_request: bool = True) -> ZE300Request:
    return build_simple_read_request(device_address, CMD_READ_ABSOLUTE_ANGLES, tagged_request=tagged_request)


def build_read_realtime_request(device_address: int, *, tagged_request: bool = True) -> ZE300Request:
    return build_simple_read_request(device_address, CMD_READ_REALTIME_STATE, tagged_request=tagged_request)


def build_read_fault_state_request(device_address: int, *, tagged_request: bool = True) -> ZE300Request:
    return build_simple_read_request(device_address, CMD_READ_FAULT_STATE, tagged_request=tagged_request)


def _require_command(data: bytes, expected_command: int) -> None:
    if not data:
        raise ZE300ProtocolError("ZE300 reply payload is empty.")
    if data[0] != expected_command:
        raise ZE300ProtocolError(
            f"Expected ZE300 command 0x{expected_command:02X}, got 0x{data[0]:02X}."
        )


def _read_u16_le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def _read_i16_le(data: bytes, offset: int) -> int:
    value = _read_u16_le(data, offset)
    if value & 0x8000:
        value -= 0x10000
    return value


def _read_i32_le(data: bytes, offset: int) -> int:
    value = (
        data[offset]
        | (data[offset + 1] << 8)
        | (data[offset + 2] << 16)
        | (data[offset + 3] << 24)
    )
    if value & 0x80000000:
        value -= 0x100000000
    return value


def parse_versions_reply(data: bytes) -> ZE300VersionInfo:
    _require_command(data, CMD_READ_VERSIONS)
    if len(data) != 8:
        raise ZE300ProtocolError(f"ZE300 0xA0 reply must be 8 bytes, got {len(data)}.")
    return ZE300VersionInfo(
        boot_version_raw=_read_u16_le(data, 1),
        app_version_raw=_read_u16_le(data, 3),
        hardware_version_raw=_read_u16_le(data, 5),
        can_protocol_version_raw=data[7],
    )


def parse_angles_reply(data: bytes) -> ZE300AngleState:
    _require_command(data, CMD_READ_ABSOLUTE_ANGLES)
    if len(data) != 7:
        raise ZE300ProtocolError(f"ZE300 0xA3 reply must be 7 bytes, got {len(data)}.")

    single_turn_counts = _read_u16_le(data, 1)
    multi_turn_counts = _read_i32_le(data, 3)
    return ZE300AngleState(
        single_turn_counts=single_turn_counts,
        multi_turn_counts=multi_turn_counts,
        single_turn_deg=counts_to_deg(single_turn_counts),
        multi_turn_deg=counts_to_deg(multi_turn_counts),
    )


def parse_realtime_reply(data: bytes) -> ZE300RealtimeState:
    _require_command(data, CMD_READ_REALTIME_STATE)
    if len(data) != 8:
        raise ZE300ProtocolError(f"ZE300 0xA4 reply must be 8 bytes, got {len(data)}.")

    single_turn_counts = _read_u16_le(data, 6)
    return ZE300RealtimeState(
        temperature_c=data[1],
        q_axis_current_a=_read_i16_le(data, 2) / 1000.0,
        speed_rpm=_read_i16_le(data, 4) / 100.0,
        single_turn_counts=single_turn_counts,
        single_turn_deg=counts_to_deg(single_turn_counts),
    )


def describe_run_mode(run_mode: int) -> str:
    return {
        0: "off",
        1: "voltage",
        2: "q_current",
        3: "speed",
        4: "position",
    }.get(run_mode, "unknown")


def describe_fault_bits(fault_code: int) -> tuple[str, ...]:
    labels: list[str] = []
    if fault_code & (1 << 0):
        labels.append("voltage_fault")
    if fault_code & (1 << 1):
        labels.append("current_fault")
    if fault_code & (1 << 2):
        labels.append("temperature_fault")
    if fault_code & (1 << 3):
        labels.append("encoder_fault")
    if fault_code & (1 << 6):
        labels.append("hardware_fault")
    if fault_code & (1 << 7):
        labels.append("software_fault")
    return tuple(labels)


def parse_fault_state_reply(data: bytes) -> ZE300FaultState:
    _require_command(data, CMD_READ_FAULT_STATE)
    if len(data) != 8:
        raise ZE300ProtocolError(f"ZE300 0xAE reply must be 8 bytes, got {len(data)}.")

    run_mode = data[6]
    fault_code = data[7]
    return ZE300FaultState(
        bus_voltage_v=_read_u16_le(data, 1) / 100.0,
        bus_current_a=_read_u16_le(data, 3) / 100.0,
        temperature_c=data[5],
        run_mode=run_mode,
        run_mode_label=describe_run_mode(run_mode),
        fault_code=fault_code,
        fault_labels=describe_fault_bits(fault_code),
    )
