from __future__ import annotations

from dataclasses import dataclass


# These command IDs and payload layouts match a MyActuator/LingKong style command
# family that appears to overlap LKTech documentation and observed traffic examples.
# Treat them as strong working assumptions that still need to be verified on your
# exact motor and firmware.
CMD_READ_MULTI_LOOP_ANGLE = 0x92
CMD_READ_SINGLE_LOOP_ANGLE = 0x94
CMD_READ_STATE_1 = 0x9A
CMD_READ_STATE_2 = 0x9C
CMD_READ_STATE_3 = 0x9D
CMD_MOTOR_OFF = 0x80
CMD_MOTOR_STOP = 0x81
CMD_MOTOR_ON = 0x88
CMD_MULTI_LOOP_ANGLE_CONTROL_2 = 0xA4
CMD_SINGLE_LOOP_ANGLE_CONTROL_2 = 0xA6


@dataclass(frozen=True)
class ArbitrationIdPolicy:
    request_base: int = 0x140
    reply_base: int | None = None
    broadcast_request_ids: tuple[int, ...] = (0x280, 0x281, 0x282, 0x288)


DEFAULT_ARB_POLICY = ArbitrationIdPolicy()


@dataclass(frozen=True)
class LKTechCommand:
    arbitration_id: int
    data: bytes
    expected_reply_id: int | None
    command: int


@dataclass(frozen=True)
class MotorStatus1:
    temperature_c: int
    mos_temperature_c: int
    brake_released: bool
    voltage_v: float
    error_flags: int


@dataclass(frozen=True)
class MotorStatus2:
    temperature_c: int
    torque_current_a: float
    speed_dps: float
    angle_deg: float


@dataclass(frozen=True)
class MotorStatus3:
    temperature_c: int
    phase_a_current_a: float
    phase_b_current_a: float
    phase_c_current_a: float


@dataclass(frozen=True)
class SingleLoopControlReply:
    temperature_c: int
    torque_current_a: float
    speed_dps: float
    encoder_counts: int


def degrees_to_centidegrees_int(angle_deg: float) -> int:
    return int(round(angle_deg * 100.0))


def centidegrees_to_degrees(angle_cdeg: int) -> float:
    return angle_cdeg / 100.0


def dps_to_centidps_int(speed_dps: float) -> int:
    return int(round(speed_dps * 100.0))


def centidps_to_dps(speed_cdps: int) -> float:
    return speed_cdps / 100.0


def pack_u16_le(value: int) -> bytes:
    _ensure_int_range(value, 0, 0xFFFF, "u16")
    return int(value).to_bytes(2, byteorder="little", signed=False)


def pack_u32_le(value: int) -> bytes:
    _ensure_int_range(value, 0, 0xFFFFFFFF, "u32")
    return int(value).to_bytes(4, byteorder="little", signed=False)


def pack_i32_le(value: int) -> bytes:
    _ensure_int_range(value, -0x80000000, 0x7FFFFFFF, "i32")
    return int(value).to_bytes(4, byteorder="little", signed=True)


def unpack_u16_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=False)


def unpack_i16_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=True)


def unpack_u32_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="little", signed=False)


def unpack_i32_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="little", signed=True)


def unpack_i56_le(data: bytes, offset: int) -> int:
    raw = int.from_bytes(data[offset : offset + 7], byteorder="little", signed=False)
    if raw & (1 << 55):
        raw -= 1 << 56
    return raw


def build_request_arbitration_id(
    motor_id: int, policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY
) -> int:
    _validate_motor_id(motor_id)
    return policy.request_base + motor_id


def build_reply_arbitration_id(
    motor_id: int, policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY
) -> int:
    _validate_motor_id(motor_id)
    if policy.reply_base is None or policy.reply_base == policy.request_base:
        return policy.request_base + motor_id
    return policy.reply_base + motor_id


def extract_motor_id_from_arbitration_id(
    arbitration_id: int, policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY
) -> int | None:
    if policy.request_base < arbitration_id <= policy.request_base + 0xFF:
        return arbitration_id - policy.request_base
    if (
        policy.reply_base is not None
        and policy.reply_base != policy.request_base
        and policy.reply_base < arbitration_id <= policy.reply_base + 0xFF
    ):
        return arbitration_id - policy.reply_base
    return None


def classify_arbitration_id(
    arbitration_id: int, policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY
) -> tuple[str | None, int | None]:
    if arbitration_id in policy.broadcast_request_ids:
        return "broadcast_request", None
    if policy.request_base < arbitration_id <= policy.request_base + 0xFF:
        if policy.reply_base is None or policy.reply_base == policy.request_base:
            return "single_motor", arbitration_id - policy.request_base
        return "request", arbitration_id - policy.request_base
    if (
        policy.reply_base is not None
        and policy.reply_base != policy.request_base
        and policy.reply_base < arbitration_id <= policy.reply_base + 0xFF
    ):
        return "reply", arbitration_id - policy.reply_base
    return None, None


def build_read_multiloop_request(
    motor_id: int,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    return _build_simple_request(
        command=CMD_READ_MULTI_LOOP_ANGLE,
        motor_id=motor_id,
        arbitration_id=arbitration_id,
        policy=policy,
    )


def build_read_singleloop_request(
    motor_id: int,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    return _build_simple_request(
        command=CMD_READ_SINGLE_LOOP_ANGLE,
        motor_id=motor_id,
        arbitration_id=arbitration_id,
        policy=policy,
    )


def build_read_state1_request(
    motor_id: int,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    return _build_simple_request(
        command=CMD_READ_STATE_1,
        motor_id=motor_id,
        arbitration_id=arbitration_id,
        policy=policy,
    )


def build_read_state2_request(
    motor_id: int,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    return _build_simple_request(
        command=CMD_READ_STATE_2,
        motor_id=motor_id,
        arbitration_id=arbitration_id,
        policy=policy,
    )


def build_read_state3_request(
    motor_id: int,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    return _build_simple_request(
        command=CMD_READ_STATE_3,
        motor_id=motor_id,
        arbitration_id=arbitration_id,
        policy=policy,
    )


def build_multiloop_control2_request(
    motor_id: int,
    target_motor_deg: float,
    speed_dps_motor: float,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    # The 0xA4 layout is treated here as LKTech "Multi Loop Angle Control 2".
    max_speed_dps = int(round(abs(speed_dps_motor)))
    target_cdeg = degrees_to_centidegrees_int(target_motor_deg)
    payload = bytes([CMD_MULTI_LOOP_ANGLE_CONTROL_2, 0x00])
    payload += pack_u16_le(max_speed_dps)
    payload += pack_i32_le(target_cdeg)
    return LKTechCommand(
        arbitration_id=arbitration_id or build_request_arbitration_id(motor_id, policy),
        data=payload,
        expected_reply_id=build_reply_arbitration_id(motor_id, policy),
        command=CMD_MULTI_LOOP_ANGLE_CONTROL_2,
    )


def build_singleloop_control2_request(
    motor_id: int,
    target_single_turn_deg: float,
    speed_dps_motor: float,
    *,
    direction: int | None = None,
    arbitration_id: int | None = None,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> LKTechCommand:
    # The 0xA6 helper is included as a scaffold because some manuals encode
    # direction separately and magnitude as a single-turn 0.01 degree value.
    max_speed_dps = int(round(abs(speed_dps_motor)))
    if direction is None:
        direction = 1 if target_single_turn_deg < 0.0 else 0
    if direction not in (0, 1):
        raise ValueError("direction must be 0 (CW) or 1 (CCW).")

    angle_magnitude_cdeg = abs(degrees_to_centidegrees_int(target_single_turn_deg))
    _ensure_int_range(angle_magnitude_cdeg, 0, 36000, "single-turn angle")

    payload = bytes([CMD_SINGLE_LOOP_ANGLE_CONTROL_2, direction])
    payload += pack_u16_le(max_speed_dps)
    payload += pack_u32_le(angle_magnitude_cdeg)
    return LKTechCommand(
        arbitration_id=arbitration_id or build_request_arbitration_id(motor_id, policy),
        data=payload,
        expected_reply_id=build_reply_arbitration_id(motor_id, policy),
        command=CMD_SINGLE_LOOP_ANGLE_CONTROL_2,
    )


def parse_multi_loop_angle_reply(data: bytes) -> float:
    _require_command(data, CMD_READ_MULTI_LOOP_ANGLE)
    # V2.36 LKTech CAN documentation describes the multi-loop angle reply as the
    # command byte followed by a signed 56-bit little-endian centi-degree value.
    angle_cdeg = unpack_i56_le(data, 1)
    return centidegrees_to_degrees(angle_cdeg)


def parse_single_loop_angle_reply(data: bytes) -> float:
    _require_command(data, CMD_READ_SINGLE_LOOP_ANGLE)
    # The current working assumption from the V2.36 CAN examples is that the
    # single-loop angle is stored as a 32-bit little-endian centi-degree value
    # in DATA[4:8]. This still needs hardware verification on the exact motor.
    angle_cdeg = unpack_u32_le(data, 4)
    return centidegrees_to_degrees(angle_cdeg)


def parse_state1_reply(data: bytes) -> MotorStatus1:
    _require_command(data, CMD_READ_STATE_1)
    return MotorStatus1(
        temperature_c=int(data[1]),
        mos_temperature_c=int(data[2]),
        brake_released=bool(data[3]),
        voltage_v=unpack_u16_le(data, 4) / 10.0,
        error_flags=unpack_u16_le(data, 6),
    )


def parse_state2_reply(data: bytes) -> MotorStatus2:
    return _parse_status2_like_reply(data, expected_command=CMD_READ_STATE_2)


def parse_state3_reply(data: bytes) -> MotorStatus3:
    _require_command(data, CMD_READ_STATE_3)
    return MotorStatus3(
        temperature_c=int(data[1]),
        phase_a_current_a=unpack_i16_le(data, 2) / 100.0,
        phase_b_current_a=unpack_i16_le(data, 4) / 100.0,
        phase_c_current_a=unpack_i16_le(data, 6) / 100.0,
    )


def parse_multiloop_control2_reply(data: bytes) -> MotorStatus2:
    return _parse_status2_like_reply(
        data,
        expected_command=CMD_MULTI_LOOP_ANGLE_CONTROL_2,
    )


def parse_singleloop_control2_reply(data: bytes) -> SingleLoopControlReply:
    _require_command(data, CMD_SINGLE_LOOP_ANGLE_CONTROL_2)
    return SingleLoopControlReply(
        temperature_c=int(data[1]),
        torque_current_a=unpack_i16_le(data, 2) / 100.0,
        speed_dps=float(unpack_i16_le(data, 4)),
        encoder_counts=unpack_u16_le(data, 6),
    )


def decode_lktech_frame(
    arbitration_id: int,
    data: bytes,
    policy: ArbitrationIdPolicy = DEFAULT_ARB_POLICY,
) -> str | None:
    if not data:
        return None

    direction, motor_id = classify_arbitration_id(arbitration_id, policy)
    command = data[0]

    try:
        if direction in ("request", "broadcast_request"):
            return _decode_request_summary(command, data, motor_id)
        if direction == "single_motor":
            return _decode_single_motor_summary(command, data, motor_id)
        if direction == "reply":
            return _decode_reply_summary(command, data, motor_id)
    except Exception as exc:
        label = "reply" if direction == "reply" else "request"
        return f"{label} cmd=0x{command:02X} parse_error={exc}"

    return None


def _build_simple_request(
    *,
    command: int,
    motor_id: int,
    arbitration_id: int | None,
    policy: ArbitrationIdPolicy,
) -> LKTechCommand:
    return LKTechCommand(
        arbitration_id=arbitration_id or build_request_arbitration_id(motor_id, policy),
        data=bytes([command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        expected_reply_id=build_reply_arbitration_id(motor_id, policy),
        command=command,
    )


def _parse_status2_like_reply(data: bytes, expected_command: int) -> MotorStatus2:
    _require_command(data, expected_command)
    return MotorStatus2(
        temperature_c=int(data[1]),
        torque_current_a=unpack_i16_le(data, 2) / 100.0,
        speed_dps=float(unpack_i16_le(data, 4)),
        angle_deg=float(unpack_i16_le(data, 6)),
    )


def _decode_request_summary(command: int, data: bytes, motor_id: int | None) -> str | None:
    prefix = f"request motor_id={motor_id}" if motor_id is not None else "request broadcast"
    if command == CMD_READ_MULTI_LOOP_ANGLE:
        return f"{prefix} read_multi_loop_angle"
    if command == CMD_READ_SINGLE_LOOP_ANGLE:
        return f"{prefix} read_single_loop_angle"
    if command == CMD_READ_STATE_1:
        return f"{prefix} read_state_1"
    if command == CMD_READ_STATE_2:
        return f"{prefix} read_state_2"
    if command == CMD_READ_STATE_3:
        return f"{prefix} read_state_3"
    if command == CMD_MULTI_LOOP_ANGLE_CONTROL_2:
        speed_dps = unpack_u16_le(data, 2)
        target_deg = centidegrees_to_degrees(unpack_i32_le(data, 4))
        return (
            f"{prefix} multiloop_control2 "
            f"speed_limit_dps={speed_dps} target_motor_deg={target_deg:.2f}"
        )
    if command == CMD_SINGLE_LOOP_ANGLE_CONTROL_2:
        direction_label = "ccw" if data[1] else "cw"
        speed_dps = unpack_u16_le(data, 2)
        target_deg = centidegrees_to_degrees(
            int.from_bytes(data[4:8], byteorder="little", signed=False)
        )
        return (
            f"{prefix} singleloop_control2 "
            f"direction={direction_label} speed_limit_dps={speed_dps} "
            f"target_single_turn_deg={target_deg:.2f}"
        )
    return f"{prefix} unknown_command=0x{command:02X}"


def _decode_single_motor_summary(
    command: int,
    data: bytes,
    motor_id: int | None,
) -> str | None:
    prefix = f"single_motor motor_id={motor_id}" if motor_id is not None else "single_motor"

    if _looks_like_read_request(command, data):
        request_summary = _decode_request_summary(command, data, motor_id)
        return request_summary.replace("request", prefix, 1) if request_summary else prefix

    if command in (CMD_MOTOR_OFF, CMD_MOTOR_ON, CMD_MOTOR_STOP):
        return f"{prefix} cmd=0x{command:02X} request_or_echo"

    if command in (CMD_MULTI_LOOP_ANGLE_CONTROL_2, CMD_SINGLE_LOOP_ANGLE_CONTROL_2) and data[1] == 0x00:
        request_summary = _decode_request_summary(command, data, motor_id)
        return request_summary.replace("request", prefix, 1) if request_summary else prefix

    reply_summary = _decode_reply_summary(command, data, motor_id)
    return reply_summary.replace("reply", prefix, 1) if reply_summary else prefix


def _decode_reply_summary(command: int, data: bytes, motor_id: int | None) -> str | None:
    prefix = f"reply motor_id={motor_id}" if motor_id is not None else "reply"
    if command == CMD_READ_MULTI_LOOP_ANGLE:
        angle_deg = parse_multi_loop_angle_reply(data)
        return f"{prefix} multi_loop_angle_deg={angle_deg:.2f}"
    if command == CMD_READ_SINGLE_LOOP_ANGLE:
        angle_deg = parse_single_loop_angle_reply(data)
        return f"{prefix} single_loop_angle_deg={angle_deg:.2f}"
    if command == CMD_READ_STATE_1:
        state = parse_state1_reply(data)
        return (
            f"{prefix} state1 temp_c={state.temperature_c} mos_temp_c={state.mos_temperature_c} "
            f"voltage_v={state.voltage_v:.1f} error_flags=0x{state.error_flags:04X}"
        )
    if command == CMD_READ_STATE_2:
        state = parse_state2_reply(data)
        return (
            f"{prefix} state2 temp_c={state.temperature_c} iq_a={state.torque_current_a:.2f} "
            f"speed_dps={state.speed_dps:.1f} angle_deg={state.angle_deg:.1f}"
        )
    if command == CMD_READ_STATE_3:
        state = parse_state3_reply(data)
        return (
            f"{prefix} state3 temp_c={state.temperature_c} "
            f"ia_a={state.phase_a_current_a:.2f} ib_a={state.phase_b_current_a:.2f} "
            f"ic_a={state.phase_c_current_a:.2f}"
        )
    if command == CMD_MULTI_LOOP_ANGLE_CONTROL_2:
        state = parse_multiloop_control2_reply(data)
        return (
            f"{prefix} multiloop_control2_ack temp_c={state.temperature_c} "
            f"iq_a={state.torque_current_a:.2f} speed_dps={state.speed_dps:.1f} "
            f"angle_deg={state.angle_deg:.1f}"
        )
    if command == CMD_SINGLE_LOOP_ANGLE_CONTROL_2:
        state = parse_singleloop_control2_reply(data)
        return (
            f"{prefix} singleloop_control2_ack temp_c={state.temperature_c} "
            f"iq_a={state.torque_current_a:.2f} speed_dps={state.speed_dps:.1f} "
            f"encoder_counts={state.encoder_counts}"
        )
    return f"{prefix} unknown_command=0x{command:02X}"


def _looks_like_read_request(command: int, data: bytes) -> bool:
    if command not in (
        CMD_READ_MULTI_LOOP_ANGLE,
        CMD_READ_SINGLE_LOOP_ANGLE,
        CMD_READ_STATE_1,
        CMD_READ_STATE_2,
        CMD_READ_STATE_3,
    ):
        return False
    return all(byte == 0x00 for byte in data[1:])


def _require_command(data: bytes, expected_command: int) -> None:
    if len(data) != 8:
        raise ValueError(f"Expected 8 data bytes, got {len(data)}")
    if data[0] != expected_command:
        raise ValueError(
            f"Expected command 0x{expected_command:02X}, got 0x{data[0]:02X}"
        )


def _ensure_int_range(value: int, lower: int, upper: int, label: str) -> None:
    if not lower <= value <= upper:
        raise ValueError(f"{label} out of range: {value}")


def _validate_motor_id(motor_id: int) -> None:
    if not 1 <= motor_id <= 0xFF:
        raise ValueError("motor_id must be in the range [1, 255].")
