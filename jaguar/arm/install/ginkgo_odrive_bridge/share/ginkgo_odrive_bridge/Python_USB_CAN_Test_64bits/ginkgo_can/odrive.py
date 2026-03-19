from __future__ import annotations

import struct

CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_INPUT_POS = 0x00C
SERVO_COMMAND_ID = 0x200


def make_cob_id(node_id: int, command_id: int) -> int:
    if not 0 <= node_id <= 0x3F:
        raise ValueError("node_id must be in [0, 63].")
    return (node_id << 5) | (command_id & 0x1F)


def encode_set_input_pos(position_turns: float, vel_ff: int = 0, torque_ff: int = 0) -> bytes:
    if not -32768 <= vel_ff <= 32767:
        raise ValueError("vel_ff must be in int16 range.")
    if not -32768 <= torque_ff <= 32767:
        raise ValueError("torque_ff must be in int16 range.")
    return struct.pack("<fhh", float(position_turns), int(vel_ff), int(torque_ff))


def decode_encoder_estimates(payload: bytes) -> tuple[float, float]:
    if len(payload) < 8:
        raise ValueError("Encoder estimate payload must be at least 8 bytes.")
    return struct.unpack("<ff", payload[:8])


def encode_servo_command(channel: int, angle_deg: int) -> bytes:
    if not 0 <= channel <= 15:
        raise ValueError("Servo channel must be in [0, 15].")
    if not 0 <= angle_deg <= 180:
        raise ValueError("Servo angle must be in [0, 180].")
    return bytes([channel & 0xFF, angle_deg & 0xFF, 0, 0, 0, 0, 0, 0])
