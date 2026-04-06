from __future__ import annotations

from datetime import datetime, timezone


def hex_bytes(data: bytes) -> str:
    """Return a compact hex string for a CAN payload."""
    return " ".join(f"{value:02X}" for value in data)


def format_timestamp(timestamp_s: float | None) -> str:
    """Format a wall-clock style timestamp for sniffer output."""
    if timestamp_s is None:
        return "--:--:--.---"
    dt = datetime.fromtimestamp(timestamp_s, tz=timezone.utc).astimezone()
    return dt.strftime("%H:%M:%S.%f")[:-3]


def format_can_identifier(arbitration_id: int, extended_id: bool = False) -> str:
    width = 8 if extended_id else 3
    return f"0x{arbitration_id:0{width}X}"


def format_can_frame(
    arbitration_id: int,
    data: bytes,
    *,
    extended_id: bool = False,
    remote_frame: bool = False,
    timestamp_s: float | None = None,
) -> str:
    frame_id = format_can_identifier(arbitration_id, extended_id)
    if remote_frame:
        return f"{format_timestamp(timestamp_s)}  {frame_id}  RTR"
    return (
        f"{format_timestamp(timestamp_s)}  {frame_id}  "
        f"[{len(data)}]  {hex_bytes(data)}"
    )


def motor_to_joint_deg(
    current_motor_deg: float,
    boot_offset_motor_deg: float,
    reduction_ratio: float,
) -> float:
    if reduction_ratio == 0.0:
        raise ValueError("reduction_ratio must not be zero.")
    return (current_motor_deg - boot_offset_motor_deg) / reduction_ratio


def joint_to_motor_deg(
    desired_joint_deg: float,
    boot_offset_motor_deg: float,
    reduction_ratio: float,
) -> float:
    if reduction_ratio == 0.0:
        raise ValueError("reduction_ratio must not be zero.")
    return boot_offset_motor_deg + desired_joint_deg * reduction_ratio
