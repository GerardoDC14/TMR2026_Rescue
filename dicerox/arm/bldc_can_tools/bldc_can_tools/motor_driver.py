from __future__ import annotations

import time
from dataclasses import dataclass

from . import lktech_protocol as protocol
from .ginkgo_can_interface import (
    BaseCANInterface,
    CANFrame,
    GinkgoAdapterConfig,
    GinkgoCANInterface,
)
from .utils import joint_to_motor_deg, motor_to_joint_deg


class MotorDriverError(RuntimeError):
    """Base error for high-level LKTech motor operations."""


class MotorReplyTimeout(MotorDriverError):
    """Raised when the motor does not answer within the configured timeout."""


@dataclass(frozen=True)
class MotorCommandAck:
    requested_motor_deg: float
    requested_joint_deg: float | None
    speed_limit_motor_dps: float
    reply: protocol.MotorStatus2


class LKTechMotorDriver:
    """High-level LKTech driver with software boot-offset zero handling."""

    def __init__(
        self,
        can_interface: BaseCANInterface | None = None,
        *,
        adapter_config: GinkgoAdapterConfig | None = None,
        motor_id: int = 15,
        reduction_ratio: float = 1.0,
        use_boot_offset_as_zero: bool = True,
        extended_id: bool = False,
        response_timeout_s: float = 0.25,
    ) -> None:
        if reduction_ratio == 0.0:
            raise ValueError("reduction_ratio must not be zero.")

        self.can = can_interface or GinkgoCANInterface(adapter_config)
        self.motor_id = motor_id
        self.reduction_ratio = reduction_ratio
        self.use_boot_offset_as_zero = use_boot_offset_as_zero
        self.extended_id = extended_id
        self.response_timeout_s = response_timeout_s
        self.boot_offset_motor_deg: float | None = None

    def connect(self, capture_boot_offset: bool | None = None) -> None:
        if not self.can.is_open:
            self.can.open()

        should_capture = (
            self.use_boot_offset_as_zero
            if capture_boot_offset is None
            else capture_boot_offset
        )

        self.can.drain()
        if should_capture:
            self.capture_boot_offset()

    def disconnect(self) -> None:
        self.can.close()

    def read_multi_loop_angle_motor_deg(self) -> float:
        command = protocol.build_read_multiloop_request(motor_id=self.motor_id)
        reply = self._request_reply(command)
        return protocol.parse_multi_loop_angle_reply(reply.data)

    def capture_boot_offset(self) -> float:
        self.boot_offset_motor_deg = self.read_multi_loop_angle_motor_deg()
        return self.boot_offset_motor_deg

    def get_joint_angle_deg(self) -> float:
        current_motor_deg = self.read_multi_loop_angle_motor_deg()
        boot_offset = self._get_effective_boot_offset()
        return motor_to_joint_deg(
            current_motor_deg=current_motor_deg,
            boot_offset_motor_deg=boot_offset,
            reduction_ratio=self.reduction_ratio,
        )

    def joint_target_to_motor_deg(self, desired_joint_deg: float) -> float:
        boot_offset = self._get_effective_boot_offset()
        return joint_to_motor_deg(
            desired_joint_deg=desired_joint_deg,
            boot_offset_motor_deg=boot_offset,
            reduction_ratio=self.reduction_ratio,
        )

    def joint_speed_to_motor_speed_dps(self, speed_dps_joint: float) -> float:
        return abs(speed_dps_joint * self.reduction_ratio)

    def command_joint_angle_deg(
        self, desired_joint_deg: float, speed_dps_joint: float
    ) -> MotorCommandAck:
        motor_target_deg = self.joint_target_to_motor_deg(desired_joint_deg)
        motor_speed_dps = self.joint_speed_to_motor_speed_dps(speed_dps_joint)
        ack = self.command_motor_angle_deg(motor_target_deg, motor_speed_dps)
        return MotorCommandAck(
            requested_motor_deg=motor_target_deg,
            requested_joint_deg=desired_joint_deg,
            speed_limit_motor_dps=motor_speed_dps,
            reply=ack.reply,
        )

    def command_motor_angle_deg(
        self, desired_motor_deg: float, speed_dps_motor: float
    ) -> MotorCommandAck:
        command = protocol.build_multiloop_control2_request(
            motor_id=self.motor_id,
            target_motor_deg=desired_motor_deg,
            speed_dps_motor=speed_dps_motor,
        )
        reply = self._request_reply(command)
        parsed = protocol.parse_multiloop_control2_reply(reply.data)
        return MotorCommandAck(
            requested_motor_deg=desired_motor_deg,
            requested_joint_deg=None,
            speed_limit_motor_dps=abs(speed_dps_motor),
            reply=parsed,
        )

    def _request_reply(
        self,
        command: protocol.LKTechCommand,
        timeout_s: float | None = None,
    ) -> CANFrame:
        effective_timeout_s = self.response_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + effective_timeout_s

        self.can.drain()
        self.can.send(
            arbitration_id=command.arbitration_id,
            data=command.data,
            extended_id=self.extended_id,
        )

        last_unmatched: CANFrame | None = None
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            frame = self.can.recv(timeout=min(0.05, remaining))
            if frame is None:
                continue
            if frame.remote_frame or not frame.data:
                continue
            if (
                command.expected_reply_id is not None
                and frame.arbitration_id != command.expected_reply_id
            ):
                last_unmatched = frame
                continue
            if frame.data[0] != command.command:
                last_unmatched = frame
                continue
            return frame

        message = (
            f"Timed out waiting for reply to command 0x{command.command:02X} "
            f"from motor_id={self.motor_id} on reply ID 0x{command.expected_reply_id:03X}."
        )
        if last_unmatched is not None:
            message += (
                f" Last unmatched frame was ID 0x{last_unmatched.arbitration_id:03X} "
                f"data={last_unmatched.data.hex(' ')}."
            )
        raise MotorReplyTimeout(message)

    def _get_effective_boot_offset(self) -> float:
        if not self.use_boot_offset_as_zero:
            return 0.0
        if self.boot_offset_motor_deg is None:
            self.capture_boot_offset()
        assert self.boot_offset_motor_deg is not None
        return self.boot_offset_motor_deg
