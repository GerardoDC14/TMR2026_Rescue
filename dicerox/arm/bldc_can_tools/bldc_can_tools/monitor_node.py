from __future__ import annotations

import math
from pathlib import Path
import sys


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
except ImportError as exc:  # pragma: no cover - optional in direct Windows usage
    rclpy = None
    Node = object  # type: ignore[assignment]
    JointState = None  # type: ignore[assignment]
    Float64 = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


if __package__ in (None, ""):
    from bldc_can_tools.ginkgo_can_interface import GinkgoAdapterConfig
    from bldc_can_tools.motor_driver import LKTechMotorDriver
    from bldc_can_tools.utils import motor_to_joint_deg
else:
    from .ginkgo_can_interface import GinkgoAdapterConfig
    from .motor_driver import LKTechMotorDriver
    from .utils import motor_to_joint_deg


class LKTechMonitorNode(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")

        super().__init__("lktech_monitor")

        self.declare_parameter("motor_id", 15)
        self.declare_parameter("reduction_ratio", 1.0)
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("use_boot_offset_as_zero", True)
        self.declare_parameter("joint_name", "lktech_joint")
        self.declare_parameter("publish_joint_state", True)
        self.declare_parameter("channel", 0)
        self.declare_parameter("bitrate", 1000)
        self.declare_parameter("extended_id", False)
        self.declare_parameter("device_index", 0)

        self.motor_id = int(self.get_parameter("motor_id").value)
        self.reduction_ratio = float(self.get_parameter("reduction_ratio").value)
        self.poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self.use_boot_offset_as_zero = bool(
            self.get_parameter("use_boot_offset_as_zero").value
        )
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.publish_joint_state = bool(self.get_parameter("publish_joint_state").value)

        self.driver = LKTechMotorDriver(
            adapter_config=GinkgoAdapterConfig(
                channel=int(self.get_parameter("channel").value),
                bitrate_kbps=int(self.get_parameter("bitrate").value),
                device_index=int(self.get_parameter("device_index").value),
            ),
            motor_id=self.motor_id,
            reduction_ratio=self.reduction_ratio,
            use_boot_offset_as_zero=self.use_boot_offset_as_zero,
            extended_id=bool(self.get_parameter("extended_id").value),
        )

        self.current_motor_pub = self.create_publisher(
            Float64, "/lktech/current_motor_deg", 10
        )
        self.current_joint_pub = self.create_publisher(
            Float64, "/lktech/current_joint_deg", 10
        )
        self.boot_offset_pub = self.create_publisher(
            Float64, "/lktech/boot_offset_motor_deg", 10
        )
        self.joint_state_pub = (
            self.create_publisher(JointState, "/joint_states", 10)
            if self.publish_joint_state
            else None
        )

        self.driver.connect(capture_boot_offset=self.use_boot_offset_as_zero)
        self.get_logger().info(
            "Connected to LKTech motor_id=%d on channel=%d bitrate=%dkbps using backend=%s"
            % (
                self.motor_id,
                self.driver.can.config.channel,  # type: ignore[attr-defined]
                self.driver.can.config.bitrate_kbps,  # type: ignore[attr-defined]
                getattr(self.driver.can, "backend_source", "unknown"),
            )
        )
        if self.driver.boot_offset_motor_deg is not None:
            self.get_logger().info(
                "Captured software zero boot_offset_motor_deg=%.2f"
                % self.driver.boot_offset_motor_deg
            )

        period_s = 1.0 / self.poll_rate_hz if self.poll_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period_s, self._poll_once)

    def _poll_once(self) -> None:
        try:
            motor_deg = self.driver.read_multi_loop_angle_motor_deg()
            boot_offset = (
                self.driver.boot_offset_motor_deg
                if self.driver.boot_offset_motor_deg is not None
                else 0.0
            )
            joint_deg = motor_to_joint_deg(
                current_motor_deg=motor_deg,
                boot_offset_motor_deg=boot_offset,
                reduction_ratio=self.reduction_ratio,
            )

            motor_msg = Float64()
            motor_msg.data = motor_deg
            self.current_motor_pub.publish(motor_msg)

            joint_msg = Float64()
            joint_msg.data = joint_deg
            self.current_joint_pub.publish(joint_msg)

            offset_msg = Float64()
            offset_msg.data = boot_offset
            self.boot_offset_pub.publish(offset_msg)

            if self.joint_state_pub is not None:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = [self.joint_name]
                joint_state.position = [math.radians(joint_deg)]
                self.joint_state_pub.publish(joint_state)
        except Exception as exc:
            self.get_logger().error(f"Angle poll failed for motor_id={self.motor_id}: {exc}")

    def destroy_node(self) -> bool:
        self.driver.disconnect()
        return super().destroy_node()


def main(argv: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")

    rclpy.init(args=argv)
    node = LKTechMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

