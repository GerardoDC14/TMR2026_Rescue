from __future__ import annotations

from pathlib import Path
import sys


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
except ImportError as exc:  # pragma: no cover - optional in direct Windows usage
    rclpy = None
    Node = object  # type: ignore[assignment]
    Float64 = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


if __package__ in (None, ""):
    from bldc_can_tools.ginkgo_can_interface import GinkgoAdapterConfig
    from bldc_can_tools.motor_driver import LKTechMotorDriver
else:
    from .ginkgo_can_interface import GinkgoAdapterConfig
    from .motor_driver import LKTechMotorDriver


class LKTechPositionTestNode(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")

        super().__init__("lktech_position_test")

        self.declare_parameter("motor_id", 15)
        self.declare_parameter("reduction_ratio", 1.0)
        self.declare_parameter("use_boot_offset_as_zero", True)
        self.declare_parameter("default_speed_dps_joint", 30.0)
        self.declare_parameter("default_speed_dps_motor", 30.0)
        self.declare_parameter("channel", 0)
        self.declare_parameter("bitrate", 1000)
        self.declare_parameter("extended_id", False)
        self.declare_parameter("device_index", 0)

        self.motor_id = int(self.get_parameter("motor_id").value)
        self.reduction_ratio = float(self.get_parameter("reduction_ratio").value)
        self.use_boot_offset_as_zero = bool(
            self.get_parameter("use_boot_offset_as_zero").value
        )
        self.default_speed_dps_joint = float(
            self.get_parameter("default_speed_dps_joint").value
        )
        self.default_speed_dps_motor = float(
            self.get_parameter("default_speed_dps_motor").value
        )

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
        self.driver.connect(capture_boot_offset=self.use_boot_offset_as_zero)

        self.create_subscription(
            Float64,
            "/lktech/target_joint_deg",
            self._on_joint_target,
            10,
        )
        self.create_subscription(
            Float64,
            "/lktech/target_motor_deg",
            self._on_motor_target,
            10,
        )

        self.get_logger().info(
            "Ready for /lktech/target_joint_deg and /lktech/target_motor_deg "
            f"with motor_id={self.motor_id}, reduction_ratio={self.reduction_ratio:.4f}, "
            f"boot_offset_motor_deg={self.driver.boot_offset_motor_deg}"
        )

    def _on_joint_target(self, msg: Float64) -> None:
        try:
            ack = self.driver.command_joint_angle_deg(
                desired_joint_deg=msg.data,
                speed_dps_joint=self.default_speed_dps_joint,
            )
            self.get_logger().info(
                "Joint target received: joint_deg=%.2f, motor_target_deg=%.2f, speed_limit_motor_dps=%.2f"
                % (
                    msg.data,
                    ack.requested_motor_deg,
                    ack.speed_limit_motor_dps,
                )
            )
        except Exception as exc:
            self.get_logger().error(f"Joint target command failed: {exc}")

    def _on_motor_target(self, msg: Float64) -> None:
        try:
            ack = self.driver.command_motor_angle_deg(
                desired_motor_deg=msg.data,
                speed_dps_motor=self.default_speed_dps_motor,
            )
            self.get_logger().info(
                "Motor target received: motor_deg=%.2f, speed_limit_motor_dps=%.2f"
                % (
                    ack.requested_motor_deg,
                    ack.speed_limit_motor_dps,
                )
            )
        except Exception as exc:
            self.get_logger().error(f"Motor target command failed: {exc}")

    def destroy_node(self) -> bool:
        self.driver.disconnect()
        return super().destroy_node()


def main(argv: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")

    rclpy.init(args=argv)
    node = LKTechPositionTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

