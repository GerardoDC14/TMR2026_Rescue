#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState

PACKAGE_NAME = "ginkgo_odrive_bridge"
VENDOR_DIR_NAME = "Python_USB_CAN_Test_64bits"


def _resolve_vendor_dir() -> Path:
    candidates: list[Path] = []

    env_path = os.environ.get("GINKGO_VENDOR_DIR")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    candidates.append(Path(__file__).resolve().parents[1] / VENDOR_DIR_NAME)

    try:
        share_dir = Path(get_package_share_directory(PACKAGE_NAME))
    except PackageNotFoundError:
        share_dir = None

    if share_dir is not None:
        candidates.append(share_dir / VENDOR_DIR_NAME)

    for candidate in candidates:
        if (candidate / "ControlCAN.py").exists() and (candidate / "ginkgo_can").is_dir():
            return candidate

    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        f"Could not find {VENDOR_DIR_NAME}. Checked: {searched or 'no candidates'}"
    )


VENDOR_DIR = _resolve_vendor_dir()
if str(VENDOR_DIR) not in sys.path:
    sys.path.insert(0, str(VENDOR_DIR))

from ginkgo_can import (  # noqa: E402
    BIT_TIMINGS,
    CANBus,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_SET_INPUT_POS,
    decode_encoder_estimates,
    encode_set_input_pos,
    frame_payload,
    make_cob_id,
)

CMD_SET_AXIS_STATE = 0x007
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


@dataclass(frozen=True)
class AxisConfig:
    joint_name: str
    node_id: int
    gear_ratio: float
    direction: float = 1.0
    use_startup_offset: bool = True
    command_mode: str = "direct"

    def relative_turns(self, joint_position_rad: float) -> float:
        turns = (joint_position_rad / (2.0 * math.pi)) * self.gear_ratio
        turns *= self.direction
        if self.command_mode == "direct":
            return turns
        if self.command_mode == "abs":
            return abs(turns)
        if self.command_mode == "neg_abs":
            return -abs(turns)
        raise ValueError(f"Unsupported command_mode={self.command_mode!r}")


class JointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("ginkgo_joint_state_bridge")
        self._declare_parameters()

        self.axes = self._load_axis_configs()
        self.can_channel = int(self.get_parameter("can_channel").value)
        self.can_bitrate_kbps = int(self.get_parameter("can_bitrate_kbps").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_states_qos = str(self.get_parameter("joint_states_qos").value)
        self.send_hz = float(self.get_parameter("send_hz").value)
        self.encoder_timeout_s = float(self.get_parameter("encoder_timeout_s").value)
        self.closed_loop_startup_delay_s = float(
            self.get_parameter("closed_loop_startup_delay_s").value
        )
        self.verbose = bool(self.get_parameter("verbose").value)
        self.verbose_period_s = float(self.get_parameter("verbose_period_s").value)
        self.require_linux_root = bool(self.get_parameter("require_linux_root").value)
        self.require_startup_offsets = bool(
            self.get_parameter("require_startup_offsets").value
        )
        self.set_axis_idle_on_shutdown = bool(
            self.get_parameter("set_axis_idle_on_shutdown").value
        )

        if self.can_bitrate_kbps not in BIT_TIMINGS:
            valid = ", ".join(str(rate) for rate in sorted(BIT_TIMINGS))
            raise ValueError(
                f"Unsupported can_bitrate_kbps={self.can_bitrate_kbps}. Valid values: {valid}"
            )
        if self.send_hz <= 0:
            raise ValueError("send_hz must be > 0.")
        if self.encoder_timeout_s <= 0:
            raise ValueError("encoder_timeout_s must be > 0.")
        if self.verbose_period_s <= 0:
            raise ValueError("verbose_period_s must be > 0.")

        self.bus: CANBus | None = None
        self.joint_index: dict[str, int] = {}
        self.encoder_zero_turns = {axis.node_id: 0.0 for axis in self.axes}
        self.desired_turns = {axis.node_id: 0.0 for axis in self.axes}
        self.last_source_position_rad = {axis.joint_name: 0.0 for axis in self.axes}
        self._last_verbose_log_s = 0.0
        self._waiting_for_joints_logged = False

        try:
            self.bus = CANBus(
                channel=self.can_channel,
                kbps=self.can_bitrate_kbps,
                accept_extended=False,
                require_linux_root=self.require_linux_root,
            )
            self.bus.open()
            self._enter_closed_loop()
            self._capture_encoder_zeros()
        except Exception:
            self.close()
            raise

        self.subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_cb,
            self._build_joint_states_qos(),
        )
        self.send_timer = self.create_timer(1.0 / self.send_hz, self._send_targets)

        summary = ", ".join(f"{axis.joint_name}=0x{axis.node_id:02X}" for axis in self.axes)
        self.get_logger().info(
            "Ginkgo ODrive bridge ready "
            f"(channel={self.can_channel}, bitrate={self.can_bitrate_kbps} kbps, joints={summary})"
        )
        self.get_logger().info(
            f"Subscribing to {self.joint_states_topic} with QoS={self.joint_states_qos}"
        )
        if self.verbose:
            self.get_logger().info(
                f"Verbose logging enabled (period={self.verbose_period_s:.2f}s, "
                f"topic={self.joint_states_topic})"
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("can_channel", 0)
        self.declare_parameter("can_bitrate_kbps", 500)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_states_qos", "sensor_data")
        self.declare_parameter("send_hz", 20.0)
        self.declare_parameter("encoder_timeout_s", 0.25)
        self.declare_parameter("closed_loop_startup_delay_s", 0.05)
        self.declare_parameter("verbose", False)
        self.declare_parameter("verbose_period_s", 1.0)
        self.declare_parameter("require_linux_root", False)
        self.declare_parameter("require_startup_offsets", True)
        self.declare_parameter("set_axis_idle_on_shutdown", True)
        self.declare_parameter("joint_names", ["Joint1", "Joint2"])
        self.declare_parameter("node_ids", [16, 17])
        self.declare_parameter("gear_ratios", [48.0, 48.0])
        self.declare_parameter("directions", [-1.0, 1.0])
        self.declare_parameter("use_startup_offsets", [True, True])
        self.declare_parameter("command_modes", ["direct", "direct"])

    def _load_axis_configs(self) -> tuple[AxisConfig, ...]:
        joint_names = list(self.get_parameter("joint_names").value)
        node_ids = [int(value) for value in self.get_parameter("node_ids").value]
        gear_ratios = [float(value) for value in self.get_parameter("gear_ratios").value]
        directions = [float(value) for value in self.get_parameter("directions").value]
        use_startup_offsets = [
            bool(value) for value in self.get_parameter("use_startup_offsets").value
        ]
        command_modes = [str(value) for value in self.get_parameter("command_modes").value]

        lengths = {
            "joint_names": len(joint_names),
            "node_ids": len(node_ids),
            "gear_ratios": len(gear_ratios),
            "directions": len(directions),
            "use_startup_offsets": len(use_startup_offsets),
            "command_modes": len(command_modes),
        }
        if len(set(lengths.values())) != 1:
            rendered = ", ".join(f"{key}={value}" for key, value in lengths.items())
            raise ValueError(f"Joint parameter arrays must have the same length. Got: {rendered}")
        if not joint_names:
            raise ValueError("At least one joint must be configured.")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("joint_names must be unique.")
        if len(set(node_ids)) != len(node_ids):
            raise ValueError("node_ids must be unique.")

        axes: list[AxisConfig] = []
        for joint_name, node_id, gear_ratio, direction, use_startup_offset, command_mode in zip(
            joint_names,
            node_ids,
            gear_ratios,
            directions,
            use_startup_offsets,
            command_modes,
            strict=True,
        ):
            if not 0 <= node_id <= 0x3F:
                raise ValueError(f"node_id for {joint_name} must be in [0, 63].")
            if gear_ratio == 0.0:
                raise ValueError(f"gear_ratio for {joint_name} cannot be zero.")
            if direction == 0.0:
                raise ValueError(f"direction for {joint_name} cannot be zero.")
            if command_mode not in {"direct", "abs", "neg_abs"}:
                raise ValueError(
                    f"command_mode for {joint_name} must be one of: direct, abs, neg_abs."
                )

            axes.append(
                AxisConfig(
                    joint_name=str(joint_name),
                    node_id=node_id,
                    gear_ratio=gear_ratio,
                    direction=direction,
                    use_startup_offset=use_startup_offset,
                    command_mode=command_mode,
                )
            )

        return tuple(axes)

    def _build_joint_states_qos(self) -> QoSProfile:
        qos_name = self.joint_states_qos.lower()
        if qos_name == "sensor_data":
            return QoSPresetProfiles.SENSOR_DATA.value
        if qos_name == "default":
            return QoSProfile(depth=10)
        if qos_name == "reliable":
            return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        if qos_name == "best_effort":
            return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        raise ValueError(
            "joint_states_qos must be one of: sensor_data, default, reliable, best_effort."
        )

    def close(self) -> None:
        timer = getattr(self, "send_timer", None)
        if timer is not None:
            timer.cancel()

        if self.bus is None:
            return

        try:
            if self.set_axis_idle_on_shutdown and self.bus.is_open:
                for axis in self.axes:
                    self._send_axis_state(axis.node_id, AXIS_STATE_IDLE)
        except Exception as exc:
            self.get_logger().warning(f"Failed to set axes idle during shutdown: {exc}")
        finally:
            self.bus.close()
            self.bus = None

    def _enter_closed_loop(self) -> None:
        for axis in self.axes:
            self._send_axis_state(axis.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(self.closed_loop_startup_delay_s)

    def _send_axis_state(self, node_id: int, state: int) -> None:
        if self.bus is None:
            raise RuntimeError("CAN bus is not open.")
        self.bus.send_frame(
            make_cob_id(node_id, CMD_SET_AXIS_STATE),
            struct.pack("<I", state),
            extended=False,
        )

    def _capture_encoder_zeros(self) -> None:
        for axis in self.axes:
            if not axis.use_startup_offset:
                self.desired_turns[axis.node_id] = 0.0
                continue

            estimate = self._read_encoder_estimate(axis.node_id)
            if estimate is None:
                message = (
                    f"Encoder estimate timeout on node 0x{axis.node_id:02X} for {axis.joint_name}."
                )
                if self.require_startup_offsets:
                    raise RuntimeError(message)
                self.get_logger().warning(message + " Using 0.0 turns as startup zero.")
                self.desired_turns[axis.node_id] = 0.0
                continue

            position_turns, _velocity_turns = estimate
            self.encoder_zero_turns[axis.node_id] = position_turns
            self.desired_turns[axis.node_id] = position_turns
            self.get_logger().info(
                f"{axis.joint_name}: encoder startup zero={position_turns:.4f} turns"
            )

    def _read_encoder_estimate(self, node_id: int) -> tuple[float, float] | None:
        if self.bus is None:
            raise RuntimeError("CAN bus is not open.")

        request_id = make_cob_id(node_id, CMD_GET_ENCODER_ESTIMATES)
        self.bus.receive_available(max_frames=200)
        self.bus.send_frame(
            request_id,
            remote=True,
            extended=False,
            remote_data_len=8,
        )

        deadline = time.monotonic() + self.encoder_timeout_s
        while time.monotonic() < deadline:
            for frame in self.bus.receive_available(max_frames=50):
                if int(frame.ID) != request_id or frame.RemoteFlag:
                    continue
                if frame.DataLen < 8:
                    continue
                return decode_encoder_estimates(frame_payload(frame))
            time.sleep(0.01)
        return None

    def _joint_state_cb(self, msg: JointState) -> None:
        if not self._refresh_joint_index(msg):
            return

        for axis in self.axes:
            index = self.joint_index[axis.joint_name]
            if index >= len(msg.position):
                self.get_logger().warning(
                    "Received /joint_states with fewer positions than expected."
                )
                return

            raw_position = float(msg.position[index])
            self.last_source_position_rad[axis.joint_name] = raw_position
            target_turns = axis.relative_turns(raw_position)
            if axis.use_startup_offset:
                target_turns += self.encoder_zero_turns[axis.node_id]
            self.desired_turns[axis.node_id] = target_turns

        self._maybe_log_verbose_state("joint_state")

    def _refresh_joint_index(self, msg: JointState) -> bool:
        if len(self.joint_index) == len(self.axes):
            return True

        for axis in self.axes:
            if axis.joint_name in self.joint_index:
                continue
            try:
                self.joint_index[axis.joint_name] = msg.name.index(axis.joint_name)
            except ValueError:
                continue

        missing = [axis.joint_name for axis in self.axes if axis.joint_name not in self.joint_index]
        if missing:
            if not self._waiting_for_joints_logged:
                joined = ", ".join(missing)
                self.get_logger().warning(
                    f"Waiting for joints on {self.joint_states_topic}: {joined}"
                )
                self._waiting_for_joints_logged = True
            return False

        if self._waiting_for_joints_logged:
            self.get_logger().info(f"Expected joints found on {self.joint_states_topic}.")
            self._waiting_for_joints_logged = False
        if self.verbose:
            mapped = ", ".join(
                f"{axis.joint_name}->index {self.joint_index[axis.joint_name]}"
                for axis in self.axes
            )
            self.get_logger().info(f"Joint mapping resolved: {mapped}")
        return True

    def _send_targets(self) -> None:
        if self.bus is None:
            return

        try:
            for axis in self.axes:
                self.bus.send_frame(
                    make_cob_id(axis.node_id, CMD_SET_INPUT_POS),
                    encode_set_input_pos(self.desired_turns[axis.node_id]),
                    extended=False,
                )
            self._maybe_log_verbose_state("can_tx")
        except Exception as exc:
            self.get_logger().error(f"CAN transmit failed: {exc}")
            self.close()
            if rclpy.ok():
                rclpy.shutdown()

    def _maybe_log_verbose_state(self, source: str) -> None:
        if not self.verbose:
            return

        now = time.monotonic()
        if now - self._last_verbose_log_s < self.verbose_period_s:
            return

        rendered_axes: list[str] = []
        for axis in self.axes:
            rendered_axes.append(
                (
                    f"{axis.joint_name}(node=0x{axis.node_id:02X}, "
                    f"src={self.last_source_position_rad[axis.joint_name]:.5f} rad, "
                    f"mode={axis.command_mode}, "
                    f"enc0={self.encoder_zero_turns[axis.node_id]:.5f} turns, "
                    f"target={self.desired_turns[axis.node_id]:.5f} turns)"
                )
            )

        self.get_logger().info(f"[verbose:{source}] " + "; ".join(rendered_axes))
        self._last_verbose_log_s = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    bridge: JointStateBridge | None = None

    try:
        bridge = JointStateBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            bridge.close()
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
