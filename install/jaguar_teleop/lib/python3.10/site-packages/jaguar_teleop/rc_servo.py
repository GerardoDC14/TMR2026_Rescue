#!/usr/bin/env python3
"""
RC Servo Node
=============
Maps FlySky RC receiver PPM channels to MoveIt Servo twist commands
when the robot is in ARM mode (determined by Ch5 / keybinds on the ESP32).

Channel-to-axis mapping is read from the /robot/keybind topic (set via
the GUI keybind dialog).  Each channel slot can be assigned to ARM_X,
ARM_Y, ARM_Z, ARM_PITCH, or ARM_YAW.  This node decodes Ch5 to find
the active keybind row, then maps each channel's normalised PPM value
to the corresponding twist axis.

Subscribes
----------
/robot/ppm       std_msgs/Int16MultiArray   raw PPM us [ch1..ch6]
/robot/mode      std_msgs/String            current RobotMode name
/robot/keybind   std_msgs/UInt8MultiArray   15 bytes (3 modes x 5 ch slots)

Publishes
---------
/servo_node/delta_twist_cmds  geometry_msgs/TwistStamped  Cartesian velocity
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Int16MultiArray, UInt8MultiArray

# ─── Tunables (change here) ──────────────────────────────────────────────────
SPEED       = 1.0          # overall velocity scale applied to all axes
FRAME_ID    = 'base_link'   # 'base_link' = global,  'Link6' = local / end-effector
PUBLISH_HZ  = 30.0
DEADBAND    = 0.08         # normalised dead-zone around stick centre
PPM_MIN     = 1000         # us
PPM_MAX     = 2000
PPM_CENTER  = 1500

# Per-axis sign flip — adjust if a stick direction feels inverted
SIGN_X      =  1.0
SIGN_Y      = -1.0
SIGN_Z      =  1.0
SIGN_PITCH  =  1.0
SIGN_YAW    = -1.0

# ─── ChannelFunction enum values (must match firmware / GUI) ─────────────────
ARM_FWD   = 8
ARM_X     = 10
ARM_Y     = 11
ARM_Z     = 12
ARM_PITCH = 13
ARM_YAW   = 14

# Keybind slot index -> raw PPM channel index  (Ch5 is mode switch, not a slot)
_SLOT_TO_PPM = [0, 1, 2, 3, 5]   # slot 0=Ch1, 1=Ch2, 2=Ch3, 3=Ch4, 4=Ch6

PPM_CHANNELS = 6
KEYBIND_SIZE = 15   # 3 modes x 5 slots


class RCServo(Node):
    def __init__(self):
        super().__init__('rc_servo')

        # ── State ─────────────────────────────────────────────────────────
        self._arm_active = False
        self._ppm = [PPM_CENTER] * PPM_CHANNELS
        self._keybind = [[0] * 5 for _ in range(3)]   # 3 modes x 5 slots

        # ── Publisher ─────────────────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        keybind_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(Int16MultiArray,  '/robot/ppm',     self._ppm_cb,     best_effort)
        self.create_subscription(String,           '/robot/mode',    self._mode_cb,    best_effort)
        self.create_subscription(UInt8MultiArray,  '/robot/keybind', self._keybind_cb, keybind_qos)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

        self.get_logger().info(
            f'RC Servo ready  (frame={FRAME_ID}, speed={SPEED}) — waiting for ARM mode')

    # ── Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _normalise(raw_us: int) -> float:
        """Map raw PPM microseconds to [-1, +1] with deadband removal."""
        ctr = PPM_CENTER
        if raw_us >= ctr:
            half = max(PPM_MAX - ctr, 1)
            v = (raw_us - ctr) / half
        else:
            half = max(ctr - PPM_MIN, 1)
            v = (raw_us - ctr) / half

        v = max(-1.0, min(1.0, v))
        if abs(v) < DEADBAND:
            return 0.0
        sign = 1.0 if v > 0.0 else -1.0
        return sign * (abs(v) - DEADBAND) / (1.0 - DEADBAND)

    @staticmethod
    def _decode_mode_index(ch5_us: int) -> int:
        """Replicate firmware Ch5 three-position decoding."""
        low  = PPM_MIN + (PPM_MAX - PPM_MIN) // 4   # ~1250
        high = PPM_MAX - (PPM_MAX - PPM_MIN) // 4   # ~1750
        if ch5_us < low:
            return 0
        if ch5_us > high:
            return 2
        return 1

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _ppm_cb(self, msg: Int16MultiArray):
        if len(msg.data) >= PPM_CHANNELS:
            self._ppm = list(msg.data[:PPM_CHANNELS])

    def _mode_cb(self, msg: String):
        was_arm = self._arm_active
        self._arm_active = (msg.data == 'ARM')

        if self._arm_active and not was_arm:
            self.get_logger().info('ARM mode active — RC controlling arm')
        elif not self._arm_active and was_arm:
            self.get_logger().info(f'Left ARM mode ({msg.data}) — stopping arm')
            self._publish_zero()

    def _keybind_cb(self, msg: UInt8MultiArray):
        if len(msg.data) < KEYBIND_SIZE:
            return
        for m in range(3):
            for c in range(5):
                self._keybind[m][c] = msg.data[m * 5 + c]
        self.get_logger().info('Keybind table updated')

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_cb(self):
        if not self._arm_active:
            return

        mode_idx = self._decode_mode_index(self._ppm[4])   # Ch5 = index 4
        row = self._keybind[mode_idx]

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        for slot in range(5):
            fn = row[slot]
            ppm_ch = _SLOT_TO_PPM[slot]
            val = self._normalise(self._ppm[ppm_ch]) * SPEED

            if fn == ARM_X:
                msg.twist.linear.x  = val * SIGN_X
            elif fn == ARM_Y:
                msg.twist.linear.y  = val * SIGN_Y
            elif fn == ARM_Z:
                msg.twist.linear.z  = val * SIGN_Z
            elif fn == ARM_PITCH:
                msg.twist.angular.y = val * SIGN_PITCH
            elif fn == ARM_YAW:
                msg.twist.angular.z = val * SIGN_YAW
            # ARM_FWD (legacy 8) and non-arm functions are ignored

        self._twist_pub.publish(msg)

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        self._twist_pub.publish(msg)


def main():
    rclpy.init()
    node = RCServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
