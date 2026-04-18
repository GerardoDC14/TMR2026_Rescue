#!/usr/bin/env python3
"""
Xbox Servo Node
===============
Maps an Xbox One controller to MoveIt Servo twist commands, providing
the same kind of real-time arm control as rc_servo.py (PPM) but using
a standard gamepad via the ROS2 joy_node.

All movements are in the global (base_link) frame by default, so stick
directions always correspond to fixed world-space axes regardless of
end-effector orientation.  Press START to cycle to LOCAL (end-effector)
frame if you want tool-relative motion.

Control layout
--------------
    Left  stick Y   → X   (forward / back)        linear.x
    Left  stick X   → Y   (strafe  left / right)  linear.y
    Right stick Y   → Z   (up / down)             linear.z
    Right stick X   → Yaw                         angular.z

    LT (trigger)    → pitch −                     angular.y
    RT (trigger)    → pitch +

    LB (bumper)     → roll −                      angular.x
    RB (bumper)     → roll +

    START           → toggle GLOBAL / LOCAL frame
    Y               → pause / resume servo
    BACK            → stop (zero velocity)

Subscribes
----------
/joy                          sensor_msgs/Joy
/servo_node/status            std_msgs/Int8

Publishes
---------
/servo_node/delta_twist_cmds  geometry_msgs/TwistStamped
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

# ── Axis indices (XInput / Linux joy_node default) ───────────────────
AX_LX = 0
AX_LY = 1
AX_LT = 2
AX_RX = 3
AX_RY = 4
AX_RT = 5
AX_DX = 6
AX_DY = 7

# ── Button indices ───────────────────────────────────────────────────
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7
BTN_GUIDE = 8

# ── Tunables ─────────────────────────────────────────────────────────
SPEED     = 1.0       # overall velocity scale (multiplied by the servo
                      # node's max_linear_speed / max_rotational_speed)
DEADZONE  = 0.10      # stick centre dead-zone (normalised)

FRAME_BASE = 'base_link'
FRAME_EE   = 'Link6'

# Per-axis sign flip (match rc_servo conventions)
SIGN_X     =  1.0
SIGN_Y     = -1.0
SIGN_Z     =  1.0
SIGN_ROLL  =  1.0
SIGN_PITCH =  1.0
SIGN_YAW   = -1.0


def _deadzone(value: float) -> float:
    """Remove stick dead-zone and rescale the remainder to [-1, 1]."""
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEADZONE) / (1.0 - DEADZONE)


class XboxServo(Node):
    def __init__(self):
        super().__init__('xbox_servo')

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self._pause_cli = self.create_client(Trigger, '/servo_node/pause_servo')
        self._start_cli = self.create_client(Trigger, '/servo_node/start_servo')

        self._frame_local   = False    # False → global, True → end-effector
        self._servo_paused  = False
        self._last_status   = 0
        self._prev_buttons  = []
        # Triggers report 0.0 until first touched (Linux joy_node quirk) —
        # track "seen" so we don't ghost-pitch at startup.
        self._lt_seen = False
        self._rt_seen = False

        self.create_subscription(Joy,  '/joy',               self._joy_cb,    10)
        self.create_subscription(Int8, '/servo_node/status', self._status_cb, 10)

        self.get_logger().info('Xbox Servo ready — global frame, 6-DOF mapping')
        self._print_help()

    def _print_help(self):
        print('\n' + '=' * 60)
        print('  Jaguar Arm — Xbox Servo (6-DOF)')
        print('=' * 60)
        print('  Left  Y     X   (forward / back)')
        print('  Left  X     Y   (strafe)')
        print('  Right Y     Z   (up / down)')
        print('  Right X     Yaw')
        print('  LT / RT     Pitch  (− / +)')
        print('  LB / RB     Roll   (− / +)')
        print()
        print('  START       Toggle GLOBAL ↔ LOCAL frame')
        print('  Y           Pause / Resume servo')
        print('  BACK        Stop (zero velocity)')
        print('=' * 60 + '\n')

    # ── Servo status ─────────────────────────────────────────────────

    _STATUS_TEXT = {
        1: 'WARNING  Approaching singularity',
        2: 'HALT     Singularity reached',
        3: 'WARNING  Leaving singularity',
        4: 'HALT     Collision detected',
        5: 'WARNING  Near joint limit',
    }

    def _status_cb(self, msg: Int8):
        code = msg.data
        if code == self._last_status:
            return
        self._last_status = code
        text = self._STATUS_TEXT.get(code)
        if text:
            self.get_logger().warn(f'[SERVO] {text}')
        elif code == 0:
            self.get_logger().info('[SERVO] OK')

    # ── Pause / resume ───────────────────────────────────────────────

    def _call_service(self, client, label):
        if not client.service_is_ready():
            self.get_logger().warn(f'{label} service not available')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{label}: {f.result().message}' if f.result() else f'{label}: no response'
            )
        )

    def _toggle_pause(self):
        if self._servo_paused:
            self._servo_paused = False
            self._call_service(self._start_cli, 'start_servo')
            self.get_logger().info('Servo RESUMED — xbox active')
        else:
            self._publish_zero()
            self._servo_paused = True
            self._call_service(self._pause_cli, 'pause_servo')
            self.get_logger().info('Servo PAUSED — RViz / move_group active')

    # ── Joy callback ─────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        axes    = msg.axes
        buttons = list(msg.buttons)

        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        def just_pressed(idx):
            return (idx < len(buttons) and
                    buttons[idx] == 1 and
                    self._prev_buttons[idx] == 0)

        # BACK → immediate zero
        if just_pressed(BTN_BACK):
            self._publish_zero()
            self._prev_buttons = buttons
            return

        # Y → toggle pause/resume
        if just_pressed(BTN_Y):
            threading.Thread(target=self._toggle_pause, daemon=True).start()
            self._prev_buttons = buttons
            return

        if self._servo_paused:
            self._prev_buttons = buttons
            return

        # START → toggle frame
        if just_pressed(BTN_START):
            self._frame_local = not self._frame_local
            self.get_logger().info(
                f'Frame → {"LOCAL (EE)" if self._frame_local else "GLOBAL (base_link)"}')

        self._publish_twist(axes, buttons)
        self._prev_buttons = buttons

    # ── Twist builder ────────────────────────────────────────────────

    def _trigger(self, axes, idx, seen_attr) -> float:
        """Read a trigger axis as a [0, 1] magnitude, handling the joy_node
        "untouched = 0" quirk (first touch flips the sign convention)."""
        raw = axes[idx] if len(axes) > idx else 0.0
        if raw != 0.0:
            setattr(self, seen_attr, True)
        if not getattr(self, seen_attr):
            return 0.0
        # After first touch: 1.0 (released) .. -1.0 (fully pressed)
        return max(0.0, min(1.0, (1.0 - raw) / 2.0))

    def _publish_twist(self, axes, buttons):
        lx = _deadzone(axes[AX_LX]) if len(axes) > AX_LX else 0.0
        ly = _deadzone(axes[AX_LY]) if len(axes) > AX_LY else 0.0
        rx = _deadzone(axes[AX_RX]) if len(axes) > AX_RX else 0.0
        ry = _deadzone(axes[AX_RY]) if len(axes) > AX_RY else 0.0

        lt = self._trigger(axes, AX_LT, '_lt_seen')
        rt = self._trigger(axes, AX_RT, '_rt_seen')
        pitch = rt - lt       # [-1, 1]

        lb = 1.0 if (len(buttons) > BTN_LB and buttons[BTN_LB]) else 0.0
        rb = 1.0 if (len(buttons) > BTN_RB and buttons[BTN_RB]) else 0.0
        roll = rb - lb        # {-1, 0, 1}

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_EE if self._frame_local else FRAME_BASE

        msg.twist.linear.x  = ly    * SPEED * SIGN_X
        msg.twist.linear.y  = lx    * SPEED * SIGN_Y
        msg.twist.linear.z  = ry    * SPEED * SIGN_Z
        msg.twist.angular.x = roll  * SPEED * SIGN_ROLL
        msg.twist.angular.y = pitch * SPEED * SIGN_PITCH
        msg.twist.angular.z = rx    * SPEED * SIGN_YAW

        self.twist_pub.publish(msg)

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_BASE
        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = XboxServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
