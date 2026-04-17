#!/usr/bin/env python3
"""
Xbox Servo Node
===============
Maps an Xbox controller to MoveIt Servo twist commands, providing the
same kind of real-time arm control as rc_servo.py (PPM) but using a
standard gamepad via the ROS2 joy_node.

All movements are in the global (base_link) frame so that pushing a
stick direction always corresponds to the same world-space axis
regardless of end-effector orientation.

Hardcoded button/axis layout (Xbox Series / Xbox One / generic XInput):

    Left stick Y    → X   (forward / back)
    Left stick X    → Y   (strafe left / right)
    Right stick Y   → Z   (up / down)
    Right stick X   → Yaw
    Hold LB         → Left X  → Roll,   Right Y → Pitch
    Hold RB         → Left X  → Y,      Right Y → Pitch

    D-pad Up/Down   → select joint (JOINT mode)
    Left stick Y    → jog selected joint (JOINT mode)

    START           → cycle mode (GLOBAL → LOCAL → JOINT)
    Y               → pause / resume servo
    BACK            → stop (zero velocity)

Subscribes
----------
/joy                          sensor_msgs/Joy
/servo_node/status            std_msgs/Int8

Publishes
---------
/servo_node/delta_twist_cmds  geometry_msgs/TwistStamped
/servo_node/delta_joint_cmds  control_msgs/JointJog
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

# ── Axis indices (XInput layout) ─────────────────────────────────────
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
SPEED     = 1.0       # overall velocity scale
DEADZONE  = 0.10      # normalised dead-zone for stick axes

FRAME_BASE = 'base_link'
FRAME_EE   = 'Link6'

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
NUM_JOINTS  = 6
MODES       = ['global', 'local', 'joint']

# Per-axis sign flip (match rc_servo conventions)
SIGN_X     =  1.0
SIGN_Y     = -1.0
SIGN_Z     =  1.0
SIGN_ROLL  =  1.0
SIGN_PITCH =  1.0
SIGN_YAW   = -1.0


def _deadzone(value: float) -> float:
    """Remove stick dead-zone and rescale the remaining range to [0, 1]."""
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEADZONE) / (1.0 - DEADZONE)


class XboxServo(Node):
    def __init__(self):
        super().__init__('xbox_servo')

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)

        self._pause_cli = self.create_client(Trigger, '/servo_node/pause_servo')
        self._start_cli = self.create_client(Trigger, '/servo_node/start_servo')

        self.mode          = 'global'
        self.active_joint  = 0
        self._servo_paused = False
        self._last_status  = 0
        self._prev_buttons = []
        self._dpad_y_prev  = 0.0

        self.create_subscription(Joy,  '/joy',               self._joy_cb,    10)
        self.create_subscription(Int8, '/servo_node/status', self._status_cb, 10)

        self.get_logger().info('Xbox Servo ready — global frame, hardcoded keybinds')
        self._print_help()

    def _print_help(self):
        print('\n' + '=' * 60)
        print('  Jaguar Arm — Xbox Servo')
        print('=' * 60)
        print('  GLOBAL mode (default) — base_link frame')
        print('  LOCAL  mode           — end-effector frame')
        print('    Left  Y      Forward / Back   (X)')
        print('    Left  X      Strafe           (Y)')
        print('    Right Y      Up / Down        (Z)')
        print('    Right X      Yaw')
        print('    Hold LB      Left X → Roll,  Right Y → Pitch')
        print('    Hold RB      Left X → Y,     Right Y → Pitch')
        print()
        print('  JOINT mode')
        print('    D-pad Up/Dn  Select joint')
        print('    Left Y       Jog selected joint')
        print()
        print('  START   Cycle mode (GLOBAL → LOCAL → JOINT)')
        print('  Y       Pause / Resume servo')
        print('  BACK    Stop (zero velocity)')
        print('=' * 60 + '\n')

    # ── Status ───────────────────────────────────────────────────────

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

        # START → cycle mode
        if just_pressed(BTN_START):
            idx = MODES.index(self.mode) if self.mode in MODES else 0
            self.mode = MODES[(idx + 1) % len(MODES)]
            self.get_logger().info(f'Mode → {self.mode.upper()}')

        if self.mode in ('global', 'local'):
            self._handle_cart(axes, buttons)
        else:
            self._handle_joint(axes, buttons)

        self._prev_buttons = buttons

    def _handle_cart(self, axes, buttons):
        lx = _deadzone(axes[AX_LX]) if len(axes) > AX_LX else 0.0
        ly = _deadzone(axes[AX_LY]) if len(axes) > AX_LY else 0.0
        rx = _deadzone(axes[AX_RX]) if len(axes) > AX_RX else 0.0
        ry = _deadzone(axes[AX_RY]) if len(axes) > AX_RY else 0.0

        lb = len(buttons) > BTN_LB and buttons[BTN_LB]
        rb = len(buttons) > BTN_RB and buttons[BTN_RB]

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_EE if self.mode == 'local' else FRAME_BASE

        if lb:
            # LB held: left X → roll, right Y → pitch
            msg.twist.linear.x  =  ly * SPEED * SIGN_X
            msg.twist.linear.y  =  0.0
            msg.twist.linear.z  =  ry * SPEED * SIGN_Z
            msg.twist.angular.x =  lx * SPEED * SIGN_ROLL
            msg.twist.angular.y =  0.0
            msg.twist.angular.z =  rx * SPEED * SIGN_YAW
        elif rb:
            # RB held: left X → Y, right Y → pitch
            msg.twist.linear.x  =  ly * SPEED * SIGN_X
            msg.twist.linear.y  =  lx * SPEED * SIGN_Y
            msg.twist.linear.z  =  0.0
            msg.twist.angular.x =  0.0
            msg.twist.angular.y =  ry * SPEED * SIGN_PITCH
            msg.twist.angular.z =  rx * SPEED * SIGN_YAW
        else:
            # Default: LY=X, LX=Y, RY=Z, RX=Yaw
            msg.twist.linear.x  =  ly * SPEED * SIGN_X
            msg.twist.linear.y  =  lx * SPEED * SIGN_Y
            msg.twist.linear.z  =  ry * SPEED * SIGN_Z
            msg.twist.angular.x =  0.0
            msg.twist.angular.y =  0.0
            msg.twist.angular.z =  rx * SPEED * SIGN_YAW

        self.twist_pub.publish(msg)

    def _handle_joint(self, axes, buttons):
        dpad_y = axes[AX_DY] if len(axes) > AX_DY else 0.0
        if dpad_y > 0.5 and self._dpad_y_prev <= 0.5:
            self.active_joint = (self.active_joint - 1) % NUM_JOINTS
            self.get_logger().info(f'Joint → {JOINT_NAMES[self.active_joint]}')
        elif dpad_y < -0.5 and self._dpad_y_prev >= -0.5:
            self.active_joint = (self.active_joint + 1) % NUM_JOINTS
            self.get_logger().info(f'Joint → {JOINT_NAMES[self.active_joint]}')
        self._dpad_y_prev = dpad_y

        velocity = _deadzone(axes[AX_LY]) * SPEED if len(axes) > AX_LY else 0.0

        msg = JointJog()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_BASE
        msg.joint_names     = [JOINT_NAMES[self.active_joint]]
        msg.velocities      = [velocity]
        msg.duration        = 0.0
        self.joint_pub.publish(msg)

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
