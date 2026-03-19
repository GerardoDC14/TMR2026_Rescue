#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

# ── Axis indices ──────────────────────────────────────────────────────
AX_LX   = 0
AX_LY   = 1
AX_LT   = 2
AX_RX   = 3
AX_RY   = 4
AX_RT   = 5
AX_DX   = 6
AX_DY   = 7

# ── Button indices ────────────────────────────────────────────────────
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7
BTN_GUIDE = 8
BTN_L3    = 9
BTN_R3    = 10

# ── MoveIt Servo status codes ─────────────────────────────────────────
_STATUS = {
    0: None,
    1: 'WARNING  Approaching singularity — slowing down',
    2: 'HALT     Singularity reached — halted',
    3: 'WARNING  Leaving singularity',
    4: 'HALT     Collision detected — halted',
    5: 'WARNING  Near joint limit',
}

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
FRAME_ID    = 'base_link'
DEADZONE    = 0.10
NUM_JOINTS  = 6


def apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_value(raw: float) -> float:
    return (1.0 - raw) / 2.0


class JoystickServo(Node):
    def __init__(self):
        super().__init__('joystick_servo')

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)

        self._pause_cli = self.create_client(Trigger, '/servo_node/pause_servo')
        self._start_cli = self.create_client(Trigger, '/servo_node/start_servo')

        self.mode          = 'cart'
        self.active_joint  = 0
        self.speed         = 1.0
        self._servo_paused = False
        self._last_status  = 0

        self._prev_buttons = []
        self._dpad_y_prev  = 0.0

        self.create_subscription(Joy,  '/joy',               self._joy_cb,    10)
        self.create_subscription(Int8, '/servo_node/status', self._status_cb, 10)

        self.get_logger().info('Joystick Servo ready.')
        self._print_help()

    def _print_help(self):
        print('\n' + '=' * 60)
        print('  Jaguar Arm — Joystick Servo  (Xbox Series X)')
        print('=' * 60)
        print('  CART mode (default)')
        print('    Left  Y      Forward / Back   (+X)')
        print('    Left  X      Strafe            (+Y)')
        print('    Right Y      Up / Down         (+Z)')
        print('    Right X      Yaw')
        print('    Hold LB      Left X  → Roll')
        print('    Hold RB      Right Y → Pitch')
        print('    Back         Stop (zero velocity)')
        print('    Start        Switch to JOINT mode')
        print()
        print('  JOINT mode')
        print('    D-pad Up/Dn  Select joint')
        print('    Left Y       Jog selected joint')
        print('    Start        Switch to CART mode')
        print()
        print('  Y              Pause servo / Resume  (hand off to RViz)')
        print('  Guide          Emergency stop (hold)')
        print('=' * 60 + '\n')

    # ── Servo status ──────────────────────────────────────────────────

    def _status_cb(self, msg: Int8):
        code = msg.data
        if code == self._last_status:
            return
        self._last_status = code
        text = _STATUS.get(code)
        if text:
            self.get_logger().warn(f'[SERVO STATUS] {text}')
        elif code == 0:
            self.get_logger().info('[SERVO STATUS] OK')

    # ── Pause / resume ────────────────────────────────────────────────

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
            self.get_logger().info('Servo RESUMED — joystick active')
        else:
            self._publish_zero()
            self._servo_paused = True
            self._call_service(self._pause_cli, 'pause_servo')
            self.get_logger().info('Servo PAUSED — RViz / move_group active')

    # ── Joy callback ──────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        axes    = msg.axes
        buttons = list(msg.buttons)

        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        def just_pressed(idx):
            return (idx < len(buttons) and
                    buttons[idx] == 1 and
                    self._prev_buttons[idx] == 0)

        if len(buttons) > BTN_GUIDE and buttons[BTN_GUIDE]:
            self._publish_zero()
            self._prev_buttons = buttons
            return

        if just_pressed(BTN_Y):
            threading.Thread(target=self._toggle_pause, daemon=True).start()
            self._prev_buttons = buttons
            return

        if self._servo_paused:
            self._prev_buttons = buttons
            return

        if just_pressed(BTN_START):
            self.mode = 'joint' if self.mode == 'cart' else 'cart'
            self.get_logger().info(f'Mode → {self.mode.upper()}')

        if just_pressed(BTN_BACK):
            self._publish_zero()
            self._prev_buttons = buttons
            return

        if self.mode == 'cart':
            self._handle_cart(axes, buttons)
        else:
            self._handle_joint(axes, buttons)

        self._prev_buttons = buttons

    def _handle_cart(self, axes, buttons):
        lx = apply_deadzone(axes[AX_LX]) if len(axes) > AX_LX else 0.0
        ly = apply_deadzone(axes[AX_LY]) if len(axes) > AX_LY else 0.0
        rx = apply_deadzone(axes[AX_RX]) if len(axes) > AX_RX else 0.0
        ry = apply_deadzone(axes[AX_RY]) if len(axes) > AX_RY else 0.0

        lb = len(buttons) > BTN_LB and buttons[BTN_LB]
        rb = len(buttons) > BTN_RB and buttons[BTN_RB]

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        if lb:
            msg.twist.linear.x  =  ly  * self.speed
            msg.twist.linear.y  =  0.0
            msg.twist.linear.z  =  ry  * self.speed
            msg.twist.angular.x =  lx  * self.speed
            msg.twist.angular.y =  0.0
            msg.twist.angular.z = -rx  * self.speed
        elif rb:
            msg.twist.linear.x  =  ly  * self.speed
            msg.twist.linear.y  = -lx  * self.speed
            msg.twist.linear.z  =  0.0
            msg.twist.angular.x =  0.0
            msg.twist.angular.y =  ry  * self.speed
            msg.twist.angular.z = -rx  * self.speed
        else:
            msg.twist.linear.x  =  ly  * self.speed
            msg.twist.linear.y  = -lx  * self.speed
            msg.twist.linear.z  =  ry  * self.speed
            msg.twist.angular.x =  0.0
            msg.twist.angular.y =  0.0
            msg.twist.angular.z = -rx  * self.speed

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

        velocity = apply_deadzone(axes[AX_LY]) * self.speed if len(axes) > AX_LY else 0.0

        msg = JointJog()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.joint_names     = [JOINT_NAMES[self.active_joint]]
        msg.velocities      = [velocity]
        msg.duration        = 0.0
        self.joint_pub.publish(msg)

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
