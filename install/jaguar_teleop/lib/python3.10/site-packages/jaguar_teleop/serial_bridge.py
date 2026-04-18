import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not found. Install with: pip install pyserial")


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port',           '/dev/ttyUSB0')
        self.declare_parameter('baud',           115200)
        self.declare_parameter('publish_rate',   30.0)
        self.declare_parameter('joint4_name',    'Joint4')
        self.declare_parameter('joint5_name',    'Joint5')
        self.declare_parameter('joint6_name',    'Joint6')
        self.declare_parameter('gripper_speed',  40.0)
        self.declare_parameter('gripper_closed', 100.0)
        self.declare_parameter('gripper_open',   180.0)
        self.declare_parameter('lt_axis',        2)
        self.declare_parameter('rt_axis',        5)

        self.joint4_name    = self.get_parameter('joint4_name').value
        self.joint5_name    = self.get_parameter('joint5_name').value
        self.joint6_name    = self.get_parameter('joint6_name').value
        self.gripper_speed  = self.get_parameter('gripper_speed').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        self.gripper_open   = self.get_parameter('gripper_open').value
        self.lt_axis        = self.get_parameter('lt_axis').value
        self.rt_axis        = self.get_parameter('rt_axis').value
        self._port          = self.get_parameter('port').value
        self._baud          = self.get_parameter('baud').value

        self.j4_rad      = 0.0
        self.j5_rad      = 0.0
        self.j6_rad      = 0.0
        self.gripper_deg = self.gripper_closed
        self.lt_val      = 0.0
        self.rt_val      = 0.0
        self._last_send_time = self.get_clock().now()

        self._ser = self._try_open()

        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._send_cb)
        self.create_timer(3.0, self._reconnect_cb)

        self.get_logger().info(
            f'Serial bridge ready  |  tracking {self.joint4_name}, '
            f'{self.joint5_name}, {self.joint6_name}  |  {rate:.0f} Hz'
        )

    def _try_open(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
            self.get_logger().info(f'Serial opened: {self._port} @ {self._baud}')
            return ser
        except serial.SerialException as e:
            self.get_logger().warn(f'Cannot open {self._port}: {e} — will retry every 3 s')
            return None

    def _reconnect_cb(self):
        if self._ser is not None:
            return
        self._ser = self._try_open()

    def _joint_states_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == self.joint4_name:
                self.j4_rad = msg.position[i]
            elif name == self.joint5_name:
                self.j5_rad = msg.position[i]
            elif name == self.joint6_name:
                self.j6_rad = msg.position[i]

    def _joy_cb(self, msg: Joy):
        n = len(msg.axes)
        if self.lt_axis < n:
            self.lt_val = (1.0 - msg.axes[self.lt_axis]) / 2.0
        if self.rt_axis < n:
            self.rt_val = (1.0 - msg.axes[self.rt_axis]) / 2.0

    def _send_cb(self):
        now = self.get_clock().now()
        dt  = (now - self._last_send_time).nanoseconds * 1e-9
        self._last_send_time = now

        delta = (self.rt_val - self.lt_val) * self.gripper_speed * dt
        self.gripper_deg = max(
            self.gripper_closed,
            min(self.gripper_open, self.gripper_deg + delta)
        )

        payload = {
            'j4': round(self.j4_rad, 4),
            'j5': round(self.j5_rad, 4),
            'j6': round(self.j6_rad, 4),
            'gr': round(self.gripper_deg, 1),
        }
        line = json.dumps(payload) + '\n'

        if self._ser is not None:
            try:
                self._ser.write(line.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e} — reconnecting')
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
        else:
            self.get_logger().info(line.strip(), throttle_duration_sec=1.0)

    def destroy_node(self):
        if self._ser is not None:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
