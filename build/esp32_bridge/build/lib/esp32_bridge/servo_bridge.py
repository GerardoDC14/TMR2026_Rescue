#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
import serial
import math

class ESP32SerialForwarder(Node):
    def __init__(self):
        super().__init__('esp32_serial_forwarder')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_period', 3.0)
        self.declare_parameter('joint_names', ['Joint4', 'Joint5', 'Joint6'])

        self._port_name       = self.get_parameter('port').value
        self._baudrate        = self.get_parameter('baudrate').value
        self._reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.target_joints    = self.get_parameter('joint_names').value
        self.serial_port: serial.Serial | None = None

        # Try to connect in background so the node starts even without the ESP32
        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._connect_thread.start()

        self._estop_active = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.gripper_value = 0.0
        self.create_subscription(
            Float32,
            '/gripper',
            self.gripper_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/robot/estop',
            self._estop_callback,
            10
        )
        self.get_logger().info("Listening to /joint_states, /gripper, and /robot/estop...")

    def _connect_loop(self):
        while rclpy.ok() and self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self._port_name, self._baudrate, timeout=0.1)
                self.get_logger().info(
                    f"Connected to ESP32 on {self._port_name} at {self._baudrate} baud.")
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(
                    f"Serial port {self._port_name} unavailable ({e}); "
                    f"retrying in {self._reconnect_period:.0f}s")
                time.sleep(self._reconnect_period)

    def gripper_callback(self, msg):
        self.gripper_value = msg.data

    def _estop_callback(self, msg: Bool):
        if msg.data and not self._estop_active:
            self.get_logger().warn('ESTOP active — suppressing serial output')
        elif not msg.data and self._estop_active:
            self.get_logger().info('ESTOP cleared — resuming serial output')
        self._estop_active = msg.data

    def _serial_write(self, data: bytes):
        if self.serial_port is None:
            return
        try:
            self.serial_port.write(data)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial write error: {e}; will reconnect")
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
            self._connect_thread.start()

    def joint_state_callback(self, msg):
        if self._estop_active:
            return
        try:
            indices = []
            for name in self.target_joints:
                if name in msg.name:
                    indices.append(msg.name.index(name))
                else:
                    self.get_logger().debug(f"Waiting for joint '{name}' in /joint_states")
                    return

            j4_deg = math.degrees(msg.position[indices[0]])
            j5_deg = math.degrees(msg.position[indices[1]])
            j6_deg = math.degrees(msg.position[indices[2]])
            grip_deg = (self.gripper_value + 1.0) * 90.0

            serial_msg = f"{j4_deg:.2f},{j5_deg:.2f},{j6_deg:.2f},{grip_deg:.2f}\n"
            self._serial_write(serial_msg.encode('utf-8'))
            self.get_logger().debug(f"TX: {serial_msg.strip()}")

        except Exception as e:
            self.get_logger().error(f"Error parsing joint states: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialForwarder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down serial forwarder...")
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()