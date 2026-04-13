#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import serial
import math

class ESP32SerialForwarder(Node):
    def __init__(self):
        super().__init__('esp32_serial_forwarder')
        
        # 1. Declare parameters for easy configuration
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        # Update these default names to match your URDF/Robot Description
        self.declare_parameter('joint_names', ['Joint4', 'Joint5', 'Joint6'])
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.target_joints = self.get_parameter('joint_names').value
        
        # 2. Setup Serial Connection
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e
        
        # 3. Subscribe to the joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 4. Subscribe to gripper value from RC keybind system
        self.gripper_value = 0.0
        self.create_subscription(
            Float32,
            '/gripper',
            self.gripper_callback,
            10
        )
        self.get_logger().info("Listening to /joint_states and /gripper...")

    def gripper_callback(self, msg):
        self.gripper_value = msg.data

    def joint_state_callback(self, msg):
        try:
            # Find the indices of our target joints in the incoming message array
            indices = []
            for name in self.target_joints:
                if name in msg.name:
                    indices.append(msg.name.index(name))
                else:
                    # If a message arrives without one of our joints, ignore it
                    self.get_logger().debug(f"Waiting for joint '{name}' in /joint_states")
                    return

            # Extract positions in radians
            j4_rad = msg.position[indices[0]]
            j5_rad = msg.position[indices[1]]
            j6_rad = msg.position[indices[2]]

            # Convert radians to degrees (ESP32 expects degrees for constrain/driveServo)
            j4_deg = math.degrees(j4_rad)
            j5_deg = math.degrees(j5_rad)
            j6_deg = math.degrees(j6_rad)
            # Gripper: normalised [-1,+1] from RC → map to degrees [0, 180]
            grip_deg = (self.gripper_value + 1.0) * 90.0
            
            # Format exactly as the ESP32 expects: "deg4,deg5,deg6,grip\n"
            # Using 2 decimal places for precision
            serial_msg = f"{j4_deg:.2f},{j5_deg:.2f},{j6_deg:.2f},{grip_deg:.2f}\n"
            
            # Send over Serial
            self.serial_port.write(serial_msg.encode('utf-8'))
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