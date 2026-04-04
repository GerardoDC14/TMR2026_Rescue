#!/usr/bin/env python3
"""
Joint State Relay
=================
Closes the loop between the MoveIt / damped_servo IK pipeline and the
physical arm motors controlled through the ESP32.

The damped_servo computes joint positions from twist commands and publishes
JointTrajectory to the JointTrajectoryController (mock hardware).  The
controller updates /joint_states.  This node reads those positions and
republishes them on /arm/joint_command so the esp32_bridge can forward
them to the ESP32, which drives the ODrive motors over CAN.

    damped_servo -> JointTrajectory -> controller -> /joint_states
                                                          |
                                              joint_relay (this node)
                                                          |
                                                  /arm/joint_command
                                                          |
                                                    esp32_bridge
                                                          |
                                                   ESP32 -> CAN -> ODrive

Subscribes
----------
/joint_states   sensor_msgs/JointState       joint positions from controller

Publishes
---------
/arm/joint_command   std_msgs/Float32MultiArray   6 joint angles in degrees
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
N_JOINTS = 6


class JointRelay(Node):
    def __init__(self):
        super().__init__('joint_relay')

        self.declare_parameter('publish_rate', 50.0)
        rate = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(Float32MultiArray, '/arm/joint_command', 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self._positions_rad = [0.0] * N_JOINTS
        self._received = False

        self.create_timer(1.0 / rate, self._publish_cb)
        self.get_logger().info(
            'Joint relay ready — forwarding /joint_states -> /arm/joint_command')

    def _js_cb(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._positions_rad[i] = msg.position[idx]
        self._received = True

    def _publish_cb(self):
        if not self._received:
            return

        msg = Float32MultiArray()
        msg.data = [math.degrees(r) for r in self._positions_rad]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = JointRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
