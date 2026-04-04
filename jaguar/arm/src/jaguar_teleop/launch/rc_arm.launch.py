"""
RC Arm Control Launch
=====================
Starts the two nodes needed for RC-to-arm control:

  rc_servo      Maps PPM channels to twist commands when in ARM mode.
                Channel-to-axis mapping comes from /robot/keybind (GUI).
                Speed, frame, and sign flips are constants at the top
                of rc_servo.py.

  joint_relay   Forwards computed joint positions to the ESP32 bridge
                so they reach the physical ODrive motors over CAN.

Prerequisites (launched separately):
  1. MoveIt demo:       ros2 launch jaguar_full demo.launch.py
  2. Damped servo:      ros2 launch jaguar_full damped_servo.launch.py
  3. ESP32 bridge:      ros2 launch esp32_bridge esp32_bridge.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jaguar_teleop',
            executable='rc_servo',
            name='rc_servo',
            output='screen',
        ),

        Node(
            package='jaguar_teleop',
            executable='joint_relay',
            name='joint_relay',
            output='screen',
        ),
    ])
