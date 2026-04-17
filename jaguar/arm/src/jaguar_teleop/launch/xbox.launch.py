"""
Xbox Arm Control Launch
=======================
Starts the nodes needed for Xbox-controller-to-arm control:

  joy_node       Reads /dev/input/js0 and publishes sensor_msgs/Joy.
  xbox_servo     Maps joystick axes/buttons to twist commands (global frame).
  joint_relay    Forwards computed joint positions to the ESP32 bridge.

Prerequisites (launched separately):
  1. MoveIt demo:       ros2 launch jaguar_full demo.launch.py
  2. Damped servo:      ros2 launch jaguar_full damped_servo.launch.py
  3. ESP32 bridge:      ros2 launch esp32_bridge esp32_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': LaunchConfiguration('joy_dev')}],
        ),

        Node(
            package='jaguar_teleop',
            executable='xbox_servo',
            name='xbox_servo',
            output='screen',
        ),

        Node(
            package='jaguar_teleop',
            executable='joint_relay',
            name='joint_relay',
            output='screen',
        ),
    ])
