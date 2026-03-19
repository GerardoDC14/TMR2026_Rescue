from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        Node(
            package='jaguar_teleop',
            executable='joystick_servo',
            name='joystick_servo',
            output='screen',
        ),

        Node(
            package='jaguar_teleop',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
    ])
