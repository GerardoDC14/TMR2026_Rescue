from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='UART baud rate (must match MINIPC_BAUD in config.h)'
        ),
        DeclareLaunchArgument(
            'audio_device',
            default_value='-1',
            description='Audio device index (-1 for default)'
        ),
        Node(
            package='esp32_bridge',
            executable='main_bridge',
            name='main_bridge',
            output='screen',
            parameters=[{
                'baud_rate':   LaunchConfiguration('baud_rate'),
            }]
        ),
        Node(
            package='esp32_bridge',
            executable='servo_bridge',
            name='servo_bridge',
            output='screen',
            parameters=[{
                'baud_rate':   LaunchConfiguration('baud_rate'),
            }]
        ),
        Node(
            package='esp32_bridge',
            executable='rc_control',
            name='rc_control',
            output='screen'
        ),
        Node(
            package='esp32_bridge',
            executable='audio_node',
            name='audio_capture',
            output='screen',
            parameters=[{
                'sample_rate': 16000,
                'chunk_size': 1024,
                'device_index': LaunchConfiguration('audio_device'),
            }]
        ),
    ])
