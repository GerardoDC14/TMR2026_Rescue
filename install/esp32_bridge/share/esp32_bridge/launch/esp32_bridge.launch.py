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
        DeclareLaunchArgument(
            'laptop_ip',
            description='Destination IP for video streams (required)'
        ),
        DeclareLaunchArgument(
            'cameras',
            default_value='[auto]',
            description='YAML list of /dev/video* paths, or [auto] to discover'
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
                'frame_ms': 60,
                'device_index': LaunchConfiguration('audio_device'),
                'opus_bitrate_kbps': 10,
                'opus_complexity': 3,
                'opus_fec': True,
                'opus_packet_loss_pct': 20,
            }]
        ),
        Node(
            package='esp32_bridge',
            executable='gst_sender',
            name='gst_sender',
            output='screen',
            parameters=[{
                'laptop_ip': LaunchConfiguration('laptop_ip'),
                'cameras':   LaunchConfiguration('cameras'),
            }]
        ),
    ])
