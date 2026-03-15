from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='/home/robotec/maps/dicerox_map',
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/slam_toolbox/serialize_map',
                'slam_toolbox/srv/SerializePoseGraph',
                ['{filename: "', map_name, '"}'],
            ],
            output='screen',
        ),
    ])
