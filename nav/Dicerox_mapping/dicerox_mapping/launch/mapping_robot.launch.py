import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('dicerox_mapping')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_x     = LaunchConfiguration('lidar_x')
    lidar_y     = LaunchConfiguration('lidar_y')
    lidar_z     = LaunchConfiguration('lidar_z')
    lidar_roll  = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw   = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_x',     default_value='0.0'),
        DeclareLaunchArgument('lidar_y',     default_value='-0.15'),
        DeclareLaunchArgument('lidar_z',     default_value='0.0'),
        DeclareLaunchArgument('lidar_roll',  default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw',   default_value='0.0'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=[
                lidar_x, lidar_y, lidar_z,
                lidar_yaw, lidar_pitch, lidar_roll,
                'zed_camera_link', 'laser',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
