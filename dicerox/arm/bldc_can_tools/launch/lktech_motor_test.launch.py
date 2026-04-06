from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_params = (
        get_package_share_directory("bldc_can_tools")
        + "/config/default_params.yaml"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the parameter YAML file.",
    )

    params_file = LaunchConfiguration("params_file")

    monitor = Node(
        package="bldc_can_tools",
        executable="lktech_monitor",
        name="lktech_monitor",
        output="screen",
        parameters=[params_file],
    )

    position_test = Node(
        package="bldc_can_tools",
        executable="lktech_position_test",
        name="lktech_position_test",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([params_file_arg, monitor, position_test])

