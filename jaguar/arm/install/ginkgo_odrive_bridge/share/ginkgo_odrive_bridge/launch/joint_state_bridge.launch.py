from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ginkgo_odrive_bridge"))
    default_params = package_share / "config" / "joint_state_bridge.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Path to the ROS 2 parameter file for the bridge node.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose bridge logging.",
            ),
            DeclareLaunchArgument(
                "verbose_period_s",
                default_value="1.0",
                description="Minimum period between verbose log lines.",
            ),
            Node(
                package="ginkgo_odrive_bridge",
                executable="joint_state_bridge",
                name="ginkgo_joint_state_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "verbose": ParameterValue(
                            LaunchConfiguration("verbose"),
                            value_type=bool,
                        ),
                        "verbose_period_s": ParameterValue(
                            LaunchConfiguration("verbose_period_s"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
