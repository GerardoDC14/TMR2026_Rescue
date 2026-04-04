"""
Launch the Damped Least-Squares servo node.

Drop-in replacement for servo.launch.py — use this instead of MoveIt Servo
to eliminate singularity hard-stops.

Usage (after demo.launch.py is already running):
    ros2 launch jaguar_full damped_servo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("jaguar_robot_full", package_name="jaguar_full")
        .to_moveit_configs()
    )

    params_path = os.path.join(
        get_package_share_directory("jaguar_full"),
        "config",
        "damped_servo_params.yaml",
    )

    servo_node = Node(
        package="jaguar_teleop",
        executable="damped_servo",
        name="servo_node",
        parameters=[
            {"use_sim_time": False},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            params_path,
        ],
        output="screen",
    )

    # Auto-start the servo after 3 seconds (same as original servo.launch.py)
    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger", "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([servo_node, start_servo])
