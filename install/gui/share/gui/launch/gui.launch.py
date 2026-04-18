from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gui",
            executable="gui",
            name="gui",
            output="screen",
        ),
        Node(
            package="gui",
            executable="gst_bridge",
            name="gst_bridge",
            output="screen",
        ),
    ])
