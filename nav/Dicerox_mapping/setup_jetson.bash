#!/usr/bin/env bash
# Source on the JETSON before launching.

MAPPING_WS=~/Projects/Robotics_2026/mapping

source /opt/ros/humble/setup.bash
source "$MAPPING_WS/install/setup.bash"

export ROS_DOMAIN_ID=42

echo "========================================"
echo " JETSON"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo " Mapping WS    = $MAPPING_WS"
echo "========================================"
echo ""
echo "Run in order (each terminal must source this file first):"
echo "  1. ./launch_zed.sh"
echo "  2. ros2 launch sllidar_ros2 sllidar_a1_launch.py"
echo "  3. ros2 launch dicerox_mapping mapping_robot.launch.py"
echo ""
echo "Save map:"
echo "  ros2 launch dicerox_mapping save_map.launch.py"
echo ""
echo "On the LAPTOP:"
echo "  source setup_laptop.bash"
echo "  ros2 launch dicerox_mapping mapping_viz.launch.py"
