#!/usr/bin/env bash
# Source on the LAPTOP before launching.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

export ROS_DOMAIN_ID=42

echo "========================================"
echo " LAPTOP"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "========================================"
echo ""
echo "Visualize map from Jetson:"
echo "  ros2 launch dicerox_mapping mapping_viz.launch.py"
echo ""
echo "Save map:"
echo "  ros2 launch dicerox_mapping save_map.launch.py"
