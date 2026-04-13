#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

source ~/TMR2026_Rescue/shared/gui_ws/install/setup.bash
source ~/TMR2026_Rescue/jaguar/arm/install/setup.bash

ros2 launch gui gui.launch.py &
PID1=$!

wait $PID1