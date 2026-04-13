#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source ~/TMR2026_Rescue/jaguar/arm/install/setup.bash

echo "Starting main launch..."
ros2 launch jaguar_full demo.launch.py &
PID1=$!

until ros2 control list_controllers | grep -q "jaguar_arm_controller.*active"; do
    sleep 0.5
done

sleep 15

ros2 launch jaguar_full damped_servo.launch.py &
PID2=$!

wait $PID1 $PID2