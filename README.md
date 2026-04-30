# TMR2026_Rescue

Monorepo for the TMR 2026 RoboCup Rescue robot software. It collects the ROS 2
arm stacks, Dicerox navigation and traction bring-up files, shared operator GUI,
ESP32 firmware, vision experiments, and CAN/ODrive bench tools used across the
robots.

This repository is not one single build target. Treat it as a collection of
focused workspaces and firmware projects. Build or flash from the subsystem
folder you are working on.

## Top-Level Map

| Path | Purpose |
|------|---------|
| [`jaguar/arm`](./jaguar/arm) | Jaguar 6-DOF arm workspace: MoveIt 2 config, robot description, joystick/keyboard teleop, serial servo bridge, and Ginkgo USB-CAN to ODrive bridge. |
| [`dicerox/arm/bldc_can_tools`](./dicerox/arm/bldc_can_tools) | Python/ROS 2 BLDC CAN utilities for LKTech, ZE300-style drivers, Ginkgo CAN access, sniffing, and ESP32 MCP2515 arm controller examples. |
| [`dicerox/traction`](./dicerox/traction) | ODrive traction bring-up notes plus ESP32 sketches for CAN velocity control and RC-style PWM testing. |
| [`dicerox/flippers`](./dicerox/flippers) | ESP32 Arduino sketches for VESC CAN feedback parsing and simple flipper/control tests. |
| [`nav/Dicerox_mapping`](./nav/Dicerox_mapping) | ROS 2 SLAM workspace for Dicerox mapping with ZED2 odometry, Slamtec A1 lidar, slam_toolbox, RViz, bag recording, and map saving. |
| [`shared/gui_ws`](./shared/gui_ws) | ROS 2 Humble GUI workspace: Qt6 teleop GUI, camera/thermal views, dashboard, digital twin, audio, YOLO support, and ESP32 UART bridge. |
| [`shared/firmware`](./shared/firmware) | PlatformIO ESP32 rescue robot controller for RC input, locomotion, sensors, telemetry, CAN, and arm command relay. |
| [`shared/ginkgo_tools`](./shared/ginkgo_tools) | Standalone Python Ginkgo USB-CAN and ODrive motor test scripts, including a PyQt6 bench GUI. |
| [`shared/vision`](./shared/vision) | HazMat YOLO detection script and trained model artifacts. |
| [`docs`](./docs) | Miscellaneous repository notes and test documents. |

## Main System Ideas

- ROS 2 Humble is the main robot middleware for arm control, mapping, GUI, and
  serial/CAN bridges.
- ESP32 firmware handles low-level robot IO: RC receiver input, PWM/CAN motor
  control, sensors, e-stop, binary UART telemetry, and command relay.
- ODrive and other BLDC motor controllers are tested through both Ginkgo USB-CAN
  tools and ESP32 MCP2515/TWAI sketches.
- The shared GUI consumes ROS 2 topics from the ESP32 bridge and camera bridges,
  and publishes operator commands back to the robot.
- Mapping uses ZED visual-inertial odometry as the odom source and a Slamtec A1
  scan topic for 2D SLAM.

## Quick Environment Notes

Common assumptions found in the repo:

- Ubuntu 22.04 / ROS 2 Humble for ROS workspaces.
- PlatformIO for the shared ESP32 firmware.
- Arduino IDE or Arduino CLI for the standalone `.ino` bring-up sketches.
- Python 3.10+ for direct Python tooling.
- Ginkgo USB-CAN vendor libraries are kept inside the arm bridge folders and are
  reused by several tools.

Root Python requirements are intentionally small:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

That installs the root-level helper dependencies currently listed in
[`requirements.txt`](./requirements.txt): PyQt6 and OpenCV. Subsystems may need
additional packages described in their own README files.

## Workspaces And Build Entry Points

### Jaguar arm

Primary docs: [`jaguar/arm/README.md`](./jaguar/arm/README.md) and
[`jaguar/arm/DLS_SERVO.md`](./jaguar/arm/DLS_SERVO.md).

Packages:

- `jaguar_robot_full_description`: URDF/Xacro, meshes, and RViz description.
- `jaguar_full`: MoveIt 2 config, controllers, Servo, DLS Servo, and launch files.
- `jaguar_teleop`: joystick/keyboard teleop, serial bridge, joint relay, DLS node.
- `ginkgo_odrive_bridge`: `/joint_states` to ODrive CAN bridge through Ginkgo USB-CAN.

Local firmware:

- [`jaguar/arm/firmware/servo_sweep`](./jaguar/arm/firmware/servo_sweep): ESP32
  serial-to-servo sketch for J4, J5, J6, and the gripper.
- [`jaguar/arm/firmware/joint5_servo`](./jaguar/arm/firmware/joint5_servo):
  standalone J5 calibration sketch.
- [`jaguar/arm/firmware/odrive_config_GIM8108-48_24V.txt`](./jaguar/arm/firmware/odrive_config_GIM8108-48_24V.txt):
  ODrive motor configuration notes.

Build:

```bash
cd jaguar/arm
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Typical launch sequence:

```bash
ros2 launch jaguar_full demo.launch.py
ros2 launch jaguar_full damped_servo.launch.py
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py verbose:=true verbose_period_s:=0.5
ros2 launch jaguar_teleop joystick.launch.py
```

Use `servo.launch.py` instead of `damped_servo.launch.py` if you want the stock
MoveIt Servo path.

### Dicerox arm CAN tools

Primary docs: [`dicerox/arm/bldc_can_tools/README.md`](./dicerox/arm/bldc_can_tools/README.md).

This is an `ament_python` package for:

- LKTech / MyActuator-style position commands.
- ZE300-style reads.
- Ginkgo CAN transport.
- CAN sniffing and health checks.
- ESP32 MCP2515 examples, including a mixed 6-joint Dicerox controller.

Build:

```bash
cd dicerox/arm
source /opt/ros/humble/setup.bash
colcon build --packages-select bldc_can_tools --symlink-install
source install/setup.bash
```

Useful direct commands:

```bash
ros2 run bldc_can_tools lktech_can_health_check
ros2 run bldc_can_tools lktech_cli_read_angle --help
ros2 run bldc_can_tools bldc_ze300_read --help
```

Many scripts can also be run directly with Python when testing on Windows or
outside a ROS shell, as described in the package README.

### Dicerox traction

Primary docs: [`dicerox/traction/README.md`](./dicerox/traction/README.md).

This folder holds ODrive v3.6 clone plus 6384 motor notes and small ESP32
sketches for:

- CAN telemetry reads through MCP2515.
- ODrive velocity commands over CANSimple.
- Single-channel and dual-axis RC-style PWM signal generation.
- Hall-sensor and sensorless bring-up notes.

Default CAN assumptions in the sketches are generally:

- ODrive node ID `0x02`
- CAN bitrate `1 Mbps`
- MCP2515 normal mode
- MCP2515 8 MHz oscillator unless changed in the sketch config

### Dicerox mapping

Primary docs: [`nav/Dicerox_mapping/README.md`](./nav/Dicerox_mapping/README.md).

Build:

```bash
cd nav/Dicerox_mapping
source /opt/ros/humble/setup.bash
colcon build --packages-select dicerox_mapping --symlink-install
source install/setup.bash
```

Robot-side run flow:

```bash
source setup_jetson.bash
./launch_zed.sh
ros2 launch sllidar_ros2 sllidar_a1_launch.py
ros2 launch dicerox_mapping mapping_robot.launch.py
```

Laptop visualization:

```bash
source setup_laptop.bash
ros2 launch dicerox_mapping mapping_viz.launch.py
```

Save a map:

```bash
ros2 launch dicerox_mapping save_map.launch.py
```

The mapping stack uses ZED odometry in `zed_camera_link`, a static transform to
the `laser` frame, and `slam_toolbox` to publish `map -> odom`.

### Shared GUI workspace

Primary docs: [`shared/gui_ws/README.md`](./shared/gui_ws/README.md).

Packages:

- `gui`: C++/Qt6 ROS 2 GUI with video, thermal overlay, dashboard, digital twin,
  speech processing hooks, keybind configuration, and PPM calibration.
- `esp32_bridge`: Python bridge from the ESP32 binary UART protocol to ROS 2
  topics, plus an audio node.

Build:

```bash
cd shared/gui_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the GUI and camera bridge:

```bash
ros2 launch gui gui.launch.py
```

Run the ESP32 bridge:

```bash
ros2 launch esp32_bridge esp32_bridge.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
```

Run the simulator for GUI development without the robot:

```bash
python3 sim_robot.py
```

The GUI workspace has additional system dependencies: Qt6, OpenCV with
GStreamer, ONNX Runtime, Vosk, PulseAudio, Assimp, and ZBar. Follow the
workspace README before building on a new machine.

### Shared ESP32 firmware

Primary docs: [`shared/firmware/README.md`](./shared/firmware/README.md).

This is the main PlatformIO firmware for a DOIT ESP32 DevKit V1. It handles:

- FlySky PPM receiver input.
- Keybind-controlled modes.
- Track and flipper locomotion.
- ESP32 TWAI CAN communication.
- QMC5883L, BNO055, MLX90640, and MQ2 sensors.
- Binary UART telemetry and commands at 921600 baud.
- Arm joint command relay.

Select the target robot in [`shared/firmware/include/config.h`](./shared/firmware/include/config.h):

```cpp
#define ROBOT_MAIN
// #define ROBOT_SECONDARY
```

Build and flash:

```bash
cd shared/firmware
pio run
pio run -t upload
pio device monitor
```

### Shared Ginkgo ODrive tools

Primary docs: [`shared/ginkgo_tools/README.md`](./shared/ginkgo_tools/README.md).

These are direct Python scripts for bench testing ODrive motors through a Ginkgo
USB-CAN adapter.

```bash
python3 shared/ginkgo_tools/read_encoder_once.py --node-id 0x10
python3 shared/ginkgo_tools/telemetry_monitor.py --node-id 0x10 --interval 0.5
python3 shared/ginkgo_tools/position_step_test.py --node-id 0x10 0.10 -0.10 0.00
python3 shared/ginkgo_tools/ginkgo_motor_tester.py
```

Defaults are channel `0`, bitrate `500 kbps`, and node ID `0x10`.

### Vision

[`shared/vision/detect.py`](./shared/vision/detect.py) runs a HazMat YOLO model
against a webcam, image, folder, or video file.

```bash
pip install ultralytics
python3 shared/vision/detect.py
python3 shared/vision/detect.py path/to/image_or_video
```

The current script points at
`shared/vision/runs/hazmat_yolo11l/weights/best.pt`. An ONNX export is also
present for GUI or CPU-oriented workflows.

## Hardware And Protocol Cheat Sheet

| Area | Main hardware/protocol |
|------|------------------------|
| Jaguar arm J1-J3 | ODrive over CANSimple through Ginkgo USB-CAN, node IDs `0x10`, `0x11`, `0x12`, 500 kbps in the ROS bridge. |
| Jaguar arm J4-J6 + gripper | ESP32 serial JSON bridge to PWM servos. |
| Dicerox mixed arm examples | ODrive CANSimple, ZE300 positional protocol, and LKTech/MyActuator-style CAN on one ESP32 MCP2515 bus. |
| Dicerox traction | ODrive v3.6 clone bring-up, CANSimple velocity control, and RC-style PWM input tests. |
| Shared robot controller | ESP32 UART protocol frames beginning with `0xAA 0x55`, plus TWAI CAN for VESC/ODrive traffic. |
| Mapping | ZED2 VIO, Slamtec A1 laser scan, `slam_toolbox`, static `zed_camera_link -> laser` transform. |
| Operator GUI | ROS 2 topics from `esp32_bridge`, camera bridge, dashboard, command publishers, and e-stop. |

## Safety Notes

- Confirm CAN bitrate, node IDs, gear ratios, and direction signs before moving
  real hardware.
- Start motor tests with small position or velocity commands.
- Keep e-stop wiring and software e-stop topics verified before full-system runs.
- Re-check ODrive startup offsets after power cycles or manual repositioning.
- Use the subsystem README closest to the hardware before flashing or commanding
  motors.

## Repository Notes

- Some subfolders contain generated ROS build/install/log directories or large
  binary assets such as STL meshes, trained model weights, and vendor USB-CAN
  libraries.
- Root `requirements.txt` is not a universal dependency list. Each workspace
  documents its own apt, pip, Arduino, or PlatformIO requirements.
- Prefer subsystem-local READMEs for exact pinouts, launch arguments, calibration
  values, and bring-up sequences.
