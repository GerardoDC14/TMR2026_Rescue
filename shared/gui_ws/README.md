# gui_ws — RoboCup Rescue Teleop GUI + ESP32 Bridge

ROS 2 Humble workspace with two packages:

| Package | Language | Purpose |
|---------|----------|---------|
| **gui** | C++ / Qt6 | Teleop GUI: live cameras, 3-D URDF viewer, sensor dashboard, YOLO detection, speech transcription, keybind & PPM calibration dialogs. |
| **esp32_bridge** | Python | Translates the ESP32's binary UART protocol into ROS 2 topics and vice-versa. Also contains an audio capture node. |

---

## Architecture

```
FlySky RC ─── PPM ──> ESP32 ──── UART 921600 ──> esp32_bridge ──> ROS 2 topics
                                                                       |
                                                                    GUI node
                                                                  (Qt6 + ROS)
```

The ESP32 bridge publishes telemetry, sensor data, and raw PPM values, and forwards keybind configuration, arm joint commands, e-stop, calibration, and sensor enable commands back to the ESP32.

---

## ROS 2 topic reference

### Published by esp32_bridge (ESP32 -> PC)

| Topic | Type | Content |
|-------|------|---------|
| `/robot/telemetry` | Float32MultiArray | [speed_l_rpm, speed_r_rpm, flipper_deg, uptime_s] |
| `/robot/mode` | String | INIT / STANDBY / NORMAL / ARM / ESTOP / FLIPPER |
| `/robot/flags` | UInt8 | Bitmask: ppm_ok, sensors, can_ok, estop |
| `/robot/ppm` | Int16MultiArray | Raw PPM us [ch1..ch6] |
| `/robot/status` | DiagnosticArray | Full diagnostic status |
| `/encoders/tracks` | Vector3 | x=left_rpm, y=right_rpm |
| `/encoders/flipper` | Float32MultiArray | [fl, fr, rl, rr] degrees |
| `/sensors/imu` | Imu | BNO055 orientation + accel + gyro |
| `/sensors/mag` | MagneticField | QMC5883L XYZ field |
| `/sensors/thermal` | Image | 32x24 float32 (C) |
| `/sensors/gas` | Float32 | Rs/Ro ratio |
| `/motors/vesc_status` | Float32MultiArray | Per-VESC telemetry |
| `/motors/main_status` | Float32MultiArray | Track + flipper duties |

### Subscribed by esp32_bridge (PC -> ESP32)

| Topic | Type | Content |
|-------|------|---------|
| `/robot/estop` | Bool | True = e-stop, False = clear |
| `/arm/joint_command` | Float32MultiArray | 6 joint angles in degrees |
| `/sensors/enable_mask` | UInt8 | Bitmask: mag, thermal, gas, imu |
| `/robot/keybind` | UInt8MultiArray | 15 bytes (3 modes x 5 channels) |
| `/robot/ppm_calib` | UInt16MultiArray | 18 values (6ch x min/neutral/max) |

### Published by GUI

| Topic | Type | Content |
|-------|------|---------|
| `/robot/keybind` | UInt8MultiArray | Keybind configuration (reliable, transient-local) |
| `/robot/ppm_calib` | UInt16MultiArray | PPM calibration (reliable, transient-local) |
| `/robot/estop` | Bool | E-stop state, republished every 100 ms |
| `/sensors/enable_mask` | UInt8 | Sensor enable bitmask |
| `/audio_enable` | Bool | Audio capture toggle |

---

## GUI panels

| Panel | Location | Features |
|-------|----------|----------|
| **Video** | Left 2/3 | Up to 3 camera feeds + thermal overlay, YOLO filter, click to enlarge |
| **Digital Twin** | Top-right | 3-D URDF viewer (OpenGL) |
| **Dashboard** | Bottom-right | Sensor readouts, connection LED, e-stop button, audio toggle |
| **Odometry** | Bottom-right (tab) | Track speeds, flipper angles, VESC/motor status |
| **Settings** | Dialog | Robot type, keybind editor, PPM calibration, thermal colormap, detection labels |

---

## Contents

- [System dependencies](#1-system-dependencies)
- [Manual installs (ONNX Runtime & Vosk)](#2-manual-installs)
- [Models & assets](#3-models--assets)
- [Python dependencies (sim / bridge)](#4-python-dependencies)
- [Build](#5-build)
- [Run](#6-run)

---

## 1. System dependencies

```bash
sudo apt update

# ROS 2 packages
sudo apt install -y \
  ros-humble-urdf \
  ros-humble-cv-bridge \
  ros-humble-launch-ros

# Qt 6
sudo apt install -y \
  qt6-base-dev \
  qt6-multimedia-dev \
  libqt6opengl6-dev

# Audio
sudo apt install -y libpulse-dev

# Computer vision / mesh loading
sudo apt install -y \
  libopencv-dev \
  python3-opencv \
  libassimp-dev \
  libzbar-dev

# GStreamer (needed by gst_bridge for live camera streams)
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
```

> **GStreamer + OpenCV check** — the `gst_bridge` node needs OpenCV built with
> GStreamer support. The apt `python3-opencv` package normally satisfies this.
> Verify with:
> ```bash
> python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer
> ```
> The line must say `YES`. If it says `NO`, reinstall with
> `sudo apt install python3-opencv` (or rebuild OpenCV from source with
> `-D WITH_GSTREAMER=ON`).

---

## 2. Manual installs

These libraries are not in the Ubuntu 22.04 apt repos and must be installed manually.

### ONNX Runtime 1.20.1 (YOLO inference)

```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
tar xzf onnxruntime-linux-x64-1.20.1.tgz

sudo mkdir -p /usr/local/include/onnxruntime
sudo cp  onnxruntime-linux-x64-1.20.1/include/* /usr/local/include/onnxruntime/
sudo cp  onnxruntime-linux-x64-1.20.1/lib/*.so* /usr/local/lib/
sudo ldconfig

rm -rf onnxruntime-linux-x64-1.20.1*
```

### Vosk C API (speech recognition)

```bash
wget https://github.com/alphacep/vosk-api/releases/download/v0.3.45/vosk-linux-x86_64-0.3.45.zip
unzip vosk-linux-x86_64-0.3.45.zip

sudo cp vosk-linux-x86_64-0.3.45/libvosk.so  /usr/local/lib/
sudo cp vosk-linux-x86_64-0.3.45/vosk_api.h  /usr/local/include/
sudo ldconfig

rm -rf vosk-linux-x86_64-0.3.45*
```

---

## 3. Models & assets

### Speech — Vosk model

Download a model from <https://alphacephei.com/vosk/models>.
The **small English lgraph model** (`vosk-model-small-en-us-0.15`) is recommended for low latency.

Extract the zip so that its contents land directly in `src/gui/assets/audio/`:

```
src/gui/assets/audio/
  am/
  conf/
  graph/
  ivector/
  README
```

The directory structure above is what Vosk expects — it is passed the folder path, not any individual file.

### Vision — YOLO ONNX model

Place your trained model at:

```
src/gui/assets/vision/best.onnx
```

The model must be exported with **opset 12 or lower** (OpenCV 4.5.x limit):

```bash
# Ultralytics example
yolo export model=best.pt format=onnx opset=12
```

Class labels are read automatically from the ONNX metadata (`names` field), so no separate `.txt` file is needed.

---

## 4. Python dependencies

Only needed for the simulator (`sim_robot.py`) and the camera bridge (`gst_bridge`). The GUI itself is pure C++.

```bash
pip3 install numpy pyaudio
# cv_bridge and rclpy come from the ROS apt packages above
```

---

## 5. Build

```bash
cd ~/gui_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

If you have a robot description package (URDF + meshes) in a separate workspace, source it too so the 3-D viewer can resolve `package://` mesh URIs:

```bash
source ~/path/to/robot_ws/install/setup.bash
```

Add both `source` lines to `~/.bashrc` to avoid running them every session.

---

## 6. Run

### GUI + camera bridge (normal operation)

```bash
ros2 launch gui gui.launch.py
```

This starts both the Qt GUI and the `gst_bridge` node. The bridge waits for a `/config` message from the robot before opening any streams.

### GUI only

```bash
ros2 run gui gui
```

### Simulator (no real robot)

```bash
python3 ~/gui_ws/sim_robot.py
```

Publishes fake sensor data, joint states, a webcam image, and audio on the same topics the GUI subscribes to.

### ESP32 bridge

```bash
ros2 launch esp32_bridge esp32_bridge.launch.py
# Override serial port:
ros2 launch esp32_bridge esp32_bridge.launch.py serial_port:=/dev/ttyUSB1
```

Parameters: `serial_port` (default `/dev/ttyUSB0`), `baud_rate` (default `921600`), `audio_device` (default `-1`).

### Simulate camera config from the terminal

```bash
# 1 camera
ros2 topic pub --once /config std_msgs/msg/String \
  'data: "{\"camera_topics\": [\"/camera/front\"]}"' \
  --qos-durability transient_local --qos-reliability reliable

# 2 cameras
ros2 topic pub --once /config std_msgs/msg/String \
  'data: "{\"camera_topics\": [\"/camera/front\", \"/camera/rear\"]}"' \
  --qos-durability transient_local --qos-reliability reliable
```
