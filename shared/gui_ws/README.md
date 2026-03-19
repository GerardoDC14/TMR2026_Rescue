# gui_ws — RoboCup Rescue Teleop GUI

Qt6/ROS 2 Humble GUI for robot teleoperation. Displays live camera feeds, a 3-D URDF viewer, sensor dashboard, speech transcription, and a YOLO-based object detection filter.

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
