# Jaguar Arm 2026

6-DOF robotic arm controlled through MoveIt 2, with real-time joystick teleoperation, ODrive brushless motor control over CAN, and PWM servo control over serial via ESP32.

---

## Hardware

| Joint | Actuator | Interface | Node/Pin |
|-------|----------|-----------|----------|
| Joint 1–3 | ODrive brushless | Ginkgo USB-CAN → CAN bus | Node IDs 0x10, 0x11, 0x12 |
| Joint 4 | JX CLS-12V7346 servo | ESP32 serial (JSON) | GPIO 22 |
| Joint 5 | RDS51160 160 kg·cm servo | ESP32 serial (JSON) | GPIO 21 |
| Joint 6 | JX CLS-12V7346 servo | ESP32 serial (JSON) | GPIO 19 |
| Gripper | JX CLS-12V7346 servo | ESP32 serial (JSON) | GPIO 18 |

---

## Stack

```
Xbox Controller ──→ /joy
                      │
                      ├──→ joystick_servo ──→ /servo_node/delta_twist_cmds
                      │                       /servo_node/delta_joint_cmds
                      │                              │
                      │                         servo_node (MoveIt Servo)
                      │                              │
                      │                     /jaguar_arm_controller/joint_trajectory
                      │                              │
                      │                       JointTrajectoryController
                      │                              │
                      │                  ┌───────────┴────────────┐
                      │                  │                         │
                      │             /joint_states             /joint_states
                      │                  │                         │
                      │         ginkgo_odrive_bridge        serial_bridge ←─── RT/LT (gripper)
                      │                  │                         │
                      │             Ginkgo USB-CAN           UART /dev/ttyUSBx
                      │                  │                         │
                      │            ODrive motors              ESP32 firmware
                      │            (Joint 1–3)            (Joint 4, 6, Gripper)
                      │
                      └──→ serial_bridge (RT/LT triggers → gripper)
```

---

## Launch sequence

```bash
# 1. MoveIt  (move_group, robot_state_publisher, RViz)
ros2 launch jaguar_full demo.launch.py

# 2. MoveIt Servo
ros2 launch jaguar_full servo.launch.py

# 3. Brushless motors  (ODrive via Ginkgo USB-CAN)
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py verbose:=true verbose_period_s:=0.5

# 4. Joystick control  (joy driver + joystick_servo + serial bridge → ESP32)
ros2 launch jaguar_teleop joystick.launch.py
```

If the ESP32 is on a different port:
```bash
ros2 launch jaguar_teleop joystick.launch.py port:=/dev/ttyUSB1
```

> Find the correct serial port with `ls /dev/ttyUSB*`.

---

## Joystick mapping (Xbox Series X)

### CARTESIAN mode (default)
| Input | Effect |
|-------|--------|
| Left stick Y | Forward / Back (+X) |
| Left stick X | Strafe (+Y) |
| Right stick Y | Up / Down (+Z) |
| Right stick X | Yaw |
| LB (hold) + Left X | Roll |
| RB (hold) + Right Y | Pitch |
| Back | Stop (zero velocity) |
| Y | Pause servo / resume (hand off to RViz) |
| Guide (hold) | Emergency stop |
| Start | Switch to JOINT mode |

### JOINT mode
| Input | Effect |
|-------|--------|
| D-pad Up/Down | Select joint (cycles through Joint 1–6) |
| Left stick Y | Jog selected joint |
| Start | Switch to CARTESIAN mode |

### Gripper (always active, via serial_bridge)
| Input | Effect |
|-------|--------|
| RT | Open gripper |
| LT | Close gripper |

---

## CAN bus — how it works

### Physical layer

The **Ginkgo USB-CAN** adapter connects to the PC via USB and exposes a CAN bus at 500 kbps. The ODrives for Joints 1–3 are nodes on this bus.

```
PC (USB) ──→ Ginkgo USB-CAN ──→ CAN H/L wires ──→ ODrive node 0x10
                                                 └──→ ODrive node 0x11
                                                 └──→ ODrive node 0x12
```

### COB-ID encoding (ODrive CANopen subset)

ODrive uses a CANopen-style 11-bit message ID called a **COB-ID**:

```
COB-ID (11 bits) = node_id (6 bits) << 5 | command_id (5 bits)
```

For example, Joint 1 (node_id = 0x10 = 16):

| Command | command_id | COB-ID calculation | COB-ID (hex) |
|---------|------------|-------------------|--------------|
| Set Axis State | 0x007 | (0x10 << 5) \| 0x007 | **0x207** |
| Get Encoder Estimates | 0x009 | (0x10 << 5) \| 0x009 | **0x209** |
| Set Input Position | 0x00C | (0x10 << 5) \| 0x00C | **0x20C** |

### Startup sequence

On node startup, the bridge:

1. **Sends `AXIS_STATE_CLOSED_LOOP_CONTROL` (state = 8)** to each ODrive:
   ```
   Frame ID : COB-ID (e.g. 0x207 for Joint 1)
   Payload  : struct.pack("<I", 8)   → 4 bytes, little-endian uint32
   ```

2. **Reads encoder estimates** via an RTR (Remote Transmission Request) frame:
   ```
   Frame ID  : COB-ID for GET_ENCODER_ESTIMATES (e.g. 0x209)
   RemoteFlag: 1  (requests the ODrive to reply)
   DataLen   : 8
   ```
   The ODrive replies with an 8-byte frame:
   ```python
   position_turns, velocity_turns = struct.unpack("<ff", payload)
   # two 32-bit floats, little-endian
   ```
   This position is stored as `encoder_zero_turns` — the physical reference point for that axis. All subsequent commands are offset by this value so the arm stays in place at startup.

### Position commands (sent at 20 Hz)

Every 50 ms the bridge sends one `SET_INPUT_POS` frame per joint:

```
Frame ID : COB-ID for SET_INPUT_POS (e.g. 0x20C for Joint 1)
Payload  : struct.pack("<fhh", position_turns, 0, 0)
           └─ 4 bytes: float  — target position in motor turns
           └─ 2 bytes: int16  — velocity feedforward (unused, 0)
           └─ 2 bytes: int16  — torque feedforward   (unused, 0)
```

### Radians → motor turns conversion

```python
turns = (joint_rad / (2π)) × gear_ratio × direction
```

With the current config (gear_ratio = 48, direction = −1):

| MoveIt angle | Motor turns commanded |
|---|---|
| 0 rad | encoder_zero_turns + 0.0 |
| 1 rad | encoder_zero_turns − 7.64 |
| −1 rad | encoder_zero_turns + 7.64 |

### Shutdown

On `Ctrl+C`, the bridge sends `AXIS_STATE_IDLE` (state = 1) to each node, de-energizing the motors.

---

## Serial bridge — ESP32 (Joint 4, Joint 6, Gripper)

`serial_bridge.py` reads `/joint_states` and `/joy`, then sends a newline-terminated JSON string over UART at 30 Hz (matched to MoveIt Servo's publish rate):

```json
{"j4": 1.2345, "j5": -0.3491, "j6": -0.7854, "gr": 145.0}
```

- `j4`, `j5`, `j6` — MoveIt joint angles in **radians** (converted to servo degrees on the ESP32)
- `gr` — gripper position in **servo degrees**, computed on the ROS 2 side from RT/LT trigger values

If the serial port disconnects, the bridge automatically retries every 3 seconds.

### Calibration (`firmware/servo_sweep/servo_config.h`)

All servo-specific mapping lives here. Edit this file when re-calibrating.

```cpp
// Joint 4: MoveIt [-π/2, +π/2] → Servo [5°, 170°]
J4_MOVEIT_MIN_RAD = -1.5708    J4_SERVO_MIN_DEG = 5
J4_MOVEIT_MAX_RAD = +1.5708    J4_SERVO_MAX_DEG = 170

// Joint 6: MoveIt [-π/2, +π/2] → Servo [0°, 180°]
J6_MOVEIT_MIN_RAD = -1.5708    J6_SERVO_MIN_DEG = 0
J6_MOVEIT_MAX_RAD = +1.5708    J6_SERVO_MAX_DEG = 180

// Gripper
GR_SERVO_MIN_DEG = 100   (closed)
GR_SERVO_MAX_DEG = 180   (open)
```

Conversion on the ESP32:
```cpp
float t = (rad - MOVEIT_MIN) / (MOVEIT_MAX - MOVEIT_MIN);
float deg = SERVO_MIN + t * (SERVO_MAX - SERVO_MIN);
int pulse_us = 500 + (deg / 180.0) * 2000;   // → 500–2500 µs
```

---

## Firmware

### `firmware/servo_sweep/` — Joint 4, 5, 6, Gripper

- Input: JSON over serial at 115200 baud → `{"j4": rad, "j5": rad, "j6": rad, "gr": deg}`
- All conversions (radians → pulse µs) done on the ESP32 via `servo_config.h`
- Libraries: `ESP32Servo`, `ArduinoJson`

| Joint | Servo | PWM | Pulse range |
|-------|-------|-----|-------------|
| Joint 4 | JX CLS-12V7346 (46 kg·cm, 12V) | 330 Hz | 500–2500 µs |
| Joint 5 | RDS51160 (160 kg·cm, 24V) | 330 Hz | 800–2100 µs |
| Joint 6 | JX CLS-12V7346 (46 kg·cm, 12V) | 330 Hz | 500–2500 µs |
| Gripper | JX CLS-12V7346 (46 kg·cm, 12V) | 330 Hz | 800–2100 µs |

### `firmware/joint5_servo/` — Joint 5 standalone test sketch

Send any angle (0–180) over serial to command J5 directly. Used for calibration.

---

## Build

```bash
cd ~/Projects/Robotics/Dicerox_arm_2026

# Python non-ROS dependencies
pip install -r requirements.txt

# ROS 2 packages
colcon build
source install/setup.bash
```

Arduino libraries required (Arduino Library Manager):
- `ESP32Servo` by Kevin Harrington
- `ArduinoJson` by Benoit Blanchon

---

## Package overview

| Package | Type | Role |
|---------|------|------|
| `jaguar_robot_full_description` | CMake | URDF, meshes, RViz config |
| `jaguar_full` | CMake | MoveIt 2 config, launch files, controller YAML |
| `ginkgo_odrive_bridge` | Python | Joint 1–3 CAN bridge (ODrive via Ginkgo adapter) |
| `jaguar_teleop` | Python | Joystick, keyboard, GUI, serial bridge |
