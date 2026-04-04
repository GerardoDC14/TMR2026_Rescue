# firmware — ESP32 Rescue Robot Controller

PlatformIO project for the DOIT ESP32 DevKit V1 that handles RC input, locomotion, sensor acquisition, and arm relay for the RoboCup Rescue robot.

---

## What it does

| Subsystem | Description |
|-----------|-------------|
| **RC receiver** | Decodes a 6-channel PPM signal from a FlySky FS-i6 receiver on GPIO34. |
| **Keybind system** | Ch5 (3-position lever) selects one of three mode rows. Each row maps the other 5 channels to functions (traction, flipper, arm axes, e-stop) configurable from the GUI. |
| **Locomotion** | Drives tracks and flippers — LEDC servo-PWM on ROBOT_MAIN, CAN (VESC) on ROBOT_SECONDARY. |
| **Arm relay** | In ARM mode the ESP32 stops the tracks and forwards joint-angle commands from the mini PC to ODrive motors over CAN. |
| **Sensors** | Reads QMC5883L magnetometer, BNO055 IMU, MLX90640 thermal camera, and MQ2 gas sensor over I2C. |
| **Protocol** | Binary UART at 921600 baud to the mini PC. Telemetry (including raw PPM) is sent at 50 Hz; the mini PC sends keybinds, calibration, arm joints, e-stop, and sensor enable commands back. |

---

## Hardware variants

Controlled by `#define ROBOT_MAIN` or `ROBOT_SECONDARY` in `include/config.h`.

| | ROBOT_MAIN (Jaguar) | ROBOT_SECONDARY (Dicerox) |
|---|---|---|
| Tracks | LEDC servo-PWM | VESC over CAN |
| Flippers | 1 joined (LEDC + PID) | 4 independent (VESC) |
| Arm joints via CAN | 3 (J1-J3, ODrive) | 6 (J1-J6, ODrive) |

---

## Architecture

Two ESP32 cores, five FreeRTOS tasks:

| Core | Task | Rate | Priority | Purpose |
|------|------|------|----------|---------|
| 0 | `commsTask` | 50 Hz | 4 | UART RX/TX, telemetry |
| 0 | `canTask` | 200 Hz | 4 | TWAI polling, VESC relay |
| 1 | `controlTask` | 50 Hz | 5 | State machine, keybind logic, motor output |
| 1 | `sensorTask` | varies | 2 | Mag/gas/IMU sampling |
| 1 | `thermalTask` | ~4 Hz | 1 | MLX90640 blocking reads |

---

## Binary protocol

Frame format: `[0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]`

CRC = XOR of TYPE, LEN_H, LEN_L, and all payload bytes.

### ESP32 -> PC

| Type | Name | Payload |
|------|------|---------|
| 0x01 | Telemetry | mode, flags, PPM[6], speeds, flipper angle, uptime |
| 0x02 | Thermal | 32x24 int16 pixels (C x10) |
| 0x03 | Magnetometer | XYZ int16 (uT x100) |
| 0x04 | Gas | Rs/Ro int16 (x100) |
| 0x05 | Status | mode, flags, sensor mask |
| 0x06 | IMU | euler, accel, gyro, calib |
| 0x07 | Encoder ext | 4 flipper angles (ROBOT_SECONDARY) |
| 0x08 | VESC status | per-motor telemetry |
| 0x09 | Motor main | track + flipper duties (ROBOT_MAIN) |

### PC -> ESP32

| Type | Name | Payload |
|------|------|---------|
| 0x10 | Arm joints | 6x int16 (deg x100) |
| 0x11 | Sensor enable | 1-byte bitmask |
| 0x12 | E-stop | (empty) |
| 0x13 | E-stop clear | (empty) |
| 0x14 | Keybind | 15 bytes (3 modes x 5 channels) |
| 0x15 | PPM calibration | 6 channels x (min, neutral, max) uint16 |

---

## Keybind system

The Ch5 3-position lever selects a row (0/1/2) from the keybind table. Each row assigns a `ChannelFunction` to 5 channel slots (Ch1, Ch2, Ch3, Ch4, Ch6):

| Value | Function |
|-------|----------|
| 0 | None |
| 1 | Traction Fwd |
| 2 | Traction Turn |
| 3 | Flippers (all) |
| 4-7 | Flipper FL/FR/RL/RR |
| 8 | Arm (generic, legacy) |
| 9 | E-Stop |
| 10 | Arm X (forward/back) |
| 11 | Arm Y (lateral) |
| 12 | Arm Z (up/down) |
| 13 | Arm Pitch |
| 14 | Arm Yaw |

Default mode 2 (Ch5 high): `ARM_Y, ARM_X, ARM_Z, ARM_YAW, ARM_PITCH`.

The table is configurable at runtime via the GUI keybind dialog (sent as MSG_KEYBIND).

---

## Project structure

```
include/
  config.h          Pin assignments, constants, protocol IDs
  robot_types.h     Enums, structs, protocol payloads

lib/
  RC/               PPM receiver decoding (ISR-based)
  Control/          State machine, keybind application
  Locomotion/       Motor output (PWM / CAN)
  CANInterface/     TWAI driver, ODrive + VESC commands
  Comms/            Binary UART protocol TX/RX
  Encoders/         Quadrature encoder reading (PCNT)
  Sensors/          I2C sensor drivers

src/
  main.cpp          setup(), FreeRTOS task creation
```

---

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```bash
# Select robot variant in include/config.h:
#   #define ROBOT_MAIN        (Jaguar)
#   #define ROBOT_SECONDARY   (Dicerox)

# Build
pio run

# Flash
pio run -t upload

# Serial monitor
pio device monitor
```
