# firmware — ESP32 Rescue Robot Controller

PlatformIO project for the DOIT ESP32 DevKit V1 that handles RC input, locomotion, sensor acquisition, and arm relay for the RoboCup Rescue robot.

---

## What it does

| Subsystem | Description |
|-----------|-------------|
| **RC receiver** | Decodes a 6-channel PPM signal from a FlySky FS-i6 receiver on GPIO34. |
| **Keybind system** | Ch5 (3-position lever) selects one of three mode rows. Each row maps the other 5 channels to functions (traction, flipper, arm axes, e-stop) configurable from the GUI. |
| **Locomotion** | Drives tracks and flippers — LEDC servo-PWM on ROBOT_MAIN, CAN (VESC) on ROBOT_SECONDARY. |
| **Flipper PID** | All flippers use closed-loop PID position control with shortest-path angular error. Encoder angles wrap [0, 360). ROBOT_MAIN: single joined flipper via PWM. ROBOT_SECONDARY: four independent flippers via VESC current commands. |
| **Arm relay** | In ARM mode the ESP32 stops the tracks and forwards joint-angle commands from the mini PC to the arm motors over CAN. |
| **CAN motors** | ODrive (J1-J3, both robots), ZE300 (J4, ROBOT_SECONDARY), LKTech (J5-J6, ROBOT_SECONDARY), VESC (tracks + flippers, ROBOT_SECONDARY). See CAN section below. |
| **Sensors** | Reads QMC5883L magnetometer, BNO055 IMU, MLX90640 thermal camera, and MQ2 gas sensor over I2C. |
| **Protocol** | Binary UART at 921600 baud to the mini PC. Telemetry (including raw PPM) is sent at 50 Hz; the mini PC sends keybinds, calibration, arm joints, e-stop, and sensor enable commands back. |

---

## Hardware variants

Controlled by `#define ROBOT_MAIN` or `ROBOT_SECONDARY` in `include/config.h`.

| | ROBOT_MAIN (Jaguar) | ROBOT_SECONDARY (Dicerox) |
|---|---|---|
| Tracks | LEDC servo-PWM | VESC over CAN |
| Flippers | 1 joined (LEDC + PID) | 4 independent (VESC + PID) |
| Flipper encoders | 1 PCNT unit | 4 PCNT units |
| Arm J1-J3 | ODrive over CAN | ODrive over CAN |
| Arm J4 | Servo (separate ESP32, not this project) | ZE300 over CAN |
| Arm J5-J6 | Servos (separate ESP32, not this project) | LKTech over CAN |

All CAN traffic runs on a single **1 Mbps TWAI bus** — every controller (ODrive, VESC, LKTech, ZE300) must be pre-configured to 1 Mbps before flashing.

---

## Architecture

Two ESP32 cores, five FreeRTOS tasks:

| Core | Task | Rate | Priority | Purpose |
|------|------|------|----------|---------|
| 0 | `commsTask` | 50 Hz | 4 | UART RX/TX, telemetry |
| 0 | `canTask` | 200 Hz | 4 | TWAI polling, ODrive telemetry RTR, VESC relay |
| 1 | `controlTask` | 50 Hz | 5 | State machine, keybind logic, flipper PID, motor output |
| 1 | `sensorTask` | varies | 2 | Mag/gas/IMU sampling |
| 1 | `thermalTask` | ~4 Hz | 1 | MLX90640 blocking reads |

---

## CAN bus

All motors share a single **1 Mbps TWAI (CAN 2.0) bus** via an SN65HVD230 transceiver. Every controller on the bus (ODrive, VESC, LKTech, ZE300) must be pre-configured to 1 Mbps. Default bitrates: ODrive = 250 kbps (set `odrv.can.config.baud_rate = 1000000`, save, reboot), VESC = 500 kbps (VESC Tool → App Settings → General → CAN Baud Rate), LKTech + ZE300 already default to 1 Mbps.

### ODrive (arm J1-J3, both robots)

- Standard 11-bit frames, COB-ID = `(node_id << 5) | cmd_id`
- `SET_INPUT_POS` (0x0C): float32 LE turns + int16 LE vel_ff + int16 LE torque_ff
- `SET_AXIS_STATE` (0x07): uint32 LE state (8 = closed-loop)
- Encoder zero captured at startup via RTR to `GET_ENCODER_ESTIMATES` (0x09)
- Runtime telemetry requested via round-robin RTR at ~16 Hz per reading:
  - `GET_ENCODER_ESTIMATES` (0x09): position + velocity
  - `GET_IQ` (0x14): Iq setpoint + measured
  - `GET_TEMPERATURE` (0x15): FET + motor temp
  - `GET_BUS_VOLTAGE_CURRENT` (0x17): bus voltage + current
- Telemetry forwarded to mini PC as `MSG_ODRIVE_STATUS` (0x0A)
- Node IDs: J1=0x10, J2=0x11, J3=0x12 (confirmed)

### VESC (tracks + flippers, ROBOT_SECONDARY only)

- Extended 29-bit frames, ID = `(cmd << 8) | vesc_id`
- `SET_CURRENT` (cmd 1): int32 BE, current_A × 1000
- Status parsed from broadcast frames:
  - Status 1 (cmd 9): eRPM, current, duty cycle
  - Status 4 (cmd 16): FET temp, motor temp
  - Status 5 (cmd 27): input voltage
- Telemetry forwarded to mini PC as `MSG_VESC_STATUS` (0x08)

### LKTech (arm J5-J6, ROBOT_SECONDARY only)

- Standard 11-bit frames, ID = `0x140 + motor_id` (requests and replies share the same ID)
- Command byte is `data[0]`, 8-byte payloads
- `MOTOR_ON` (0x88) sent at startup
- `READ_MULTI_LOOP_ANGLE` (0x92) sent once at startup — reply parses a signed 56-bit LE centideg value from `data[1..7]`; stored as `lktech_zero_offset` so boot pose becomes software zero (analogous to ODrive zero capture)
- `MULTI_LOOP_ANGLE_CONTROL_2` (0xA4): `{0xA4, 0x00, spd_lo, spd_hi, cdeg[0..3]}` — target in centidegrees + max speed in dps. Runtime commands are fire-and-forget
- Telemetry parsed from the A4 command acknowledge (same arbitration ID, same 8-byte layout as `READ_STATE_2`): temp at `data[1]`, Iq (i16 LE /100 A) at `data[2..3]`, speed (i16 LE dps) at `data[4..5]`, angle (i16 LE deg) at `data[6..7]`
- Telemetry forwarded to mini PC as `MSG_LKTECH_STATUS` (0x0B)
- Motor IDs: J5=14, J6=15 (confirmed from `dicerox_mixed_motor_config.h`); gear ratios 10.0, verify directions on bench

### ZE300 (arm J4, ROBOT_SECONDARY only)

- Standard 11-bit frames, tagged request ID = `0x100 | device_id`, reply ID = `device_id` (different IDs — watch out)
- Variable DLC (1 / 5 / 7 bytes depending on command)
- Position units are **encoder counts at 16384 counts/rev** (1:1 output-to-command ratio — the driver internally handles the 1:8 gearbox)
- Startup sequence (blocking, once in `begin()`):
  1. `SET_POSITION_MAX_SPEED` (0xB2) — 5-byte TX: `{0xB2, i32 LE centi-rpm}`, wait for 5-byte reply
  2. `READ_ABSOLUTE_ANGLES` (0xA3) — 1-byte TX: `{0xA3}`, wait for 7-byte reply; parse multi-turn counts from `data[3..6]` (i32 LE), store as `ze300_zero_offset_counts`
- Runtime commands (fire-and-forget):
  - `ABSOLUTE_POSITION` (0xC2) — 5-byte TX: `{0xC2, i32 LE motor_counts}` where `motor_counts = zero_offset + deg * 16384 / 360`
- Disable output: `DISABLE_OUTPUT` (0xCF), 1-byte
- Telemetry parsed passively from C2 command replies (position) + active low-rate poll of `READ_REALTIME_STATE` (0xA4, ~5 Hz): temp at `data[1]`, Iq (i16 LE /1000 A) at `data[2..3]`, speed (i16 LE rpm /100) at `data[4..5]`, single-turn counts (u16 LE) at `data[6..7]`
- Telemetry forwarded to mini PC as `MSG_ZE300_STATUS` (0x0C)
- Device ID: J4=1 (confirmed); `ZE_CMD_*` constants and 0xA4 realtime layout extrapolated from `ze300_protocol.py` and `esp32_mcp2515_ze300_zero_offset_controller.ino`

---

## Flipper control

All flippers have full 360 rotation with quadrature encoders read via PCNT.

- **Angle wrapping**: encoder counts are converted to degrees and wrapped to [0, 360) via `fmodf`.
- **Shortest-path PID**: angular error uses `shortestAngleDiff()` which returns the signed shortest distance in [-180, +180]. Going from 330 to 30 correctly computes +60 (not -300).
- **PID normalization**: error is divided by 180 (max possible error), so KP=1.0 gives full effort only at the worst-case 180 error.
- **Stick-to-angle mapping**: RC stick [-1, 1] maps to [0, 360) via `(stick + 1) * 180`, wrapping at 360. Stick center = 180, extremes = 0.
- **PID coefficients**: `FLIPPER_PID_KP/KI/KD/I_MAX` in `config.h` — both robots use the same defines (KP=1.0, KI=0.0, KD=0.0 by default). Tune on bench.

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
| 0x0A | ODrive status | per-joint telemetry (both robots) |
| 0x0B | LKTech status | per-joint telemetry (ROBOT_SECONDARY J5-J6) |
| 0x0C | ZE300 status | J4 telemetry (ROBOT_SECONDARY) |

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
  config.h          Pin assignments, constants, PID coefficients, protocol IDs
  robot_types.h     Enums, structs, protocol payloads

lib/
  RC/               PPM receiver decoding (ISR-based)
  Control/          State machine, keybind application, flipper PID
  Locomotion/       Motor output (PWM / CAN)
  CANInterface/     TWAI driver, ODrive + VESC + LKTech commands
  Comms/            Binary UART protocol TX/RX
  Encoders/         Quadrature encoder reading (PCNT), 360 wrapping
  Sensors/          I2C sensor drivers

src/
  main.cpp          setup(), FreeRTOS task creation

```

---

## Known TODOs

- **CAN bus bring-up**: Every controller (ODrive x3, VESC x5 for tracks+flippers, LKTech x2, ZE300 x1) must be pre-set to 1 Mbps before wiring to the shared TWAI bus.
- **LKTech direction/gear verification**: `LKTECH_DIR_J5/J6` and `LKTECH_GEAR_J5/J6` (default 10.0 from `dicerox_mixed_motor_config.h`) need bench verification.
- **ZE300 direction**: `ZE300_DIR_J4` default to +1.0; verify on bench (gear ratio is 1:1 output-to-command since driver handles gearbox internally).
- **VESC IDs**: `VESC_ID_*` in config.h need confirmation via VESC Tool.
- **Encoder wiring**: ROBOT_SECONDARY flipper encoder pins are marked TODO in config.h.
- **PID tuning**: Flipper PID coefficients (KP=1.0, KI=0, KD=0) are placeholder; tune on bench for both robots.

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
