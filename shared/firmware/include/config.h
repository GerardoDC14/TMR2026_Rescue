#pragma once

// ─── Robot Identity ───────────────────────────────────────────────────────────
// Select which robot this binary targets by uncommenting one define.
// Robot-specific hardware differences (drivetrain, CAN IDs, encoder ratios)
// are isolated inside the individual module implementations.
#define ROBOT_MAIN
// #define ROBOT_SECONDARY

// ─── I2C ─────────────────────────────────────────────────────────────────────
#define PIN_I2C_SDA         33
#define PIN_I2C_SCL         25

// ─── TWAI (ESP32 built-in CAN) + SN65HVD230 transceiver ──────────────────────
// GPIO assignments — route TX→CTX and RX←CRX on the SN65HVD230.
#define PIN_CAN_TX          22   // ESP32 CTX → SN65HVD230 D
#define PIN_CAN_RX          21   // ESP32 CRX ← SN65HVD230 R
#define CAN_BITRATE_BPS  500000  // 500 kbps

// ─── UART2 — Mini PC ─────────────────────────────────────────────────────────
#define MINIPC_BAUD     921600

// ─── PPM input ───────────────────────────────────────────────────────────────
#define PIN_PPM             34   // input-only GPIO; no pull required if signal is driven
#define PPM_CHANNELS         6
#define PPM_SYNC_US       3000   // gap > this µs → frame sync
#define PPM_MIN_US        1000   // nominal min pulse
#define PPM_MAX_US        2000   // nominal max pulse
#define PPM_TIMEOUT_MS     500   // failsafe: no valid frame within this window

// Channel assignments (1-indexed to match physical Flysky labels)
#define PPM_CH_MODE          5   // Ch5 → main mode vs arm mode switch

// ─── Motor PWM outputs (servo-style: 50 Hz, 1000–2000 µs) ───────────────────
#define PIN_MOTOR_LEFT      16
#define PIN_MOTOR_RIGHT     23
#define PIN_MOTOR_FLIPPER   17

#define MOTOR_NEUTRAL_US  1500   // no movement
#define MOTOR_MIN_US      1000   // full reverse
#define MOTOR_MAX_US      2000   // full forward

// LEDC channels (ESP32 PWM peripheral)
#define LEDC_CH_LEFT         0
#define LEDC_CH_RIGHT        1
#define LEDC_CH_FLIPPER      2
#define PWM_FREQ_HZ         50
#define PWM_RESOLUTION      14   // bits  → 16384 ticks/period
#define PWM_PERIOD_US    20000   // = 1/50 Hz in µs

// ─── Quadrature Encoders (PCNT hardware) ─────────────────────────────────────
// Pins shared by both robots (track encoders)
#define PIN_ENC_LEFT_A      36
#define PIN_ENC_LEFT_B      39
#define PIN_ENC_RIGHT_A     35
#define PIN_ENC_RIGHT_B     32

// PCNT units 0–1 are always LEFT and RIGHT tracks
#define PCNT_UNIT_LEFT       0
#define PCNT_UNIT_RIGHT      1
#define PCNT_HIGH_LIM    30000
#define PCNT_LOW_LIM    -30000

// ROBOT_MAIN: one joined flipper on unit 2
#ifdef ROBOT_MAIN
  #define PIN_ENC_FLIP_A      26
  #define PIN_ENC_FLIP_B       4
  #define PCNT_UNIT_FLIPPER    2
  #define NUM_ENCODER_UNITS    3

  // Encoder constants — fill from datasheet / measurement
  #define ENC_CPR_TRACK        1000.0f
  #define ENC_CPR_FLIPPER      1000.0f
  #define TRACK_GEAR_RATIO       20.0f   // motor→wheel reduction
  #define FLIPPER_GEAR_RATIO     30.0f
  #define FLIPPER_ANGLE_MIN     -10.0f   // mechanical limits (degrees)
  #define FLIPPER_ANGLE_MAX     120.0f

  // Flipper position PID coefficients — tune on bench
  #define FLIPPER_PID_KP        1.0f   // proportional gain
  #define FLIPPER_PID_KI        0.0f   // integral gain
  #define FLIPPER_PID_KD        0.0f   // derivative gain
  #define FLIPPER_PID_I_MAX     0.5f   // integral windup clamp (normalised effort units)

  // ── ODrive arm J1–J3 (joints 4–6 are servos on a separate ESP32) ────────────
  #define ODRIVE_MAIN_NUM_JOINTS    3
  #define ODRIVE_NODE_J1          0x10   // same CAN nodes as secondary robot
  #define ODRIVE_NODE_J2          0x11
  #define ODRIVE_NODE_J3          0x12
  #define ODRIVE_GEAR_J1          48.0f
  #define ODRIVE_GEAR_J2          48.0f
  #define ODRIVE_GEAR_J3          48.0f
  #define ODRIVE_DIR_J1          (-1.0f)
  #define ODRIVE_DIR_J2          (-1.0f)
  #define ODRIVE_DIR_J3          (-1.0f)
  #define ODRIVE_ZERO_TIMEOUT_MS  50
#endif

// ROBOT_SECONDARY: 4 independent flippers on units 2–5
// Pin assignments use GPIOs freed from LEDC (25/26/27) plus spare pins.
// TODO: confirm physical wiring before deploying.
#ifdef ROBOT_SECONDARY
  #define PIN_ENC_FLIP_FL_A   26   // front-left flipper — TODO: confirm
  #define PIN_ENC_FLIP_FL_B    4
  #define PIN_ENC_FLIP_FR_A   16   // front-right flipper — TODO: confirm
  #define PIN_ENC_FLIP_FR_B   13
  #define PIN_ENC_FLIP_RL_A   23   // rear-left  flipper — TODO: confirm
  #define PIN_ENC_FLIP_RL_B   19
  #define PIN_ENC_FLIP_RR_A   28   // rear-right flipper — TODO: confirm
  #define PIN_ENC_FLIP_RR_B   17
  #define PCNT_UNIT_FLIP_FL    2
  #define PCNT_UNIT_FLIP_FR    3
  #define PCNT_UNIT_FLIP_RL    4
  #define PCNT_UNIT_FLIP_RR    5
  #define NUM_ENCODER_UNITS    6

  // Encoder constants — TODO: fill from motor/encoder datasheet
  #define ENC_CPR_TRACK        1000.0f
  #define ENC_CPR_FLIPPER      1000.0f   // shared by all 4 flipper encoders
  #define TRACK_GEAR_RATIO       20.0f
  #define FLIPPER_GEAR_RATIO     30.0f
  #define FLIPPER_ANGLE_MIN     -10.0f
  #define FLIPPER_ANGLE_MAX     120.0f
#endif

// ROBOT_SECONDARY: VESC motor controllers over CAN (500 kbps, TWAI)
// IDs are set in VESC Tool under "Controller ID".
// Currents are the peak value commanded at full stick deflection; tune on bench.
#ifdef ROBOT_SECONDARY
  #define VESC_ID_TRACK_LEFT      1   // TODO: confirm via VESC Tool
  #define VESC_ID_TRACK_RIGHT     2   // TODO: confirm
  #define VESC_ID_FLIPPER_FL      3   // front-left  — TODO: confirm
  #define VESC_ID_FLIPPER_FR      4   // front-right — TODO: confirm
  #define VESC_ID_FLIPPER_RL      5   // rear-left   — TODO: confirm
  #define VESC_ID_FLIPPER_RR      6   // rear-right  — TODO: confirm

  #define VESC_TRACK_I_MAX_A      5.0f   // TODO: tune for traction motors
  #define VESC_FLIPPER_I_MAX_A    3.0f   // TODO: tune for flipper motors

  // ── ODrive arm — CAN node IDs (set in odrivetool / DIP switches) ────────────
  // J1–J3: confirmed from ginkgo_odrive_bridge yaml (node_ids: [16, 17, 18])
  // J4–J6: all-brushless variant — node IDs TODO (assumed sequential)
  #define ODRIVE_NODE_J1          0x10   // 16
  #define ODRIVE_NODE_J2          0x11   // 17
  #define ODRIVE_NODE_J3          0x12   // 18
  #define ODRIVE_NODE_J4          0x13   // TODO: confirm
  #define ODRIVE_NODE_J5          0x14   // TODO: confirm
  #define ODRIVE_NODE_J6          0x15   // TODO: confirm

  // Gear ratios (motor turns → output turns).
  // J1–J3: confirmed (48.0). J4–J6: assumed same; verify on bench.
  #define ODRIVE_GEAR_J1          48.0f
  #define ODRIVE_GEAR_J2          48.0f
  #define ODRIVE_GEAR_J3          48.0f
  #define ODRIVE_GEAR_J4          48.0f  // TODO: confirm
  #define ODRIVE_GEAR_J5          48.0f  // TODO: confirm
  #define ODRIVE_GEAR_J6          48.0f  // TODO: confirm

  // Direction multiplier (+1 or -1): flips sign so positive angle = positive motion.
  // J1–J3: confirmed (-1). J4–J6: assumed same; verify on bench.
  #define ODRIVE_DIR_J1          (-1.0f)
  #define ODRIVE_DIR_J2          (-1.0f)
  #define ODRIVE_DIR_J3          (-1.0f)
  #define ODRIVE_DIR_J4          (-1.0f) // TODO: confirm
  #define ODRIVE_DIR_J5          (-1.0f) // TODO: confirm
  #define ODRIVE_DIR_J6          (-1.0f) // TODO: confirm

  // Encoder zero capture: max time to wait for one RTR response (ms).
  #define ODRIVE_ZERO_TIMEOUT_MS  50
#endif

#define ENC_SPEED_INTERVAL_MS   50       // speed recalculation period

// ─── Sensors ─────────────────────────────────────────────────────────────────
#define PIN_MQ2              27

#define MLX90640_I2C_ADDR    0x33

#define BNO055_I2C_ADDR      0x28        // SA0=GND; use 0x29 if SA0=VCC

#define MQ2_RL_KOHM          10.0f       // load resistor on MQ2 board (kΩ)
#define MQ2_RO_KOHM          10.0f       // Rs in clean air — calibrate on bench
#define MQ2_SAMPLE_COUNT         5       // ADC samples to average per reading

// Sensor enable bitmask bits
#define SENSOR_BIT_MAG       (1 << 0)
#define SENSOR_BIT_THERMAL   (1 << 1)
#define SENSOR_BIT_GAS       (1 << 2)
#define SENSOR_BIT_IMU       (1 << 3)

// ─── Mini-PC Binary Protocol ──────────────────────────────────────────────────
// Frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
// CRC = XOR of TYPE + LEN_H + LEN_L + all PAYLOAD bytes
#define PROTO_SOF_0          0xAA
#define PROTO_SOF_1          0x55
#define PROTO_MAX_PAYLOAD    1600        // worst case: full thermal frame

// ESP32 → PC message types
#define MSG_TELEMETRY        0x01        // PPM + encoder + state, ~50 Hz
#define MSG_SENSOR_THERMAL   0x02        // 32×24 thermal pixels (int16 ×10 °C)
#define MSG_SENSOR_MAG       0x03        // magnetometer XYZ
#define MSG_SENSOR_GAS       0x04        // gas sensor Rs/Ro ratio
#define MSG_STATUS           0x05        // system status / heartbeat
#define MSG_SENSOR_IMU       0x06        // BNO055 orientation + accel + gyro
#define MSG_ENCODER_EXT      0x07        // ROBOT_SECONDARY: 4 independent flipper angles
#define MSG_VESC_STATUS      0x08        // ROBOT_SECONDARY: per-VESC feedback (erpm, current, temp, voltage)
#define MSG_MOTOR_MAIN       0x09        // ROBOT_MAIN: track + flipper duty cycles

// PC → ESP32 message types
#define MSG_ARM_JOINTS       0x10        // 6 × int16 joint angles (×100 deg)
#define MSG_SENSOR_ENABLE    0x11        // 1-byte bitmask
#define MSG_ESTOP            0x12        // 0-byte payload — immediate stop
#define MSG_ESTOP_CLEAR      0x13        // 0-byte payload — resume
#define MSG_KEYBIND          0x14        // 15 bytes: 3 modes × 5 channel slots
#define MSG_PPM_CALIB        0x15        // 6 bytes: min_us, neutral_us, max_us (uint16 each)

// ─── Sensor rate caps (Hz) ────────────────────────────────────────────────────
// Throttle individual sensors so we don't flood the UART or waste CPU.
// Thermal is hardware-limited by MLX90640_REFRESH_HZ and runs in its own task.
#define SENSOR_MAG_HZ           50
#define SENSOR_IMU_HZ           50
#define SENSOR_GAS_HZ           10

// ─── FreeRTOS Task Config ────────────────────────────────────────────────────
// Core 0: protocol tasks (comms + CAN)
// Core 1: control + sensor tasks (thermal gets its own task)
#define TASK_CORE_COMMS      0
#define TASK_CORE_CAN        0
#define TASK_CORE_CONTROL    1
#define TASK_CORE_SENSORS    1
#define TASK_CORE_THERMAL    1

#define STACK_CONTROL        5120
#define STACK_COMMS          4096
#define STACK_CAN            3072
#define STACK_SENSORS        4096    // fast sensors only (mag/gas/imu)
#define STACK_THERMAL        8192   // MLX90640 needs a bigger stack

#define PRIO_CONTROL            5    // highest — real-time loop
#define PRIO_COMMS              4
#define PRIO_CAN                4
#define PRIO_SENSORS            2    // low priority — background
#define PRIO_THERMAL            1    // lowest — thermal never blocks anything

#define CONTROL_LOOP_HZ        50    // target control cycle rate