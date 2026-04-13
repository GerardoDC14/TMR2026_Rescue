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
#define CAN_BITRATE_BPS 500000  // 1 Mbps — shared by ODrive, VESC, LKTech, ZE300

// Set to 0 to remove ODrive Get_Error (0x03) from the telemetry round-robin.
// This disables MSG_ODRIVE_ERROR forwarding but frees one slot in the poll cycle.
#define ODRIVE_ENABLE_ERROR_POLL  1

// ODrive E-stop mode (applies to arm + traction ODrives on v3.6):
//   1 = native hard stop (cmd 0x02).  Forces axis to IDLE immediately.
//       Recovery requires Clear_Errors + Set_Axis_State(CLOSED_LOOP).
//   0 = soft stop.  Commands zero velocity (traction) / holds position (arm).
//       Axes stay in closed-loop, no recovery handshake needed.
#define ODRIVE_ESTOP_USE_NATIVE   1

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
#define PIN_MOTOR_LEFT      23
#define PIN_MOTOR_RIGHT     17

// ─── Flipper direction pins (regular PWM mode) ─────────────────────────────────
#define PIN_FLIPPER_PWM     16   // regular PWM (0-100% duty), frequency > 1 kHz
#define PIN_FLIPPER_DIR_A   18   // direction control (HIGH/LOW = forward/reverse)
#define PIN_FLIPPER_DIR_B   19   // direction control (opposite of DIR_A)

#define MOTOR_NEUTRAL_US  1500   // no movement
#define MOTOR_MIN_US      1000   // full reverse
#define MOTOR_MAX_US      2000   // full forward

// ROBOT_MAIN only: cap traction output magnitude (1.0 = full PWM range,
// 0.5 = ±50% of the configured min/max pulse around neutral). Flipper PWM
// is unaffected.
#define TRACTION_MAX_NORM   0.1f

// Motor direction correction (1.0 = normal, -1.0 = reversed wiring)
#define TRACTION_DIR_LEFT    (1.0f)
#define TRACTION_DIR_RIGHT   (1.0f)

// Uncomment to enable closed-loop PID on traction and flipper.
// When commented out, stick input is passed directly to the drivers (open-loop).
// #define ENABLE_TRACTION_PID
// #define ENABLE_FLIPPER_PID

// Traction velocity PID (RPM feedback control) — only used when ENABLE_TRACTION_PID
#define TRACTION_MAX_RPM              200.0f

// Left track velocity PID
#define TRACTION_VEL_PID_LEFT_KP      0.5f   // effort per RPM error
#define TRACTION_VEL_PID_LEFT_KI      0.0f    // integral gain
#define TRACTION_VEL_PID_LEFT_KD      0.0f     // derivative gain
#define TRACTION_VEL_PID_LEFT_I_MAX   2.0f     // integral clamp (matched to output range)
#define TRACTION_VEL_PID_LEFT_D_ALPHA 0.8f     // D-term low-pass (0=off, 1=frozen)

// Right track velocity PID
#define TRACTION_VEL_PID_RIGHT_KP      0.1f
#define TRACTION_VEL_PID_RIGHT_KI      0.0f
#define TRACTION_VEL_PID_RIGHT_KD      0.0f
#define TRACTION_VEL_PID_RIGHT_I_MAX   2.0f
#define TRACTION_VEL_PID_RIGHT_D_ALPHA 0.8f

#define ENABLE_COMMS  // uncomment for mini-PC binary protocol (disables Serial debug)
 //define DEBUG_ARM     // arm/ODrive debug over Serial (works alongside ENABLE_COMMS)

// LEDC channel assignments (v2 API: ledcSetup/ledcAttachPin/ledcWrite).
// Left track uses ESP32Servo library on timer 0 (channels 0-1).
// Right track and flipper use explicit channels on separate timers.
//   Channel 2 → timer 1 (right track, 50 Hz servo)
//   Channel 4 → timer 2 (flipper, 20 kHz PWM+DIR)
#define LEDC_CH_RIGHT           2
#define LEDC_CH_FLIPPER         4

#define SERVO_LEDC_FREQ_HZ      50
#define SERVO_LEDC_RESOLUTION   16       // 16-bit → 65535 ticks/period
#define SERVO_LEDC_MAX_DUTY     65535
#define SERVO_LEDC_PERIOD_US    20000    // = 1/50 Hz in µs

#define FLIPPER_PWM_FREQ_HZ     20000
#define FLIPPER_PWM_RESOLUTION  10       // 10-bit → 0–1023

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
  #define ENC_CPR_TRACK        500.0f
  #define ENC_CPR_FLIPPER      1500.0f
  #define TRACK_GEAR_RATIO       20.0f   // motor→wheel reduction
  #define FLIPPER_GEAR_RATIO     1.0f
  
  // Flipper position PID — error in degrees, output in effort [-1, +1]
  #define FLIPPER_PID_KP        0.05f   // effort per degree of error
  #define FLIPPER_PID_KI        0.0f    // integral gain
  #define FLIPPER_PID_KD        0.0f    // derivative gain
  #define FLIPPER_PID_I_MAX     10.0f   // integral clamp (degree-seconds)
  #define FLIPPER_PID_D_ALPHA   0.8f    // D-term low-pass (0=off, 1=frozen)
  #define FLIPPER_RATE_DPS      180.0f  // flipper speed at full stick (°/s)

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

  // Flipper position PID — error in degrees, output in effort [-1, +1]
  #define FLIPPER_PID_KP        0.05f   // effort per degree of error
  #define FLIPPER_PID_KI        0.0f    // integral gain
  #define FLIPPER_PID_KD        0.0f    // derivative gain
  #define FLIPPER_PID_I_MAX     10.0f   // integral clamp (degree-seconds)
  #define FLIPPER_PID_D_ALPHA   0.8f    // D-term low-pass (0=off, 1=frozen)
  #define FLIPPER_RATE_DPS      180.0f  // flipper speed at full stick (°/s)
#endif

// ROBOT_SECONDARY: mixed-controller setup over a single 1 Mbps TWAI bus.
//   - Left/right traction: ODrive (native velocity control, Set_Input_Vel)
//   - Four flippers:       VESC   (native velocity control, SET_RPM)
#ifdef ROBOT_SECONDARY

  // ── Traction ODrive left/right — VELOCITY_CONTROL + VEL_RAMP ───────────────
  // Physically and logically separate from the arm ODrives even though they
  // share the CANSimple protocol.  Bring-up handshake: Clear_Errors →
  // Set_Controller_Mode(VELOCITY, VEL_RAMP) → Set_Axis_State(CLOSED_LOOP)
  // with heartbeat confirmation.
  // Node IDs are still being finalised — update once wiring is settled.
  #define TRACTION_NODE_LEFT              0x02   // TODO: confirm via odrivetool
  #define TRACTION_NODE_RIGHT             0x03   // TODO: confirm
  #define TRACTION_DIR_LEFT               1.0f   // TODO: verify direction on bench
  #define TRACTION_DIR_RIGHT              1.0f   // TODO: verify
  #define TRACTION_MAX_VEL_TURNS_S        8.0f   // normalised stick → turns/s (reference: 0–8 nominal)
  #define TRACTION_CLOSED_LOOP_TIMEOUT_MS 800    // handshake confirmation window
  #define TRACTION_CLOSED_LOOP_RETRY_MS   100    // retry interval inside the window

  // ── Flipper VESCs — native velocity mode (SET_RPM, cmd 3) ──────────────────
  // eRPM at full stick = mechanical_rpm × pole_pairs.  Tune for the actual
  // flipper motors.  If SET_RPM doesn't work on the bench, set
  // VESC_FLIPPER_USE_RPM to 0 to fall back to SET_CURRENT (cmd 1).
  #define VESC_ID_FLIPPER_FL      3   // front-left  — TODO: confirm
  #define VESC_ID_FLIPPER_FR      4   // front-right — TODO: confirm
  #define VESC_ID_FLIPPER_RL      5   // rear-left   — TODO: confirm
  #define VESC_ID_FLIPPER_RR      6   // rear-right  — TODO: confirm
  #define VESC_FLIPPER_USE_RPM        1        // 1 = SET_RPM (velocity), 0 = SET_CURRENT (torque)
  #define VESC_FLIPPER_ERPM_MAX    10000       // eRPM at full stick — TODO: tune
  #define VESC_FLIPPER_I_MAX_A     3.0f        // used only when VESC_FLIPPER_USE_RPM=0

  // ── ODrive arm J1–J3 — CAN node IDs (set in odrivetool / DIP switches) ──────
  // Confirmed from ginkgo_odrive_bridge yaml (node_ids: [16, 17, 18])
  #define ODRIVE_NODE_J1          0x10   // 16
  #define ODRIVE_NODE_J2          0x11   // 17
  #define ODRIVE_NODE_J3          0x12   // 18
  #define ODRIVE_GEAR_J1          48.0f
  #define ODRIVE_GEAR_J2          48.0f
  #define ODRIVE_GEAR_J3          48.0f
  #define ODRIVE_DIR_J1          (-1.0f)
  #define ODRIVE_DIR_J2          (-1.0f)
  #define ODRIVE_DIR_J3          (-1.0f)

  // ── ZE300 arm J4 — CAN device ID (set via ZE300 tool) ──────────────────────
  // Standard 11-bit frames, tagged request ID = 0x100 | device_id, reply ID = device_id.
  // Position command ratio is 1:1 (driver internally handles the 1:8 gearbox),
  // so we command directly in output degrees.  16384 counts per output rev.
  // Confirmed from dicerox_mixed_motor_config.h.
  #define ZE300_ID_J4                  1     // device_id
  #define ZE300_REQ_ID_BASE        0x100     // tagged request ID = base | device_id
  #define ZE300_COUNTS_PER_REV     16384     // output counts per revolution
  #define ZE300_DIR_J4              1.0f     // TODO: verify direction on bench
  #define ZE300_MAX_SPEED_CRPM      3000     // position max speed in centi-RPM (30 RPM)
  #define ZE300_ZERO_TIMEOUT_MS       50     // blocking timeout for startup RTR
  #define ZE300_TELEM_INTERVAL_MS    200     // active realtime-state poll period

  // ── LKTech arm J5–J6 — CAN motor IDs (set via LKTech Tool) ─────────────────
  // Standard 11-bit frames, ID = 0x140 + motor_id.  Reply shares the same ID.
  // Confirmed IDs / gear ratios from dicerox_mixed_motor_config.h.
  #define LKTECH_ID_BASE          0x140
  #define LKTECH_ID_J5              14   // confirmed from dicerox_mixed_motor_config.h
  #define LKTECH_ID_J6              15   // confirmed from dicerox_mixed_motor_config.h
  #define LKTECH_GEAR_J5          10.0f  // confirmed from dicerox_mixed_motor_config.h
  #define LKTECH_GEAR_J6          10.0f  // confirmed
  #define LKTECH_DIR_J5            1.0f  // TODO: verify direction on bench
  #define LKTECH_DIR_J6            1.0f  // TODO: verify direction on bench
  #define LKTECH_DEFAULT_SPEED_DPS  360  // max speed for position commands (°/s)
  #define LKTECH_ZERO_TIMEOUT_MS    50   // blocking timeout for READ_MULTI_LOOP_ANGLE

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
#define MSG_ODRIVE_STATUS    0x0A        // Per-ODrive joint telemetry (both robots)
#define MSG_LKTECH_STATUS    0x0B        // ROBOT_SECONDARY: per-LKTech joint telemetry (J5-J6)
#define MSG_ZE300_STATUS     0x0C        // ROBOT_SECONDARY: ZE300 J4 telemetry
#define MSG_ODRIVE_ERROR     0x0D        // Per-ODrive error snapshot (both robots, optional)

// PC → ESP32 message types
#define MSG_ARM_JOINTS       0x10        // 6 × int16 joint angles (×100 deg)
#define MSG_SENSOR_ENABLE    0x11        // 1-byte bitmask
#define MSG_ESTOP            0x12        // 0-byte payload — immediate stop
#define MSG_ESTOP_CLEAR      0x13        // 0-byte payload — resume
#define MSG_KEYBIND          0x14        // 15 bytes: 3 modes × 5 channel slots
#define MSG_PPM_CALIB        0x15        // 6 bytes: min_us, neutral_us, max_us (uint16 each)
#define MSG_GRIPPER          0x16        // 2 bytes: int16 normalised × 1000 (−1000 to +1000)

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