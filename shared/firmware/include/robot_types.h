#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ─── Operating Mode ───────────────────────────────────────────────────────────
enum class RobotMode : uint8_t {
    INIT    = 0,  // startup / hardware init
    STANDBY = 1,  // idle, waiting for RC link
    NORMAL  = 2,  // RC drives tracks (+ single flipper on ROBOT_MAIN)
    ARM     = 3,  // RC input forwarded to mini PC for IK
    ESTOP   = 4,  // all outputs neutralised; cleared only by mini PC
    FLIPPER = 5,  // RC drives flipper(s): single joined on ROBOT_MAIN,
                  //   four independent via CAN on ROBOT_SECONDARY
};

// ─── Channel Function (keybind system) ───────────────────────────────────────
// What a PPM channel can be bound to.  Values match the GUI enum.
enum class ChannelFunction : uint8_t {
    NONE         = 0,
    TRACTION_FWD = 1,   // forward / back
    TRACTION_TURN = 2,  // left / right differential
    FLIPPER_ALL  = 3,   // all flippers move together
    FLIPPER_FL   = 4,   // individual front-left
    FLIPPER_FR   = 5,   // individual front-right
    FLIPPER_RL   = 6,   // individual rear-left
    FLIPPER_RR   = 7,   // individual rear-right
    ARM_FWD      = 8,   // forward to mini-PC for arm IK (legacy / generic)
    ESTOP        = 9,   // virtual e-stop
    ARM_X        = 10,  // arm Cartesian +X (forward / back)
    ARM_Y        = 11,  // arm Cartesian +Y (lateral)
    ARM_Z        = 12,  // arm Cartesian +Z (up / down)
    ARM_PITCH    = 13,  // arm pitch rotation
    ARM_YAW      = 14,  // arm yaw rotation
};

// Returns true for any arm-related channel function.
inline bool isArmFunction(ChannelFunction fn) {
    return fn == ChannelFunction::ARM_FWD  ||
           fn == ChannelFunction::ARM_X    ||
           fn == ChannelFunction::ARM_Y    ||
           fn == ChannelFunction::ARM_Z    ||
           fn == ChannelFunction::ARM_PITCH ||
           fn == ChannelFunction::ARM_YAW;
}

// Keybind table: 3 Ch5 lever positions × 5 channel slots (Ch1,Ch2,Ch3,Ch4,Ch6)
struct KeybindTable {
    ChannelFunction map[3][5];
};

// ─── PPM / RC ────────────────────────────────────────────────────────────────
struct PPMFrame {
    uint16_t ch[PPM_CHANNELS];  // raw µs values, index 0 = channel 1
    uint32_t timestamp_ms;
    bool     valid;
};

// Normalised helper: maps raw µs to [-1.0, +1.0]
inline float ppmNormalise(uint16_t raw_us) {
    constexpr float mid  = (PPM_MIN_US + PPM_MAX_US) * 0.5f;
    constexpr float half = (PPM_MAX_US - PPM_MIN_US) * 0.5f;
    float v = (static_cast<float>(raw_us) - mid) / half;
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

// ─── Encoder / Drive ─────────────────────────────────────────────────────────
struct EncoderState {
    // Track encoders — present on both robots
    int32_t  count_left;
    int32_t  count_right;
    float    speed_left_rpm;
    float    speed_right_rpm;

    // ROBOT_MAIN: single joined flipper
    int32_t  count_flipper;
    float    flipper_angle_deg;

    // ROBOT_SECONDARY: four independent flippers (zero on ROBOT_MAIN)
    int32_t  count_flip_fl, count_flip_fr, count_flip_rl, count_flip_rr;
    float    flipper_angle_fl_deg, flipper_angle_fr_deg;
    float    flipper_angle_rl_deg, flipper_angle_rr_deg;

    uint32_t timestamp_ms;
};

// ─── Arm ─────────────────────────────────────────────────────────────────────
struct ArmJoints {
    float angle_deg[6];   // one entry per DOF, degrees
    bool  valid;
};

// ─── IMU ─────────────────────────────────────────────────────────────────────
struct ImuData {
    // Euler angles (degrees, BNO055 fused output)
    float yaw_deg, pitch_deg, roll_deg;

    // Linear acceleration (m/s², gravity-compensated)
    float accel_x, accel_y, accel_z;

    // Angular velocity (rad/s)
    float gyro_x, gyro_y, gyro_z;

    // Packed calibration: bits[7:6]=sys, bits[5:4]=gyro, bits[3:2]=accel, bits[1:0]=mag
    // Each field is 0–3 (3 = fully calibrated)
    uint8_t calib;

    bool valid;
};

// ─── Sensor Data ─────────────────────────────────────────────────────────────
struct MagData {
    int x_uT, y_uT, z_uT;
    bool  valid;
};

struct ThermalData {
    float pixels[32 * 24];   // °C, row-major (32 columns × 24 rows)
    bool  valid;
};

struct GasData {
    float rs_ro_ratio;    // sensor ratio (lower → more gas)
    bool  valid;
};

// ─── System Status ────────────────────────────────────────────────────────────
struct SystemStatus {
    RobotMode mode;
    bool      ppm_connected;
    bool      minipc_connected;
    bool      can_ok;
    uint8_t   sensor_mask;   // active sensor bitmask (SENSOR_BIT_*)
    bool      estop;
    uint32_t  uptime_ms;
};

// ─── Mini-PC Protocol Payloads ────────────────────────────────────────────────
// These structs are packed and sent/received over UART as binary payloads.

#pragma pack(push, 1)

struct TelemetryPayload {
    uint8_t  mode;             // RobotMode value
    uint8_t  flags;            // bit0=ppm_ok bit1=sensors_active bit2=can_ok bit3=estop
    uint16_t ppm[PPM_CHANNELS];// raw µs per channel
    int16_t  speed_left;       // RPM × 10
    int16_t  speed_right;      // RPM × 10
    int16_t  flipper_angle;    // degrees × 10
    uint32_t uptime_ms;
};

struct ArmJointsPayload {
    int16_t joint[6];          // degrees × 100 per joint
};

struct SensorEnablePayload {
    uint8_t mask;              // SENSOR_BIT_* flags
};

struct MagPayload {
    int16_t x_uT100;           // µT × 100
    int16_t y_uT100;
    int16_t z_uT100;
};

struct GasPayload {
    int16_t rs_ro_100;         // ratio × 100
};

// ThermalPayload: 32×24 int16_t (°C × 10) = 1536 bytes
struct ThermalPayload {
    int16_t pixels[32 * 24];   // °C × 10
};

// ImuPayload (MSG_SENSOR_IMU = 0x06) — 19 bytes
struct ImuPayload {
    int16_t yaw_deg10;          // degrees × 10
    int16_t pitch_deg10;
    int16_t roll_deg10;
    int16_t accel_x_ms2_100;    // m/s² × 100
    int16_t accel_y_ms2_100;
    int16_t accel_z_ms2_100;
    int16_t gyro_x_rads1000;    // rad/s × 1000
    int16_t gyro_y_rads1000;
    int16_t gyro_z_rads1000;
    uint8_t calib;              // bits[7:6]=sys, bits[5:4]=gyro, bits[3:2]=accel, bits[1:0]=mag
};

// EncoderExtPayload (MSG_ENCODER_EXT = 0x07) — ROBOT_SECONDARY only — 8 bytes
struct EncoderExtPayload {
    int16_t flipper_fl_deg10;   // angle × 10 degrees; front-left
    int16_t flipper_fr_deg10;   // front-right
    int16_t flipper_rl_deg10;   // rear-left
    int16_t flipper_rr_deg10;   // rear-right
};

// KeybindPayload (MSG_KEYBIND = 0x14) — 15 bytes
// 3 modes (Ch5 lever positions) × 5 channel slots (Ch1,Ch2,Ch3,Ch4,Ch6)
// Each byte is a ChannelFunction enum value.
struct KeybindPayload {
    uint8_t map[3][5];
};

// VescStatusPayload (MSG_VESC_STATUS = 0x08) — per-controller status
// Sent from ROBOT_SECONDARY for each VESC that broadcasts status frames.
struct VescStatusPayload {
    uint8_t vesc_id;            // VESC controller ID (1–6)
    int32_t erpm;               // electrical RPM
    int16_t current_10;         // motor current × 10 (A)
    int16_t duty_1000;          // duty cycle × 1000
    int16_t temp_fet_10;        // FET temperature × 10 (°C)
    int16_t temp_motor_10;      // motor temperature × 10 (°C)
    int16_t v_in_10;            // input voltage × 10 (V)
};

// PpmCalibPayload (MSG_PPM_CALIB = 0x15) — PPM_CHANNELS × 6 bytes = 36 bytes
// Per-channel runtime calibration for the RC receiver.
struct PpmChannelCalibEntry {
    uint16_t min_us;      // minimum pulse width  (default 1000 µs)
    uint16_t neutral_us;  // centre/neutral pulse  (default 1500 µs)
    uint16_t max_us;      // maximum pulse width   (default 2000 µs)
};
struct PpmCalibPayload {
    PpmChannelCalibEntry ch[PPM_CHANNELS];  // one entry per channel (0 = Ch1 … 5 = Ch6)
};

// OdriveStatusPayload (MSG_ODRIVE_STATUS = 0x0A) — per-joint ODrive telemetry — 11 bytes
// Note: no temperature available via CAN on ODrive v3.6 (0x15 = sensorless, not temp).
struct OdriveStatusPayload {
    uint8_t joint_idx;            // 0-based joint index (0=J1, 1=J2, 2=J3)
    int16_t pos_turns_100;        // encoder position × 100 (motor turns)
    int16_t vel_turns_s_100;      // velocity × 100 (turns/s)
    int16_t iq_measured_100;      // measured Iq × 100 (A)
    int16_t bus_voltage_10;       // bus voltage × 10 (V)
    int16_t bus_current_100;      // bus current × 100 (A)
};

// MainMotorPayload (MSG_MOTOR_MAIN = 0x09) — ROBOT_MAIN only — 6 bytes
// Commanded normalised effort for each actuator, scaled × 1000.
struct MainMotorPayload {
    int16_t duty_left_1000;     // left track effort × 1000 (−1000 to +1000)
    int16_t duty_right_1000;    // right track effort × 1000
    int16_t duty_flipper_1000;  // flipper effort × 1000
};

// OdriveErrorPayload (MSG_ODRIVE_ERROR = 0x0D) — optional, gated by ODRIVE_ENABLE_ERROR_POLL.
// v3.6: Get_Motor_Error (0x03) returns a single u64 motor_error. Sent whenever non-zero.
struct OdriveErrorPayload {
    uint8_t  node_id;       // CAN node id of the ODrive
    uint64_t motor_error;   // v3.6 motor_error bitfield (see ODrive v0.5.x docs)
};

// LktechStatusPayload (MSG_LKTECH_STATUS = 0x0B) — ROBOT_SECONDARY J5/J6 — 10 bytes
// Parsed from A4 command acknowledges (same arb ID, same layout as READ_STATE_2).
struct LktechStatusPayload {
    uint8_t joint_idx;          // 0 = J5, 1 = J6
    uint8_t motor_id;           // LKTech CAN motor ID (e.g. 14, 15)
    int8_t  temp_c;             // motor temperature (°C, signed)
    int16_t iq_100;             // Iq current × 100 (A) — i16 /100 per spec
    int16_t speed_dps;          // motor speed (°/s)
    int16_t angle_deg;          // motor angle (degrees, wraps every i16 range)
    int16_t output_deg_10;      // output angle × 10 (gear-compensated, software-zeroed)
};

// Ze300StatusPayload (MSG_ZE300_STATUS = 0x0C) — ROBOT_SECONDARY J4 — 12 bytes
// Parsed from passive C2 command replies (position) + active 0xA4 realtime state polls.
struct Ze300StatusPayload {
    uint8_t device_id;          // ZE300 device_id (e.g. 1)
    int8_t  temp_c;             // motor temperature (°C)
    int16_t iq_1000;            // Iq × 1000 (A) — i16 /1000 per spec
    int16_t speed_rpm_100;      // motor speed × 100 (rpm) — i16 /100 per spec
    int16_t single_turn_counts; // single-turn position (u16 LE, stored as signed for packing)
    int32_t position_counts;    // multi-turn position (counts, from C2 replies)
    int16_t output_deg_10;      // software-zeroed output angle × 10 (degrees)
};

#pragma pack(pop)