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
    int32_t  count_left;
    int32_t  count_right;
    int32_t  count_flipper;
    float    speed_left_rpm;
    float    speed_right_rpm;
    float    flipper_angle_deg;
    uint32_t timestamp_ms;
};

// ─── Arm ─────────────────────────────────────────────────────────────────────
struct ArmJoints {
    float angle_deg[6];   // one entry per DOF, degrees
    bool  valid;
};

struct ArmEndEffector {
    // cartesian position (mm, robot frame) + orientation (degrees)
    float x, y, z;
    float roll, pitch, yaw;
};

// ─── Sensor Data ─────────────────────────────────────────────────────────────
struct MagData {
    float x_uT, y_uT, z_uT;
    float heading_deg;    // 0–360°, North referenced if calibrated
    bool  valid;
};

struct ThermalData {
    float pixels[32 * 24];   // °C, row-major (32 columns × 24 rows)
    float ambient_temp_C;
    float max_temp_C;
    bool  valid;
};

struct GasData {
    float rs_ro_ratio;    // sensor ratio (lower → more gas)
    float ppm_lpg;
    float ppm_co;
    float ppm_smoke;
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
    int16_t heading_deg10;     // degrees × 10
};

struct GasPayload {
    int16_t rs_ro_100;         // ratio × 100
    int16_t ppm_lpg;
    int16_t ppm_co;
    int16_t ppm_smoke;
};

// ThermalPayload: 32×24 int16_t (°C × 10) + 1 int16_t ambient = 769 int16_t = 1538 bytes
// Sent as raw int16_t array — ambient is last element
struct ThermalPayload {
    int16_t pixels[32 * 24];   // °C × 10
    int16_t ambient_C10;
};

#pragma pack(pop)