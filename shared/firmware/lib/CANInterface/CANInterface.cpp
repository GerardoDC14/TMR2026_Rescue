#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include "driver/twai.h"

// ─── TWAI state ───────────────────────────────────────────────────────────────
static bool s_ok = false;

// Transmit a pre-built TWAI message.  Returns true if the driver accepted it.
static bool twaiSend(twai_message_t& msg) {
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

// ─── VESC CAN helpers ─────────────────────────────────────────────────────────
// Standard VESC extended-frame protocol.
// Extended ID = (command << 8) | controller_id, CAN_EFF_FLAG equivalent →
// set extd = 1 in the TWAI message flags.

static constexpr uint8_t VESC_CMD_SET_CURRENT = 1;

static inline uint32_t vescEID(uint8_t ctrl_id, uint8_t cmd) {
    return ((uint32_t)cmd << 8) | ctrl_id;
}

static inline void putInt32BE(uint8_t* d, int32_t v) {
    d[0] = (v >> 24) & 0xFF;
    d[1] = (v >> 16) & 0xFF;
    d[2] = (v >>  8) & 0xFF;
    d[3] = (v >>  0) & 0xFF;
}

static bool vescSendCurrent(uint8_t ctrl_id, float current_a) {
    twai_message_t msg = {};
    msg.extd       = 1;                              // extended 29-bit ID
    msg.identifier = vescEID(ctrl_id, VESC_CMD_SET_CURRENT);
    msg.data_length_code = 4;
    putInt32BE(msg.data, static_cast<int32_t>(current_a * 1000.0f));  // A → mA
    return twaiSend(msg);
}

// ─── ODrive CAN helpers (ROBOT_SECONDARY arm) ─────────────────────────────────
// Standard 11-bit frames, little-endian payloads.
// COB-ID = (node_id << 5) | cmd_id.
//
//   0x07  SET_AXIS_STATE   uint32 LE  (8 = CLOSED_LOOP_CONTROL)
//   0x09  GET_ENCODER_EST  RTR        reply: float32 LE pos_est, float32 LE vel_est
//   0x0C  SET_INPUT_POS    float32 LE (turns) + int16 LE vel_ff=0 + int16 LE torque_ff=0

#ifdef ROBOT_SECONDARY

static constexpr uint8_t ODRIVE_CMD_SET_AXIS_STATE  = 0x07;
static constexpr uint8_t ODRIVE_CMD_GET_ENCODER_EST = 0x09;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_POS   = 0x0C;
static constexpr uint8_t ODRIVE_NUM_JOINTS          = 6;

static const uint8_t s_node[ODRIVE_NUM_JOINTS] = {
    ODRIVE_NODE_J1, ODRIVE_NODE_J2, ODRIVE_NODE_J3,
    ODRIVE_NODE_J4, ODRIVE_NODE_J5, ODRIVE_NODE_J6
};
static const float s_gear[ODRIVE_NUM_JOINTS] = {
    ODRIVE_GEAR_J1, ODRIVE_GEAR_J2, ODRIVE_GEAR_J3,
    ODRIVE_GEAR_J4, ODRIVE_GEAR_J5, ODRIVE_GEAR_J6
};
static const float s_dir[ODRIVE_NUM_JOINTS] = {
    ODRIVE_DIR_J1, ODRIVE_DIR_J2, ODRIVE_DIR_J3,
    ODRIVE_DIR_J4, ODRIVE_DIR_J5, ODRIVE_DIR_J6
};

static float s_odrive_zero[ODRIVE_NUM_JOINTS] = {};

static inline uint32_t odriveCOBID(uint8_t node_id, uint8_t cmd_id) {
    return ((uint32_t)node_id << 5) | cmd_id;
}

static inline void putFloat32LE(uint8_t* buf, float v) {
    uint32_t raw;
    memcpy(&raw, &v, 4);
    buf[0] = (raw >>  0) & 0xFF;
    buf[1] = (raw >>  8) & 0xFF;
    buf[2] = (raw >> 16) & 0xFF;
    buf[3] = (raw >> 24) & 0xFF;
}

static inline void putUint32LE(uint8_t* buf, uint32_t v) {
    buf[0] = (v >>  0) & 0xFF;
    buf[1] = (v >>  8) & 0xFF;
    buf[2] = (v >> 16) & 0xFF;
    buf[3] = (v >> 24) & 0xFF;
}

static inline float getFloat32LE(const uint8_t* buf) {
    uint32_t raw = (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)
                 | ((uint32_t)buf[3] << 24);
    float v;
    memcpy(&v, &raw, 4);
    return v;
}

static void odriveSendAxisState(uint8_t node_id, uint32_t state) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_SET_AXIS_STATE);
    msg.data_length_code = 4;
    putUint32LE(msg.data, state);
    twaiSend(msg);
}

static bool odriveSendInputPos(uint8_t node_id, float turns) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_SET_INPUT_POS);
    msg.data_length_code = 8;
    putFloat32LE(msg.data, turns);
    msg.data[4] = 0; msg.data[5] = 0;   // vel_ff   = 0 (int16 LE)
    msg.data[6] = 0; msg.data[7] = 0;   // torque_ff = 0 (int16 LE)
    return twaiSend(msg);
}

static bool odriveReadEncoderZero(uint8_t node_id, float& out_turns) {
    // Send RTR (remote frame request)
    twai_message_t rtr = {};
    rtr.extd       = 0;
    rtr.rtr        = 1;
    rtr.identifier = odriveCOBID(node_id, ODRIVE_CMD_GET_ENCODER_EST);
    rtr.data_length_code = 8;   // GET_ENCODER_EST response is float32 pos + float32 vel
    twaiSend(rtr);

    uint32_t deadline = millis() + ODRIVE_ZERO_TIMEOUT_MS;
    while (millis() < deadline) {
        twai_message_t rx;
        if (twai_receive(&rx, 0) == ESP_OK) {
            if (!rx.rtr
                && rx.identifier == odriveCOBID(node_id, ODRIVE_CMD_GET_ENCODER_EST)
                && rx.data_length_code >= 4) {
                out_turns = getFloat32LE(rx.data);
                return true;
            }
        }
    }
    return false;
}

#endif  // ROBOT_SECONDARY

// ─── Public API ───────────────────────────────────────────────────────────────

bool CANInterface::begin() {
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_cfg, &t_cfg, &f_cfg) != ESP_OK) {
        s_ok = false;
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        s_ok = false;
        return false;
    }
    s_ok = true;

#ifdef ROBOT_SECONDARY
    // ── ODrive arm startup ─────────────────────────────────────────────────────
    // 1. Put all joints into closed-loop control.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        odriveSendAxisState(s_node[j], 8 /*CLOSED_LOOP_CONTROL*/);
    }
    // 2. Allow ODrives to enter closed-loop (≥50 ms per ginkgo_odrive_bridge).
    delay(50);
    // 3. Capture encoder zeros so the arm holds its current pose on startup.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float zero = 0.0f;
        odriveReadEncoderZero(s_node[j], zero);  // stays 0.0 on timeout
        s_odrive_zero[j] = zero;
    }
#endif

    return true;
}

bool CANInterface::sendArmJoints(const float angles_deg[6]) {
    if (!s_ok) return false;

#ifdef ROBOT_SECONDARY
    static constexpr float TWO_PI = 6.28318530718f;
    bool ok = true;
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float rad   = angles_deg[j] * (3.14159265359f / 180.0f);
        float turns = s_odrive_zero[j] + (rad / TWO_PI) * s_gear[j] * s_dir[j];
        ok &= odriveSendInputPos(s_node[j], turns);
    }
    return ok;
#else
    // ROBOT_MAIN: arm is handled by its dedicated ESP32 — nothing to relay here.
    (void)angles_deg;
    return true;
#endif
}

bool CANInterface::sendTrackSpeeds(float left_norm, float right_norm) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
    bool ok = vescSendCurrent(VESC_ID_TRACK_LEFT,  left_norm  * VESC_TRACK_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_TRACK_RIGHT, right_norm * VESC_TRACK_I_MAX_A);
    return ok;
#else
    (void)left_norm; (void)right_norm;
    return false;
#endif
}

bool CANInterface::sendFlipperSpeeds(float fl, float fr, float rl, float rr) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
    bool ok = vescSendCurrent(VESC_ID_FLIPPER_FL, fl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_FR, fr * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_RL, rl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_RR, rr * VESC_FLIPPER_I_MAX_A);
    return ok;
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
    return false;
#endif
}

void CANInterface::poll() {
    if (!s_ok) return;

    twai_message_t frame;
    while (twai_receive(&frame, 0) == ESP_OK) {
        // TODO: handle incoming ODrive status / fault frames
        (void)frame;
    }
}

bool CANInterface::isOk() {
    return s_ok;
}
