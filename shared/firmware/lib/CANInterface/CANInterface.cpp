#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include "driver/twai.h"

// ─── TWAI state ───────────────────────────────────────────────────────────────
static bool s_ok = false;

static bool twaiSend(twai_message_t& msg) {
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

// ─── VESC CAN helpers ─────────────────────────────────────────────────────────
// Standard VESC extended-frame protocol.
// Extended ID = (command << 8) | controller_id.

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

static inline int32_t getInt32BE(const uint8_t* d) {
    return ((int32_t)d[0] << 24) | ((int32_t)d[1] << 16) |
           ((int32_t)d[2] << 8)  | (int32_t)d[3];
}

static inline int16_t getInt16BE(const uint8_t* d) {
    return ((int16_t)d[0] << 8) | d[1];
}

static bool vescSendCurrent(uint8_t ctrl_id, float current_a) {
    twai_message_t msg = {};
    msg.extd       = 1;
    msg.identifier = vescEID(ctrl_id, VESC_CMD_SET_CURRENT);
    msg.data_length_code = 4;
    putInt32BE(msg.data, static_cast<int32_t>(current_a * 1000.0f));
    return twaiSend(msg);
}

// ─── VESC feedback storage ───────────────────────────────────────────────────
// Indexed by (vesc_id - 1).  Only used on ROBOT_SECONDARY but compiled always
// to keep the interface uniform (the poll() function is a no-op on ROBOT_MAIN).

struct VescFeedback {
    int32_t erpm;
    int16_t current_10;       // A × 10
    int16_t duty_1000;        // × 1000
    int16_t temp_fet_10;      // °C × 10
    int16_t temp_motor_10;    // °C × 10
    int16_t v_in_10;          // V × 10
    bool    fresh;            // set on new data, cleared by getVescStatus
};

static VescFeedback s_vesc[VESC_MAX_CONTROLLERS] = {};
static portMUX_TYPE s_vesc_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── ODrive CAN helpers ──────────────────────────────────────────────────────
// Standard 11-bit frames, little-endian payloads.
// COB-ID = (node_id << 5) | cmd_id.

static constexpr uint8_t ODRIVE_CMD_SET_AXIS_STATE  = 0x07;
static constexpr uint8_t ODRIVE_CMD_GET_ENCODER_EST = 0x09;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_POS   = 0x0C;

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
    msg.data[4] = 0; msg.data[5] = 0;   // vel_ff   = 0
    msg.data[6] = 0; msg.data[7] = 0;   // torque_ff = 0
    return twaiSend(msg);
}

static bool odriveReadEncoderZero(uint8_t node_id, float& out_turns) {
    twai_message_t rtr = {};
    rtr.extd       = 0;
    rtr.rtr        = 1;
    rtr.identifier = odriveCOBID(node_id, ODRIVE_CMD_GET_ENCODER_EST);
    rtr.data_length_code = 8;
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

// ─── ODrive joint tables (shared by both robots) ─────────────────────────────

#ifdef ROBOT_MAIN
static constexpr uint8_t ODRIVE_NUM_JOINTS = ODRIVE_MAIN_NUM_JOINTS;  // 3
#else
static constexpr uint8_t ODRIVE_NUM_JOINTS = 6;
#endif

static const uint8_t s_node[ODRIVE_NUM_JOINTS] = {
    ODRIVE_NODE_J1, ODRIVE_NODE_J2, ODRIVE_NODE_J3,
#ifdef ROBOT_SECONDARY
    ODRIVE_NODE_J4, ODRIVE_NODE_J5, ODRIVE_NODE_J6
#endif
};
static const float s_gear[ODRIVE_NUM_JOINTS] = {
    ODRIVE_GEAR_J1, ODRIVE_GEAR_J2, ODRIVE_GEAR_J3,
#ifdef ROBOT_SECONDARY
    ODRIVE_GEAR_J4, ODRIVE_GEAR_J5, ODRIVE_GEAR_J6
#endif
};
static const float s_dir[ODRIVE_NUM_JOINTS] = {
    ODRIVE_DIR_J1, ODRIVE_DIR_J2, ODRIVE_DIR_J3,
#ifdef ROBOT_SECONDARY
    ODRIVE_DIR_J4, ODRIVE_DIR_J5, ODRIVE_DIR_J6
#endif
};

static float s_odrive_zero[ODRIVE_NUM_JOINTS] = {};

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

    // ── ODrive arm startup (both robots) ──────────────────────────────────────
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        odriveSendAxisState(s_node[j], 8 /*CLOSED_LOOP_CONTROL*/);
    }
    delay(50);
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float zero = 0.0f;
        odriveReadEncoderZero(s_node[j], zero);
        s_odrive_zero[j] = zero;
    }

    return true;
}

bool CANInterface::sendArmJoints(const float angles_deg[6]) {
    if (!s_ok) return false;

    static constexpr float TWO_PI_F = 6.28318530718f;
    bool ok = true;
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float rad   = angles_deg[j] * (3.14159265359f / 180.0f);
        float turns = s_odrive_zero[j] + (rad / TWO_PI_F) * s_gear[j] * s_dir[j];
        ok &= odriveSendInputPos(s_node[j], turns);
    }
    return ok;
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
        // VESC status frames use extended IDs: cmd = (id >> 8), vesc_id = (id & 0xFF)
        if (frame.extd && frame.data_length_code == 8) {
            uint8_t vesc_id = frame.identifier & 0xFF;
            uint8_t cmd     = (frame.identifier >> 8) & 0xFF;

            if (vesc_id < 1 || vesc_id > VESC_MAX_CONTROLLERS) continue;
            uint8_t idx = vesc_id - 1;

            portENTER_CRITICAL(&s_vesc_mux);
            switch (cmd) {
                case 9:   // Status 1: eRPM, current, duty
                    s_vesc[idx].erpm        = getInt32BE(frame.data);
                    s_vesc[idx].current_10  = getInt16BE(frame.data + 4);  // already × 10
                    s_vesc[idx].duty_1000   = getInt16BE(frame.data + 6);
                    s_vesc[idx].fresh       = true;
                    break;
                case 16:  // Status 4: temps, current_in
                    s_vesc[idx].temp_fet_10   = getInt16BE(frame.data);
                    s_vesc[idx].temp_motor_10 = getInt16BE(frame.data + 2);
                    s_vesc[idx].fresh         = true;
                    break;
                case 27:  // Status 5: tachometer, voltage
                    s_vesc[idx].v_in_10 = getInt16BE(frame.data + 4);
                    s_vesc[idx].fresh   = true;
                    break;
                default:
                    break;
            }
            portEXIT_CRITICAL(&s_vesc_mux);
        }
    }
}

bool CANInterface::getVescStatus(uint8_t vesc_id, VescStatusPayload& out) {
    if (vesc_id < 1 || vesc_id > VESC_MAX_CONTROLLERS) return false;
    uint8_t idx = vesc_id - 1;

    portENTER_CRITICAL(&s_vesc_mux);
    bool have = s_vesc[idx].fresh;
    if (have) {
        out.vesc_id       = vesc_id;
        out.erpm          = s_vesc[idx].erpm;
        out.current_10    = s_vesc[idx].current_10;
        out.duty_1000     = s_vesc[idx].duty_1000;
        out.temp_fet_10   = s_vesc[idx].temp_fet_10;
        out.temp_motor_10 = s_vesc[idx].temp_motor_10;
        out.v_in_10       = s_vesc[idx].v_in_10;
        s_vesc[idx].fresh = false;
    }
    portEXIT_CRITICAL(&s_vesc_mux);
    return have;
}

bool CANInterface::isOk() {
    return s_ok;
}
