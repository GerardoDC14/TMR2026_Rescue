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
static constexpr uint8_t VESC_CMD_SET_RPM     = 3;

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

static bool vescSendRpm(uint8_t ctrl_id, int32_t erpm) {
    twai_message_t msg = {};
    msg.extd       = 1;
    msg.identifier = vescEID(ctrl_id, VESC_CMD_SET_RPM);
    msg.data_length_code = 4;
    putInt32BE(msg.data, erpm);
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

static constexpr uint8_t ODRIVE_CMD_HEARTBEAT         = 0x01;
static constexpr uint8_t ODRIVE_CMD_ESTOP              = 0x02;
static constexpr uint8_t ODRIVE_CMD_GET_ERROR          = 0x03;
static constexpr uint8_t ODRIVE_CMD_SET_AXIS_STATE     = 0x07;
static constexpr uint8_t ODRIVE_CMD_GET_ENCODER_EST    = 0x09;
static constexpr uint8_t ODRIVE_CMD_SET_CONTROLLER_MODE = 0x0B;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_POS      = 0x0C;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_VEL      = 0x0D;
static constexpr uint8_t ODRIVE_CMD_GET_IQ             = 0x14;
// Note: 0x15 = Get_Sensorless_Estimates on v3.6, NOT temperature.
// Temperature is not available via CAN on v3.6.
static constexpr uint8_t ODRIVE_CMD_GET_BUS_VOLTAGE    = 0x17;
static constexpr uint8_t ODRIVE_CMD_CLEAR_ERRORS       = 0x18;

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

static bool odriveSendInputVel(uint8_t node_id, float turns_per_sec) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_SET_INPUT_VEL);
    msg.data_length_code = 8;
    putFloat32LE(msg.data, turns_per_sec);
    msg.data[4] = 0; msg.data[5] = 0;   // torque_ff  = 0
    msg.data[6] = 0; msg.data[7] = 0;
    return twaiSend(msg);
}

static void odriveSendClearErrors(uint8_t node_id) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_CLEAR_ERRORS);
    msg.data_length_code = 0;   // v3.6: DLC=0 (no identify byte)
    twaiSend(msg);
}

// ODrive native E-stop (cmd 0x02).  Forces axis to IDLE immediately.
// To recover: Clear_Errors + Set_Axis_State(CLOSED_LOOP).
static void odriveSendEstop(uint8_t node_id) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_ESTOP);
    msg.data_length_code = 0;
    twaiSend(msg);
}

static void odriveSendControllerMode(uint8_t node_id,
                                      uint32_t control_mode,
                                      uint32_t input_mode) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = odriveCOBID(node_id, ODRIVE_CMD_SET_CONTROLLER_MODE);
    msg.data_length_code = 8;
    putUint32LE(msg.data, control_mode);
    putUint32LE(msg.data + 4, input_mode);
    twaiSend(msg);
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

// ─── ODrive feedback storage ─────────────────────────────────────────────────
// Populated by parsing standard-frame responses to RTR telemetry requests.

struct OdriveFeedback {
    float pos_turns;
    float vel_turns_s;
    float iq_setpoint_a;
    float iq_measured_a;
    // No temperature available via CAN on ODrive v3.6.
    float bus_voltage_v;
    float bus_current_a;
    bool  fresh;
};

static OdriveFeedback s_odrive_fb[ODRIVE_MAX_JOINTS] = {};
static portMUX_TYPE   s_odrive_mux = portMUX_INITIALIZER_UNLOCKED;

// Round-robin state for periodic ODrive ARM telemetry RTR requests.
// Note: no temperature endpoint on v3.6 (0x15 is sensorless, not temp).
static const uint8_t s_odrive_telem_cmds[] = {
    ODRIVE_CMD_GET_ENCODER_EST,
    ODRIVE_CMD_GET_IQ,
    ODRIVE_CMD_GET_BUS_VOLTAGE,
#if ODRIVE_ENABLE_ERROR_POLL
    ODRIVE_CMD_GET_ERROR,
#endif
};
static constexpr uint8_t ODRIVE_TELEM_CMD_COUNT =
    sizeof(s_odrive_telem_cmds) / sizeof(s_odrive_telem_cmds[0]);
static uint8_t s_odrive_telem_slot = 0;

// ─── ODrive error feedback (shared between arm + traction) ──────────────────
// Indexed by a flat "node slot" — arm slots 0..2 always present, traction
// slots 3..4 exist only on ROBOT_SECONDARY.
#if ODRIVE_ENABLE_ERROR_POLL
struct OdriveErrorFb {
    uint8_t  node_id;
    uint64_t motor_error;    // v3.6: single u64 from Get_Motor_Error (0x03)
    bool     fresh;
};
static constexpr uint8_t ODRIVE_ERROR_SLOT_COUNT =
    ODRIVE_MAX_JOINTS
#ifdef ROBOT_SECONDARY
    + TRACTION_ODRIVE_NUM
#endif
    ;
static OdriveErrorFb s_odrive_err[ODRIVE_ERROR_SLOT_COUNT] = {};
static portMUX_TYPE  s_odrive_err_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

// ─── Traction ODrive (ROBOT_SECONDARY only) ─────────────────────────────────
#ifdef ROBOT_SECONDARY

static const uint8_t s_traction_node[TRACTION_ODRIVE_NUM] = {
    TRACTION_NODE_LEFT, TRACTION_NODE_RIGHT
};
static const float s_traction_dir[TRACTION_ODRIVE_NUM] = {
    TRACTION_DIR_LEFT, TRACTION_DIR_RIGHT
};

static OdriveFeedback s_traction_fb[TRACTION_ODRIVE_NUM] = {};
static portMUX_TYPE   s_traction_mux = portMUX_INITIALIZER_UNLOCKED;

// Separate round-robin for traction telemetry.
static uint8_t s_traction_telem_slot = 0;

// Blocking handshake: Clear_Errors → Set_Controller_Mode(VELOCITY, VEL_RAMP)
// → Set_Axis_State(CLOSED_LOOP) → poll heartbeat until confirmed or timeout.
static bool tractionOdriveStartup(uint8_t node_id) {
    odriveSendClearErrors(node_id);
    delay(5);
    odriveSendControllerMode(node_id,
                             2 /*VELOCITY_CONTROL*/,
                             2 /*VEL_RAMP*/);
    delay(5);
    odriveSendAxisState(node_id, 8 /*CLOSED_LOOP_CONTROL*/);

    const uint32_t deadline = millis() + TRACTION_CLOSED_LOOP_TIMEOUT_MS;
    uint32_t last_retry = millis();

    while (millis() < deadline) {
        twai_message_t rx;
        if (twai_receive(&rx, 0) == ESP_OK) {
            if (!rx.rtr && !rx.extd && rx.data_length_code >= 5) {
                uint8_t n = (rx.identifier >> 5) & 0x3F;
                uint8_t c = rx.identifier & 0x1F;
                if (n == node_id && c == ODRIVE_CMD_HEARTBEAT) {
                    uint8_t axis_state = rx.data[4];
                    uint32_t err = (uint32_t)rx.data[0]
                                 | ((uint32_t)rx.data[1] << 8)
                                 | ((uint32_t)rx.data[2] << 16)
                                 | ((uint32_t)rx.data[3] << 24);
                    if (axis_state == 8 && err == 0) return true;
                }
            }
        }
        if (millis() - last_retry >= TRACTION_CLOSED_LOOP_RETRY_MS) {
            last_retry = millis();
            odriveSendAxisState(node_id, 8);
        }
    }
    return false;
}

#endif  // ROBOT_SECONDARY (traction)

// ─── LKTech CAN helpers (ROBOT_SECONDARY J5–J6) ─────────────────────────────
// Standard 11-bit frames.  Request and reply share ID = LKTECH_ID_BASE + motor_id.
// Command byte is data[0].  All payloads are 8 bytes.
// Protocol V2.36 — confirmed via tools/arm/bldc_can_tools.

#ifdef ROBOT_SECONDARY

static constexpr uint8_t LKTECH_CMD_MOTOR_ON               = 0x88;
static constexpr uint8_t LKTECH_CMD_MOTOR_OFF              = 0x80;
static constexpr uint8_t LKTECH_CMD_MOTOR_STOP             = 0x81;
static constexpr uint8_t LKTECH_CMD_READ_MULTI_LOOP_ANGLE  = 0x92;
static constexpr uint8_t LKTECH_CMD_MULTI_LOOP_CONTROL_2   = 0xA4;

static bool lktechSendRaw(uint8_t motor_id, const uint8_t payload[8]) {
    twai_message_t msg = {};
    msg.extd       = 0;
    msg.identifier = LKTECH_ID_BASE + motor_id;
    msg.data_length_code = 8;
    memcpy(msg.data, payload, 8);
    return twaiSend(msg);
}

static bool lktechMotorOn(uint8_t motor_id) {
    uint8_t payload[8] = { LKTECH_CMD_MOTOR_ON };
    return lktechSendRaw(motor_id, payload);
}

// Multi-loop angle control (cmd 0xA4): target in centidegrees, speed limit °/s.
static bool lktechSendPosition(uint8_t motor_id, int32_t angle_cdeg,
                                uint16_t max_speed_dps) {
    uint8_t payload[8] = {};
    payload[0] = LKTECH_CMD_MULTI_LOOP_CONTROL_2;
    payload[1] = 0x00;
    payload[2] = (max_speed_dps >> 0) & 0xFF;
    payload[3] = (max_speed_dps >> 8) & 0xFF;
    payload[4] = (angle_cdeg >>  0) & 0xFF;
    payload[5] = (angle_cdeg >>  8) & 0xFF;
    payload[6] = (angle_cdeg >> 16) & 0xFF;
    payload[7] = (angle_cdeg >> 24) & 0xFF;
    return lktechSendRaw(motor_id, payload);
}

// Blocking: send READ_MULTI_LOOP_ANGLE (0x92), wait for reply, parse signed
// 56-bit LE centidegree value from data[1..7].  Returns true on success.
static bool lktechReadMultiLoopAngle(uint8_t motor_id, int64_t& out_cdeg) {
    uint8_t payload[8] = { LKTECH_CMD_READ_MULTI_LOOP_ANGLE };
    if (!lktechSendRaw(motor_id, payload)) return false;

    const uint32_t deadline = millis() + LKTECH_ZERO_TIMEOUT_MS;
    while (millis() < deadline) {
        twai_message_t rx;
        if (twai_receive(&rx, 0) != ESP_OK) continue;
        if (rx.rtr || rx.extd) continue;
        if (rx.identifier != (uint32_t)(LKTECH_ID_BASE + motor_id)) continue;
        if (rx.data_length_code < 8) continue;
        if (rx.data[0] != LKTECH_CMD_READ_MULTI_LOOP_ANGLE) continue;

        // Assemble signed 56-bit LE from data[1..7], sign-extend via shift.
        uint64_t raw = 0;
        for (int i = 0; i < 7; ++i) {
            raw |= ((uint64_t)rx.data[1 + i]) << (8 * i);
        }
        // Sign-extend from bit 55 to 64.
        int64_t sraw = (int64_t)(raw << 8) >> 8;
        out_cdeg = sraw;
        return true;
    }
    return false;
}

static constexpr uint8_t LKTECH_NUM_JOINTS = LKTECH_MAX_JOINTS;   // 2 (J5, J6)

static const uint8_t s_lktech_id[LKTECH_NUM_JOINTS] = {
    LKTECH_ID_J5, LKTECH_ID_J6
};
static const float s_lktech_gear[LKTECH_NUM_JOINTS] = {
    LKTECH_GEAR_J5, LKTECH_GEAR_J6
};
static const float s_lktech_dir[LKTECH_NUM_JOINTS] = {
    LKTECH_DIR_J5, LKTECH_DIR_J6
};

// Boot-pose software zero (motor-side centidegrees).
static int64_t s_lktech_zero_cdeg[LKTECH_NUM_JOINTS] = {};

// ─── LKTech feedback storage ─────────────────────────────────────────────────
struct LktechFeedback {
    int8_t  temp_c;
    int16_t iq_100;        // A × 100 per V2.36 protocol
    int16_t speed_dps;
    int16_t angle_deg;
    float   output_deg;    // software-zeroed, gear-compensated
    bool    fresh;
};
static LktechFeedback s_lktech_fb[LKTECH_NUM_JOINTS] = {};
static portMUX_TYPE   s_lktech_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── ZE300 CAN helpers (ROBOT_SECONDARY J4) ─────────────────────────────────
// Standard 11-bit frames.  Tagged request ID = ZE300_REQ_ID_BASE | device_id.
// Reply ID = device_id (no tag bit).  Variable DLC.
// Position units: output counts at ZE300_COUNTS_PER_REV (1:1 output-to-command —
// the driver handles the 1:8 gearbox internally).

static constexpr uint8_t ZE_CMD_READ_ABSOLUTE_ANGLES   = 0xA3;
static constexpr uint8_t ZE_CMD_READ_REALTIME_STATE    = 0xA4;
static constexpr uint8_t ZE_CMD_SET_POSITION_MAX_SPEED = 0xB2;
static constexpr uint8_t ZE_CMD_ABSOLUTE_POSITION      = 0xC2;
static constexpr uint8_t ZE_CMD_DISABLE_OUTPUT         = 0xCF;

static inline uint32_t ze300ReqId(uint8_t device_id) {
    return ZE300_REQ_ID_BASE | device_id;
}
static inline uint32_t ze300ReplyId(uint8_t device_id) {
    return device_id;
}

static inline void putInt32LE(uint8_t* d, int32_t v) {
    d[0] = (uint8_t)(v >>  0);
    d[1] = (uint8_t)(v >>  8);
    d[2] = (uint8_t)(v >> 16);
    d[3] = (uint8_t)(v >> 24);
}
static inline int32_t getInt32LE(const uint8_t* d) {
    return  (int32_t)d[0]
         | ((int32_t)d[1] <<  8)
         | ((int32_t)d[2] << 16)
         | ((int32_t)d[3] << 24);
}
static inline int16_t getInt16LE(const uint8_t* d) {
    return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
}
static inline uint16_t getUint16LE(const uint8_t* d) {
    return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}

// Blocking: set position max-speed on ZE300 (0xB2, 5-byte TX + 5-byte reply).
static bool ze300SetPositionMaxSpeed(uint8_t device_id, int32_t centi_rpm) {
    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = ze300ReqId(device_id);
    msg.data_length_code = 5;
    msg.data[0] = ZE_CMD_SET_POSITION_MAX_SPEED;
    putInt32LE(msg.data + 1, centi_rpm);
    if (!twaiSend(msg)) return false;

    const uint32_t deadline = millis() + ZE300_ZERO_TIMEOUT_MS;
    while (millis() < deadline) {
        twai_message_t rx;
        if (twai_receive(&rx, 0) != ESP_OK) continue;
        if (rx.rtr || rx.extd) continue;
        if (rx.identifier != ze300ReplyId(device_id)) continue;
        if (rx.data_length_code < 1) continue;
        if (rx.data[0] == ZE_CMD_SET_POSITION_MAX_SPEED) return true;
    }
    return false;
}

// Blocking: read absolute angles (0xA3, 1-byte TX + 7-byte reply).
// Parses multi-turn counts from data[3..6] (i32 LE), writes to out_counts.
static bool ze300ReadAbsoluteAngles(uint8_t device_id, int32_t& out_counts) {
    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = ze300ReqId(device_id);
    msg.data_length_code = 1;
    msg.data[0] = ZE_CMD_READ_ABSOLUTE_ANGLES;
    if (!twaiSend(msg)) return false;

    const uint32_t deadline = millis() + ZE300_ZERO_TIMEOUT_MS;
    while (millis() < deadline) {
        twai_message_t rx;
        if (twai_receive(&rx, 0) != ESP_OK) continue;
        if (rx.rtr || rx.extd) continue;
        if (rx.identifier != ze300ReplyId(device_id)) continue;
        if (rx.data_length_code < 7) continue;
        if (rx.data[0] != ZE_CMD_READ_ABSOLUTE_ANGLES) continue;
        out_counts = getInt32LE(rx.data + 3);
        return true;
    }
    return false;
}

// Fire-and-forget: ABSOLUTE_POSITION (0xC2, 5-byte TX).  Target is in output
// counts relative to mechanical zero; we offset by the startup zero snapshot.
static bool ze300SendAbsolutePosition(uint8_t device_id, int32_t motor_counts) {
    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = ze300ReqId(device_id);
    msg.data_length_code = 5;
    msg.data[0] = ZE_CMD_ABSOLUTE_POSITION;
    putInt32LE(msg.data + 1, motor_counts);
    return twaiSend(msg);
}

static bool ze300SendDisableOutput(uint8_t device_id) {
    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = ze300ReqId(device_id);
    msg.data_length_code = 1;
    msg.data[0] = ZE_CMD_DISABLE_OUTPUT;
    return twaiSend(msg);
}

// Low-rate active poll of 0xA4 realtime state (7-byte TX command, reply parsed
// passively in poll()).  Sent as a plain 1-byte command — driver replies
// with the command byte + state payload on the reply ID.
static bool ze300SendReadRealtimeState(uint8_t device_id) {
    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = ze300ReqId(device_id);
    msg.data_length_code = 1;
    msg.data[0] = ZE_CMD_READ_REALTIME_STATE;
    return twaiSend(msg);
}

// Boot-pose software zero in output counts.
static int32_t s_ze300_zero_counts = 0;

struct Ze300Feedback {
    int8_t  temp_c;
    int16_t iq_1000;         // A × 1000 per protocol
    int16_t speed_rpm_100;   // rpm × 100 per protocol
    uint16_t single_turn;    // raw single-turn counts
    int32_t position_counts; // multi-turn, from C2 replies
    bool    fresh;
};
static Ze300Feedback s_ze300_fb = {};
static portMUX_TYPE  s_ze300_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_ze300_last_telem_req_ms = 0;

#endif  // ROBOT_SECONDARY

// ─── ODrive joint tables (J1–J3, both robots) ───────────────────────────────

static constexpr uint8_t ODRIVE_NUM_JOINTS = ODRIVE_MAX_JOINTS;  // 3

static const uint8_t s_node[ODRIVE_NUM_JOINTS] = {
    ODRIVE_NODE_J1, ODRIVE_NODE_J2, ODRIVE_NODE_J3
};
static const float s_gear[ODRIVE_NUM_JOINTS] = {
    ODRIVE_GEAR_J1, ODRIVE_GEAR_J2, ODRIVE_GEAR_J3
};
static const float s_dir[ODRIVE_NUM_JOINTS] = {
    ODRIVE_DIR_J1, ODRIVE_DIR_J2, ODRIVE_DIR_J3
};

static float s_odrive_zero[ODRIVE_NUM_JOINTS] = {};

// ─── Public API ───────────────────────────────────────────────────────────────

bool CANInterface::begin() {
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    // 1 Mbps — shared bus for ODrive, VESC, LKTech, ZE300.
    // Every controller must be pre-configured to 1 Mbps before flashing.
    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_1MBITS();
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

    // ── ODrive arm startup (J1–J3, both robots) ────────────────────────────────
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        odriveSendAxisState(s_node[j], 8 /*CLOSED_LOOP_CONTROL*/);
    }
    delay(50);
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float zero = 0.0f;
        odriveReadEncoderZero(s_node[j], zero);
        s_odrive_zero[j] = zero;
    }

#ifdef ROBOT_SECONDARY
    // ── LKTech arm startup (J5–J6) ──────────────────────────────────────────
    // Enable the motor, then capture boot pose as software zero via 0x92.
    for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
        lktechMotorOn(s_lktech_id[j]);
    }
    delay(10);
    for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
        int64_t zero_cdeg = 0;
        lktechReadMultiLoopAngle(s_lktech_id[j], zero_cdeg);
        s_lktech_zero_cdeg[j] = zero_cdeg;
    }

    // ── ZE300 arm startup (J4) ─────────────────────────────────────────────
    // 1) push position max speed, 2) read absolute angles → store zero offset.
    ze300SetPositionMaxSpeed(ZE300_ID_J4, ZE300_MAX_SPEED_CRPM);
    {
        int32_t counts = 0;
        if (ze300ReadAbsoluteAngles(ZE300_ID_J4, counts)) {
            s_ze300_zero_counts = counts;
        }
    }

    // ── Traction ODrive startup (left + right) ─────────────────────────────
    // Full handshake: ClearErrors → SetControllerMode(VELOCITY,VEL_RAMP) →
    // SetAxisState(CLOSED_LOOP) with heartbeat confirmation.
    for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++) {
        tractionOdriveStartup(s_traction_node[m]);
    }
#endif

    return true;
}

bool CANInterface::sendArmJoints(const float angles_deg[6]) {
    if (!s_ok) return false;

    static constexpr float TWO_PI_F = 6.28318530718f;
    bool ok = true;

    // J1–J3: ODrive (both robots) — fire-and-forget SET_INPUT_POS.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float rad   = angles_deg[j] * (3.14159265359f / 180.0f);
        float turns = s_odrive_zero[j] + (rad / TWO_PI_F) * s_gear[j] * s_dir[j];
        ok &= odriveSendInputPos(s_node[j], turns);
    }

#ifdef ROBOT_SECONDARY
    // J4: ZE300 ABSOLUTE_POSITION — counts = zero_offset + deg × cnt/rev / 360.
    // The driver handles the 1:8 gearbox internally (POSITION_COMMAND_RATIO = 1:1),
    // so we command directly in output degrees.
    {
        float out_deg = angles_deg[3] * ZE300_DIR_J4;
        int32_t motor_counts = s_ze300_zero_counts +
            static_cast<int32_t>(out_deg * (float)ZE300_COUNTS_PER_REV / 360.0f);
        ok &= ze300SendAbsolutePosition(ZE300_ID_J4, motor_counts);
    }

    // J5–J6: LKTech multi-loop angle control (fire-and-forget).
    // Convert output degrees → motor centidegrees via gear ratio × direction,
    // then offset by the boot-pose zero snapshot so current pose = software zero.
    for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
        float motor_deg = angles_deg[4 + j] * s_lktech_gear[j] * s_lktech_dir[j];
        int64_t target_cdeg = s_lktech_zero_cdeg[j] +
            static_cast<int64_t>(motor_deg * 100.0f);
        ok &= lktechSendPosition(s_lktech_id[j],
                                 static_cast<int32_t>(target_cdeg),
                                 LKTECH_DEFAULT_SPEED_DPS);
    }
#endif

    return ok;
}

bool CANInterface::sendTrackSpeeds(float left_norm, float right_norm) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
    // Traction ODrives — native velocity control (Set_Input_Vel, turns/s).
    float l_tps = left_norm  * TRACTION_MAX_VEL_TURNS_S * TRACTION_DIR_LEFT;
    float r_tps = right_norm * TRACTION_MAX_VEL_TURNS_S * TRACTION_DIR_RIGHT;
    bool ok = odriveSendInputVel(TRACTION_NODE_LEFT,  l_tps);
    ok     &= odriveSendInputVel(TRACTION_NODE_RIGHT, r_tps);
    return ok;
#else
    (void)left_norm; (void)right_norm;
    return false;
#endif
}

bool CANInterface::sendFlipperSpeeds(float fl, float fr, float rl, float rr) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
  #if VESC_FLIPPER_USE_RPM
    // VESC native velocity mode — SET_RPM (cmd 3).
    bool ok = vescSendRpm(VESC_ID_FLIPPER_FL, static_cast<int32_t>(fl * VESC_FLIPPER_ERPM_MAX));
    ok     &= vescSendRpm(VESC_ID_FLIPPER_FR, static_cast<int32_t>(fr * VESC_FLIPPER_ERPM_MAX));
    ok     &= vescSendRpm(VESC_ID_FLIPPER_RL, static_cast<int32_t>(rl * VESC_FLIPPER_ERPM_MAX));
    ok     &= vescSendRpm(VESC_ID_FLIPPER_RR, static_cast<int32_t>(rr * VESC_FLIPPER_ERPM_MAX));
  #else
    // Fallback — VESC current/torque mode — SET_CURRENT (cmd 1).
    bool ok = vescSendCurrent(VESC_ID_FLIPPER_FL, fl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_FR, fr * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_RL, rl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(VESC_ID_FLIPPER_RR, rr * VESC_FLIPPER_I_MAX_A);
  #endif
    return ok;
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
    return false;
#endif
}

void CANInterface::poll() {
    if (!s_ok) return;

    // ── Drain RX queue: parse VESC (extended), ODrive + LKTech + ZE300 (std) ─
    twai_message_t frame;
    while (twai_receive(&frame, 0) == ESP_OK) {
        if (frame.rtr) continue;

        // ── VESC status: extended frames, DLC 8 ─────────────────────────────
        if (frame.extd && frame.data_length_code == 8) {
            uint8_t vesc_id = frame.identifier & 0xFF;
            uint8_t cmd     = (frame.identifier >> 8) & 0xFF;

            if (vesc_id < 1 || vesc_id > VESC_MAX_CONTROLLERS) continue;
            uint8_t idx = vesc_id - 1;

            portENTER_CRITICAL(&s_vesc_mux);
            switch (cmd) {
                case 9:   // Status 1: eRPM, current, duty
                    s_vesc[idx].erpm        = getInt32BE(frame.data);
                    s_vesc[idx].current_10  = getInt16BE(frame.data + 4);
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
            continue;
        }

        // ── ODrive telemetry responses: standard frames, DLC ≥ 8 ────────────
        if (!frame.extd && frame.data_length_code >= 8) {
            uint8_t node_id = (frame.identifier >> 5) & 0x3F;
            uint8_t cmd_id  = frame.identifier & 0x1F;

            bool matched_odrive = false;
            for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
                if (s_node[j] != node_id) continue;
                matched_odrive = true;

                portENTER_CRITICAL(&s_odrive_mux);
                switch (cmd_id) {
                    case ODRIVE_CMD_GET_ENCODER_EST:
                        s_odrive_fb[j].pos_turns   = getFloat32LE(frame.data);
                        s_odrive_fb[j].vel_turns_s = getFloat32LE(frame.data + 4);
                        s_odrive_fb[j].fresh = true;
                        break;
                    case ODRIVE_CMD_GET_IQ:
                        s_odrive_fb[j].iq_setpoint_a = getFloat32LE(frame.data);
                        s_odrive_fb[j].iq_measured_a = getFloat32LE(frame.data + 4);
                        s_odrive_fb[j].fresh = true;
                        break;
                    case ODRIVE_CMD_GET_BUS_VOLTAGE:
                        s_odrive_fb[j].bus_voltage_v = getFloat32LE(frame.data);
                        s_odrive_fb[j].bus_current_a = getFloat32LE(frame.data + 4);
                        s_odrive_fb[j].fresh = true;
                        break;
#if ODRIVE_ENABLE_ERROR_POLL
                    case ODRIVE_CMD_GET_ERROR: {
                        // v3.6: Get_Motor_Error returns a single u64 LE.
                        uint64_t err64 = 0;
                        for (int b = 0; b < 8; b++)
                            err64 |= (uint64_t)frame.data[b] << (8 * b);
                        portENTER_CRITICAL(&s_odrive_err_mux);
                        s_odrive_err[j].node_id     = node_id;
                        s_odrive_err[j].motor_error = err64;
                        s_odrive_err[j].fresh       = true;
                        portEXIT_CRITICAL(&s_odrive_err_mux);
                        break;
                    }
#endif
                    default:
                        break;
                }
                portEXIT_CRITICAL(&s_odrive_mux);
                break;  // found the joint, stop searching
            }
            if (matched_odrive) continue;
        }

#ifdef ROBOT_SECONDARY
        // ── Traction ODrive frames (same protocol, separate node/fb tables) ──
        if (!frame.extd) {
            uint8_t node_id = (frame.identifier >> 5) & 0x3F;
            uint8_t cmd_id  = frame.identifier & 0x1F;

            for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++) {
                if (s_traction_node[m] != node_id) continue;

                if (cmd_id == ODRIVE_CMD_HEARTBEAT) {
                    // We don't store heartbeats in the telemetry struct;
                    // heartbeat is only used for the startup handshake.
                    // However, we DO parse axis_error from the heartbeat for the error path.
                    // Heartbeat: u32 axis_error (data[0..3]) + u8 current_state (data[4]).
#if ODRIVE_ENABLE_ERROR_POLL
                    if (frame.data_length_code >= 5) {
                        uint32_t axis_err = (uint32_t)frame.data[0]
                                          | ((uint32_t)frame.data[1] << 8)
                                          | ((uint32_t)frame.data[2] << 16)
                                          | ((uint32_t)frame.data[3] << 24);
                        if (axis_err != 0) {
                            uint8_t slot = ODRIVE_MAX_JOINTS + m;
                            portENTER_CRITICAL(&s_odrive_err_mux);
                            s_odrive_err[slot].node_id     = node_id;
                            s_odrive_err[slot].motor_error = (uint64_t)axis_err;
                            s_odrive_err[slot].fresh       = true;
                            portEXIT_CRITICAL(&s_odrive_err_mux);
                        }
                    }
#endif
                    break;
                }

                if (frame.data_length_code >= 8) {
                    portENTER_CRITICAL(&s_traction_mux);
                    switch (cmd_id) {
                        case ODRIVE_CMD_GET_ENCODER_EST:
                            s_traction_fb[m].pos_turns   = getFloat32LE(frame.data);
                            s_traction_fb[m].vel_turns_s = getFloat32LE(frame.data + 4);
                            s_traction_fb[m].fresh = true;
                            break;
                        case ODRIVE_CMD_GET_IQ:
                            s_traction_fb[m].iq_setpoint_a = getFloat32LE(frame.data);
                            s_traction_fb[m].iq_measured_a = getFloat32LE(frame.data + 4);
                            s_traction_fb[m].fresh = true;
                            break;
                        case ODRIVE_CMD_GET_BUS_VOLTAGE:
                            s_traction_fb[m].bus_voltage_v = getFloat32LE(frame.data);
                            s_traction_fb[m].bus_current_a = getFloat32LE(frame.data + 4);
                            s_traction_fb[m].fresh = true;
                            break;
#if ODRIVE_ENABLE_ERROR_POLL
                        case ODRIVE_CMD_GET_ERROR: {
                            // v3.6: Get_Motor_Error returns a single u64 LE.
                            uint64_t err64 = 0;
                            for (int b = 0; b < 8; b++)
                                err64 |= (uint64_t)frame.data[b] << (8 * b);
                            uint8_t slot = ODRIVE_MAX_JOINTS + m;
                            portENTER_CRITICAL(&s_odrive_err_mux);
                            s_odrive_err[slot].node_id     = node_id;
                            s_odrive_err[slot].motor_error = err64;
                            s_odrive_err[slot].fresh       = true;
                            portEXIT_CRITICAL(&s_odrive_err_mux);
                            break;
                        }
#endif
                        default:
                            break;
                    }
                    portEXIT_CRITICAL(&s_traction_mux);
                }
                break;  // found the traction motor
            }
        }
#endif  // traction ODrive parsing

#ifdef ROBOT_SECONDARY
        // ── LKTech replies: standard 11-bit, ID = 0x140 + motor_id, DLC 8 ────
        // The A4 control command echoes back a READ_STATE_2-like payload on
        // the same arbitration ID.  Layout: data[0]=cmd, data[1]=temp (i8),
        // data[2..3]=iq i16 LE /100 A, data[4..5]=speed i16 LE dps,
        // data[6..7]=angle i16 LE deg.
        if (!frame.extd && frame.data_length_code >= 8) {
            uint32_t id = frame.identifier;
            if (id >= (uint32_t)LKTECH_ID_BASE && id < (uint32_t)(LKTECH_ID_BASE + 256)) {
                uint8_t motor_id = (uint8_t)(id - LKTECH_ID_BASE);
                for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
                    if (s_lktech_id[j] != motor_id) continue;
                    // We only care about replies that echo the control/state 2 command.
                    uint8_t cmd = frame.data[0];
                    if (cmd != LKTECH_CMD_MULTI_LOOP_CONTROL_2 &&
                        cmd != 0x9C /*READ_STATE_2*/) {
                        break;
                    }
                    portENTER_CRITICAL(&s_lktech_mux);
                    s_lktech_fb[j].temp_c    = (int8_t)frame.data[1];
                    s_lktech_fb[j].iq_100    = getInt16LE(frame.data + 2);
                    s_lktech_fb[j].speed_dps = getInt16LE(frame.data + 4);
                    s_lktech_fb[j].angle_deg = getInt16LE(frame.data + 6);
                    // Compute software-zeroed output angle from angle reply.
                    // The 0xA4 reply's angle is single-turn motor degrees and
                    // wraps every i16 range; it is only useful for relative
                    // motion feedback.  We expose the raw value; consumers
                    // should use the delta, not the absolute reading.
                    float motor_deg = (float)s_lktech_fb[j].angle_deg;
                    float out_deg = (motor_deg / s_lktech_gear[j]) * s_lktech_dir[j];
                    s_lktech_fb[j].output_deg = out_deg;
                    s_lktech_fb[j].fresh = true;
                    portEXIT_CRITICAL(&s_lktech_mux);
                    break;
                }
                continue;
            }

            // ── ZE300 replies: reply ID = device_id (no 0x100 tag) ─────────
            // C2 replies carry updated multi-turn position counts; A4 replies
            // carry realtime state.
            if (id == (uint32_t)ze300ReplyId(ZE300_ID_J4)) {
                uint8_t cmd = frame.data[0];
                if (cmd == ZE_CMD_ABSOLUTE_POSITION && frame.data_length_code >= 5) {
                    portENTER_CRITICAL(&s_ze300_mux);
                    s_ze300_fb.position_counts = getInt32LE(frame.data + 1);
                    s_ze300_fb.fresh = true;
                    portEXIT_CRITICAL(&s_ze300_mux);
                } else if (cmd == ZE_CMD_READ_REALTIME_STATE && frame.data_length_code >= 8) {
                    portENTER_CRITICAL(&s_ze300_mux);
                    s_ze300_fb.temp_c        = (int8_t)frame.data[1];
                    s_ze300_fb.iq_1000       = getInt16LE(frame.data + 2);
                    s_ze300_fb.speed_rpm_100 = getInt16LE(frame.data + 4);
                    s_ze300_fb.single_turn   = getUint16LE(frame.data + 6);
                    s_ze300_fb.fresh = true;
                    portEXIT_CRITICAL(&s_ze300_mux);
                }
                continue;
            }
        }
#endif  // ROBOT_SECONDARY
    }

    // ── Send one ODrive telemetry RTR request (round-robin) ─────────────────
    // Cycles through joints × telemetry commands.  At 200 Hz poll rate with
    // 3 joints × 4 commands = 12 slots, each reading updates at ~16 Hz.
    {
        uint8_t joint_idx = s_odrive_telem_slot / ODRIVE_TELEM_CMD_COUNT;
        uint8_t cmd_idx   = s_odrive_telem_slot % ODRIVE_TELEM_CMD_COUNT;

        twai_message_t rtr = {};
        rtr.extd       = 0;
        rtr.rtr        = 1;
        rtr.identifier = odriveCOBID(s_node[joint_idx],
                                     s_odrive_telem_cmds[cmd_idx]);
        rtr.data_length_code = 8;
        twaiSend(rtr);

        s_odrive_telem_slot =
            (s_odrive_telem_slot + 1) % (ODRIVE_NUM_JOINTS * ODRIVE_TELEM_CMD_COUNT);
    }

#ifdef ROBOT_SECONDARY
    // ── Traction ODrive telemetry (same round-robin pattern, separate slot) ──
    {
        uint8_t motor_idx = s_traction_telem_slot / ODRIVE_TELEM_CMD_COUNT;
        uint8_t cmd_idx   = s_traction_telem_slot % ODRIVE_TELEM_CMD_COUNT;

        twai_message_t rtr = {};
        rtr.extd       = 0;
        rtr.rtr        = 1;
        rtr.identifier = odriveCOBID(s_traction_node[motor_idx],
                                     s_odrive_telem_cmds[cmd_idx]);
        rtr.data_length_code = 8;
        twaiSend(rtr);

        s_traction_telem_slot =
            (s_traction_telem_slot + 1) % (TRACTION_ODRIVE_NUM * ODRIVE_TELEM_CMD_COUNT);
    }

    // ── ZE300 realtime-state poll (~5 Hz) ────────────────────────────────────
    {
        uint32_t now = millis();
        if (now - s_ze300_last_telem_req_ms >= ZE300_TELEM_INTERVAL_MS) {
            s_ze300_last_telem_req_ms = now;
            ze300SendReadRealtimeState(ZE300_ID_J4);
        }
    }
#endif
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

bool CANInterface::getOdriveStatus(uint8_t joint_idx, OdriveStatusPayload& out) {
    if (joint_idx >= ODRIVE_NUM_JOINTS) return false;

    portENTER_CRITICAL(&s_odrive_mux);
    bool have = s_odrive_fb[joint_idx].fresh;
    if (have) {
        out.joint_idx       = joint_idx;
        out.pos_turns_100   = static_cast<int16_t>(s_odrive_fb[joint_idx].pos_turns   * 100.0f);
        out.vel_turns_s_100 = static_cast<int16_t>(s_odrive_fb[joint_idx].vel_turns_s * 100.0f);
        out.iq_measured_100 = static_cast<int16_t>(s_odrive_fb[joint_idx].iq_measured_a * 100.0f);
        out.bus_voltage_10  = static_cast<int16_t>(s_odrive_fb[joint_idx].bus_voltage_v * 10.0f);
        out.bus_current_100 = static_cast<int16_t>(s_odrive_fb[joint_idx].bus_current_a * 100.0f);
        s_odrive_fb[joint_idx].fresh = false;
    }
    portEXIT_CRITICAL(&s_odrive_mux);
    return have;
}

bool CANInterface::getTractionStatus(uint8_t motor_idx, OdriveStatusPayload& out) {
#ifdef ROBOT_SECONDARY
    if (motor_idx >= TRACTION_ODRIVE_NUM) return false;

    portENTER_CRITICAL(&s_traction_mux);
    bool have = s_traction_fb[motor_idx].fresh;
    if (have) {
        out.joint_idx       = motor_idx;   // 0=left, 1=right
        out.pos_turns_100   = static_cast<int16_t>(s_traction_fb[motor_idx].pos_turns   * 100.0f);
        out.vel_turns_s_100 = static_cast<int16_t>(s_traction_fb[motor_idx].vel_turns_s * 100.0f);
        out.iq_measured_100 = static_cast<int16_t>(s_traction_fb[motor_idx].iq_measured_a * 100.0f);
        out.bus_voltage_10  = static_cast<int16_t>(s_traction_fb[motor_idx].bus_voltage_v * 10.0f);
        out.bus_current_100 = static_cast<int16_t>(s_traction_fb[motor_idx].bus_current_a * 100.0f);
        s_traction_fb[motor_idx].fresh = false;
    }
    portEXIT_CRITICAL(&s_traction_mux);
    return have;
#else
    (void)motor_idx; (void)out;
    return false;
#endif
}

bool CANInterface::getOdriveError(uint8_t node_idx, OdriveErrorPayload& out) {
#if ODRIVE_ENABLE_ERROR_POLL
    if (node_idx >= ODRIVE_ERROR_SLOT_COUNT) return false;
    portENTER_CRITICAL(&s_odrive_err_mux);
    bool have = s_odrive_err[node_idx].fresh;
    if (have) {
        out.node_id     = s_odrive_err[node_idx].node_id;
        out.motor_error = s_odrive_err[node_idx].motor_error;
        s_odrive_err[node_idx].fresh = false;
    }
    portEXIT_CRITICAL(&s_odrive_err_mux);
    return have;
#else
    (void)node_idx; (void)out;
    return false;
#endif
}

uint8_t CANInterface::odriveNodeCount() {
#ifdef ROBOT_SECONDARY
    return ODRIVE_MAX_JOINTS + TRACTION_ODRIVE_NUM;
#else
    return ODRIVE_MAX_JOINTS;
#endif
}

bool CANInterface::getLktechStatus(uint8_t joint_idx, LktechStatusPayload& out) {
#ifdef ROBOT_SECONDARY
    if (joint_idx >= LKTECH_NUM_JOINTS) return false;

    portENTER_CRITICAL(&s_lktech_mux);
    bool have = s_lktech_fb[joint_idx].fresh;
    if (have) {
        out.joint_idx    = joint_idx;
        out.motor_id     = s_lktech_id[joint_idx];
        out.temp_c       = s_lktech_fb[joint_idx].temp_c;
        out.iq_100       = s_lktech_fb[joint_idx].iq_100;
        out.speed_dps    = s_lktech_fb[joint_idx].speed_dps;
        out.angle_deg    = s_lktech_fb[joint_idx].angle_deg;
        out.output_deg_10 = static_cast<int16_t>(
            s_lktech_fb[joint_idx].output_deg * 10.0f);
        s_lktech_fb[joint_idx].fresh = false;
    }
    portEXIT_CRITICAL(&s_lktech_mux);
    return have;
#else
    (void)joint_idx; (void)out;
    return false;
#endif
}

bool CANInterface::getZe300Status(Ze300StatusPayload& out) {
#ifdef ROBOT_SECONDARY
    portENTER_CRITICAL(&s_ze300_mux);
    bool have = s_ze300_fb.fresh;
    if (have) {
        out.device_id          = ZE300_ID_J4;
        out.temp_c             = s_ze300_fb.temp_c;
        out.iq_1000            = s_ze300_fb.iq_1000;
        out.speed_rpm_100      = s_ze300_fb.speed_rpm_100;
        out.single_turn_counts = (int16_t)s_ze300_fb.single_turn;
        out.position_counts    = s_ze300_fb.position_counts;
        // Software-zeroed output angle (degrees × 10).
        int32_t rel = s_ze300_fb.position_counts - s_ze300_zero_counts;
        float deg = ((float)rel * 360.0f) / (float)ZE300_COUNTS_PER_REV;
        deg *= ZE300_DIR_J4;
        out.output_deg_10 = static_cast<int16_t>(deg * 10.0f);
        s_ze300_fb.fresh = false;
    }
    portEXIT_CRITICAL(&s_ze300_mux);
    return have;
#else
    (void)out;
    return false;
#endif
}

void CANInterface::estopAllOdrives() {
    if (!s_ok) return;

#if ODRIVE_ESTOP_USE_NATIVE
    // Native ODrive E-stop (cmd 0x02): forces axis to IDLE immediately.
    // Recovery requires Clear_Errors + Set_Axis_State(CLOSED_LOOP).
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++)
        odriveSendEstop(s_node[j]);
  #ifdef ROBOT_SECONDARY
    for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++)
        odriveSendEstop(s_traction_node[m]);
  #endif
#else
    // Soft E-stop: command zero velocity / zero torque.
    // Axes stay in closed-loop; no recovery handshake needed.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++)
        odriveSendInputPos(s_node[j], s_odrive_zero[j]);  // hold at boot pose
  #ifdef ROBOT_SECONDARY
    for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++)
        odriveSendInputVel(s_traction_node[m], 0.0f);
  #endif
#endif
}

void CANInterface::clearEstopAllOdrives() {
    if (!s_ok) return;

#if ODRIVE_ESTOP_USE_NATIVE
    // Recover from native E-stop: clear errors, then re-enter closed-loop.
    // Arm ODrives (position control):
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        odriveSendClearErrors(s_node[j]);
    }
    delay(5);
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        odriveSendAxisState(s_node[j], 8 /*CLOSED_LOOP_CONTROL*/);
    }

  #ifdef ROBOT_SECONDARY
    // Traction ODrives (velocity control): full re-handshake.
    for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++) {
        tractionOdriveStartup(s_traction_node[m]);
    }
  #endif
#else
    // Soft E-stop: nothing to clear — axes never left closed-loop.
    (void)0;
#endif
}

bool CANInterface::isOk() {
    return s_ok;
}
