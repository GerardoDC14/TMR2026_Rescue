#include "Control.h"
#include "config.h"
#include "RC.h"
#include "Encoders.h"
#include "Locomotion.h"
#include "Sensors.h"
#include "CANInterface.h"
#include "Comms.h"
#include <Arduino.h>

// ─── Static members ───────────────────────────────────────────────────────────
RobotMode    Control::s_mode        = RobotMode::INIT;
ArmJoints    Control::s_arm_joints  = {};
uint8_t      Control::s_sensor_mask = 0;

// Default keybind table (matches original hardcoded behaviour)
KeybindTable Control::s_keybind = {{
    // mode0 (Ch5 low):  FLIPPER_ALL, TRACTION_FWD, NONE, TRACTION_TURN, NONE
    {ChannelFunction::FLIPPER_ALL, ChannelFunction::TRACTION_FWD, ChannelFunction::NONE,
     ChannelFunction::TRACTION_TURN, ChannelFunction::NONE},
    // mode1 (Ch5 mid):  same as mode0
    {ChannelFunction::FLIPPER_ALL, ChannelFunction::TRACTION_FWD, ChannelFunction::NONE,
     ChannelFunction::TRACTION_TURN, ChannelFunction::NONE},
    // mode2 (Ch5 high): ARM on all channels
    {ChannelFunction::ARM_FWD, ChannelFunction::ARM_FWD, ChannelFunction::ARM_FWD,
     ChannelFunction::ARM_FWD, ChannelFunction::NONE},
}, true};

// File-level mutex
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── Flipper PID (ROBOT_MAIN only) ──────────────────────────────────────────
#ifdef ROBOT_MAIN
float Control::s_pid_integral = 0.0f;
float Control::s_pid_prev_err = 0.0f;

float Control::flipperPID(float setpoint_deg, float measured_deg) {
    constexpr float dt = 1.0f / CONTROL_LOOP_HZ;
    float err         = setpoint_deg - measured_deg;
    s_pid_integral   += err * dt;
    if (s_pid_integral >  FLIPPER_PID_I_MAX) s_pid_integral =  FLIPPER_PID_I_MAX;
    if (s_pid_integral < -FLIPPER_PID_I_MAX) s_pid_integral = -FLIPPER_PID_I_MAX;
    float deriv     = (err - s_pid_prev_err) / dt;
    s_pid_prev_err  = err;
    float effort = FLIPPER_PID_KP * err
                 + FLIPPER_PID_KI * s_pid_integral
                 + FLIPPER_PID_KD * deriv;
    if (effort >  1.0f) effort =  1.0f;
    if (effort < -1.0f) effort = -1.0f;
    return effort;
}
#endif

// ─── begin() ─────────────────────────────────────────────────────────────────
void Control::begin() {
    Comms::onArmJoints(   [](const ArmJointsPayload& p) { Control::setArmJoints(p); });
    Comms::onSensorEnable([](uint8_t mask)              { Control::setSensorMask(mask); });
    Comms::onEstop(       [](bool active)               {
        if (active) Control::triggerEstop();
        else        Control::clearEstop();
    });
    Comms::onKeybind([](const KeybindPayload& p) { Control::setKeybind(p); });

    s_mode = RobotMode::STANDBY;
}

// ─── tick() — called at CONTROL_LOOP_HZ ─────────────────────────────────────
void Control::tick() {
    PPMFrame     ppm;
    EncoderState enc;
    bool         have_ppm = RC::getFrame(ppm);
    Encoders::getState(enc);

    // ── ESTOP: override everything ──────────────────────────────────────────
    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        Locomotion::neutralise();
        return;
    }

    // ── PPM link watchdog ───────────────────────────────────────────────────
    if (!RC::isConnected()) {
        Locomotion::neutralise();
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::STANDBY;
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    if (!have_ppm) return;

    // ── Decode Ch5 lever → mode index (0, 1, 2) ────────────────────────────
    int mode_idx = decodeModeIndex(ppm);

    // Determine the high-level RobotMode for telemetry/status reporting
    // by scanning what functions are bound in this mode row
    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    portEXIT_CRITICAL(&s_mux);

    bool has_arm = false;
    bool has_traction = false;
    bool has_flipper = false;
    bool has_estop = false;
    for (int c = 0; c < 5; ++c) {
        auto fn = kb.map[mode_idx][c];
        if (fn == ChannelFunction::ARM_FWD) has_arm = true;
        if (fn == ChannelFunction::TRACTION_FWD || fn == ChannelFunction::TRACTION_TURN) has_traction = true;
        if (fn >= ChannelFunction::FLIPPER_ALL && fn <= ChannelFunction::FLIPPER_RR) has_flipper = true;
        if (fn == ChannelFunction::ESTOP) has_estop = true;
    }

    // Virtual ESTOP from keybind: neutralise but don't lock into ESTOP state
    // (the real ESTOP from MSG_ESTOP is handled at the top of tick())
    if (has_estop) {
        Locomotion::neutralise();
        portENTER_CRITICAL(&s_mux);
        if (s_mode != RobotMode::ESTOP)
            s_mode = RobotMode::STANDBY;  // report as STANDBY, not hard ESTOP
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    RobotMode new_mode;
    if (has_arm && !has_traction && !has_flipper) {
        new_mode = RobotMode::ARM;
    } else if (has_flipper && !has_traction) {
        new_mode = RobotMode::FLIPPER;
    } else {
        new_mode = RobotMode::NORMAL;
    }

    portENTER_CRITICAL(&s_mux);
    if (s_mode != RobotMode::ESTOP)
        s_mode = new_mode;
    current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        Locomotion::neutralise();
        return;
    }

    // ── Apply the keybind row ───────────────────────────────────────────────
    applyKeybindRow(mode_idx, ppm, enc);
}

// ─── Apply keybind row ──────────────────────────────────────────────────────
// Maps each channel to its bound function and actuates accordingly.
void Control::applyKeybindRow(int mode_idx, const PPMFrame& ppm,
                              const EncoderState& enc) {
    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    portEXIT_CRITICAL(&s_mux);

    // Channel slot → raw PPM channel index mapping: slot 0=Ch1, 1=Ch2, 2=Ch3, 3=Ch4, 4=Ch6
    static const int slot_to_ppm[5] = {0, 1, 2, 3, 5};

    constexpr float kDeadband = 0.05f;

    // Accumulate traction and flipper commands from all channels
    float forward = 0.0f;
    float turn = 0.0f;
    float flipper_all = 0.0f;
    float flipper_fl = 0.0f, flipper_fr = 0.0f, flipper_rl = 0.0f, flipper_rr = 0.0f;
    bool has_traction = false;
    bool has_flipper = false;
    bool has_arm = false;

    for (int c = 0; c < 5; ++c) {
        ChannelFunction fn = kb.map[mode_idx][c];
        if (fn == ChannelFunction::NONE) continue;

        float val = ppmNormalise(ppm.ch[slot_to_ppm[c]]);
        if (fabsf(val) < kDeadband) val = 0.0f;

        switch (fn) {
            case ChannelFunction::TRACTION_FWD:
                forward = val;
                has_traction = true;
                break;
            case ChannelFunction::TRACTION_TURN:
                turn = val;
                has_traction = true;
                break;
            case ChannelFunction::FLIPPER_ALL:
                flipper_all = val;
                has_flipper = true;
                break;
            case ChannelFunction::FLIPPER_FL:
                flipper_fl = val;
                has_flipper = true;
                break;
            case ChannelFunction::FLIPPER_FR:
                flipper_fr = val;
                has_flipper = true;
                break;
            case ChannelFunction::FLIPPER_RL:
                flipper_rl = val;
                has_flipper = true;
                break;
            case ChannelFunction::FLIPPER_RR:
                flipper_rr = val;
                has_flipper = true;
                break;
            case ChannelFunction::ARM_FWD:
                has_arm = true;
                break;
            default:
                break;
        }
    }

    // ── Apply traction ──────────────────────────────────────────────────────
    if (has_traction) {
        Locomotion::setDriveCommand(forward, turn);
    } else if (!has_arm) {
        // No traction bound and not in arm mode — stop tracks
        Locomotion::setDriveCommand(0.0f, 0.0f);
    }

    // ── Apply flippers ──────────────────────────────────────────────────────
    if (has_flipper) {
#ifdef ROBOT_MAIN
        // Jaguar: single joined flipper — use FLIPPER_ALL value (or first individual)
        float target_norm = flipper_all;
        if (target_norm == 0.0f) target_norm = flipper_fl;  // fallback to individual
        float target_deg = target_norm * FLIPPER_ANGLE_MAX;
        Locomotion::setFlipperEffort(flipperPID(target_deg, enc.flipper_angle_deg));
#elif defined(ROBOT_SECONDARY)
        // Dicerox: 4 independent flippers
        if (flipper_all != 0.0f) {
            // FLIPPER_ALL: all four get the same value
            Locomotion::setFlipperTargets(flipper_all, flipper_all, flipper_all, flipper_all);
        } else {
            Locomotion::setFlipperTargets(flipper_fl, flipper_fr, flipper_rl, flipper_rr);
        }
#endif
    }

    // ── Apply arm mode ──────────────────────────────────────────────────────
    if (has_arm) {
        // Halt tracks
        Locomotion::setDriveCommand(0.0f, 0.0f);

        // Forward all PPM to mini PC for IK
        TelemetryPayload telem;
        telem.mode  = static_cast<uint8_t>(RobotMode::ARM);
        telem.flags = 0;
        for (int i = 0; i < PPM_CHANNELS; i++) telem.ppm[i] = ppm.ch[i];
        Comms::sendTelemetry(telem);

#ifdef ROBOT_SECONDARY
        // Relay joint angles from mini PC to arm over CAN
        portENTER_CRITICAL(&s_mux);
        ArmJoints joints = s_arm_joints;
        portEXIT_CRITICAL(&s_mux);
        if (joints.valid)
            CANInterface::sendArmJoints(joints.angle_deg);
#endif
    }
}

// ─── Callbacks ────────────────────────────────────────────────────────────────
void Control::triggerEstop() {
    portENTER_CRITICAL(&s_mux);
    s_mode = RobotMode::ESTOP;
    portEXIT_CRITICAL(&s_mux);
    Locomotion::neutralise();
}

void Control::clearEstop() {
    portENTER_CRITICAL(&s_mux);
    if (s_mode == RobotMode::ESTOP) s_mode = RobotMode::STANDBY;
    portEXIT_CRITICAL(&s_mux);
}

void Control::setArmJoints(const ArmJointsPayload& payload) {
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 6; i++)
        s_arm_joints.angle_deg[i] = payload.joint[i] * 0.01f;
    s_arm_joints.valid = true;
    portEXIT_CRITICAL(&s_mux);
}

void Control::setSensorMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_sensor_mask = mask;
    portEXIT_CRITICAL(&s_mux);
    Sensors::setEnabledMask(mask);
}

void Control::setKeybind(const KeybindPayload& payload) {
    portENTER_CRITICAL(&s_mux);
    for (int m = 0; m < 3; ++m)
        for (int c = 0; c < 5; ++c)
            s_keybind.map[m][c] = static_cast<ChannelFunction>(payload.map[m][c]);
    s_keybind.valid = true;
    portEXIT_CRITICAL(&s_mux);
}

// ─── Accessors ────────────────────────────────────────────────────────────────
RobotMode Control::getMode() {
    portENTER_CRITICAL(&s_mux);
    RobotMode m = s_mode;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Control::getSystemStatus(SystemStatus& out) {
    portENTER_CRITICAL(&s_mux);
    out.mode         = s_mode;
    out.sensor_mask  = s_sensor_mask;
    out.estop        = (s_mode == RobotMode::ESTOP);
    portEXIT_CRITICAL(&s_mux);
    out.ppm_connected   = RC::isConnected();
    out.minipc_connected = Comms::isConnected();
    out.can_ok           = CANInterface::isOk();
    out.uptime_ms        = millis();
}

// ─── Helper ───────────────────────────────────────────────────────────────────
// Decode Ch5 3-position lever into mode index 0, 1, or 2
int Control::decodeModeIndex(const PPMFrame& ppm) {
    constexpr uint16_t kLow  = PPM_MIN_US + (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1250
    constexpr uint16_t kHigh = PPM_MAX_US - (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1750
    uint16_t ch5 = ppm.ch[PPM_CH_MODE - 1];
    if (ch5 < kLow)  return 0;   // low position
    if (ch5 > kHigh) return 2;   // high position
    return 1;                     // mid position
}
