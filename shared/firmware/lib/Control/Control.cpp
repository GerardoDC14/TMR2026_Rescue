#include "Control.h"
#include "config.h"
#include "RC.h"
#include "Encoders.h"
#include "Locomotion.h"
#include "Sensors.h"
#include "CANInterface.h"
#include "Comms.h"
#include "Debug.h"
#include <Arduino.h>
#include <cmath>

// ─── Static members ───────────────────────────────────────────────────────────
RobotMode    Control::s_mode        = RobotMode::INIT;
ArmJoints    Control::s_arm_joints  = {};
uint8_t      Control::s_sensor_mask = 0;

#ifdef ROBOT_MAIN
  #ifdef ENABLE_TRACTION_PID
PID   Control::s_pid_traction_left;
PID   Control::s_pid_traction_right;
  #endif
  #ifdef ENABLE_FLIPPER_PID
PID   Control::s_pid_flipper;
float Control::s_flipper_target_angle = 0.0f;
  #endif
#endif
#ifdef ROBOT_SECONDARY
PID   Control::s_pid_flipper[4];
#endif

// Default keybind table (matches original hardcoded behaviour)
// NOTE: Ch6 (slot 4) is NOT bound here — it's monitored as a direct ESTOP trigger in tick()
KeybindTable Control::s_keybind = {{
    // mode0 (Ch5 low):  FLIPPER_ALL, TRACTION_FWD, NONE, TRACTION_TURN, NONE
    {ChannelFunction::FLIPPER_ALL, ChannelFunction::TRACTION_FWD, ChannelFunction::NONE,
     ChannelFunction::TRACTION_TURN, ChannelFunction::NONE},
    // mode1 (Ch5 mid):  same as mode0
    {ChannelFunction::FLIPPER_ALL, ChannelFunction::TRACTION_FWD, ChannelFunction::NONE,
     ChannelFunction::TRACTION_TURN, ChannelFunction::NONE},
    // mode2 (Ch5 high): per-axis arm control
    {ChannelFunction::ARM_Y, ChannelFunction::ARM_X, ChannelFunction::ARM_Z,
     ChannelFunction::ARM_YAW, ChannelFunction::NONE},
}};

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── begin() ─────────────────────────────────────────────────────────────────
void Control::begin() {
    Comms::onArmJoints(   [](const ArmJointsPayload& p) { Control::setArmJoints(p); });
    Comms::onSensorEnable([](uint8_t mask)              { Control::setSensorMask(mask); });
    Comms::onEstop(       [](bool active) {
        if (active) Control::triggerEstop();
        else        Control::clearEstop();
    });
    Comms::onKeybind([](const KeybindPayload& p) { Control::setKeybind(p); });
    Comms::onPpmCalib([](const PpmCalibPayload& p) { RC::setCalib(p); });

    // ── Configure PID controllers (only when enabled) ─────────────────────
#ifdef ROBOT_MAIN
  #ifdef ENABLE_TRACTION_PID
    s_pid_traction_left.configure(
        TRACTION_VEL_PID_LEFT_KP,  TRACTION_VEL_PID_LEFT_KI,  TRACTION_VEL_PID_LEFT_KD,
        TRACTION_VEL_PID_LEFT_I_MAX,
        -TRACTION_MAX_NORM, TRACTION_MAX_NORM,
        TRACTION_VEL_PID_LEFT_D_ALPHA);

    s_pid_traction_right.configure(
        TRACTION_VEL_PID_RIGHT_KP, TRACTION_VEL_PID_RIGHT_KI, TRACTION_VEL_PID_RIGHT_KD,
        TRACTION_VEL_PID_RIGHT_I_MAX,
        -TRACTION_MAX_NORM, TRACTION_MAX_NORM,
        TRACTION_VEL_PID_RIGHT_D_ALPHA);
  #endif
  #ifdef ENABLE_FLIPPER_PID
    s_pid_flipper.configure(
        FLIPPER_PID_KP, FLIPPER_PID_KI, FLIPPER_PID_KD,
        FLIPPER_PID_I_MAX,
        -1.0f, 1.0f,
        FLIPPER_PID_D_ALPHA);
  #endif
#endif
#ifdef ROBOT_SECONDARY
    for (int i = 0; i < 4; i++) {
        s_pid_flipper[i].configure(
            FLIPPER_PID_KP, FLIPPER_PID_KI, FLIPPER_PID_KD,
            FLIPPER_PID_I_MAX, -1.0f, 1.0f, FLIPPER_PID_D_ALPHA);
    }
#endif

    s_mode = RobotMode::STANDBY;
}

// ─── tick() — called at CONTROL_LOOP_HZ ─────────────────────────────────────
void Control::tick() {
    PPMFrame     ppm;
    EncoderState enc;
    bool         have_ppm = RC::getFrame(ppm);
    Encoders::getState(enc);

    // ── Ch6 ESTOP: highest priority ─────────────────────────────────────────
    if (have_ppm) {
        if (ppm.ch[5] > 1800) {
            triggerEstop();
        } else {
            portENTER_CRITICAL(&s_mux);
            if (s_mode == RobotMode::ESTOP)
                s_mode = RobotMode::STANDBY;
            portEXIT_CRITICAL(&s_mux);
        }
    }

    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        DBG_EVERY_MS(Dbg::MODE, 500, "ESTOP active\n");
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
    /*
    DBG_EVERY_MS(Dbg::PPM, 200, "PPM: %u %u %u %u %u %u\n",
                 ppm.ch[0], ppm.ch[1], ppm.ch[2],
                 ppm.ch[3], ppm.ch[4], ppm.ch[5]);
    */

    // ── Decode Ch5 lever → mode index (0, 1, 2) ────────────────────────────
    int mode_idx = decodeModeIndex(ppm);

    // Determine the high-level RobotMode from what's bound in this row
    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    portEXIT_CRITICAL(&s_mux);

    bool has_arm = false, has_traction = false, has_flipper = false;
    for (int c = 0; c < 5; ++c) {
        auto fn = kb.map[mode_idx][c];
        if (isArmFunction(fn)) has_arm = true;
        if (fn == ChannelFunction::TRACTION_FWD || fn == ChannelFunction::TRACTION_TURN)
            has_traction = true;
        if (fn >= ChannelFunction::FLIPPER_ALL && fn <= ChannelFunction::FLIPPER_RR)
            has_flipper = true;
    }

    RobotMode new_mode;
    if (has_arm && !has_traction && !has_flipper)
        new_mode = RobotMode::ARM;
    else if (has_flipper && !has_traction)
        new_mode = RobotMode::FLIPPER;
    else
        new_mode = RobotMode::NORMAL;

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
void Control::applyKeybindRow(int mode_idx, const PPMFrame& ppm,
                              const EncoderState& enc) {
    constexpr float dt = 1.0f / CONTROL_LOOP_HZ;

    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    portEXIT_CRITICAL(&s_mux);

    // Channel slot → raw PPM channel index: slot 0=Ch1, 1=Ch2, 2=Ch3, 3=Ch4, 4=Ch6
    static const int slot_to_ppm[5] = {0, 1, 2, 3, 5};
    constexpr float kDeadband = 0.05f;

    // Accumulate commands from all bound channels
    float forward = 0.0f, turn = 0.0f;
    float flipper_all = 0.0f;
    float flipper_fl = 0.0f, flipper_fr = 0.0f, flipper_rl = 0.0f, flipper_rr = 0.0f;
    bool has_traction = false, has_flipper = false, has_arm = false;

    for (int c = 0; c < 5; ++c) {
        ChannelFunction fn = kb.map[mode_idx][c];
        if (fn == ChannelFunction::NONE) continue;

        int ppm_idx = slot_to_ppm[c];
        uint16_t raw_us = ppm.ch[ppm_idx];
        float val = RC::normalise(static_cast<uint8_t>(ppm_idx), raw_us);
        if (fabsf(val) < kDeadband) val = 0.0f;

        switch (fn) {
            case ChannelFunction::TRACTION_FWD:  forward = val; has_traction = true; break;
            case ChannelFunction::TRACTION_TURN: turn = val;    has_traction = true; break;
            case ChannelFunction::FLIPPER_ALL:   flipper_all = val; has_flipper = true; break;
            case ChannelFunction::FLIPPER_FL:    flipper_fl = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_FR:    flipper_fr = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_RL:    flipper_rl = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_RR:    flipper_rr = val;  has_flipper = true; break;
            case ChannelFunction::ARM_FWD:
            case ChannelFunction::ARM_X:
            case ChannelFunction::ARM_Y:
            case ChannelFunction::ARM_Z:
            case ChannelFunction::ARM_PITCH:
            case ChannelFunction::ARM_YAW:       has_arm = true; break;
            default: break;
        }
    }

    // ── Traction ────────────────────────────────────────────────────────────
    if (has_traction) {
#ifdef ROBOT_MAIN
  #ifdef ENABLE_TRACTION_PID
        // Closed-loop: each channel drives one motor's RPM setpoint independently.
        float left_rpm  = forward * TRACTION_MAX_RPM;
        float right_rpm = turn    * TRACTION_MAX_RPM;

        float left_eff  = s_pid_traction_left.update(left_rpm,  enc.speed_left_rpm,  dt);
        float right_eff = s_pid_traction_right.update(right_rpm, enc.speed_right_rpm, dt);

        Locomotion::setTrackSpeeds(left_eff, right_eff);

        DBG_EVERY_MS(Dbg::PID_TRACK, 200,
            "TRACK L(tgt=%.0f cur=%.0f eff=%.3f) R(tgt=%.0f cur=%.0f eff=%.3f)\n",
            left_rpm, enc.speed_left_rpm, left_eff,
            right_rpm, enc.speed_right_rpm, right_eff);
  #else
        // Open-loop: stick → normalised effort directly (clamped by TRACTION_MAX_NORM in Locomotion)
        Locomotion::setTrackSpeeds(forward, turn);

        DBG_EVERY_MS(Dbg::TRACTION, 200,
            "PPM: %u %u %u %u %u %u | fwd=%.3f turn=%.3f\n",
            ppm.ch[0], ppm.ch[1], ppm.ch[2], ppm.ch[3], ppm.ch[4], ppm.ch[5],
            forward, turn);
  #endif
#else
        Locomotion::setDriveCommand(forward, turn);
#endif
    } else if (!has_arm) {
        // No traction bound and not in arm mode — stop tracks
#if defined(ROBOT_MAIN) && defined(ENABLE_TRACTION_PID)
        s_pid_traction_left.reset();
        s_pid_traction_right.reset();
#endif
        Locomotion::setTrackSpeeds(0.0f, 0.0f);
    }

    // ── Flippers ─────────────────────────────────────────────────────────────
    if (has_flipper) {
#ifdef ROBOT_MAIN
  #ifdef ENABLE_FLIPPER_PID
        // Closed-loop: stick = velocity command, integrated to target angle
        float cmd = flipper_all;
        if (cmd == 0.0f) cmd = flipper_fl;

        s_flipper_target_angle += cmd * FLIPPER_RATE_DPS * dt;
        // Wrap to [0, 360)
        s_flipper_target_angle = fmodf(s_flipper_target_angle, 360.0f);
        if (s_flipper_target_angle < 0.0f) s_flipper_target_angle += 360.0f;

        float effort = s_pid_flipper.updateAngular(
            s_flipper_target_angle, enc.flipper_angle_deg, dt);
        Locomotion::setFlipperEffort(effort);

        DBG_EVERY_MS(Dbg::PID_FLIP, 200,
            "FLIP: cur=%.1f tgt=%.1f cmd=%.2f eff=%.3f\n",
            enc.flipper_angle_deg, s_flipper_target_angle, cmd, effort);
  #else
        // Open-loop: stick → flipper effort directly
        float cmd = flipper_all;
        if (cmd == 0.0f) cmd = flipper_fl;
        Locomotion::setFlipperEffort(cmd);

        DBG_EVERY_MS(Dbg::FLIPPER, 200,
            "FLIP OL  cmd=%.3f\n", cmd);
  #endif
#elif defined(ROBOT_SECONDARY)
        // 4 independent flippers: stick → target angle, per-flipper PID
        auto normToAngle = [](float n) -> float {
            float deg = fmodf((n + 1.0f) * 180.0f, 360.0f);
            if (deg < 0.0f) deg += 360.0f;
            return deg;
        };

        float norms[4];
        if (flipper_all != 0.0f) {
            norms[0] = norms[1] = norms[2] = norms[3] = flipper_all;
        } else {
            norms[0] = flipper_fl;  norms[1] = flipper_fr;
            norms[2] = flipper_rl;  norms[3] = flipper_rr;
        }
        float measured[4] = {
            enc.flipper_angle_fl_deg, enc.flipper_angle_fr_deg,
            enc.flipper_angle_rl_deg, enc.flipper_angle_rr_deg
        };
        float efforts[4];
        for (int i = 0; i < 4; i++) {
            efforts[i] = s_pid_flipper[i].updateAngular(
                normToAngle(norms[i]), measured[i], dt);
        }
        Locomotion::setFlipperTargets(efforts[0], efforts[1], efforts[2], efforts[3]);
#endif
    }

    // ── Arm mode ────────────────────────────────────────────────────────────
    if (has_arm) {
#if defined(ROBOT_MAIN) && defined(ENABLE_TRACTION_PID)
        s_pid_traction_left.reset();
        s_pid_traction_right.reset();
#endif
        Locomotion::setTrackSpeeds(0.0f, 0.0f);

#ifdef ROBOT_SECONDARY
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
#ifdef ROBOT_MAIN
  #ifdef ENABLE_TRACTION_PID
    s_pid_traction_left.reset();
    s_pid_traction_right.reset();
  #endif
  #ifdef ENABLE_FLIPPER_PID
    s_pid_flipper.reset();
  #endif
#endif
#ifdef ROBOT_SECONDARY
    for (int i = 0; i < 4; i++) s_pid_flipper[i].reset();
#endif
    portEXIT_CRITICAL(&s_mux);
    Locomotion::neutralise();
#ifdef ENABLE_COMMS
    CANInterface::estopAllOdrives();
#endif
}

void Control::clearEstop() {
    portENTER_CRITICAL(&s_mux);
    bool was_estop = (s_mode == RobotMode::ESTOP);
    if (was_estop) s_mode = RobotMode::STANDBY;
    portEXIT_CRITICAL(&s_mux);
#ifdef ENABLE_COMMS
    if (was_estop) CANInterface::clearEstopAllOdrives();
#endif
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
    out.ppm_connected    = RC::isConnected();
    out.minipc_connected = Comms::isConnected();
    out.can_ok           = CANInterface::isOk();
    out.uptime_ms        = millis();
}

// ─── Helper ───────────────────────────────────────────────────────────────────
int Control::decodeModeIndex(const PPMFrame& ppm) {
    constexpr uint16_t kLow  = PPM_MIN_US + (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1250
    constexpr uint16_t kHigh = PPM_MAX_US - (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1750
    uint16_t ch5 = ppm.ch[PPM_CH_MODE - 1];
    if (ch5 < kLow)  return 0;
    if (ch5 > kHigh) return 2;
    return 1;
}
