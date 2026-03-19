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

// File-level mutex: keeps FreeRTOS types out of the class header
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── begin() — called once from setup() ──────────────────────────────────────
void Control::begin() {
    // Wire up callbacks from Comms to Control setters
    Comms::onArmJoints(   [](const ArmJointsPayload& p) { Control::setArmJoints(p); });
    Comms::onSensorEnable([](uint8_t mask)              { Control::setSensorMask(mask); });
    Comms::onEstop(       [](bool active)               {
        if (active) Control::triggerEstop();
        else        Control::clearEstop();
    });

    s_mode = RobotMode::STANDBY;
    // Note: setArmJoints callback is wired for both robots.
    // On ROBOT_MAIN the received joints are unused (arm has its own ESP32).
    // On ROBOT_SECONDARY they are forwarded via CAN in updateArmMode().
}

// ─── tick() — called from control task at CONTROL_LOOP_HZ ────────────────────
void Control::tick() {
    PPMFrame     ppm;
    EncoderState enc;
    bool         have_ppm = RC::getFrame(ppm);
    Encoders::getState(enc);

    // ── ESTOP: override everything ────────────────────────────────────────────
    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        Locomotion::neutralise();
        return;
    }

    // ── PPM link watchdog: go to STANDBY if signal lost ──────────────────────
    if (!RC::isConnected()) {
        Locomotion::neutralise();
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::STANDBY;
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    // ── Mode transition from Ch5 (3-position lever) ──────────────────────────
    if (have_ppm) {
        RobotMode decoded = decodeModeFromCh5(ppm);
        portENTER_CRITICAL(&s_mux);
        // ESTOP can only be cleared by mini PC; all other transitions follow Ch5
        if (s_mode != RobotMode::ESTOP) {
            s_mode = decoded;
        }
        current = s_mode;
        portEXIT_CRITICAL(&s_mux);
    }

    // ── Per-mode update ───────────────────────────────────────────────────────
    if (have_ppm) {
        switch (current) {
            case RobotMode::NORMAL:  updateNormalMode(ppm, enc);  break;
            case RobotMode::FLIPPER: updateFlipperMode(ppm, enc); break;
            case RobotMode::ARM:     updateArmMode(ppm);          break;
            default: break;
        }
    }

    // ── Periodic telemetry to mini PC ─────────────────────────────────────────
    // (Triggered by the comms task reading the shared state; nothing to do here.)

    // ── Sensor data forwarding (when enabled) ────────────────────────────────
    // Sent from the sensor task after each successful reading cycle.
}

// ─── NORMAL mode ─────────────────────────────────────────────────────────────
// Ch2/Ch4 → tank-drive tracks.
// ROBOT_MAIN also drives the single forward flipper from Ch1.
// ROBOT_SECONDARY leaves flippers at neutral (controlled only in FLIPPER mode).
void Control::updateNormalMode(const PPMFrame& ppm, const EncoderState& /*enc*/) {
    float forward = ppmNormalise(ppm.ch[PPM_CH_FORWARD - 1]);
    float turn    = ppmNormalise(ppm.ch[PPM_CH_TURN    - 1]);

    constexpr float kDeadband = 0.05f;
    if (fabsf(forward) < kDeadband) forward = 0.0f;
    if (fabsf(turn)    < kDeadband) turn    = 0.0f;

    Locomotion::setDriveCommand(forward, turn);

#ifdef ROBOT_MAIN
    float flipper_norm  = ppmNormalise(ppm.ch[PPM_CH_FLIPPER - 1]);
    float flipper_angle = flipper_norm * FLIPPER_ANGLE_MAX;
    Locomotion::setFlipperTarget(flipper_angle);
#elif defined(ROBOT_SECONDARY)
    // No flipper in normal mode on ROBOT_SECONDARY
#endif
}

// ─── FLIPPER mode ────────────────────────────────────────────────────────────
// ROBOT_MAIN: identical to NORMAL — tracks on Ch2/Ch4, single flipper on Ch1.
//   (Both modes behave the same because the joined front flippers are always
//    part of normal driving on ROBOT_MAIN.)
// ROBOT_SECONDARY: Ch1-4 drive each of the four flippers independently;
//   tracks are stopped.
void Control::updateFlipperMode(const PPMFrame& ppm, const EncoderState& enc) {
#ifdef ROBOT_MAIN
    // Same behaviour as NORMAL mode on ROBOT_MAIN
    updateNormalMode(ppm, enc);

#elif defined(ROBOT_SECONDARY)
    // Tracks stopped while operating flippers
    Locomotion::setDriveCommand(0.0f, 0.0f);

    // Ch1-4 map individually to FL, FR, RL, RR flippers
    // TODO: confirm which physical flipper each channel maps to.
    float fl = ppmNormalise(ppm.ch[0]);   // Ch1
    float fr = ppmNormalise(ppm.ch[1]);   // Ch2
    float rl = ppmNormalise(ppm.ch[2]);   // Ch3
    float rr = ppmNormalise(ppm.ch[3]);   // Ch4
    Locomotion::setFlipperTargets(fl, fr, rl, rr);
#endif
}

// ─── ARM mode ────────────────────────────────────────────────────────────────
// All channels forwarded to mini PC for IK on both robots.
// ROBOT_MAIN:      arm has its own ESP32; this ESP32 just sends PPM and waits.
// ROBOT_SECONDARY: this ESP32 also receives the resulting joint angles from the
//                  mini PC and forwards them via CAN to the 6DOF arm.
void Control::updateArmMode(const PPMFrame& ppm) {
    // Halt tracks while the arm is active (both robots)
    Locomotion::setDriveCommand(0.0f, 0.0f);

    // Send all raw PPM channels to the mini PC so it can run IK.
    // The comms task populates encoder/speed fields from shared state.
    TelemetryPayload telem;
    telem.mode  = static_cast<uint8_t>(RobotMode::ARM);
    telem.flags = 0;
    for (int i = 0; i < PPM_CHANNELS; i++) telem.ppm[i] = ppm.ch[i];
    Comms::sendTelemetry(telem);

#ifdef ROBOT_SECONDARY
    // Relay the joint angles received from the mini PC to the arm over CAN
    portENTER_CRITICAL(&s_mux);
    ArmJoints joints = s_arm_joints;
    portEXIT_CRITICAL(&s_mux);

    if (joints.valid) {
        CANInterface::sendArmJoints(joints.angle_deg);
    }
    // ROBOT_MAIN does NOT relay joints — the arm's dedicated ESP32 handles that
    // communication independently with the mini PC.
#endif
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
    for (int i = 0; i < 6; i++) {
        s_arm_joints.angle_deg[i] = payload.joint[i] * 0.01f;   // ×100 → degrees
    }
    s_arm_joints.valid = true;
    portEXIT_CRITICAL(&s_mux);
}

void Control::setSensorMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_sensor_mask = mask;
    portEXIT_CRITICAL(&s_mux);
    Sensors::setEnabledMask(mask);
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
// Ch5 is a 3-position lever:
//   ~1000 µs  →  FLIPPER
//   ~1500 µs  →  NORMAL
//   ~2000 µs  →  ARM
RobotMode Control::decodeModeFromCh5(const PPMFrame& ppm) {
    constexpr uint16_t kLow  = PPM_MIN_US + (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1250 µs
    constexpr uint16_t kHigh = PPM_MAX_US - (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1750 µs
    uint16_t ch5 = ppm.ch[PPM_CH_MODE - 1];
    if (ch5 < kLow)  return RobotMode::FLIPPER;
    if (ch5 > kHigh) return RobotMode::ARM;
    return RobotMode::NORMAL;
}
