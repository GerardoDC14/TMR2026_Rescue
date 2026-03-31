#include "Locomotion.h"
#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <cmath>

float Locomotion::s_flipper_target_deg = 0.0f;

// ─── Helpers ──────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Normalised [-1, +1] → LEDC duty count
// Neutral (0.0) → MOTOR_NEUTRAL_US, full forward (1.0) → MOTOR_MAX_US
uint32_t Locomotion::normToDuty(float norm) {
    float us = MOTOR_NEUTRAL_US + norm * (MOTOR_MAX_US - MOTOR_NEUTRAL_US);
    us = clampf(us, MOTOR_MIN_US, MOTOR_MAX_US);
    // duty = (pulse_us / period_us) × (2^resolution)
    return static_cast<uint32_t>((us / PWM_PERIOD_US) * (1u << PWM_RESOLUTION));
}

// ─── Public API ──────────────────────────────────────────────────────────────

void Locomotion::begin() {
#ifdef ROBOT_MAIN
    // Configure LEDC channels for servo-style PWM (50 Hz, 14-bit)
    ledcSetup(LEDC_CH_LEFT,    PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_RIGHT,   PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_FLIPPER, PWM_FREQ_HZ, PWM_RESOLUTION);

    ledcAttachPin(PIN_MOTOR_LEFT,    LEDC_CH_LEFT);
    ledcAttachPin(PIN_MOTOR_RIGHT,   LEDC_CH_RIGHT);
    ledcAttachPin(PIN_MOTOR_FLIPPER, LEDC_CH_FLIPPER);
#elif defined(ROBOT_SECONDARY)
    // No local PWM: all actuators are CAN-controlled.
    // CANInterface is initialised separately in setup().
#endif
    neutralise();
}

void Locomotion::setDriveCommand(float forward, float turn) {
    // Standard tank-drive mixing
    float left  = forward + turn;
    float right = forward - turn;

    // Scale down if either channel saturates, preserving the turn ratio
    float mag = fmaxf(fabsf(left), fabsf(right));
    if (mag > 1.0f) {
        left  /= mag;
        right /= mag;
    }

    setTrackSpeeds(left, right);
}

void Locomotion::setTrackSpeeds(float left_norm, float right_norm) {
    left_norm  = clampf(left_norm,  -1.0f, 1.0f);
    right_norm = clampf(right_norm, -1.0f, 1.0f);
    applyTrackSpeeds(left_norm, right_norm);
}

void Locomotion::setFlipperTarget(float angle_deg) {
    s_flipper_target_deg = clampf(angle_deg, FLIPPER_ANGLE_MIN, FLIPPER_ANGLE_MAX);
    // Map angle to normalised PWM:
    // 0° → neutral pulse, FLIPPER_ANGLE_MAX → full forward, FLIPPER_ANGLE_MIN → full reverse
    float range = FLIPPER_ANGLE_MAX - FLIPPER_ANGLE_MIN;
    float mid   = (FLIPPER_ANGLE_MAX + FLIPPER_ANGLE_MIN) * 0.5f;
    float norm  = (s_flipper_target_deg - mid) / (range * 0.5f);
    applyFlipperPWM(clampf(norm, -1.0f, 1.0f));
}

void Locomotion::neutralise() {
#ifdef ROBOT_MAIN
    uint32_t neutral = normToDuty(0.0f);
    ledcWrite(LEDC_CH_LEFT,    neutral);
    ledcWrite(LEDC_CH_RIGHT,   neutral);
    ledcWrite(LEDC_CH_FLIPPER, neutral);
#elif defined(ROBOT_SECONDARY)
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    CANInterface::sendFlipperSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
#endif
}

// ─── Platform-specific output ────────────────────────────────────────────────

void Locomotion::applyTrackSpeeds(float left_norm, float right_norm) {
#ifdef ROBOT_MAIN
    // Servo-PWM motor drivers on ROBOT_MAIN
    ledcWrite(LEDC_CH_LEFT,  normToDuty(left_norm));
    ledcWrite(LEDC_CH_RIGHT, normToDuty(right_norm));
#elif defined(ROBOT_SECONDARY)
    // CAN motor controllers on ROBOT_SECONDARY
    CANInterface::sendTrackSpeeds(left_norm, right_norm);
#endif
}

void Locomotion::applyFlipperPWM(float norm) {
#ifdef ROBOT_MAIN
    // NOTE: the ROBOT_MAIN flipper driver may change from the current
    // "regular PWM + two direction pins" wiring to a servo-PWM signal
    // identical to the traction ESCs.  If that change is made, this function
    // body needs no changes (normToDuty already produces a servo-style pulse),
    // but the direction-pin GPIO setup and any H-bridge enable logic in begin()
    // should be removed and the flipper pin remapped in config.h.
    ledcWrite(LEDC_CH_FLIPPER, normToDuty(norm));
#endif
    // ROBOT_SECONDARY uses applyFlipperSpeeds() instead — not called here.
}

void Locomotion::applyFlipperSpeeds(float fl, float fr, float rl, float rr) {
#ifdef ROBOT_SECONDARY
    CANInterface::sendFlipperSpeeds(fl, fr, rl, rr);
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
#endif
}

void Locomotion::setFlipperEffort(float norm) {
    applyFlipperPWM(clampf(norm, -1.0f, 1.0f));
}

void Locomotion::setFlipperTargets(float fl, float fr, float rl, float rr) {
    // Clamp all four to [-1, 1] before forwarding to CAN
    fl = clampf(fl, -1.0f, 1.0f);
    fr = clampf(fr, -1.0f, 1.0f);
    rl = clampf(rl, -1.0f, 1.0f);
    rr = clampf(rr, -1.0f, 1.0f);
    applyFlipperSpeeds(fl, fr, rl, rr);
}
