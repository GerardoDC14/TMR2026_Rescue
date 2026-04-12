#include "Locomotion.h"
#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <cmath>

float Locomotion::s_track_left_norm     = 0.0f;
float Locomotion::s_track_right_norm    = 0.0f;
float Locomotion::s_flipper_effort_norm = 0.0f;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Write a servo-style pulse width using the native LEDC v3 API (pin-based).
// Converts microseconds → duty count for 50 Hz / 16-bit resolution.
// Matches the approach in the known-good bare-minimum test.
void Locomotion::writeMicroseconds(uint8_t pin, uint16_t us) {
    uint32_t duty = (static_cast<uint32_t>(us) * SERVO_LEDC_MAX_DUTY) / SERVO_LEDC_PERIOD_US;
    ledcWrite(pin, duty);
}

// ─── Initialisation ──────────────────────────────────────────────────────────

void Locomotion::begin() {
#ifdef ROBOT_MAIN
    // Traction: servo-style 50 Hz / 16-bit via native LEDC (pin-based API)
    ledcAttach(PIN_MOTOR_LEFT,  SERVO_LEDC_FREQ_HZ, SERVO_LEDC_RESOLUTION);
    ledcAttach(PIN_MOTOR_RIGHT, SERVO_LEDC_FREQ_HZ, SERVO_LEDC_RESOLUTION);

    // Flipper: regular PWM + direction pins (5 kHz, 10-bit)
    ledcAttach(PIN_FLIPPER_PWM, 5000, 10);

    pinMode(PIN_FLIPPER_DIR_A, OUTPUT);
    pinMode(PIN_FLIPPER_DIR_B, OUTPUT);
    digitalWrite(PIN_FLIPPER_DIR_A, LOW);
    digitalWrite(PIN_FLIPPER_DIR_B, LOW);
#endif
    neutralise();
}

// ─── Public API ──────────────────────────────────────────────────────────────

void Locomotion::setTrackSpeeds(float left_norm, float right_norm) {
    left_norm  = clampf(left_norm,  -1.0f, 1.0f);
    right_norm = clampf(right_norm, -1.0f, 1.0f);
    applyTrackSpeeds(left_norm, right_norm);
}

void Locomotion::setDriveCommand(float forward, float turn) {
#ifdef ROBOT_MAIN
    // Passthrough — caller is responsible for mixing
    setTrackSpeeds(forward, turn);
#else
    // Standard tank-drive mixing (ROBOT_SECONDARY)
    float left  = forward + turn;
    float right = forward - turn;
    float mag = fmaxf(fabsf(left), fabsf(right));
    if (mag > 1.0f) { left /= mag; right /= mag; }
    setTrackSpeeds(left, right);
#endif
}

void Locomotion::setFlipperEffort(float norm) {
    applyFlipperPWM(clampf(norm, -1.0f, 1.0f));
}

void Locomotion::setFlipperTargets(float fl, float fr, float rl, float rr) {
    applyFlipperSpeeds(clampf(fl, -1.0f, 1.0f), clampf(fr, -1.0f, 1.0f),
                       clampf(rl, -1.0f, 1.0f), clampf(rr, -1.0f, 1.0f));
}

void Locomotion::neutralise() {
#ifdef ROBOT_MAIN
    s_track_left_norm = s_track_right_norm = s_flipper_effort_norm = 0.0f;
    writeMicroseconds(PIN_MOTOR_LEFT,  MOTOR_NEUTRAL_US);
    writeMicroseconds(PIN_MOTOR_RIGHT, MOTOR_NEUTRAL_US);
    digitalWrite(PIN_FLIPPER_DIR_A, LOW);
    digitalWrite(PIN_FLIPPER_DIR_B, LOW);
    ledcWrite(PIN_FLIPPER_PWM, 0);
#elif defined(ROBOT_SECONDARY)
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    CANInterface::sendFlipperSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
#endif
}

// ─── Platform-specific output ────────────────────────────────────────────────

void Locomotion::applyTrackSpeeds(float left_norm, float right_norm) {
#ifdef ROBOT_MAIN
    // Apply motor direction correction (config.h)
    left_norm  *= TRACTION_DIR_LEFT;
    right_norm *= TRACTION_DIR_RIGHT;

    // Safety clamp to maximum output
    left_norm  = clampf(left_norm,  -TRACTION_MAX_NORM, TRACTION_MAX_NORM);
    right_norm = clampf(right_norm, -TRACTION_MAX_NORM, TRACTION_MAX_NORM);

    s_track_left_norm  = left_norm;
    s_track_right_norm = right_norm;

    // Normalised → microseconds → LEDC duty
    uint16_t left_us  = MOTOR_NEUTRAL_US + static_cast<int16_t>(left_norm  * (MOTOR_MAX_US - MOTOR_NEUTRAL_US));
    uint16_t right_us = MOTOR_NEUTRAL_US + static_cast<int16_t>(right_norm * (MOTOR_MAX_US - MOTOR_NEUTRAL_US));
    writeMicroseconds(PIN_MOTOR_LEFT,  left_us);
    writeMicroseconds(PIN_MOTOR_RIGHT, right_us);
#elif defined(ROBOT_SECONDARY)
    CANInterface::sendTrackSpeeds(left_norm, right_norm);
#endif
}

void Locomotion::applyFlipperPWM(float norm) {
#ifdef ROBOT_MAIN
    s_flipper_effort_norm = norm;
    norm = clampf(norm, -1.0f, 1.0f);
    uint32_t pwm_duty = static_cast<uint32_t>(fabsf(norm) * 1023.0f);  // 10-bit

    if (norm > 0.01f) {
        digitalWrite(PIN_FLIPPER_DIR_A, HIGH);
        digitalWrite(PIN_FLIPPER_DIR_B, LOW);
    } else if (norm < -0.01f) {
        digitalWrite(PIN_FLIPPER_DIR_A, LOW);
        digitalWrite(PIN_FLIPPER_DIR_B, HIGH);
    } else {
        // Brake: both LOW + 0 duty
        digitalWrite(PIN_FLIPPER_DIR_A, LOW);
        digitalWrite(PIN_FLIPPER_DIR_B, LOW);
        pwm_duty = 0;
    }

    ledcWrite(PIN_FLIPPER_PWM, pwm_duty);
#endif
}

void Locomotion::applyFlipperSpeeds(float fl, float fr, float rl, float rr) {
#ifdef ROBOT_SECONDARY
    CANInterface::sendFlipperSpeeds(fl, fr, rl, rr);
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
#endif
}
