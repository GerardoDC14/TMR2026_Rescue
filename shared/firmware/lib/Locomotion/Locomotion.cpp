#include "Locomotion.h"
#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <cmath>
#ifdef ROBOT_MAIN
#include <ESP32Servo.h>
#endif

float Locomotion::s_track_left_norm     = 0.0f;
float Locomotion::s_track_right_norm    = 0.0f;
float Locomotion::s_flipper_effort_norm = 0.0f;

#ifdef ROBOT_MAIN
Servo Locomotion::s_servo_left;
#endif

// ─── Helpers ─────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

uint16_t Locomotion::normToUs(float norm) {
    float us = MOTOR_NEUTRAL_US + norm * (MOTOR_MAX_US - MOTOR_NEUTRAL_US);
    return static_cast<uint16_t>(clampf(us, static_cast<float>(MOTOR_MIN_US),
                                            static_cast<float>(MOTOR_MAX_US)));
}

// ─── Initialisation ──────────────────────────────────────────────────────────

void Locomotion::begin() {
#ifdef ROBOT_MAIN
    // Left track: ESP32Servo library (timer 0)
    ESP32PWM::allocateTimer(0);
    s_servo_left.setPeriodHertz(50);
    s_servo_left.attach(PIN_MOTOR_LEFT, MOTOR_MIN_US, MOTOR_MAX_US);

    // Right track: native LEDC channel 2 (timer 1) — must be different backend than left
    ledcSetup(LEDC_CH_RIGHT, SERVO_LEDC_FREQ_HZ, SERVO_LEDC_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_RIGHT, LEDC_CH_RIGHT);

    // Flipper: native LEDC channel 4 (timer 2) — PWM + direction pins
    ledcSetup(LEDC_CH_FLIPPER, FLIPPER_PWM_FREQ_HZ, FLIPPER_PWM_RESOLUTION);
    ledcAttachPin(PIN_FLIPPER_PWM, LEDC_CH_FLIPPER);
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
    setTrackSpeeds(forward, turn);
#else
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

    // Left: ESP32Servo
    s_servo_left.writeMicroseconds(MOTOR_NEUTRAL_US);

    // Right: native LEDC
    uint32_t neutral_duty = (static_cast<uint32_t>(MOTOR_NEUTRAL_US) * SERVO_LEDC_MAX_DUTY) / SERVO_LEDC_PERIOD_US;
    ledcWrite(LEDC_CH_RIGHT, neutral_duty);

    // Flipper: brake
    digitalWrite(PIN_FLIPPER_DIR_A, LOW);
    digitalWrite(PIN_FLIPPER_DIR_B, LOW);
    ledcWrite(LEDC_CH_FLIPPER, 0);
#elif defined(ROBOT_SECONDARY)
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    CANInterface::sendFlipperSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
#endif
}

// ─── Platform-specific output ────────────────────────────────────────────────

void Locomotion::applyTrackSpeeds(float left_norm, float right_norm) {
#ifdef ROBOT_MAIN
    left_norm  *= TRACTION_DIR_LEFT;
    right_norm *= TRACTION_DIR_RIGHT;

    left_norm  = clampf(left_norm,  -TRACTION_MAX_NORM, TRACTION_MAX_NORM);
    right_norm = clampf(right_norm, -TRACTION_MAX_NORM, TRACTION_MAX_NORM);

    s_track_left_norm  = left_norm;
    s_track_right_norm = right_norm;

    uint16_t left_us  = normToUs(left_norm);
    uint16_t right_us = normToUs(right_norm);

    // Left: ESP32Servo writeMicroseconds
    s_servo_left.writeMicroseconds(left_us);

    // Right: native LEDC duty (same math as the proven test code)
    uint32_t duty_r = (static_cast<uint32_t>(right_us) * SERVO_LEDC_MAX_DUTY) / SERVO_LEDC_PERIOD_US;
    ledcWrite(LEDC_CH_RIGHT, duty_r);
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
        digitalWrite(PIN_FLIPPER_DIR_A, LOW);
        digitalWrite(PIN_FLIPPER_DIR_B, LOW);
        pwm_duty = 0;
    }

    ledcWrite(LEDC_CH_FLIPPER, pwm_duty);
#endif
}

void Locomotion::applyFlipperSpeeds(float fl, float fr, float rl, float rr) {
#ifdef ROBOT_SECONDARY
    CANInterface::sendFlipperSpeeds(fl, fr, rl, rr);
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
#endif
}
