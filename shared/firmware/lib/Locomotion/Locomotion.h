#pragma once
#include <stdint.h>
#include "config.h"

#ifdef ROBOT_MAIN
#include <ESP32Servo.h>
#endif

// ─── Locomotion ───────────────────────────────────────────────────────────────
// Pure output layer — takes normalised efforts and writes them to hardware.
//
// ROBOT_MAIN hardware quirk: the two track ESCs MUST use different PWM backends
// (one ESP32Servo, one native LEDC) to avoid LEDC timer crosstalk on the ESP32.
//
// ROBOT_SECONDARY — CAN motor controllers for two tracks + four flippers.
//
// All normalised values are in the range [-1.0, +1.0] unless otherwise noted.

class Locomotion {
public:
    static void begin();

    static void setTrackSpeeds(float left_norm, float right_norm);
    static void setDriveCommand(float forward, float turn);
    static void setFlipperEffort(float norm);
    static void setFlipperTargets(float fl, float fr, float rl, float rr);
    static void neutralise();

    static float getTrackLeft()     { return s_track_left_norm; }
    static float getTrackRight()    { return s_track_right_norm; }
    static float getFlipperEffort() { return s_flipper_effort_norm; }

private:
    static void applyTrackSpeeds(float left_norm, float right_norm);
    static void applyFlipperPWM(float norm);
    static void applyFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Convert normalised [-1,+1] to microseconds [MOTOR_MIN_US, MOTOR_MAX_US]
    static uint16_t normToUs(float norm);

    static float s_track_left_norm;
    static float s_track_right_norm;
    static float s_flipper_effort_norm;

#ifdef ROBOT_MAIN
    static Servo s_servo_left;   // left track — ESP32Servo library
    // right track uses native ledcWrite(PIN_MOTOR_RIGHT, ...)
#endif
};
