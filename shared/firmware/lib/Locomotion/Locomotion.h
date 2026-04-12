#pragma once
#include <stdint.h>
#include "config.h"

// ─── Locomotion ───────────────────────────────────────────────────────────────
// Pure output layer — takes normalised efforts and writes them to hardware.
// No control logic or PID here; all closed-loop control lives in Control.
//
// ROBOT_MAIN      — LEDC servo PWM (writeMicroseconds) for two tracks + PWM+DIR flipper.
// ROBOT_SECONDARY — CAN motor controllers for two tracks + four flippers.
//
// All normalised values are in the range [-1.0, +1.0] unless otherwise noted.

class Locomotion {
public:
    static void begin();

    // Set track motor efforts directly [-1, 1].
    // Direction correction and TRACTION_MAX_NORM safety clamp applied internally.
    static void setTrackSpeeds(float left_norm, float right_norm);

    // Tank-drive mixing: forward ∈ [-1,1], turn ∈ [-1,1] → left/right speeds.
    // ROBOT_SECONDARY uses standard tank-drive; ROBOT_MAIN passes through.
    static void setDriveCommand(float forward, float turn);

    // ROBOT_MAIN: apply normalised flipper effort [-1, 1].
    static void setFlipperEffort(float norm);

    // ROBOT_SECONDARY: command four independent flippers [-1, 1].
    static void setFlipperTargets(float fl, float fr, float rl, float rr);

    // Stop all outputs immediately.
    static void neutralise();

    // Last commanded efforts (for telemetry).
    static float getTrackLeft()     { return s_track_left_norm; }
    static float getTrackRight()    { return s_track_right_norm; }
    static float getFlipperEffort() { return s_flipper_effort_norm; }

private:
    static void applyTrackSpeeds(float left_norm, float right_norm);
    static void applyFlipperPWM(float norm);
    static void applyFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Write a microsecond pulse width to a pin via LEDC (v3 API)
    static void writeMicroseconds(uint8_t pin, uint16_t us);

    static float s_track_left_norm;
    static float s_track_right_norm;
    static float s_flipper_effort_norm;
};
