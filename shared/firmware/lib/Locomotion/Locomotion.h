#pragma once
#include <stdint.h>

// ─── Locomotion ───────────────────────────────────────────────────────────────
// Provides high-level movement commands for the traction and flipper systems.
// The public API is robot-agnostic; hardware-specific output is encapsulated
// inside the private apply* methods and gated with #ifdef ROBOT_MAIN /
// ROBOT_SECONDARY.
//
// ROBOT_MAIN      — LEDC servo-PWM for two tracks + one flipper.
// ROBOT_SECONDARY — CAN motor controllers for two tracks + four flippers.
//
// All normalised values are in the range [-1.0, +1.0] unless otherwise noted.

class Locomotion {
public:
    static void begin();

    // High-level commands (platform-independent) ──────────────────────────────

    // Tank-drive mixing: forward ∈ [-1,1], turn ∈ [-1,1]
    // Computes left/right track speeds and routes to applyTrackSpeeds().
    static void setDriveCommand(float forward, float turn);

    // Set both tracks directly (e.g. from a speed controller).
    static void setTrackSpeeds(float left_norm, float right_norm);

    // ROBOT_MAIN: command the single flipper to a target angle (degrees).
    // Clamps to [FLIPPER_ANGLE_MIN, FLIPPER_ANGLE_MAX].
    static void setFlipperTarget(float angle_deg);

    // ROBOT_MAIN: apply a normalised effort [-1,1] directly (used by PID).
    static void setFlipperEffort(float norm);

    // ROBOT_SECONDARY: command all four independent flippers individually.
    // Each value is normalised [-1,1].  Order: front-left, front-right,
    // rear-left, rear-right.
    static void setFlipperTargets(float fl, float fr, float rl, float rr);

    // Immediately neutralise all outputs (tracks + flipper(s) → stopped).
    static void neutralise();

private:
    // Low-level output (platform-specific) ───────────────────────────────────

    // Write normalised [-1,1] speed to left and right track actuators.
    static void applyTrackSpeeds(float left_norm, float right_norm);

    // ROBOT_MAIN: write normalised [-1,1] command to the single flipper.
    static void applyFlipperPWM(float norm);

    // ROBOT_SECONDARY: write normalised [-1,1] commands to four flipper CAN
    // controllers.  Order: fl, fr, rl, rr.
    static void applyFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Convert normalised [-1,1] → PWM duty count for LEDC peripheral.
    // Used by ROBOT_MAIN only.
    static uint32_t normToDuty(float norm);

    static float  s_flipper_target_deg;
};
