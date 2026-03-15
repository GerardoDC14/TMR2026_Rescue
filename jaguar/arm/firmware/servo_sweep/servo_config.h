#pragma once

// ── Joint 4 calibration ────────────────────────────────────────────────────────
// MoveIt  0 rad  →  Servo  90°
// MoveIt +PI/2   →  Servo 170°
// MoveIt -PI/2   →  Servo   5°
#define J4_MOVEIT_MIN_RAD  (-1.5708f)
#define J4_MOVEIT_MAX_RAD  ( 1.5708f)
#define J4_SERVO_MIN_DEG   (  5.0f)
#define J4_SERVO_MAX_DEG   (170.0f)
#define J4_PULSE_MIN_US    500
#define J4_PULSE_MAX_US    2500

// ── Joint 5 calibration  ───────────────────────
// MoveIt -PI/2   →  Servo   0°
// MoveIt  0 rad  →  Servo  75°
// MoveIt +PI/2   →  Servo 170°
#define J5_P1_MOVEIT_DEG   (-90.0f)
#define J5_P1_SERVO_DEG    (  0.0f)
#define J5_P2_MOVEIT_DEG   (  0.0f)
#define J5_P2_SERVO_DEG    ( 75.0f)
#define J5_P3_MOVEIT_DEG   ( 90.0f)
#define J5_P3_SERVO_DEG    (170.0f)
#define J5_PULSE_MIN_US    800
#define J5_PULSE_MAX_US    2100

// ── Joint 6 calibration ────────────────────────────────────────────────────────
// MoveIt  0 rad  →  Servo  90°
// MoveIt +PI/2   →  Servo 180°
// MoveIt -PI/2   →  Servo   0°
#define J6_MOVEIT_MIN_RAD  (-1.5708f)
#define J6_MOVEIT_MAX_RAD  ( 1.5708f)
#define J6_SERVO_MIN_DEG   (  0.0f)
#define J6_SERVO_MAX_DEG   (180.0f)
#define J6_PULSE_MIN_US    500
#define J6_PULSE_MAX_US    2500

// ── Gripper ────────────────────────────────────────────────────────────────────
// Degrees sent directly from ROS 2; clamped here before writing
#define GR_SERVO_MIN_DEG   (100.0f)   // fully closed
#define GR_SERVO_MAX_DEG   (180.0f)   // fully open
#define GR_PULSE_MIN_US    800
#define GR_PULSE_MAX_US    2100

// ── Shared PWM ─────────────────────────────────────────────────────────────────
#define SERVO_FREQ_HZ      330

// ── Motion smoothing ───────────────────────────────────────────────────────────
// µs advanced per millisecond.
// Full 2000 µs range: 30 → ~67 ms travel (fast enough to follow MoveIt,
// slow enough to ramp safely on servo reconnect instead of snapping).
#define SPEED_US_PER_MS    30.0f
