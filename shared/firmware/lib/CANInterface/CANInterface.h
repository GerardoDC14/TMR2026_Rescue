#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"
#include <mcp2515.h>
#include "ODrive.hpp"

// ─── CANInterface ─────────────────────────────────────────────────────────────
// Wraps ESP32 TWAI (built-in CAN controller) with SN65HVD230 transceiver.
//
// Robot-type usage:
//   ROBOT_MAIN      — sendArmJoints() drives ODrive J1–J3 directly.
//                     (J4–J6 are servos on a separate ESP32.)
//   ROBOT_SECONDARY — sendArmJoints() drives ODrive J1–J6,
//                     sendTrackSpeeds() / sendFlipperSpeeds() drive VESCs,
//                     poll() reads VESC status feedback.

// Maximum number of VESC controllers we track feedback for
#define VESC_MAX_CONTROLLERS   6

class CANInterface {
public:
    // Initialise TWAI (ESP32 built-in CAN).  Returns false on driver failure.
    static bool begin();

    // Send joint angle targets to the arm over CAN.
    // ROBOT_MAIN: drives ODrive J1–J3 (ignores joints 4–6).
    // ROBOT_SECONDARY: drives ODrive J1–J6.
    static bool sendArmJoints(const float angles_deg[6]);

    // ── ROBOT_SECONDARY traction + flipper ────────────────────────────────────
    static bool sendTrackSpeeds(float left_norm, float right_norm);
    static bool sendFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Poll for incoming CAN frames (VESC status feedback).
    // Should be called periodically from the CAN task.
    static void poll();

    // True if the TWAI driver is installed and running.
    static bool isOk();

    // ── VESC feedback (ROBOT_SECONDARY) ───────────────────────────────────────
    // Returns true if status data is available for the given VESC ID (1-based).
    // Copies the latest status into `out` and clears the "fresh" flag.
    static bool getVescStatus(uint8_t vesc_id, VescStatusPayload& out);
};
