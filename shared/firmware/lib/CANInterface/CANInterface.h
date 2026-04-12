#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── CANInterface ─────────────────────────────────────────────────────────────
// Wraps ESP32 TWAI (built-in CAN controller) with SN65HVD230 transceiver.
//
// Robot-type usage:
//   ROBOT_MAIN      — sendArmJoints() drives ODrive J1–J3 directly.
//                     (J4–J6 are servos on a separate ESP32.)
//   ROBOT_SECONDARY — sendArmJoints() drives ODrive J1–J6,
//                     sendTrackSpeeds() / sendFlipperSpeeds() drive VESCs,
//                     poll() reads VESC status feedback.

// Maximum number of VESC controllers we track feedback for (flippers only now)
#define VESC_MAX_CONTROLLERS   6

// ODrive arm joints (J1–J3 on both robots)
#define ODRIVE_MAX_JOINTS      3

// Traction ODrives on ROBOT_SECONDARY (left, right — separate from arm)
#define TRACTION_ODRIVE_NUM    2

// LKTech arm joints on ROBOT_SECONDARY (J5, J6)
#define LKTECH_MAX_JOINTS      2

class CANInterface {
public:
    // Initialise TWAI (ESP32 built-in CAN).  Returns false on driver failure.
    static bool begin();

    // Send joint angle targets to the arm over CAN.
    // ROBOT_MAIN:      drives ODrive J1–J3 (ignores joints 4–6).
    // ROBOT_SECONDARY: drives ODrive J1–J3, ZE300 J4, LKTech J5–J6.
    static bool sendArmJoints(const float angles_deg[6]);

    // ── ROBOT_SECONDARY traction + flipper ────────────────────────────────────
    // Inputs are normalised −1..+1 (full reverse..full forward).
    // Traction: scaled to turns/s internally via TRACTION_MAX_VEL_TURNS_S.
    // Flippers: scaled to eRPM (SET_RPM) or current (SET_CURRENT) per config.
    static bool sendTrackSpeeds(float left_norm, float right_norm);
    static bool sendFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Poll for incoming CAN frames and send periodic telemetry RTR requests.
    // Should be called periodically from the CAN task (~200 Hz).
    static void poll();

    // True if the TWAI driver is installed and running.
    static bool isOk();

    // ── VESC feedback (ROBOT_SECONDARY flippers) ─────────────────────────────
    static bool getVescStatus(uint8_t vesc_id, VescStatusPayload& out);

    // ── ODrive arm feedback (both robots, J1–J3) ─────────────────────────────
    static bool getOdriveStatus(uint8_t joint_idx, OdriveStatusPayload& out);

    // ── Traction ODrive feedback (ROBOT_SECONDARY, left/right) ───────────────
    // motor_idx: 0=left, 1=right.
    static bool getTractionStatus(uint8_t motor_idx, OdriveStatusPayload& out);

    // ── ODrive error snapshot (arm + traction, gated by ODRIVE_ENABLE_ERROR_POLL)
    // node_idx: iterates over ALL ODrive nodes (arm 0..2, then traction 3..4
    // on ROBOT_SECONDARY).  Returns true + clears fresh flag if error data ready.
    static bool getOdriveError(uint8_t node_idx, OdriveErrorPayload& out);

    // Total number of ODrive nodes on this robot variant (for iterating errors).
    static uint8_t odriveNodeCount();

    // ── ODrive E-stop / recovery ────────────────────────────────────────────
    // Behaviour controlled by ODRIVE_ESTOP_USE_NATIVE in config.h:
    //   1 = native cmd 0x02 (forces IDLE, needs clearEstop to recover)
    //   0 = soft stop (zero velocity/hold position, no recovery needed)
    static void estopAllOdrives();
    static void clearEstopAllOdrives();

    // ── LKTech feedback (ROBOT_SECONDARY J5/J6) ──────────────────────────────
    static bool getLktechStatus(uint8_t joint_idx, LktechStatusPayload& out);

    // ── ZE300 feedback (ROBOT_SECONDARY J4) ──────────────────────────────────
    static bool getZe300Status(Ze300StatusPayload& out);
};
