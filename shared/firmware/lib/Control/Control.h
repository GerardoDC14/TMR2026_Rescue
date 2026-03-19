#pragma once
#include "robot_types.h"

// ─── Control ──────────────────────────────────────────────────────────────────
// Top-level state machine.  Runs on Core 1 at PRIO_CONTROL.
//
// State transitions:
//
//  INIT ──(hw ready)──► STANDBY ──(PPM link)──► NORMAL / FLIPPER / ARM
//                                                         (follows Ch5)
//
//  Ch5 positions (3-position lever):
//    ~1000 µs  →  FLIPPER  (ROBOT_MAIN: tracks+flipper same as NORMAL;
//                           ROBOT_SECONDARY: Ch1-4 control individual flippers)
//    ~1500 µs  →  NORMAL   (Ch2/Ch4 tracks; Ch1 flipper on ROBOT_MAIN)
//    ~2000 µs  →  ARM      (all channels forwarded to mini PC for IK;
//                           ROBOT_SECONDARY also relays returned joint angles via CAN)
//
//  Any state ──(ESTOP from mini PC)──► ESTOP ──(ESTOP_CLEAR)──► STANDBY

class Control {
public:
    // One-time setup: register callbacks and store latest state.
    static void begin();

    // Main loop body — call from the control FreeRTOS task.
    static void tick();

    // ── Setters called from other tasks / callbacks ───────────────────────────
    static void triggerEstop();
    static void clearEstop();
    static void setArmJoints(const ArmJointsPayload& payload);
    static void setSensorMask(uint8_t mask);

    static RobotMode getMode();
    static void      getSystemStatus(SystemStatus& out);

private:
    // Per-mode update functions
    static void updateNormalMode(const PPMFrame& ppm, const EncoderState& enc);
    static void updateFlipperMode(const PPMFrame& ppm, const EncoderState& enc);
    static void updateArmMode(const PPMFrame& ppm);

    // Decode the 3-position Ch5 lever into a target RobotMode.
    // Returns NORMAL, FLIPPER, or ARM; never INIT / STANDBY / ESTOP.
    static RobotMode decodeModeFromCh5(const PPMFrame& ppm);

    static RobotMode  s_mode;
    static ArmJoints  s_arm_joints;
    static uint8_t    s_sensor_mask;
    // s_mux lives as a file-level static in Control.cpp (avoids FreeRTOS header in .h)
};
