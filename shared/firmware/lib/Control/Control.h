#pragma once
#include "robot_types.h"

// ─── Control ──────────────────────────────────────────────────────────────────
// Top-level state machine.  Runs on Core 1 at PRIO_CONTROL.
//
// State transitions:
//
//  INIT ──(hw ready)──► STANDBY ──(PPM link)──► active mode (follows Ch5)
//
//  Ch5 positions (3-position lever) select which row of the keybind table
//  is active.  Each row maps channels 1-4,6 to a ChannelFunction.
//
//  The keybind table is received from the mini PC (MSG_KEYBIND).
//  Until a keybind is received, the default mapping is used:
//    mode0: Ch1=FLIPPER_ALL Ch2=TRACTION_FWD Ch4=TRACTION_TURN
//    mode1: Ch1=FLIPPER_ALL Ch2=TRACTION_FWD Ch4=TRACTION_TURN
//    mode2: Ch1-4=ARM_FWD
//
//  Any state ──(ESTOP from mini PC)──► ESTOP ──(ESTOP_CLEAR)──► STANDBY

class Control {
public:
    static void begin();
    static void tick();

    // ── Setters called from other tasks / callbacks ───────────────────────────
    static void triggerEstop();
    static void clearEstop();
    static void setArmJoints(const ArmJointsPayload& payload);
    static void setSensorMask(uint8_t mask);
    static void setKeybind(const KeybindPayload& payload);

    static RobotMode getMode();
    static void      getSystemStatus(SystemStatus& out);

private:
    // Apply the active keybind row to PPM channels
    static void applyKeybindRow(int mode_index, const PPMFrame& ppm,
                                const EncoderState& enc);

    // Decode the 3-position Ch5 lever into a mode index (0, 1, or 2)
    static int decodeModeIndex(const PPMFrame& ppm);

    static RobotMode     s_mode;
    static ArmJoints     s_arm_joints;
    static uint8_t       s_sensor_mask;
    static KeybindTable  s_keybind;

    // Flipper PID state (ROBOT_MAIN)
#ifdef ROBOT_MAIN
    static float s_pid_integral;
    static float s_pid_prev_err;
    static float flipperPID(float setpoint_deg, float measured_deg);
#endif
};
