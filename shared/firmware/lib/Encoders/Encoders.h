#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Encoders ─────────────────────────────────────────────────────────────────
// Reads all three quadrature encoders using the ESP32 PCNT hardware peripheral.
// PCNT counts A/B phase transitions in hardware (4× decoding), eliminating
// CPU overhead on high-frequency pulses.
//
// Overflow: PCNT is a 16-bit signed counter.  An internal ISR detects the
// ±PCNT_HIGH_LIM / PCNT_LOW_LIM thresholds and accumulates overflow into a
// 32-bit shadow counter, giving effectively unlimited range.
//
// Derived values (speed & angle) are recomputed every ENC_SPEED_INTERVAL_MS;
// call updateDerivedValues() from a periodic task.

class Encoders {
public:
    static void begin();

    // Recalculate speed (RPM) and flipper angle from raw counts.
    // Call every ENC_SPEED_INTERVAL_MS milliseconds.
    static void updateDerivedValues();

    // Snapshot of the latest encoder state.
    static void getState(EncoderState& out);

    // Reset absolute position counters (e.g. after homing).
    static void resetFlipperAngle();
    static void resetTrackCounts();

private:
    static void initUnit(int unit, int pin_a, int pin_b);
    static int32_t getCount(int unit);
    static void overflowISR(void* arg);   // IRAM_ATTR applied on definition in .cpp
};