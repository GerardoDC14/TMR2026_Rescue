#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── RC ───────────────────────────────────────────────────────────────────────
// Decodes a standard RC PPM stream using a GPIO interrupt on falling edges.
//
// Timing model (Flysky FS-i6):
//   Falling-to-falling interval = channel pulse width (1000–2000 µs), directly
//   comparable to PPM_MIN_US / PPM_MAX_US without any separator offset.
//   An interval > PPM_SYNC_US marks the frame boundary (sync HIGH >> 2000 µs).
//   The first PPM_CHANNELS intervals after the sync populate channels[0..5].
//
// Usage:
//   RC::begin(PIN_PPM);
//   RC::getFrame(frame);   // non-blocking; returns false if no new frame

class RC {
public:
    static void begin(uint8_t pin);

    // Returns true if a new, complete frame is available since last call.
    // 'out' is populated with the latest frame's values.
    static bool getFrame(PPMFrame& out);

    // True if a valid frame has arrived within PPM_TIMEOUT_MS.
    static bool isConnected();

private:
    static void isr();   // IRAM_ATTR applied on definition in .cpp
};
