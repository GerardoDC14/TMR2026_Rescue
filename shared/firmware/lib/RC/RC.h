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
    // Clears the "fresh" flag — only ONE caller receives true per frame.
    // Use this in the control task which must process each frame exactly once.
    static bool getFrame(PPMFrame& out);

    // Copies the last captured frame WITHOUT consuming the freshness flag.
    // Safe to call from any task that just needs the latest values for
    // reporting (e.g. telemetry), without racing against the control task.
    static void peekFrame(PPMFrame& out);

    // True if a valid frame has arrived within PPM_TIMEOUT_MS.
    static bool isConnected();

    // Update runtime PPM calibration for all channels at once.
    // Safe to call from any task.
    static void setCalib(const PpmCalibPayload& p);

    // Normalise raw PPM µs for a specific channel to [-1.0, +1.0].
    // Maps [min_us, neutral_us] → [-1, 0] and [neutral_us, max_us] → [0, +1].
    // channel is 0-indexed (0 = Ch1 … PPM_CHANNELS-1 = Ch6).
    static float normalise(uint8_t channel, uint16_t raw_us);

private:
    static void isr();   // IRAM_ATTR applied on definition in .cpp
};
