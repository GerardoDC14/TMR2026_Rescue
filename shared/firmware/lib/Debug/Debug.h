#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "config.h"

class PID;   // forward declaration for pidState()

// ─── Debug Topics ────────────────────────────────────────────────────────────
// Each topic is a single bit.  Combine with | for the initial mask.
namespace Dbg {
    constexpr uint32_t PPM       = (1 << 0);   // raw PPM channel values
    constexpr uint32_t TRACTION  = (1 << 1);   // traction commands / efforts
    constexpr uint32_t FLIPPER   = (1 << 2);   // flipper commands / efforts
    constexpr uint32_t ENCODER   = (1 << 3);   // raw encoder counts / speeds
    constexpr uint32_t MODE      = (1 << 4);   // state machine transitions
    constexpr uint32_t PID_TRACK = (1 << 5);   // traction PID internals
    constexpr uint32_t PID_FLIP  = (1 << 6);   // flipper PID internals
    constexpr uint32_t MOTOR     = (1 << 7);   // PWM / CAN motor output
    constexpr uint32_t ALL       = 0xFFFFFFFF;
}

// ─── Debug Class ─────────────────────────────────────────────────────────────
// Lightweight topic-gated debug output.  When ENABLE_COMMS is defined (binary
// protocol mode), all macros compile to no-ops so there is zero overhead.
//
// Usage:
//   Debug::begin(Dbg::ALL);           // enable every topic
//   Debug::disable(Dbg::ENCODER);     // turn one off at runtime
//
//   DBG(Dbg::MODE, "mode=%d\n", m);                  // immediate print
//   DBG_EVERY_MS(Dbg::PPM, 200, "ch1=%u\n", ch1);   // rate-limited
//   DBG_PID(Dbg::PID_TRACK, 100, "L", pid_left);     // PID state dump

class Debug {
public:
    static void     begin(uint32_t initial_mask = 0);
    static void     setMask(uint32_t mask);
    static uint32_t getMask();
    static void     enable(uint32_t topics);
    static void     disable(uint32_t topics);
    static bool     enabled(uint32_t topic);

    // Pretty-print a PID controller's internal state on one line.
    static void pidState(uint32_t topic, const char* label, const PID& pid);

private:
    static uint32_t s_mask;
};

// ─── Macros ──────────────────────────────────────────────────────────────────
// When ENABLE_COMMS is defined the Serial line carries the binary protocol, so
// debug output must be silenced.  The macros compile to nothing in that case.

#ifndef ENABLE_COMMS

// Immediate topic-gated print (no rate limit).
#define DBG(topic, fmt, ...) do { \
    if (Debug::enabled(topic)) { \
        Serial.printf(fmt, ##__VA_ARGS__); \
    } \
} while (0)

// Rate-limited topic-gated print.  Each call site has its own static timer so
// multiple DBG_EVERY_MS with the same topic don't share a rate limiter.
#define DBG_EVERY_MS(topic, interval_ms, fmt, ...) do { \
    if (Debug::enabled(topic)) { \
        static uint32_t _dbg_last = 0; \
        uint32_t _dbg_now = millis(); \
        if (_dbg_now - _dbg_last >= (uint32_t)(interval_ms)) { \
            _dbg_last = _dbg_now; \
            Serial.printf(fmt, ##__VA_ARGS__); \
        } \
    } \
} while (0)

// Rate-limited PID state dump.
#define DBG_PID(topic, interval_ms, label, pid) do { \
    if (Debug::enabled(topic)) { \
        static uint32_t _dbg_last = 0; \
        uint32_t _dbg_now = millis(); \
        if (_dbg_now - _dbg_last >= (uint32_t)(interval_ms)) { \
            _dbg_last = _dbg_now; \
            Debug::pidState(topic, label, pid); \
        } \
    } \
} while (0)

#else  // ENABLE_COMMS defined — compile everything away

#define DBG(topic, fmt, ...)                          ((void)0)
#define DBG_EVERY_MS(topic, interval_ms, fmt, ...)    ((void)0)
#define DBG_PID(topic, interval_ms, label, pid)       ((void)0)

#endif
