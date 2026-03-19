#include "RC.h"
#include "config.h"
#include <Arduino.h>

// ─── ISR-shared state (volatile) ─────────────────────────────────────────────
static volatile uint32_t s_last_edge_us  = 0;
static volatile uint8_t  s_channel_idx   = 0;
static volatile uint16_t s_raw[PPM_CHANNELS];   // captured in ISR

// ─── Snapshot for consumer (updated atomically under portENTER_CRITICAL) ──────
static portMUX_TYPE    s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t        s_frame[PPM_CHANNELS];
static uint32_t        s_frame_ms = 0;
static bool            s_frame_fresh = false;

// ─── ISR ─────────────────────────────────────────────────────────────────────
// Triggered on FALLING edges.
// Falling-to-falling interval = channel pulse width (1000–2000 µs), matching
// PPM_MIN_US / PPM_MAX_US in config.h.  Intervals > PPM_SYNC_US mark the
// frame boundary (sync gap HIGH period >> any channel value).
void IRAM_ATTR RC::isr() {
    const uint32_t now      = micros();
    const uint32_t interval = now - s_last_edge_us;
    s_last_edge_us = now;

    if (interval > PPM_SYNC_US) {
        // Sync gap: commit the previous frame if it was complete
        if (s_channel_idx >= PPM_CHANNELS) {
            portENTER_CRITICAL_ISR(&s_mux);
            for (uint8_t i = 0; i < PPM_CHANNELS; i++) s_frame[i] = s_raw[i];
            s_frame_ms    = millis();
            s_frame_fresh = true;
            portEXIT_CRITICAL_ISR(&s_mux);
        }
        s_channel_idx = 0;
    } else if (s_channel_idx < PPM_CHANNELS) {
        // Clamp to sane range before storing
        uint16_t clamped = (interval < 800u) ? 800u :
                           (interval > 2500u) ? 2500u : (uint16_t)interval;
        s_raw[s_channel_idx++] = clamped;
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────
void RC::begin(uint8_t pin) {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);
}

bool RC::getFrame(PPMFrame& out) {
    portENTER_CRITICAL(&s_mux);
    if (!s_frame_fresh) {
        portEXIT_CRITICAL(&s_mux);
        return false;
    }
    for (uint8_t i = 0; i < PPM_CHANNELS; i++) out.ch[i] = s_frame[i];
    out.timestamp_ms = s_frame_ms;
    out.valid        = true;
    s_frame_fresh    = false;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

bool RC::isConnected() {
    portENTER_CRITICAL(&s_mux);
    uint32_t last = s_frame_ms;
    portEXIT_CRITICAL(&s_mux);
    return (millis() - last) < PPM_TIMEOUT_MS;
}
