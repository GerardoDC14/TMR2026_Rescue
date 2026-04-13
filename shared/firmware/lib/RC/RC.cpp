#include "RC.h"
#include "config.h"
#include <Arduino.h>

// ─── ISR-shared state (volatile) ─────────────────────────────────────────────
static volatile uint32_t s_last_edge_us  = 0;
static volatile uint8_t  s_channel_idx   = 0;
static volatile uint16_t s_raw[PPM_CHANNELS];   // captured in ISR

// ─── Snapshot for consumer (updated atomically under portENTER_CRITICAL) ──────
static portMUX_TYPE    s_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── Runtime calibration (defaults match compile-time constants) ─────────────
// Initialise all 6 channels to identical defaults; updated via setCalib().
static PpmCalibPayload s_calib = {
    {
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
        {PPM_MIN_US, (PPM_MIN_US + PPM_MAX_US) / 2, PPM_MAX_US},
    },
    50,  // deadband_1000: default 0.05 (5%)
};
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
        // Ignore physically impossible pulses (hardware noise/bounce)
        if (interval < 400u) {
            return; // Exit ISR entirely, wait for the real edge
        }
        
        // Clamp out-of-bounds but valid-looking signals
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

void RC::peekFrame(PPMFrame& out) {
    portENTER_CRITICAL(&s_mux);
    for (uint8_t i = 0; i < PPM_CHANNELS; i++) out.ch[i] = s_frame[i];
    out.timestamp_ms = s_frame_ms;
    out.valid        = (s_frame_ms != 0);
    portEXIT_CRITICAL(&s_mux);
    // s_frame_fresh is intentionally NOT cleared
}

bool RC::isConnected() {
    portENTER_CRITICAL(&s_mux);
    uint32_t last = s_frame_ms;
    portEXIT_CRITICAL(&s_mux);
    return (millis() - last) < PPM_TIMEOUT_MS;
}

void RC::setCalib(const PpmCalibPayload& p) {
    portENTER_CRITICAL(&s_mux);
    s_calib = p;
    portEXIT_CRITICAL(&s_mux);
}

float RC::normalise(uint8_t channel, uint16_t raw_us) {
    if (channel >= PPM_CHANNELS) return 0.0f;
    portENTER_CRITICAL(&s_mux);
    uint16_t mn  = s_calib.ch[channel].min_us;
    uint16_t neu = s_calib.ch[channel].neutral_us;
    uint16_t mx  = s_calib.ch[channel].max_us;
    portEXIT_CRITICAL(&s_mux);

    float v;
    if (raw_us >= neu) {
        float half = static_cast<float>(mx - neu);
        v = (half > 0.0f) ? static_cast<float>(raw_us - neu) / half : 0.0f;
    } else {
        float half = static_cast<float>(neu - mn);
        v = (half > 0.0f) ? -static_cast<float>(neu - raw_us) / half : 0.0f;
    }
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}
