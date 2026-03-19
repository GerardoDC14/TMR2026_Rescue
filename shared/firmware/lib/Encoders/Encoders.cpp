#include "Encoders.h"
#include "config.h"
#include <Arduino.h>
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"    // PCNT hardware register struct (for ISR)
#include "esp_intr_alloc.h"

// ─── Per-unit overflow accumulators (written in ISR) ─────────────────────────
static volatile int32_t s_overflow[3] = {0, 0, 0};

// ─── Derived state (protected by mutex) ──────────────────────────────────────
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
static EncoderState  s_state = {};

// Previous raw counts for delta calculation
static int32_t s_prev_count[3] = {0, 0, 0};
static uint32_t s_last_update_ms = 0;

// ─── PCNT overflow ISR ────────────────────────────────────────────────────────
// Called when any PCNT unit crosses its high or low limit.
void IRAM_ATTR Encoders::overflowISR(void* /*arg*/) {
    uint32_t status = PCNT.int_st.val;
    PCNT.int_clr.val = status;

    for (int u = 0; u < 3; u++) {
        if (status & BIT(u)) {
            if (PCNT.status_unit[u].h_lim_lat) {
                s_overflow[u] += PCNT_HIGH_LIM;
            } else if (PCNT.status_unit[u].l_lim_lat) {
                s_overflow[u] += PCNT_LOW_LIM;
            }
        }
    }
}

// ─── Private helpers ─────────────────────────────────────────────────────────
void Encoders::initUnit(int unit, int pin_a, int pin_b) {
    pcnt_unit_t u = static_cast<pcnt_unit_t>(unit);

    // Channel 0: A is pulse, B is control → 2× decode on A edges
    pcnt_config_t cfg_a = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num  = pin_b,
        .lctrl_mode     = PCNT_MODE_REVERSE,  // B=LOW  → reverse count direction
        .hctrl_mode     = PCNT_MODE_KEEP,     // B=HIGH → normal count direction
        .pos_mode       = PCNT_COUNT_INC,     // rising A  → increment
        .neg_mode       = PCNT_COUNT_DEC,     // falling A → decrement
        .counter_h_lim  = PCNT_HIGH_LIM,
        .counter_l_lim  = PCNT_LOW_LIM,
        .unit           = u,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg_a);

    // Channel 1: B is pulse, A is control → adds 2× decode on B edges (total 4×)
    pcnt_config_t cfg_b = {
        .pulse_gpio_num = pin_b,
        .ctrl_gpio_num  = pin_a,
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_REVERSE,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DEC,
        .counter_h_lim  = PCNT_HIGH_LIM,
        .counter_l_lim  = PCNT_LOW_LIM,
        .unit           = u,
        .channel        = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&cfg_b);

    // Enable glitch filter (100 ns min pulse) to reject electrical noise
    pcnt_set_filter_value(u, 100);
    pcnt_filter_enable(u);

    // Enable overflow events on this unit
    pcnt_event_enable(u, PCNT_EVT_H_LIM);
    pcnt_event_enable(u, PCNT_EVT_L_LIM);

    pcnt_counter_pause(u);
    pcnt_counter_clear(u);
    pcnt_counter_resume(u);
}

int32_t Encoders::getCount(int unit) {
    int16_t raw = 0;
    pcnt_get_counter_value(static_cast<pcnt_unit_t>(unit), &raw);
    // s_overflow is written in ISR — read with a critical section snapshot
    portENTER_CRITICAL(&s_mux);
    int32_t ov = s_overflow[unit];
    portEXIT_CRITICAL(&s_mux);
    return ov + raw;
}

// ─── Public API ──────────────────────────────────────────────────────────────
void Encoders::begin() {
    initUnit(PCNT_UNIT_LEFT,    PIN_ENC_LEFT_A,  PIN_ENC_LEFT_B);
    initUnit(PCNT_UNIT_RIGHT,   PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B);
    initUnit(PCNT_UNIT_FLIPPER, PIN_ENC_FLIP_A,  PIN_ENC_FLIP_B);

    // Register a single ISR for all PCNT units
    pcnt_isr_register(overflowISR, nullptr, 0, nullptr);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_1);
    pcnt_intr_enable(PCNT_UNIT_2);

    s_last_update_ms = millis();
}

void Encoders::updateDerivedValues() {
    const uint32_t now    = millis();
    const float    dt_s   = (now - s_last_update_ms) * 1e-3f;
    if (dt_s <= 0.0f) return;
    s_last_update_ms = now;

    const int32_t cnt_left    = getCount(PCNT_UNIT_LEFT);
    const int32_t cnt_right   = getCount(PCNT_UNIT_RIGHT);
    const int32_t cnt_flipper = getCount(PCNT_UNIT_FLIPPER);

    // Delta counts since last call
    const int32_t d_left    = cnt_left    - s_prev_count[PCNT_UNIT_LEFT];
    const int32_t d_right   = cnt_right   - s_prev_count[PCNT_UNIT_RIGHT];
    const int32_t d_flipper = cnt_flipper - s_prev_count[PCNT_UNIT_FLIPPER];

    s_prev_count[PCNT_UNIT_LEFT]    = cnt_left;
    s_prev_count[PCNT_UNIT_RIGHT]   = cnt_right;
    s_prev_count[PCNT_UNIT_FLIPPER] = cnt_flipper;

    // Speed: (counts / CPR) / dt * 60 / gear_ratio → output shaft RPM
    const float rpm_left  = (d_left  / ENC_CPR_TRACK)   / dt_s * 60.0f / TRACK_GEAR_RATIO;
    const float rpm_right = (d_right / ENC_CPR_TRACK)    / dt_s * 60.0f / TRACK_GEAR_RATIO;

    // Flipper angle: cumulative position from power-on (or last reset)
    const float angle_deg = (static_cast<float>(cnt_flipper) / ENC_CPR_FLIPPER)
                            * 360.0f / FLIPPER_GEAR_RATIO;

    portENTER_CRITICAL(&s_mux);
    s_state.count_left        = cnt_left;
    s_state.count_right       = cnt_right;
    s_state.count_flipper     = cnt_flipper;
    s_state.speed_left_rpm    = rpm_left;
    s_state.speed_right_rpm   = rpm_right;
    s_state.flipper_angle_deg = angle_deg;
    s_state.timestamp_ms      = now;
    portEXIT_CRITICAL(&s_mux);
}

void Encoders::getState(EncoderState& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_state;
    portEXIT_CRITICAL(&s_mux);
}

void Encoders::resetFlipperAngle() {
    portENTER_CRITICAL(&s_mux);
    s_overflow[PCNT_UNIT_FLIPPER] = 0;
    s_prev_count[PCNT_UNIT_FLIPPER] = 0;
    portEXIT_CRITICAL(&s_mux);
    pcnt_counter_clear(static_cast<pcnt_unit_t>(PCNT_UNIT_FLIPPER));
}

void Encoders::resetTrackCounts() {
    for (int u : {PCNT_UNIT_LEFT, PCNT_UNIT_RIGHT}) {
        portENTER_CRITICAL(&s_mux);
        s_overflow[u] = 0;
        s_prev_count[u] = 0;
        portEXIT_CRITICAL(&s_mux);
        pcnt_counter_clear(static_cast<pcnt_unit_t>(u));
    }
}