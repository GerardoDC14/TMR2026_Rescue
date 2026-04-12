#include "PID.h"
#include <cmath>

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Shortest angular distance from `current` to `target`, result in [-180, +180].
static inline float shortestAngleDiff(float target, float current) {
    float d = fmodf(target - current, 360.0f);
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ─── Configuration ───────────────────────────────────────────────────────────

void PID::configure(float kp, float ki, float kd,
                    float i_max, float out_min, float out_max, float d_alpha) {
    m_kp      = kp;
    m_ki      = ki;
    m_kd      = kd;
    m_i_max   = i_max;
    m_out_min = out_min;
    m_out_max = out_max;
    m_d_alpha = d_alpha;
    reset();
}

// ─── Linear update ───────────────────────────────────────────────────────────

float PID::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return m_output;

    m_error = setpoint - measurement;

    // Derivative on measurement (negated) — avoids setpoint kick
    float d_raw = m_first ? 0.0f : -(measurement - m_prev_meas) / dt;
    m_prev_meas = measurement;

    // D-term low-pass filter
    m_d_filtered = m_d_alpha * m_d_filtered + (1.0f - m_d_alpha) * d_raw;

    // Anti-windup: stop integrating when output is saturated in the
    // same direction as the error (more integral would only make it worse)
    bool at_limit = (m_output >= m_out_max && m_error > 0)
                 || (m_output <= m_out_min && m_error < 0);
    if (!at_limit) {
        m_integral += m_error * dt;
    }

    // Decay integral toward zero when error is small — prevents stale
    // integral from producing nonzero output at rest
    if (fabsf(m_error) < 1.0f) {
        m_integral *= 0.95f;
    }
    m_integral = clampf(m_integral, -m_i_max, m_i_max);

    m_p_term = m_kp * m_error;
    m_i_term = m_ki * m_integral;
    m_d_term = m_kd * m_d_filtered;

    m_output = clampf(m_p_term + m_i_term + m_d_term, m_out_min, m_out_max);
    m_first  = false;
    return m_output;
}

// ─── Angular update ──────────────────────────────────────────────────────────

float PID::updateAngular(float setpoint_deg, float measurement_deg, float dt) {
    if (dt <= 0.0f) return m_output;

    m_error = shortestAngleDiff(setpoint_deg, measurement_deg);

    // Derivative on error — measurement wrapping makes measurement-based
    // derivative unreliable at the 0°/360° boundary
    float d_raw = m_first ? 0.0f : (m_error - m_prev_err) / dt;
    m_prev_err = m_error;

    // D-term low-pass filter
    m_d_filtered = m_d_alpha * m_d_filtered + (1.0f - m_d_alpha) * d_raw;

    // Anti-windup (same logic as linear)
    bool at_limit = (m_output >= m_out_max && m_error > 0)
                 || (m_output <= m_out_min && m_error < 0);
    if (!at_limit) {
        m_integral += m_error * dt;
    }

    // Decay integral when error is small (5° threshold for angular)
    if (fabsf(m_error) < 5.0f) {
        m_integral *= 0.95f;
    }
    m_integral = clampf(m_integral, -m_i_max, m_i_max);

    m_p_term = m_kp * m_error;
    m_i_term = m_ki * m_integral;
    m_d_term = m_kd * m_d_filtered;

    m_output = clampf(m_p_term + m_i_term + m_d_term, m_out_min, m_out_max);
    m_first  = false;
    return m_output;
}

// ─── Reset / setGains ────────────────────────────────────────────────────────

void PID::reset() {
    m_error = m_integral = m_d_filtered = 0;
    m_prev_meas = m_prev_err = 0;
    m_p_term = m_i_term = m_d_term = 0;
    m_output = 0;
    m_first  = true;
}

void PID::setGains(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}
