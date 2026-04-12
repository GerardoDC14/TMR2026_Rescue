#pragma once

// ─── PID Controller ──────────────────────────────────────────────────────────
// Reusable PID with two update modes:
//
//   update()        — linear error, derivative-on-measurement (no setpoint kick)
//   updateAngular() — shortest-path angular error, derivative-on-error
//
// Features:
//   • D-term low-pass filter (first-order IIR, configurable alpha)
//   • Anti-windup: conditional integration (stops when output saturates
//     in the same direction as the error)
//   • Per-instance output clamping
//   • Full state inspection for debug/telemetry

class PID {
public:
    // Configure all parameters.  Call once at startup or when re-tuning.
    //   kp, ki, kd     — standard gains
    //   i_max          — integral magnitude clamp (positive)
    //   out_min/max    — output clamp range
    //   d_alpha        — D-term EMA filter coefficient (0 = no filter, 0.9 = heavy)
    void configure(float kp, float ki, float kd,
                   float i_max   = 10.0f,
                   float out_min = -1.0f, float out_max = 1.0f,
                   float d_alpha = 0.0f);

    // Linear PID.  Derivative is computed on the measurement (negated) to
    // avoid derivative kick when the setpoint changes abruptly.
    float update(float setpoint, float measurement, float dt);

    // Angular PID.  Error is the shortest path in [-180°, +180°].
    // Derivative is computed on the error (measurement wrapping makes
    // measurement-based derivative unreliable at the 0°/360° boundary).
    float updateAngular(float setpoint_deg, float measurement_deg, float dt);

    // Zero all internal state.  Safe to call at any time.
    void reset();

    // Hot-swap gains without resetting accumulated state.
    void setGains(float kp, float ki, float kd);

    // ── State inspection (read-only, for debug) ─────────────────────────────
    float error()    const { return m_error; }
    float integral() const { return m_integral; }
    float pTerm()    const { return m_p_term; }
    float iTerm()    const { return m_i_term; }
    float dTerm()    const { return m_d_term; }
    float output()   const { return m_output; }

private:
    // Gains
    float m_kp = 0, m_ki = 0, m_kd = 0;
    float m_i_max   = 10.0f;
    float m_out_min = -1.0f, m_out_max = 1.0f;
    float m_d_alpha = 0.0f;

    // Running state
    float m_error      = 0;
    float m_integral   = 0;
    float m_prev_meas  = 0;   // for derivative-on-measurement (linear)
    float m_prev_err   = 0;   // for derivative-on-error (angular)
    float m_d_filtered = 0;
    float m_p_term = 0, m_i_term = 0, m_d_term = 0;
    float m_output = 0;
    bool  m_first  = true;
};
