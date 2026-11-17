#pragma once
#include <cmath>
#include <algorithm>

class ADRC3 {
public:
    struct Config {
        float omega_o;      // Observer bandwidth (rad/s)
        float omega_c;      // Controller bandwidth (rad/s)
        float b0;           // Nominal control effectiveness
        float dt;           // Sample period (s), e.g. 0.005f for 200 Hz
        float u_min;        // Actuator min limit
        float u_max;        // Actuator max limit
        float u_rate_max;   // Actuator slew rate limit (units/s)
        
        // Optional nonlinear (NLADRC) error feedback parameters
        bool  use_nonlinear = false;
        float alpha1 = 0.75f;  // 0 < alpha1 < 1 (proportional nonlinearity)
        float alpha2 = 1.25f;  // alpha2 > 1     (derivative nonlinearity)
        float delta  = 0.05f;  // Linear zone threshold around zero error
    };

    ADRC3() = default;
    explicit ADRC3(const Config& cfg) { init(cfg); }

    // Call at boot or whenever tuning parameters change (bumpless: preserves z state)
    void configure(const Config& cfg);

    // Call at boot or after disarming to zero states
    void init(const Config& cfg);
    void reset(float initial_y = 0.0f);

    // Fast gain-scheduling update for airspeed (dynamic pressure scaling) without recomputing Ad/M
    void set_b0(float new_b0) { cfg_.b0 = new_b0; }

    // Main 200 Hz control step
    float step(float ref, float ref_dot, float y_meas);

    // Telemetry getters for logging/tuning
    float position_est()    const { return z_[0]; }
    float velocity_est()    const { return z_[1]; }
    float disturbance_est() const { return z_[2]; }
    

private:
    Config cfg_{};

    // Precomputed ZOH matrices & gains
    float Ad_[3][3]{};
    float M_[3][3]{};
    float L_[3]{};
    float kp_ = 0.0f;
    float kd_ = 0.0f;

    // Runtime state
    float z_[3]{};
    float u_prev_ = 0.0f;

    float inverse_dpow1_ = 0.0f;
    float inverse_dpow2_ = 0.0f;

    void update_eso(float y, float u);
    static float fal_p(float e);
    static float fal_d(float e);
    static float rate_limit(float target, float prev, float max_rate, float dt);
};