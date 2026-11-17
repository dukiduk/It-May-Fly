
#include "adrc3.hpp"
#include <cstring>
#include <cmath>

void ADRC3::init(const Config& cfg) {
    configure(cfg);
    reset(0.0f);
}

void ADRC3::reset(float initial_y) {
    z_[0] = initial_y;
    z_[1] = 0.0f;
    z_[2] = 0.0f;
    u_prev_ = 0.0f;
}

void ADRC3::configure(const Config& cfg) {
    cfg_ = cfg;

    // 1. Controller PD gains
    kp_ = cfg_.omega_c * cfg_.omega_c;
    kd_ = 2.0f * cfg_.omega_c;

    // Precompute nonlinear function parameters
    inverse_dpow1_ =
        std::pow(cfg_.delta, 1.0f - cfg_.alpha1);

    inverse_dpow2_ =
        std::pow(cfg_.delta, 1.0f - cfg_.alpha2);

    // 2. Observer gains (triple pole at -omega_o)
    const float wo = cfg_.omega_o;

    L_[0] = 3.0f * wo;
    L_[1] = 3.0f * wo * wo;
    L_[2] = wo * wo * wo;

    // 3. ZOH discretization via 10-term Taylor expansion
    const float dt = cfg_.dt;

    const float A_dt[3][3] = {
        {-L_[0] * dt, dt, 0.0f},
        {-L_[1] * dt, 0.0f, dt},
        {-L_[2] * dt, 0.0f, 0.0f}
    };

    float term[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    std::memcpy(Ad_, term, sizeof(term));
    std::memcpy(M_, term, sizeof(term));

    for (int k = 1; k <= 10; ++k) {
        float next[3][3] = {};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int m = 0; m < 3; ++m) {
                    next[i][j] += term[i][m] * A_dt[m][j];
                }
            }
        }

        const float inv_k = 1.0f / static_cast<float>(k);
        const float inv_k1 = 1.0f / static_cast<float>(k + 1);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                term[i][j] = next[i][j] * inv_k;
                Ad_[i][j] += term[i][j];
                M_[i][j] += term[i][j] * inv_k1;
            }
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M_[i][j] *= dt;
        }
    }
}

float ADRC3::fal_p(float e) {
    const float abs_e = std::fabs(e);

    if (abs_e <= cfg_.delta) {
        return e / inverse_dpow1_;
    }

    return std::copysign(
        std::pow(abs_e, cfg_.alpha1), e
    );
}

float ADRC3::fal_d(float e) {
    const float abs_e = std::fabs(e);

    if (abs_e <= cfg_.delta) {
        return e / inverse_dpow2_;
    }

    return std::copysign(
        std::pow(abs_e, cfg_.alpha2), e
    );
}

float ADRC3::rate_limit(
    float target,
    float prev,
    float max_rate,
    float dt
) {
    const float max_step = max_rate * dt;
    const float change = fmaxf(
        -max_step,
        fminf(target - prev, max_step)
    );

    return prev + change;
}

void ADRC3::update_eso(float y, float u) {
    const float forcing[3] = {
        L_[0] * y,
        cfg_.b0 * u + L_[1] * y,
        L_[2] * y
    };

    float z_new[3] = {};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            z_new[i] += Ad_[i][j] * z_[j]
                      + M_[i][j] * forcing[j];
        }
    }

    z_[0] = z_new[0];
    z_[1] = z_new[1];
    z_[2] = z_new[2];
}

float ADRC3::step(float ref, float ref_dot, float y_meas) {
    const float e1 = ref - z_[0];
    const float e2 = ref_dot - z_[1];

    float u0 = 0.0f;

    if (cfg_.use_nonlinear) {
        u0 = kp_ * fal_p(e1) + kd_ * fal_d(e2);
    } else {
        u0 = kp_ * e1 + kd_ * e2;
    }

    // Total disturbance cancellation
    const float u_raw = (u0 - z_[2]) / cfg_.b0;

    // Rate limiting and actuator saturation
    const float u_rl = rate_limit(
        u_raw, u_prev_, cfg_.u_rate_max, cfg_.dt
    );

    const float u_final = fmaxf(
        cfg_.u_min,
        fminf(u_rl, cfg_.u_max)
    );

    u_prev_ = u_final;

    // Feed actual actuator command into ESO
    update_eso(y_meas, u_final);

    return u_final;
}
