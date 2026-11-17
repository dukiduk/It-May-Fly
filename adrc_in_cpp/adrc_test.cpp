
#include <iostream>
#include <fstream>
#include <iomanip>
#include "adrc3.hpp"

int main() {
    // 1. Configure ADRC
    ADRC3::Config cfg{};

    cfg.omega_c = 3.0f;
    cfg.omega_o = 15.0f;
    cfg.b0 = 2.0f;

    cfg.delta = 0.01f;
    cfg.alpha1 = 0.25f;
    cfg.alpha2 = 0.5f;

    cfg.dt = 0.005f;

    cfg.u_min = -3.3f;
    cfg.u_max = 3.3f;
    cfg.u_rate_max = 50.0f;

    cfg.use_nonlinear = true;

    ADRC3 controller;
    controller.init(cfg);

    // 2. Simulation parameters
    float x = 0.0f;
    float x_dot = 0.0f;

    const float dt = cfg.dt;
    const float ref = 10.0f;
    const float b0_true = 2.4f;

    const int steps = 2000;

    // 3. Open CSV file
    std::ofstream file("adrc_in_cpp/log.csv");

    if (!file.is_open()) {
        std::cerr << "Error: could not open log.csv for writing\n";
        return 1;
    }

    file << "t,x,u,z1_hat,z2_hat,z3_hat\n";
    file << std::fixed << std::setprecision(5);

    // 4. Simulation loop
    for (int k = 0; k < steps; ++k) {
        const float t = k * dt;

        const float disturbance = (t > 5.0f) ? 3.0f * std::sin(t) : 4.0f * std::cos(t) + 5.0f * std::sin(t);

        // Calculate controller output
        const float u = controller.step(ref, 0.0f, x);

        // Simulate second-order plant:
        // x_ddot = disturbance + b0_true * u
        const float x_ddot = disturbance + b0_true * u;

        x_dot += x_ddot * dt;
        x += x_dot * dt;

        // Log simulation data
        file << t << ","
             << x << ","
             << u << ","
             << controller.position_est() << ","
             << controller.velocity_est() << ","
             << controller.disturbance_est() << "\n";
    }

    file.close();

    // 5. Results
    std::cout << "Wrote " << steps << " rows to log.csv\n";
    std::cout << "Final x = " << x
              << " (target = " << ref << ")\n";
    return 0;
}
