#include "../include/pid_controller.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

/**
 * Rotational Single-Axis Quadcopter Plant (Pitch/Roll):
 *   I * theta_ddot + b * theta_dot = tau_control + tau_disturbance
 *
 * Physical parameters derived from custom B-CUBE LX350 Pro quadcopter:
 *   Motor mass m = 0.055 kg, arm length d = 0.225 m
 *   Moment of inertia I = 2 * m * d^2 = 0.00557 kg*m^2
 *   Aerodynamic/bearing rotational damping b = 0.001 N*m*s/rad
 *
 * Pole-placement target (zeta = 0.70, omega_n = 5.0 rad/s):
 *   Kp = I * omega_n^2 = 0.1393 N*m/rad
 *   Kd = 2 * zeta * omega_n * I - b = 0.0380 N*m*s/rad
 */
struct State {
    double theta;      // rad
    double theta_dot;  // rad/s
};

State rk4_step(const State& s, double tau, double I, double b, double dt) {
    auto deriv = [&](const State& st) -> State {
        return {st.theta_dot, (tau - b * st.theta_dot) / I};
    };
    State k1 = deriv(s);
    State k2 = deriv({s.theta + 0.5 * dt * k1.theta, s.theta_dot + 0.5 * dt * k1.theta_dot});
    State k3 = deriv({s.theta + 0.5 * dt * k2.theta, s.theta_dot + 0.5 * dt * k2.theta_dot});
    State k4 = deriv({s.theta + dt * k3.theta, s.theta_dot + dt * k3.theta_dot});

    return {
        s.theta + (dt / 6.0) * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta),
        s.theta_dot + (dt / 6.0) * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot)
    };
}

struct Metrics {
    double overshoot_pct;
    double settling_time_s;
    double steady_state_err_pct;
};

Metrics run_case(const std::string& label, double kp, double ki, double kd,
                 std::ofstream& csv, double setpoint = 0.25) {
    constexpr double I = 0.00557;
    constexpr double b = 0.001;
    constexpr double dt = 0.002;     // 500 Hz Pixhawk attitude loop rate
    constexpr double t_end = 3.0;
    constexpr double wind_gust = -0.008; // Constant external wind-gust torque (N*m)

    quadcontrol::PIDController pid(kp, ki, kd);
    State st{0.0, 0.0};

    const int steps = static_cast<int>(t_end / dt);
    std::vector<double> hist_t, hist_theta;
    hist_t.reserve(steps + 1);
    hist_theta.reserve(steps + 1);

    double peak = 0.0;
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double u = pid.update(setpoint, st.theta, dt);
        csv << label << "," << std::fixed << std::setprecision(4)
            << t << "," << st.theta << "," << setpoint << "," << u << "\n";
        hist_t.push_back(t);
        hist_theta.push_back(st.theta);
        if (st.theta > peak) peak = st.theta;
        st = rk4_step(st, u + wind_gust, I, b, dt);
    }

    double final_val = hist_theta.back();
    double os_pct = ((peak - setpoint) / setpoint) * 100.0;
    double ess_pct = ((final_val - setpoint) / setpoint) * 100.0;

    // 5% tolerance band around setpoint
    double settle_t = t_end;
    for (int i = steps; i >= 0; --i) {
        if (std::abs(hist_theta[i] - setpoint) > 0.05 * setpoint) {
            settle_t = (i < steps) ? hist_t[i + 1] : t_end;
            break;
        }
        if (i == 0) settle_t = 0.0;
    }
    return {os_pct, settle_t, ess_pct};
}

int main() {
    std::ofstream csv("data/cpp_attitude_sim.csv");
    csv << "controller,time_s,theta_rad,setpoint_rad,torque_nm\n";

    auto m_p   = run_case("P_Only",             0.1393, 0.000, 0.0000, csv);
    auto m_pd  = run_case("Laplace_PD",         0.1393, 0.000, 0.0380, csv);
    auto m_pid = run_case("Full_PID_WindReject",0.1393, 0.220, 0.0380, csv);

    std::cout << "=== Quadcopter Rotational Attitude Simulation (500 Hz RK4, Wind Gust = -0.008 N*m) ===\n";
    auto print_row = [](const std::string& name, const Metrics& m) {
        std::cout << std::left << std::setw(22) << name
                  << " | Peak Overshoot: " << std::setw(7) << std::fixed << std::setprecision(2) << m.overshoot_pct << "%"
                  << " | 5% Settling Time: " << std::setw(5) << m.settling_time_s << " s"
                  << " | Steady-State Err: " << std::setw(7) << m.steady_state_err_pct << "%\n";
    };
    print_row("P-Only (Undamped)", m_p);
    print_row("Theoretical PD", m_pd);
    print_row("Full PID (Mission)", m_pid);
    return 0;
}
