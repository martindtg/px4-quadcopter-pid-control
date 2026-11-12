#pragma once
#include <algorithm>
#include <cmath>

namespace quadcontrol {

/**
 * Discrete-time PID controller with integral clamping (anti-windup)
 * and first-order low-pass derivative filtering on measurement error.
 */
class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double max_output = 2.5,
                  double integral_limit = 1.0,
                  double derivative_tau = 0.015)
        : kp_(kp), ki_(ki), kd_(kd),
          max_out_(max_output), int_lim_(integral_limit), tau_(derivative_tau),
          integral_(0.0), prev_error_(0.0), d_filtered_(0.0), initialized_(false) {}

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        d_filtered_ = 0.0;
        initialized_ = false;
    }

    double update(double setpoint, double measurement, double dt) {
        if (dt <= 0.0) return 0.0;
        const double error = setpoint - measurement;

        // Trapezoidal integral with anti-windup clamping
        if (initialized_) {
            integral_ += 0.5 * (error + prev_error_) * dt;
        } else {
            integral_ += error * dt;
        }
        integral_ = std::clamp(integral_, -int_lim_, int_lim_);

        // Filtered derivative term
        double raw_deriv = initialized_ ? (error - prev_error_) / dt : 0.0;
        const double alpha = dt / (tau_ + dt);
        d_filtered_ = initialized_ ? (alpha * raw_deriv + (1.0 - alpha) * d_filtered_) : 0.0;

        prev_error_ = error;
        initialized_ = true;

        const double u = kp_ * error + ki_ * integral_ + kd_ * d_filtered_;
        return std::clamp(u, -max_out_, max_out_);
    }

private:
    double kp_, ki_, kd_;
    double max_out_, int_lim_, tau_;
    double integral_, prev_error_, d_filtered_;
    bool initialized_;
};

}  // namespace quadcontrol
