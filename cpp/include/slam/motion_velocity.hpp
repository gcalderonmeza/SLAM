#pragma once
#include <cmath>
#include <functional>
#include "pose.hpp"
#include "control_rotation.hpp"
#include "distributions.hpp"

namespace slam {

// Velocity motion model — Probabilistic Robotics, Table 5.3.
struct MotionVelocity {
    double alpha1{0.0};  // translational noise from translation
    double alpha2{0.0};  // translational noise from rotation
    double alpha3{0.0};  // rotational noise from translation
    double alpha4{0.0};  // rotational noise from rotation
    double alpha5{0.0};  // drift noise from translation
    double alpha6{0.0};  // drift noise from rotation
    double dt{1.0};      // time step (s)

    // Noise sampler: takes variance, returns zero-mean noise sample.
    // Default: Gaussian via sum-of-uniforms approximation.
    std::function<double(double)> sampler{Distributions::make_normal_sampler()};

    // Samples a new pose from the velocity motion model.
    Pose sample_model(const ControlRotation& u_t, const Pose& x_t_1) const {
        double v2 = u_t.v * u_t.v;
        double w2 = u_t.w * u_t.w;
        double v_bar = u_t.v + sampler(alpha1 * v2 + alpha2 * w2);
        double w_bar = u_t.w + sampler(alpha3 * v2 + alpha4 * w2);
        double g_bar =         sampler(alpha5 * v2 + alpha6 * w2);

        double x_prime, y_prime, theta_prime;

        if (std::abs(w_bar) < 1e-6) {
            // Straight-line limit of the circular-arc formula as w → 0.
            // Avoids division by near-zero when angular velocity is negligible.
            x_prime     = x_t_1.x + v_bar * dt * std::cos(x_t_1.theta);
            y_prime     = x_t_1.y + v_bar * dt * std::sin(x_t_1.theta);
            theta_prime = x_t_1.theta + g_bar * dt;
        } else {
            x_prime     = x_t_1.x - (v_bar / w_bar)
                          * (std::sin(x_t_1.theta) - std::sin(x_t_1.theta + w_bar * dt));
            y_prime     = x_t_1.y + (v_bar / w_bar)
                          * (std::cos(x_t_1.theta) - std::cos(x_t_1.theta + w_bar * dt));
            theta_prime = x_t_1.theta + (w_bar + g_bar) * dt;
        }

        return Pose(x_prime, y_prime, theta_prime);
    }
};

}  // namespace slam
