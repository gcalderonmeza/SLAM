// Translation of UnitTests/MotionModelTests.cs
// Reference: Probabilistic Robotics, Thrun et al., Table 5.3.

#include <gtest/gtest.h>
#include <cmath>
#include "slam/motion_velocity.hpp"
#include "slam/control_rotation.hpp"
#include "slam/pose.hpp"

using namespace slam;

namespace {

const double kTol = 1e-9;

// Zero-noise model: all alphas = 0, sampler always returns 0.
// With zero noise: v_bar = v, w_bar = w, g_bar = 0.
MotionVelocity zero_noise_model(double dt = 1.0) {
    MotionVelocity m;
    m.alpha1 = m.alpha2 = m.alpha3 = m.alpha4 = m.alpha5 = m.alpha6 = 0.0;
    m.sampler = [](double) { return 0.0; };
    m.dt = dt;
    return m;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// Division-by-zero guard
// ─────────────────────────────────────────────────────────────

TEST(MotionModel, ZeroAngularVelocity_ProducesFiniteResult) {
    auto model = zero_noise_model(1.0);
    Pose pose(0, 0, 0);
    ControlRotation ctrl{1.0, 0.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_FALSE(std::isnan(r.x))     << "x must not be NaN when w=0";
    EXPECT_FALSE(std::isnan(r.y))     << "y must not be NaN when w=0";
    EXPECT_FALSE(std::isnan(r.theta)) << "theta must not be NaN when w=0";
    EXPECT_FALSE(std::isinf(r.x))     << "x must not be Inf when w=0";
    EXPECT_FALSE(std::isinf(r.y))     << "y must not be Inf when w=0";
    EXPECT_FALSE(std::isinf(r.theta)) << "theta must not be Inf when w=0";
}

TEST(MotionModel, NearZeroAngularVelocity_ProducesFiniteResult) {
    auto model = zero_noise_model(1.0);
    Pose pose(0, 0, 0);
    ControlRotation ctrl{1.0, 1e-10};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_FALSE(std::isnan(r.x) || std::isinf(r.x))     << "x finite for near-zero w";
    EXPECT_FALSE(std::isnan(r.y) || std::isinf(r.y))     << "y finite for near-zero w";
    EXPECT_FALSE(std::isnan(r.theta) || std::isinf(r.theta)) << "theta finite for near-zero w";
}

// ─────────────────────────────────────────────────────────────
// Straight-line motion (w = 0)
// ─────────────────────────────────────────────────────────────

TEST(MotionModel, ZeroAngularVelocity_PointingRight_MovesAlongX) {
    double dt = 0.5, v = 2.0;
    auto model = zero_noise_model(dt);
    Pose pose(3.0, 4.0, 0.0);
    ControlRotation ctrl{v, 0.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_NEAR(pose.x + v * dt, r.x,     kTol) << "x should advance by v*dt";
    EXPECT_NEAR(pose.y,          r.y,     kTol) << "y unchanged";
    EXPECT_NEAR(pose.theta,      r.theta, kTol) << "theta unchanged";
}

TEST(MotionModel, ZeroAngularVelocity_PointingUp_MovesAlongY) {
    double dt = 1.0, v = 3.0;
    auto model = zero_noise_model(dt);
    Pose pose(1.0, 2.0, M_PI / 2.0);
    ControlRotation ctrl{v, 0.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_NEAR(pose.x,          r.x,     1e-6) << "x nearly unchanged";
    EXPECT_NEAR(pose.y + v * dt, r.y,     kTol) << "y advances by v*dt";
    EXPECT_NEAR(pose.theta,      r.theta, kTol) << "theta unchanged";
}

TEST(MotionModel, ZeroVelocityAndAngularVelocity_RobotDoesNotMove) {
    auto model = zero_noise_model(1.0);
    Pose pose(5.0, 7.0, 1.2);
    ControlRotation ctrl{0.0, 0.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_NEAR(pose.x,     r.x,     kTol);
    EXPECT_NEAR(pose.y,     r.y,     kTol);
    EXPECT_NEAR(pose.theta, r.theta, kTol);
}

// ─────────────────────────────────────────────────────────────
// Circular arc motion (w != 0)
// ─────────────────────────────────────────────────────────────

// v=1, w=pi/4, dt=1, start (0,0,0).
// radius r = v/w = 4/pi
// x' = r*sin(w*dt), y' = r*(1-cos(w*dt)), theta' = w*dt
TEST(MotionModel, CircularArc_MatchesAnalyticFormula) {
    double dt = 1.0, v = 1.0, w = M_PI / 4.0;
    auto model = zero_noise_model(dt);
    Pose pose(0, 0, 0);
    ControlRotation ctrl{v, w};
    Pose r = model.sample_model(ctrl, pose);

    double radius   = v / w;
    double exp_x    = radius * std::sin(w * dt);
    double exp_y    = radius * (1.0 - std::cos(w * dt));
    double exp_th   = w * dt;

    EXPECT_NEAR(exp_x,  r.x,     kTol) << "x does not match circular-arc formula";
    EXPECT_NEAR(exp_y,  r.y,     kTol) << "y does not match circular-arc formula";
    EXPECT_NEAR(exp_th, r.theta, kTol) << "theta does not match circular-arc formula";
}

TEST(MotionModel, NegativeAngularVelocity_ProducesFiniteResult) {
    auto model = zero_noise_model(1.0);
    Pose pose(0, 0, 0);
    ControlRotation ctrl{1.0, -M_PI / 4.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_FALSE(std::isnan(r.x) || std::isinf(r.x));
    EXPECT_FALSE(std::isnan(r.y) || std::isinf(r.y));
    EXPECT_FALSE(std::isnan(r.theta) || std::isinf(r.theta));
}

TEST(MotionModel, LargeAngularVelocity_ProducesFiniteResult) {
    auto model = zero_noise_model(0.1);
    Pose pose(0, 0, 0);
    ControlRotation ctrl{1.0, 1000.0};
    Pose r = model.sample_model(ctrl, pose);
    EXPECT_FALSE(std::isnan(r.x) || std::isinf(r.x));
    EXPECT_FALSE(std::isnan(r.y) || std::isinf(r.y));
    EXPECT_FALSE(std::isnan(r.theta) || std::isinf(r.theta));
}

// ─────────────────────────────────────────────────────────────
// Continuity at the straight-line / arc boundary
// ─────────────────────────────────────────────────────────────

TEST(MotionModel, ContinuousAtBoundary_NearEpsilon) {
    double dt = 1.0, v = 1.0;
    auto model = zero_noise_model(dt);
    Pose pose(0, 0, 0);
    Pose at_zero   = model.sample_model({v, 0.0},  pose);
    Pose below_eps = model.sample_model({v, 5e-7},  pose);
    Pose above_eps = model.sample_model({v, 2e-6},  pose);
    EXPECT_NEAR(at_zero.x, below_eps.x, 1e-5) << "x discontinuity below epsilon";
    EXPECT_NEAR(at_zero.x, above_eps.x, 1e-5) << "x discontinuity above epsilon";
    EXPECT_NEAR(at_zero.y, below_eps.y, 1e-5) << "y discontinuity below epsilon";
    EXPECT_NEAR(at_zero.y, above_eps.y, 1e-5) << "y discontinuity above epsilon";
}

// ─────────────────────────────────────────────────────────────
// Noise integration
// ─────────────────────────────────────────────────────────────

TEST(MotionModel, WithNoise_ResultDiffersFromNoiselessBaseline) {
    double dt = 1.0;
    ControlRotation ctrl{1.0, 0.5};
    Pose pose(0, 0, 0);

    MotionVelocity noiseless = zero_noise_model(dt);
    Pose baseline = noiseless.sample_model(ctrl, pose);

    MotionVelocity noisy;
    noisy.alpha1 = noisy.alpha2 = noisy.alpha3 =
    noisy.alpha4 = noisy.alpha5 = noisy.alpha6 = 1.0;
    noisy.sampler = [](double) { return 0.1; };  // fixed additive noise
    noisy.dt = dt;
    Pose noisy_r = noisy.sample_model(ctrl, pose);

    bool differs = std::abs(noisy_r.x     - baseline.x)     > 1e-9
                || std::abs(noisy_r.y     - baseline.y)     > 1e-9
                || std::abs(noisy_r.theta - baseline.theta) > 1e-9;
    EXPECT_TRUE(differs) << "Noisy model produced same result as noiseless";
}
