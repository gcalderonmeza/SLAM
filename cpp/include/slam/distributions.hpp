#pragma once
#include <cmath>
#include <random>
#include <functional>

namespace slam {

namespace Distributions {

// Global Mersenne-Twister engine, seeded once per process.
inline std::mt19937& global_rng() {
    static std::mt19937 rng(std::random_device{}());
    return rng;
}

// Uniform sample in [min, max].
inline double rand_uniform(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(global_rng());
}

// Box-Muller approximation: sum of 12 uniform samples (Probabilistic Robotics §5.4).
inline double sample_normal(double variance) {
    double b = std::sqrt(variance);
    double sum = 0.0;
    for (int i = 0; i < 12; i++)
        sum += rand_uniform(-b, b);
    return 0.5 * sum;
}

// Triangular approximation of normal.
inline double sample_triangular(double variance) {
    static const double factor = std::sqrt(6.0) / 2.0;
    double b = std::sqrt(variance);
    return factor * (rand_uniform(-b, b) + rand_uniform(-b, b));
}

// Gaussian PDF.
inline double normal(double z, double mean, double variance) {
    return std::exp(-0.5 * (z - mean) * (z - mean) / variance)
           / std::sqrt(2.0 * M_PI * variance);
}

// Convenience: a std::function wrapping sample_normal (use as `sampler` in MotionVelocity).
inline std::function<double(double)> make_normal_sampler() {
    return [](double variance) { return sample_normal(variance); };
}

}  // namespace Distributions

}  // namespace slam
