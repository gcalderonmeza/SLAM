#pragma once
#include <vector>
#include <random>
#include "belief_occupancy_grid.hpp"
#include "beam_range_finder_model.hpp"
#include "motion_velocity.hpp"
#include "control_rotation.hpp"
#include "ultrasonic_measurement.hpp"

namespace slam {

struct BeliefWeightPair {
    BeliefeOccupancyGrid grid;
    double weight{0.0};
};

// Low-variance (systematic) resampler — Probabilistic Robotics, Table 4.4.
//
// Draws a single random offset r in [0, 1/N) and steps through the
// cumulative-weight ladder in equal strides of 1/N. This guarantees that
// every particle with weight >= 1/N is selected at least once, which
// preserves diversity far better than the multinomial sampler used in
// FastSLAM::sample_prob_distribution().
//
// Usage: construct once, call resample() each iteration.
class LowVarianceSampler {
public:
    explicit LowVarianceSampler(uint32_t seed = std::random_device{}())
        : rng_(seed) {}

    void set_seed(uint32_t seed) { rng_.seed(seed); }

    std::vector<BeliefWeightPair> resample(
        int total_samples,
        const std::vector<BeliefWeightPair>& chi_t_bar);

private:
    std::mt19937 rng_;
};

class FastSLAM {
public:
    MotionVelocity motion_model;
    BeamRangeFinderModel measurement_model;

    FastSLAM() : rng_(std::random_device{}()) {}

    // Seed both RNGs for deterministic runs (e.g. unit tests).
    void set_seed(uint32_t seed) { rng_.seed(seed); sampler_.set_seed(seed + 1); }

    // One FastSLAM iteration: propagate → weight → resample.
    // Resampling uses LowVarianceSampler (Probabilistic Robotics Table 4.4).
    // Input particles (chi_t_1) are never modified.
    std::vector<BeliefWeightPair> iterate(
        const std::vector<BeliefeOccupancyGrid>& chi_t_1,
        const ControlRotation& u_t,
        const UltraSonicMeasurement& z_t);

private:
    std::mt19937 rng_;
    LowVarianceSampler sampler_;

    // Multinomial resampler: draws N independent uniform samples over the
    // cumulative-weight ladder.
    //
    // Known limitation: because each draw is independent, the same high-weight
    // particle can be selected many times while low-weight particles are
    // dropped entirely. After several iterations this causes particle
    // degeneracy — the filter collapses to a few (or one) unique particle(s),
    // and the diversity needed for accurate SLAM is lost.
    //
    // Use LowVarianceSampler instead (Probabilistic Robotics, Table 4.4),
    // which draws a single random offset and steps through the ladder in equal
    // strides of 1/N, guaranteeing better coverage of the weight distribution.
    std::vector<BeliefWeightPair> sample_prob_distribution(
        int total_samples,
        const std::vector<BeliefWeightPair>& chi_t_bar);
};

}  // namespace slam
