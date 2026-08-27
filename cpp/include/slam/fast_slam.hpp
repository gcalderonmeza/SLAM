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

class FastSLAM {
public:
    MotionVelocity motion_model;
    BeamRangeFinderModel measurement_model;

    FastSLAM() : rng_(std::random_device{}()) {}

    // Seed the resampling RNG (useful in unit tests for determinism).
    void set_seed(uint32_t seed) { rng_.seed(seed); }

    // One FastSLAM iteration: propagate → weight → resample.
    // Input particles (chi_t_1) are never modified.
    std::vector<BeliefWeightPair> iterate(
        const std::vector<BeliefeOccupancyGrid>& chi_t_1,
        const ControlRotation& u_t,
        const UltraSonicMeasurement& z_t);

private:
    std::mt19937 rng_;

    std::vector<BeliefWeightPair> sample_prob_distribution(
        int total_samples,
        const std::vector<BeliefWeightPair>& chi_t_bar);
};

}  // namespace slam
