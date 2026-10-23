#pragma once
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <vector>
#include <memory>
#include <random>
#include "pose.hpp"
#include "occupancy_grid_map.hpp"
#include "beam_range_finder_model.hpp"
#include "motion_velocity.hpp"
#include "control_rotation.hpp"
#include "ultrasonic_measurement.hpp"

namespace slam {

// Lightweight result type for localization: pose hypothesis + importance weight.
// Unlike BeliefWeightPair there is no per-particle map — the fixed world map is shared.
struct PoseWeightPair {
    Pose   pose;
    double weight{0.0};
};

// Monte Carlo Localization (MCL) — Probabilistic Robotics, Chapter 8.
//
// Use this instead of FastSLAM when the map is already known.
// All particles share one fixed OccupancyGridMap that is never modified.
// Each iteration:
//   1. Sample a new pose from the motion model.
//   2. Weight the pose against the fixed map using the beam sensor model.
//   3. Resample with LowVarianceSampler (Probabilistic Robotics, Table 4.4).
//
// Memory is O(map_size + N) rather than FastSLAM's O(N × map_size).
class MCLocalization {
public:
    MotionVelocity       motion_model;
    BeamRangeFinderModel measurement_model;

    // map_in is the pre-built world map; it is never modified by this class.
    explicit MCLocalization(std::shared_ptr<const OccupancyGridMap> map_in,
                            uint32_t seed = std::random_device{}())
        : map_(std::move(map_in)), rng_(seed) {}

    // Seed the resampling RNG (useful in unit tests for determinism).
    void set_seed(uint32_t seed) { rng_.seed(seed); }

    // One localization step.
    // Input:  N pose hypotheses from the previous step (or initial distribution).
    // Output: N resampled pose+weight pairs; the shared map is untouched.
    std::vector<PoseWeightPair> iterate(
        const std::vector<Pose>& poses,
        const ControlRotation& u_t,
        const UltraSonicMeasurement& z_t);

    const OccupancyGridMap& map() const { return *map_; }

private:
    std::shared_ptr<const OccupancyGridMap> map_;
    std::mt19937 rng_;
};

}  // namespace slam
