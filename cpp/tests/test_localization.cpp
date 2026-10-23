// Tests for MCLocalization — Monte Carlo Localization with a fixed map.
// Reference: Probabilistic Robotics, Thrun et al., Chapter 8.
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include "slam/mc_localization.hpp"
#include "slam/occupancy_grid_map.hpp"
#include "slam/grid_cell.hpp"
#include "slam/distributions.hpp"

using namespace slam;

namespace {

// 10×10 grid, 10 cm cells. Column 3 fully occupied — same fixture as sensor tests.
// Robot at (50, 50, 0) pointing right sees the wall at ~25 cm.
std::shared_ptr<OccupancyGridMap> make_map_with_obstacle() {
    auto map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
    for (int y = 0; y < 10; y++)
        map->m[3 + y * 10].occupancy_log_odds =
            GridCell::convert_prob_to_log_odds(0.9);
    return map;
}

MCLocalization make_localizer(std::shared_ptr<OccupancyGridMap> map) {
    MCLocalization loc(map, /*seed=*/42);
    loc.motion_model.alpha1 = 0.005;
    loc.motion_model.alpha2 = 0.005;
    loc.motion_model.alpha3 = 0.001;
    loc.motion_model.alpha4 = 0.001;
    loc.motion_model.alpha5 = 0.0;
    loc.motion_model.alpha6 = 0.0;
    loc.motion_model.sampler = [](double v) { return Distributions::sample_normal(v); };
    loc.motion_model.dt = 0.5;
    loc.measurement_model = BeamRangeFinderModel(
        40, 10, 0.71, 0.08, 0.09, 0.12, 5.0, 0.5);
    return loc;
}

ControlRotation zero_control() { return ControlRotation{0.0, 0.0}; }

UltraSonicMeasurement make_meas(double z, double theta = 0.0) {
    UltraSonicMeasurement m;
    m.z     = {z};
    m.theta = {theta};
    return m;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Basic invariants
// ─────────────────────────────────────────────────────────────────────────────

TEST(MCLocalization, Iterate_OutputCount_EqualsInputCount) {
    auto map = make_map_with_obstacle();
    auto loc = make_localizer(map);

    const int N = 30;
    std::vector<Pose> poses(N, Pose(50, 50, 0));
    auto result = loc.iterate(poses, zero_control(), make_meas(25.0));

    EXPECT_EQ(N, static_cast<int>(result.size()))
        << "iterate() must return exactly as many particles as it received.";
}

TEST(MCLocalization, Iterate_AllWeights_AreNonNegative) {
    auto map = make_map_with_obstacle();
    auto loc = make_localizer(map);

    std::vector<Pose> poses(20, Pose(50, 50, 0));
    auto result = loc.iterate(poses, zero_control(), make_meas(25.0));

    for (const auto& p : result)
        EXPECT_GE(p.weight, 0.0) << "Weight " << p.weight << " is negative.";
}

// ─────────────────────────────────────────────────────────────────────────────
// The fixed map must never be modified
// ─────────────────────────────────────────────────────────────────────────────

TEST(MCLocalization, Iterate_DoesNotModifyFixedMap) {
    auto map = make_map_with_obstacle();

    std::vector<double> original;
    for (const auto& c : map->m) original.push_back(c.occupancy_log_odds);

    auto loc = make_localizer(map);
    std::vector<Pose> poses(20, Pose(50, 50, 0));
    loc.iterate(poses, zero_control(), make_meas(25.0));

    for (size_t i = 0; i < map->m.size(); i++)
        EXPECT_NEAR(original[i], map->m[i].occupancy_log_odds, 1e-10)
            << "Cell " << i << " was modified. The fixed map must not be touched.";
}

// ─────────────────────────────────────────────────────────────────────────────
// A particle at the correct pose must outscore one at the wrong pose
// ─────────────────────────────────────────────────────────────────────────────

TEST(MCLocalization, CorrectPose_GetsHigherWeight_ThanWrongPose) {
    auto map = make_map_with_obstacle();
    auto loc = make_localizer(map);

    // Matching pose: robot at (50, 50, 0) pointing right → obstacle at ~25 cm.
    // Mismatching pose: robot pointing away (pi) → no obstacle in that direction.
    std::vector<Pose> poses = {
        Pose(50, 50, 0),        // match
        Pose(50, 50, M_PI)      // mismatch
    };

    // Zero-noise control so poses don't drift.
    loc.motion_model.alpha1 = loc.motion_model.alpha2 = 0.0;
    loc.motion_model.alpha3 = loc.motion_model.alpha4 = 0.0;
    loc.motion_model.sampler = [](double) { return 0.0; };

    // Run one step and collect weights before resampling distorts them.
    // We test the measurement model score directly via the map accessor.
    double w_match    = loc.measurement_model.beam_range_finder(
                            make_meas(25.0), Pose(50, 50, 0),    *map);
    double w_mismatch = loc.measurement_model.beam_range_finder(
                            make_meas(25.0), Pose(50, 50, M_PI), *map);

    EXPECT_GT(w_match, w_mismatch)
        << "Correct pose (w=" << w_match << ") must outscore wrong pose (w=" << w_mismatch << ").";
}

// ─────────────────────────────────────────────────────────────────────────────
// Localization vs SLAM: memory footprint
// The map pointer must be shared, not copied per particle
// ─────────────────────────────────────────────────────────────────────────────

TEST(MCLocalization, Map_IsShared_NotCopiedPerParticle) {
    auto map = make_map_with_obstacle();
    long use_before = map.use_count();   // 1 (this test) + 1 inside localizer after construction

    auto loc = make_localizer(map);
    long use_after = map.use_count();

    // The localizer holds one extra shared_ptr; iterate() must not add more.
    std::vector<Pose> poses(50, Pose(50, 50, 0));
    loc.iterate(poses, zero_control(), make_meas(25.0));
    loc.iterate(poses, zero_control(), make_meas(25.0));

    EXPECT_EQ(use_after, map.use_count())
        << "iterate() must not create additional shared_ptr copies of the map.";
    (void)use_before;
}

// ─────────────────────────────────────────────────────────────────────────────
// Multiple iterations do not degrade — best weight stays stable
// ─────────────────────────────────────────────────────────────────────────────

TEST(MCLocalization, MultipleIterations_BestWeight_StaysPositive) {
    auto map = make_map_with_obstacle();
    auto loc = make_localizer(map);
    loc.set_seed(7);

    std::vector<Pose> poses(30, Pose(50, 50, 0));
    auto meas = make_meas(25.0);
    auto ctrl = ControlRotation{1.0, 0.0};

    for (int i = 0; i < 20; i++) {
        auto result = loc.iterate(poses, ctrl, meas);
        double best = 0.0;
        poses.clear();
        for (const auto& p : result) {
            poses.push_back(p.pose);
            best = std::max(best, p.weight);
        }
        EXPECT_GT(best, 0.0) << "Best weight must remain positive at step " << i;
    }
}
