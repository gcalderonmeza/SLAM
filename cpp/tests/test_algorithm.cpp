// Translation of UnitTests/AlgorithmFixTests.cs

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include "slam/fast_slam.hpp"
#include "slam/occupancy_grid_map.hpp"
#include "slam/grid_cell.hpp"
#include "slam/belief_occupancy_grid.hpp"
#include "slam/control_rotation.hpp"
#include "slam/ultrasonic_measurement.hpp"
#include "slam/distributions.hpp"

using namespace slam;

namespace {

ControlRotation make_control() {
    return ControlRotation{1.0, 0.1};
}

UltraSonicMeasurement make_measurement(std::vector<double> z, std::vector<double> theta) {
    UltraSonicMeasurement m;
    m.z     = std::move(z);
    m.theta = std::move(theta);
    return m;
}

FastSLAM make_fast_slam() {
    FastSLAM slam;
    slam.motion_model.alpha1 = 0.005;
    slam.motion_model.alpha2 = 0.005;
    slam.motion_model.alpha3 = 0.001;
    slam.motion_model.alpha4 = 0.001;
    slam.motion_model.alpha5 = 0.0;
    slam.motion_model.alpha6 = 0.0;
    slam.motion_model.sampler = [](double v) { return Distributions::sample_normal(v); };
    slam.motion_model.dt = 0.5;
    slam.measurement_model = BeamRangeFinderModel(
        40, 10, 0.71, 0.08, 0.09, 0.12, 5.0, 0.5);
    slam.set_seed(42);  // deterministic resampling for test reproducibility
    return slam;
}

// 10×10 grid, 10 cm cells. Column 3 occupied.
// Robot at (5, 5, 0) pointing right → ray-cast ≈ 25 cm.
OccupancyGridMap make_map_with_obstacle() {
    OccupancyGridMap map(10.0, 10, 10);
    for (int y = 0; y < 10; y++)
        map.m[3 + y * 10].occupancy_log_odds =
            GridCell::convert_prob_to_log_odds(0.9);
    return map;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// Map-sharing bug fix
// ─────────────────────────────────────────────────────────────

// Core regression: iterate() must never modify the input particles' maps.
// All 5 particles intentionally share the same map (worst-case scenario).
TEST(AlgorithmFix, Iterate_DoesNotModifyInputMap) {
    auto shared_map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
    std::vector<double> original;
    for (auto& c : shared_map->m) original.push_back(c.occupancy_log_odds);

    std::vector<BeliefeOccupancyGrid> beliefs;
    for (int i = 0; i < 5; i++)
        beliefs.emplace_back(Pose(50, 50, i * M_PI / 5.0), shared_map, nullptr);

    make_fast_slam().iterate(beliefs, make_control(),
        make_measurement({40.0}, {0.0}));

    for (size_t i = 0; i < shared_map->m.size(); i++) {
        EXPECT_NEAR(original[i], shared_map->m[i].occupancy_log_odds, 1e-10)
            << "Cell " << i << " of the input map was modified. "
               "iterate() must clone before update_map().";
    }
}

// After iterate(), mutating one output particle's map must not affect others.
TEST(AlgorithmFix, Iterate_OutputMaps_AreIndependentObjects) {
    std::vector<BeliefeOccupancyGrid> beliefs;
    for (int i = 0; i < 10; i++) {
        auto map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
        beliefs.emplace_back(Pose(50, 50, 0), map, nullptr);
    }

    auto result = make_fast_slam().iterate(beliefs, make_control(),
        make_measurement({40.0}, {0.0}));

    std::vector<OccupancyGridMap*> maps;
    for (auto& bwp : result) maps.push_back(bwp.grid.map.get());

    std::vector<double> before;
    for (auto* m : maps) before.push_back(m->m[0].occupancy_log_odds);

    maps[0]->m[0].occupancy_log_odds = 9999.0;

    for (size_t i = 1; i < maps.size(); i++) {
        EXPECT_NEAR(before[i], maps[i]->m[0].occupancy_log_odds, 1e-10)
            << "Particle " << i << " map changed when particle 0's map was mutated.";
    }
}

// ─────────────────────────────────────────────────────────────
// Basic algorithm invariants
// ─────────────────────────────────────────────────────────────

TEST(AlgorithmFix, Iterate_OutputParticleCount_EqualsInputCount) {
    const int n = 50;
    std::vector<BeliefeOccupancyGrid> beliefs;
    for (int i = 0; i < n; i++) {
        auto map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
        beliefs.emplace_back(Pose(50, 50, i * 0.1), map, nullptr);
    }

    auto result = make_fast_slam().iterate(beliefs, make_control(),
        make_measurement({40.0, 40.0}, {0.0, M_PI / 2.0}));

    EXPECT_EQ(n, static_cast<int>(result.size()))
        << "iterate() must return exactly as many particles as it received.";
}

TEST(AlgorithmFix, Iterate_AllOutputWeights_AreNonNegative) {
    const int n = 20;
    std::vector<BeliefeOccupancyGrid> beliefs;
    for (int i = 0; i < n; i++) {
        auto map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
        beliefs.emplace_back(Pose(50, 50, i * 0.3), map, nullptr);
    }

    auto result = make_fast_slam().iterate(beliefs, make_control(),
        make_measurement({30.0}, {0.0}));

    for (auto& bwp : result) {
        EXPECT_GE(bwp.weight, 0.0)
            << "Weight " << bwp.weight << " is negative.";
    }
}

TEST(AlgorithmFix, Iterate_PoseHistory_GrowsByOnePerIteration) {
    const int n = 5;
    std::vector<BeliefeOccupancyGrid> beliefs;
    for (int i = 0; i < n; i++) {
        auto map = std::make_shared<OccupancyGridMap>(10.0, 10, 10);
        beliefs.emplace_back(Pose(50, 50, 0), map, nullptr);
    }

    // BeliefeOccupancyGrid constructor appends the initial pose → path starts at 1.
    EXPECT_EQ(1, static_cast<int>(beliefs[0].path.size()))
        << "Initial path length must be 1.";

    auto ctrl = make_control();
    auto meas = make_measurement({40.0}, {0.0});
    auto slam = make_fast_slam();

    auto result1 = slam.iterate(beliefs, ctrl, meas);
    EXPECT_EQ(2, static_cast<int>(result1[0].grid.path.size()))
        << "After iteration 1 path length must be 2.";

    std::vector<BeliefeOccupancyGrid> beliefs2;
    for (auto& bwp : result1) beliefs2.push_back(bwp.grid);

    auto result2 = slam.iterate(beliefs2, ctrl, meas);
    EXPECT_EQ(3, static_cast<int>(result2[0].grid.path.size()))
        << "After iteration 2 path length must be 3.";
}

// ─────────────────────────────────────────────────────────────
// Sensor model correctness
// ─────────────────────────────────────────────────────────────

TEST(AlgorithmFix, SensorModel_MatchingPose_GetsHigherWeight_ThanNonMatching) {
    auto map = make_map_with_obstacle();
    Pose pose_match   (5, 5, 0);
    Pose pose_mismatch(5, 5, M_PI);

    auto meas = make_measurement({25.0}, {0.0});
    auto sensor = make_fast_slam().measurement_model;

    double w_match    = sensor.beam_range_finder(meas, pose_match,    map);
    double w_mismatch = sensor.beam_range_finder(meas, pose_mismatch, map);

    EXPECT_GT(w_match, w_mismatch)
        << "Particle facing obstacle (w=" << w_match << ") must outweigh "
        << "particle facing away (w=" << w_mismatch << ").";
}

TEST(AlgorithmFix, SensorModel_MaxRangeReading_GetsHigherWeight_OnEmptyMap) {
    OccupancyGridMap empty_map(10.0, 10, 10);
    Pose pose(5, 5, 0);
    auto sensor = make_fast_slam().measurement_model;

    double w_max   = sensor.beam_range_finder(make_measurement({255.0}, {0.0}), pose, empty_map);
    double w_short = sensor.beam_range_finder(make_measurement({10.0},  {0.0}), pose, empty_map);

    EXPECT_GT(w_max, w_short)
        << "Max-range reading should outweigh short reading on empty map.";
}

// ─────────────────────────────────────────────────────────────
// Log-odds grid cell
// ─────────────────────────────────────────────────────────────

TEST(AlgorithmFix, GridCell_ConvertProbToLogOdds_RoundTrip) {
    std::vector<double> probs = {0.1, 0.3, 0.5, 0.7, 0.9};
    for (double p : probs) {
        double log_odds = GridCell::convert_prob_to_log_odds(p);
        double back     = GridCell::convert_log_odds_to_prob(log_odds);
        EXPECT_NEAR(p, back, 1e-10) << "Round-trip failed for p=" << p;
    }
}

TEST(AlgorithmFix, GridCell_ConvertProbToLogOdds_BoundaryThrows) {
    EXPECT_THROW(GridCell::convert_prob_to_log_odds(-0.01), std::invalid_argument)
        << "Expected exception for p=-0.01";
    EXPECT_THROW(GridCell::convert_prob_to_log_odds(1.01), std::invalid_argument)
        << "Expected exception for p=1.01";
}

TEST(AlgorithmFix, OccupancyGridMap_InitializedToUnknown) {
    OccupancyGridMap map(10.0, 5, 5);
    double expected = GridCell::convert_prob_to_log_odds(0.5);
    for (auto& cell : map.m) {
        EXPECT_NEAR(expected, cell.occupancy_log_odds,   1e-10)
            << "All cells must start at 50% prior.";
        EXPECT_NEAR(expected, cell.prior_occupancy_prob, 1e-10)
            << "Prior must also be 50%.";
    }
}
