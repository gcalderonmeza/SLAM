// Translation of UnitTests/SensorModelTuningTests.cs
//
// Verifies that sigmaHit must be set to a realistic value for the sensor model
// to drive particle-filter convergence.

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include "slam/beam_range_finder_model.hpp"
#include "slam/occupancy_grid_map.hpp"
#include "slam/ultrasonic_measurement.hpp"
#include "slam/grid_cell.hpp"
#include "slam/pose.hpp"

using namespace slam;

namespace {

const int X_CELLS  = 10;
const int Y_CELLS  = 10;
const double CELL_SIZE = 10.0;

// 10×10 grid, 10 cm cells. Column 3 is fully occupied.
// Robot at (5, 5, 0) pointing right → ray hits column 3 at ~25 cm.
OccupancyGridMap make_map_with_obstacle_at_col3() {
    OccupancyGridMap map(CELL_SIZE, X_CELLS, Y_CELLS);
    for (int y = 0; y < Y_CELLS; y++)
        map.m[3 + y * X_CELLS].occupancy_log_odds =
            GridCell::convert_prob_to_log_odds(0.9);
    return map;
}

UltraSonicMeasurement make_measurement(double z, double theta) {
    UltraSonicMeasurement m;
    m.z     = {z};
    m.theta = {theta};
    return m;
}

BeamRangeFinderModel make_sensor(double sigma_hit) {
    return BeamRangeFinderModel(
        /*beta_deg=*/40, /*alpha=*/10,
        /*z_hit=*/0.71, /*z_short=*/0.08, /*z_max=*/0.09, /*z_rand=*/0.12,
        /*sigma_hit=*/sigma_hit, /*lambda_short=*/0.5);
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// Weight ratio as a function of sigmaHit
// ─────────────────────────────────────────────────────────────

// With sigma=5 cm the matching particle must outscore the mismatching one by >10×.
TEST(SensorModelTuning, AppropiateSigma_MatchingOutscoresMismatching_ByLargeFactor) {
    auto map  = make_map_with_obstacle_at_col3();
    auto meas = make_measurement(25.0, 0.0);

    Pose pose_match   (5, 5, 0);        // kTStar ≈ 25 cm
    Pose pose_mismatch(5, 5, M_PI);     // kTStar = 255 cm

    auto sensor = make_sensor(5.0);

    double w_match    = sensor.beam_range_finder(meas, pose_match,    map);
    double w_mismatch = sensor.beam_range_finder(meas, pose_mismatch, map);
    double ratio      = w_match / w_mismatch;

    EXPECT_GT(ratio, 10.0)
        << "With sigma=5 cm the matching particle should score >10x the mismatch. "
        << "ratio=" << ratio;
}

// With sigma=0.13 cm PHit collapses to ~0; ratio should be near 1.
TEST(SensorModelTuning, TightSigma_MatchingAndMismatchingGetSimilarWeights) {
    auto map  = make_map_with_obstacle_at_col3();
    auto meas = make_measurement(25.0, 0.0);

    Pose pose_match   (5, 5, 0);
    Pose pose_mismatch(5, 5, M_PI);

    auto sensor = make_sensor(0.13);

    double w_match    = sensor.beam_range_finder(meas, pose_match,    map);
    double w_mismatch = sensor.beam_range_finder(meas, pose_mismatch, map);
    double ratio      = w_match / w_mismatch;

    EXPECT_LT(ratio, 5.0)
        << "With sigma=0.13 cm PHit≈0 for both so ratio should be near 1. "
        << "ratio=" << ratio;
}

// ─────────────────────────────────────────────────────────────
// PHit shape: weight decreases with deviation
// ─────────────────────────────────────────────────────────────

TEST(SensorModelTuning, AppropiateSigma_WeightDecreasesAsDeviationIncreases) {
    auto map = make_map_with_obstacle_at_col3();
    Pose pose(5, 5, 0);  // kTStar ≈ 25 cm

    auto sensor = make_sensor(5.0);

    double w25 = sensor.beam_range_finder(make_measurement(25.0, 0.0), pose, map);
    double w30 = sensor.beam_range_finder(make_measurement(30.0, 0.0), pose, map);
    double w35 = sensor.beam_range_finder(make_measurement(35.0, 0.0), pose, map);
    double w40 = sensor.beam_range_finder(make_measurement(40.0, 0.0), pose, map);

    EXPECT_GT(w25, w30) << "Weight at z=25 (perfect match) > z=30";
    EXPECT_GT(w30, w35) << "Weight at z=30 > z=35";
    EXPECT_GT(w35, w40) << "Weight at z=35 > z=40";
}

TEST(SensorModelTuning, TightSigma_WeightIsNearlyFlatAcrossReasonableRange) {
    auto map = make_map_with_obstacle_at_col3();
    Pose pose(5, 5, 0);

    auto sensor = make_sensor(0.13);

    double w25 = sensor.beam_range_finder(make_measurement(25.0, 0.0), pose, map);
    double w27 = sensor.beam_range_finder(make_measurement(27.0, 0.0), pose, map);
    double w30 = sensor.beam_range_finder(make_measurement(30.0, 0.0), pose, map);

    double max_w = std::max({w25, w27, w30});
    double min_w = std::min({w25, w27, w30});
    double flatness = max_w / min_w;

    EXPECT_LT(flatness, 2.0)
        << "With sigma=0.13 cm all weights should be nearly equal. flatness=" << flatness;
}

// ─────────────────────────────────────────────────────────────
// PRand floor: weights never reach zero
// ─────────────────────────────────────────────────────────────

TEST(SensorModelTuning, BeamRangeFinder_Weight_IsAlwaysPositive) {
    OccupancyGridMap empty_map(CELL_SIZE, X_CELLS, Y_CELLS);
    Pose pose(5, 5, 0);

    std::vector<double> sigmas = {0.1, 0.5, 1.0, 5.0, 20.0};
    std::vector<double> zvals  = {1.0, 50.0, 100.0, 200.0};

    for (double sigma : sigmas) {
        auto sensor = make_sensor(sigma);
        for (double z : zvals) {
            double w = sensor.beam_range_finder(make_measurement(z, 0.0), pose, empty_map);
            EXPECT_GT(w, 0.0)
                << "Weight must be > 0 (PRand floor) for sigma=" << sigma << " z=" << z;
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Quantify improvement from sigma fix (sigma=5 ratio >> sigma=0.13 ratio)
// ─────────────────────────────────────────────────────────────

TEST(SensorModelTuning, AppropiateSigma_WeightRatioMuchHigherThan_TightSigma) {
    auto map  = make_map_with_obstacle_at_col3();
    auto meas = make_measurement(25.0, 0.0);
    Pose pose_match   (5, 5, 0);
    Pose pose_mismatch(5, 5, M_PI);

    auto sensor_tight = make_sensor(0.13);
    double ratio_tight = sensor_tight.beam_range_finder(meas, pose_match, map)
                       / sensor_tight.beam_range_finder(meas, pose_mismatch, map);

    auto sensor_good  = make_sensor(5.0);
    double ratio_good = sensor_good.beam_range_finder(meas, pose_match, map)
                      / sensor_good.beam_range_finder(meas, pose_mismatch, map);

    EXPECT_GT(ratio_good, 100.0 * ratio_tight)
        << "sigma=5 cm ratio should be >100x the sigma=0.13 cm ratio. "
        << "ratio_good=" << ratio_good << ", ratio_tight=" << ratio_tight;
}
