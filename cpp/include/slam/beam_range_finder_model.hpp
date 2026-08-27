#pragma once
#include <vector>
#include <limits>
#include "pose.hpp"
#include "grid_cell.hpp"
#include "occupancy_grid_map.hpp"
#include "ultrasonic_measurement.hpp"

namespace slam {

// Beam range finder sensor model — Probabilistic Robotics §6.3.
class BeamRangeFinderModel {
public:
    // Maximum and minimum measurable range (cm).
    static constexpr double MAX_RANGE = 255.0;
    static constexpr double MIN_RANGE = 5.0;

    // Public parameters (match C# public fields).
    double z_max_prob;  // probability of max-range reading (zMax in C#)
    double beta;        // half-angle of field of view (radians, stored)

    BeamRangeFinderModel() = default;

    // beta_deg: field-of-view half-angle in DEGREES.
    // alpha: obstacle thickness (cm).
    // sigma_hit: std-dev of the Gaussian hit component (cm).
    BeamRangeFinderModel(double beta_deg, double alpha,
                         double z_hit, double z_short, double z_max, double z_rand,
                         double sigma_hit, double lambda_short);

    // p(z_t | x_t, m) — product over beams.
    double beam_range_finder(const UltraSonicMeasurement& z_t,
                             const Pose& x_t,
                             const OccupancyGridMap& m) const;

    // Inverse sensor model (log-odds update value for a single cell).
    double inverse_range_sensor_model(const GridCell& mi, const Pose& x_t,
                                      const UltraSonicMeasurement& z_t,
                                      int index_closest = -1,
                                      double angle_closest = std::numeric_limits<double>::max(),
                                      double dist_cm = 0.0) const;

    // True if the cell mi falls inside the perceptual field of any beam.
    // Outputs: index of the closest beam, angle to it, and distance to mi.
    bool in_perceptual_field(const GridCell& mi, const Pose& x_t,
                             const UltraSonicMeasurement& z_t,
                             int& index_closest,
                             double& angle_closest,
                             double& dist_cm) const;

    // EM algorithm to calibrate intrinsic parameters from logged data.
    void learn_intrinsic_parameters(const std::vector<UltraSonicMeasurement>& z,
                                    const std::vector<Pose>& x,
                                    const OccupancyGridMap& m);

private:
    double z_hit_{0.0};
    double z_short_{0.0};
    double z_rand_{0.0};
    double lambda_short_{0.0};
    double alpha_{0.0};
    double l_occ_{0.0};
    double l_free_{0.0};
    double sigma_hit_{1.0};

    double p_hit(double z_t, double k_t_star) const;
    double p_short(double z_t, double k_t_star) const;
    double p_max(double z_t) const;
    double p_rand(double z_t) const;
    double compute_eta(double mean, double variance) const;
    double compute_eta_iterative(double mean, double variance) const;

    // Ray-cast: returns expected range along beam direction (cm).
    double compute_z_star(const Pose& x_t, const OccupancyGridMap& m,
                          double angle_robot_coords) const;

    // Steps a ray to the boundary of the next grid cell.
    static void move_to_next_cell(const Pose& x_temp, double cell_size,
                                   double delta_x, double delta_y,
                                   double& rx2, double& ry2);
};

}  // namespace slam
