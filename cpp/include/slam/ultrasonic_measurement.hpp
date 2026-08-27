#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include "pose.hpp"
#include "grid_cell.hpp"

namespace slam {

struct UltraSonicMeasurement {
    std::vector<double> z;      // range readings (cm), one per beam
    std::vector<double> theta;  // beam angles in robot frame (rad)

    int beams() const { return static_cast<int>(theta.size()); }

    // Returns the Euclidean distance from x_t to mi's centre of mass.
    // Outputs the angular difference (rad) and the index of the closest beam.
    double find_range_and_angle_to_closest_beam(
        const GridCell& mi,
        const Pose& x_t,
        double& angle_to_closest_beam,
        int& index_to_closest) const
    {
        double dx = mi.xi - x_t.x;
        double dy = mi.yi - x_t.y;
        double r = std::sqrt(dx * dx + dy * dy);

        // Angle to cell centre in robot coordinates [-pi, pi]
        double phi = std::atan2(dy, dx) - x_t.theta;
        double normalized_phi = normalize_angle(phi);

        angle_to_closest_beam = std::numeric_limits<double>::max();
        index_to_closest = -1;

        for (int k = 0; k < static_cast<int>(z.size()); k++) {
            double diff = std::abs(normalize_angle(normalized_phi - theta[k]));
            if (diff < angle_to_closest_beam) {
                angle_to_closest_beam = diff;
                index_to_closest = k;
            }
        }
        return r;
    }

private:
    // Wraps angle_rad to [-pi, pi] — faithful translation of C# NormalizeAngle.
    static double normalize_angle(double angle_rad) {
        const double two_pi = 2.0 * M_PI;
        double sign = (angle_rad >= 0.0) ? 1.0 : -1.0;
        angle_rad = std::abs(angle_rad);
        if (angle_rad > two_pi) {
            double ratio = std::trunc(angle_rad / two_pi);
            angle_rad -= ratio * two_pi;
        }
        if (sign < 0.0) {
            return (angle_rad > M_PI) ? two_pi - angle_rad : -angle_rad;
        } else {
            return (angle_rad > M_PI) ? angle_rad - two_pi : angle_rad;
        }
    }
};

}  // namespace slam
