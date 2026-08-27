#include "slam/occupancy_grid_map.hpp"
#include "slam/beam_range_finder_model.hpp"
#include "slam/ultrasonic_measurement.hpp"

namespace slam {

OccupancyGridMap::OccupancyGridMap(double cell_sz, int x_cells, int y_cells)
    : x_cells(x_cells), y_cells(y_cells)
{
    int n = x_cells * y_cells;
    m.resize(n);

    // Cells are stored column-major: index = x + y * x_cells.
    // Initialized to 50% unknown (log-odds = 0).
    double prior = GridCell::convert_prob_to_log_odds(0.5);
    for (int x = 0; x < x_cells; x++) {
        for (int y = 0; y < y_cells; y++) {
            int i = x + y * x_cells;
            m[i].xi               = cell_sz * x + cell_sz / 2.0;
            m[i].yi               = cell_sz * y + cell_sz / 2.0;
            m[i].cell_size        = cell_sz;
            m[i].prior_occupancy_prob  = prior;
            m[i].occupancy_log_odds    = prior;
        }
    }
}

std::pair<int,int> OccupancyGridMap::get_cell_from_pose(const Pose& x_t) const {
    int x = static_cast<int>(std::floor(x_t.x / cell_size()));
    int y = static_cast<int>(std::floor(x_t.y / cell_size()));
    return {x, y};
}

void OccupancyGridMap::update_map(const Pose& x_t,
                                   const UltraSonicMeasurement& z_t,
                                   const BeamRangeFinderModel& sensor)
{
    for (auto& mi : m) {
        int idx_closest;
        double angle_closest, dist_cm;
        if (sensor.in_perceptual_field(mi, x_t, z_t, idx_closest, angle_closest, dist_cm)) {
            double inv = sensor.inverse_range_sensor_model(
                mi, x_t, z_t, idx_closest, angle_closest, dist_cm);
            mi.occupancy_log_odds += inv - mi.prior_occupancy_prob;
        }
    }
}

}  // namespace slam
