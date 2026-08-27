#pragma once
#include <vector>
#include <cmath>
#include "grid_cell.hpp"
#include "pose.hpp"

namespace slam {

// Forward declarations to break circular dependency with BeamRangeFinderModel.
struct UltraSonicMeasurement;
class BeamRangeFinderModel;

class OccupancyGridMap {
public:
    std::vector<GridCell> m;
    int x_cells{0};
    int y_cells{0};

    OccupancyGridMap() = default;

    OccupancyGridMap(double cell_sz, int x_cells, int y_cells);

    // Deep-copy constructor (replaces C# ICloneable.Clone()).
    OccupancyGridMap(const OccupancyGridMap&) = default;
    OccupancyGridMap& operator=(const OccupancyGridMap&) = default;

    double cell_size() const {
        if (m.empty() || x_cells == 0 || y_cells == 0) return -1.0;
        return m[0].cell_size;
    }

    // Returns {col, row} of the cell containing the given pose.
    std::pair<int,int> get_cell_from_pose(const Pose& x_t) const;

    // Updates log-odds for every cell visible to the sensor.
    // Defined in occupancy_grid_map.cpp to avoid including beam_range_finder_model.hpp here.
    void update_map(const Pose& x_t,
                    const UltraSonicMeasurement& z_t,
                    const BeamRangeFinderModel& sensor);
};

}  // namespace slam
