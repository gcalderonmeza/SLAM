#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include "slam/map_io.hpp"
#include <fstream>
#include <stdexcept>
#include <iomanip>

namespace slam {

void save_map(const OccupancyGridMap& map, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open())
        throw std::runtime_error("save_map: cannot open '" + path + "'");

    f << std::setprecision(17)
      << map.x_cells << " " << map.y_cells << " " << map.cell_size() << "\n";

    for (const auto& cell : map.m)
        f << std::setprecision(17) << cell.occupancy_log_odds << "\n";
}

OccupancyGridMap load_map(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("load_map: cannot open '" + path + "'");

    int x_cells = 0, y_cells = 0;
    double cell_sz = 0.0;
    if (!(f >> x_cells >> y_cells >> cell_sz))
        throw std::runtime_error("load_map: malformed header in '" + path + "'");

    OccupancyGridMap map(cell_sz, x_cells, y_cells);
    for (auto& cell : map.m) {
        if (!(f >> cell.occupancy_log_odds))
            throw std::runtime_error("load_map: truncated data in '" + path + "'");
    }
    return map;
}

}  // namespace slam
