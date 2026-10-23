#pragma once
#include <string>
#include "occupancy_grid_map.hpp"

namespace slam {

// Saves an OccupancyGridMap to a plain-text file.
//
// Format:
//   Line 1: x_cells y_cells cell_size_cm
//   Lines 2…(N+1): one occupancy_log_odds value per cell,
//                  in col-major order (index = x + y * x_cells)
//
// The xi, yi, cell_size, and prior fields are fully determined by the
// grid dimensions and are not stored — they are recomputed on load.
//
// Throws std::runtime_error if the file cannot be opened.
void save_map(const OccupancyGridMap& map, const std::string& path);

// Loads an OccupancyGridMap previously saved with save_map().
// Returns a fully initialised map with log-odds values restored.
// Throws std::runtime_error if the file cannot be opened or is malformed.
OccupancyGridMap load_map(const std::string& path);

}  // namespace slam
