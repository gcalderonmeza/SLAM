#pragma once
#include <memory>
#include <vector>
#include "pose.hpp"
#include "occupancy_grid_map.hpp"

namespace slam {

struct BeliefeOccupancyGrid {
    Pose pose;
    std::shared_ptr<OccupancyGridMap> map;
    std::vector<Pose> path;

    // Default-constructible so BeliefWeightPair can be value-initialized.
    BeliefeOccupancyGrid() : pose(0.0, 0.0, 0.0), map(nullptr) {}

    // parent_path may be nullptr (first particle).
    // The new pose is always appended so path records the trajectory.
    BeliefeOccupancyGrid(const Pose& pose_in,
                         std::shared_ptr<OccupancyGridMap> map_in,
                         const std::vector<Pose>* parent_path)
        : pose(pose_in), map(std::move(map_in))
    {
        if (parent_path) path = *parent_path;
        path.push_back(this->pose);
    }
};

}  // namespace slam
