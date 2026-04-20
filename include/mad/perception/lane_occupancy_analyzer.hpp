#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/perception/occupancy_grid.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::perception {

struct LaneFlowMetrics {
    int lane_id {0};
    int tracked_count {0};
    double average_speed {0.0};
    double closing_speed {0.0};
    double congestion_score {0.0};
    bool dense_traffic {false};
    bool route_blocked {false};
};

class LaneOccupancyAnalyzer {
public:
    std::vector<LaneFlowMetrics> Analyze(const mad::simulation::WorldSnapshot& snapshot,
                                         const std::vector<TrackedObject>& tracks,
                                         const std::vector<LaneOccupancyCell>& occupancy,
                                         const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::perception
