#pragma once

#include "mad/map/lane_map.hpp"
#include <string>
#include <vector>

namespace mad::map {

struct CorridorLaneSegment {
    int lane_id {0};
    double start_x {0.0};
    double end_x {0.0};
    double lane_cost {0.0};
    bool preferred {false};
};

struct LaneCorridor {
    int start_lane {0};
    int goal_lane {0};
    std::vector<CorridorLaneSegment> segments;
    double total_cost {0.0};
    std::string label {"nominal"};
};

class LaneCorridorBuilder {
public:
    LaneCorridor Build(const LaneMap& lane_map,
                       int current_lane,
                       int goal_lane,
                       double ego_x,
                       double horizon_distance,
                       const std::vector<int>& blocked_lanes = {}) const;
};

} // namespace mad::map
