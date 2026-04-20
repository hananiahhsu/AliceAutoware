#pragma once

#include "mad/map/lane_map.hpp"

#include <vector>

namespace mad::map {

struct RoutePlan {
    int start_lane {0};
    int goal_lane {0};
    std::vector<int> lane_sequence;
};

class RoutePlanner {
public:
    explicit RoutePlanner(mad::map::LaneMap lane_map);
    RoutePlan PlanRoute(int current_lane, int preferred_goal_lane) const;

private:
    mad::map::LaneMap m_laneMap;
};

} // namespace mad::map
