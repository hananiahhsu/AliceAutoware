#include "mad/map/lane_corridor_builder.hpp"

#include <algorithm>
#include <cmath>

namespace mad::map {

LaneCorridor LaneCorridorBuilder::Build(const LaneMap& lane_map,
                                        int current_lane,
                                        int goal_lane,
                                        double ego_x,
                                        double horizon_distance,
                                        const std::vector<int>& blocked_lanes) const {
    LaneCorridor corridor;
    corridor.start_lane = current_lane;
    corridor.goal_lane = goal_lane;
    corridor.label = current_lane == goal_lane ? "keep_lane" : "lane_change_corridor";

    const int step = (goal_lane >= current_lane) ? 1 : -1;
    const double lane_change_penalty = 2.5;
    const double block_penalty = 25.0;
    const double base_span = std::max(30.0, horizon_distance);

    for (int lane = current_lane;; lane += step) {
        CorridorLaneSegment segment;
        segment.lane_id = lane;
        segment.start_x = ego_x;
        segment.end_x = ego_x + base_span + 8.0 * std::abs(lane - current_lane);
        segment.preferred = lane == goal_lane;
        segment.lane_cost = 1.0 + lane_change_penalty * std::abs(lane - current_lane);
        if (std::find(blocked_lanes.begin(), blocked_lanes.end(), lane) != blocked_lanes.end()) {
            segment.lane_cost += block_penalty;
        }
        corridor.total_cost += segment.lane_cost;
        corridor.segments.push_back(segment);
        if (lane == goal_lane) {
            break;
        }
        if (!lane_map.IsLaneValid(lane + step)) {
            break;
        }
    }

    if (corridor.segments.empty()) {
        corridor.segments.push_back({current_lane, ego_x, ego_x + base_span, 99.0, true});
        corridor.total_cost = 99.0;
        corridor.label = "fallback_single_lane";
    }
    return corridor;
}

} // namespace mad::map
