#include "mad/map/route_planner.hpp"

namespace mad::map {

RoutePlanner::RoutePlanner(mad::map::LaneMap lane_map)
    : m_laneMap(std::move(lane_map)) {
}

RoutePlan RoutePlanner::PlanRoute(int current_lane, int preferred_goal_lane) const {
    RoutePlan plan;
    plan.start_lane = current_lane;
    plan.goal_lane = m_laneMap.IsLaneValid(preferred_goal_lane) ? preferred_goal_lane : current_lane;

    int lane = current_lane;
    plan.lane_sequence.push_back(lane);
    while (lane != plan.goal_lane) {
        lane += (plan.goal_lane > lane) ? 1 : -1;
        plan.lane_sequence.push_back(lane);
    }
    return plan;
}

} // namespace mad::map
