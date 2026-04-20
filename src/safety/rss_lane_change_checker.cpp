#include "mad/safety/rss_lane_change_checker.hpp"

#include <algorithm>

namespace mad::safety {

namespace {
constexpr double kResponseTime = 0.6;
constexpr double kEgoAccel = 1.8;
constexpr double kComfortBrake = 4.5;
constexpr double kRearBrake = 5.5;
}

double RssLaneChangeChecker::ComputeSafeForwardDistance(double ego_speed, double front_speed) {
    const double reacted_speed = ego_speed + kResponseTime * kEgoAccel;
    const double reaction_distance = ego_speed * kResponseTime + 0.5 * kEgoAccel * kResponseTime * kResponseTime;
    const double ego_brake_distance = (reacted_speed * reacted_speed) / (2.0 * kComfortBrake);
    const double front_brake_distance = (front_speed * front_speed) / (2.0 * 6.0);
    return std::max(6.0, reaction_distance + ego_brake_distance - front_brake_distance);
}

double RssLaneChangeChecker::ComputeSafeRearDistance(double rear_speed, double ego_speed) {
    const double reacted_rear_speed = rear_speed + kResponseTime * kEgoAccel;
    const double reaction_distance = rear_speed * kResponseTime + 0.5 * kEgoAccel * kResponseTime * kResponseTime;
    const double rear_brake_distance = (reacted_rear_speed * reacted_rear_speed) / (2.0 * kRearBrake);
    const double ego_brake_distance = (ego_speed * ego_speed) / (2.0 * kComfortBrake);
    return std::max(5.0, reaction_distance + rear_brake_distance - ego_brake_distance);
}

RssLaneChangeDecision RssLaneChangeChecker::Evaluate(const mad::simulation::ActorState& ego,
                                                     int current_lane,
                                                     int target_lane,
                                                     const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                                     const std::vector<mad::prediction::RiskObject>& top_risks) const {
    RssLaneChangeDecision decision;
    if (current_lane == target_lane) {
        decision.reason = "keep_current_lane";
        return decision;
    }

    for (const auto& cell : occupancy) {
        if (cell.lane_id == target_lane) {
            decision.observed_front_gap = cell.nearest_front_gap;
            decision.observed_rear_gap = cell.nearest_rear_gap;
            break;
        }
    }

    double front_speed = ego.speed;
    double rear_speed = ego.speed;
    for (const auto& risk : top_risks) {
        if (risk.predicted_target_lane != target_lane && risk.lane_id != target_lane) {
            continue;
        }
        if (risk.longitudinal_gap >= 0.0) {
            front_speed = std::min(front_speed, ego.speed - risk.relative_speed);
        } else {
            rear_speed = std::max(rear_speed, ego.speed - risk.relative_speed);
        }
    }
    decision.estimated_rear_speed = rear_speed;
    decision.required_front_distance = ComputeSafeForwardDistance(ego.speed, front_speed);
    decision.required_rear_distance = ComputeSafeRearDistance(rear_speed, ego.speed);

    if (decision.observed_front_gap < decision.required_front_distance) {
        decision.safe = false;
        decision.reason = "rss_front_distance_violation";
    } else if (decision.observed_rear_gap < decision.required_rear_distance) {
        decision.safe = false;
        decision.reason = "rss_rear_distance_violation";
    } else {
        decision.safe = true;
        decision.reason = "rss_lane_change_safe";
    }
    return decision;
}

} // namespace mad::safety
