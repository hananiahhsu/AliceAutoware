#include "mad/planning/behavior_tree.hpp"

namespace mad::planning {

BehaviorTreeDecision BehaviorTreePlanner::Tick(const BehaviorTreeInput& input) const {
    if (input.emergency_recommended || input.min_ttc < 0.9 || input.front_gap < 5.5) {
        return {BehaviorDirective::EmergencyBrake, "bt_emergency_guard"};
    }
    if (input.likely_cut_in_count > 0 && input.current_lane_risk > 0.65) {
        return {BehaviorDirective::Yield, "bt_predicted_cut_in_yield"};
    }
    if (!input.lane_change_in_progress && input.preferred_lane != input.current_lane && input.preferred_lane_risk + 0.12 < input.current_lane_risk) {
        if (input.preferred_lane < input.current_lane) {
            return {BehaviorDirective::ChangeLaneLeft, "bt_change_left_reduce_risk"};
        }
        return {BehaviorDirective::ChangeLaneRight, "bt_change_right_reduce_risk"};
    }
    if (input.front_gap < 18.0 || input.min_ttc < 3.0 || input.current_lane_risk > 0.50) {
        return {BehaviorDirective::Follow, "bt_follow_due_to_front_risk"};
    }
    return {BehaviorDirective::Cruise, "bt_cruise_default"};
}

std::string BehaviorTreePlanner::ToString(BehaviorDirective directive) {
    switch (directive) {
    case BehaviorDirective::Cruise: return "cruise";
    case BehaviorDirective::Follow: return "follow";
    case BehaviorDirective::Yield: return "yield";
    case BehaviorDirective::ChangeLaneLeft: return "change_left";
    case BehaviorDirective::ChangeLaneRight: return "change_right";
    case BehaviorDirective::EmergencyBrake: return "emergency_brake";
    }
    return "cruise";
}

} // namespace mad::planning
