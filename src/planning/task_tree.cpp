#include "mad/planning/task_tree.hpp"

#include <algorithm>

namespace mad::planning {

TaskTreeDecision TaskTreePlanner::Tick(const TaskTreeInput& input) const {
    const mad::prediction::RiskObject* top_risk = input.top_risk_objects.empty() ? nullptr : &input.top_risk_objects.front();

    if (top_risk != nullptr && (top_risk->label == "critical_same_lane" || input.scene_risk.emergency_recommended)) {
        return {TaskDirective::HardBrakeFallback, "task_hard_brake_due_to_top_risk", input.current_lane, true, -12.0};
    }

    if (top_risk != nullptr && top_risk->label == "predicted_cut_in") {
        return {TaskDirective::StabilizeAndYield, "task_stabilize_for_predicted_cut_in", input.current_lane, true, -4.0};
    }

    const auto lane_it = std::find_if(input.lane_flows.begin(), input.lane_flows.end(), [&](const mad::perception::LaneFlowMetrics& lane_metrics) {
        return lane_metrics.lane_id == input.current_lane;
    });

    if (lane_it != input.lane_flows.end() && lane_it->route_blocked && !input.lane_change_in_progress) {
        if (input.scene_risk.preferred_lane != input.current_lane) {
            return {TaskDirective::OvertakeSlowerTraffic, "task_overtake_route_blocked_lane", input.scene_risk.preferred_lane, false, 1.5};
        }
        return {TaskDirective::KeepLaneSafety, "task_keep_lane_route_blocked_no_safe_exit", input.current_lane, true, -3.0};
    }

    if (input.interaction_conflict_count >= 2 && !input.lane_change_in_progress) {
        return {TaskDirective::KeepLaneSafety, "task_freeze_for_joint_interactions", input.current_lane, true, -3.5};
    }

    if (input.lane_sequence_target_lane >= 0 && input.lane_sequence_target_lane != input.current_lane && !input.lane_change_in_progress) {
        return {TaskDirective::OvertakeSlowerTraffic, "task_follow_lane_sequence_plan", input.lane_sequence_target_lane, false, 1.2};
    }

    if (!input.lane_change_in_progress && input.scene_risk.preferred_lane != input.current_lane && input.scene_risk.preferred_lane_risk + 0.10 < input.scene_risk.current_lane_risk) {
        return {TaskDirective::OvertakeSlowerTraffic, "task_move_to_lower_risk_lane", input.scene_risk.preferred_lane, false, 2.0};
    }

    if (input.mission_state == "yielding") {
        return {TaskDirective::StabilizeAndYield, "task_follow_mission_yield", input.current_lane, true, -2.0};
    }

    if (lane_it != input.lane_flows.end() && lane_it->dense_traffic) {
        return {TaskDirective::KeepLaneSafety, "task_keep_lane_dense_traffic", input.current_lane, true, -1.0};
    }

    return {TaskDirective::NominalDrive, "task_nominal_drive", input.route_goal_lane, false, 0.0};
}

std::string TaskTreePlanner::ToString(TaskDirective directive) {
    switch (directive) {
    case TaskDirective::NominalDrive: return "nominal_drive";
    case TaskDirective::OvertakeSlowerTraffic: return "overtake_slower_traffic";
    case TaskDirective::StabilizeAndYield: return "stabilize_and_yield";
    case TaskDirective::KeepLaneSafety: return "keep_lane_safety";
    case TaskDirective::HardBrakeFallback: return "hard_brake_fallback";
    }
    return "nominal_drive";
}

} // namespace mad::planning
