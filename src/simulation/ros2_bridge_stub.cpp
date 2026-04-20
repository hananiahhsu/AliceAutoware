#include "mad/simulation/ros2_bridge_stub.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::simulation {

Ros2BridgeStub::Ros2BridgeStub(const std::string& output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream.open(output_path);
    m_stream << "time,/localization/ego_x,/localization/ego_y,/localization/ego_speed,/planning/bt_directive,/planning/bt_reason,/runtime/task_directive,/runtime/task_reason,/prediction/current_lane_risk,/prediction/preferred_lane_risk,/prediction/speed_cap,/prediction/top_risk_actor,/prediction/top_risk_score,/control/accel,/control/steer\n";
}

void Ros2BridgeStub::PublishTopics(const WorldSnapshot& snapshot,
                                   const mad::planning::BehaviorTreeDecision& bt_decision,
                                   const mad::planning::TaskTreeDecision& task_decision,
                                   const mad::prediction::SceneRiskSummary& risk_summary,
                                   const std::vector<mad::prediction::RiskObject>& top_risks,
                                   const ControlCommand& command) {
    const auto* top_risk = top_risks.empty() ? nullptr : &top_risks.front();
    m_stream << std::fixed << std::setprecision(3)
             << snapshot.sim_time << ','
             << snapshot.ego.x << ','
             << snapshot.ego.y << ','
             << snapshot.ego.speed << ','
             << mad::planning::BehaviorTreePlanner::ToString(bt_decision.directive) << ','
             << bt_decision.reason << ','
             << mad::planning::TaskTreePlanner::ToString(task_decision.directive) << ','
             << task_decision.reason << ','
             << risk_summary.current_lane_risk << ','
             << risk_summary.preferred_lane_risk << ','
             << risk_summary.global_speed_cap << ','
             << (top_risk != nullptr ? top_risk->actor_id : -1) << ','
             << (top_risk != nullptr ? top_risk->risk_score : 0.0) << ','
             << command.acceleration << ','
             << command.steering_angle << '\n';
}

} // namespace mad::simulation
