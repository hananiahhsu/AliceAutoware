#include "mad/runtime/decision_trace_writer.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::runtime {

DecisionTraceWriter::DecisionTraceWriter(const std::string& output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream.open(output_path);
    m_stream << "time,planner_state,bt_directive,bt_reason,task_directive,task_reason,current_lane,preferred_lane,current_lane_risk,preferred_lane_risk,global_speed_cap,top_risk_actor,top_risk_label,top_risk_score,final_target_lane,final_target_speed,scene_reason\n";
}

void DecisionTraceWriter::Log(double sim_time,
                              const std::string& planner_state,
                              const mad::planning::BehaviorTreeDecision& bt_decision,
                              const mad::planning::TaskTreeDecision& task_decision,
                              const mad::prediction::SceneRiskSummary& risk_summary,
                              const std::vector<mad::prediction::RiskObject>& top_risks,
                              int final_target_lane,
                              double final_target_speed) {
    const auto* top_risk = top_risks.empty() ? nullptr : &top_risks.front();
    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ','
             << planner_state << ','
             << mad::planning::BehaviorTreePlanner::ToString(bt_decision.directive) << ','
             << bt_decision.reason << ','
             << mad::planning::TaskTreePlanner::ToString(task_decision.directive) << ','
             << task_decision.reason << ','
             << risk_summary.current_lane << ','
             << risk_summary.preferred_lane << ','
             << risk_summary.current_lane_risk << ','
             << risk_summary.preferred_lane_risk << ','
             << risk_summary.global_speed_cap << ','
             << (top_risk != nullptr ? top_risk->actor_id : -1) << ','
             << (top_risk != nullptr ? top_risk->label : std::string("none")) << ','
             << (top_risk != nullptr ? top_risk->risk_score : 0.0) << ','
             << final_target_lane << ','
             << final_target_speed << ','
             << risk_summary.reason << '\n';
}

} // namespace mad::runtime
