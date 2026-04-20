#pragma once

#include "mad/planning/behavior_tree.hpp"
#include "mad/planning/task_tree.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"
#include "mad/prediction/risk_object_ranker.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::runtime {

class DecisionTraceWriter {
public:
    explicit DecisionTraceWriter(const std::string& output_path);
    void Log(double sim_time,
             const std::string& planner_state,
             const mad::planning::BehaviorTreeDecision& bt_decision,
             const mad::planning::TaskTreeDecision& task_decision,
             const mad::prediction::SceneRiskSummary& risk_summary,
             const std::vector<mad::prediction::RiskObject>& top_risks,
             int final_target_lane,
             double final_target_speed);

private:
    std::ofstream m_stream;
};

} // namespace mad::runtime
