#pragma once

#include "mad/planning/behavior_tree.hpp"
#include "mad/planning/task_tree.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/simulation/world.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::simulation {

class CarlaBridgeStub {
public:
    explicit CarlaBridgeStub(const std::string& output_path);
    void PublishFrame(const WorldSnapshot& snapshot,
                      const mad::planning::BehaviorTreeDecision& bt_decision,
                      const mad::planning::TaskTreeDecision& task_decision,
                      const mad::prediction::SceneRiskSummary& risk_summary,
                      const std::vector<mad::prediction::RiskObject>& top_risks);

private:
    std::ofstream m_stream;
};

} // namespace mad::simulation
