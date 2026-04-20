#pragma once

#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"

#include <string>
#include <vector>

namespace mad::planning {

enum class TaskDirective {
    NominalDrive,
    OvertakeSlowerTraffic,
    StabilizeAndYield,
    KeepLaneSafety,
    HardBrakeFallback,
};

struct TaskTreeInput {
    std::string mission_state {"standby"};
    int current_lane {0};
    int route_goal_lane {0};
    bool lane_change_in_progress {false};
    mad::prediction::SceneRiskSummary scene_risk;
    std::vector<mad::prediction::RiskObject> top_risk_objects;
    std::vector<mad::perception::LaneFlowMetrics> lane_flows;
    int interaction_conflict_count {0};
    int lane_sequence_target_lane {-1};
};

struct TaskTreeDecision {
    TaskDirective directive {TaskDirective::NominalDrive};
    std::string reason {"task_nominal_drive"};
    int requested_lane {-1};
    bool freeze_lane_change {false};
    double speed_cap_bias {0.0};
};

class TaskTreePlanner {
public:
    TaskTreeDecision Tick(const TaskTreeInput& input) const;
    static std::string ToString(TaskDirective directive);
};

} // namespace mad::planning
