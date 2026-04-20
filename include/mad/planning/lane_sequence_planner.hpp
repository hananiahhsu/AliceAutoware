#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/map/route_planner.hpp"
#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"

#include <string>
#include <vector>

namespace mad::planning {

struct LaneSequenceDecision {
    std::vector<int> lane_sequence;
    int immediate_target_lane {0};
    double confidence {0.0};
    bool route_aligned {true};
    std::string reason {"stay_current_lane"};
};

class LaneSequencePlanner {
public:
    LaneSequenceDecision Plan(const mad::map::RoutePlan& route_plan,
                              const mad::prediction::SceneRiskSummary& scene_risk,
                              const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                              const std::vector<mad::prediction::RiskObject>& top_risks,
                              int current_lane,
                              const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::planning
