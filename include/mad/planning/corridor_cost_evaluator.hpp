#pragma once

#include "mad/map/lane_corridor_builder.hpp"
#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/prediction/risk_object_ranker.hpp"

#include <string>
#include <vector>

namespace mad::planning {

struct CorridorEvaluation {
    mad::map::LaneCorridor corridor;
    double cost {0.0};
    int recommended_lane {0};
    std::string reason {"nominal_corridor"};
};

class CorridorCostEvaluator {
public:
    CorridorEvaluation Evaluate(const mad::map::LaneCorridor& corridor,
                                const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                                const std::vector<mad::prediction::RiskObject>& top_risks) const;

    CorridorEvaluation SelectBest(const std::vector<mad::map::LaneCorridor>& corridors,
                                  const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                                  const std::vector<mad::prediction::RiskObject>& top_risks) const;
};

} // namespace mad::planning
