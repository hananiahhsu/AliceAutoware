#pragma once

#include "mad/perception/occupancy_grid.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::planning {

struct GapAcceptanceDecision {
    bool accepted {true};
    double score {0.0};
    double front_gap {1.0e9};
    double rear_gap {1.0e9};
    double front_time_gap {1.0e9};
    double rear_time_gap {1.0e9};
    double conflict_probability {0.0};
    bool rear_pressure {false};
    std::string reason {"current_lane"};
};

class GapAcceptanceEvaluator {
public:
    GapAcceptanceDecision Evaluate(const mad::simulation::ActorState& ego,
                                   int current_lane,
                                   int target_lane,
                                   const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                   const std::vector<mad::prediction::RiskObject>& top_risks,
                                   const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses) const;
};

} // namespace mad::planning
