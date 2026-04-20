#pragma once

#include "mad/perception/occupancy_grid.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::safety {

struct RssLaneChangeDecision {
    bool safe {true};
    double observed_front_gap {1.0e9};
    double observed_rear_gap {1.0e9};
    double required_front_distance {0.0};
    double required_rear_distance {0.0};
    double estimated_rear_speed {0.0};
    std::string reason {"current_lane"};
};

class RssLaneChangeChecker {
public:
    RssLaneChangeDecision Evaluate(const mad::simulation::ActorState& ego,
                                   int current_lane,
                                   int target_lane,
                                   const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                   const std::vector<mad::prediction::RiskObject>& top_risks) const;

    static double ComputeSafeForwardDistance(double ego_speed, double front_speed);
    static double ComputeSafeRearDistance(double rear_speed, double ego_speed);
};

} // namespace mad::safety
