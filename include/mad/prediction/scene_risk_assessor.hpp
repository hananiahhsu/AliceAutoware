#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/perception/occupancy_grid.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::prediction {

struct LaneRiskProfile {
    int lane_id {0};
    double front_gap {1.0e9};
    double rear_gap {1.0e9};
    double min_ttc {1.0e9};
    int likely_cut_in_count {0};
    double risk_score {0.0};
    double recommended_speed_cap {25.0};
};

struct SceneRiskSummary {
    std::vector<LaneRiskProfile> lane_profiles;
    int current_lane {0};
    int preferred_lane {0};
    double current_lane_risk {0.0};
    double preferred_lane_risk {0.0};
    int current_lane_cut_in_count {0};
    double global_speed_cap {25.0};
    bool emergency_recommended {false};
    std::string reason {"stable_current_lane"};
};

class SceneRiskAssessor {
public:
    SceneRiskSummary Evaluate(const mad::simulation::ActorState& ego,
                              const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                              const std::vector<PredictedObject>& predictions,
                              const mad::map::LaneMap& lane_map,
                              int route_goal_lane) const;

private:
    const mad::perception::LaneOccupancyCell* FindLaneCell(const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                                           int lane_id) const;
};

} // namespace mad::prediction
