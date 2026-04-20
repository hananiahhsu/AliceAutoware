#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::prediction {

struct RiskObject {
    int actor_id {0};
    int lane_id {0};
    int predicted_target_lane {0};
    double longitudinal_gap {1.0e9};
    double lateral_gap {1.0e9};
    double relative_speed {0.0};
    double time_to_collision {1.0e9};
    double cut_in_probability {0.0};
    double conflict_probability {0.0};
    double risk_score {0.0};
    bool route_conflict {false};
    std::string label {"nominal"};
};

class RiskObjectRanker {
public:
    std::vector<RiskObject> Rank(const mad::simulation::ActorState& ego,
                                 const std::vector<PredictedObject>& predictions,
                                 const mad::map::LaneMap& lane_map,
                                 int route_goal_lane,
                                 std::size_t max_count = 5) const;
};

} // namespace mad::prediction
