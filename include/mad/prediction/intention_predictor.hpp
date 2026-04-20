#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::prediction {

struct IntentionEstimate {
    int actor_id {0};
    int current_lane_id {0};
    int target_lane_id {0};
    double lateral_velocity {0.0};
    double cut_in_probability {0.0};
    bool likely_cut_in {false};
};

class IntentionPredictor {
public:
    std::vector<IntentionEstimate> Evaluate(const std::vector<mad::perception::TrackedObject>& tracks,
                                            const mad::simulation::ActorState& ego,
                                            const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::prediction
