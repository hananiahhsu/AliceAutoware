#pragma once

#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

namespace mad::prediction {

class InteractionPredictor {
public:
    double MinTimeToCollisionInLane(const mad::simulation::ActorState& ego,
                                    const std::vector<PredictedObject>& predictions,
                                    int lane_id) const;
};

} // namespace mad::prediction
