#pragma once

#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/prediction/intention_predictor.hpp"

#include <vector>

namespace mad::prediction {

struct PredictedObject {
    int actor_id {0};
    int lane_id {0};
    int predicted_target_lane {0};
    double current_x {0.0};
    double current_y {0.0};
    double speed {0.0};
    double lateral_velocity {0.0};
    double cut_in_probability {0.0};
    bool likely_cut_in {false};
    std::vector<mad::common::Vec2> future_positions;
};

class ConstantVelocityPredictor {
public:
    std::vector<PredictedObject> Predict(const std::vector<mad::perception::TrackedObject>& tracks,
                                         const std::vector<IntentionEstimate>& intentions,
                                         const mad::map::LaneMap& lane_map,
                                         double horizon_seconds,
                                         double dt) const;
};

} // namespace mad::prediction
