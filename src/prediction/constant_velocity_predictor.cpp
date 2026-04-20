#include "mad/prediction/constant_velocity_predictor.hpp"

#include <algorithm>
#include <unordered_map>

namespace mad::prediction {

std::vector<PredictedObject> ConstantVelocityPredictor::Predict(const std::vector<mad::perception::TrackedObject>& tracks,
                                                                const std::vector<IntentionEstimate>& intentions,
                                                                const mad::map::LaneMap& lane_map,
                                                                double horizon_seconds,
                                                                double dt) const {
    std::unordered_map<int, IntentionEstimate> intention_map;
    for (const auto& intention : intentions) {
        intention_map[intention.actor_id] = intention;
    }

    std::vector<PredictedObject> predictions;
    predictions.reserve(tracks.size());

    for (const auto& track : tracks) {
        PredictedObject predicted;
        predicted.actor_id = track.actor_id;
        predicted.lane_id = track.lane_id;
        predicted.predicted_target_lane = track.lane_id;
        predicted.current_x = track.x;
        predicted.current_y = track.y;
        predicted.speed = track.speed;
        predicted.lateral_velocity = track.lateral_velocity;

        auto intention_it = intention_map.find(track.actor_id);
        if (intention_it != intention_map.end()) {
            predicted.predicted_target_lane = intention_it->second.target_lane_id;
            predicted.cut_in_probability = intention_it->second.cut_in_probability;
            predicted.likely_cut_in = intention_it->second.likely_cut_in;
            predicted.lateral_velocity = intention_it->second.lateral_velocity;
        }

        for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
            double future_y = track.y + predicted.lateral_velocity * t;
            if (lane_map.IsLaneValid(predicted.predicted_target_lane) && predicted.predicted_target_lane != track.lane_id) {
                const double target_y = lane_map.LaneCenterY(predicted.predicted_target_lane);
                const double blend = std::clamp(t / 2.2, 0.0, 1.0);
                future_y = (1.0 - blend) * future_y + blend * target_y;
            }
            predicted.future_positions.push_back({track.x + track.speed * t, future_y});
        }
        predictions.push_back(predicted);
    }
    return predictions;
}

} // namespace mad::prediction
