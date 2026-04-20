#include "mad/prediction/intention_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace mad::prediction {

std::vector<IntentionEstimate> IntentionPredictor::Evaluate(const std::vector<mad::perception::TrackedObject>& tracks,
                                                            const mad::simulation::ActorState& ego,
                                                            const mad::map::LaneMap& lane_map) const {
    std::vector<IntentionEstimate> output;
    output.reserve(tracks.size());

    for (const auto& track : tracks) {
        IntentionEstimate estimate;
        estimate.actor_id = track.actor_id;
        estimate.current_lane_id = track.lane_id;
        estimate.target_lane_id = track.lane_id;
        estimate.lateral_velocity = track.lateral_velocity;

        if (std::abs(track.lateral_velocity) > 0.12) {
            const double projected_y = track.y + track.lateral_velocity * 1.5;
            estimate.target_lane_id = lane_map.ClosestLane(projected_y);
        }

        double score = 0.0;
        if (estimate.target_lane_id != track.lane_id) {
            score += 0.45;
        }
        if (std::abs(track.lane_offset) > 0.20) {
            score += 0.20;
        }
        if (track.age_seconds > 0.25) {
            score += 0.10;
        }
        const bool near_ego_window = track.x > ego.x - 8.0 && track.x < ego.x + 35.0;
        if (near_ego_window && estimate.target_lane_id == ego.preferred_lane) {
            score += 0.25;
        }
        if (track.lane_id != ego.preferred_lane && estimate.target_lane_id == ego.preferred_lane) {
            score += 0.15;
        }

        estimate.cut_in_probability = std::clamp(score, 0.0, 0.95);
        estimate.likely_cut_in = estimate.cut_in_probability > 0.55;
        output.push_back(estimate);
    }

    return output;
}

} // namespace mad::prediction
