#include "mad/prediction/joint_interaction_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace mad::prediction {

std::vector<InteractionConflict> JointInteractionPredictor::Analyze(const std::vector<PredictedObject>& predictions,
                                                                    const std::vector<TrajectoryHypothesis>& hypotheses,
                                                                    int ego_lane,
                                                                    std::size_t max_count) const {
    std::unordered_map<int, PredictedObject> predicted_by_actor;
    for (const auto& prediction : predictions) {
        predicted_by_actor[prediction.actor_id] = prediction;
    }

    std::vector<InteractionConflict> conflicts;
    for (std::size_t i = 0; i < hypotheses.size(); ++i) {
        for (std::size_t j = i + 1; j < hypotheses.size(); ++j) {
            const auto& a = hypotheses[i];
            const auto& b = hypotheses[j];
            if (a.actor_id == b.actor_id) {
                continue;
            }
            const bool same_target_lane = a.target_lane == b.target_lane;
            const bool ego_lane_conflict = a.merges_into_ego_lane && b.merges_into_ego_lane && a.target_lane == ego_lane;
            if (!same_target_lane && !ego_lane_conflict) {
                continue;
            }

            const auto pa_it = predicted_by_actor.find(a.actor_id);
            const auto pb_it = predicted_by_actor.find(b.actor_id);
            if (pa_it == predicted_by_actor.end() || pb_it == predicted_by_actor.end()) {
                continue;
            }

            const auto& pa = pa_it->second;
            const auto& pb = pb_it->second;
            const double longitudinal_separation = std::abs(pa.current_x - pb.current_x);
            const double overlap_time = std::min(a.earliest_conflict_time, b.earliest_conflict_time);
            const double combined_probability = std::clamp(a.probability * b.probability + 0.5 * (pa.cut_in_probability + pb.cut_in_probability), 0.0, 1.0);
            double severity = combined_probability;
            severity += (longitudinal_separation < 12.0) ? 0.45 : ((longitudinal_separation < 20.0) ? 0.20 : 0.0);
            severity += (overlap_time < 1.8) ? 0.55 : ((overlap_time < 3.0) ? 0.25 : 0.0);
            if (a.target_lane == ego_lane || b.target_lane == ego_lane) {
                severity += 0.15;
            }

            InteractionConflict conflict;
            conflict.primary_actor_id = a.actor_id;
            conflict.secondary_actor_id = b.actor_id;
            conflict.target_lane = same_target_lane ? a.target_lane : ego_lane;
            conflict.time_to_overlap = overlap_time;
            conflict.longitudinal_separation = longitudinal_separation;
            conflict.combined_probability = combined_probability;
            conflict.severity = severity;
            conflict.label = (conflict.target_lane == ego_lane) ? "ego_lane_joint_merge" : "same_target_lane_conflict";
            conflicts.push_back(conflict);
        }
    }

    std::sort(conflicts.begin(), conflicts.end(), [](const InteractionConflict& lhs, const InteractionConflict& rhs) {
        if (lhs.severity == rhs.severity) {
            return lhs.time_to_overlap < rhs.time_to_overlap;
        }
        return lhs.severity > rhs.severity;
    });
    if (conflicts.size() > max_count) {
        conflicts.resize(max_count);
    }
    return conflicts;
}

} // namespace mad::prediction
