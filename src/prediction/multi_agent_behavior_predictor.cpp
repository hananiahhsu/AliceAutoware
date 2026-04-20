#include "mad/prediction/multi_agent_behavior_predictor.hpp"

#include <algorithm>

namespace mad::prediction {

MultiAgentInteractionSummary MultiAgentBehaviorPredictor::Evaluate(const std::vector<PredictedObject>& predictions,
                                                                   const std::vector<InteractionConflict>& conflicts,
                                                                   int ego_lane) const {
    MultiAgentInteractionSummary summary;
    summary.pair_count = static_cast<int>(conflicts.size());

    for (const auto& conflict : conflicts) {
        summary.global_conflict_score += conflict.severity * std::max(0.15, conflict.combined_probability);
        summary.nearest_overlap_time = std::min(summary.nearest_overlap_time, conflict.time_to_overlap);
        if (conflict.target_lane == ego_lane || conflict.label.find("lane") != std::string::npos) {
            ++summary.lane_change_conflict_count;
        }
    }

    for (const auto& item : predictions) {
        if (item.likely_cut_in && item.predicted_target_lane == ego_lane) {
            summary.global_conflict_score += 0.6 + item.cut_in_probability;
            ++summary.lane_change_conflict_count;
        }
    }

    if (summary.global_conflict_score > 5.0 || summary.nearest_overlap_time < 1.2) {
        summary.label = "critical_interaction_cluster";
    } else if (summary.global_conflict_score > 2.0 || summary.lane_change_conflict_count >= 2) {
        summary.label = "elevated_interaction_cluster";
    } else {
        summary.label = "nominal";
    }
    return summary;
}

} // namespace mad::prediction
