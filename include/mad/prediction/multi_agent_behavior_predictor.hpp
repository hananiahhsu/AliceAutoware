#pragma once

#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/prediction/joint_interaction_predictor.hpp"

#include <string>
#include <vector>

namespace mad::prediction {

struct MultiAgentInteractionSummary {
    int pair_count {0};
    int lane_change_conflict_count {0};
    double global_conflict_score {0.0};
    double nearest_overlap_time {1.0e9};
    std::string label {"nominal"};
};

class MultiAgentBehaviorPredictor {
public:
    MultiAgentInteractionSummary Evaluate(const std::vector<PredictedObject>& predictions,
                                          const std::vector<InteractionConflict>& conflicts,
                                          int ego_lane) const;
};

} // namespace mad::prediction
