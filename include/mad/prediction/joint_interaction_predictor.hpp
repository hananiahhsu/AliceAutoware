#pragma once

#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"

#include <string>
#include <vector>

namespace mad::prediction {

struct InteractionConflict {
    int primary_actor_id {0};
    int secondary_actor_id {0};
    int target_lane {0};
    double time_to_overlap {1.0e9};
    double longitudinal_separation {1.0e9};
    double combined_probability {0.0};
    double severity {0.0};
    std::string label {"nominal"};
};

class JointInteractionPredictor {
public:
    std::vector<InteractionConflict> Analyze(const std::vector<PredictedObject>& predictions,
                                             const std::vector<TrajectoryHypothesis>& hypotheses,
                                             int ego_lane,
                                             std::size_t max_count = 6) const;
};

} // namespace mad::prediction
