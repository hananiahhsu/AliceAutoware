#include "mad/prediction/interaction_predictor.hpp"

#include <algorithm>

namespace mad::prediction {

double InteractionPredictor::MinTimeToCollisionInLane(const mad::simulation::ActorState& ego,
                                                      const std::vector<PredictedObject>& predictions,
                                                      int lane_id) const {
    double best = 1.0e9;
    for (const auto& prediction : predictions) {
        if (prediction.lane_id != lane_id || prediction.current_x <= ego.x) {
            continue;
        }
        const double relative_speed = ego.speed - prediction.speed;
        const double gap = prediction.current_x - ego.x;
        if (relative_speed > 1e-3) {
            best = std::min(best, gap / relative_speed);
        }
    }
    return best;
}

} // namespace mad::prediction
