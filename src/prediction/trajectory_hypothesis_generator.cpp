#include "mad/prediction/trajectory_hypothesis_generator.hpp"

#include <algorithm>
#include <cmath>

namespace mad::prediction {
namespace {

constexpr double kMinimumProbability = 0.05;

mad::common::Vec2 Lerp(const mad::common::Vec2& a, const mad::common::Vec2& b, double alpha) {
    return {a.x + (b.x - a.x) * alpha, a.y + (b.y - a.y) * alpha};
}

} // namespace

std::vector<TrajectoryHypothesis> TrajectoryHypothesisGenerator::Generate(const mad::simulation::ActorState& ego,
                                                                          const std::vector<PredictedObject>& predictions,
                                                                          const mad::map::LaneMap& lane_map,
                                                                          double horizon_seconds,
                                                                          double dt) const {
    std::vector<TrajectoryHypothesis> output;
    output.reserve(predictions.size() * 2);

    for (const auto& prediction : predictions) {
        const double lane_change_probability = std::clamp(prediction.cut_in_probability, 0.0, 0.95);
        const double keep_lane_probability = std::max(kMinimumProbability, 1.0 - lane_change_probability);
        output.push_back(BuildKeepLaneHypothesis(ego, prediction, lane_map, keep_lane_probability, horizon_seconds, dt));

        if (prediction.predicted_target_lane != prediction.lane_id && lane_change_probability > 0.10) {
            output.push_back(BuildLaneChangeHypothesis(ego, prediction, lane_map, lane_change_probability, horizon_seconds, dt));
        }
    }

    std::sort(output.begin(), output.end(), [](const TrajectoryHypothesis& lhs, const TrajectoryHypothesis& rhs) {
        if (lhs.actor_id != rhs.actor_id) {
            return lhs.actor_id < rhs.actor_id;
        }
        return lhs.probability > rhs.probability;
    });
    return output;
}

TrajectoryHypothesis TrajectoryHypothesisGenerator::BuildKeepLaneHypothesis(const mad::simulation::ActorState& ego,
                                                                            const PredictedObject& prediction,
                                                                            const mad::map::LaneMap& lane_map,
                                                                            double probability,
                                                                            double horizon_seconds,
                                                                            double dt) const {
    TrajectoryHypothesis hypothesis;
    hypothesis.actor_id = prediction.actor_id;
    hypothesis.source_lane = prediction.lane_id;
    hypothesis.target_lane = prediction.lane_id;
    hypothesis.probability = probability;
    hypothesis.label = "keep_lane";
    hypothesis.merges_into_ego_lane = prediction.lane_id == lane_map.ClosestLane(ego.y);

    for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
        hypothesis.future_positions.push_back({prediction.current_x + prediction.speed * t, prediction.current_y});
    }

    if (!hypothesis.future_positions.empty()) {
        hypothesis.terminal_x = hypothesis.future_positions.back().x;
        hypothesis.terminal_y = hypothesis.future_positions.back().y;
    }
    FinalizeConflictEstimate(ego, hypothesis, prediction.speed, dt);
    return hypothesis;
}

TrajectoryHypothesis TrajectoryHypothesisGenerator::BuildLaneChangeHypothesis(const mad::simulation::ActorState& ego,
                                                                              const PredictedObject& prediction,
                                                                              const mad::map::LaneMap& lane_map,
                                                                              double probability,
                                                                              double horizon_seconds,
                                                                              double dt) const {
    TrajectoryHypothesis hypothesis;
    hypothesis.actor_id = prediction.actor_id;
    hypothesis.source_lane = prediction.lane_id;
    hypothesis.target_lane = prediction.predicted_target_lane;
    hypothesis.probability = std::max(kMinimumProbability, probability);
    hypothesis.label = "lane_change";
    hypothesis.merges_into_ego_lane = prediction.predicted_target_lane == lane_map.ClosestLane(ego.y);

    const mad::common::Vec2 target_terminal {prediction.current_x + prediction.speed * horizon_seconds,
                                             lane_map.LaneCenterY(prediction.predicted_target_lane)};

    for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
        const double alpha = std::clamp(t / std::max(1.4, horizon_seconds * 0.65), 0.0, 1.0);
        const double smooth = alpha * alpha * (3.0 - 2.0 * alpha);
        const mad::common::Vec2 point = Lerp({prediction.current_x + prediction.speed * t, prediction.current_y},
                                             {prediction.current_x + prediction.speed * t, target_terminal.y},
                                             smooth);
        hypothesis.future_positions.push_back(point);
    }

    if (!hypothesis.future_positions.empty()) {
        hypothesis.terminal_x = hypothesis.future_positions.back().x;
        hypothesis.terminal_y = hypothesis.future_positions.back().y;
    }
    FinalizeConflictEstimate(ego, hypothesis, prediction.speed, dt);
    return hypothesis;
}

void TrajectoryHypothesisGenerator::FinalizeConflictEstimate(const mad::simulation::ActorState& ego,
                                                             TrajectoryHypothesis& hypothesis,
                                                             double ego_speed,
                                                             double dt) const {
    hypothesis.earliest_conflict_time = 1.0e9;
    for (std::size_t i = 0; i < hypothesis.future_positions.size(); ++i) {
        const double t = static_cast<double>(i) * dt;
        const double ego_x = ego.x + ego_speed * t;
        const double dx = hypothesis.future_positions[i].x - ego_x;
        const double dy = std::abs(hypothesis.future_positions[i].y - ego.y);
        if (dx > -4.0 && dx < 16.0 && dy < 2.6) {
            hypothesis.earliest_conflict_time = t;
            break;
        }
    }
}

} // namespace mad::prediction
