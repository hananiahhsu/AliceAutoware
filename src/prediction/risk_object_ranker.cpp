#include "mad/prediction/risk_object_ranker.hpp"

#include <algorithm>
#include <cmath>

namespace mad::prediction {
namespace {

double Clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

} // namespace

std::vector<RiskObject> RiskObjectRanker::Rank(const mad::simulation::ActorState& ego,
                                               const std::vector<PredictedObject>& predictions,
                                               const mad::map::LaneMap& lane_map,
                                               int route_goal_lane,
                                               std::size_t max_count) const {
    std::vector<RiskObject> ranked;
    ranked.reserve(predictions.size());

    const int ego_lane = lane_map.ClosestLane(ego.y);
    for (const auto& prediction : predictions) {
        RiskObject item;
        item.actor_id = prediction.actor_id;
        item.lane_id = prediction.lane_id;
        item.predicted_target_lane = prediction.predicted_target_lane;
        item.longitudinal_gap = prediction.current_x - ego.x;
        item.lateral_gap = std::abs(prediction.current_y - ego.y);
        item.relative_speed = ego.speed - prediction.speed;
        item.cut_in_probability = prediction.cut_in_probability;
        item.route_conflict = prediction.predicted_target_lane == route_goal_lane || prediction.lane_id == route_goal_lane;

        if (item.longitudinal_gap > 0.0) {
            item.time_to_collision = item.longitudinal_gap / std::max(0.2, item.relative_speed);
        }

        const bool same_lane_conflict = prediction.lane_id == ego_lane || prediction.predicted_target_lane == ego_lane;
        const bool adjacent_conflict = std::abs(prediction.predicted_target_lane - ego_lane) == 1 && item.longitudinal_gap > -12.0 && item.longitudinal_gap < 30.0;
        const double forward_window_score = item.longitudinal_gap < 0.0 ? 0.15 : Clamp01((35.0 - item.longitudinal_gap) / 35.0);
        const double ttc_score = item.time_to_collision > 1000.0 ? 0.0 : Clamp01((4.0 - item.time_to_collision) / 4.0);
        const double lateral_score = Clamp01((4.0 - item.lateral_gap) / 4.0);
        const double route_score = item.route_conflict ? 0.18 : 0.0;
        const double lane_score = same_lane_conflict ? 0.30 : (adjacent_conflict ? 0.14 : 0.04);
        item.conflict_probability = Clamp01(lane_score + 0.35 * ttc_score + 0.25 * prediction.cut_in_probability + 0.15 * lateral_score);
        item.risk_score = Clamp01(0.28 * forward_window_score + 0.34 * ttc_score + 0.22 * prediction.cut_in_probability + 0.10 * lateral_score + route_score + lane_score);

        if (item.time_to_collision < 1.1 && same_lane_conflict) {
            item.label = "critical_same_lane";
        } else if (prediction.likely_cut_in && adjacent_conflict) {
            item.label = "predicted_cut_in";
        } else if (adjacent_conflict) {
            item.label = "adjacent_interaction";
        } else if (same_lane_conflict) {
            item.label = "same_lane_follow";
        } else {
            item.label = "background_actor";
        }

        ranked.push_back(item);
    }

    std::sort(ranked.begin(), ranked.end(), [](const RiskObject& a, const RiskObject& b) {
        if (a.risk_score != b.risk_score) {
            return a.risk_score > b.risk_score;
        }
        return a.time_to_collision < b.time_to_collision;
    });

    if (ranked.size() > max_count) {
        ranked.resize(max_count);
    }
    return ranked;
}

} // namespace mad::prediction
