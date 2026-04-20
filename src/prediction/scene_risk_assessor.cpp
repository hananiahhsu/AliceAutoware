#include "mad/prediction/scene_risk_assessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mad::prediction {
namespace {

double RiskFromGap(double gap) {
    if (gap < 6.0) return 1.0;
    if (gap < 12.0) return 0.75;
    if (gap < 20.0) return 0.45;
    if (gap < 35.0) return 0.20;
    return 0.05;
}

double RiskFromTtc(double min_ttc) {
    if (min_ttc < 0.9) return 1.0;
    if (min_ttc < 1.5) return 0.8;
    if (min_ttc < 2.5) return 0.45;
    if (min_ttc < 4.0) return 0.20;
    return 0.05;
}

} // namespace

SceneRiskSummary SceneRiskAssessor::Evaluate(const mad::simulation::ActorState& ego,
                                             const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                             const std::vector<PredictedObject>& predictions,
                                             const mad::map::LaneMap& lane_map,
                                             int route_goal_lane) const {
    SceneRiskSummary summary;
    summary.current_lane = lane_map.ClosestLane(ego.y);
    summary.preferred_lane = summary.current_lane;

    double best_lane_score = std::numeric_limits<double>::infinity();
    for (int lane_id = 0; lane_id < lane_map.lane_count(); ++lane_id) {
        const auto* cell = FindLaneCell(occupancy, lane_id);
        LaneRiskProfile profile;
        profile.lane_id = lane_id;
        profile.front_gap = cell ? cell->nearest_front_gap : 1.0e9;
        profile.rear_gap = cell ? cell->nearest_rear_gap : 1.0e9;
        profile.min_ttc = 1.0e9;

        for (const auto& p : predictions) {
            const bool lane_relevant = (p.lane_id == lane_id || p.predicted_target_lane == lane_id);
            if (!lane_relevant || p.current_x <= ego.x) {
                continue;
            }
            const double relative_speed = std::max(0.1, ego.speed - p.speed);
            profile.min_ttc = std::min(profile.min_ttc, (p.current_x - ego.x) / relative_speed);
            if (p.likely_cut_in && p.predicted_target_lane == lane_id && p.current_x < ego.x + 40.0) {
                ++profile.likely_cut_in_count;
            }
        }

        const double gap_risk = RiskFromGap(profile.front_gap);
        const double ttc_risk = RiskFromTtc(profile.min_ttc);
        const double rear_risk = profile.rear_gap < 10.0 ? 0.35 : (profile.rear_gap < 16.0 ? 0.18 : 0.02);
        const double cut_in_risk = 0.22 * static_cast<double>(profile.likely_cut_in_count);
        const double route_penalty = 0.08 * std::abs(route_goal_lane - lane_id);
        profile.risk_score = std::clamp(0.45 * gap_risk + 0.30 * ttc_risk + rear_risk + cut_in_risk + route_penalty, 0.0, 1.5);
        profile.recommended_speed_cap = std::max(4.0, 24.0 - 8.0 * gap_risk - 5.0 * ttc_risk - 2.0 * static_cast<double>(profile.likely_cut_in_count));

        if (lane_id == summary.current_lane) {
            summary.current_lane_risk = profile.risk_score;
            summary.current_lane_cut_in_count = profile.likely_cut_in_count;
        }

        const double lane_choice_score = profile.risk_score + 0.05 * std::abs(summary.current_lane - lane_id);
        if (lane_choice_score < best_lane_score) {
            best_lane_score = lane_choice_score;
            summary.preferred_lane = lane_id;
            summary.preferred_lane_risk = profile.risk_score;
            summary.global_speed_cap = profile.recommended_speed_cap;
        }

        summary.lane_profiles.push_back(profile);
    }

    summary.global_speed_cap = std::min(summary.global_speed_cap, std::max(5.0, 25.0 - 7.0 * summary.current_lane_risk));
    summary.emergency_recommended = summary.current_lane_risk > 0.95;

    if (summary.emergency_recommended) {
        summary.reason = "scene_risk_emergency";
    } else if (summary.preferred_lane != summary.current_lane && summary.preferred_lane_risk + 0.15 < summary.current_lane_risk) {
        summary.reason = "lane_change_reduces_scene_risk";
    } else if (summary.current_lane_cut_in_count > 0) {
        summary.reason = "yield_for_predicted_cut_in";
    }

    return summary;
}

const mad::perception::LaneOccupancyCell* SceneRiskAssessor::FindLaneCell(const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                                                          int lane_id) const {
    for (const auto& cell : occupancy) {
        if (cell.lane_id == lane_id) {
            return &cell;
        }
    }
    return nullptr;
}

} // namespace mad::prediction
