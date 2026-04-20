#include "mad/planning/lane_sequence_planner.hpp"

#include <algorithm>
#include <cmath>

namespace mad::planning {

LaneSequenceDecision LaneSequencePlanner::Plan(const mad::map::RoutePlan& route_plan,
                                               const mad::prediction::SceneRiskSummary& scene_risk,
                                               const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                                               const std::vector<mad::prediction::RiskObject>& top_risks,
                                               int current_lane,
                                               const mad::map::LaneMap& lane_map) const {
    LaneSequenceDecision decision;
    decision.immediate_target_lane = current_lane;
    decision.lane_sequence = {current_lane};

    auto lane_cost = [&](int lane_id) {
        double cost = 0.0;
        bool saw_profile = false;
        bool saw_flow = false;
        if (lane_id == scene_risk.current_lane) {
            cost += scene_risk.current_lane_risk;
        }
        for (const auto& lane_profile : scene_risk.lane_profiles) {
            if (lane_profile.lane_id == lane_id) {
                cost += lane_profile.risk_score;
                saw_profile = true;
                break;
            }
        }
        for (const auto& flow : lane_flows) {
            if (flow.lane_id == lane_id) {
                cost += 0.35 * flow.congestion_score;
                if (flow.route_blocked) {
                    cost += 0.35;
                }
                saw_flow = true;
                break;
            }
        }
        if (!saw_profile) {
            cost += 0.30;
        }
        if (!saw_flow) {
            cost += 0.20;
        }
        for (const auto& risk : top_risks) {
            if (risk.predicted_target_lane == lane_id || risk.lane_id == lane_id) {
                cost += 0.18 * risk.conflict_probability;
            }
        }
        cost += 0.08 * std::abs(lane_id - route_plan.goal_lane);
        return cost;
    };

    int best_lane = current_lane;
    double best_cost = lane_cost(current_lane);
    for (int lane_id = 0; lane_id < lane_map.lane_count(); ++lane_id) {
        if (!lane_map.IsLaneValid(lane_id)) {
            continue;
        }
        const double cost = lane_cost(lane_id);
        if (cost + 0.08 < best_cost) {
            best_cost = cost;
            best_lane = lane_id;
        }
    }

    decision.immediate_target_lane = best_lane;
    decision.route_aligned = (best_lane == route_plan.goal_lane) || (std::abs(best_lane - route_plan.goal_lane) <= std::abs(current_lane - route_plan.goal_lane));
    decision.confidence = std::clamp(1.25 - best_cost, 0.0, 1.0);
    decision.reason = decision.route_aligned ? "route_aligned_low_cost_sequence" : "defensive_low_cost_sequence";

    if (best_lane == current_lane) {
        decision.lane_sequence = {current_lane};
        return decision;
    }

    const int step = (best_lane > current_lane) ? 1 : -1;
    for (int lane = current_lane; lane != best_lane; lane += step) {
        decision.lane_sequence.push_back(lane + step);
    }
    return decision;
}

} // namespace mad::planning
