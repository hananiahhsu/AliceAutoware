#include "mad/planning/corridor_cost_evaluator.hpp"

#include <limits>

namespace mad::planning {

CorridorEvaluation CorridorCostEvaluator::Evaluate(const mad::map::LaneCorridor& corridor,
                                                   const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                                                   const std::vector<mad::prediction::RiskObject>& top_risks) const {
    CorridorEvaluation result;
    result.corridor = corridor;
    result.recommended_lane = corridor.goal_lane;
    result.cost = corridor.total_cost;

    for (const auto& segment : corridor.segments) {
        for (const auto& flow : lane_flows) {
            if (flow.lane_id != segment.lane_id) {
                continue;
            }
            result.cost += 0.45 * flow.congestion_score;
            if (flow.route_blocked) {
                result.cost += 4.0;
                result.reason = "route_blocked_penalty";
            }
        }
        for (const auto& risk : top_risks) {
            if (risk.lane_id == segment.lane_id || risk.predicted_target_lane == segment.lane_id) {
                result.cost += 0.6 * risk.risk_score * std::max(0.25, risk.conflict_probability);
            }
        }
    }

    if (result.cost < corridor.total_cost + 1.0) {
        result.reason = "low_risk_corridor";
    }
    return result;
}

CorridorEvaluation CorridorCostEvaluator::SelectBest(const std::vector<mad::map::LaneCorridor>& corridors,
                                                     const std::vector<mad::perception::LaneFlowMetrics>& lane_flows,
                                                     const std::vector<mad::prediction::RiskObject>& top_risks) const {
    CorridorEvaluation best;
    best.cost = std::numeric_limits<double>::infinity();

    for (const auto& corridor : corridors) {
        auto current = Evaluate(corridor, lane_flows, top_risks);
        if (current.cost < best.cost) {
            best = current;
        }
    }
    return best;
}

} // namespace mad::planning
