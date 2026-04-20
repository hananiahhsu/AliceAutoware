#include "mad/planning/gap_acceptance_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace mad::planning {

GapAcceptanceDecision GapAcceptanceEvaluator::Evaluate(const mad::simulation::ActorState& ego,
                                                       int current_lane,
                                                       int target_lane,
                                                       const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                                       const std::vector<mad::prediction::RiskObject>& top_risks,
                                                       const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses) const {
    GapAcceptanceDecision decision;
    if (target_lane == current_lane) {
        decision.reason = "keep_current_lane";
        decision.accepted = true;
        decision.score = 1.0;
        return decision;
    }

    const auto cell_it = std::find_if(occupancy.begin(), occupancy.end(), [target_lane](const mad::perception::LaneOccupancyCell& cell) {
        return cell.lane_id == target_lane;
    });
    if (cell_it != occupancy.end()) {
        decision.front_gap = cell_it->nearest_front_gap;
        decision.rear_gap = cell_it->nearest_rear_gap;
        decision.front_time_gap = decision.front_gap / std::max(ego.speed, 0.5);
        decision.rear_time_gap = decision.rear_gap / std::max(ego.speed, 0.5);
    }

    double dynamic_penalty = 0.0;
    for (const auto& risk : top_risks) {
        if (risk.predicted_target_lane != target_lane && risk.lane_id != target_lane) {
            continue;
        }
        if (risk.longitudinal_gap < 0.0 && risk.relative_speed < -1.0) {
            decision.rear_pressure = true;
            dynamic_penalty += 0.35;
        }
        dynamic_penalty += 0.45 * risk.conflict_probability;
    }

    for (const auto& hypothesis : hypotheses) {
        if (hypothesis.target_lane != target_lane) {
            continue;
        }
        if (hypothesis.earliest_conflict_time < 1.8) {
            dynamic_penalty += 0.40 * hypothesis.probability;
        } else if (hypothesis.earliest_conflict_time < 3.0) {
            dynamic_penalty += 0.18 * hypothesis.probability;
        }
        if (hypothesis.merges_into_ego_lane && hypothesis.earliest_conflict_time < 2.5) {
            decision.conflict_probability = std::max(decision.conflict_probability, hypothesis.probability);
        }
    }

    const double front_score = std::clamp((decision.front_gap - 10.0) / 20.0, -1.0, 1.0);
    const double rear_score = std::clamp((decision.rear_gap - 9.0) / 16.0, -1.0, 1.0);
    decision.score = front_score + rear_score - dynamic_penalty;

    if (decision.front_gap < 10.0) {
        decision.accepted = false;
        decision.reason = "target_front_gap_too_small";
    } else if (decision.rear_gap < 8.5 || decision.rear_pressure) {
        decision.accepted = false;
        decision.reason = "target_rear_gap_too_small";
    } else if (decision.conflict_probability > 0.55 || decision.score < -0.05) {
        decision.accepted = false;
        decision.reason = "dynamic_conflict_penalty";
    } else {
        decision.accepted = true;
        decision.reason = "gap_accepted";
    }
    return decision;
}

} // namespace mad::planning
