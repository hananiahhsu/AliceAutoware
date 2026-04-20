#include "mad/planning/behavior_planner.hpp"

#include <algorithm>
#include <limits>

namespace mad::planning {
namespace {
constexpr double kSafeFrontGap = 18.0;
constexpr double kEmergencyGap = 7.0;
}

void BehaviorPlanner::Reset() {
    m_fsm.Reset();
    m_laneChangeManager.Reset();
}

BehaviorDecision BehaviorPlanner::Plan(const mad::simulation::WorldSnapshot& snapshot,
                                       const mad::map::RoutePlan& route_plan,
                                       const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                       const std::vector<mad::prediction::PredictedObject>& predictions,
                                       const mad::map::LaneMap& lane_map,
                                       double dt) {
    const mad::map::SemanticMap semantic_map(lane_map);
    const int current_lane = lane_map.ClosestLane(snapshot.ego.y);
    const auto* current_cell = FindLaneCell(occupancy, current_lane);

    BehaviorDecision decision;
    decision.target_lane = current_lane;
    decision.target_speed = semantic_map.RecommendedCruiseSpeed(current_lane);
    decision.front_gap = current_cell ? current_cell->nearest_front_gap : std::numeric_limits<double>::infinity();
    decision.front_speed = current_cell ? current_cell->front_object_speed : decision.target_speed;
    decision.min_ttc = 1.0e9;

    bool cut_in_risk_on_current_lane = false;
    for (const auto& p : predictions) {
        const bool in_front = p.current_x > snapshot.ego.x;
        const double relative_speed = std::max(0.1, snapshot.ego.speed - p.speed);
        if ((p.lane_id == current_lane || p.predicted_target_lane == current_lane) && in_front) {
            decision.min_ttc = std::min(decision.min_ttc, (p.current_x - snapshot.ego.x) / relative_speed);
        }
        if (p.likely_cut_in && p.predicted_target_lane == current_lane && p.current_x < snapshot.ego.x + 30.0) {
            cut_in_risk_on_current_lane = true;
        }
    }

    int preferred_lane = current_lane;
    double preferred_score = -1.0e9;
    double preferred_front_gap = decision.front_gap;
    double preferred_rear_gap = current_cell ? current_cell->nearest_rear_gap : 1.0e9;

    std::vector<int> candidate_lanes = route_plan.lane_sequence;
    for (int n : {current_lane - 1, current_lane + 1}) {
        if (lane_map.IsLaneValid(n) && std::find(candidate_lanes.begin(), candidate_lanes.end(), n) == candidate_lanes.end()) {
            candidate_lanes.push_back(n);
        }
    }

    for (int lane_id : candidate_lanes) {
        const auto* cell = FindLaneCell(occupancy, lane_id);
        if (cell == nullptr) {
            continue;
        }
        if (!semantic_map.AllowsLaneChange(current_lane, lane_id)) {
            continue;
        }

        double cut_in_penalty = 0.0;
        for (const auto& p : predictions) {
            if (p.predicted_target_lane == lane_id && p.likely_cut_in && p.current_x < snapshot.ego.x + 35.0) {
                cut_in_penalty += 6.0 * std::max(0.2, p.cut_in_probability);
            }
        }

        double score = semantic_map.LanePreferenceScore(lane_id, route_plan.goal_lane) + 0.25 * cell->nearest_front_gap;
        if (cell->rear_blocked) {
            score -= 8.0;
        }
        if (cell->front_blocked) {
            score -= 5.0;
        }
        score -= cut_in_penalty;

        if (score > preferred_score) {
            preferred_score = score;
            preferred_lane = lane_id;
            preferred_front_gap = cell->nearest_front_gap;
            preferred_rear_gap = cell->nearest_rear_gap;
        }
    }

    const bool front_slow_vehicle = decision.front_gap < kSafeFrontGap || cut_in_risk_on_current_lane;
    const bool emergency = decision.front_gap < kEmergencyGap || decision.min_ttc < 1.0;
    const bool lane_change_in_progress = std::abs(snapshot.ego.y - lane_map.LaneCenterY(current_lane)) > 0.45;

    const LaneChangeRequest req {
        current_lane,
        preferred_lane,
        preferred_lane != current_lane && preferred_front_gap > decision.front_gap + 4.0 && preferred_rear_gap > 10.0,
        emergency,
        lane_change_in_progress,
        decision.front_gap,
        preferred_front_gap,
        preferred_rear_gap};

    const int committed_lane = m_laneChangeManager.ResolveTargetLane(req, dt);
    const bool lane_change_desired = committed_lane != current_lane;
    const BehaviorFsmInput input {emergency, front_slow_vehicle, lane_change_desired, lane_change_in_progress};
    const BehaviorStateId state = m_fsm.Step(input, dt);
    decision.state = m_fsm.DebugName();

    switch (state) {
    case BehaviorStateId::Cruise:
        decision.target_lane = current_lane;
        decision.target_speed = semantic_map.RecommendedCruiseSpeed(current_lane);
        break;
    case BehaviorStateId::Follow:
        decision.target_lane = current_lane;
        decision.target_speed = std::max(6.0, std::min(snapshot.ego.speed, decision.front_speed + 1.0));
        if (cut_in_risk_on_current_lane) {
            decision.target_speed = std::min(decision.target_speed, snapshot.ego.speed - 1.5);
        }
        break;
    case BehaviorStateId::PrepareLaneChange:
        decision.target_lane = committed_lane;
        decision.target_speed = std::min(semantic_map.RecommendedCruiseSpeed(committed_lane), snapshot.ego.speed + 0.8);
        if (decision.front_gap < 24.0) {
            decision.target_speed = std::min(decision.target_speed, std::max(8.0, decision.front_speed + 0.5));
        }
        break;
    case BehaviorStateId::ExecutingLaneChange:
        decision.target_lane = committed_lane;
        decision.target_speed = std::min(semantic_map.RecommendedCruiseSpeed(committed_lane), snapshot.ego.speed + 1.2);
        if (decision.front_gap < 18.0) {
            decision.target_speed = std::min(decision.target_speed, std::max(7.0, decision.front_speed + 0.3));
        }
        break;
    case BehaviorStateId::EmergencyBrake:
        decision.target_lane = current_lane;
        decision.target_speed = 0.0;
        decision.emergency_brake = true;
        break;
    }

    if (decision.target_lane == current_lane && decision.state == "cruise") {
        const double predicted_front_gap = ClosestPredictedFrontGap(snapshot, predictions, current_lane);
        if (predicted_front_gap < 14.0) {
            decision.state = "follow";
            decision.target_speed = std::max(8.0, decision.front_speed);
        }
    }

    return decision;
}

const mad::perception::LaneOccupancyCell* BehaviorPlanner::FindLaneCell(const std::vector<mad::perception::LaneOccupancyCell>& occupancy,
                                                                        int lane_id) const {
    for (const auto& cell : occupancy) {
        if (cell.lane_id == lane_id) {
            return &cell;
        }
    }
    return nullptr;
}

double BehaviorPlanner::ClosestPredictedFrontGap(const mad::simulation::WorldSnapshot& snapshot,
                                                 const std::vector<mad::prediction::PredictedObject>& predictions,
                                                 int lane_id) const {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& p : predictions) {
        if ((p.lane_id != lane_id && p.predicted_target_lane != lane_id) || p.current_x <= snapshot.ego.x) {
            continue;
        }
        best = std::min(best, p.current_x - snapshot.ego.x);
    }
    return best;
}

} // namespace mad::planning
