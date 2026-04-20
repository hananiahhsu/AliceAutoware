#include "mad/safety/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace mad::safety {

void SafetySupervisor::Reset() {
    m_vetoCount = 0;
    m_interventionCount = 0;
}

double SafetySupervisor::ComputeRssSafeDistance(double ego_speed,
                                                double front_speed,
                                                double response_time,
                                                double ego_max_accel,
                                                double ego_comfort_brake,
                                                double front_brake) {
    const double reacted_speed = ego_speed + response_time * ego_max_accel;
    const double ego_reaction_distance = ego_speed * response_time + 0.5 * ego_max_accel * response_time * response_time;
    const double ego_brake_distance = (reacted_speed * reacted_speed) / (2.0 * std::max(ego_comfort_brake, 0.1));
    const double front_brake_distance = (front_speed * front_speed) / (2.0 * std::max(front_brake, 0.1));
    return std::max(4.0, ego_reaction_distance + ego_brake_distance - front_brake_distance);
}

SafetySupervisorDecision SafetySupervisor::Evaluate(const mad::simulation::WorldSnapshot& snapshot,
                                                    const mad::planning::BehaviorDecision& behavior,
                                                    const mad::planning::GapAcceptanceDecision& gap_acceptance,
                                                    const std::vector<mad::prediction::RiskObject>& top_risks,
                                                    const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses) {
    SafetySupervisorDecision decision;
    decision.override_target_speed = behavior.target_speed;

    const double rss_safe_distance = ComputeRssSafeDistance(snapshot.ego.speed,
                                                            behavior.front_speed,
                                                            0.6,
                                                            1.8,
                                                            4.5,
                                                            6.0);
    decision.rss_safe_distance = rss_safe_distance;

    if (behavior.target_lane != snapshot.ego.preferred_lane && !gap_acceptance.accepted) {
        decision.veto_lane_change = true;
        decision.speed_clamp_active = true;
        decision.override_target_speed = std::min(decision.override_target_speed, std::max(6.0, snapshot.ego.speed - 1.2));
        decision.intervention_level = 1;
        decision.reason = gap_acceptance.reason;
        ++m_vetoCount;
        ++m_interventionCount;
    }

    if (behavior.front_gap < rss_safe_distance) {
        decision.speed_clamp_active = true;
        decision.override_target_speed = std::min(decision.override_target_speed, std::max(0.0, behavior.front_speed + 0.5));
        decision.override_acceleration = -2.0;
        decision.intervention_level = std::max(decision.intervention_level, 2);
        decision.reason = "rss_speed_clamp";
    }

    for (const auto& risk : top_risks) {
        if (risk.label == "critical_same_lane" && risk.time_to_collision < 1.2) {
            decision.emergency_brake = true;
            decision.speed_clamp_active = true;
            decision.override_target_speed = 0.0;
            decision.override_acceleration = -4.8;
            decision.intervention_level = 3;
            decision.reason = "critical_same_lane_risk";
            ++m_interventionCount;
            return decision;
        }
    }

    for (const auto& hypothesis : hypotheses) {
        if (!hypothesis.merges_into_ego_lane) {
            continue;
        }
        if (hypothesis.earliest_conflict_time < 1.4 && hypothesis.probability > 0.45) {
            decision.speed_clamp_active = true;
            decision.override_target_speed = std::min(decision.override_target_speed, std::max(4.0, snapshot.ego.speed - 3.0));
            decision.override_acceleration = std::min(decision.override_acceleration, -2.8);
            decision.intervention_level = std::max(decision.intervention_level, 2);
            decision.reason = "merge_conflict_supervision";
        }
    }

    if (decision.intervention_level > 0) {
        ++m_interventionCount;
    }
    return decision;
}

} // namespace mad::safety
