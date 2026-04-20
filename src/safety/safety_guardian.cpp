#include "mad/safety/safety_guardian.hpp"

#include <algorithm>
#include <cmath>

namespace mad::safety {

void SafetyGuardian::Reset() {
    m_aebTriggerCount = 0;
}

SafetyDecision SafetyGuardian::Evaluate(const mad::simulation::WorldSnapshot& snapshot,
                                        const mad::planning::BehaviorDecision& behavior,
                                        const std::vector<mad::prediction::PredictedObject>& predictions,
                                        const std::vector<mad::common::TrajectoryPoint>& trajectory) {
    SafetyDecision result;

    if (!trajectory.empty()) {
        const auto& last = trajectory.back();
        if (std::abs(last.y) > 8.0) {
            result.hold_current_lane = true;
            result.override_target_speed = std::min(behavior.target_speed, snapshot.ego.speed);
            result.reason = "trajectory_leaves_road_bounds";
            return result;
        }
    }

    double closest_gap = 1.0e9;
    double strongest_cut_in_probability = 0.0;
    for (const auto& p : predictions) {
        const bool same_lane_risk = (p.lane_id == snapshot.ego.preferred_lane || p.predicted_target_lane == snapshot.ego.preferred_lane);
        if (!same_lane_risk || p.current_x <= snapshot.ego.x) {
            continue;
        }
        closest_gap = std::min(closest_gap, p.current_x - snapshot.ego.x);
        strongest_cut_in_probability = std::max(strongest_cut_in_probability, p.cut_in_probability);
    }

    if (behavior.min_ttc < 0.9 || behavior.front_gap < 5.5 || closest_gap < 5.0) {
        result.aeb_active = true;
        result.hold_current_lane = true;
        result.override_target_speed = 0.0;
        result.override_acceleration = -5.0;
        result.reason = "hard_brake_ttc_or_gap";
        ++m_aebTriggerCount;
        return result;
    }

    if (strongest_cut_in_probability > 0.70 && closest_gap < 12.0) {
        result.hold_current_lane = true;
        result.override_target_speed = std::max(0.0, snapshot.ego.speed - 3.0);
        result.override_acceleration = -2.5;
        result.reason = "yield_to_cut_in_risk";
        return result;
    }

    result.override_target_speed = behavior.target_speed;
    result.override_acceleration = 0.0;
    return result;
}

} // namespace mad::safety
