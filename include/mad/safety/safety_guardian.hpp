#pragma once
#include "mad/planning/behavior_planner.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"
#include <string>
#include <vector>
namespace mad::safety {
struct SafetyDecision { bool aeb_active{false}; bool hold_current_lane{false}; double override_target_speed{0.0}; double override_acceleration{0.0}; std::string reason; };
class SafetyGuardian {
public:
    void Reset();
    SafetyDecision Evaluate(const mad::simulation::WorldSnapshot& snapshot, const mad::planning::BehaviorDecision& behavior, const std::vector<mad::prediction::PredictedObject>& predictions, const std::vector<mad::common::TrajectoryPoint>& trajectory);
    int aeb_trigger_count() const { return m_aebTriggerCount; }
private:
    int m_aebTriggerCount{0};
};
}
