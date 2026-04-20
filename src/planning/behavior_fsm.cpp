#include "mad/planning/behavior_fsm.hpp"

namespace mad::planning {

void BehaviorStateMachine::Reset() {
    m_state = BehaviorStateId::Cruise;
    m_holdTimer = 0.0;
}

BehaviorStateId BehaviorStateMachine::Step(const BehaviorFsmInput& input, double dt) {
    m_holdTimer += dt;

    if (input.emergency_condition) {
        m_state = BehaviorStateId::EmergencyBrake;
        m_holdTimer = 0.0;
        return m_state;
    }

    switch (m_state) {
    case BehaviorStateId::Cruise:
        if (input.front_slow_vehicle && input.lane_change_desired) {
            m_state = BehaviorStateId::PrepareLaneChange;
            m_holdTimer = 0.0;
        } else if (input.front_slow_vehicle) {
            m_state = BehaviorStateId::Follow;
            m_holdTimer = 0.0;
        }
        break;
    case BehaviorStateId::Follow:
        if (!input.front_slow_vehicle) {
            m_state = BehaviorStateId::Cruise;
            m_holdTimer = 0.0;
        } else if (input.lane_change_desired && m_holdTimer > 0.3) {
            m_state = BehaviorStateId::PrepareLaneChange;
            m_holdTimer = 0.0;
        }
        break;
    case BehaviorStateId::PrepareLaneChange:
        if (input.lane_change_in_progress) {
            m_state = BehaviorStateId::ExecutingLaneChange;
            m_holdTimer = 0.0;
        } else if (!input.lane_change_desired && m_holdTimer > 0.5) {
            m_state = input.front_slow_vehicle ? BehaviorStateId::Follow : BehaviorStateId::Cruise;
            m_holdTimer = 0.0;
        }
        break;
    case BehaviorStateId::ExecutingLaneChange:
        if (!input.lane_change_in_progress && m_holdTimer > 0.8) {
            m_state = BehaviorStateId::Cruise;
            m_holdTimer = 0.0;
        }
        break;
    case BehaviorStateId::EmergencyBrake:
        if (!input.front_slow_vehicle && m_holdTimer > 0.5) {
            m_state = BehaviorStateId::Cruise;
            m_holdTimer = 0.0;
        }
        break;
    }

    return m_state;
}

std::string BehaviorStateMachine::DebugName() const {
    switch (m_state) {
    case BehaviorStateId::Cruise:
        return "cruise";
    case BehaviorStateId::Follow:
        return "follow";
    case BehaviorStateId::PrepareLaneChange:
        return "prepare_lane_change";
    case BehaviorStateId::ExecutingLaneChange:
        return "executing_lane_change";
    case BehaviorStateId::EmergencyBrake:
        return "emergency_brake";
    }
    return "unknown";
}

} // namespace mad::planning
