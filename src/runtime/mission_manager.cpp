#include "mad/runtime/mission_manager.hpp"

namespace mad::runtime {

void MissionManager::Reset() {
    m_state = MissionState::Standby;
}

MissionDecision MissionManager::Step(const MissionInput& input) {
    const MissionState previous = m_state;
    std::string reason = "cruise";

    if (input.collision) {
        m_state = MissionState::EmergencyStop;
        reason = "collision";
    } else if (input.aeb_active) {
        m_state = MissionState::EmergencyStop;
        reason = "aeb_active";
    } else if (input.progress_x >= input.route_length - 5.0) {
        m_state = MissionState::Completed;
        reason = "goal_reached";
    } else if (input.yield_required) {
        m_state = MissionState::Yielding;
        reason = "yield_for_risk";
    } else if (input.ego_speed > 0.5) {
        m_state = MissionState::Driving;
        reason = "tracking_route";
    } else {
        m_state = MissionState::Standby;
        reason = "low_speed";
    }

    return {m_state, previous != m_state, reason};
}

std::string MissionManager::DebugName() const {
    switch (m_state) {
    case MissionState::Standby:
        return "standby";
    case MissionState::Driving:
        return "driving";
    case MissionState::Yielding:
        return "yielding";
    case MissionState::EmergencyStop:
        return "emergency_stop";
    case MissionState::Completed:
        return "completed";
    }
    return "unknown";
}

} // namespace mad::runtime
