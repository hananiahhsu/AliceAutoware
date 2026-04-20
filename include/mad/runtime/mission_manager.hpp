#pragma once

#include <string>

namespace mad::runtime {

struct MissionInput {
    bool collision {false};
    bool aeb_active {false};
    bool yield_required {false};
    double ego_speed {0.0};
    double progress_x {0.0};
    double route_length {0.0};
};

enum class MissionState {
    Standby,
    Driving,
    Yielding,
    EmergencyStop,
    Completed,
};

struct MissionDecision {
    MissionState state {MissionState::Standby};
    bool state_changed {false};
    std::string reason;
};

class MissionManager {
public:
    void Reset();
    MissionDecision Step(const MissionInput& input);
    MissionState state() const { return m_state; }
    std::string DebugName() const;

private:
    MissionState m_state {MissionState::Standby};
};

} // namespace mad::runtime
