#pragma once

#include <string>

namespace mad::planning {

enum class BehaviorStateId {
    Cruise,
    Follow,
    PrepareLaneChange,
    ExecutingLaneChange,
    EmergencyBrake,
};

struct BehaviorFsmInput {
    bool emergency_condition {false};
    bool front_slow_vehicle {false};
    bool lane_change_desired {false};
    bool lane_change_in_progress {false};
};

class BehaviorStateMachine {
public:
    void Reset();
    BehaviorStateId Step(const BehaviorFsmInput& input, double dt);
    BehaviorStateId state() const { return m_state; }
    std::string DebugName() const;

private:
    BehaviorStateId m_state {BehaviorStateId::Cruise};
    double m_holdTimer {0.0};
};

} // namespace mad::planning
