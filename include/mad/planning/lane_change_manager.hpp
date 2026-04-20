#pragma once
namespace mad::planning {
struct LaneChangeRequest { int current_lane{0}; int preferred_lane{0}; bool preferred_lane_safe{false}; bool emergency_condition{false}; bool lane_change_in_progress{false}; double current_front_gap{1.0e9}; double preferred_front_gap{1.0e9}; double preferred_rear_gap{1.0e9}; };
class LaneChangeManager {
public:
    void Reset();
    int ResolveTargetLane(const LaneChangeRequest& request, double dt);
private:
    bool m_committed{false}; int m_committedLane{0}; double m_commitTimer{0.0};
};
}
