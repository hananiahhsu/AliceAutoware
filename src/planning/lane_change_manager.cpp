#include "mad/planning/lane_change_manager.hpp"
namespace mad::planning {
namespace { constexpr double kCommitHoldSeconds=2.5; constexpr double kSafeRearGap=10.0; constexpr double kGain=5.0; }
void LaneChangeManager::Reset(){ m_committed=false; m_committedLane=0; m_commitTimer=0.0; }
int LaneChangeManager::ResolveTargetLane(const LaneChangeRequest& request, double dt) {
    m_commitTimer += dt;
    if (request.emergency_condition) { m_committed=false; m_commitTimer=0.0; return request.current_lane; }
    if (m_committed) {
        if (request.current_lane == m_committedLane && m_commitTimer > 0.6) { m_committed=false; m_commitTimer=0.0; return request.current_lane; }
        if (m_commitTimer < kCommitHoldSeconds) return m_committedLane;
        m_committed=false; m_commitTimer=0.0;
    }
    if (request.preferred_lane != request.current_lane && request.preferred_lane_safe && request.preferred_rear_gap > kSafeRearGap && request.preferred_front_gap > request.current_front_gap + kGain) {
        m_committed=true; m_committedLane=request.preferred_lane; m_commitTimer=0.0; return m_committedLane;
    }
    return request.current_lane;
}
}
