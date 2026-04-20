#include "mad/map/semantic_map.hpp"
#include <cmath>
#include <stdexcept>
namespace mad::map {
SemanticMap::SemanticMap(const LaneMap& lane_map): m_laneMap(lane_map) {
    for (int lane_id=0; lane_id<m_laneMap.lane_count(); ++lane_id) {
        LaneSemanticInfo info; info.lane_id=lane_id; info.speed_limit_mps=20.0 + 1.5 * static_cast<double>(m_laneMap.lane_count()-lane_id);
        info.allow_left_change = lane_id > 0; info.allow_right_change = lane_id + 1 < m_laneMap.lane_count(); info.route_priority = static_cast<double>(m_laneMap.lane_count()-lane_id); m_infos.push_back(info);
    }
}
const LaneSemanticInfo& SemanticMap::LaneInfo(int lane_id) const { if(!m_laneMap.IsLaneValid(lane_id)) throw std::out_of_range("invalid lane"); return m_infos[static_cast<std::size_t>(lane_id)]; }
double SemanticMap::RecommendedCruiseSpeed(int lane_id) const { return LaneInfo(lane_id).speed_limit_mps; }
bool SemanticMap::AllowsLaneChange(int from_lane, int to_lane) const {
    if (!m_laneMap.IsLaneValid(from_lane) || !m_laneMap.IsLaneValid(to_lane)) {
        return false;
    }
    if (from_lane == to_lane) {
        return true;
    }
    if (to_lane == from_lane - 1) {
        return LaneInfo(from_lane).allow_left_change;
    }
    if (to_lane == from_lane + 1) {
        return LaneInfo(from_lane).allow_right_change;
    }
    return false;
}
double SemanticMap::LanePreferenceScore(int lane_id, int route_goal_lane) const { const auto& info=LaneInfo(lane_id); return info.route_priority + (8.0 - 2.0 * std::abs(route_goal_lane - lane_id)) + info.speed_limit_mps * 0.1; }
}
