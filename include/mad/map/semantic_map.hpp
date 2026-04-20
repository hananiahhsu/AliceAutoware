#pragma once
#include "mad/map/lane_map.hpp"
#include <vector>
namespace mad::map {
struct LaneSemanticInfo { int lane_id{0}; double speed_limit_mps{22.0}; bool allow_left_change{true}; bool allow_right_change{true}; double route_priority{0.0}; };
class SemanticMap {
public:
    explicit SemanticMap(const LaneMap& lane_map);
    const LaneSemanticInfo& LaneInfo(int lane_id) const;
    double RecommendedCruiseSpeed(int lane_id) const;
    bool AllowsLaneChange(int from_lane, int to_lane) const;
    double LanePreferenceScore(int lane_id, int route_goal_lane) const;
private:
    LaneMap m_laneMap; std::vector<LaneSemanticInfo> m_infos;
};
}
