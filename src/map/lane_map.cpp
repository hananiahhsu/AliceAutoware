#include "mad/map/lane_map.hpp"

#include <cmath>

namespace mad::map {

LaneMap::LaneMap(double lane_width, int lane_count, double road_length)
    : m_laneWidth(lane_width), m_laneCount(lane_count), m_roadLength(road_length) {
}

double LaneMap::LaneCenterY(int lane_id) const {
    return (static_cast<double>(lane_id) - static_cast<double>(m_laneCount - 1) / 2.0) * m_laneWidth;
}

int LaneMap::ClosestLane(double y) const {
    int best_lane = 0;
    double best_distance = std::abs(y - LaneCenterY(0));
    for (int lane = 1; lane < m_laneCount; ++lane) {
        const double distance = std::abs(y - LaneCenterY(lane));
        if (distance < best_distance) {
            best_distance = distance;
            best_lane = lane;
        }
    }
    return best_lane;
}

bool LaneMap::IsInRoadBounds(double y) const {
    const double half_width = m_laneWidth * static_cast<double>(m_laneCount) * 0.5;
    return y >= -half_width && y <= half_width;
}

bool LaneMap::IsLaneValid(int lane_id) const {
    return lane_id >= 0 && lane_id < m_laneCount;
}

} // namespace mad::map
