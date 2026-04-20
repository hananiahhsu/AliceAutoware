#pragma once

namespace mad::map {

class LaneMap {
public:
    LaneMap(double lane_width, int lane_count, double road_length);

    double LaneCenterY(int lane_id) const;
    int ClosestLane(double y) const;
    bool IsInRoadBounds(double y) const;
    bool IsLaneValid(int lane_id) const;

    double lane_width() const { return m_laneWidth; }
    int lane_count() const { return m_laneCount; }
    double road_length() const { return m_roadLength; }

private:
    double m_laneWidth {3.7};
    int m_laneCount {3};
    double m_roadLength {500.0};
};

} // namespace mad::map
