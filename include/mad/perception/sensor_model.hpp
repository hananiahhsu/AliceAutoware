#pragma once

#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace mad::perception {

struct Detection {
    int actor_id {0};
    std::string source;
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    double confidence {0.0};
};

class SensorModel {
public:
    std::vector<Detection> Observe(const mad::simulation::WorldSnapshot& snapshot) const;
};

struct TrackedObject {
    int actor_id {0};
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    int lane_id {0};
    double confidence {0.0};
    double longitudinal_velocity {0.0};
    double lateral_velocity {0.0};
    double age_seconds {0.0};
    double lane_offset {0.0};
};

class TrackManager {
public:
    std::vector<TrackedObject> Update(const std::vector<Detection>& detections,
                                      const mad::map::LaneMap& lane_map,
                                      double sim_time);
    void Reset();

private:
    struct TrackMemory {
        double x {0.0};
        double y {0.0};
        double speed {0.0};
        double longitudinal_velocity {0.0};
        double lateral_velocity {0.0};
        double first_seen_time {0.0};
        double last_seen_time {0.0};
        bool initialized {false};
    };

    std::unordered_map<int, TrackMemory> m_tracks;
};

} // namespace mad::perception
