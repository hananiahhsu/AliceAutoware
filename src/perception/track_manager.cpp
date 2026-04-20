#include "mad/perception/sensor_model.hpp"

#include <algorithm>
#include <unordered_map>

namespace mad::perception {

std::vector<TrackedObject> TrackManager::Update(const std::vector<Detection>& detections,
                                                const mad::map::LaneMap& lane_map,
                                                double sim_time) {
    struct Accumulator {
        double weighted_x {0.0};
        double weighted_y {0.0};
        double weighted_speed {0.0};
        double weight_sum {0.0};
    };

    std::unordered_map<int, Accumulator> accumulators;
    for (const auto& detection : detections) {
        auto& acc = accumulators[detection.actor_id];
        acc.weighted_x += detection.x * detection.confidence;
        acc.weighted_y += detection.y * detection.confidence;
        acc.weighted_speed += detection.speed * detection.confidence;
        acc.weight_sum += detection.confidence;
    }

    std::vector<TrackedObject> tracks;
    tracks.reserve(accumulators.size());

    for (const auto& [actor_id, acc] : accumulators) {
        if (acc.weight_sum <= 1.0e-6) {
            continue;
        }

        const double x = acc.weighted_x / acc.weight_sum;
        const double y = acc.weighted_y / acc.weight_sum;
        const double speed = acc.weighted_speed / acc.weight_sum;

        auto& memory = m_tracks[actor_id];
        double vx = speed;
        double vy = 0.0;
        if (memory.initialized) {
            const double dt = std::max(1.0e-3, sim_time - memory.last_seen_time);
            const double measured_vx = (x - memory.x) / dt;
            const double measured_vy = (y - memory.y) / dt;
            vx = 0.6 * memory.longitudinal_velocity + 0.4 * measured_vx;
            vy = 0.5 * memory.lateral_velocity + 0.5 * measured_vy;
        }

        memory.x = x;
        memory.y = y;
        memory.speed = speed;
        memory.longitudinal_velocity = vx;
        memory.lateral_velocity = vy;
        if (!memory.initialized) {
            memory.first_seen_time = sim_time;
        }
        memory.last_seen_time = sim_time;
        memory.initialized = true;

        const int lane_id = lane_map.ClosestLane(y);
        const double lane_offset = y - lane_map.LaneCenterY(lane_id);
        tracks.push_back({actor_id,
                          x,
                          y,
                          speed,
                          lane_id,
                          std::min(1.0, acc.weight_sum / 2.0),
                          vx,
                          vy,
                          std::max(0.0, sim_time - memory.first_seen_time),
                          lane_offset});
    }

    std::sort(tracks.begin(), tracks.end(), [](const TrackedObject& lhs, const TrackedObject& rhs) {
        return lhs.x < rhs.x;
    });
    return tracks;
}

void TrackManager::Reset() {
    m_tracks.clear();
}

} // namespace mad::perception
