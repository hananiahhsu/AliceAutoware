#include "mad/perception/sensor_model.hpp"

namespace mad::perception {

std::vector<Detection> SensorModel::Observe(const mad::simulation::WorldSnapshot& snapshot) const {
    std::vector<Detection> detections;
    detections.reserve(snapshot.actors.size() * 2);

    for (const auto& actor : snapshot.actors) {
        if (!actor.active) {
            continue;
        }
        const double dx = actor.x - snapshot.ego.x;
        if (dx < -10.0 || dx > 120.0) {
            continue;
        }

        detections.push_back({actor.id, "lidar", actor.x, actor.y, actor.speed, 0.92});
        detections.push_back({actor.id, "radar", actor.x + 0.2, actor.y, actor.speed, 0.88});
    }
    return detections;
}

} // namespace mad::perception
