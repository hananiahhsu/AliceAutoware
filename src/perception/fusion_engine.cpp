#include "mad/perception/fusion_engine.hpp"

#include <algorithm>
#include <unordered_map>

namespace mad::perception {

std::vector<FusedObject> FusionEngine::Fuse(const std::vector<Detection>& detections, const mad::map::LaneMap& lane_map) const {
    struct Accumulator {
        double weighted_x {0.0};
        double weighted_y {0.0};
        double weighted_speed {0.0};
        double weight_sum {0.0};
        std::vector<std::string> sources;
    };

    std::unordered_map<int, Accumulator> map;
    for (const auto& detection : detections) {
        auto& acc = map[detection.actor_id];
        acc.weighted_x += detection.x * detection.confidence;
        acc.weighted_y += detection.y * detection.confidence;
        acc.weighted_speed += detection.speed * detection.confidence;
        acc.weight_sum += detection.confidence;
        acc.sources.push_back(detection.source);
    }

    std::vector<FusedObject> output;
    output.reserve(map.size());
    for (const auto& [actor_id, acc] : map) {
        if (acc.weight_sum <= 1e-6) {
            continue;
        }
        auto sources = acc.sources;
        std::sort(sources.begin(), sources.end());
        sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
        const double x = acc.weighted_x / acc.weight_sum;
        const double y = acc.weighted_y / acc.weight_sum;
        const double speed = acc.weighted_speed / acc.weight_sum;
        output.push_back({actor_id, x, y, speed, lane_map.ClosestLane(y), std::min(1.0, acc.weight_sum / 2.0), sources});
    }

    std::sort(output.begin(), output.end(), [](const FusedObject& a, const FusedObject& b) {
        return a.x < b.x;
    });
    return output;
}

} // namespace mad::perception
