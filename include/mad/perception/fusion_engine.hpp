#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/perception/sensor_model.hpp"

#include <string>
#include <vector>

namespace mad::perception {

struct FusedObject {
    int actor_id {0};
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    int lane_id {0};
    double confidence {0.0};
    std::vector<std::string> sources;
};

class FusionEngine {
public:
    std::vector<FusedObject> Fuse(const std::vector<Detection>& detections, const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::perception
