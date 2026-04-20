#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/perception/fusion_engine.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::perception {

struct LaneOccupancyCell {
    int lane_id {0};
    double nearest_front_gap {1.0e9};
    double nearest_rear_gap {1.0e9};
    double front_object_speed {0.0};
    int front_object_id {-1};
    int rear_object_id {-1};
    bool front_blocked {false};
    bool rear_blocked {false};
};

class OccupancyGridBuilder {
public:
    std::vector<LaneOccupancyCell> Build(const mad::simulation::WorldSnapshot& snapshot,
                                         const std::vector<FusedObject>& fused_objects,
                                         const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::perception
