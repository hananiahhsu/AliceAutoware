#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::safety {

struct RssLateralResult {
    bool safe {true};
    double nearest_lateral_clearance {1.0e9};
    std::string reason {"safe"};
};

class RssLateralChecker {
public:
    RssLateralResult Evaluate(const mad::simulation::ActorState& ego,
                              const std::vector<mad::simulation::ActorState>& actors,
                              int target_lane,
                              const mad::map::LaneMap& lane_map) const;
};

} // namespace mad::safety
