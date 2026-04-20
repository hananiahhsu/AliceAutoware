#pragma once

#include "mad/common/types.hpp"
#include "mad/planning/behavior_planner.hpp"
#include "mad/planning/speed_planner.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::planning {

class TrajectoryPlanner {
public:
    std::vector<mad::common::TrajectoryPoint> Plan(const mad::simulation::WorldSnapshot& snapshot,
                                                   const BehaviorDecision& decision,
                                                   const std::vector<SpeedPoint>& speed_profile,
                                                   const std::vector<mad::common::TrajectoryPoint>& stitched_prefix,
                                                   const mad::map::LaneMap& lane_map,
                                                   double horizon_seconds,
                                                   double dt) const;
};

} // namespace mad::planning
