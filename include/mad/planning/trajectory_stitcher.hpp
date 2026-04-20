#pragma once

#include "mad/common/types.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::planning {

class TrajectoryStitcher {
public:
    std::vector<mad::common::TrajectoryPoint> Stitch(const mad::simulation::ActorState& ego,
                                                     const std::vector<mad::common::TrajectoryPoint>& previous_trajectory,
                                                     double dt) const;
};

} // namespace mad::planning
