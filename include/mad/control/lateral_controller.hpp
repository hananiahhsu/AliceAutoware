#pragma once

#include "mad/common/types.hpp"
#include "mad/simulation/world.hpp"

namespace mad::control {

class LateralController {
public:
    double ComputeSteering(const mad::simulation::ActorState& ego,
                           const mad::common::TrajectoryPoint& target) const;
    double ComputeSteering(const mad::simulation::ActorState& ego,
                           const mad::common::TrajectoryPoint& target,
                           double heading_gain,
                           double lateral_gain,
                           double steering_limit) const;
};

} // namespace mad::control
