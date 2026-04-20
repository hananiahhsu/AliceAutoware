#include "mad/control/lateral_controller.hpp"

namespace mad::control {

double LateralController::ComputeSteering(const mad::simulation::ActorState& ego,
                                          const mad::common::TrajectoryPoint& target) const {
    return ComputeSteering(ego, target, 0.78, 0.10, 0.45);
}

double LateralController::ComputeSteering(const mad::simulation::ActorState& ego,
                                          const mad::common::TrajectoryPoint& target,
                                          double heading_gain,
                                          double lateral_gain,
                                          double steering_limit) const {
    const double heading_error = mad::common::NormalizeAngle(target.yaw - ego.yaw);
    const double lateral_error = target.y - ego.y;
    return mad::common::Clamp(heading_gain * heading_error + lateral_gain * lateral_error, -steering_limit, steering_limit);
}

} // namespace mad::control
