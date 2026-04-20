#include "mad/control/feedforward_controller.hpp"
#include "mad/common/types.hpp"

#include <algorithm>

namespace mad::control {

FeedforwardCommand FeedforwardController::Compute(const std::vector<mad::common::TrajectoryPoint>& trajectory,
                                                  double current_speed) const {
    FeedforwardCommand command;
    if (trajectory.size() < 3U) {
        return command;
    }

    const auto& p0 = trajectory[0];
    const auto& p1 = trajectory[1];
    const auto& p2 = trajectory[2];
    const double heading_change_1 = mad::common::NormalizeAngle(p1.yaw - p0.yaw);
    const double heading_change_2 = mad::common::NormalizeAngle(p2.yaw - p1.yaw);
    const double curvature_proxy = std::clamp(0.5 * (heading_change_1 + heading_change_2), -0.35, 0.35);
    command.steering_bias = 0.25 * curvature_proxy + 0.002 * current_speed * curvature_proxy;
    const double target_speed = p2.target_speed;
    command.acceleration_bias = std::clamp((target_speed - current_speed) * 0.08, -0.8, 0.6);
    return command;
}

} // namespace mad::control
