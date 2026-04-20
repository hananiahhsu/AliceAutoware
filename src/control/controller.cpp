#include "mad/control/controller.hpp"

#include <cmath>
#include <limits>

namespace mad::control {

mad::simulation::ControlCommand Controller::Compute(const mad::simulation::ActorState& ego,
                                                    const std::vector<mad::common::TrajectoryPoint>& trajectory,
                                                    double dt) {
    mad::simulation::ControlCommand command {};
    if (trajectory.empty()) {
        return command;
    }

    const mad::common::TrajectoryPoint* target = &trajectory.back();
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& point : trajectory) {
        const double distance = std::hypot(point.x - ego.x, point.y - ego.y);
        if (distance < best_distance && point.t > 0.15) {
            best_distance = distance;
            target = &point;
        }
    }

    double curvature_hint = 0.0;
    if (trajectory.size() >= 2U) {
        const auto& second = trajectory[std::min<std::size_t>(trajectory.size() - 1U, 1U)];
        curvature_hint = mad::common::NormalizeAngle(second.yaw - ego.yaw);
    }
    const auto gains = m_gainScheduler.Schedule(ego.speed, curvature_hint);
    const auto ff = m_feedforwardController.Compute(trajectory, ego.speed);
    command.steering_angle = m_lateralController.ComputeSteering(ego,
                                                                 *target,
                                                                 gains.heading_gain,
                                                                 gains.lateral_gain,
                                                                 gains.steering_limit) + ff.steering_bias;
    command.steering_angle = std::clamp(command.steering_angle, -gains.steering_limit, gains.steering_limit);
    command.acceleration = m_longitudinalController.ComputeAcceleration(ego.speed,
                                                                        target->target_speed,
                                                                        dt,
                                                                        gains.speed_p,
                                                                        gains.speed_i,
                                                                        gains.accel_min,
                                                                        gains.accel_max) + ff.acceleration_bias;
    command.acceleration = std::clamp(command.acceleration, gains.accel_min, gains.accel_max);
    return command;
}

} // namespace mad::control
