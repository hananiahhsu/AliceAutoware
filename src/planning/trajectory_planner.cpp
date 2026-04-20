#include "mad/planning/trajectory_planner.hpp"

#include <cmath>

namespace mad::planning {

std::vector<mad::common::TrajectoryPoint> TrajectoryPlanner::Plan(const mad::simulation::WorldSnapshot& snapshot,
                                                                  const BehaviorDecision& decision,
                                                                  const std::vector<SpeedPoint>& speed_profile,
                                                                  const std::vector<mad::common::TrajectoryPoint>& stitched_prefix,
                                                                  const mad::map::LaneMap& lane_map,
                                                                  double horizon_seconds,
                                                                  double dt) const {
    std::vector<mad::common::TrajectoryPoint> trajectory = stitched_prefix;
    const double target_y = lane_map.LaneCenterY(decision.target_lane);
    const double y0 = snapshot.ego.y;
    const double x0 = snapshot.ego.x;
    const double lane_change_horizon = (decision.target_lane != lane_map.ClosestLane(snapshot.ego.y)) ? std::min(1.1, horizon_seconds) : horizon_seconds;

    std::size_t speed_index = 0;
    for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
        if (speed_index >= speed_profile.size()) {
            break;
        }
        const double alpha = lane_change_horizon > 1e-6 ? std::min(1.0, t / lane_change_horizon) : 1.0;
        const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        const double y = y0 + (target_y - y0) * smooth_alpha;
        const double target_speed = decision.emergency_brake ? 0.0 : speed_profile[speed_index].target_speed;
        const double x = trajectory.empty() ? (x0 + target_speed * dt) : (trajectory.back().x + target_speed * dt);
        const double dxdt = std::max(0.1, target_speed);
        const double dydt = (target_y - y0) * (6.0 * alpha * (1.0 - alpha)) / std::max(lane_change_horizon, 1e-6);
        const double yaw = std::atan2(dydt, dxdt);
        trajectory.push_back({t, x, y, yaw, target_speed});
        ++speed_index;
    }
    return trajectory;
}

} // namespace mad::planning
