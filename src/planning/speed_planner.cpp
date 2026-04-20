#include "mad/planning/speed_planner.hpp"

#include <algorithm>

namespace mad::planning {

std::vector<SpeedPoint> SpeedPlanner::Plan(double current_speed,
                                           double desired_speed,
                                           double front_gap,
                                           double front_speed,
                                           double min_ttc,
                                           double horizon_seconds,
                                           double dt) const {
    std::vector<SpeedPoint> profile;
    const double capped_desired_speed = std::max(0.0, desired_speed);
    double speed = current_speed;

    for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
        double target_accel = 0.0;

        if (min_ttc < 1.2 || front_gap < 8.0) {
            target_accel = -4.5;
        } else if (front_gap < 18.0 && speed > front_speed) {
            target_accel = -2.0;
        } else if (speed < capped_desired_speed) {
            target_accel = 1.5;
        } else if (speed > capped_desired_speed) {
            target_accel = -1.0;
        }

        speed = std::max(0.0, speed + target_accel * dt);
        if (speed > capped_desired_speed + 0.5 && target_accel >= 0.0) {
            speed = capped_desired_speed + 0.5;
        }
        profile.push_back({t, speed, target_accel});
    }
    return profile;
}

} // namespace mad::planning
