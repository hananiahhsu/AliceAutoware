#include "mad/control/gain_scheduler.hpp"

#include <algorithm>
#include <cmath>

namespace mad::control {

ControlGains GainScheduler::Schedule(double speed, double curvature_hint) const {
    ControlGains gains;
    const double normalized_speed = std::clamp(speed / 30.0, 0.0, 1.0);
    const double curvature_boost = std::clamp(std::abs(curvature_hint) / 0.18, 0.0, 1.0);

    gains.heading_gain = 0.82 - 0.18 * normalized_speed + 0.06 * curvature_boost;
    gains.lateral_gain = 0.12 - 0.03 * normalized_speed + 0.02 * curvature_boost;
    gains.speed_p = 0.90 - 0.20 * normalized_speed;
    gains.speed_i = 0.09 - 0.03 * normalized_speed;
    gains.steering_limit = 0.45 - 0.10 * normalized_speed + 0.03 * curvature_boost;
    gains.accel_min = -5.5 + 0.5 * normalized_speed;
    gains.accel_max = 3.0 - 0.4 * normalized_speed;
    return gains;
}

} // namespace mad::control
