#pragma once

#include <vector>

namespace mad::planning {

struct SpeedPoint {
    double t {0.0};
    double target_speed {0.0};
    double target_acceleration {0.0};
};

class SpeedPlanner {
public:
    std::vector<SpeedPoint> Plan(double current_speed,
                                 double desired_speed,
                                 double front_gap,
                                 double front_speed,
                                 double min_ttc,
                                 double horizon_seconds,
                                 double dt) const;
};

} // namespace mad::planning
