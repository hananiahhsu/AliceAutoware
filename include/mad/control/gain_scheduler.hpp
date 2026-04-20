#pragma once

namespace mad::control {

struct ControlGains {
    double heading_gain {0.78};
    double lateral_gain {0.10};
    double speed_p {0.85};
    double speed_i {0.08};
    double steering_limit {0.45};
    double accel_min {-5.0};
    double accel_max {3.0};
};

class GainScheduler {
public:
    ControlGains Schedule(double speed, double curvature_hint) const;
};

} // namespace mad::control
