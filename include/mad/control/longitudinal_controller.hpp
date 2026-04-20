#pragma once

namespace mad::control {

class LongitudinalController {
public:
    double ComputeAcceleration(double current_speed, double target_speed, double dt);
    double ComputeAcceleration(double current_speed, double target_speed, double dt, double p_gain, double i_gain, double accel_min, double accel_max);

private:
    double m_integralSpeedError {0.0};
};

} // namespace mad::control
