#include "mad/control/longitudinal_controller.hpp"

#include "mad/common/types.hpp"

namespace mad::control {

double LongitudinalController::ComputeAcceleration(double current_speed, double target_speed, double dt) {
    return ComputeAcceleration(current_speed, target_speed, dt, 0.85, 0.08, -5.0, 3.0);
}

double LongitudinalController::ComputeAcceleration(double current_speed,
                                                   double target_speed,
                                                   double dt,
                                                   double p_gain,
                                                   double i_gain,
                                                   double accel_min,
                                                   double accel_max) {
    const double speed_error = target_speed - current_speed;
    m_integralSpeedError += speed_error * dt;
    return mad::common::Clamp(p_gain * speed_error + i_gain * m_integralSpeedError, accel_min, accel_max);
}

} // namespace mad::control
