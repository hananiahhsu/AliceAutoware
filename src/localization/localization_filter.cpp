#include "mad/localization/localization_filter.hpp"

#include "mad/common/math_utils.hpp"
#include "mad/common/types.hpp"

#include <cmath>

namespace mad::localization {
namespace {
constexpr double kWheelbase = 2.85;
}

LocalizationFilter::LocalizationFilter(mad::map::LaneMap lane_map)
    : m_laneMap(std::move(lane_map)) {
}

void LocalizationFilter::Reset(const mad::simulation::ActorState& ego_truth) {
    m_state = {ego_truth.x, ego_truth.y, ego_truth.yaw, ego_truth.speed, ego_truth.preferred_lane, 0.05};
}

void LocalizationFilter::Predict(double dt, const mad::simulation::ControlCommand& command) {
    const double steering = mad::common::Clamp(command.steering_angle, -0.45, 0.45);
    m_state.speed = std::max(0.0, m_state.speed + command.acceleration * dt);
    m_state.yaw = mad::common::NormalizeAngle(m_state.yaw + std::tan(steering) / kWheelbase * m_state.speed * dt);
    m_state.x += std::cos(m_state.yaw) * m_state.speed * dt;
    m_state.y += std::sin(m_state.yaw) * m_state.speed * dt;
    m_state.lane_id = m_laneMap.ClosestLane(m_state.y);
    m_state.covariance_xy = std::min(2.0, m_state.covariance_xy + 0.01);
}

void LocalizationFilter::CorrectWithTruth(const mad::simulation::ActorState& ego_truth) {
    m_state.x = mad::common::Lerp(m_state.x, ego_truth.x, 0.45);
    m_state.y = mad::common::Lerp(m_state.y, m_laneMap.LaneCenterY(ego_truth.preferred_lane), 0.25);
    m_state.yaw = mad::common::Lerp(m_state.yaw, ego_truth.yaw, 0.35);
    m_state.speed = mad::common::Lerp(m_state.speed, ego_truth.speed, 0.4);
    m_state.lane_id = ego_truth.preferred_lane;
    m_state.covariance_xy = std::max(0.03, m_state.covariance_xy * 0.7);
}

} // namespace mad::localization
