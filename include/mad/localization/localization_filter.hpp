#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

namespace mad::localization {

struct EgoLocalizationState {
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    double speed {0.0};
    int lane_id {0};
    double covariance_xy {0.05};
};

class LocalizationFilter {
public:
    explicit LocalizationFilter(mad::map::LaneMap lane_map);

    void Reset(const mad::simulation::ActorState& ego_truth);
    void Predict(double dt, const mad::simulation::ControlCommand& command);
    void CorrectWithTruth(const mad::simulation::ActorState& ego_truth);

    const EgoLocalizationState& state() const { return m_state; }

private:
    mad::map::LaneMap m_laneMap;
    EgoLocalizationState m_state {};
};

} // namespace mad::localization
