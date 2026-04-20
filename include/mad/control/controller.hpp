#pragma once

#include "mad/control/feedforward_controller.hpp"
#include "mad/control/gain_scheduler.hpp"
#include "mad/control/lateral_controller.hpp"
#include "mad/control/longitudinal_controller.hpp"
#include "mad/common/types.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::control {

class Controller {
public:
    mad::simulation::ControlCommand Compute(const mad::simulation::ActorState& ego,
                                            const std::vector<mad::common::TrajectoryPoint>& trajectory,
                                            double dt);

private:
    FeedforwardController m_feedforwardController;
    GainScheduler m_gainScheduler;
    LateralController m_lateralController;
    LongitudinalController m_longitudinalController;
};

} // namespace mad::control
