#pragma once

#include "mad/common/types.hpp"
#include <vector>

namespace mad::control {

struct FeedforwardCommand {
    double steering_bias {0.0};
    double acceleration_bias {0.0};
};

class FeedforwardController {
public:
    FeedforwardCommand Compute(const std::vector<mad::common::TrajectoryPoint>& trajectory,
                               double current_speed) const;
};

} // namespace mad::control
