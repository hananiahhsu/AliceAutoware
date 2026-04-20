#pragma once

#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::safety {

struct RssLongitudinalResult {
    bool safe {true};
    double required_distance {0.0};
    double actual_distance {1.0e9};
    std::string reason {"safe"};
};

class RssLongitudinalChecker {
public:
    RssLongitudinalResult Evaluate(const mad::simulation::ActorState& ego,
                                   const std::vector<mad::simulation::ActorState>& actors,
                                   int ego_lane) const;
};

} // namespace mad::safety
