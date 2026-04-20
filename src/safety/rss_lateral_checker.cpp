#include "mad/safety/rss_lateral_checker.hpp"

#include <algorithm>
#include <cmath>

namespace mad::safety {

RssLateralResult RssLateralChecker::Evaluate(const mad::simulation::ActorState& ego,
                                             const std::vector<mad::simulation::ActorState>& actors,
                                             int target_lane,
                                             const mad::map::LaneMap& lane_map) const {
    RssLateralResult result;
    if (!lane_map.IsLaneValid(target_lane)) {
        result.safe = false;
        result.reason = "invalid_target_lane";
        result.nearest_lateral_clearance = 0.0;
        return result;
    }

    const double target_y = lane_map.LaneCenterY(target_lane);
    for (const auto& actor : actors) {
        if (!actor.active || actor.preferred_lane != target_lane) {
            continue;
        }
        const double lateral_clearance = std::abs(actor.y - target_y) - 0.5 * (ego.width + actor.width);
        const double longitudinal_gap = std::abs(actor.x - ego.x) - 0.5 * (ego.length + actor.length);
        result.nearest_lateral_clearance = std::min(result.nearest_lateral_clearance, lateral_clearance);
        if (longitudinal_gap < 6.0 && lateral_clearance < 0.4) {
            result.safe = false;
            result.reason = "rss_lateral_overlap_violation";
        }
    }
    return result;
}

} // namespace mad::safety
