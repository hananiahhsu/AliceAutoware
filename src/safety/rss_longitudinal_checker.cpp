#include "mad/safety/rss_longitudinal_checker.hpp"

#include <algorithm>

namespace mad::safety {

RssLongitudinalResult RssLongitudinalChecker::Evaluate(const mad::simulation::ActorState& ego,
                                                       const std::vector<mad::simulation::ActorState>& actors,
                                                       int ego_lane) const {
    constexpr double response_time = 0.6;
    constexpr double ego_brake = 5.0;
    constexpr double actor_brake = 4.0;

    RssLongitudinalResult result;
    for (const auto& actor : actors) {
        if (!actor.active || actor.preferred_lane != ego_lane || actor.x <= ego.x) {
            continue;
        }
        const double actual_distance = actor.x - ego.x - 0.5 * (ego.length + actor.length);
        const double required_distance = ego.speed * response_time
            + 0.5 * ego.speed * ego.speed / ego_brake
            - 0.5 * actor.speed * actor.speed / actor_brake;
        if (actual_distance < result.actual_distance) {
            result.actual_distance = actual_distance;
            result.required_distance = std::max(0.0, required_distance);
        }
        if (actual_distance < required_distance) {
            result.safe = false;
            result.reason = "rss_longitudinal_front_violation";
        }
    }
    return result;
}

} // namespace mad::safety
