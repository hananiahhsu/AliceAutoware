#include "mad/planning/trajectory_stitcher.hpp"

namespace mad::planning {

std::vector<mad::common::TrajectoryPoint> TrajectoryStitcher::Stitch(const mad::simulation::ActorState& ego,
                                                                     const std::vector<mad::common::TrajectoryPoint>& previous_trajectory,
                                                                     double dt) const {
    std::vector<mad::common::TrajectoryPoint> stitched;
    stitched.push_back({0.0, ego.x, ego.y, ego.yaw, ego.speed});

    for (const auto& point : previous_trajectory) {
        if (point.t < dt || point.t > 0.8) {
            continue;
        }
        stitched.push_back({point.t - dt, point.x, point.y, point.yaw, point.target_speed});
    }
    return stitched;
}

} // namespace mad::planning
