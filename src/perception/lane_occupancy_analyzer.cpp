#include "mad/perception/lane_occupancy_analyzer.hpp"

#include <algorithm>

namespace mad::perception {

std::vector<LaneFlowMetrics> LaneOccupancyAnalyzer::Analyze(const mad::simulation::WorldSnapshot& snapshot,
                                                            const std::vector<TrackedObject>& tracks,
                                                            const std::vector<LaneOccupancyCell>& occupancy,
                                                            const mad::map::LaneMap& lane_map) const {
    std::vector<LaneFlowMetrics> metrics;
    metrics.resize(static_cast<std::size_t>(lane_map.lane_count()));

    for (int lane_id = 0; lane_id < lane_map.lane_count(); ++lane_id) {
        auto& lane_metrics = metrics[static_cast<std::size_t>(lane_id)];
        lane_metrics.lane_id = lane_id;
        const auto occupancy_it = std::find_if(occupancy.begin(), occupancy.end(), [lane_id](const LaneOccupancyCell& cell) {
            return cell.lane_id == lane_id;
        });

        double speed_sum = 0.0;
        for (const auto& track : tracks) {
            if (track.lane_id != lane_id) {
                continue;
            }
            ++lane_metrics.tracked_count;
            speed_sum += track.speed;
        }

        lane_metrics.average_speed = lane_metrics.tracked_count > 0 ? speed_sum / static_cast<double>(lane_metrics.tracked_count) : snapshot.ego.speed;
        if (occupancy_it != occupancy.end()) {
            lane_metrics.closing_speed = std::max(0.0, snapshot.ego.speed - occupancy_it->front_object_speed);
            const double front_gap_penalty = occupancy_it->nearest_front_gap < 12.0 ? 0.55 : (occupancy_it->nearest_front_gap < 24.0 ? 0.25 : 0.05);
            const double rear_gap_penalty = occupancy_it->nearest_rear_gap < 10.0 ? 0.28 : (occupancy_it->nearest_rear_gap < 18.0 ? 0.14 : 0.03);
            const double density_penalty = 0.12 * static_cast<double>(lane_metrics.tracked_count);
            lane_metrics.congestion_score = std::clamp(front_gap_penalty + rear_gap_penalty + density_penalty, 0.0, 1.5);
            lane_metrics.dense_traffic = lane_metrics.tracked_count >= 2 || occupancy_it->nearest_front_gap < 15.0;
            lane_metrics.route_blocked = occupancy_it->front_blocked && occupancy_it->nearest_front_gap < 14.0;
        }
    }

    return metrics;
}

} // namespace mad::perception
