#include "mad/perception/occupancy_grid.hpp"

namespace mad::perception {

std::vector<LaneOccupancyCell> OccupancyGridBuilder::Build(const mad::simulation::WorldSnapshot& snapshot,
                                                           const std::vector<FusedObject>& fused_objects,
                                                           const mad::map::LaneMap& lane_map) const {
    std::vector<LaneOccupancyCell> cells;
    cells.reserve(static_cast<std::size_t>(lane_map.lane_count()));
    for (int lane_id = 0; lane_id < lane_map.lane_count(); ++lane_id) {
        LaneOccupancyCell cell;
        cell.lane_id = lane_id;
        cells.push_back(cell);
    }

    for (const auto& object : fused_objects) {
        if (!lane_map.IsLaneValid(object.lane_id)) {
            continue;
        }

        auto& cell = cells[static_cast<std::size_t>(object.lane_id)];
        if (object.x >= snapshot.ego.x) {
            const double gap = object.x - snapshot.ego.x;
            if (gap < cell.nearest_front_gap) {
                cell.nearest_front_gap = gap;
                cell.front_object_id = object.actor_id;
                cell.front_object_speed = object.speed;
            }
        } else {
            const double gap = snapshot.ego.x - object.x;
            if (gap < cell.nearest_rear_gap) {
                cell.nearest_rear_gap = gap;
                cell.rear_object_id = object.actor_id;
            }
        }
    }

    for (auto& cell : cells) {
        cell.front_blocked = cell.nearest_front_gap < 22.0;
        cell.rear_blocked = cell.nearest_rear_gap < 14.0;
    }
    return cells;
}

} // namespace mad::perception
