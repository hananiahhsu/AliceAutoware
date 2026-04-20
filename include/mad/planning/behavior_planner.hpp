#pragma once
#include "mad/map/route_planner.hpp"
#include "mad/map/semantic_map.hpp"
#include "mad/perception/occupancy_grid.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/planning/behavior_fsm.hpp"
#include "mad/planning/lane_change_manager.hpp"
#include "mad/simulation/world.hpp"
#include <string>
#include <vector>
namespace mad::planning {
struct BehaviorDecision { std::string state; int target_lane{1}; double target_speed{0.0}; bool emergency_brake{false}; double front_gap{1.0e9}; double front_speed{0.0}; double min_ttc{1.0e9}; };
class BehaviorPlanner {
public:
    void Reset();
    BehaviorDecision Plan(const mad::simulation::WorldSnapshot& snapshot, const mad::map::RoutePlan& route_plan, const std::vector<mad::perception::LaneOccupancyCell>& occupancy, const std::vector<mad::prediction::PredictedObject>& predictions, const mad::map::LaneMap& lane_map, double dt);
private:
    const mad::perception::LaneOccupancyCell* FindLaneCell(const std::vector<mad::perception::LaneOccupancyCell>& occupancy, int lane_id) const;
    double ClosestPredictedFrontGap(const mad::simulation::WorldSnapshot& snapshot, const std::vector<mad::prediction::PredictedObject>& predictions, int lane_id) const;
    BehaviorStateMachine m_fsm; LaneChangeManager m_laneChangeManager;
};
}
