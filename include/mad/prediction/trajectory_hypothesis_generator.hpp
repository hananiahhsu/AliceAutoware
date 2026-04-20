#pragma once

#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::prediction {

struct TrajectoryHypothesis {
    int actor_id {0};
    int source_lane {0};
    int target_lane {0};
    double probability {0.0};
    double earliest_conflict_time {1.0e9};
    double terminal_x {0.0};
    double terminal_y {0.0};
    bool merges_into_ego_lane {false};
    std::string label {"keep_lane"};
    std::vector<mad::common::Vec2> future_positions;
};

class TrajectoryHypothesisGenerator {
public:
    std::vector<TrajectoryHypothesis> Generate(const mad::simulation::ActorState& ego,
                                               const std::vector<PredictedObject>& predictions,
                                               const mad::map::LaneMap& lane_map,
                                               double horizon_seconds,
                                               double dt) const;

private:
    TrajectoryHypothesis BuildKeepLaneHypothesis(const mad::simulation::ActorState& ego,
                                                 const PredictedObject& prediction,
                                                 const mad::map::LaneMap& lane_map,
                                                 double probability,
                                                 double horizon_seconds,
                                                 double dt) const;

    TrajectoryHypothesis BuildLaneChangeHypothesis(const mad::simulation::ActorState& ego,
                                                   const PredictedObject& prediction,
                                                   const mad::map::LaneMap& lane_map,
                                                   double probability,
                                                   double horizon_seconds,
                                                   double dt) const;

    void FinalizeConflictEstimate(const mad::simulation::ActorState& ego,
                                  TrajectoryHypothesis& hypothesis,
                                  double ego_speed,
                                  double dt) const;
};

} // namespace mad::prediction
