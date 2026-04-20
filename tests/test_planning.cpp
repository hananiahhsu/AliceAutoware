#include "test_framework.hpp"

#include "mad/map/lane_map.hpp"
#include "mad/map/route_planner.hpp"
#include "mad/planning/behavior_tree.hpp"
#include "mad/planning/gap_acceptance_evaluator.hpp"
#include "mad/planning/lane_change_manager.hpp"
#include "mad/planning/speed_planner.hpp"
#include "mad/planning/task_tree.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"

MAD_TEST(Planning, RoutePlannerPreservesGoalLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::RoutePlanner planner(lane_map);
    const auto route = planner.PlanRoute(1, 0);
    MAD_REQUIRE(route.goal_lane == 0);
    MAD_REQUIRE(route.lane_sequence.size() >= 2);
}

MAD_TEST(Planning, BehaviorTreeRequestsEmergencyBrake) {
    mad::planning::BehaviorTreePlanner planner;
    mad::planning::BehaviorTreeInput input;
    input.current_lane = 1;
    input.preferred_lane = 1;
    input.front_gap = 4.0;
    input.min_ttc = 0.7;
    const auto decision = planner.Tick(input);
    MAD_REQUIRE(decision.directive == mad::planning::BehaviorDirective::EmergencyBrake);
}

MAD_TEST(Planning, TaskTreeRequestsOvertakeWhenRouteBlocked) {
    mad::planning::TaskTreePlanner planner;
    mad::planning::TaskTreeInput input;
    input.mission_state = "driving";
    input.current_lane = 1;
    input.route_goal_lane = 0;
    input.scene_risk.current_lane = 1;
    input.scene_risk.preferred_lane = 0;
    input.scene_risk.current_lane_risk = 0.8;
    input.scene_risk.preferred_lane_risk = 0.3;
    input.lane_flows.push_back({1, 2, 11.0, 5.0, 0.8, true, true});
    const auto decision = planner.Tick(input);
    MAD_REQUIRE(decision.directive == mad::planning::TaskDirective::OvertakeSlowerTraffic);
    MAD_REQUIRE(decision.requested_lane == 0);
}

MAD_TEST(Planning, SpeedPlannerDeceleratesForSmallGap) {
    mad::planning::SpeedPlanner planner;
    const auto profile = planner.Plan(22.0, 24.0, 9.0, 11.0, 1.4, 3.0, 0.2);
    MAD_REQUIRE(!profile.empty());
    MAD_REQUIRE(profile.front().target_speed < 22.0);
}

MAD_TEST(Planning, LaneChangeManagerCommitsRequestedLane) {
    mad::planning::LaneChangeManager manager;
    mad::planning::LaneChangeRequest request {1, 0, true, false, false, 12.0, 25.0, 18.0};
    const int target_lane = manager.ResolveTargetLane(request, 0.1);
    MAD_REQUIRE(target_lane == 0);
}

MAD_TEST(Planning, GapAcceptanceRejectsRearPressure) {
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 22.0, 4.8, 1.9, 1, true};
    std::vector<mad::perception::LaneOccupancyCell> occupancy(3);
    for (int lane = 0; lane < 3; ++lane) occupancy[static_cast<std::size_t>(lane)].lane_id = lane;
    occupancy[0].nearest_front_gap = 20.0;
    occupancy[0].nearest_rear_gap = 7.0;
    std::vector<mad::prediction::RiskObject> top_risks {
        {300, 0, 0, -4.0, 0.2, -5.0, 1.0e9, 0.0, 0.7, 0.8, false, "adjacent_interaction"}
    };
    std::vector<mad::prediction::TrajectoryHypothesis> hypotheses;
    mad::planning::GapAcceptanceEvaluator evaluator;
    const auto decision = evaluator.Evaluate(ego, 1, 0, occupancy, top_risks, hypotheses);
    MAD_REQUIRE(!decision.accepted);
}

MAD_TEST(Planning, GapAcceptanceAcceptsCleanTargetLane) {
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 20.0, 4.8, 1.9, 1, true};
    std::vector<mad::perception::LaneOccupancyCell> occupancy(3);
    for (int lane = 0; lane < 3; ++lane) occupancy[static_cast<std::size_t>(lane)].lane_id = lane;
    occupancy[0].nearest_front_gap = 35.0;
    occupancy[0].nearest_rear_gap = 25.0;
    mad::planning::GapAcceptanceEvaluator evaluator;
    const auto decision = evaluator.Evaluate(ego, 1, 0, occupancy, {}, {});
    MAD_REQUIRE(decision.accepted);
}
