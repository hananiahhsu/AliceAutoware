#include "test_framework.hpp"

#include "mad/map/lane_map.hpp"
#include "mad/planning/lane_sequence_planner.hpp"
#include "mad/prediction/joint_interaction_predictor.hpp"
#include "mad/safety/rss_lane_change_checker.hpp"

MAD_TEST(Interaction, JointInteractionPredictorDetectsSameTargetLaneConflict) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    std::vector<mad::prediction::PredictedObject> predictions {
        {10, 0, 1, 18.0, lane_map.LaneCenterY(0), 18.0, 0.7, 0.8, true, {}},
        {11, 2, 1, 20.0, lane_map.LaneCenterY(2), 19.0, -0.7, 0.7, true, {}}
    };
    std::vector<mad::prediction::TrajectoryHypothesis> hypotheses {
        {10, 0, 1, 0.75, 1.4, 40.0, lane_map.LaneCenterY(1), true, "lane_change", {}},
        {11, 2, 1, 0.70, 1.6, 42.0, lane_map.LaneCenterY(1), true, "lane_change", {}}
    };
    mad::prediction::JointInteractionPredictor predictor;
    const auto conflicts = predictor.Analyze(predictions, hypotheses, 1);
    MAD_REQUIRE(!conflicts.empty());
    MAD_REQUIRE(conflicts.front().severity > 1.0);
    MAD_REQUIRE(conflicts.front().target_lane == 1);
}

MAD_TEST(Interaction, LaneSequencePlannerPrefersLowerRiskRouteLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::RoutePlan route_plan;
    route_plan.goal_lane = 0;
    route_plan.lane_sequence = {1, 0};
    mad::prediction::SceneRiskSummary risk;
    risk.current_lane = 1;
    risk.current_lane_risk = 0.85;
    risk.preferred_lane = 0;
    risk.preferred_lane_risk = 0.25;
    risk.lane_profiles = {{0, 30.0, 20.0, 3.0, 0, 0.25, 22.0}, {1, 8.0, 16.0, 1.2, 1, 0.85, 12.0}};
    std::vector<mad::perception::LaneFlowMetrics> flows {{0, 2, 20.0, 2.0, 0.20, false, false}, {1, 3, 10.0, 6.0, 0.85, true, true}};
    mad::planning::LaneSequencePlanner planner;
    const auto decision = planner.Plan(route_plan, risk, flows, {}, 1, lane_map);
    MAD_REQUIRE(decision.immediate_target_lane == 0);
    MAD_REQUIRE(decision.route_aligned);
    MAD_REQUIRE(decision.confidence > 0.2);
}

MAD_TEST(Interaction, RssLaneChangeCheckerRejectsUnsafeRearDistance) {
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 20.0, 4.8, 1.9, 1, true};
    std::vector<mad::perception::LaneOccupancyCell> occupancy(3);
    for (int i = 0; i < 3; ++i) occupancy[static_cast<std::size_t>(i)].lane_id = i;
    occupancy[0].nearest_front_gap = 70.0;
    occupancy[0].nearest_rear_gap = 6.0;
    std::vector<mad::prediction::RiskObject> risks {{50, 0, 0, -6.0, 0.0, -5.0, 99.0, 0.0, 0.2, 0.4, false, "rear_pressure"}};
    mad::safety::RssLaneChangeChecker checker;
    const auto result = checker.Evaluate(ego, 1, 0, occupancy, risks);
    MAD_REQUIRE(!result.safe);
    MAD_REQUIRE(result.reason == "rss_rear_distance_violation");
}
