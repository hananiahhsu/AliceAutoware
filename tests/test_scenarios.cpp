#include "test_framework.hpp"

#include "mad/map/lane_map.hpp"
#include "mad/runtime/autonomy_stack.hpp"

MAD_TEST(Scenarios, FreeCruiseRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("free_cruise", 10.0, 0.1, "out/tests/free_cruise_log.csv");
    MAD_REQUIRE(!summary.collided);
    MAD_REQUIRE(summary.final_x > 50.0);
}

MAD_TEST(Scenarios, HighwayLaneChangeRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("highway_lane_change", 14.0, 0.1, "out/tests/highway_lane_change_log.csv");
    MAD_REQUIRE(!summary.collided);
    MAD_REQUIRE(summary.lane_changes >= 1);
}

MAD_TEST(Scenarios, CooperativeLaneChangeRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("cooperative_lane_change", 14.0, 0.1, "out/tests/cooperative_lane_change_log.csv");
    MAD_REQUIRE(!summary.collided);
}

MAD_TEST(Scenarios, SuddenLaneBlockageRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("sudden_lane_blockage", 14.0, 0.1, "out/tests/sudden_lane_blockage_log.csv");
    MAD_REQUIRE(!summary.collided);
}


MAD_TEST(Scenarios, MultiInteractionWeaveRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("multi_interaction_weave", 16.0, 0.1, "out/tests/multi_interaction_weave_log.csv");
    MAD_REQUIRE(!summary.collided);
}

MAD_TEST(Scenarios, TruckCutInBrakeRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("truck_cut_in_brake", 16.0, 0.1, "out/tests/truck_cut_in_brake_log.csv");
    MAD_REQUIRE(!summary.collided);
}

MAD_TEST(Scenarios, CorridorBlockedRecoveryRunsWithoutCollision) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("corridor_blocked_recovery", 16.0, 0.1, "out/tests/corridor_blocked_recovery_log.csv");
    MAD_REQUIRE(!summary.collided);
    MAD_REQUIRE(summary.corridor_replans >= 1 || summary.fallback_activations >= 1);
}
