#include "test_framework.hpp"
#include "mad/map/lane_corridor_builder.hpp"
#include "mad/map/lane_map.hpp"

MAD_TEST(Map, LaneCorridorBuilderBuildsGoalAlignedCorridor) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::LaneCorridorBuilder builder;
    const auto corridor = builder.Build(lane_map, 2, 0, 12.0, 80.0);
    MAD_REQUIRE(corridor.segments.size() >= 2);
    MAD_REQUIRE(corridor.goal_lane == 0);
    MAD_REQUIRE(corridor.segments.back().lane_id == 0);
    MAD_REQUIRE(corridor.total_cost > 0.0);
}

MAD_TEST(Map, LaneCorridorBuilderPenalizesBlockedLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::LaneCorridorBuilder builder;
    const auto nominal = builder.Build(lane_map, 1, 0, 0.0, 70.0);
    const auto blocked = builder.Build(lane_map, 1, 0, 0.0, 70.0, {0});
    MAD_REQUIRE(blocked.total_cost > nominal.total_cost);
}
