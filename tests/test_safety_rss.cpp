#include "test_framework.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/safety/rss_lateral_checker.hpp"
#include "mad/safety/rss_longitudinal_checker.hpp"

MAD_TEST(SafetyRSS, LongitudinalCheckerDetectsUnsafeFrontGap) {
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 22.0, 4.8, 1.9, 1, true};
    std::vector<mad::simulation::ActorState> actors {{2, mad::common::ActorType::Vehicle, 12.0, 0.0, 0.0, 5.0, 4.8, 1.9, 1, true}};
    mad::safety::RssLongitudinalChecker checker;
    const auto result = checker.Evaluate(ego, actors, 1);
    MAD_REQUIRE(!result.safe);
    MAD_REQUIRE(result.reason == "rss_longitudinal_front_violation");
}

MAD_TEST(SafetyRSS, LateralCheckerDetectsOccupiedTargetLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 20.0, lane_map.LaneCenterY(1), 0.0, 18.0, 4.8, 1.9, 1, true};
    std::vector<mad::simulation::ActorState> actors {{3, mad::common::ActorType::Vehicle, 22.0, lane_map.LaneCenterY(0), 0.0, 17.0, 4.8, 1.9, 0, true}};
    mad::safety::RssLateralChecker checker;
    const auto result = checker.Evaluate(ego, actors, 0, lane_map);
    MAD_REQUIRE(!result.safe);
    MAD_REQUIRE(result.reason == "rss_lateral_overlap_violation");
}
