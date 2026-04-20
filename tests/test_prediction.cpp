#include "test_framework.hpp"

#include "mad/map/lane_map.hpp"
#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/prediction/intention_predictor.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"

MAD_TEST(Prediction, IntentionPredictorFlagsCutIn) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::perception::TrackManager manager;
    const std::vector<mad::perception::Detection> detections0 {{10, "lidar", 20.0, lane_map.LaneCenterY(0), 16.0, 0.9}};
    const std::vector<mad::perception::Detection> detections1 {{10, "lidar", 21.5, lane_map.LaneCenterY(0) + 0.65, 16.0, 0.9}};
    (void)manager.Update(detections0, lane_map, 0.0);
    const auto tracks1 = manager.Update(detections1, lane_map, 0.1);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 20.0, 4.8, 1.9, 1, true};
    mad::prediction::IntentionPredictor predictor;
    const auto intentions = predictor.Evaluate(tracks1, ego, lane_map);
    MAD_REQUIRE(!intentions.empty());
    MAD_REQUIRE(intentions.front().cut_in_probability > 0.4);
}

MAD_TEST(Prediction, TrajectoryHypothesisGeneratorBuildsLaneChangeHypothesis) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 22.0, 4.8, 1.9, 1, true};
    std::vector<mad::prediction::PredictedObject> predictions {
        {42, 0, 1, 14.0, lane_map.LaneCenterY(0) + 0.4, 18.0, 0.6, 0.75, true, {}}
    };
    mad::prediction::TrajectoryHypothesisGenerator generator;
    const auto hypotheses = generator.Generate(ego, predictions, lane_map, 3.0, 0.5);
    MAD_REQUIRE(hypotheses.size() >= 2);
    bool saw_lane_change = false;
    for (const auto& hypothesis : hypotheses) {
        if (hypothesis.label == "lane_change") {
            saw_lane_change = true;
            MAD_REQUIRE(hypothesis.merges_into_ego_lane);
            MAD_REQUIRE(hypothesis.probability > 0.5);
        }
    }
    MAD_REQUIRE(saw_lane_change);
}

MAD_TEST(Prediction, SceneRiskAssessorPrefersSaferLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 22.0, 4.8, 1.9, 1, true};
    std::vector<mad::perception::LaneOccupancyCell> occupancy(3);
    for (int lane = 0; lane < 3; ++lane) {
        occupancy[static_cast<std::size_t>(lane)].lane_id = lane;
        occupancy[static_cast<std::size_t>(lane)].nearest_front_gap = (lane == 1) ? 8.0 : 30.0;
        occupancy[static_cast<std::size_t>(lane)].nearest_rear_gap = 20.0;
    }
    std::vector<mad::prediction::PredictedObject> predictions {
        {7, 1, 1, 12.0, lane_map.LaneCenterY(1), 10.0, 0.0, 0.0, false, {}}
    };
    mad::prediction::SceneRiskAssessor assessor;
    const auto summary = assessor.Evaluate(ego, occupancy, predictions, lane_map, 0);
    MAD_REQUIRE(summary.preferred_lane == 0);
    MAD_REQUIRE(summary.current_lane_risk > summary.preferred_lane_risk);
}

MAD_TEST(Prediction, RiskRankerPrioritizesCriticalSameLaneObject) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 24.0, 4.8, 1.9, 1, true};
    std::vector<mad::prediction::PredictedObject> predictions {
        {101, 1, 1, 6.0, lane_map.LaneCenterY(1), 4.0, 0.0, 0.0, false, {}},
        {102, 0, 1, 18.0, lane_map.LaneCenterY(0) + 0.4, 18.0, 0.6, 0.55, true, {}}
    };
    mad::prediction::RiskObjectRanker ranker;
    const auto ranked = ranker.Rank(ego, predictions, lane_map, 0);
    MAD_REQUIRE(!ranked.empty());
    MAD_REQUIRE(ranked.front().actor_id == 101);
    MAD_REQUIRE(ranked.front().label == "critical_same_lane");
}

MAD_TEST(Prediction, LaneOccupancyAnalyzerDetectsDenseTraffic) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::WorldSnapshot snapshot;
    snapshot.ego = {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 20.0, 4.8, 1.9, 1, true};
    std::vector<mad::perception::TrackedObject> tracks {
        {10, 10.0, lane_map.LaneCenterY(1), 10.0, 1, 0.9, 10.0, 0.0, 1.0, 0.0},
        {11, 20.0, lane_map.LaneCenterY(1), 11.0, 1, 0.9, 11.0, 0.0, 1.0, 0.0}
    };
    std::vector<mad::perception::LaneOccupancyCell> occupancy(3);
    for (int lane = 0; lane < 3; ++lane) occupancy[static_cast<std::size_t>(lane)].lane_id = lane;
    occupancy[1].nearest_front_gap = 10.0;
    occupancy[1].nearest_rear_gap = 15.0;
    occupancy[1].front_blocked = true;
    occupancy[1].front_object_speed = 10.0;
    mad::perception::LaneOccupancyAnalyzer analyzer;
    const auto flows = analyzer.Analyze(snapshot, tracks, occupancy, lane_map);
    MAD_REQUIRE(flows[1].dense_traffic);
    MAD_REQUIRE(flows[1].route_blocked);
}
