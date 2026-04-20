#include "test_framework.hpp"

#include "mad/map/lane_corridor_builder.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/planning/corridor_cost_evaluator.hpp"
#include "mad/prediction/risk_object_ranker.hpp"

MAD_TEST(Corridor, EvaluatorPrefersLowRiskLowCongestionLane) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::LaneCorridorBuilder builder;
    mad::planning::CorridorCostEvaluator evaluator;

    auto lane0 = builder.Build(lane_map, 1, 0, 0.0, 70.0);
    auto lane2 = builder.Build(lane_map, 1, 2, 0.0, 70.0);

    std::vector<mad::perception::LaneFlowMetrics> flows;
    flows.push_back({0, 1, 22.0, 0.0, 0.2, false, false});
    flows.push_back({2, 4, 9.0, 6.0, 1.8, true, true});

    mad::prediction::RiskObject low_risk;
    low_risk.actor_id = 10;
    low_risk.lane_id = 0;
    low_risk.predicted_target_lane = 0;
    low_risk.risk_score = 0.3;
    low_risk.conflict_probability = 0.2;

    mad::prediction::RiskObject high_risk;
    high_risk.actor_id = 11;
    high_risk.lane_id = 2;
    high_risk.predicted_target_lane = 2;
    high_risk.risk_score = 1.4;
    high_risk.conflict_probability = 0.9;

    const auto best = evaluator.SelectBest({lane0, lane2}, flows, {low_risk, high_risk});
    MAD_REQUIRE(best.recommended_lane == 0);
    MAD_REQUIRE(best.cost > 0.0);
}

MAD_TEST(Corridor, EvaluatorPenalizesBlockedCorridor) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::map::LaneCorridorBuilder builder;
    mad::planning::CorridorCostEvaluator evaluator;

    auto nominal = builder.Build(lane_map, 1, 0, 0.0, 60.0);
    auto blocked = builder.Build(lane_map, 1, 0, 0.0, 60.0, {0});

    std::vector<mad::perception::LaneFlowMetrics> flows;
    flows.push_back({0, 3, 10.0, 8.0, 1.0, true, true});

    const auto nominal_eval = evaluator.Evaluate(nominal, {}, {});
    const auto blocked_eval = evaluator.Evaluate(blocked, flows, {});
    MAD_REQUIRE(blocked_eval.cost > nominal_eval.cost);
}
