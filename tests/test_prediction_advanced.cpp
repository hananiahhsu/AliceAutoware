#include "test_framework.hpp"
#include "mad/prediction/multi_agent_behavior_predictor.hpp"

MAD_TEST(PredictionAdvanced, MultiAgentBehaviorPredictorFlagsCriticalCluster) {
    std::vector<mad::prediction::PredictedObject> predictions {
        {10, 1, 1, 12.0, 0.0, 12.0, 0.0, 0.0, false, {}},
        {11, 0, 1, 10.0, 0.0, 15.0, 0.7, 0.8, true, {}},
    };
    std::vector<mad::prediction::InteractionConflict> conflicts {
        {10, 11, 1, 0.9, 2.5, 0.8, 3.0, "lane_merge_conflict"},
        {11, 12, 1, 1.1, 3.0, 0.7, 2.4, "lane_merge_conflict"},
    };
    mad::prediction::MultiAgentBehaviorPredictor predictor;
    const auto summary = predictor.Evaluate(predictions, conflicts, 1);
    MAD_REQUIRE(summary.label == "critical_interaction_cluster");
    MAD_REQUIRE(summary.lane_change_conflict_count >= 2);
}
