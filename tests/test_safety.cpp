#include "test_framework.hpp"

#include "mad/planning/gap_acceptance_evaluator.hpp"
#include "mad/safety/safety_guardian.hpp"
#include "mad/safety/safety_supervisor.hpp"

MAD_TEST(Safety, SafetyGuardianTriggersAebForShortGap) {
    mad::simulation::WorldSnapshot snapshot;
    snapshot.ego = {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 18.0, 4.8, 1.9, 1, true};
    mad::planning::BehaviorDecision decision;
    decision.front_gap = 4.5;
    decision.min_ttc = 0.8;
    std::vector<mad::prediction::PredictedObject> predictions {
        {101, 1, 1, 4.0, 0.0, 10.0, 0.0, 0.0, false, {}}
    };
    std::vector<mad::common::TrajectoryPoint> trajectory {{0.0, 0.0, 0.0, 0.0, 0.0}};
    mad::safety::SafetyGuardian guardian;
    const auto result = guardian.Evaluate(snapshot, decision, predictions, trajectory);
    MAD_REQUIRE(result.aeb_active);
}

MAD_TEST(Safety, RssDistanceGrowsWithHigherSpeed) {
    const double low_speed = mad::safety::SafetySupervisor::ComputeRssSafeDistance(10.0, 8.0, 0.6, 1.8, 4.5, 6.0);
    const double high_speed = mad::safety::SafetySupervisor::ComputeRssSafeDistance(25.0, 8.0, 0.6, 1.8, 4.5, 6.0);
    MAD_REQUIRE(high_speed > low_speed);
}

MAD_TEST(Safety, SafetySupervisorVetoesLaneChangeOnRejectedGap) {
    mad::simulation::WorldSnapshot snapshot;
    snapshot.ego = {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 22.0, 4.8, 1.9, 1, true};
    mad::planning::BehaviorDecision behavior;
    behavior.target_lane = 0;
    behavior.target_speed = 20.0;
    behavior.front_gap = 16.0;
    behavior.front_speed = 10.0;
    mad::planning::GapAcceptanceDecision gap;
    gap.accepted = false;
    gap.reason = "rear_pressure";
    mad::safety::SafetySupervisor supervisor;
    const auto decision = supervisor.Evaluate(snapshot, behavior, gap, {}, {});
    MAD_REQUIRE(decision.veto_lane_change);
    MAD_REQUIRE(decision.speed_clamp_active);
}

MAD_TEST(Safety, SafetySupervisorEscalatesForCriticalRisk) {
    mad::simulation::WorldSnapshot snapshot;
    snapshot.ego = {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 22.0, 4.8, 1.9, 1, true};
    mad::planning::BehaviorDecision behavior;
    behavior.target_lane = 1;
    behavior.target_speed = 18.0;
    behavior.front_gap = 8.0;
    behavior.front_speed = 7.0;
    mad::planning::GapAcceptanceDecision gap;
    gap.accepted = true;
    std::vector<mad::prediction::RiskObject> risks {
        {42, 1, 1, 8.0, 0.0, 10.0, 0.9, 0.0, 0.9, 0.9, false, "critical_same_lane"}
    };
    mad::safety::SafetySupervisor supervisor;
    const auto decision = supervisor.Evaluate(snapshot, behavior, gap, risks, {});
    MAD_REQUIRE(decision.emergency_brake);
    MAD_REQUIRE(decision.override_target_speed == 0.0);
}
