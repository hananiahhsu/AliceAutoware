#include "test_framework.hpp"

#include "mad/runtime/fallback_manager.hpp"

MAD_TEST(Fallback, ManagerActivatesOnAccumulatedStress) {
    mad::runtime::FallbackManager manager;
    manager.Reset();

    mad::runtime::FallbackDecision last;
    for (int i = 0; i < 3; ++i) {
        last = manager.Update({1.8, true, false, true, true, 16.0});
    }
    MAD_REQUIRE(last.active);
    MAD_REQUIRE(!last.minimal_risk_stop);
    MAD_REQUIRE(last.override_target_speed <= 5.0);
    MAD_REQUIRE(manager.activation_count() >= 1);
    MAD_REQUIRE(manager.stress_score() < 8.5);
}

MAD_TEST(Fallback, ManagerEscalatesToMinimalRiskStop) {
    mad::runtime::FallbackManager manager;
    manager.Reset();
    const auto decision = manager.Update({0.7, true, true, true, true, 12.0});
    MAD_REQUIRE(decision.active);
    MAD_REQUIRE(decision.minimal_risk_stop);
    MAD_REQUIRE_NEAR(decision.override_target_speed, 0.0, 1.0e-6);
}
