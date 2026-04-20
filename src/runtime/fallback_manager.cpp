#include "mad/runtime/fallback_manager.hpp"

#include <algorithm>

namespace mad::runtime {

void FallbackManager::Reset() {
    m_stressScore = 0.0;
    m_active = false;
    m_activationCount = 0;
}

FallbackDecision FallbackManager::Update(const FallbackFrameInput& input) {
    FallbackDecision decision;

    m_stressScore = std::max(0.0, m_stressScore - 0.50);
    if (input.supervisor_intervened) {
        m_stressScore += 1.2;
    }
    if (input.lane_change_rejected) {
        m_stressScore += 0.4;
    }
    if (input.route_blocked) {
        m_stressScore += 1.2;
    }
    if (input.min_ttc < 1.5) {
        m_stressScore += 1.2;
    }
    if (input.min_ttc < 1.2) {
        m_stressScore += 2.0;
    }
    if (input.guardian_aeb_active) {
        m_stressScore += 3.0;
    }

    const bool should_activate = input.guardian_aeb_active || (m_stressScore >= 5.5 && (input.route_blocked || input.min_ttc < 1.4));
    if (should_activate && !m_active) {
        ++m_activationCount;
    }
    m_active = should_activate;

    if (!m_active) {
        return decision;
    }

    decision.active = true;
    decision.override_target_speed = std::max(0.0, std::min(input.current_speed, 5.0));
    decision.escalation_level = 1;
    decision.reason = "stability_fallback";

    if (input.guardian_aeb_active || (m_stressScore >= 8.5 && input.route_blocked) || input.min_ttc < 0.8) {
        decision.minimal_risk_stop = true;
        decision.override_target_speed = 0.0;
        decision.escalation_level = 2;
        decision.reason = input.guardian_aeb_active ? "guardian_triggered_mrm" : "persistent_high_stress_mrm";
    }
    return decision;
}

} // namespace mad::runtime
