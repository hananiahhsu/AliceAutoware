#pragma once

#include <string>

namespace mad::runtime {

struct FallbackFrameInput {
    double min_ttc {1.0e9};
    bool supervisor_intervened {false};
    bool guardian_aeb_active {false};
    bool route_blocked {false};
    bool lane_change_rejected {false};
    double current_speed {0.0};
};

struct FallbackDecision {
    bool active {false};
    bool minimal_risk_stop {false};
    double override_target_speed {0.0};
    int escalation_level {0};
    std::string reason {"none"};
};

class FallbackManager {
public:
    void Reset();
    FallbackDecision Update(const FallbackFrameInput& input);

    double stress_score() const { return m_stressScore; }
    int activation_count() const { return m_activationCount; }

private:
    double m_stressScore {0.0};
    bool m_active {false};
    int m_activationCount {0};
};

} // namespace mad::runtime
