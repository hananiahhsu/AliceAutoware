#pragma once

#include "mad/planning/behavior_planner.hpp"
#include "mad/planning/gap_acceptance_evaluator.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::safety {

struct SafetySupervisorDecision {
    bool veto_lane_change {false};
    bool speed_clamp_active {false};
    bool emergency_brake {false};
    double override_target_speed {0.0};
    double override_acceleration {0.0};
    double rss_safe_distance {0.0};
    int intervention_level {0};
    std::string reason {"none"};
};

class SafetySupervisor {
public:
    void Reset();

    SafetySupervisorDecision Evaluate(const mad::simulation::WorldSnapshot& snapshot,
                                      const mad::planning::BehaviorDecision& behavior,
                                      const mad::planning::GapAcceptanceDecision& gap_acceptance,
                                      const std::vector<mad::prediction::RiskObject>& top_risks,
                                      const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses) ;

    int veto_count() const { return m_vetoCount; }
    int intervention_count() const { return m_interventionCount; }

    static double ComputeRssSafeDistance(double ego_speed,
                                         double front_speed,
                                         double response_time,
                                         double ego_max_accel,
                                         double ego_comfort_brake,
                                         double front_brake);

private:
    int m_vetoCount {0};
    int m_interventionCount {0};
};

} // namespace mad::safety
