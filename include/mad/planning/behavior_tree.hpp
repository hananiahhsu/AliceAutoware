#pragma once

#include <string>

namespace mad::planning {

enum class BehaviorDirective {
    Cruise,
    Follow,
    Yield,
    ChangeLaneLeft,
    ChangeLaneRight,
    EmergencyBrake,
};

struct BehaviorTreeInput {
    int current_lane {0};
    int preferred_lane {0};
    int route_goal_lane {0};
    double front_gap {1.0e9};
    double min_ttc {1.0e9};
    double current_lane_risk {0.0};
    double preferred_lane_risk {0.0};
    int likely_cut_in_count {0};
    bool lane_change_in_progress {false};
    bool emergency_recommended {false};
};

struct BehaviorTreeDecision {
    BehaviorDirective directive {BehaviorDirective::Cruise};
    std::string reason {"bt_cruise"};
};

class BehaviorTreePlanner {
public:
    BehaviorTreeDecision Tick(const BehaviorTreeInput& input) const;
    static std::string ToString(BehaviorDirective directive);
};

} // namespace mad::planning
