#include "mad/simulation/scenario_catalog.hpp"

namespace mad::simulation {

std::vector<std::string> BuiltInScenarioNames() {
    return {
        "free_cruise",
        "highway_lane_change",
        "dense_following",
        "merge_pressure"
    };
}

std::vector<std::string> ExternalScenarioNames() {
    return {
        "cut_in_event",
        "lead_vehicle_brake",
        "aggressive_cut_in",
        "double_cut_in",
        "stop_and_go_wave",
        "rear_approach_pressure",
        "boxed_in_following",
        "cooperative_lane_change",
        "sudden_lane_blockage",
        "multi_interaction_weave",
        "truck_cut_in_brake",
        "corridor_blocked_recovery"
    };
}

std::vector<std::string> AllScenarioNames() {
    auto scenarios = BuiltInScenarioNames();
    const auto external = ExternalScenarioNames();
    scenarios.insert(scenarios.end(), external.begin(), external.end());
    return scenarios;
}

} // namespace mad::simulation
