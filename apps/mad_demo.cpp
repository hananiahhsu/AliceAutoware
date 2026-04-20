#include "mad/runtime/autonomy_stack.hpp"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
    const std::string scenario_name = (argc > 1) ? argv[1] : "highway_lane_change";
    const std::filesystem::path log_path = std::filesystem::path("out") / "logs" / (scenario_name + "_log.csv");

    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario(scenario_name, 18.0, 0.1, log_path.string());

    std::cout << "[MAD] scenario      : " << summary.scenario_name << "\n";
    std::cout << "[MAD] sim_duration  : " << summary.sim_duration << " s\n";
    std::cout << "[MAD] final_pose    : x=" << summary.final_x << ", y=" << summary.final_y << "\n";
    std::cout << "[MAD] lane_changes  : " << summary.lane_changes << "\n";
    std::cout << "[MAD] avg_speed     : " << summary.avg_speed << "\n";
    std::cout << "[MAD] min_ttc       : " << summary.min_ttc << "\n";
    std::cout << "[MAD] avg_lane_err  : " << summary.avg_lane_error << "\n";
    std::cout << "[MAD] aeb_triggers  : " << summary.aeb_triggers << "\n";
    std::cout << "[MAD] gap_reject    : " << summary.gap_rejections << "\n";
    std::cout << "[MAD] supervisor    : " << summary.supervisor_interventions << "\n";
    std::cout << "[MAD] yield_events  : " << summary.yield_events << "\n";
    std::cout << "[MAD] mission       : " << summary.mission_result << "\n";
    std::cout << "[MAD] collided      : " << (summary.collided ? "yes" : "no") << "\n";
    std::cout << "[MAD] log           : " << log_path.string() << "\n";

    return summary.collided ? 2 : 0;
}
