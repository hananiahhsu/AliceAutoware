#include "mad/map/lane_map.hpp"
#include "mad/runtime/autonomy_stack.hpp"
#include "mad/visualization/dashboard_renderer.hpp"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
    const std::string scenario_name = (argc > 1) ? argv[1] : "highway_lane_change";
    const double sim_duration = (argc > 2) ? std::stod(argv[2]) : 18.0;
    const double dt = (argc > 3) ? std::stod(argv[3]) : 0.1;

    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);

    const std::filesystem::path output_root = std::filesystem::path("out") / "visualization";
    const std::filesystem::path csv_log_path = std::filesystem::path("out") / "logs" / (scenario_name + "_log.csv");
    const std::filesystem::path svg_path = output_root / (scenario_name + "_dashboard.svg");

    const auto summary = stack.RunScenario(scenario_name, sim_duration, dt, csv_log_path.string());

    mad::visualization::VisualizationOptions options;
    options.title = "MAD Visualizer";

    if (!mad::visualization::RenderDashboardSvg(csv_log_path.string(), summary, lane_map, svg_path.string(), options)) {
        std::cerr << "[MAD] failed to generate dashboard: " << svg_path.string() << '\n';
        return 1;
    }

    std::cout << "[MAD] visualizer scenario : " << summary.scenario_name << '\n';
    std::cout << "[MAD] dashboard svg      : " << svg_path.string() << '\n';
    std::cout << "[MAD] csv log            : " << csv_log_path.string() << '\n';
    std::cout << "[MAD] mission            : " << summary.mission_result << '\n';
    std::cout << "[MAD] final pose         : x=" << summary.final_x << ", y=" << summary.final_y << '\n';
    std::cout << "[MAD] min_ttc            : " << summary.min_ttc << '\n';
    std::cout << "[MAD] collided           : " << (summary.collided ? "yes" : "no") << '\n';
    return 0;
}
