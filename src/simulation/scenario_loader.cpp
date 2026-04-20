#include "mad/simulation/scenario_loader.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace mad::simulation {
namespace {

std::filesystem::path ResolveScenarioPath(const std::string& scenario_name,
                                          const std::string& scenario_directory) {
    const std::vector<std::filesystem::path> roots {
        std::filesystem::current_path(),
        std::filesystem::current_path().parent_path(),
        std::filesystem::current_path().parent_path().parent_path(),
        std::filesystem::current_path().parent_path().parent_path().parent_path()
    };

    for (const auto& root : roots) {
        const auto candidate = root / scenario_directory / (scenario_name + ".csv");
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return {};
}

} // namespace

std::optional<ScenarioDefinition> ScenarioLoader::TryLoad(const std::string& scenario_name,
                                                          const mad::map::LaneMap& lane_map,
                                                          const std::string& scenario_directory) const {
    const auto scenario_path = ResolveScenarioPath(scenario_name, scenario_directory);
    if (scenario_path.empty()) {
        return std::nullopt;
    }

    ScenarioDefinition scenario;
    scenario.name = scenario_name;

    std::ifstream input(scenario_path);
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string role;
        std::string lane_str;
        std::string x_str;
        std::string speed_str;
        std::string behavior_str;
        std::string target_lane_str;
        std::string trigger_x_str;
        std::string desired_speed_str;
        std::getline(ss, role, ',');
        std::getline(ss, lane_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, speed_str, ',');
        std::getline(ss, behavior_str, ',');
        std::getline(ss, target_lane_str, ',');
        std::getline(ss, trigger_x_str, ',');
        std::getline(ss, desired_speed_str, ',');

        const int lane_id = std::stoi(lane_str);
        const double x = std::stod(x_str);
        const double speed = std::stod(speed_str);
        const TrafficBehavior behavior = behavior_str.empty() ? TrafficBehavior::Cruise : TrafficBehaviorFromString(behavior_str);
        const int target_lane = target_lane_str.empty() ? -1 : std::stoi(target_lane_str);
        const double trigger_x = trigger_x_str.empty() ? 1.0e9 : std::stod(trigger_x_str);
        const double desired_speed = desired_speed_str.empty() ? speed : std::stod(desired_speed_str);

        if (role == "ego") {
            scenario.ego = {1,
                            mad::common::ActorType::Ego,
                            x,
                            lane_map.LaneCenterY(lane_id),
                            0.0,
                            speed,
                            4.8,
                            1.9,
                            lane_id,
                            true,
                            behavior,
                            target_lane,
                            trigger_x,
                            1.2,
                            desired_speed};
        } else {
            const int actor_id = static_cast<int>(scenario.actors.size()) + 1001;
            scenario.actors.push_back({actor_id,
                                       mad::common::ActorType::Vehicle,
                                       x,
                                       lane_map.LaneCenterY(lane_id),
                                       0.0,
                                       speed,
                                       4.8,
                                       1.9,
                                       lane_id,
                                       true,
                                       behavior,
                                       target_lane,
                                       trigger_x,
                                       1.2,
                                       desired_speed});
        }
    }
    return scenario;
}

} // namespace mad::simulation
