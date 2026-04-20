#include "mad/simulation/external_sim_bridge.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::simulation {

ExternalSimBridge::ExternalSimBridge(const std::string& state_output_path,
                                     const std::string& command_output_path) {
    std::filesystem::create_directories(std::filesystem::path(state_output_path).parent_path());
    std::filesystem::create_directories(std::filesystem::path(command_output_path).parent_path());
    m_stateStream.open(state_output_path);
    m_commandStream.open(command_output_path);
    m_stateStream << "time,ego_x,ego_y,ego_speed,actor_count\n";
    m_commandStream << "time,acceleration,steering_angle\n";
}

void ExternalSimBridge::PublishState(const WorldSnapshot& snapshot) {
    m_stateStream << std::fixed << std::setprecision(3)
                  << snapshot.sim_time << ','
                  << snapshot.ego.x << ','
                  << snapshot.ego.y << ','
                  << snapshot.ego.speed << ','
                  << snapshot.actors.size() << '\n';
}

void ExternalSimBridge::PublishCommand(double sim_time, const ControlCommand& command) {
    m_commandStream << std::fixed << std::setprecision(3)
                    << sim_time << ','
                    << command.acceleration << ','
                    << command.steering_angle << '\n';
}

} // namespace mad::simulation
