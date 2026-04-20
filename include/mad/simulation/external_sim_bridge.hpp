#pragma once
#include "mad/simulation/world.hpp"
#include <fstream>
#include <string>
namespace mad::simulation {
class ExternalSimBridge {
public:
    ExternalSimBridge(const std::string& state_output_path, const std::string& command_output_path);
    void PublishState(const WorldSnapshot& snapshot);
    void PublishCommand(double sim_time, const ControlCommand& command);
private:
    std::ofstream m_stateStream; std::ofstream m_commandStream;
};
}
