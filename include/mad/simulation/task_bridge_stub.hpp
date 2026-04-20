#pragma once

#include "mad/planning/task_tree.hpp"

#include <fstream>
#include <string>

namespace mad::simulation {

class TaskBridgeStub {
public:
    explicit TaskBridgeStub(const std::string& output_path);
    void Publish(double sim_time, const mad::planning::TaskTreeDecision& decision);

private:
    std::ofstream m_stream;
};

} // namespace mad::simulation
