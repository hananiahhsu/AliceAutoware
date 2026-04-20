#include "mad/simulation/task_bridge_stub.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::simulation {

TaskBridgeStub::TaskBridgeStub(const std::string& output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream.open(output_path);
    m_stream << "time,task_directive,task_reason,requested_lane,freeze_lane_change,speed_cap_bias\n";
}

void TaskBridgeStub::Publish(double sim_time, const mad::planning::TaskTreeDecision& decision) {
    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ','
             << mad::planning::TaskTreePlanner::ToString(decision.directive) << ','
             << decision.reason << ','
             << decision.requested_lane << ','
             << (decision.freeze_lane_change ? 1 : 0) << ','
             << decision.speed_cap_bias << '\n';
}

} // namespace mad::simulation
