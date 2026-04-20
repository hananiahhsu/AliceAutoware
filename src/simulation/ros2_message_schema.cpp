#include "mad/simulation/ros2_message_schema.hpp"

#include <filesystem>
#include <fstream>

namespace mad::simulation {

Ros2MessageSchemaWriter::Ros2MessageSchemaWriter(std::string output_path)
    : m_outputPath(std::move(output_path)) {
}

void Ros2MessageSchemaWriter::WriteDefaultSchemas() const {
    std::filesystem::create_directories(std::filesystem::path(m_outputPath).parent_path());
    std::ofstream out(m_outputPath);
    out << "# ROS2 Topic Schema Stub\n\n";
    out << "topic: /mad/localization/ego_state\n";
    out << "fields: stamp,x,y,yaw,speed,current_lane\n\n";
    out << "topic: /mad/prediction/scene_risk\n";
    out << "fields: stamp,current_lane_risk,preferred_lane_risk,global_speed_cap,reason\n\n";
    out << "topic: /mad/prediction/hypothesis\n";
    out << "fields: stamp,actor_id,target_lane,probability,earliest_conflict_time,label\n\n";
    out << "topic: /mad/planning/behavior_tree\n";
    out << "fields: stamp,directive,reason,target_lane,target_speed\n\n";
    out << "topic: /mad/runtime/task_tree\n";
    out << "fields: stamp,task_directive,task_reason,requested_lane,freeze_lane_change,speed_cap_bias\n\n";
    out << "topic: /mad/safety/supervisor\n";
    out << "fields: stamp,intervention_level,reason,veto_lane_change,override_target_speed,override_acceleration\n\n";
    out << "topic: /mad/control/command\n";
    out << "fields: stamp,acceleration,steering_angle\n";
}

} // namespace mad::simulation
