#include "mad/simulation/carla_bridge_stub.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>

namespace mad::simulation {

CarlaBridgeStub::CarlaBridgeStub(const std::string& output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream.open(output_path);
    m_stream << "time,ego_x,ego_y,ego_speed,actor_count,actors,bt_directive,task_directive,scene_risk_reason,current_lane_risk,speed_cap,top_risk_actor,top_risk_score\n";
}

void CarlaBridgeStub::PublishFrame(const WorldSnapshot& snapshot,
                                   const mad::planning::BehaviorTreeDecision& bt_decision,
                                   const mad::planning::TaskTreeDecision& task_decision,
                                   const mad::prediction::SceneRiskSummary& risk_summary,
                                   const std::vector<mad::prediction::RiskObject>& top_risks) {
    std::ostringstream encoded;
    for (std::size_t i = 0; i < snapshot.actors.size(); ++i) {
        const auto& actor = snapshot.actors[i];
        encoded << actor.id << ':' << actor.x << ':' << actor.y << ':' << actor.speed << ':' << ToString(actor.behavior);
        if (i + 1 < snapshot.actors.size()) {
            encoded << '|';
        }
    }

    const auto* top_risk = top_risks.empty() ? nullptr : &top_risks.front();
    m_stream << std::fixed << std::setprecision(3)
             << snapshot.sim_time << ','
             << snapshot.ego.x << ','
             << snapshot.ego.y << ','
             << snapshot.ego.speed << ','
             << snapshot.actors.size() << ','
             << encoded.str() << ','
             << mad::planning::BehaviorTreePlanner::ToString(bt_decision.directive) << ','
             << mad::planning::TaskTreePlanner::ToString(task_decision.directive) << ','
             << risk_summary.reason << ','
             << risk_summary.current_lane_risk << ','
             << risk_summary.global_speed_cap << ','
             << (top_risk != nullptr ? top_risk->actor_id : -1) << ','
             << (top_risk != nullptr ? top_risk->risk_score : 0.0) << '\n';
}

} // namespace mad::simulation
