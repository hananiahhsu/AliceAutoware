#include "mad/runtime/risk_trace_writer.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>

namespace mad::runtime {

RiskTraceWriter::RiskTraceWriter(const std::string& output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream.open(output_path);
    m_stream << "time,top_risks,lane_flow\n";
}

void RiskTraceWriter::Log(double sim_time,
                          const std::vector<mad::prediction::RiskObject>& top_risks,
                          const std::vector<mad::perception::LaneFlowMetrics>& lane_flows) {
    std::ostringstream risk_encoded;
    for (std::size_t i = 0; i < top_risks.size(); ++i) {
        const auto& risk = top_risks[i];
        risk_encoded << risk.actor_id << ':' << risk.label << ':' << std::fixed << std::setprecision(2) << risk.risk_score << ':' << risk.time_to_collision;
        if (i + 1 < top_risks.size()) {
            risk_encoded << '|';
        }
    }

    std::ostringstream lane_encoded;
    for (std::size_t i = 0; i < lane_flows.size(); ++i) {
        const auto& lane = lane_flows[i];
        lane_encoded << lane.lane_id << ':' << lane.tracked_count << ':' << std::fixed << std::setprecision(2) << lane.congestion_score << ':' << lane.closing_speed;
        if (i + 1 < lane_flows.size()) {
            lane_encoded << '|';
        }
    }

    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ','
             << risk_encoded.str() << ','
             << lane_encoded.str() << '\n';
}

} // namespace mad::runtime
