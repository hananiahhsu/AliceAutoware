#include "mad/runtime/hypothesis_trace_writer.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::runtime {

HypothesisTraceWriter::HypothesisTraceWriter(const std::string& output_path)
    : m_stream(output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream << "time,actor_id,source_lane,target_lane,probability,earliest_conflict_time,merges_into_ego_lane,label,terminal_x,terminal_y\n";
}

void HypothesisTraceWriter::Log(double sim_time,
                                const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses) {
    for (const auto& item : hypotheses) {
        m_stream << std::fixed << std::setprecision(3)
                 << sim_time << ','
                 << item.actor_id << ','
                 << item.source_lane << ','
                 << item.target_lane << ','
                 << item.probability << ','
                 << item.earliest_conflict_time << ','
                 << (item.merges_into_ego_lane ? 1 : 0) << ','
                 << item.label << ','
                 << item.terminal_x << ','
                 << item.terminal_y << '\n';
    }
}

} // namespace mad::runtime
