#include "mad/runtime/interaction_trace_writer.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::runtime {

InteractionTraceWriter::InteractionTraceWriter(const std::string& output_path)
    : m_stream(output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream << "time,primary_actor,secondary_actor,target_lane,time_to_overlap,longitudinal_separation,combined_probability,severity,label\n";
}

void InteractionTraceWriter::Log(double sim_time, const std::vector<mad::prediction::InteractionConflict>& conflicts) {
    for (const auto& conflict : conflicts) {
        m_stream << std::fixed << std::setprecision(3)
                 << sim_time << ','
                 << conflict.primary_actor_id << ','
                 << conflict.secondary_actor_id << ','
                 << conflict.target_lane << ','
                 << conflict.time_to_overlap << ','
                 << conflict.longitudinal_separation << ','
                 << conflict.combined_probability << ','
                 << conflict.severity << ','
                 << conflict.label << '\n';
    }
}

} // namespace mad::runtime
