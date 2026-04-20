#include "mad/runtime/report_writer.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace mad::runtime {

void ReportWriter::WriteMarkdown(const std::vector<RunSummary>& summaries,
                                 const std::string& output_path) const {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    std::ofstream out(output_path);
    out << "# Batch Evaluation Report\n\n";
    out << "| Scenario | Duration(s) | Avg Speed(m/s) | Min TTC(s) | Avg Lane Error(m) | Avg Headway(s) | Max |a|(m/s^2) | AEB | Yield Events | Gap Reject | Supervisor | RSS Reject | Corridor Replan | Fallback | Interaction Peaks | Mission Result | Collision | Lane Changes |\n";
    out << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|\n";
    for (const auto& item : summaries) {
        out << '|' << item.scenario_name
            << '|' << std::fixed << std::setprecision(2) << item.sim_duration
            << '|' << item.avg_speed
            << '|' << item.min_ttc
            << '|' << item.avg_lane_error
            << '|' << item.avg_headway
            << '|' << item.max_abs_accel
            << '|' << item.aeb_triggers
            << '|' << item.yield_events
            << '|' << item.gap_rejections
            << '|' << item.supervisor_interventions
            << '|' << item.rss_lane_change_rejections
            << '|' << item.corridor_replans
            << '|' << item.fallback_activations
            << '|' << item.max_interaction_conflicts
            << '|' << item.mission_result
            << '|' << (item.collided ? "yes" : "no")
            << '|' << item.lane_changes
            << "|\n";
    }
}

} // namespace mad::runtime
