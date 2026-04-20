#include "mad/map/lane_map.hpp"
#include "mad/runtime/autonomy_stack.hpp"
#include "mad/runtime/report_writer.hpp"
#include "mad/simulation/scenario_catalog.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    namespace fs = std::filesystem;
    fs::create_directories("out/batch");

    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    std::vector<mad::runtime::RunSummary> summaries;

    for (const auto& scenario_name : mad::simulation::AllScenarioNames()) {
        const std::string csv_log_path = "out/batch/" + scenario_name + "_eval.csv";
        auto summary = stack.RunScenario(scenario_name, 18.0, 0.1, csv_log_path);
        summaries.push_back(summary);
        std::cout << "[MAD] ran scenario: " << scenario_name
                  << " collided=" << (summary.collided ? "yes" : "no")
                  << " min_ttc=" << std::fixed << std::setprecision(2) << summary.min_ttc
                  << " aeb=" << summary.aeb_triggers
                  << " gap_reject=" << summary.gap_rejections
                  << " supervisor=" << summary.supervisor_interventions
                  << " mission=" << summary.mission_result
                  << "\n";
    }

    std::ofstream summary_csv("out/batch/scenario_summary.csv");
    summary_csv << "scenario,duration,avg_speed,min_ttc,avg_lane_error,avg_headway,max_abs_accel,aeb_triggers,yield_events,gap_rejections,supervisor_interventions,rss_lane_change_rejections,lane_sequence_replans,corridor_replans,fallback_activations,max_interaction_conflicts,diagnostics_warn_count,diagnostics_error_count,mission_result,collided,lane_changes\n";
    for (const auto& item : summaries) {
        summary_csv << item.scenario_name << ','
                    << item.sim_duration << ','
                    << item.avg_speed << ','
                    << item.min_ttc << ','
                    << item.avg_lane_error << ','
                    << item.avg_headway << ','
                    << item.max_abs_accel << ','
                    << item.aeb_triggers << ','
                    << item.yield_events << ','
                    << item.gap_rejections << ','
                    << item.supervisor_interventions << ','
                    << item.rss_lane_change_rejections << ','
                    << item.lane_sequence_replans << ','
                    << item.corridor_replans << ','
                    << item.fallback_activations << ','
                    << item.max_interaction_conflicts << ','
                    << item.diagnostics_warn_count << ','
                    << item.diagnostics_error_count << ','
                    << item.mission_result << ','
                    << (item.collided ? 1 : 0) << ','
                    << item.lane_changes << '\n';
    }

    mad::runtime::ReportWriter writer;
    writer.WriteMarkdown(summaries, "out/batch/scenario_report.md");
    return 0;
}
