#include "mad/runtime/diagnostics_monitor.hpp"
#include "mad/runtime/autonomy_stack.hpp"

namespace mad::runtime {

DiagnosticSnapshot DiagnosticsMonitor::Evaluate(const RunSummary& summary) const {
    DiagnosticSnapshot snapshot;
    snapshot.records.push_back({"min_ttc", summary.min_ttc, summary.min_ttc < 1.2 ? "warn" : "info"});
    snapshot.records.push_back({"avg_lane_error", summary.avg_lane_error, summary.avg_lane_error > 0.8 ? "warn" : "info"});
    snapshot.records.push_back({"aeb_triggers", static_cast<double>(summary.aeb_triggers), summary.aeb_triggers > 0 ? "warn" : "info"});
    snapshot.records.push_back({"gap_rejections", static_cast<double>(summary.gap_rejections), summary.gap_rejections > 2 ? "warn" : "info"});
    snapshot.records.push_back({"supervisor_interventions", static_cast<double>(summary.supervisor_interventions), summary.supervisor_interventions > 0 ? "warn" : "info"});
    snapshot.records.push_back({"rss_lane_change_rejections", static_cast<double>(summary.rss_lane_change_rejections), summary.rss_lane_change_rejections > 0 ? "warn" : "info"});
    snapshot.records.push_back({"fallback_activations", static_cast<double>(summary.fallback_activations), summary.fallback_activations > 0 ? "warn" : "info"});
    snapshot.records.push_back({"collided", summary.collided ? 1.0 : 0.0, summary.collided ? "error" : "info"});
    for (const auto& item : snapshot.records) {
        if (item.level == "error") {
            snapshot.overall_level = "error";
            return snapshot;
        }
        if (item.level == "warn") {
            snapshot.overall_level = "warn";
        }
    }
    return snapshot;
}

} // namespace mad::runtime
