#include "test_framework.hpp"
#include "mad/runtime/diagnostics_monitor.hpp"
#include "mad/runtime/autonomy_stack.hpp"

MAD_TEST(RuntimeDiagnostics, DiagnosticsMonitorEscalatesWarningsAndErrors) {
    mad::runtime::RunSummary summary;
    summary.min_ttc = 0.9;
    summary.avg_lane_error = 1.0;
    summary.collided = false;
    summary.aeb_triggers = 1;
    mad::runtime::DiagnosticsMonitor monitor;
    const auto snapshot = monitor.Evaluate(summary);
    MAD_REQUIRE(snapshot.overall_level == "warn");
    MAD_REQUIRE(snapshot.records.size() >= 4);
}
