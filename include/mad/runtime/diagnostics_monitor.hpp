#pragma once

#include <string>
#include <vector>

namespace mad::runtime {

struct RunSummary;

struct DiagnosticRecord {
    std::string key;
    double value {0.0};
    std::string level {"info"};
};

struct DiagnosticSnapshot {
    std::vector<DiagnosticRecord> records;
    std::string overall_level {"info"};
};

class DiagnosticsMonitor {
public:
    DiagnosticSnapshot Evaluate(const RunSummary& summary) const;
};

} // namespace mad::runtime
