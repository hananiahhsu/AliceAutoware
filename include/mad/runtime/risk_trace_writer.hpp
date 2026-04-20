#pragma once

#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/prediction/risk_object_ranker.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::runtime {

class RiskTraceWriter {
public:
    explicit RiskTraceWriter(const std::string& output_path);
    void Log(double sim_time,
             const std::vector<mad::prediction::RiskObject>& top_risks,
             const std::vector<mad::perception::LaneFlowMetrics>& lane_flows);

private:
    std::ofstream m_stream;
};

} // namespace mad::runtime
