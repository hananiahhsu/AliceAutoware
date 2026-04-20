#pragma once

#include "mad/prediction/trajectory_hypothesis_generator.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::runtime {

class HypothesisTraceWriter {
public:
    explicit HypothesisTraceWriter(const std::string& output_path);

    void Log(double sim_time,
             const std::vector<mad::prediction::TrajectoryHypothesis>& hypotheses);

private:
    std::ofstream m_stream;
};

} // namespace mad::runtime
