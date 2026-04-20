#pragma once

#include "mad/prediction/joint_interaction_predictor.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::runtime {

class InteractionTraceWriter {
public:
    explicit InteractionTraceWriter(const std::string& output_path);
    void Log(double sim_time, const std::vector<mad::prediction::InteractionConflict>& conflicts);

private:
    std::ofstream m_stream;
};

} // namespace mad::runtime
