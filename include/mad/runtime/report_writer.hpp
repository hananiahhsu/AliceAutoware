#pragma once

#include "mad/runtime/autonomy_stack.hpp"

#include <string>
#include <vector>

namespace mad::runtime {

class ReportWriter {
public:
    void WriteMarkdown(const std::vector<RunSummary>& summaries, const std::string& output_path) const;
};

} // namespace mad::runtime
