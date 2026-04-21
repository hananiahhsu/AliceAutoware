#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/runtime/autonomy_stack.hpp"

#include <string>

namespace mad::visualization {

struct VisualizationOptions {
    int width {1440};
    int height {900};
    std::string title {"MAD Visualizer"};
};

bool RenderDashboardSvg(const std::string& csv_log_path,
                        const mad::runtime::RunSummary& summary,
                        const mad::map::LaneMap& lane_map,
                        const std::string& output_svg_path,
                        const VisualizationOptions& options = {});

} // namespace mad::visualization
