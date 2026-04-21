#include "mad/visualization/dashboard_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace mad::visualization {
namespace {

struct ActorVisual {
    int id {0};
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    int preferred_lane {0};
    std::string behavior {"cruise"};
};

struct LogRecord {
    double time {0.0};
    double ego_x {0.0};
    double ego_y {0.0};
    double ego_speed {0.0};
    std::string decision;
    int target_lane {0};
    std::string task_directive;
    int top_risk_actor {-1};
    double top_risk_score {0.0};
    double min_ttc {0.0};
    double lane_error {0.0};
    double headway {0.0};
    double longitudinal_accel {0.0};
    bool collided {false};
    bool aeb_active {false};
    std::string mission_state;
    int likely_cut_in_objects {0};
    std::string safety_reason;
    std::vector<ActorVisual> actors;
};

std::string EscapeXml(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
        case '&': escaped += "&amp;"; break;
        case '<': escaped += "&lt;"; break;
        case '>': escaped += "&gt;"; break;
        case '\"': escaped += "&quot;"; break;
        case '\'': escaped += "&apos;"; break;
        default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

std::vector<std::string> Split(const std::string& text, const char delimiter) {
    std::vector<std::string> parts;
    std::string current;
    std::istringstream stream(text);
    while (std::getline(stream, current, delimiter)) {
        parts.push_back(current);
    }
    return parts;
}

std::vector<ActorVisual> ParseActors(const std::string& encoded) {
    std::vector<ActorVisual> actors;
    if (encoded.empty()) {
        return actors;
    }
    for (const auto& token : Split(encoded, '|')) {
        if (token.empty()) {
            continue;
        }
        const auto fields = Split(token, ':');
        if (fields.size() < 6) {
            continue;
        }
        ActorVisual actor;
        actor.id = std::stoi(fields[0]);
        actor.x = std::stod(fields[1]);
        actor.y = std::stod(fields[2]);
        actor.speed = std::stod(fields[3]);
        actor.preferred_lane = std::stoi(fields[4]);
        actor.behavior = fields[5];
        actors.push_back(actor);
    }
    return actors;
}

std::vector<LogRecord> LoadRecords(const std::string& csv_log_path) {
    std::ifstream stream(csv_log_path);
    std::vector<LogRecord> records;
    if (!stream.is_open()) {
        return records;
    }

    std::string line;
    std::getline(stream, line); // header
    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = Split(line, ',');
        if (fields.size() < 23) {
            continue;
        }

        LogRecord record;
        record.time = std::stod(fields[0]);
        record.ego_x = std::stod(fields[1]);
        record.ego_y = std::stod(fields[2]);
        record.ego_speed = std::stod(fields[4]);
        record.decision = fields[5];
        record.target_lane = std::stoi(fields[6]);
        record.task_directive = fields[7];
        record.top_risk_actor = std::stoi(fields[8]);
        record.top_risk_score = std::stod(fields[9]);
        record.min_ttc = std::stod(fields[10]);
        record.lane_error = std::stod(fields[11]);
        record.headway = std::stod(fields[12]);
        record.longitudinal_accel = std::stod(fields[13]);
        record.collided = std::stoi(fields[17]) != 0;
        record.aeb_active = std::stoi(fields[18]) != 0;
        record.mission_state = fields[19];
        record.likely_cut_in_objects = std::stoi(fields[20]);
        record.safety_reason = fields[21];
        record.actors = ParseActors(fields[22]);
        records.push_back(std::move(record));
    }
    return records;
}

double Clamp01(const double value) {
    return std::max(0.0, std::min(1.0, value));
}

double Remap(const double value, const double src_min, const double src_max, const double dst_min, const double dst_max) {
    if (std::abs(src_max - src_min) < 1.0e-9) {
        return dst_min;
    }
    const double alpha = Clamp01((value - src_min) / (src_max - src_min));
    return dst_min + alpha * (dst_max - dst_min);
}

std::string FormatDouble(const double value, const int precision = 2) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::string BuildPolyline(const std::vector<LogRecord>& records,
                          const double x0,
                          const double y0,
                          const double width,
                          const double height,
                          const double value_min,
                          const double value_max,
                          const bool use_speed) {
    std::ostringstream path;
    for (std::size_t index = 0; index < records.size(); ++index) {
        const double x = x0 + (records.size() > 1 ? (width * static_cast<double>(index) / static_cast<double>(records.size() - 1)) : 0.0);
        const double metric = use_speed ? records[index].ego_speed : records[index].min_ttc;
        const double y = Remap(metric, value_min, value_max, y0 + height, y0);
        path << (index == 0 ? "M " : " L ") << FormatDouble(x, 1) << ' ' << FormatDouble(y, 1);
    }
    return path.str();
}

void AppendText(std::ostringstream& svg,
                const double x,
                const double y,
                const std::string& text,
                const int font_size,
                const std::string& fill,
                const std::string& font_weight = "400") {
    svg << "<text x=\"" << FormatDouble(x, 1)
        << "\" y=\"" << FormatDouble(y, 1)
        << "\" fill=\"" << fill
        << "\" font-size=\"" << font_size
        << "\" font-family=\"Inter,Segoe UI,Arial,sans-serif\" font-weight=\"" << font_weight
        << "\">" << EscapeXml(text) << "</text>\n";
}

void AppendRoundedRect(std::ostringstream& svg,
                       const double x,
                       const double y,
                       const double width,
                       const double height,
                       const std::string& fill,
                       const std::string& stroke = "none",
                       const double stroke_width = 1.0,
                       const double radius = 18.0) {
    svg << "<rect x=\"" << FormatDouble(x, 1)
        << "\" y=\"" << FormatDouble(y, 1)
        << "\" width=\"" << FormatDouble(width, 1)
        << "\" height=\"" << FormatDouble(height, 1)
        << "\" rx=\"" << FormatDouble(radius, 1)
        << "\" fill=\"" << fill
        << "\" stroke=\"" << stroke
        << "\" stroke-width=\"" << FormatDouble(stroke_width, 1)
        << "\"/>\n";
}

} // namespace

bool RenderDashboardSvg(const std::string& csv_log_path,
                        const mad::runtime::RunSummary& summary,
                        const mad::map::LaneMap& lane_map,
                        const std::string& output_svg_path,
                        const VisualizationOptions& options) {
    const auto records = LoadRecords(csv_log_path);
    if (records.empty()) {
        return false;
    }

    const LogRecord& final_record = records.back();
    const double width = static_cast<double>(options.width);
    const double height = static_cast<double>(options.height);

    const double title_bar_height = 52.0;
    const double content_padding = 24.0;
    const double road_panel_x = content_padding;
    const double road_panel_y = title_bar_height + content_padding;
    const double road_panel_width = 890.0;
    const double road_panel_height = 560.0;
    const double chart_panel_x = content_padding;
    const double chart_panel_y = road_panel_y + road_panel_height + 18.0;
    const double chart_panel_width = road_panel_width;
    const double chart_panel_height = 220.0;
    const double side_panel_x = road_panel_x + road_panel_width + 18.0;
    const double side_panel_y = road_panel_y;
    const double side_panel_width = width - side_panel_x - content_padding;
    const double side_panel_height = height - side_panel_y - content_padding;

    double x_min = final_record.ego_x - 45.0;
    double x_max = final_record.ego_x + 140.0;
    for (const auto& actor : final_record.actors) {
        x_min = std::min(x_min, actor.x - 12.0);
        x_max = std::max(x_max, actor.x + 12.0);
    }

    const double road_half_width = lane_map.lane_width() * static_cast<double>(lane_map.lane_count()) * 0.5;
    const double road_y_min = -road_half_width;
    const double road_y_max = road_half_width;

    double max_speed = 0.0;
    double max_ttc = 0.0;
    for (const auto& record : records) {
        max_speed = std::max(max_speed, record.ego_speed);
        max_ttc = std::max(max_ttc, record.min_ttc);
    }
    max_speed = std::max(max_speed, 25.0);
    max_ttc = std::max(max_ttc, 8.0);

    std::ostringstream svg;
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << options.width
        << "\" height=\"" << options.height
        << "\" viewBox=\"0 0 " << options.width << ' ' << options.height << "\">\n";
    svg << "<defs>\n"
        << "  <linearGradient id=\"bg\" x1=\"0\" x2=\"0\" y1=\"0\" y2=\"1\">\n"
        << "    <stop offset=\"0%\" stop-color=\"#0f172a\"/>\n"
        << "    <stop offset=\"100%\" stop-color=\"#111827\"/>\n"
        << "  </linearGradient>\n"
        << "  <linearGradient id=\"roadGlow\" x1=\"0\" x2=\"1\" y1=\"0\" y2=\"0\">\n"
        << "    <stop offset=\"0%\" stop-color=\"#111827\"/>\n"
        << "    <stop offset=\"100%\" stop-color=\"#1f2937\"/>\n"
        << "  </linearGradient>\n"
        << "  <filter id=\"shadow\" x=\"-20%\" y=\"-20%\" width=\"140%\" height=\"140%\">\n"
        << "    <feDropShadow dx=\"0\" dy=\"12\" stdDeviation=\"16\" flood-color=\"#020617\" flood-opacity=\"0.45\"/>\n"
        << "  </filter>\n"
        << "</defs>\n";

    svg << "<rect width=\"100%\" height=\"100%\" fill=\"url(#bg)\"/>\n";
    svg << "<rect x=\"0\" y=\"0\" width=\"" << options.width << "\" height=\"" << title_bar_height
        << "\" fill=\"#0b1220\"/>\n";
    svg << "<circle cx=\"22\" cy=\"26\" r=\"6\" fill=\"#ef4444\"/><circle cx=\"42\" cy=\"26\" r=\"6\" fill=\"#f59e0b\"/><circle cx=\"62\" cy=\"26\" r=\"6\" fill=\"#22c55e\"/>\n";
    AppendText(svg, 92.0, 31.0, options.title + " - " + summary.scenario_name, 20, "#e5e7eb", "600");
    AppendText(svg, width - 230.0, 31.0, "Industrial AD Stack Snapshot", 14, "#94a3b8", "500");

    AppendRoundedRect(svg, road_panel_x, road_panel_y, road_panel_width, road_panel_height, "#0b1325", "#1f2937", 1.0, 24.0);
    AppendRoundedRect(svg, chart_panel_x, chart_panel_y, chart_panel_width, chart_panel_height, "#0b1325", "#1f2937", 1.0, 24.0);
    AppendRoundedRect(svg, side_panel_x, side_panel_y, side_panel_width, side_panel_height, "#0b1325", "#1f2937", 1.0, 24.0);

    AppendText(svg, road_panel_x + 26.0, road_panel_y + 34.0, "Ego-centric highway view", 22, "#f8fafc", "600");
    AppendText(svg, road_panel_x + 26.0, road_panel_y + 58.0, "Lane topology, actors, and current planning state", 13, "#94a3b8");

    const double road_x = road_panel_x + 22.0;
    const double road_y = road_panel_y + 84.0;
    const double road_width = road_panel_width - 44.0;
    const double road_height = road_panel_height - 108.0;

    svg << "<rect x=\"" << FormatDouble(road_x, 1) << "\" y=\"" << FormatDouble(road_y, 1)
        << "\" width=\"" << FormatDouble(road_width, 1) << "\" height=\"" << FormatDouble(road_height, 1)
        << "\" rx=\"18\" fill=\"url(#roadGlow)\" stroke=\"#334155\" stroke-width=\"1.2\"/>\n";

    for (int lane = 0; lane < lane_map.lane_count(); ++lane) {
        const double lane_center = lane_map.LaneCenterY(lane);
        const double y = Remap(lane_center, road_y_min, road_y_max, road_y + road_height - 26.0, road_y + 26.0);
        AppendText(svg, road_x + 18.0, y - 8.0, "Lane " + std::to_string(lane), 13, "#94a3b8", lane == final_record.target_lane ? "700" : "500");
    }

    for (int boundary = 0; boundary <= lane_map.lane_count(); ++boundary) {
        const double boundary_y_world = -road_half_width + static_cast<double>(boundary) * lane_map.lane_width();
        const double y = Remap(boundary_y_world, road_y_min, road_y_max, road_y + road_height, road_y);
        if (boundary == 0 || boundary == lane_map.lane_count()) {
            svg << "<line x1=\"" << FormatDouble(road_x + 12.0, 1) << "\" y1=\"" << FormatDouble(y, 1)
                << "\" x2=\"" << FormatDouble(road_x + road_width - 12.0, 1) << "\" y2=\"" << FormatDouble(y, 1)
                << "\" stroke=\"#94a3b8\" stroke-width=\"3\" opacity=\"0.7\"/>\n";
        } else {
            svg << "<line x1=\"" << FormatDouble(road_x + 12.0, 1) << "\" y1=\"" << FormatDouble(y, 1)
                << "\" x2=\"" << FormatDouble(road_x + road_width - 12.0, 1) << "\" y2=\"" << FormatDouble(y, 1)
                << "\" stroke=\"#f8fafc\" stroke-width=\"2\" stroke-dasharray=\"16 16\" opacity=\"0.35\"/>\n";
        }
    }

    for (int tick = 0; tick <= 5; ++tick) {
        const double world_x = x_min + (x_max - x_min) * static_cast<double>(tick) / 5.0;
        const double x = Remap(world_x, x_min, x_max, road_x + 28.0, road_x + road_width - 28.0);
        svg << "<line x1=\"" << FormatDouble(x, 1) << "\" y1=\"" << FormatDouble(road_y + 18.0, 1)
            << "\" x2=\"" << FormatDouble(x, 1) << "\" y2=\"" << FormatDouble(road_y + road_height - 18.0, 1)
            << "\" stroke=\"#1e293b\" stroke-width=\"1\" opacity=\"0.65\"/>\n";
        AppendText(svg, x - 14.0, road_y + road_height - 8.0, FormatDouble(world_x, 0) + " m", 12, "#64748b");
    }

    auto append_vehicle = [&](const double actor_x,
                              const double actor_y,
                              const double length,
                              const double width_value,
                              const std::string& fill,
                              const std::string& stroke,
                              const std::string& label,
                              const bool highlight) {
        const double screen_x = Remap(actor_x, x_min, x_max, road_x + 28.0, road_x + road_width - 28.0);
        const double screen_y = Remap(actor_y, road_y_min, road_y_max, road_y + road_height - 26.0, road_y + 26.0);
        const double pixel_length = std::max(32.0, length * 4.8);
        const double pixel_width = std::max(18.0, width_value * 7.0);
        const double rect_x = screen_x - pixel_length * 0.5;
        const double rect_y = screen_y - pixel_width * 0.5;
        if (highlight) {
            svg << "<rect x=\"" << FormatDouble(rect_x - 6.0, 1) << "\" y=\"" << FormatDouble(rect_y - 6.0, 1)
                << "\" width=\"" << FormatDouble(pixel_length + 12.0, 1) << "\" height=\"" << FormatDouble(pixel_width + 12.0, 1)
                << "\" rx=\"12\" fill=\"none\" stroke=\"#f59e0b\" stroke-width=\"2.5\" stroke-dasharray=\"7 5\" opacity=\"0.9\"/>\n";
        }
        svg << "<rect x=\"" << FormatDouble(rect_x, 1) << "\" y=\"" << FormatDouble(rect_y, 1)
            << "\" width=\"" << FormatDouble(pixel_length, 1) << "\" height=\"" << FormatDouble(pixel_width, 1)
            << "\" rx=\"10\" fill=\"" << fill << "\" stroke=\"" << stroke << "\" stroke-width=\"1.5\" filter=\"url(#shadow)\"/>\n";
        AppendText(svg, rect_x + 6.0, rect_y - 9.0, label, 12, "#e2e8f0", "600");
    };

    append_vehicle(final_record.ego_x, final_record.ego_y, 4.8, 1.9, "#38bdf8", "#e0f2fe", "EGO", true);
    for (const auto& actor : final_record.actors) {
        const bool is_top_risk = actor.id == final_record.top_risk_actor;
        const std::string fill = is_top_risk ? "#fb7185" : "#a78bfa";
        const std::string stroke = is_top_risk ? "#fecdd3" : "#ede9fe";
        append_vehicle(actor.x, actor.y, 4.8, 1.9, fill, stroke, "ID " + std::to_string(actor.id), is_top_risk);
    }

    AppendText(svg, road_x + road_width - 180.0, road_y + 30.0, "Decision: " + final_record.decision, 15, "#f8fafc", "600");
    AppendText(svg, road_x + road_width - 180.0, road_y + 54.0, "Safety: " + final_record.safety_reason, 13, "#f59e0b");

    AppendText(svg, chart_panel_x + 26.0, chart_panel_y + 34.0, "Run traces", 22, "#f8fafc", "600");
    AppendText(svg, chart_panel_x + 26.0, chart_panel_y + 58.0, "Ego speed and minimum TTC over simulation time", 13, "#94a3b8");

    const double chart_x = chart_panel_x + 26.0;
    const double chart_y = chart_panel_y + 78.0;
    const double chart_width = chart_panel_width - 52.0;
    const double chart_height = chart_panel_height - 104.0;
    svg << "<rect x=\"" << FormatDouble(chart_x, 1) << "\" y=\"" << FormatDouble(chart_y, 1)
        << "\" width=\"" << FormatDouble(chart_width, 1) << "\" height=\"" << FormatDouble(chart_height, 1)
        << "\" rx=\"16\" fill=\"#0f172a\" stroke=\"#1e293b\" stroke-width=\"1\"/>\n";
    for (int row = 0; row <= 4; ++row) {
        const double y = chart_y + chart_height * static_cast<double>(row) / 4.0;
        svg << "<line x1=\"" << FormatDouble(chart_x + 14.0, 1) << "\" y1=\"" << FormatDouble(y, 1)
            << "\" x2=\"" << FormatDouble(chart_x + chart_width - 14.0, 1) << "\" y2=\"" << FormatDouble(y, 1)
            << "\" stroke=\"#1e293b\" stroke-width=\"1\"/>\n";
    }
    const std::string speed_path = BuildPolyline(records, chart_x + 18.0, chart_y + 12.0, chart_width - 36.0, chart_height - 24.0, 0.0, max_speed, true);
    const std::string ttc_path = BuildPolyline(records, chart_x + 18.0, chart_y + 12.0, chart_width - 36.0, chart_height - 24.0, 0.0, max_ttc, false);
    svg << "<path d=\"" << speed_path << "\" fill=\"none\" stroke=\"#38bdf8\" stroke-width=\"3\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/>\n";
    svg << "<path d=\"" << ttc_path << "\" fill=\"none\" stroke=\"#f59e0b\" stroke-width=\"3\" stroke-linecap=\"round\" stroke-linejoin=\"round\" opacity=\"0.95\"/>\n";
    AppendText(svg, chart_x + 18.0, chart_y + chart_height + 24.0, "Blue: ego speed (m/s)", 13, "#38bdf8", "600");
    AppendText(svg, chart_x + 210.0, chart_y + chart_height + 24.0, "Amber: min TTC (s)", 13, "#f59e0b", "600");

    AppendText(svg, side_panel_x + 24.0, side_panel_y + 34.0, "Mission overview", 22, "#f8fafc", "600");
    AppendText(svg, side_panel_x + 24.0, side_panel_y + 58.0, "Runtime KPIs, safety state, and scenario status", 13, "#94a3b8");

    struct MetricCard {
        std::string label;
        std::string value;
        std::string accent;
    };

    const std::vector<MetricCard> cards = {
        {"Mission", summary.mission_result, "#22c55e"},
        {"Lane changes", std::to_string(summary.lane_changes), "#38bdf8"},
        {"Min TTC", FormatDouble(summary.min_ttc) + " s", "#f59e0b"},
        {"Avg speed", FormatDouble(summary.avg_speed) + " m/s", "#a78bfa"},
        {"Gap rejects", std::to_string(summary.gap_rejections), "#fb7185"},
        {"Supervisor", std::to_string(summary.supervisor_interventions), "#f97316"},
        {"AEB triggers", std::to_string(summary.aeb_triggers), "#ef4444"},
        {"Diagnostics warn", std::to_string(summary.diagnostics_warn_count), "#facc15"}
    };

    const double card_width = (side_panel_width - 58.0) * 0.5;
    const double card_height = 92.0;
    for (std::size_t index = 0; index < cards.size(); ++index) {
        const int row = static_cast<int>(index / 2);
        const int col = static_cast<int>(index % 2);
        const double x = side_panel_x + 24.0 + static_cast<double>(col) * (card_width + 10.0);
        const double y = side_panel_y + 84.0 + static_cast<double>(row) * (card_height + 10.0);
        AppendRoundedRect(svg, x, y, card_width, card_height, "#111827", "#1f2937", 1.0, 18.0);
        svg << "<rect x=\"" << FormatDouble(x + 14.0, 1) << "\" y=\"" << FormatDouble(y + 14.0, 1)
            << "\" width=\"5\" height=\"" << FormatDouble(card_height - 28.0, 1)
            << "\" rx=\"3\" fill=\"" << cards[index].accent << "\"/>\n";
        AppendText(svg, x + 28.0, y + 34.0, cards[index].label, 13, "#94a3b8", "500");
        AppendText(svg, x + 28.0, y + 68.0, cards[index].value, 24, "#f8fafc", "700");
    }

    const double narrative_y = side_panel_y + 84.0 + 4.0 * (card_height + 10.0) + 10.0;
    AppendRoundedRect(svg, side_panel_x + 24.0, narrative_y, side_panel_width - 48.0, 188.0, "#111827", "#1f2937", 1.0, 18.0);
    AppendText(svg, side_panel_x + 40.0, narrative_y + 34.0, "Scenario narrative", 18, "#f8fafc", "600");
    AppendText(svg, side_panel_x + 40.0, narrative_y + 64.0, "Scenario: " + summary.scenario_name, 14, "#cbd5e1");
    AppendText(svg, side_panel_x + 40.0, narrative_y + 90.0, "Final pose: x=" + FormatDouble(summary.final_x, 1) + " m, y=" + FormatDouble(summary.final_y, 1) + " m", 14, "#cbd5e1");
    AppendText(svg, side_panel_x + 40.0, narrative_y + 116.0, "Task directive: " + final_record.task_directive, 14, "#cbd5e1");
    AppendText(svg, side_panel_x + 40.0, narrative_y + 142.0, "Target lane: " + std::to_string(final_record.target_lane) + " | likely cut-ins: " + std::to_string(final_record.likely_cut_in_objects), 14, "#cbd5e1");
    AppendText(svg, side_panel_x + 40.0, narrative_y + 168.0, std::string("Collision: ") + (summary.collided ? "yes" : "no") + " | AEB active: " + (final_record.aeb_active ? "yes" : "no"), 14, summary.collided ? "#fecaca" : "#bbf7d0");

    svg << "</svg>\n";

    std::filesystem::create_directories(std::filesystem::path(output_svg_path).parent_path());
    std::ofstream stream(output_svg_path, std::ios::binary);
    if (!stream.is_open()) {
        return false;
    }
    stream << svg.str();
    return true;
}

} // namespace mad::visualization
