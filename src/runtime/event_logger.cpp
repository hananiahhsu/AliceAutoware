#include "mad/runtime/event_logger.hpp"

#include <filesystem>
#include <iomanip>

namespace mad::runtime {

EventLogger::EventLogger(const std::string& output_path)
    : m_stream(output_path) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    m_stream << "time,event_type,detail,value\n";
}

void EventLogger::Log(double sim_time, const std::string& event_type, const std::string& detail, double value) {
    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ',' << event_type << ',' << detail << ',' << value << '\n';
}

} // namespace mad::runtime
