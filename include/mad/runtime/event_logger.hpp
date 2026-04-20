#pragma once

#include <fstream>
#include <string>

namespace mad::runtime {

class EventLogger {
public:
    explicit EventLogger(const std::string& output_path);
    void Log(double sim_time, const std::string& event_type, const std::string& detail, double value = 0.0);

private:
    std::ofstream m_stream;
};

} // namespace mad::runtime
