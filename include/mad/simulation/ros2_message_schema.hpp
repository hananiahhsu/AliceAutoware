#pragma once

#include <string>

namespace mad::simulation {

class Ros2MessageSchemaWriter {
public:
    explicit Ros2MessageSchemaWriter(std::string output_path);
    void WriteDefaultSchemas() const;

private:
    std::string m_outputPath;
};

} // namespace mad::simulation
