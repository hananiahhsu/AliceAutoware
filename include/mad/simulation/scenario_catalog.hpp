#pragma once

#include <string>
#include <vector>

namespace mad::simulation {

std::vector<std::string> BuiltInScenarioNames();
std::vector<std::string> ExternalScenarioNames();
std::vector<std::string> AllScenarioNames();

} // namespace mad::simulation
