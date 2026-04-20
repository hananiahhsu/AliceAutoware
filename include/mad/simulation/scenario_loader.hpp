#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

#include <optional>
#include <string>

namespace mad::simulation {

class ScenarioLoader {
public:
    std::optional<ScenarioDefinition> TryLoad(const std::string& scenario_name,
                                              const mad::map::LaneMap& lane_map,
                                              const std::string& scenario_directory) const;
};

} // namespace mad::simulation
