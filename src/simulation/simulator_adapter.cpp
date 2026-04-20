#include "mad/simulation/simulator_adapter.hpp"

namespace mad::simulation {

BuiltInWorldAdapter::BuiltInWorldAdapter(mad::map::LaneMap lane_map)
    : m_world(std::move(lane_map)) {
}

void BuiltInWorldAdapter::Reset(const ScenarioDefinition& scenario) {
    m_world.Reset(scenario);
}

void BuiltInWorldAdapter::Step(double dt, const ControlCommand& command) {
    m_world.Step(dt, command);
}

const WorldSnapshot& BuiltInWorldAdapter::Snapshot() const {
    return m_world.Snapshot();
}

bool BuiltInWorldAdapter::HasCollision() const {
    return m_world.HasCollision();
}

std::unique_ptr<ISimulatorAdapter> CreateBuiltInAdapter(const mad::map::LaneMap& lane_map) {
    return std::make_unique<BuiltInWorldAdapter>(lane_map);
}

} // namespace mad::simulation
