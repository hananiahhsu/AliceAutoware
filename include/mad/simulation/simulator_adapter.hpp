#pragma once

#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

#include <memory>

namespace mad::simulation {

class ISimulatorAdapter {
public:
    virtual ~ISimulatorAdapter() = default;
    virtual void Reset(const ScenarioDefinition& scenario) = 0;
    virtual void Step(double dt, const ControlCommand& command) = 0;
    virtual const WorldSnapshot& Snapshot() const = 0;
    virtual bool HasCollision() const = 0;
};

class BuiltInWorldAdapter final : public ISimulatorAdapter {
public:
    explicit BuiltInWorldAdapter(mad::map::LaneMap lane_map);

    void Reset(const ScenarioDefinition& scenario) override;
    void Step(double dt, const ControlCommand& command) override;
    const WorldSnapshot& Snapshot() const override;
    bool HasCollision() const override;

private:
    World m_world;
};

std::unique_ptr<ISimulatorAdapter> CreateBuiltInAdapter(const mad::map::LaneMap& lane_map);

} // namespace mad::simulation
