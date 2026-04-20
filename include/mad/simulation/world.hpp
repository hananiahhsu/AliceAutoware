#pragma once
#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"
#include <string>
#include <vector>
namespace mad::simulation {
enum class TrafficBehavior { Cruise, CutInLeft, CutInRight, Brake };
struct ActorState {
    int id{0}; mad::common::ActorType type{mad::common::ActorType::Vehicle}; double x{0.0}; double y{0.0}; double yaw{0.0}; double speed{0.0}; double length{4.8}; double width{1.9}; int preferred_lane{1}; bool active{true}; TrafficBehavior behavior{TrafficBehavior::Cruise}; int behavior_target_lane{-1}; double behavior_trigger_x{1.0e9}; double lane_change_rate{1.2}; double desired_speed{0.0};
};
struct ControlCommand { double acceleration{0.0}; double steering_angle{0.0}; };
struct WorldSnapshot { double sim_time{0.0}; ActorState ego; std::vector<ActorState> actors; };
struct ScenarioDefinition { std::string name; ActorState ego; std::vector<ActorState> actors; };
class World {
public:
    explicit World(mad::map::LaneMap lane_map); void Reset(const ScenarioDefinition& scenario); void Step(double dt, const ControlCommand& ego_command); const WorldSnapshot& Snapshot() const { return m_snapshot; } const mad::map::LaneMap& lane_map() const { return m_laneMap; } bool HasCollision() const; double DistanceToFrontActorInLane(int lane_id) const;
private:
    void UpdateEgo(double dt, const ControlCommand& ego_command); void UpdateTraffic(double dt); void UpdateTrafficBehavior(ActorState& actor, double dt); mad::map::LaneMap m_laneMap; WorldSnapshot m_snapshot{};
};
ScenarioDefinition MakeScenario(const std::string& scenario_name, const mad::map::LaneMap& lane_map); TrafficBehavior TrafficBehaviorFromString(const std::string& text); std::string ToString(TrafficBehavior behavior);
}
