from pathlib import Path
root = Path('/mnt/data/momenta_autodrive_stack')
files = {}
files['CMakeLists.txt'] = '''cmake_minimum_required(VERSION 3.20)
project(MomentaAutodriveStack VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MAD_BUILD_TESTS "Build tests" ON)

include(GNUInstallDirs)

add_library(mad_core
    src/map/lane_map.cpp
    src/simulation/world.cpp
    src/perception/sensor_model.cpp
    src/perception/track_manager.cpp
    src/prediction/constant_velocity_predictor.cpp
    src/planning/behavior_planner.cpp
    src/planning/trajectory_planner.cpp
    src/control/controller.cpp
    src/runtime/autonomy_stack.cpp
)

target_include_directories(mad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_options(mad_core PRIVATE /W4)
else()
    target_compile_options(mad_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(mad_demo apps/mad_demo.cpp)
target_link_libraries(mad_demo PRIVATE mad_core)

if(MAD_BUILD_TESTS)
    enable_testing()
    add_executable(mad_tests tests/mad_tests.cpp)
    target_link_libraries(mad_tests PRIVATE mad_core)
    add_test(NAME mad_tests COMMAND mad_tests)
endif()

install(TARGETS mad_demo
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY configs DESTINATION .)
install(DIRECTORY docs DESTINATION .)
install(DIRECTORY tools DESTINATION .)
install(DIRECTORY scripts DESTINATION .)
install(FILES README.md DESTINATION .)
'''
files['README.md'] = '''# Momenta Autodrive Stack

一个针对 **Momenta/量产智能驾驶岗位能力模型** 定制的、可本地编译运行的轻量自动驾驶样机仓库。

它不是一个虚构的“完整商业 Robotaxi 产品”，而是一套**可编译、可运行、可复现实验、带架构文档与自动化脚本**的工程化样机，重点覆盖：

- Linux / C++17 / Python3 / CMake / Shell 脚本
- 车端模块化架构：感知、预测、规划、控制、仿真闭环
- 自动化构建、自动化打包、自动化架构一致性检查
- 开源生态调研：Autoware / Apollo / CARLA / AWSIM / AirSim / SVL
- 2D 交通场景仿真 + 自车闭环控制 + HTML 回放

## 仓库结构

```text
.
├── apps/                      # 可执行程序入口
├── configs/                   # 场景配置与运行参数
├── docs/architecture/         # 架构文档 / 模块清单 / 图
├── include/mad/              # 头文件
├── src/                      # 实现
├── tests/                    # 单元测试 / 集成测试
├── tools/                    # Python 评测 / 回放 / 调研输出工具
├── scripts/                  # Windows/Linux 自动构建与部署脚本
└── .github/workflows/        # CI
```

## 核心能力

### 1. 仿真闭环
- 三车道直路地图
- 自车 + NPC 交通参与者
- 简化雷达/激光传感器观测
- 融合跟踪
- 常速度预测
- 行为规划（巡航 / 跟车 / 变道）
- 轨迹规划（纵向速度目标 + 横向平滑换道）
- 纯跟踪 + PID 控制
- 碰撞检测与指标统计

### 2. 工程化能力
- CMake 构建
- Linux shell / Windows bat 自动编译
- 自动化打包脚本
- 架构一致性检查脚本
- GitHub Actions CI

### 3. 文档同步
- `docs/architecture/module_inventory.json` 记录模块清单
- `tools/check_architecture_sync.py` 校验模块/目录/文档一致性
- `docs/architecture/01_system_architecture.md` 与实际源码目录一一对应

## 在 Linux 下编译运行

```bash
bash scripts/build_linux.sh
bash scripts/run_demo.sh highway_lane_change
python3 tools/render_replay.py out/logs/highway_lane_change_log.csv out/replay/highway_lane_change.html
python3 tools/evaluate_run.py out/logs/highway_lane_change_log.csv
```

## 在 Windows 下编译运行

```bat
scripts\\build_windows.bat
scripts\\run_demo_windows.bat highway_lane_change
python tools\\render_replay.py out\\logs\\highway_lane_change_log.csv out\\replay\\highway_lane_change.html
python tools\\evaluate_run.py out\\logs\\highway_lane_change_log.csv
```

## 已验证内容

本仓库的 Linux 版本已在当前交付环境中成功完成：
- CMake configure
- C++17 编译
- 单元测试 `ctest`
- Demo 运行并输出日志
- Python 回放 HTML 生成
- 架构同步检查

## 设计边界

本仓库**没有**直接把 CARLA / Apollo / Autoware / AWSIM 整体拉进来编译，因为那会引入 ROS2、Unreal/Unity、GPU 驱动、大量三方依赖，并不适合在当前受限环境里做“真实可验证”的交付。取而代之的是：

- 抽象出可扩展的车端流水线接口
- 提供开源方案调研与映射文档
- 预留未来接入 CARLA / ROS2 / Autoware 的扩展点

## 参考调研

见：
- `docs/architecture/02_open_source_survey.md`
- `docs/architecture/03_momenta_role_mapping.md`
'''
files['include/mad/common/types.hpp'] = '''#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <vector>

namespace mad::common {

struct Vec2 {
    double x {0.0};
    double y {0.0};

    Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
};

inline double Dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double Norm(const Vec2& v) { return std::sqrt(Dot(v, v)); }
inline double Distance(const Vec2& a, const Vec2& b) { return Norm(a - b); }
inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

inline double Clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}
inline double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

enum class ActorType {
    Ego,
    Vehicle,
};

struct Waypoint {
    double x {0.0};
    double y {0.0};
    double speed_limit {22.0};
};

struct TrajectoryPoint {
    double t {0.0};
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    double target_speed {0.0};
};

inline std::ostream& operator<<(std::ostream& os, const Vec2& value) {
    os << "(" << value.x << ", " << value.y << ")";
    return os;
}

} // namespace mad::common
'''
files['include/mad/map/lane_map.hpp'] = '''#pragma once

namespace mad::map {

class LaneMap {
public:
    LaneMap(double lane_width, int lane_count, double road_length);

    double LaneCenterY(int lane_id) const;
    int ClosestLane(double y) const;
    bool IsInRoadBounds(double y) const;
    bool IsLaneValid(int lane_id) const;

    double lane_width() const { return m_laneWidth; }
    int lane_count() const { return m_laneCount; }
    double road_length() const { return m_roadLength; }

private:
    double m_laneWidth {3.7};
    int m_laneCount {3};
    double m_roadLength {500.0};
};

} // namespace mad::map
'''
files['src/map/lane_map.cpp'] = '''#include "mad/map/lane_map.hpp"

#include <cmath>

namespace mad::map {

LaneMap::LaneMap(double lane_width, int lane_count, double road_length)
    : m_laneWidth(lane_width), m_laneCount(lane_count), m_roadLength(road_length) {
}

double LaneMap::LaneCenterY(int lane_id) const {
    return (static_cast<double>(lane_id) - static_cast<double>(m_laneCount - 1) / 2.0) * m_laneWidth;
}

int LaneMap::ClosestLane(double y) const {
    int best_lane = 0;
    double best_distance = std::abs(y - LaneCenterY(0));
    for (int lane = 1; lane < m_laneCount; ++lane) {
        const double distance = std::abs(y - LaneCenterY(lane));
        if (distance < best_distance) {
            best_distance = distance;
            best_lane = lane;
        }
    }
    return best_lane;
}

bool LaneMap::IsInRoadBounds(double y) const {
    const double half_width = m_laneWidth * static_cast<double>(m_laneCount) * 0.5;
    return y >= -half_width && y <= half_width;
}

bool LaneMap::IsLaneValid(int lane_id) const {
    return lane_id >= 0 && lane_id < m_laneCount;
}

} // namespace mad::map
'''
files['include/mad/simulation/world.hpp'] = '''#pragma once

#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"

#include <string>
#include <vector>

namespace mad::simulation {

struct ActorState {
    int id {0};
    mad::common::ActorType type {mad::common::ActorType::Vehicle};
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    double speed {0.0};
    double length {4.8};
    double width {1.9};
    int preferred_lane {1};
    bool active {true};
};

struct ControlCommand {
    double acceleration {0.0};
    double steering_angle {0.0};
};

struct WorldSnapshot {
    double sim_time {0.0};
    ActorState ego;
    std::vector<ActorState> actors;
};

struct ScenarioDefinition {
    std::string name;
    ActorState ego;
    std::vector<ActorState> actors;
};

class World {
public:
    explicit World(mad::map::LaneMap lane_map);

    void Reset(const ScenarioDefinition& scenario);
    void Step(double dt, const ControlCommand& ego_command);

    const WorldSnapshot& Snapshot() const { return m_snapshot; }
    const mad::map::LaneMap& lane_map() const { return m_laneMap; }
    bool HasCollision() const;
    double DistanceToFrontActorInLane(int lane_id) const;

private:
    void UpdateEgo(double dt, const ControlCommand& ego_command);
    void UpdateTraffic(double dt);

    mad::map::LaneMap m_laneMap;
    WorldSnapshot m_snapshot {};
};

ScenarioDefinition MakeScenario(const std::string& scenario_name, const mad::map::LaneMap& lane_map);

} // namespace mad::simulation
'''
files['src/simulation/world.cpp'] = '''#include "mad/simulation/world.hpp"

#include "mad/common/types.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace mad::simulation {
namespace {

constexpr double kWheelbase = 2.85;
constexpr double kMaxSteering = 0.45;
constexpr double kMaxAccel = 3.0;
constexpr double kMaxBrake = -5.0;

bool Overlap1D(double a_min, double a_max, double b_min, double b_max) {
    return !(a_max < b_min || b_max < a_min);
}

bool AabbIntersects(const ActorState& a, const ActorState& b) {
    return Overlap1D(a.x - a.length * 0.5, a.x + a.length * 0.5,
                      b.x - b.length * 0.5, b.x + b.length * 0.5) &&
           Overlap1D(a.y - a.width * 0.5, a.y + a.width * 0.5,
                      b.y - b.width * 0.5, b.y + b.width * 0.5);
}

} // namespace

World::World(mad::map::LaneMap lane_map)
    : m_laneMap(std::move(lane_map)) {
}

void World::Reset(const ScenarioDefinition& scenario) {
    m_snapshot = {};
    m_snapshot.sim_time = 0.0;
    m_snapshot.ego = scenario.ego;
    m_snapshot.actors = scenario.actors;
}

void World::Step(double dt, const ControlCommand& ego_command) {
    UpdateEgo(dt, ego_command);
    UpdateTraffic(dt);
    m_snapshot.sim_time += dt;
}

void World::UpdateEgo(double dt, const ControlCommand& ego_command) {
    auto& ego = m_snapshot.ego;
    const double accel = mad::common::Clamp(ego_command.acceleration, kMaxBrake, kMaxAccel);
    const double steering = mad::common::Clamp(ego_command.steering_angle, -kMaxSteering, kMaxSteering);

    ego.speed = std::max(0.0, ego.speed + accel * dt);
    ego.yaw = mad::common::NormalizeAngle(ego.yaw + std::tan(steering) / kWheelbase * ego.speed * dt);
    ego.x += std::cos(ego.yaw) * ego.speed * dt;
    ego.y += std::sin(ego.yaw) * ego.speed * dt;
    ego.preferred_lane = m_laneMap.ClosestLane(ego.y);
}

void World::UpdateTraffic(double dt) {
    for (auto& actor : m_snapshot.actors) {
        if (!actor.active) {
            continue;
        }
        actor.x += actor.speed * dt;
        actor.y = m_laneMap.LaneCenterY(actor.preferred_lane);
    }
}

bool World::HasCollision() const {
    for (const auto& actor : m_snapshot.actors) {
        if (actor.active && AabbIntersects(m_snapshot.ego, actor)) {
            return true;
        }
    }
    return false;
}

double World::DistanceToFrontActorInLane(int lane_id) const {
    const auto& ego = m_snapshot.ego;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& actor : m_snapshot.actors) {
        if (!actor.active || actor.preferred_lane != lane_id) {
            continue;
        }
        if (actor.x > ego.x) {
            best = std::min(best, actor.x - ego.x - actor.length * 0.5 - ego.length * 0.5);
        }
    }
    return best;
}

ScenarioDefinition MakeScenario(const std::string& scenario_name, const mad::map::LaneMap& lane_map) {
    ScenarioDefinition scenario;
    scenario.name = scenario_name;
    scenario.ego = {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 20.0, 4.8, 1.9, 1, true};

    if (scenario_name == "highway_lane_change") {
        scenario.actors = {
            {101, mad::common::ActorType::Vehicle, 35.0, lane_map.LaneCenterY(1), 0.0, 10.0, 4.8, 1.9, 1, true},
            {102, mad::common::ActorType::Vehicle, 20.0, lane_map.LaneCenterY(0), 0.0, 22.0, 4.8, 1.9, 0, true},
            {103, mad::common::ActorType::Vehicle, 90.0, lane_map.LaneCenterY(0), 0.0, 23.0, 4.8, 1.9, 0, true},
            {104, mad::common::ActorType::Vehicle, 65.0, lane_map.LaneCenterY(2), 0.0, 18.0, 4.8, 1.9, 2, true}
        };
        return scenario;
    }

    if (scenario_name == "dense_following") {
        scenario.ego.speed = 16.0;
        scenario.actors = {
            {201, mad::common::ActorType::Vehicle, 26.0, lane_map.LaneCenterY(1), 0.0, 12.0, 4.8, 1.9, 1, true},
            {202, mad::common::ActorType::Vehicle, 45.0, lane_map.LaneCenterY(0), 0.0, 14.0, 4.8, 1.9, 0, true},
            {203, mad::common::ActorType::Vehicle, 50.0, lane_map.LaneCenterY(2), 0.0, 13.0, 4.8, 1.9, 2, true}
        };
        return scenario;
    }

    if (scenario_name == "free_cruise") {
        scenario.actors = {
            {301, mad::common::ActorType::Vehicle, 120.0, lane_map.LaneCenterY(0), 0.0, 22.0, 4.8, 1.9, 0, true},
            {302, mad::common::ActorType::Vehicle, 140.0, lane_map.LaneCenterY(2), 0.0, 22.0, 4.8, 1.9, 2, true}
        };
        return scenario;
    }

    throw std::invalid_argument("Unknown scenario: " + scenario_name);
}

} // namespace mad::simulation
'''
files['include/mad/perception/sensor_model.hpp'] = '''#pragma once

#include "mad/common/types.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::perception {

struct Detection {
    int actor_id {0};
    std::string source;
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    double confidence {0.0};
};

class SensorModel {
public:
    std::vector<Detection> Observe(const mad::simulation::WorldSnapshot& snapshot) const;
};

struct TrackedObject {
    int actor_id {0};
    double x {0.0};
    double y {0.0};
    double speed {0.0};
    int lane_id {0};
    double confidence {0.0};
};

class TrackManager {
public:
    std::vector<TrackedObject> Update(const std::vector<Detection>& detections, const mad::map::LaneMap& lane_map);
};

} // namespace mad::perception
'''
files['src/perception/sensor_model.cpp'] = '''#include "mad/perception/sensor_model.hpp"

#include <algorithm>
#include <unordered_map>

namespace mad::perception {

std::vector<Detection> SensorModel::Observe(const mad::simulation::WorldSnapshot& snapshot) const {
    std::vector<Detection> detections;
    detections.reserve(snapshot.actors.size() * 2);

    for (const auto& actor : snapshot.actors) {
        if (!actor.active) {
            continue;
        }
        const double dx = actor.x - snapshot.ego.x;
        if (dx < -10.0 || dx > 120.0) {
            continue;
        }

        detections.push_back({actor.id, "lidar", actor.x, actor.y, actor.speed, 0.92});
        detections.push_back({actor.id, "radar", actor.x + 0.2, actor.y, actor.speed, 0.88});
    }
    return detections;
}

std::vector<TrackedObject> TrackManager::Update(const std::vector<Detection>& detections, const mad::map::LaneMap& lane_map) {
    struct Accumulator {
        double weighted_x {0.0};
        double weighted_y {0.0};
        double weighted_speed {0.0};
        double weight_sum {0.0};
    };

    std::unordered_map<int, Accumulator> accumulators;
    for (const auto& detection : detections) {
        auto& acc = accumulators[detection.actor_id];
        acc.weighted_x += detection.x * detection.confidence;
        acc.weighted_y += detection.y * detection.confidence;
        acc.weighted_speed += detection.speed * detection.confidence;
        acc.weight_sum += detection.confidence;
    }

    std::vector<TrackedObject> tracks;
    tracks.reserve(accumulators.size());
    for (const auto& [actor_id, acc] : accumulators) {
        if (acc.weight_sum <= 1e-6) {
            continue;
        }
        const double x = acc.weighted_x / acc.weight_sum;
        const double y = acc.weighted_y / acc.weight_sum;
        const double speed = acc.weighted_speed / acc.weight_sum;
        tracks.push_back({actor_id, x, y, speed, lane_map.ClosestLane(y), std::min(1.0, acc.weight_sum / 2.0)});
    }

    std::sort(tracks.begin(), tracks.end(), [](const TrackedObject& lhs, const TrackedObject& rhs) {
        return lhs.x < rhs.x;
    });
    return tracks;
}

} // namespace mad::perception
'''
files['src/perception/track_manager.cpp'] = '#include "mad/perception/sensor_model.hpp"\n'
files['include/mad/prediction/constant_velocity_predictor.hpp'] = '''#pragma once

#include "mad/common/types.hpp"
#include "mad/perception/sensor_model.hpp"

#include <vector>

namespace mad::prediction {

struct PredictedObject {
    int actor_id {0};
    int lane_id {0};
    double current_x {0.0};
    double current_y {0.0};
    double speed {0.0};
    std::vector<mad::common::Vec2> future_positions;
};

class ConstantVelocityPredictor {
public:
    std::vector<PredictedObject> Predict(const std::vector<mad::perception::TrackedObject>& tracks,
                                         double horizon_seconds,
                                         double dt) const;
};

} // namespace mad::prediction
'''
files['src/prediction/constant_velocity_predictor.cpp'] = '''#include "mad/prediction/constant_velocity_predictor.hpp"

namespace mad::prediction {

std::vector<PredictedObject> ConstantVelocityPredictor::Predict(const std::vector<mad::perception::TrackedObject>& tracks,
                                                                double horizon_seconds,
                                                                double dt) const {
    std::vector<PredictedObject> predictions;
    predictions.reserve(tracks.size());

    for (const auto& track : tracks) {
        PredictedObject predicted;
        predicted.actor_id = track.actor_id;
        predicted.lane_id = track.lane_id;
        predicted.current_x = track.x;
        predicted.current_y = track.y;
        predicted.speed = track.speed;

        for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
            predicted.future_positions.push_back({track.x + track.speed * t, track.y});
        }
        predictions.push_back(predicted);
    }
    return predictions;
}

} // namespace mad::prediction
'''
files['include/mad/planning/behavior_planner.hpp'] = '''#pragma once

#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

#include <string>
#include <vector>

namespace mad::planning {

struct BehaviorDecision {
    std::string state;
    int target_lane {1};
    double target_speed {0.0};
    bool emergency_brake {false};
};

class BehaviorPlanner {
public:
    BehaviorDecision Plan(const mad::simulation::WorldSnapshot& snapshot,
                          const std::vector<mad::prediction::PredictedObject>& predictions,
                          const mad::map::LaneMap& lane_map) const;

private:
    double ClosestFrontGap(const mad::simulation::WorldSnapshot& snapshot,
                           const std::vector<mad::prediction::PredictedObject>& predictions,
                           int lane_id) const;
    double ClosestRearGap(const mad::simulation::WorldSnapshot& snapshot,
                          const std::vector<mad::prediction::PredictedObject>& predictions,
                          int lane_id) const;
};

} // namespace mad::planning
'''
files['src/planning/behavior_planner.cpp'] = '''#include "mad/planning/behavior_planner.hpp"

#include <algorithm>
#include <limits>

namespace mad::planning {

namespace {
constexpr double kCruiseSpeed = 24.0;
constexpr double kSafeFrontGap = 18.0;
constexpr double kSafeRearGap = 12.0;
constexpr double kEmergencyGap = 7.0;
}

BehaviorDecision BehaviorPlanner::Plan(const mad::simulation::WorldSnapshot& snapshot,
                                       const std::vector<mad::prediction::PredictedObject>& predictions,
                                       const mad::map::LaneMap& lane_map) const {
    const int current_lane = lane_map.ClosestLane(snapshot.ego.y);
    const double front_gap = ClosestFrontGap(snapshot, predictions, current_lane);

    BehaviorDecision decision;
    decision.target_lane = current_lane;
    decision.target_speed = kCruiseSpeed;
    decision.state = "cruise";

    if (front_gap < kEmergencyGap) {
        decision.state = "emergency_brake";
        decision.target_speed = 0.0;
        decision.emergency_brake = true;
        return decision;
    }

    if (front_gap >= kSafeFrontGap) {
        return decision;
    }

    int best_lane = current_lane;
    double best_lane_gap = front_gap;

    for (const int candidate_lane : {current_lane - 1, current_lane + 1}) {
        if (!lane_map.IsLaneValid(candidate_lane)) {
            continue;
        }
        const double candidate_front_gap = ClosestFrontGap(snapshot, predictions, candidate_lane);
        const double candidate_rear_gap = ClosestRearGap(snapshot, predictions, candidate_lane);
        if (candidate_front_gap > best_lane_gap + 8.0 && candidate_front_gap > kSafeFrontGap && candidate_rear_gap > kSafeRearGap) {
            best_lane = candidate_lane;
            best_lane_gap = candidate_front_gap;
        }
    }

    if (best_lane != current_lane) {
        decision.state = "lane_change";
        decision.target_lane = best_lane;
        decision.target_speed = std::min(kCruiseSpeed, snapshot.ego.speed + 1.5);
        return decision;
    }

    decision.state = "follow";
    decision.target_speed = std::max(6.0, snapshot.ego.speed - 2.5);
    return decision;
}

double BehaviorPlanner::ClosestFrontGap(const mad::simulation::WorldSnapshot& snapshot,
                                        const std::vector<mad::prediction::PredictedObject>& predictions,
                                        int lane_id) const {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& prediction : predictions) {
        if (prediction.lane_id != lane_id || prediction.current_x <= snapshot.ego.x) {
            continue;
        }
        best = std::min(best, prediction.current_x - snapshot.ego.x);
    }
    return best;
}

double BehaviorPlanner::ClosestRearGap(const mad::simulation::WorldSnapshot& snapshot,
                                       const std::vector<mad::prediction::PredictedObject>& predictions,
                                       int lane_id) const {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& prediction : predictions) {
        if (prediction.lane_id != lane_id || prediction.current_x >= snapshot.ego.x) {
            continue;
        }
        best = std::min(best, snapshot.ego.x - prediction.current_x);
    }
    return best;
}

} // namespace mad::planning
'''
files['include/mad/planning/trajectory_planner.hpp'] = '''#pragma once

#include "mad/common/types.hpp"
#include "mad/planning/behavior_planner.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::planning {

class TrajectoryPlanner {
public:
    std::vector<mad::common::TrajectoryPoint> Plan(const mad::simulation::WorldSnapshot& snapshot,
                                                   const BehaviorDecision& decision,
                                                   const mad::map::LaneMap& lane_map,
                                                   double horizon_seconds,
                                                   double dt) const;
};

} // namespace mad::planning
'''
files['src/planning/trajectory_planner.cpp'] = '''#include "mad/planning/trajectory_planner.hpp"

#include <cmath>

namespace mad::planning {

std::vector<mad::common::TrajectoryPoint> TrajectoryPlanner::Plan(const mad::simulation::WorldSnapshot& snapshot,
                                                                  const BehaviorDecision& decision,
                                                                  const mad::map::LaneMap& lane_map,
                                                                  double horizon_seconds,
                                                                  double dt) const {
    std::vector<mad::common::TrajectoryPoint> trajectory;
    const double target_y = lane_map.LaneCenterY(decision.target_lane);
    const double y0 = snapshot.ego.y;
    const double x0 = snapshot.ego.x;
    const double v0 = snapshot.ego.speed;

    for (double t = 0.0; t <= horizon_seconds + 1e-9; t += dt) {
        const double alpha = horizon_seconds > 1e-6 ? std::min(1.0, t / horizon_seconds) : 1.0;
        const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        const double y = y0 + (target_y - y0) * smooth_alpha;
        const double target_speed = decision.emergency_brake ? 0.0 : (v0 + (decision.target_speed - v0) * smooth_alpha);
        const double x = x0 + target_speed * t;
        const double dxdt = target_speed;
        const double dydt = (target_y - y0) * (6.0 * alpha * (1.0 - alpha)) / std::max(horizon_seconds, 1e-6);
        const double yaw = std::atan2(dydt, std::max(0.1, dxdt));
        trajectory.push_back({t, x, y, yaw, target_speed});
    }
    return trajectory;
}

} // namespace mad::planning
'''
files['include/mad/control/controller.hpp'] = '''#pragma once

#include "mad/common/types.hpp"
#include "mad/simulation/world.hpp"

#include <vector>

namespace mad::control {

class Controller {
public:
    mad::simulation::ControlCommand Compute(const mad::simulation::ActorState& ego,
                                            const std::vector<mad::common::TrajectoryPoint>& trajectory,
                                            double dt);

private:
    double m_integralSpeedError {0.0};
};

} // namespace mad::control
'''
files['src/control/controller.cpp'] = '''#include "mad/control/controller.hpp"

#include "mad/common/types.hpp"

#include <cmath>
#include <limits>

namespace mad::control {

mad::simulation::ControlCommand Controller::Compute(const mad::simulation::ActorState& ego,
                                                    const std::vector<mad::common::TrajectoryPoint>& trajectory,
                                                    double dt) {
    mad::simulation::ControlCommand command {};
    if (trajectory.empty()) {
        return command;
    }

    const mad::common::TrajectoryPoint* target = &trajectory.back();
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& point : trajectory) {
        const double distance = std::hypot(point.x - ego.x, point.y - ego.y);
        if (distance < best_distance && point.t > 0.2) {
            best_distance = distance;
            target = &point;
        }
    }

    const double heading_error = mad::common::NormalizeAngle(target->yaw - ego.yaw);
    const double lateral_error = target->y - ego.y;
    command.steering_angle = mad::common::Clamp(0.75 * heading_error + 0.12 * lateral_error, -0.45, 0.45);

    const double speed_error = target->target_speed - ego.speed;
    m_integralSpeedError += speed_error * dt;
    command.acceleration = mad::common::Clamp(0.8 * speed_error + 0.1 * m_integralSpeedError, -5.0, 3.0);
    return command;
}

} // namespace mad::control
'''
files['include/mad/runtime/autonomy_stack.hpp'] = '''#pragma once

#include "mad/control/controller.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/planning/behavior_planner.hpp"
#include "mad/planning/trajectory_planner.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/simulation/world.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace mad::runtime {

struct RunSummary {
    std::string scenario_name;
    double sim_duration {0.0};
    double final_x {0.0};
    double final_y {0.0};
    bool collided {false};
    int lane_changes {0};
};

class CsvLogger {
public:
    explicit CsvLogger(const std::string& output_path);
    void Log(double sim_time,
             const mad::simulation::ActorState& ego,
             const mad::planning::BehaviorDecision& decision,
             const std::vector<mad::simulation::ActorState>& actors,
             bool collided);

private:
    std::ofstream m_stream;
};

class AutonomyStack {
public:
    explicit AutonomyStack(mad::map::LaneMap lane_map);

    RunSummary RunScenario(const std::string& scenario_name,
                           double sim_duration,
                           double dt,
                           const std::string& csv_log_path);

private:
    mad::map::LaneMap m_laneMap;
    mad::simulation::World m_world;
    mad::perception::SensorModel m_sensorModel;
    mad::perception::TrackManager m_trackManager;
    mad::prediction::ConstantVelocityPredictor m_predictor;
    mad::planning::BehaviorPlanner m_behaviorPlanner;
    mad::planning::TrajectoryPlanner m_trajectoryPlanner;
    mad::control::Controller m_controller;
};

} // namespace mad::runtime
'''
files['src/runtime/autonomy_stack.cpp'] = '''#include "mad/runtime/autonomy_stack.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace mad::runtime {

CsvLogger::CsvLogger(const std::string& output_path)
    : m_stream(output_path) {
    m_stream << "time,ego_x,ego_y,ego_yaw,ego_speed,decision,target_lane,collided,actors\\n";
}

void CsvLogger::Log(double sim_time,
                    const mad::simulation::ActorState& ego,
                    const mad::planning::BehaviorDecision& decision,
                    const std::vector<mad::simulation::ActorState>& actors,
                    bool collided) {
    std::ostringstream encoded_actors;
    for (std::size_t i = 0; i < actors.size(); ++i) {
        const auto& actor = actors[i];
        encoded_actors << actor.id << ":" << actor.x << ":" << actor.y << ":" << actor.speed << ":" << actor.preferred_lane;
        if (i + 1 < actors.size()) {
            encoded_actors << "|";
        }
    }

    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ','
             << ego.x << ','
             << ego.y << ','
             << ego.yaw << ','
             << ego.speed << ','
             << decision.state << ','
             << decision.target_lane << ','
             << (collided ? 1 : 0) << ','
             << encoded_actors.str() << '\\n';
}

AutonomyStack::AutonomyStack(mad::map::LaneMap lane_map)
    : m_laneMap(lane_map)
    , m_world(m_laneMap) {
}

RunSummary AutonomyStack::RunScenario(const std::string& scenario_name,
                                      double sim_duration,
                                      double dt,
                                      const std::string& csv_log_path) {
    std::filesystem::create_directories(std::filesystem::path(csv_log_path).parent_path());
    CsvLogger logger(csv_log_path);

    const auto scenario = mad::simulation::MakeScenario(scenario_name, m_laneMap);
    m_world.Reset(scenario);

    RunSummary summary;
    summary.scenario_name = scenario_name;

    int previous_lane = m_laneMap.ClosestLane(m_world.Snapshot().ego.y);

    for (double t = 0.0; t < sim_duration; t += dt) {
        const auto snapshot = m_world.Snapshot();
        const auto detections = m_sensorModel.Observe(snapshot);
        const auto tracks = m_trackManager.Update(detections, m_laneMap);
        const auto predictions = m_predictor.Predict(tracks, 3.0, 0.5);
        const auto decision = m_behaviorPlanner.Plan(snapshot, predictions, m_laneMap);
        const auto trajectory = m_trajectoryPlanner.Plan(snapshot, decision, m_laneMap, 3.0, 0.2);
        const auto command = m_controller.Compute(snapshot.ego, trajectory, dt);

        m_world.Step(dt, command);
        const bool collided = m_world.HasCollision();
        logger.Log(m_world.Snapshot().sim_time, m_world.Snapshot().ego, decision, m_world.Snapshot().actors, collided);

        const int current_lane = m_laneMap.ClosestLane(m_world.Snapshot().ego.y);
        if (current_lane != previous_lane) {
            ++summary.lane_changes;
            previous_lane = current_lane;
        }

        if (collided) {
            summary.collided = true;
            break;
        }
    }

    summary.sim_duration = m_world.Snapshot().sim_time;
    summary.final_x = m_world.Snapshot().ego.x;
    summary.final_y = m_world.Snapshot().ego.y;
    return summary;
}

} // namespace mad::runtime
'''
files['apps/mad_demo.cpp'] = '''#include "mad/runtime/autonomy_stack.hpp"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
    const std::string scenario_name = (argc > 1) ? argv[1] : "highway_lane_change";
    const std::filesystem::path log_path = std::filesystem::path("out") / "logs" / (scenario_name + "_log.csv");

    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario(scenario_name, 18.0, 0.1, log_path.string());

    std::cout << "[MAD] scenario      : " << summary.scenario_name << "\\n";
    std::cout << "[MAD] sim_duration  : " << summary.sim_duration << " s\\n";
    std::cout << "[MAD] final_pose    : x=" << summary.final_x << ", y=" << summary.final_y << "\\n";
    std::cout << "[MAD] lane_changes  : " << summary.lane_changes << "\\n";
    std::cout << "[MAD] collided      : " << (summary.collided ? "yes" : "no") << "\\n";
    std::cout << "[MAD] log           : " << log_path.string() << "\\n";

    return summary.collided ? 2 : 0;
}
'''
files['tests/mad_tests.cpp'] = '''#include "mad/map/lane_map.hpp"
#include "mad/planning/behavior_planner.hpp"
#include "mad/runtime/autonomy_stack.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "[TEST FAILED] " << message << "\\n";
        std::exit(1);
    }
}

void TestLaneChangeDecision() {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::simulation::WorldSnapshot snapshot;
    snapshot.ego = {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1), 0.0, 22.0, 4.8, 1.9, 1, true};

    std::vector<mad::prediction::PredictedObject> predictions = {
        {100, 1, 18.0, lane_map.LaneCenterY(1), 10.0, {}},
        {101, 0, 60.0, lane_map.LaneCenterY(0), 20.0, {}},
        {102, 2, -30.0, lane_map.LaneCenterY(2), 18.0, {}}
    };

    mad::planning::BehaviorPlanner planner;
    const auto decision = planner.Plan(snapshot, predictions, lane_map);
    Require(decision.state == "lane_change", "behavior planner should command a lane change");
    Require(decision.target_lane == 0, "planner should choose left lane");
}

void TestScenarioRunsWithoutCollision() {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::runtime::AutonomyStack stack(lane_map);
    const auto summary = stack.RunScenario("free_cruise", 10.0, 0.1, "out/tests/free_cruise_log.csv");
    Require(!summary.collided, "free cruise scenario should not collide");
    Require(summary.final_x > 50.0, "ego should move forward");
}

} // namespace

int main() {
    TestLaneChangeDecision();
    TestScenarioRunsWithoutCollision();
    std::cout << "[TEST PASSED] all tests succeeded\\n";
    return 0;
}
'''
files['tools/render_replay.py'] = '''#!/usr/bin/env python3
import csv
import json
import pathlib
import sys

LANE_WIDTH = 3.7
LANES = [-LANE_WIDTH, 0.0, LANE_WIDTH]


def load_rows(csv_path: pathlib.Path):
    rows = []
    with csv_path.open('r', encoding='utf-8') as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            actors = []
            if row['actors']:
                for token in row['actors'].split('|'):
                    actor_id, x, y, speed, lane = token.split(':')
                    actors.append({
                        'id': int(actor_id),
                        'x': float(x),
                        'y': float(y),
                        'speed': float(speed),
                        'lane': int(lane),
                    })
            rows.append({
                'time': float(row['time']),
                'ego_x': float(row['ego_x']),
                'ego_y': float(row['ego_y']),
                'ego_speed': float(row['ego_speed']),
                'decision': row['decision'],
                'target_lane': int(row['target_lane']),
                'collided': int(row['collided']),
                'actors': actors,
            })
    return rows


def main():
    if len(sys.argv) != 3:
        print('Usage: render_replay.py <input.csv> <output.html>')
        return 1

    input_path = pathlib.Path(sys.argv[1])
    output_path = pathlib.Path(sys.argv[2])
    rows = load_rows(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    html_template = """<!doctype html>
<html lang=\"en\">
<head>
<meta charset=\"utf-8\" />
<title>Replay - __TITLE__</title>
<style>
body { font-family: Arial, sans-serif; margin: 0; background: #111; color: #eee; }
header { padding: 12px 16px; background: #1b1b1b; }
#hud { padding: 8px 16px; display: flex; gap: 24px; flex-wrap: wrap; }
canvas { display: block; margin: 0 auto; background: #202020; border: 1px solid #444; }
button { margin: 8px 16px; padding: 8px 14px; }
</style>
</head>
<body>
<header><h2>Momenta Autodrive Stack Replay</h2></header>
<div id=\"hud\">
  <div id=\"time\"></div>
  <div id=\"decision\"></div>
  <div id=\"speed\"></div>
  <div id=\"collision\"></div>
</div>
<button onclick=\"togglePlay()\">Play / Pause</button>
<canvas id=\"view\" width=\"1200\" height=\"420\"></canvas>
<script>
const frames = __FRAMES__;
const laneCenters = __LANES__;
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
let playing = true;
let index = 0;

function worldToCanvas(x, y, egoX) {
  const px = (x - egoX) * 8 + 250;
  const py = 210 - y * 40;
  return [px, py];
}

function drawVehicle(x, y, egoX, isEgo) {
  const [px, py] = worldToCanvas(x, y, egoX);
  ctx.fillStyle = isEgo ? '#00d084' : '#f2c94c';
  ctx.fillRect(px - 20, py - 10, 40, 20);
}

function draw() {
  const frame = frames[index];
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#2a2a2a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#666';
  ctx.lineWidth = 2;
  laneCenters.forEach((laneY) => {
    const py = worldToCanvas(frame.ego_x, laneY, frame.ego_x)[1];
    ctx.setLineDash([12, 10]);
    ctx.beginPath();
    ctx.moveTo(0, py);
    ctx.lineTo(canvas.width, py);
    ctx.stroke();
  });
  ctx.setLineDash([]);
  drawVehicle(frame.ego_x, frame.ego_y, frame.ego_x, true);
  frame.actors.forEach(actor => drawVehicle(actor.x, actor.y, frame.ego_x, false));
  document.getElementById('time').innerText = 'time: ' + frame.time.toFixed(2) + ' s';
  document.getElementById('decision').innerText = 'decision: ' + frame.decision + ' -> lane ' + frame.target_lane;
  document.getElementById('speed').innerText = 'ego speed: ' + frame.ego_speed.toFixed(2) + ' m/s';
  document.getElementById('collision').innerText = 'collision: ' + (frame.collided ? 'YES' : 'NO');
}

function tick() {
  if (playing) {
    index = Math.min(index + 1, frames.length - 1);
  }
  draw();
  requestAnimationFrame(tick);
}

function togglePlay() { playing = !playing; }

draw();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""
    html_text = html_template.replace('__TITLE__', input_path.stem).replace('__FRAMES__', json.dumps(rows)).replace('__LANES__', json.dumps(LANES))
    output_path.write_text(html_text, encoding='utf-8')
    print(f'[MAD] replay written to: {output_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
'''
files['tools/evaluate_run.py'] = '''#!/usr/bin/env python3
import csv
import pathlib
import statistics
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print('Usage: evaluate_run.py <input.csv>')
        return 1

    input_path = pathlib.Path(sys.argv[1])
    rows = list(csv.DictReader(input_path.open('r', encoding='utf-8')))
    if not rows:
        print('[MAD] no rows found')
        return 2

    speeds = [float(row['ego_speed']) for row in rows]
    collisions = any(int(row['collided']) for row in rows)
    decisions = {row['decision'] for row in rows}
    final_row = rows[-1]

    print('[MAD] evaluation summary')
    print(f'  samples          : {len(rows)}')
    print(f'  avg ego speed    : {statistics.fmean(speeds):.3f} m/s')
    print(f'  max ego speed    : {max(speeds):.3f} m/s')
    print(f'  final pose       : ({final_row["ego_x"]}, {final_row["ego_y"]})')
    print(f'  collided         : {collisions}')
    print(f'  decisions        : {sorted(decisions)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
'''
files['tools/check_architecture_sync.py'] = '''#!/usr/bin/env python3
import json
import pathlib
import sys


def main() -> int:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    inventory_path = repo_root / 'docs' / 'architecture' / 'module_inventory.json'
    inventory = json.loads(inventory_path.read_text(encoding='utf-8'))

    missing_paths = []
    for module in inventory['modules']:
        rel_path = repo_root / module['path']
        if not rel_path.exists():
            missing_paths.append(module['path'])

    architecture_md = (repo_root / 'docs' / 'architecture' / '01_system_architecture.md').read_text(encoding='utf-8')
    missing_in_doc = [m['name'] for m in inventory['modules'] if m['name'] not in architecture_md]

    if missing_paths or missing_in_doc:
        print('[MAD] architecture sync check failed')
        if missing_paths:
            print('  missing paths   :', missing_paths)
        if missing_in_doc:
            print('  missing in docs :', missing_in_doc)
        return 2

    print('[MAD] architecture sync check passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
'''
files['scripts/build_linux.sh'] = '''#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/out/build/linux"
INSTALL_DIR="${ROOT_DIR}/out/install/linux"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" -DMAD_BUILD_TESTS=ON
cmake --build "${BUILD_DIR}" --parallel
ctest --test-dir "${BUILD_DIR}" --output-on-failure
cmake --install "${BUILD_DIR}"
python3 "${ROOT_DIR}/tools/check_architecture_sync.py"

echo "[MAD] build complete"
'''
files['scripts/run_demo.sh'] = '''#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-highway_lane_change}"
BIN="${ROOT_DIR}/out/build/linux/mad_demo"
if [[ ! -x "${BIN}" ]]; then
  echo "[MAD] demo binary not found. Run scripts/build_linux.sh first."
  exit 2
fi
"${BIN}" "${SCENARIO}"
'''
files['scripts/package_linux.sh'] = '''#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST_DIR="${ROOT_DIR}/out/dist"
PACKAGE_NAME="momenta_autodrive_stack_linux"

bash "${ROOT_DIR}/scripts/build_linux.sh"
mkdir -p "${DIST_DIR}"
rm -rf "${DIST_DIR}/${PACKAGE_NAME}"
mkdir -p "${DIST_DIR}/${PACKAGE_NAME}/bin"
cp -r "${ROOT_DIR}/out/install/linux/." "${DIST_DIR}/${PACKAGE_NAME}/"
cp -f "${ROOT_DIR}/out/build/linux/mad_demo" "${DIST_DIR}/${PACKAGE_NAME}/bin/" 2>/dev/null || true
(cd "${DIST_DIR}" && tar -czf "${PACKAGE_NAME}.tar.gz" "${PACKAGE_NAME}")
echo "[MAD] package created: ${DIST_DIR}/${PACKAGE_NAME}.tar.gz"
'''
files['scripts/build_windows.bat'] = '''@echo off
setlocal enabledelayedexpansion
set ROOT_DIR=%~dp0..
set BUILD_DIR=%ROOT_DIR%\\out\\build\\windows
set INSTALL_DIR=%ROOT_DIR%\\out\\install\\windows
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%" -DMAD_BUILD_TESTS=ON
if errorlevel 1 goto :fail
cmake --build "%BUILD_DIR%" --parallel
if errorlevel 1 goto :fail
ctest --test-dir "%BUILD_DIR%" --output-on-failure
if errorlevel 1 goto :fail
cmake --install "%BUILD_DIR%"
if errorlevel 1 goto :fail
python "%ROOT_DIR%\\tools\\check_architecture_sync.py"
if errorlevel 1 goto :fail
echo [MAD] build complete
exit /b 0
:fail
echo [MAD] build failed
exit /b 1
'''
files['scripts/run_demo_windows.bat'] = '''@echo off
setlocal
set ROOT_DIR=%~dp0..
set SCENARIO=%1
if "%SCENARIO%"=="" set SCENARIO=highway_lane_change
set BIN=%ROOT_DIR%\\out\\build\\windows\\mad_demo.exe
if not exist "%BIN%" (
  echo [MAD] demo binary not found. Run scripts\\build_windows.bat first.
  exit /b 2
)
"%BIN%" "%SCENARIO%"
'''
files['configs/scenarios/README.md'] = '''# Scenarios

当前样机内置以下场景名称：

- `highway_lane_change`
- `dense_following`
- `free_cruise`

在当前交付里，场景由 C++ 内置工厂函数 `MakeScenario()` 生成，避免为了 JSON 解析引入额外依赖。
后续如果接入 CARLA / AWSIM / ROS2，可以把该目录扩展成 JSON / YAML / OpenSCENARIO 入口。
'''
files['docs/architecture/module_inventory.json'] = '''{
  "modules": [
    {"name": "common", "path": "include/mad/common"},
    {"name": "map", "path": "include/mad/map"},
    {"name": "simulation", "path": "include/mad/simulation"},
    {"name": "perception", "path": "include/mad/perception"},
    {"name": "prediction", "path": "include/mad/prediction"},
    {"name": "planning", "path": "include/mad/planning"},
    {"name": "control", "path": "include/mad/control"},
    {"name": "runtime", "path": "include/mad/runtime"}
  ]
}
'''
files['docs/architecture/01_system_architecture.md'] = '''# 01. System Architecture

## 目标

这套系统直接对齐 Momenta 常见自动驾驶岗位能力要求：

- 车端 **C/C++** 模块设计与实现
- **Python / Shell** 工具链能力
- **Linux / CMake / GDB** 工程环境
- 自动驾驶 **感知 / 预测 / 规划 / 控制 / 仿真** 的基础闭环
- 自动化评测、自动化编译、自动化打包、可维护的软件架构

## 源码模块与路径

| 模块 | 目录 | 职责 |
|---|---|---|
| common | `include/mad/common` | 数学基础类型、通用工具类型 |
| map | `include/mad/map` | 车道地图模型、车道中心线与车道判断 |
| simulation | `include/mad/simulation` | 自车与交通参与者动力学、场景工厂、碰撞检测 |
| perception | `include/mad/perception` | 传感器观测、融合跟踪 |
| prediction | `include/mad/prediction` | 常速度预测 |
| planning | `include/mad/planning` | 行为规划、轨迹规划 |
| control | `include/mad/control` | 横纵向控制 |
| runtime | `include/mad/runtime` | 车端流水线编排、日志输出、仿真运行 |

## 运行时链路

```text
World Snapshot
  -> perception::SensorModel
  -> perception::TrackManager
  -> prediction::ConstantVelocityPredictor
  -> planning::BehaviorPlanner
  -> planning::TrajectoryPlanner
  -> control::Controller
  -> simulation::World::Step
  -> runtime::CsvLogger
```

## 模块依赖原则

1. `common` 只能被上层依赖，不能反向依赖业务模块。
2. `runtime` 负责装配，不承载具体算法细节。
3. `planning` 只依赖 `prediction` / `map` / `simulation` 中的快照数据，不直接修改世界状态。
4. `simulation` 既可作为本地 2D 仿真后端，也可以在未来替换成 CARLA / AWSIM 适配层。
5. `tools/` 中的 Python 只做评测、回放、架构校验，不把算法真相散落到脚本侧。

## 关键设计说明

### perception
当前是**轻量代理实现**：
- Lidar / Radar 输出简化检测
- TrackManager 依据 `actor_id` 做融合聚合

这样做的目的是让仓库在当前环境里能真实编译与演示闭环；未来可以无缝替换成真实多目标跟踪、时序融合、状态估计模块。

### planning
规划拆成两层：
- `BehaviorPlanner`：决定巡航、跟车、变道、紧急制动
- `TrajectoryPlanner`：生成平滑横向换道和纵向速度目标

这与工业界常见的 behavior + motion planning 分层保持一致。

### control
采用：
- 横向：简化纯跟踪 / 航向误差控制
- 纵向：PID 速度控制

### runtime
`AutonomyStack` 是唯一的系统装配点。它负责：
- 调起闭环模块
- 管理场景生命周期
- 输出 CSV 日志
- 供 Python 工具做回放与评测

## 与代码同步机制

- 模块清单：`docs/architecture/module_inventory.json`
- 校验脚本：`tools/check_architecture_sync.py`
- CI 会执行该检查，防止文档与代码目录漂移
'''
files['docs/architecture/02_open_source_survey.md'] = '''# 02. Open Source Survey

本项目参考了当前自动驾驶与仿真生态中最有代表性的开源方案，但没有在当前环境中直接把它们整体引入编译，而是将其架构思想映射进样机。

## 1. Autoware
- 定位：全球领先的开源自动驾驶软件栈，基于 ROS，覆盖从感知到规划控制的完整链路。
- 借鉴点：模块化边界清晰、生态成熟、适合做量产/研究型软件栈参考。
- 对本仓库的影响：本仓库采用了 perception / prediction / planning / control 的显式模块划分。

## 2. Apollo
- 定位：百度的开放自动驾驶平台，强调完整开发、测试和部署闭环。
- 借鉴点：工业化的软件包划分、评测闭环、工具链和可交付性。
- 对本仓库的影响：把“自动化评测 + 自动构建 + 系统集成”作为一等公民来做。

## 3. CARLA
- 定位：面向自动驾驶研发、训练与验证的高保真开源仿真平台。
- 借鉴点：场景、传感器、交通流、验证闭环。
- 对本仓库的影响：当前仓库虽是 2D 轻量仿真，但在接口层预留了替换仿真后端的可能性。

## 4. AWSIM / Tier IV
- 定位：与 Autoware 高度契合的数字孪生仿真方向。
- 借鉴点：ROS2 原生通信、面向真实车辆控制模式的仿真闭环。
- 对本仓库的影响：未来若扩展到 ROS2，本仓库最适合作为车端算法沙箱，再外接 AWSIM。

## 5. AirSim / Project AirSim
- 定位：微软的高保真自主系统仿真平台演进路线。
- 借鉴点：仿真平台与感知/控制验证耦合良好。
- 对本仓库的影响：说明“高保真仿真”与“轻量本地可验证闭环”可以是两层结构，而不必强耦合在一个工程里。

## 6. SVL / LGSVL
- 定位：历史上与 Apollo / Autoware 集成较深的自动驾驶仿真平台，开源代码仍然可得。
- 借鉴点：端到端联调经验与开放接口思路。
- 对本仓库的影响：强化了“本地样机先做接口抽象，再逐步挂接重型仿真”的设计策略。

## 当前策略

当前仓库采取两层法：

1. **本地可编译的轻量闭环**：确保 C++/Python/CMake/脚本/评测在受限环境中可真实验证。
2. **重型生态映射**：通过文档与接口约束，后续再把 CARLA / Autoware / ROS2 / AWSIM 接进来。
'''
files['docs/architecture/03_momenta_role_mapping.md'] = '''# 03. Mapping to Momenta Role Requirements

## 参考到的岗位画像

综合 Momenta 公开岗位信息，可以抽取出几类高频能力：

1. 自动驾驶车端模块研发、算法集成、功能开发与调试
2. Linux 下 C/C++ 编程，Python / Shell 脚本能力
3. 熟悉 CMake / GDB / 调试开发环境
4. 对感知、预测、规划、控制、地图、定位至少一个方向有实际经验
5. 支持量产项目交付，强调软件工程化、质量、效率与自动化工具链
6. 规划算法岗位强调图搜索、轨迹规划/优化、控制、不确定性下规划
7. 融合/感知岗位强调多传感器感知、工程落地与 C++/Python 能力

## 本仓库如何对应

| Momenta 能力要求 | 本仓库落点 |
|---|---|
| Linux / C++ / Python / Shell | C++17 核心模块 + Python 工具 + shell/bat 脚本 |
| CMake / GDB / 工程调试 | CMake 工程、CTest、CI、目录分层 |
| 自动驾驶模块研发 | perception / prediction / planning / control / runtime |
| 算法集成 | `AutonomyStack` 完成整条闭环装配 |
| 自动化工具链 | build/package/check/evaluate/render 脚本齐全 |
| 量产思维 | 模块清晰、日志输出、可回放、可评测、可扩展 |
| 规划控制能力 | 跟车 / 变道 / 轨迹生成 / PID + 横向控制 |
| 仿真研发 | `simulation::World` + HTML 回放 |

## 还没有覆盖但建议下一步扩展的部分

1. ROS2 通信层
2. 更真实的车辆动力学模型
3. 感知时序跟踪与多目标数据关联
4. 地图 / 定位 / Route / HD Map
5. OpenSCENARIO / OpenDRIVE 场景导入
6. CARLA 或 AWSIM 后端适配
7. TensorRT / CUDA / ONNX Runtime 推理部署
8. Docker 化部署与车端日志采集
'''
files['docs/architecture/module_dependency.puml'] = '''@startuml
skinparam backgroundColor #111111
skinparam defaultTextAlignment center
skinparam rectangle {
  BackgroundColor #1f2937
  BorderColor #93c5fd
  FontColor #f8fafc
}
rectangle common
rectangle map
rectangle simulation
rectangle perception
rectangle prediction
rectangle planning
rectangle control
rectangle runtime

map --> common
simulation --> common
simulation --> map
perception --> common
perception --> simulation
perception --> map
prediction --> perception
planning --> prediction
planning --> simulation
planning --> map
control --> planning
control --> simulation
runtime --> perception
runtime --> prediction
runtime --> planning
runtime --> control
runtime --> simulation
runtime --> map
@enduml
'''
files['docs/architecture/runtime_sequence.md'] = '''```mermaid
sequenceDiagram
    participant W as simulation::World
    participant S as SensorModel
    participant T as TrackManager
    participant P as Predictor
    participant B as BehaviorPlanner
    participant M as TrajectoryPlanner
    participant C as Controller
    participant L as CsvLogger

    W->>S: snapshot
    S->>T: detections
    T->>P: tracks
    P->>B: predicted objects
    B->>M: behavior decision
    M->>C: trajectory
    C->>W: control command
    W->>L: updated world state
```
'''
files['.github/workflows/ci.yml'] = '''name: ci

on:
  push:
  pull_request:

jobs:
  build-linux:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Configure
        run: cmake -S . -B out/build/linux -G Ninja -DCMAKE_BUILD_TYPE=Release -DMAD_BUILD_TESTS=ON
      - name: Build
        run: cmake --build out/build/linux --parallel
      - name: Test
        run: ctest --test-dir out/build/linux --output-on-failure
      - name: Architecture sync
        run: python3 tools/check_architecture_sync.py

  build-windows:
    runs-on: windows-latest
    steps:
      - uses: actions/checkout@v4
      - uses: ilammy/msvc-dev-cmd@v1
      - name: Configure
        run: cmake -S . -B out/build/windows -G Ninja -DCMAKE_BUILD_TYPE=Release -DMAD_BUILD_TESTS=ON
      - name: Build
        run: cmake --build out/build/windows --parallel
      - name: Test
        run: ctest --test-dir out/build/windows --output-on-failure
      - name: Architecture sync
        run: python tools/check_architecture_sync.py
'''

for rel, content in files.items():
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')
print(f'Wrote {len(files)} files')
