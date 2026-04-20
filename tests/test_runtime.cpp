#include "test_framework.hpp"

#include "mad/localization/localization_filter.hpp"
#include "mad/map/lane_map.hpp"
#include "mad/runtime/mission_manager.hpp"
#include "mad/simulation/ros2_message_schema.hpp"
#include "mad/simulation/scenario_catalog.hpp"

#include <filesystem>
#include <fstream>

MAD_TEST(Runtime, LocalizationMovesTowardTruth) {
    mad::map::LaneMap lane_map(3.7, 3, 500.0);
    mad::localization::LocalizationFilter filter(lane_map);
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, lane_map.LaneCenterY(1) + 0.45, 0.0, 20.0, 4.8, 1.9, 1, true};
    filter.Reset(ego);
    filter.Predict(0.1, {0.0, 0.0});
    filter.CorrectWithTruth(ego);
    MAD_REQUIRE(std::abs(filter.state().y - lane_map.LaneCenterY(1)) < 0.5);
}

MAD_TEST(Runtime, MissionManagerTransitionsToEmergencyStop) {
    mad::runtime::MissionManager manager;
    manager.Reset();
    (void)manager.Step({false, false, false, 10.0, 0.0, 100.0});
    const auto decision = manager.Step({false, true, false, 8.0, 10.0, 100.0});
    MAD_REQUIRE(decision.state == mad::runtime::MissionState::EmergencyStop);
}

MAD_TEST(Runtime, Ros2SchemaWriterIncludesTaskAndControlTopics) {
    std::filesystem::create_directories("out/tests");
    const std::string path = "out/tests/ros2_schema_v9.txt";
    mad::simulation::Ros2MessageSchemaWriter writer(path);
    writer.WriteDefaultSchemas();
    std::ifstream input(path);
    std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    MAD_REQUIRE(content.find("/mad/runtime/task_tree") != std::string::npos);
    MAD_REQUIRE(content.find("/mad/control/command") != std::string::npos);
}

MAD_TEST(Runtime, ScenarioCatalogIncludesV11Scenarios) {
    const auto scenarios = mad::simulation::AllScenarioNames();
    MAD_REQUIRE(scenarios.size() >= 16);
    bool saw_cooperative = false;
    bool saw_blockage = false;
    bool saw_weave = false;
    bool saw_truck = false;
    bool saw_corridor = false;
    for (const auto& scenario : scenarios) {
        if (scenario == "cooperative_lane_change") saw_cooperative = true;
        if (scenario == "sudden_lane_blockage") saw_blockage = true;
        if (scenario == "multi_interaction_weave") saw_weave = true;
        if (scenario == "truck_cut_in_brake") saw_truck = true;
        if (scenario == "corridor_blocked_recovery") saw_corridor = true;
    }
    MAD_REQUIRE(saw_cooperative);
    MAD_REQUIRE(saw_blockage);
    MAD_REQUIRE(saw_weave);
    MAD_REQUIRE(saw_truck);
    MAD_REQUIRE(saw_corridor);
}
