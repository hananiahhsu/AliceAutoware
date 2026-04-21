cmake_minimum_required(VERSION 3.20)
project(MomentaAutodriveStack VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MAD_BUILD_TESTS "Build tests" ON)

include(GNUInstallDirs)

set(MAD_CORE_SOURCES
    src/map/lane_corridor_builder.cpp
    src/planning/corridor_cost_evaluator.cpp
    src/prediction/multi_agent_behavior_predictor.cpp
    src/safety/rss_longitudinal_checker.cpp
    src/safety/rss_lateral_checker.cpp
    src/control/feedforward_controller.cpp
    src/runtime/diagnostics_monitor.cpp
    src/runtime/fallback_manager.cpp
    src/localization/localization_filter.cpp
    src/map/lane_map.cpp
    src/map/route_planner.cpp
    src/map/semantic_map.cpp
    src/simulation/world.cpp
    src/simulation/scenario_catalog.cpp
    src/simulation/scenario_loader.cpp
    src/simulation/simulator_adapter.cpp
    src/simulation/external_sim_bridge.cpp
    src/simulation/ros2_message_schema.cpp
    src/simulation/ros2_bridge_stub.cpp
    src/simulation/carla_bridge_stub.cpp
    src/simulation/task_bridge_stub.cpp
    src/perception/sensor_model.cpp
    src/perception/track_manager.cpp
    src/perception/fusion_engine.cpp
    src/perception/occupancy_grid.cpp
    src/perception/lane_occupancy_analyzer.cpp
    src/prediction/intention_predictor.cpp
    src/prediction/constant_velocity_predictor.cpp
    src/prediction/interaction_predictor.cpp
    src/prediction/scene_risk_assessor.cpp
    src/prediction/risk_object_ranker.cpp
    src/prediction/trajectory_hypothesis_generator.cpp
    src/prediction/joint_interaction_predictor.cpp
    src/planning/behavior_fsm.cpp
    src/planning/behavior_tree.cpp
    src/planning/task_tree.cpp
    src/planning/behavior_planner.cpp
    src/planning/gap_acceptance_evaluator.cpp
    src/planning/lane_sequence_planner.cpp
    src/planning/lane_change_manager.cpp
    src/planning/speed_planner.cpp
    src/planning/trajectory_stitcher.cpp
    src/planning/trajectory_planner.cpp
    src/control/gain_scheduler.cpp
    src/control/lateral_controller.cpp
    src/control/longitudinal_controller.cpp
    src/control/controller.cpp
    src/runtime/evaluator.cpp
    src/runtime/mission_manager.cpp
    src/runtime/event_logger.cpp
    src/runtime/decision_trace_writer.cpp
    src/runtime/risk_trace_writer.cpp
    src/runtime/hypothesis_trace_writer.cpp
    src/runtime/interaction_trace_writer.cpp
    src/safety/safety_guardian.cpp
    src/safety/rss_lane_change_checker.cpp
    src/safety/safety_supervisor.cpp
    src/runtime/report_writer.cpp
    src/runtime/autonomy_stack.cpp
    src/visualization/dashboard_renderer.cpp
)

add_library(mad_core ${MAD_CORE_SOURCES})
target_include_directories(mad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_options(mad_core PRIVATE /W4)
else()
    target_compile_options(mad_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(mad_demo apps/mad_demo.cpp)
target_link_libraries(mad_demo PRIVATE mad_core)

add_executable(mad_batch_eval apps/mad_batch_eval.cpp)
target_link_libraries(mad_batch_eval PRIVATE mad_core)

add_executable(mad_visualizer apps/mad_visualizer.cpp)
target_link_libraries(mad_visualizer PRIVATE mad_core)

if(MAD_BUILD_TESTS)
    enable_testing()
    add_executable(mad_tests
        tests/test_main.cpp
        tests/test_framework.cpp
        tests/test_prediction.cpp
        tests/test_interaction.cpp
        tests/test_control.cpp
        tests/test_planning.cpp
        tests/test_safety.cpp
        tests/test_runtime.cpp
        tests/test_scenarios.cpp
        tests/test_map.cpp
        tests/test_control_advanced.cpp
        tests/test_prediction_advanced.cpp
        tests/test_safety_rss.cpp
        tests/test_runtime_diagnostics.cpp
        tests/test_corridor.cpp
        tests/test_fallback.cpp
    )
    target_link_libraries(mad_tests PRIVATE mad_core)
    add_test(NAME mad_tests COMMAND mad_tests)
endif()

install(TARGETS mad_demo mad_batch_eval mad_visualizer
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY configs DESTINATION .)
install(DIRECTORY docs DESTINATION .)
install(DIRECTORY tools DESTINATION .)
install(DIRECTORY scripts DESTINATION .)
install(FILES README.md DESTINATION .)
