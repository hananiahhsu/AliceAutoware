# Momenta Autodrive Stack v8 System Architecture

## Overview

v8 keeps the stable v7 closed loop and adds four harder layers that make the stack more like a real ADS development baseline:

1. `prediction.trajectory_hypothesis_generator` for multi-hypothesis object motion reasoning.
2. `planning.gap_acceptance_evaluator` for explainable lane-change admission control.
3. `safety.supervisor` for RSS-style longitudinal safety and lane-change veto.
4. `runtime.hypothesis_trace_writer` plus a modular unit-test tree for stronger validation and observability.

The runtime order is:

`localization -> map -> perception.sensor_model -> perception.fusion_engine -> perception.track_manager -> perception.occupancy_grid -> perception.lane_occupancy_analyzer -> prediction.intention -> prediction.constant_velocity -> prediction.trajectory_hypothesis_generator -> prediction.scene_risk_assessor -> prediction.risk_object_ranker -> planning.behavior_tree -> planning.task_tree -> planning.behavior_planner -> planning.gap_acceptance_evaluator -> planning.speed_planner -> planning.trajectory_stitcher -> planning.trajectory_planner -> safety.supervisor -> control.lateral/control.longitudinal -> safety.guardian -> simulation.external_bridge/simulation.ros2_bridge_stub/simulation.carla_bridge_stub/simulation.task_bridge_stub -> runtime.evaluator/runtime.event_logger/runtime.decision_trace_writer/runtime.risk_trace_writer/runtime.hypothesis_trace_writer`

## Module Catalog

- `localization`: ego pose smoothing and lane association.
- `map`: route lane sequence construction.
- `map.semantic`: lane policy, lane preference, speed recommendation.
- `simulation.world`: world evolution, scripted traffic behavior, collision checks.
- `simulation.adapter`: built-in simulator abstraction.
- `simulation.loader`: external scenario CSV loader with parent-directory probing for test/build trees.
- `simulation.external_bridge`: state / command CSV export for offline inspection.
- `simulation.ros2_message_schema`: ROS2 topic schema stub emission.
- `simulation.ros2_bridge_stub`: topic-style CSV emission for ego state / risk / planning / control.
- `simulation.carla_bridge_stub`: CARLA-style frame export scaffold.
- `simulation.task_bridge_stub`: task tree decision export channel.
- `perception.sensor_model`: synthetic sensor detections from world truth.
- `perception.fusion_engine`: source merge into fused objects.
- `perception.track_manager`: temporal track memory, lane offset and velocity estimation.
- `perception.occupancy_grid`: nearest front/rear object summary per lane.
- `perception.lane_occupancy_analyzer`: lane density, closing speed, congestion and route blocked state.
- `prediction.intention`: cut-in tendency estimation.
- `prediction.constant_velocity`: horizon rollout with lateral intent blending.
- `prediction.interaction`: pairwise prediction helper retained for future extension.
- `prediction.scene_risk_assessor`: lane-level risk and preferred lane selection.
- `prediction.risk_object_ranker`: actor-level risk ordering and conflict labeling.
- `prediction.trajectory_hypothesis_generator`: keep-lane / lane-change hypothesis generation with conflict-time estimation.
- `planning.behavior_fsm`: low-level execution state machine.
- `planning.lane_change_manager`: lane-change commitment and anti-oscillation handling.
- `planning.behavior_planner`: route-aware tactical behavior planner.
- `planning.behavior_tree`: high-level reactive BT directive layer.
- `planning.task_tree`: mission/tactical directive layer above BT for overtake, stabilize, keep-lane or hard brake biasing.
- `planning.gap_acceptance_evaluator`: target-lane entry scoring with front/rear gap and hypothesis conflict penalties.
- `planning.speed_planner`: longitudinal profile synthesis.
- `planning.trajectory_stitcher`: trajectory warm start.
- `planning.trajectory_planner`: path geometry generation.
- `control.lateral`: steering command generation.
- `control.longitudinal`: acceleration command generation.
- `safety.guardian`: low-level AEB and lane-hold override.
- `safety.supervisor`: high-level RSS-like safety supervision, lane-change veto and speed clamp.
- `runtime.mission_manager`: standby/driving/yielding/emergency_stop/completed states.
- `runtime.autonomy_stack`: full pipeline orchestration.
- `runtime.evaluator`: metric aggregation.
- `runtime.event_logger`: event stream logging.
- `runtime.decision_trace_writer`: planner + BT + task-tree trace emission.
- `runtime.risk_trace_writer`: top-risk object and lane-flow trace emission.
- `runtime.hypothesis_trace_writer`: per-actor hypothesis trace emission.
- `runtime.report_writer`: batch markdown report output.

## v8 Safety and Tactical Maturity

v8 inserts a new decision fence between tactical desire and trajectory execution:

- `planning.behavior_tree` decides whether the situation wants cruise/follow/yield/change/emergency behavior.
- `planning.task_tree` biases the solution toward overtake, stabilize, keep-lane or fallback.
- `planning.gap_acceptance_evaluator` decides whether the requested lane change is still dynamically admissible.
- `safety.supervisor` applies a higher-level veto or RSS-style speed clamp before low-level control.
- `safety.guardian` remains the final local AEB / hold-lane layer.

This layering reduces oscillation, rejects unsafe merge windows, and produces more explainable lane-change behavior.

## v8 Validation Baseline

v8 no longer keeps all tests in one source file. The test tree is split into:

- `tests/test_prediction.cpp`
- `tests/test_planning.cpp`
- `tests/test_safety.cpp`
- `tests/test_runtime.cpp`
- `tests/test_scenarios.cpp`
- `tests/test_framework.*`

The automated validation now covers:

- prediction intent and hypothesis generation
- planning state and gap acceptance
- RSS-like supervisor behavior
- runtime state transitions and schema generation
- scenario-level non-collision regression

## Scenarios

The v8 batch suite contains 13 scenarios, adding:

- `cooperative_lane_change`
- `sudden_lane_blockage`

## Build Baseline

- C++17
- CMake
- Linux build validated in this delivery
- automated tests enabled with `ctest`


## v9 platform upgrades

- Added `prediction.joint_interaction_predictor` for pairwise / multi-actor merge conflicts.
- Added `planning.lane_sequence_planner` for route-aligned tactical lane selection.
- Added `safety.rss_lane_change_checker` as an explicit RSS-style lane-change gate.
- Added `control.gain_scheduler` to make control less fixed-gain and more speed-aware.
- Added `runtime.interaction_trace_writer` and expanded evaluation outputs.

- `control.controller` remains the final user-space control assembly that consumes gain scheduling and trajectory targets.


## v10 additions

- lane corridor builder
- multi-agent behavior predictor
- RSS longitudinal checker
- RSS lateral checker
- feedforward controller
- diagnostics monitor
- expanded generated message/schema/test assets


## v10 modules (exact inventory names)

- map.lane_corridor_builder
- prediction.multi_agent_behavior_predictor
- safety.rss_longitudinal_checker
- safety.rss_lateral_checker
- control.feedforward_controller
- runtime.diagnostics_monitor


## v11 additions

- planning.corridor_cost_evaluator: evaluates candidate lane corridors against lane-flow congestion and ranked risks, then feeds tactical lane selection.
- runtime.fallback_manager: accumulates repeated safety stress and escalates into keep-lane fallback or minimal-risk stop.
- scenario corridor_blocked_recovery: validates blocked-lane recovery without collision.
