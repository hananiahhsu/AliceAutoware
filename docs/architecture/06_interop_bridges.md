# Interop Bridges v7

## Purpose

v7 keeps bridge code as stubs that are easy to replace with real middleware adapters later.

## Files

- `simulation.external_bridge`: writes deterministic state / command CSV pairs.
- `simulation.ros2_message_schema`: writes topic schema definitions to `out/interop/ros2_message_schema_v7.txt`.
- `simulation.ros2_bridge_stub`: emits topic-like CSV rows.
- `simulation.task_bridge_stub`: emits task-tree directive rows.
- `simulation.carla_bridge_stub`: emits CARLA-style world frame summaries.

## Topic Coverage

`simulation.ros2_message_schema` documents these topic families:
- `/mad/localization/ego_state`
- `/mad/prediction/scene_risk`
- `/mad/planning/behavior_tree`
- `/mad/runtime/task_tree`
- `/mad/control/command`

## Upgrade Path

The intended path is:
1. Replace CSV writers with typed ROS2 publishers.
2. Map `simulation.carla_bridge_stub` to a real CARLA world adapter.
3. Keep CSV fallback for deterministic regression and CI artifact inspection.
