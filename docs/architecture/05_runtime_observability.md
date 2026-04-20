# Runtime Observability v8

v8 writes the following runtime artifacts per scenario:

- `out/logs/*.csv`: primary closed-loop log
- `out/events/*.csv`: discrete event log
- `out/decision_traces/*.csv`: planner / BT / task-tree trace
- `out/risk_traces/*.csv`: ranked risk objects and lane-flow trace
- `out/hypothesis_traces/*.csv`: multi-hypothesis prediction trace
- `out/bridge/*_state.csv` and `*_command.csv`: external bridge export
- `out/interop/*_ros2_topics.csv`: ROS2 stub messages
- `out/interop/*_carla_bridge.csv`: CARLA bridge stub export
- `out/interop/*_task_bridge.csv`: task-layer bridge export

## New v8 Signals

Compared with v7, v8 adds explicit visibility into:

- lane-change gap rejection
- safety supervisor intervention counts
- per-object hypothesis probabilities
- earliest conflict time per hypothesis

This makes it easier to understand why a lane change was refused, why speed was clamped, and which predicted actor motion caused the intervention.
