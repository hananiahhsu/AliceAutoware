# Risk Ranking and Task Tree v7

## New modules

- `prediction.risk_object_ranker`
- `planning.task_tree`
- `perception.lane_occupancy_analyzer`
- `runtime.risk_trace_writer`

## Risk Object Ranking

`prediction.risk_object_ranker` transforms predicted objects into ranked conflict candidates using:
- longitudinal gap
- lateral gap
- relative speed
- TTC
- cut-in probability
- route conflict flag

Risk labels include:
- `critical_same_lane`
- `predicted_cut_in`
- `adjacent_interaction`
- `same_lane_follow`
- `background_actor`

## Task Tree

`planning.task_tree` consumes mission state, scene risk, lane flow and top-risk objects. It outputs one of:
- `nominal_drive`
- `overtake_slower_traffic`
- `stabilize_and_yield`
- `keep_lane_safety`
- `hard_brake_fallback`

## Why both BT and task tree

`planning.behavior_tree` is immediate and reactive.

`planning.task_tree` is slower tactical arbitration. It allows the runtime to keep BT logic simple while still expressing stronger tactical intent.
