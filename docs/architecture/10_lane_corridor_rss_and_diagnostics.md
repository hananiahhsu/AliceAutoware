# v10: Lane Corridor, RSS Longitudinal/Lateral, Diagnostics

This version adds a stronger middle layer between route intent and low-level control.

## New capability blocks

- `map.lane_corridor_builder`: builds an explicit lane corridor over the planning horizon.
- `prediction.multi_agent_behavior_predictor`: aggregates multi-actor interaction risk into a scenario-level cluster score.
- `safety.rss_longitudinal_checker`: front-gap braking safety check inspired by RSS-style longitudinal distance logic.
- `safety.rss_lateral_checker`: target-lane lateral overlap check for lane-change suppression.
- `control.feedforward_controller`: curvature-derived feedforward steering and speed bias.
- `runtime.diagnostics_monitor`: converts run summary metrics into warn/error style health records.

## Default build policy

The default build compiles the core stack plus a growing test suite.
Large generated interface and regression assets are stored under `generated/` and `configs/generated_regression_suite/` to increase coverage and downstream integration readiness without bloating default compile time.
