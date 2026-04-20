# Advanced Prediction, Tactical Planning, and Safety (v9)

## New v9 modules

- `prediction.joint_interaction_predictor`: scores pairwise / multi-actor merge conflicts at the lane level.
- `planning.lane_sequence_planner`: converts route goal + risk + traffic flow into a tactical lane sequence.
- `safety.rss_lane_change_checker`: applies RSS-style front/rear distance checks before lane commitment.
- `control.gain_scheduler`: schedules lateral and longitudinal gains by speed and curvature demand.
- `runtime.interaction_trace_writer`: records joint-interaction hazards for replay and offline audit.

## Tactical flow

1. Build object tracks and lane occupancy.
2. Predict per-object intentions and trajectory hypotheses.
3. Build joint interaction conflicts from simultaneous target-lane merges.
4. Score lane-level risk and choose a route-aligned lane sequence.
5. Run gap acceptance and RSS lane-change validation.
6. Run safety supervisor and low-level guardian before final control.

## Why this matters

This makes the stack closer to a production ADS architecture:
- lane changes are no longer driven only by a single front obstacle;
- tactical decisions consider route alignment, congestion, and interaction density;
- lane-change safety has both heuristic gap checks and RSS-style guardrails;
- control is gain-scheduled rather than fixed-gain only.
