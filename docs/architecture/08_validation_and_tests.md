# Validation and Unit Test Architecture v8

## Test Layout

The unit tests are intentionally split by subsystem instead of keeping one monolithic test file.

- `test_prediction.cpp`: intent, hypotheses, risk ranking, lane-flow interpretation
- `test_planning.cpp`: route planning, BT, task tree, speed planning, gap acceptance
- `test_safety.cpp`: guardian and supervisor behavior
- `test_runtime.cpp`: localization, mission manager, schema writer, scenario catalog
- `test_scenarios.cpp`: closed-loop regression on representative scenarios

## Test Runtime Model

All tests are linked into a single lightweight executable `mad_tests`, but each case is registered independently through `tests/test_framework.*`.

This keeps the build simple while still making the test tree modular and easy to extend.

## Regression Philosophy

v8 uses two layers of verification:

1. **Unit tests** for fast deterministic checks on planning, prediction, and safety logic.
2. **Batch scenario regression** for closed-loop validation across 13 scenarios.

Both are run in the Linux validation path of this delivery.


## v9 test expansion

The v9 baseline adds dedicated test modules for `interaction` and `control`, and increases coverage for:

- joint interaction conflict detection
- lane-sequence tactical planning
- RSS lane-change rejection
- gain scheduling and bounded controller output
- two new regression scenarios
