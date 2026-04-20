# Momenta Autodrive Stack v11

This repository delivers a compact but engineering-structured autonomous driving stack sample in C++17.

## v8 additions

- multi-hypothesis object trajectory generation
- explainable gap-acceptance evaluation for lane changes
- RSS-style safety supervisor above the low-level guardian
- hypothesis trace outputs for debugging
- modular unit-test tree instead of one monolithic test file
- 13-scenario batch regression suite

## Build

```bash
cmake -S . -B out/build_v8 -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build out/build_v8 -j
ctest --test-dir out/build_v8 --output-on-failure
./out/build_v8/mad_batch_eval
```

## Key outputs

- `out/batch/scenario_summary.csv`
- `out/batch/scenario_report.md`
- `out/events`
- `out/decision_traces`
- `out/risk_traces`
- `out/hypothesis_traces`
- `out/interop`
- `out/replay`

## Test tree

- `tests/test_prediction.cpp`
- `tests/test_planning.cpp`
- `tests/test_safety.cpp`
- `tests/test_runtime.cpp`
- `tests/test_scenarios.cpp`
- `tests/test_framework.*`


## v9 highlights

- Joint interaction prediction
- Lane-sequence tactical planner
- RSS-style lane-change checker
- Gain-scheduled controller
- Expanded test matrix and regression scenarios


## Repository scale note

This repo now includes large generated source/spec assets under `generated/` and `configs/generated_regression_suite/`. They are intentionally kept out of the default build so the core stack still compiles quickly while the repository carries a much broader interface, validation, and scenario surface.


## v11 highlights

- corridor-cost tactical evaluator wired into the stack
- fallback manager for minimal-risk lane keeping / stop escalation
- expanded runtime diagnostics and recovery scenario
- additional unit tests for corridor selection and fallback escalation


## Windows build

Use `scripts\build_windows.bat`.

What the script does now:
- detects `cmake`
- detects `python` or `py -3`
- prefers `Ninja` when available
- falls back to Visual Studio 2022/2019 with C++ tools via `vswhere` + `VsDevCmd.bat`
- writes a persistent log to `out\logs\build_windows.log`
- pauses on failure unless `MAD_NO_PAUSE=1`

Run examples:

```bat
scripts\build_windows.bat
scripts\run_demo_windows.bat highway_lane_change
scripts\run_batch_eval_windows.bat
```
