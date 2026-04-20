# Safety Guardian and Scenario Authoring
## Why this layer exists
A realistic AD stack cannot depend on a single planning decision path. Production systems keep an independent safety layer that can clamp, veto, or override motion outputs when TTC, road boundary, or interaction metrics become unacceptable.
The `safety.guardian` module in this repository performs three jobs:
1. inspect predicted TTC and front-gap risk
2. reject trajectories that leave the modeled road envelope
3. provide a hard-stop override path that can clamp acceleration
## External scenario CSV format
`role,lane_id,x,speed,behavior,target_lane,trigger_x,desired_speed`
Behavior values:
- `cruise`
- `cut_in_left`
- `cut_in_right`
- `brake`
## Bridge files
The repository exports state and command channels under `out/bridge/` for future CARLA / AWSIM / custom simulator adapters.
