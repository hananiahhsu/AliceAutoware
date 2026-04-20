# External scenario format
CSV columns:
`role,lane_id,x,speed,behavior,target_lane,trigger_x,desired_speed`
Examples:
- `behavior=cut_in_left` / `cut_in_right`
- `behavior=brake`
- `target_lane` is optional for cut-in actors
- `trigger_x` uses ego longitudinal position as activation threshold
- `desired_speed` is the steady-state target after the behavior begins
