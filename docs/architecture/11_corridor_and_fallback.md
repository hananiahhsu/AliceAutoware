# Corridor and Fallback in v11

This revision adds two tactical/runtime modules.

- `planning.corridor_cost_evaluator` scores every candidate lane corridor using base corridor cost, lane-flow congestion, route-blocked hints, and ranked risk objects.
- `runtime.fallback_manager` monitors repeated supervisor interventions, blocked-current-lane signals, and collapsing TTC to trigger keep-lane fallback or minimal-risk stop.

The intent is to make the stack more resilient in blocked-lane recovery scenarios while keeping nominal lane-change scenarios functional.
