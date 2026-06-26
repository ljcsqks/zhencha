# Phase 8b-4 Failure Analysis

`adaptive_component_sweep_v1` passes all hard validation checks and fixes the main 2/3/4 UAV regression from `segment_sweep_v1`. It is close to the performance target set, but one strict target is not fully satisfied: `area_search_5uav` matches baseline `time_to_95` instead of beating it.

## Summary

- Hard checks pass: base scenarios reach final coverage >= 0.95.
- Safety passes: `no_fly_violations = 0`.
- Command reliability passes: `task_route_not_found = 0`.
- Target confirmation passes: `confirm_success_rate = 1.0` and `interrupted_task_resume_rate = 1.0`.
- 2/3/4 UAV ordinary scenarios are now within the target envelope versus baseline.
- `stress_obstacle_maze_3uav` remains better than baseline for both `time_to_95` and total distance.
- `area_search_5uav` total distance and redundant coverage are not worse than baseline, but `time_to_95` is equal to baseline rather than better.

## 1. Simple/Complex Component Classification

The adaptive classifier now correctly treats high-fill, moderately fragmented ordinary scenarios as simple components, which routes them through baseline-style sparse boustrophedon. This fixed the large 2/3/4 UAV regressions from `segment_sweep_v1`.

The current classifier is intentionally conservative. It classifies `area_search_5uav` as simple, so it inherits baseline timing. That is why `time_to_95` is stable but not improved.

## 2. Cluster Size

Cluster sweep is active in genuinely complex components such as `stress_obstacle_maze_3uav` and `stress_fragmented_area_4uav_reachable`. The current cluster size is stable enough for safety and distance, but it does not optimize `time_to_95` in fragmented components.

## 3. Cluster Allocation

Cluster allocation is greedy and component-aware. It reduces the severe route overhead seen in single-segment assignment, but cluster exchange is still minimal. This is acceptable for Phase 8b-4 because ordinary scenarios are routed through the simple planner path.

## 4. Component Jumps

The planner visits clusters component-first and only then crosses components. No command route failures were observed. Further gains would likely come from component-level route scheduling, not from single segment tuning.

## 5. Planned Coverage Estimate

Planned coverage is now computed from ordered coverage waypoints with sensor footprint simulation. The estimate is useful for diagnostics, but simple baseline-style tasks still report per-task planned coverage rather than a fleet-level plan aggregate. This can make planned-vs-actual error noisy in multi-UAV simple scenarios.

## 6. Supplemental Recovery

Supplemental counts are controlled. The 2UAV and target-confirm scenarios do not show the large supplemental explosion from earlier phases, and post-95 search distance is near zero in the ordinary 2UAV scenario.

## 7. Priority Coverage

Priority cells no longer force a component into cluster mode by themselves. They contribute to complexity score, but simple high-fill regions remain simple. This avoids unnecessary priority-driven detours.

## Recommendation

`adaptive_component_sweep_v1` is a strong candidate for the next default, but the strict 5UAV `time_to_95` target is not yet beaten. Recommended next step:

1. Add a 5UAV-specific simple-component pre-sweep split that front-loads high-yield central bands without increasing total route length.
2. Keep the current conservative classifier because it fixed the major 2/3/4 UAV regressions.
3. Improve fleet-level planned coverage aggregation so planned-vs-actual diagnostics are easier to interpret.
