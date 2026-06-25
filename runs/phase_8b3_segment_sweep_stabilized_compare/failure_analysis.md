# Phase 8b-3 Failure Analysis

`segment_sweep_v1` passes the hard safety and coverage checks in this run, but it does not satisfy all performance targets, so it should not become the default algorithm yet.

## Summary

- Basic coverage passes: all six base scenarios are at or above 0.95 final coverage.
- Safety passes: `no_fly_violations = 0` and `task_route_not_found = 0` for all compared scenarios.
- Target confirmation passes: `area_search_2uav_target_confirm` keeps `confirm_success_rate = 1.0` and `interrupted_task_resume_rate = 1.0`.
- Efficiency still fails: `area_search_2uav`, `area_search_3uav`, and `area_search_4uav` remain more than 8% above baseline total distance.
- 5UAV benefit is not stable: `area_search_5uav` is still faster to 95% than baseline, but total distance and redundant coverage are no longer below baseline.

## 1. Segment Selection Too Many Or Too Few

The threshold-aware selection reduced post-95 chasing and supplemental count, but the initial segment set still does not map cleanly to final mission coverage. In `area_search_4uav`, selecting 21 of 22 generated initial segments only reaches about 82% before supplemental recovery. The planner now recovers to 0.9535 final coverage, but it needs supplemental work to do so.

This means segment generation/selection is still too coarse: the selected segment coverage estimate is not a reliable predictor of final searchable-cell coverage.

## 2. Connector Cost Estimate Still Imperfect

A* connector cost improved safety and avoids unreachable connectors, but the current local use is still greedy. It estimates point-to-segment and segment-to-segment cost better than Manhattan, yet the chosen segment order can still produce long logical connectors.

Evidence: `area_search_3uav` and `area_search_4uav` still have high total distance versus baseline despite no route failures.

## 3. Bundle Exchange Partially Effective

Bundle exchange reduces some max bundle costs, but not enough to meet 2/3/4 UAV targets. It is intentionally lightweight and only moves a limited number of segments from the max-cost UAV. This avoids global TSP complexity, but it cannot consistently fix poor initial bundle shape.

Next improvement should consider segment clustering before assignment, not only exchange after assignment.

## 4. Supplemental Strategy Improved But Still Affects Mid Coverage

Compared with 8b-2.1, supplemental counts dropped sharply:

- `area_search_2uav`: 28 -> 0
- `area_search_4uav`: 47 -> 5
- `area_search_2uav_target_confirm`: 28 -> 0

This is a major stabilization win. However, when initial segment coverage undershoots, supplemental recovery still adds distance in 3/4/5 UAV scenarios.

## 5. Post-Goal Cancellation Works

Post-goal cancellation is active: post-95 search distance is much lower than 8b-2.1 in the problematic scenarios, and ordinary supplemental tasks are not created after the mission goal is reached. This part is working as intended.

The remaining performance gap is mostly pre-goal routing and segment allocation, not post-goal chasing.

## Recommendation

Do not make `segment_sweep_v1` the default yet.

Recommended Phase 8b-4 focus:

1. Improve segment generation so estimated selected coverage tracks actual searchable coverage.
2. Add local segment clustering before bundle assignment.
3. Use A* connector-aware ordering inside each component before cross-component assignment.
4. Add a stricter workload objective for 2/3/4 UAV scenarios.
5. Preserve the 8b-3 post-goal cancellation and supplemental gating, since those clearly reduced late wasted distance.
