# Phase 8b-5 Default Readiness Failure Analysis

Candidate `adaptive_component_sweep_v1` is not promoted to default in this run.

## What Improved

- `area_search_5uav` now improves `time_to_95_coverage_s` by `3.65%`, total distance by `7.82%`, and redundant coverage by `4.55%`.
- `stress_5uav_balance` improves `time_to_95_coverage_s` by `1.06%`, total distance by `1.69%`, and redundant coverage by `3.14%`.
- `area_search_4uav`, `stress_obstacle_maze_3uav`, and the 5UAV scenarios reduce total distance.

## Blocking Reasons

- `area_search_3uav` regresses slightly: `time_to_95_coverage_s +0.40%` and `total_distance_m +0.38%`.
- `stress_fragmented_area_4uav_reachable` still regresses `time_to_95_coverage_s` by about `21.24%`, even though it improves final coverage, distance, redundancy, and post-95 distance.
- `stress_obstacle_maze_3uav` remains faster and shorter, but final coverage is lower by about `1.19 percentage points`; it stays above mission threshold, but this is not clean enough for default promotion.

## Safety Checks Observed

- `no_fly_violations_delta` is `0` for all compared scenarios.
- `task_route_not_found_delta_abs` is `0` for all compared scenarios.
- Target confirmation scenario keeps `confirm_success_rate = 1.0` and `interrupted_task_resume_rate = 1.0`.

## Relevant Deltas

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | No-fly delta | Route-not-found delta |
|---|---:|---:|---:|---:|---:|---:|
| area_search_1uav | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0 | 0 |
| area_search_2uav | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0 | 0 |
| area_search_2uav_target_confirm | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0 | 0 |
| area_search_3uav | 0.000000 | 0.004000 | 0.003778 | -0.022260 | 0 | 0 |
| area_search_4uav | 0.001823 | -0.029126 | -0.026273 | -0.015095 | 0 | 0 |
| area_search_5uav | -0.002735 | -0.036458 | -0.078167 | -0.045478 | 0 | 0 |
| stress_5uav_balance | 0.000000 | -0.010582 | -0.016889 | -0.031447 | 0 | 0 |
| stress_fragmented_area_4uav_reachable | 0.019542 | 0.212389 | -0.088954 | -0.121379 | 0 | 0 |
| stress_obstacle_maze_3uav | -0.011929 | -0.193878 | -0.164573 | -0.001329 | 0 | 0 |

## Recommendation

Keep `config/default.yaml` on `baseline_sparse_boustrophedon`. Next phase should target fragmented-component time-to-95 ordering and the small 3UAV regression before promoting adaptive planning as default.
