# Phase 8b-5 Default Readiness Failure Analysis

Candidate `adaptive_component_sweep_v1` is not promoted to default in this run.

## Blocking Reasons

- `area_search_5uav` did not improve `time_to_95_coverage_s` versus baseline: delta is `0.0%`, while the readiness target requires it to be lower than baseline.
- `stress_fragmented_area_4uav_reachable` improved distance and redundancy, but `time_to_95_coverage_s` regressed by about `21.24%`.
- `stress_obstacle_maze_3uav` is faster and shorter, but final coverage is lower by about `1.19 percentage points`; it still remains above the mission threshold, but this makes default promotion risky.

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
| area_search_5uav | 0.000000 | 0.000000 | -0.001540 | -0.003077 | 0 | 0 |
| stress_5uav_balance | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0 | 0 |
| stress_fragmented_area_4uav_reachable | 0.019542 | 0.212389 | -0.088954 | -0.121379 | 0 | 0 |
| stress_obstacle_maze_3uav | -0.011929 | -0.193878 | -0.164573 | -0.001329 | 0 | 0 |

## Recommendation

Proceed to Phase 8b-6 focused on 5UAV simple-component frontload behavior and fragmented-area time-to-95 ordering. Keep `config/default.yaml` on `baseline_sparse_boustrophedon` until 5UAV time-to-95 is lower than baseline without increasing total distance or redundancy.
