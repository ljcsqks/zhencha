# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `segment_sweep_v1`

| Scenario | Coverage Δ | Time95 Δ% | Distance Δ% | Redundant Δ% | Workload Δ | No-fly Δ |
|---|---:|---:|---:|---:|---:|---:|
| area_search_1uav | 0.000000 | -0.013333 | -0.013854 | -0.003268 | 0.000000 | 0.000000 |
| area_search_2uav | 0.034019 | 0.098551 | 0.234236 | 0.816700 | -0.071917 | 0.000000 |
| area_search_2uav_target_confirm | 0.038219 | -0.012920 | 0.174684 | 0.405087 | -0.013010 | 0.000000 |
| area_search_3uav | 0.026189 | 0.060000 | 0.228819 | 0.638153 | -0.015135 | 0.000000 |
| area_search_4uav | 0.023245 | 0.228155 | 0.275232 | 0.301530 | 0.024965 | 0.000000 |
| area_search_5uav | 0.029170 | -0.151042 | -0.091171 | -0.164498 | 0.086406 | 0.000000 |
| stress_5uav_balance | -0.013129 | -0.058201 | 0.049047 | 0.202709 | -0.026854 | 0.000000 |
| stress_fragmented_area_4uav_reachable | -0.002383 | 0.017699 | 0.019034 | 0.100432 | 0.012152 | 0.000000 |
| stress_obstacle_maze_3uav | -0.000519 | -0.278061 | -0.130103 | 0.145944 | 0.086564 | 0.000000 |

## Notes

`segment_sweep_v1` is active and produces non-zero segment deltas, but the first implementation is mixed:

- It improves the base 5-UAV scenario distance, redundancy, workload balance, and time-to-95.
- It regresses 2/3/4-UAV base scenarios because segment bundles are greedy and connector estimates are too coarse.
- It regresses post-95 search distance in several scenarios because remaining priority or large uncovered components can be completed late.
- See `failure_analysis.md` in this directory for the detailed root-cause breakdown and Phase 8c recommendations.
