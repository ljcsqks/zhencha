# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `segment_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| area_search_1uav | 0.000000 | -0.013333 | -0.013854 | -0.003268 | 0.000000 | 15.000000 | 0.000000 |
| area_search_2uav | 0.034019 | 0.098551 | 0.234236 | 0.816700 | -0.071917 | 26.000000 | 0.000000 |
| area_search_2uav_target_confirm | 0.038219 | -0.012920 | 0.174684 | 0.405087 | -0.013010 | 26.000000 | 0.000000 |
| area_search_3uav | 0.026189 | 0.060000 | 0.228819 | 0.638153 | -0.015135 | 31.000000 | 0.000000 |
| area_search_4uav | 0.023245 | 0.228155 | 0.275232 | 0.301530 | 0.024965 | 50.000000 | 0.000000 |
| area_search_5uav | 0.029170 | -0.151042 | -0.091171 | -0.164498 | 0.086406 | 41.000000 | 0.000000 |
| stress_5uav_balance | -0.013129 | -0.058201 | 0.049047 | 0.202709 | -0.026854 | 21.000000 | 0.000000 |
| stress_fragmented_area_4uav_reachable | -0.002383 | 0.017699 | 0.019034 | 0.100432 | 0.012152 | 51.000000 | 0.000000 |
| stress_obstacle_maze_3uav | -0.000519 | -0.278061 | -0.130103 | 0.145944 | 0.086564 | 31.000000 | 0.000000 |
