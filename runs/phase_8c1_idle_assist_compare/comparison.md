# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `adaptive_component_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Post-95 search abs | Supplemental abs | Route-not-found abs | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| area_search_2uav_target_confirm | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| area_search_3uav | 0.000000 | 0.004000 | 0.003778 | -0.022260 | 0.000000 | 0.000000 | 0.000000 | -0.000007 | 5.000000 | 0.000000 |
| area_search_4uav | -0.002279 | -0.004902 | -0.001180 | -0.002172 | 0.000000 | 0.000000 | 0.000000 | 0.000697 | 10.000000 | 0.000000 |
| area_search_5uav | 0.000000 | -0.057292 | -0.237374 | -0.295567 | -62.426407 | -2.000000 | 0.000000 | 0.048600 | 10.000000 | 0.000000 |
| stress_5uav_balance | 0.001313 | -0.033708 | -0.035654 | -0.216401 | 0.000000 | 0.000000 | 0.000000 | -0.001938 | 7.000000 | 0.000000 |
| stress_obstacle_maze_3uav | 0.000000 | -0.137821 | -0.136446 | -0.073306 | 0.000000 | 0.000000 | 0.000000 | -0.000073 | 31.000000 | 0.000000 |
