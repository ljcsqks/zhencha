# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `adaptive_component_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Post-95 search abs | Supplemental abs | Route-not-found abs | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| area_search_2uav_target_confirm | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| area_search_3uav | 0.000000 | 0.004000 | 0.003778 | -0.022260 | 0.000000 | 0.000000 | 0.000000 | -0.000007 | 5.000000 | 0.000000 |
| area_search_5uav | 0.000000 | -0.057292 | -0.237374 | -0.295567 | -62.426407 | -2.000000 | 0.000000 | 0.048600 | 10.000000 | 0.000000 |
| clustered_launch_3uav_dynamic_obstacle | 0.001233 | -0.010444 | -0.009165 | -0.020477 | 0.000000 | 0.000000 | 0.000000 | 0.001599 | 13.000000 | 0.000000 |
| clustered_launch_3uav_open | 0.000400 | 0.009009 | 0.011799 | 0.082092 | 0.000000 | 0.000000 | 0.000000 | 0.001791 | 2.000000 | 0.000000 |
| clustered_launch_3uav_sparse_obstacles | -0.001238 | 0.000000 | 0.000597 | 0.022690 | 0.000000 | 0.000000 | 0.000000 | -0.000345 | 2.000000 | 0.000000 |
| clustered_launch_5uav_open | -0.012800 | 0.112500 | 0.066180 | -1.000000 | -350.000000 | 0.000000 | 0.000000 | -0.000016 | 4.000000 | 0.000000 |
| stress_obstacle_maze_3uav | 0.000000 | -0.137821 | -0.136446 | -0.073306 | 0.000000 | 0.000000 | 0.000000 | -0.000073 | 31.000000 | 0.000000 |
