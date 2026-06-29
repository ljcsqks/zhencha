# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `adaptive_component_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Post-95 search abs | Supplemental abs | Route-not-found abs | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| area_search_2uav_target_confirm | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| area_search_3uav | 0.000873 | 0.056000 | 0.206700 | 0.334393 | 42.426407 | 2.000000 | 0.000000 | -0.048455 | 14.000000 | 0.000000 |
| area_search_5uav | 0.005469 | -0.166667 | -0.209590 | -0.162444 | -4.142136 | -1.000000 | 0.000000 | -0.008756 | 13.000000 | 0.000000 |
| clustered_launch_3uav_open | 0.000400 | 0.009009 | 0.011799 | 0.082092 | 0.000000 | 0.000000 | 0.000000 | 0.001791 | 2.000000 | 0.000000 |
| common_edge_3uav_sparse_obstacles | -0.001240 | -0.361111 | -0.360678 | -0.785885 | 0.000000 | 0.000000 | 0.000000 | -0.000208 | 5.000000 | 0.000000 |
| common_edge_3uav_spread_bottom | -0.000400 | -0.331476 | -0.329934 | -0.892283 | 0.000000 | 0.000000 | 0.000000 | 0.000110 | 0.000000 | 0.000000 |
| common_edge_3uav_spread_left | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| common_edge_4uav_spread_bottom | -0.017600 | -0.332155 | -0.416122 | -0.993095 | -240.710678 | -1.000000 | 0.000000 | 0.012626 | 0.000000 | 0.000000 |
| distributed_3uav_should_not_sector | -0.000400 | 0.054348 | 0.056965 | 0.083404 | 0.000000 | 0.000000 | 0.000000 | 0.001714 | 9.000000 | 0.000000 |
