# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `adaptive_component_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Post-95 search abs | Supplemental abs | Route-not-found abs | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| area_search_2uav_target_confirm | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| area_search_3uav | 0.020952 | 0.048000 | 0.125996 | 0.192924 | 34.142136 | 1.000000 | 0.000000 | -0.037064 | 9.000000 | 0.000000 |
| area_search_5uav | 0.005014 | -0.203125 | -0.341688 | -0.360999 | 77.573593 | 0.000000 | 0.000000 | 0.049001 | 10.000000 | 0.000000 |
| clustered_launch_3uav_open | 0.000400 | 0.009009 | 0.011799 | 0.082092 | 0.000000 | 0.000000 | 0.000000 | 0.001791 | 2.000000 | 0.000000 |
| common_edge_3uav_sparse_obstacles | -0.000413 | -0.371212 | -0.370225 | -0.815433 | 0.000000 | 0.000000 | 0.000000 | -0.000150 | 3.000000 | 0.000000 |
| common_edge_3uav_spread_bottom | -0.001200 | -0.317549 | -0.316668 | -0.812684 | 0.000000 | 0.000000 | 0.000000 | -0.000064 | 3.000000 | 0.000000 |
| common_edge_3uav_spread_left | -0.002800 | 0.014563 | 0.015141 | 0.036178 | 0.000000 | 0.000000 | 0.000000 | -0.000154 | 0.000000 | 0.000000 |
| common_edge_4uav_spread_bottom | -0.016800 | -0.310954 | -0.398088 | -0.936755 | -240.710678 | -1.000000 | 0.000000 | 0.012549 | 7.000000 | 0.000000 |
| distributed_3uav_should_not_sector | -0.000400 | 0.054348 | 0.056965 | 0.083404 | 0.000000 | 0.000000 | 0.000000 | 0.001714 | 9.000000 | 0.000000 |
