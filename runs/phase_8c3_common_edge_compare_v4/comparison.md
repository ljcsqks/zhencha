# Algorithm Comparison

- Baseline: `baseline_sparse_boustrophedon`
- Candidate: `adaptive_component_sweep_v1`

| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Post-95 search abs | Supplemental abs | Route-not-found abs | Workload delta | Unique segments delta | No-fly delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| area_search_2uav_target_confirm | -0.000840 | -0.007752 | -0.007985 | 0.004231 | 0.000000 | 0.000000 | 0.000000 | 0.000370 | 0.000000 | 0.000000 |
| area_search_3uav | 0.000000 | 0.004000 | 0.003778 | -0.022260 | 0.000000 | 0.000000 | 0.000000 | -0.000007 | 5.000000 | 0.000000 |
| area_search_5uav | 0.000000 | -0.057292 | -0.237374 | -0.295567 | -62.426407 | -2.000000 | 0.000000 | 0.048600 | 10.000000 | 0.000000 |
| clustered_launch_3uav_open | 0.000400 | 0.009009 | 0.011799 | 0.082092 | 0.000000 | 0.000000 | 0.000000 | 0.001791 | 2.000000 | 0.000000 |
| common_edge_3uav_sparse_obstacles | -0.000413 | -0.373737 | -0.374035 | -0.819628 | 0.000000 | 0.000000 | 0.000000 | -0.002451 | 4.000000 | 0.000000 |
| common_edge_3uav_spread_bottom | 0.001200 | -0.317549 | -0.316283 | -0.821221 | 0.000000 | 0.000000 | 0.000000 | 0.000279 | 3.000000 | 0.000000 |
| common_edge_3uav_spread_left | -0.002400 | -0.004854 | -0.004306 | 0.011584 | 0.000000 | 0.000000 | 0.000000 | -0.000180 | 0.000000 | 0.000000 |
| common_edge_4uav_spread_bottom | -0.017600 | -0.325088 | -0.410011 | -0.936702 | -240.710678 | -1.000000 | 0.000000 | 0.013083 | 7.000000 | 0.000000 |
| distributed_3uav_should_not_sector | -0.000400 | 0.054348 | 0.056965 | 0.083404 | 0.000000 | 0.000000 | 0.000000 | 0.001714 | 9.000000 | 0.000000 |
