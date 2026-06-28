# Default Algorithm Regression

- Default: `adaptive_component_sweep_v1`
- Baseline: `baseline_sparse_boustrophedon`
- Overall: **PASS**

| Scenario | Status | Coverage | Time95 | Distance | Redundant | Workload | Planned vs actual |
|---|---|---:|---:|---:|---:|---:|---|
| area_search_1uav | PASS | 0.9508 | 750.0000 | 7495.9798 | 0.2660 | 1.0000 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| area_search_2uav | PASS | 0.9509 | 345.0000 | 6889.8276 | 0.1979 | 0.9996 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| area_search_2uav_target_confirm | PASS | 0.9513 | 387.0000 | 7728.9444 | 0.2640 | 0.9992 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| area_search_3uav | PASS | 0.9520 | 251.0000 | 7515.6854 | 0.2618 | 0.9991 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| area_search_4uav | PASS | 0.9599 | 200.0000 | 8243.5029 | 0.3898 | 0.8916 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| area_search_5uav | PASS | 0.9544 | 185.0000 | 9916.0512 | 0.4432 | 0.8549 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| stress_5uav_balance | PASS | 0.9799 | 187.0000 | 10136.6400 | 0.4127 | 0.9001 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| stress_dynamic_obstacle_mid_route | PASS | 0.9508 | 225.0000 | 6741.5433 | 0.1650 | 0.9991 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| stress_fragmented_area_4uav_reachable | PASS | 0.9576 | 226.0000 | 10383.7468 | 0.4619 | 0.9572 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
| stress_obstacle_maze_3uav | PASS | 0.9549 | 316.0000 | 8693.7973 | 0.4709 | 0.8311 | actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells |
