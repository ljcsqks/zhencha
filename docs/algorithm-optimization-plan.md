# Algorithm Optimization Plan

Phase 8 starts the algorithm optimization track. Phase 8a establishes versioning, diagnostics, A/B comparison, and stress scenarios. It does not change search planning behavior.

## Algorithm Versions

Current version:

- `baseline_sparse_boustrophedon`

Planned versions:

- `segment_sweep_v1`: replace cell-by-cell coverage waypoint thinking with sweep segments.
- `balanced_segment_sweep_v2`: add stronger multi-UAV workload balance and connector minimization.

Every run, export, metrics file, and comparison should preserve:

- `algorithm_version`
- `code_version`
- `config_hash`
- `scenario_name`
- `run_id`

## Optimization Goals

Coverage:

- All basic scenarios final coverage `>= 0.95`.
- Priority coverage meets the configured priority threshold.

Efficiency:

- In 3/4/5 UAV scenarios, `time_to_95_coverage_s` improves by at least 10% versus baseline.
- In the 5 UAV scenario, `total_distance_m` improves by at least 15% versus baseline.
- In the 5 UAV scenario, `redundant_coverage_rate` drops below 35%.
- `post_95_extra_distance_m` improves by at least 30%.

Coordination:

- `per_uav_workload_balance >= 0.92`.
- The 5 UAV scenario should be meaningfully faster than 4 UAV; at minimum, its `time_to_95_coverage_s` must not be worse.

Safety:

- `no_fly_violations = 0`.
- Rejected commands should not increase materially.
- Target confirmation scenario keeps `confirm_success_rate = 1.0`.
- Target confirmation scenario keeps `interrupted_task_resume_rate = 1.0`.

## Phase 8b: Segment Sweep V1

Recommended implementation path:

1. Keep the existing map, A*, simulation, event, scheduler, and command boundary.
2. Add a search planner abstraction behind the current scheduler planning layer.
3. Convert coverage generation from individual sparse points into sweep segments:
   - generate obstacle-aware strips;
   - split strips at blocked cells;
   - connect adjacent strips with short local connectors;
   - keep `coverage_waypoints` separate from executable A* paths.
4. Assign segment bundles to UAVs by estimated route cost, not only area size.
5. Stop ordinary supplemental pursuit after mission threshold unless uncovered components are large.
6. Preserve dynamic obstacle connected-component splitting and target confirmation recovery.
7. Run `compare_algorithms` against baseline after each small change.

Primary diagnostics to watch:

- `route_quality.max_connector_length`
- `route_quality.long_connector_count`
- `allocation_quality.workload_balance`
- `coverage_quality.post_95_distance_m`
- `per_uav.*.average_coverage_gain_per_meter`

## Phase 8c: Balanced Segment Sweep V2

Recommended implementation path:

1. Improve cost model with connector penalties, turn penalties, priority weighting, and expected A* length.
2. Add local segment exchange between UAVs when workload imbalance is high.
3. Add route smoothing that preserves obstacle/no-fly constraints.
4. Tune supplemental task rules using stress scenarios.
5. Freeze candidate metrics and compare against `baseline_sparse_boustrophedon`.

## A/B Workflow

Run baseline versus candidate:

```bash
python -m uav_search.tools.compare_algorithms \
  --baseline baseline_sparse_boustrophedon \
  --candidate segment_sweep_v1 \
  --scenarios area_search_1uav area_search_2uav area_search_3uav area_search_4uav area_search_5uav area_search_2uav_target_confirm \
  --output runs/algorithm_compare_segment_sweep_v1
```

For Phase 8a, candidate may intentionally equal baseline. The expected deltas are near zero.

## Stress Scenarios

- `stress_obstacle_maze_3uav`: route quality around narrow passages.
- `stress_fragmented_area_4uav`: allocation balance across disconnected fragments.
- `stress_5uav_balance`: 5 UAV coordination and diminishing returns.
- `stress_target_confirm_mid_search`: target confirmation disruption during search.
- `stress_dynamic_obstacle_mid_route`: local replanning while following long paths.

## Acceptance

Before changing the algorithm in Phase 8b:

- `pytest -q` passes.
- `cd web && npm run test` passes.
- `cd web && npm run build` passes.
- `cd web && npm run e2e` passes.
- The 6 baseline CLI scenarios still pass mission coverage and safety checks.
- `compare_algorithms` baseline versus baseline produces near-zero deltas.
- Stress scenarios run and export metrics.
