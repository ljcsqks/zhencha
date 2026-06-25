# Phase 8b-2 Segment Sweep V1 Failure Analysis

`segment_sweep_v1` is a real planner path, not a metadata-only alias of baseline. The comparison deltas are non-zero and candidate runs include segment metadata. However, the first implementation does not meet all performance targets.

## Hard Acceptance

Passed:

- All 6 base scenarios reached final coverage >= 0.95.
- `no_fly_violations = 0` in all compared scenarios.
- Target confirmation scenario kept `confirm_success_rate = 1.0`.
- Target confirmation scenario kept `interrupted_task_resume_rate = 1.0`.
- `stress_fragmented_area_4uav_reachable` reached final coverage >= 0.95.
- No large `task_route_not_found` pattern was observed in candidate snapshots.

## Performance Misses

The 5-UAV base scenario improved the main distance and redundancy targets:

- `area_search_5uav.total_distance_m`: -9.1% versus baseline.
- `area_search_5uav.redundant_coverage_rate`: -16.4% relative versus baseline.
- `area_search_5uav.time_to_95_coverage_s`: -15.1% versus baseline.
- `area_search_5uav.workload_balance_all_uavs`: improved.

But the broader target set was not met:

- `area_search_2uav`, `area_search_3uav`, and `area_search_4uav` increased total distance and redundant coverage.
- `area_search_4uav.time_to_95_coverage_s` was 22.8% worse than baseline.
- `stress_5uav_balance` increased total distance and redundant coverage.
- `post_95_search_distance_m` improved in some stress cases but regressed badly in `area_search_4uav`, `area_search_5uav`, and `stress_5uav_balance`.

## Likely Root Causes

1. Segment allocation is too greedy at the segment level.
   The allocator considers current projected route end and segment sweep cost, but it does not perform bundle exchange after initial assignment. Some UAVs receive long connector-heavy bundles even when all-UAV distance balance looks acceptable.

2. Segment ordering is nearest-neighbor only.
   It reduces local connector length but does not reason about finishing priority zones before global coverage reaches 95%. This leads to extra post-95 search flight while remaining priority or large uncovered components are still pending.

3. Connector cost estimate is Manhattan-based.
   The actual executable route uses A*, while bundle assignment uses an approximation. Around obstacles and no-fly zones, the planner underestimates connector cost and creates inefficient bundles.

4. Segment granularity is not adaptive.
   The first version samples lines at `2 * sensor_radius_cells` and creates line fragments directly. This works for 5-UAV base distance but creates too many logical segments in fragmented or obstacle-heavy areas, increasing repeated sensor overlap.

5. Supplemental strategy still reacts after the fact.
   Large supplemental regions use the current planner, but the main segment planner does not reserve enough coverage completeness per responsibility area. This causes late supplemental work and contributes to post-95 distance.

## Phase 8c Recommendations

1. Add bundle exchange after greedy assignment.
   Try single-segment moves and swaps when they reduce max bundle cost or improve `segment_workload_balance`.

2. Use A* connector estimates selectively.
   Cache endpoint-to-endpoint A* distances for candidate bundle insertion, especially when obstacles/no-fly zones are near the connector.

3. Add priority-aware route ordering with bounded detour.
   Priority segments should move earlier only when the connector penalty is within a configurable bound. A naive priority-first order was tested and worsened both 2-UAV and 5-UAV samples.

4. Merge short adjacent segments.
   Reduce excessive small segment count before assignment, while preserving obstacle/no-fly splits.

5. Stop or split active segment tasks at coverage threshold more aggressively.
   When global coverage is reached and priority coverage is also complete, active segment tasks should be trimmed by segment coverage cells rather than only waypoint footprint.
