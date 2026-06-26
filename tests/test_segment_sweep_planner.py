from __future__ import annotations

import pytest

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.planning.coverage_planner import (
    AdaptiveComponentSweepPlanner,
    ComponentComplexityAnalyzer,
    SegmentConnectorCostCache,
    SegmentSweepPlanner,
    SweepSegment,
    create_coverage_planner,
    simulate_planned_coverage,
)
from uav_search.planning.reachability import build_reachability_index


def _uav(uav_id: str, position: Position) -> UAVState:
    return UAVState(
        id=uav_id,
        position=position,
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=UAVStatus.IDLE,
        home_position=position,
    )


def test_factory_selects_segment_sweep_and_rejects_unknown_version() -> None:
    assert isinstance(create_coverage_planner({"algorithm": {"version": "segment_sweep_v1"}}), SegmentSweepPlanner)
    assert isinstance(
        create_coverage_planner({"algorithm": {"version": "adaptive_component_sweep_v1"}}),
        AdaptiveComponentSweepPlanner,
    )
    with pytest.raises(ValueError, match="unknown algorithm.version"):
        create_coverage_planner({"algorithm": {"version": "missing_version"}})


def test_segment_generation_splits_at_obstacles_and_keeps_segments_passable() -> None:
    grid_map = GridMap(width_m=80, height_m=50, resolution_m=10)
    grid_map.set_cell(Position(3, 2), {"cell_type": CellType.OBSTACLE})
    uavs = [_uav("uav_01", Position(0, 2))]
    reachability = build_reachability_index(grid_map, uavs)
    planner = SegmentSweepPlanner({})

    segments = planner.generate_segments(
        searchable_cells=set(grid_map.get_searchable_cells()),
        grid_map=grid_map,
        uav_states=uavs,
        sensor_radius_cells=1,
        reachability=reachability,
    )
    row_segments = [segment for segment in segments if segment.orientation == "horizontal" and segment.line_index == 2]

    assert len(row_segments) >= 2
    assert all(grid_map.is_passable(point) for segment in segments for point in segment.sampled_waypoints)
    assert all(Position(3, 2) not in segment.coverage_cells for segment in row_segments)
    assert all(segment.allowed_uav_ids == {"uav_01"} for segment in segments)


def test_segment_generation_does_not_duplicate_segment_identity() -> None:
    grid_map = GridMap(width_m=120, height_m=80, resolution_m=10)
    uavs = [_uav("uav_01", Position(0, 0))]
    reachability = build_reachability_index(grid_map, uavs)
    planner = SegmentSweepPlanner({})

    segments = planner.generate_segments(
        searchable_cells=set(grid_map.get_searchable_cells()),
        grid_map=grid_map,
        uav_states=uavs,
        sensor_radius_cells=1,
        reachability=reachability,
    )
    identities = [
        (
            segment.component_id,
            segment.orientation,
            segment.line_index,
            segment.start,
            segment.end,
        )
        for segment in segments
    ]

    assert len(identities) == len(set(identities))


def test_segment_sweep_initial_tasks_are_preassigned_bundles_with_metadata() -> None:
    grid_map = GridMap(width_m=120, height_m=60, resolution_m=10)
    uavs = [_uav("uav_01", Position(0, 1)), _uav("uav_02", Position(0, 4))]
    reachability = build_reachability_index(grid_map, uavs)
    planner = SegmentSweepPlanner({})

    tasks = planner.plan_initial_tasks(
        grid_map=grid_map,
        uav_states=uavs,
        sensor_radius_cells=1,
        created_at=0.0,
        reachability=reachability,
        searchable_cells=set(grid_map.get_searchable_cells()),
    )

    assert {next(iter(task.allowed_uav_ids or set())) for task in tasks} == {"uav_01", "uav_02"}
    assert all(task.coverage_waypoints for task in tasks)
    assert all(task.metadata["planner_version"] == "segment_sweep_v1" for task in tasks)
    assert sum(task.metadata["segment_count"] for task in tasks) > 0
    for task in tasks:
        assert len(task.metadata["segment_ids"]) == len(set(task.metadata["segment_ids"]))
    assert len({tuple(task.coverage_waypoints) for task in tasks}) == len(tasks)


def test_segment_selection_stops_after_coverage_goal_and_keeps_priority() -> None:
    grid_map = GridMap(width_m=100, height_m=50, resolution_m=10)
    grid_map.set_cell(Position(8, 0), {"search_priority": 3.0})
    uavs = [_uav("uav_01", Position(0, 0))]
    reachability = build_reachability_index(grid_map, uavs)
    planner = SegmentSweepPlanner(
        {
            "search": {"mission_complete_coverage_threshold": 0.5, "priority_complete_threshold": 1.0},
            "algorithm": {"segment_sweep": {"coverage_margin": 0.0, "max_initial_coverage_target": 0.55}},
        }
    )

    segments = planner.generate_segments(
        searchable_cells=set(grid_map.get_searchable_cells()),
        grid_map=grid_map,
        uav_states=uavs,
        sensor_radius_cells=1,
        reachability=reachability,
    )
    selected = planner.select_segments_for_coverage_goal(
        segments,
        grid_map,
        mission_complete_coverage_threshold=0.5,
        priority_complete_threshold=1.0,
    )

    assert 0 < len(selected) < len(segments)
    assert any(Position(8, 0) in segment.coverage_cells for segment in selected)
    assert planner.last_diagnostics["generated_segment_count"] == len(segments)
    assert planner.last_diagnostics["selected_segment_count"] == len(selected)


def test_connector_cache_uses_astar_cost_and_reuses_hits() -> None:
    grid_map = GridMap(width_m=70, height_m=50, resolution_m=10)
    for y in range(4):
        grid_map.set_cell(Position(3, y), {"cell_type": CellType.OBSTACLE})
    cache = SegmentConnectorCostCache(planner_run_id="test")

    cost = cache.cost(Position(0, 0), Position(6, 0), grid_map)
    cached = cache.cost(Position(0, 0), Position(6, 0), grid_map)

    assert cost > 6 * grid_map.resolution_m
    assert cached == cost
    assert cache.misses == 1
    assert cache.hits == 1


def test_unreachable_connector_is_not_selected_for_assignment() -> None:
    grid_map = GridMap(width_m=70, height_m=50, resolution_m=10)
    for y in range(5):
        grid_map.set_cell(Position(3, y), {"cell_type": CellType.OBSTACLE})
    uav = _uav("uav_01", Position(0, 0))
    segment = SweepSegment(
        id="blocked",
        component_id="c1",
        orientation="horizontal",
        line_index=0,
        start=Position(5, 0),
        end=Position(6, 0),
        sampled_waypoints=[Position(5, 0), Position(6, 0)],
        coverage_cells={Position(5, 0), Position(6, 0)},
        priority_value=0.0,
        uncovered_value=2.0,
        allowed_uav_ids={"uav_01"},
        sweep_cost_m=10.0,
    )
    planner = SegmentSweepPlanner({})

    bundles = planner.assign_segments_to_uavs([segment], [uav], grid_map)

    assert bundles["uav_01"] == []
    assert planner.last_diagnostics["unreachable_connector_count"] >= 1


def test_bundle_exchange_reduces_max_cost_without_losing_segments() -> None:
    grid_map = GridMap(width_m=100, height_m=40, resolution_m=10)
    planner = SegmentSweepPlanner({"algorithm": {"segment_sweep": {"bundle_exchange_iterations": 20}}})
    uavs = [_uav("uav_01", Position(0, 0)), _uav("uav_02", Position(9, 0))]
    segments = [
        SweepSegment(
            id=f"s{i}",
            component_id="c1",
            orientation="horizontal",
            line_index=i,
            start=Position(i, 0),
            end=Position(i + 1, 0),
            sampled_waypoints=[Position(i, 0), Position(i + 1, 0)],
            coverage_cells={Position(i, 0), Position(i + 1, 0)},
            priority_value=0.0,
            uncovered_value=2.0,
            allowed_uav_ids={"uav_01", "uav_02"},
            sweep_cost_m=10.0,
        )
        for i in range(5)
    ]
    bundles = {"uav_01": segments, "uav_02": []}
    before = planner.bundle_costs(bundles, uavs, grid_map)

    improved = planner.improve_segment_bundles(bundles, uavs, grid_map)
    after = planner.bundle_costs(improved, uavs, grid_map)

    assert max(after.values()) < max(before.values())
    assert sorted(segment.id for bundle in improved.values() for segment in bundle) == [segment.id for segment in segments]


def test_component_complexity_analyzer_classifies_simple_and_fragmented_components() -> None:
    grid_map = GridMap(width_m=100, height_m=60, resolution_m=10)
    uavs = [_uav("uav_01", Position(0, 0))]
    analyzer = ComponentComplexityAnalyzer({})
    simple_component = {Position(x, y) for x in range(8) for y in range(4)}

    simple = analyzer.analyze(simple_component, "simple", grid_map, uavs)

    assert simple.kind == "simple"
    assert simple.fill_ratio > 0.9
    assert simple.avg_segments_per_scanline == 1.0

    fragmented = set(simple_component)
    for y in range(4):
        fragmented.discard(Position(3, y))
        fragmented.discard(Position(4, y))
    complex_result = analyzer.analyze(fragmented, "complex", grid_map, uavs)

    assert complex_result.kind == "complex"
    assert complex_result.fragmented_line_count >= 4
    assert complex_result.avg_segments_per_scanline > 1.0


def test_simulate_planned_coverage_uses_waypoint_sensor_footprint() -> None:
    cells = {Position(x, y) for x in range(5) for y in range(3)}
    covered = simulate_planned_coverage([Position(2, 1)], sensor_radius_cells=1, target_cells=cells)

    assert Position(2, 1) in covered
    assert Position(2, 0) in covered
    assert Position(0, 0) not in covered


def test_adaptive_planner_clusters_complex_segments_and_records_metadata() -> None:
    grid_map = GridMap(width_m=120, height_m=80, resolution_m=10)
    for y in range(1, 7):
        grid_map.set_cell(Position(5, y), {"cell_type": CellType.OBSTACLE})
    uavs = [_uav("uav_01", Position(0, 0)), _uav("uav_02", Position(11, 7))]
    reachability = build_reachability_index(grid_map, uavs)
    planner = AdaptiveComponentSweepPlanner(
        {
            "algorithm": {
                "adaptive_component_sweep": {
                    "simple_fill_ratio": 0.98,
                    "simple_max_avg_segments_per_scanline": 1.05,
                    "cluster_min_segments": 2,
                    "cluster_max_segments": 4,
                }
            },
            "search": {"mission_complete_coverage_threshold": 0.95, "priority_complete_threshold": 0.98},
        }
    )

    tasks = planner.plan_initial_tasks(
        grid_map=grid_map,
        uav_states=uavs,
        sensor_radius_cells=1,
        created_at=0.0,
        reachability=reachability,
        searchable_cells=set(grid_map.get_searchable_cells()),
    )

    assert tasks
    assert all(task.metadata["planner_version"] == "adaptive_component_sweep_v1" for task in tasks)
    assert sum(len(task.metadata.get("cluster_ids", [])) for task in tasks) > 0
    assert all(len(task.metadata.get("segment_ids", [])) == len(set(task.metadata.get("segment_ids", []))) for task in tasks)
    assert all(0.0 < task.metadata["planned_coverage_ratio"] <= 1.0 for task in tasks)
    assert planner.last_diagnostics["complex_component_count"] >= 1
    assert planner.last_diagnostics["cluster_count_total"] >= 1
