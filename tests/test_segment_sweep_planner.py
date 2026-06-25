from __future__ import annotations

import pytest

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.planning.coverage_planner import SegmentSweepPlanner, create_coverage_planner
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
