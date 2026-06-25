from __future__ import annotations

from uav_search.allocation.bid_calculator import calculate_bid
from uav_search.core.data_types import CellType, Position, Task, TaskType, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap


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


def _task(entry: Position) -> Task:
    return Task(
        id="task_001",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={entry},
        entry_point=entry,
        waypoints=[entry],
        coverage_waypoints=[entry],
        estimated_cost_m=10.0,
    )


def test_bid_rejects_uav_outside_allowed_uav_ids() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    task = _task(Position(1, 1))
    task.allowed_uav_ids = {"uav_02"}

    assert calculate_bid(_uav("uav_01", Position(0, 0)), task, grid_map, {"auction": {}}) is None
    assert calculate_bid(_uav("uav_02", Position(0, 0)), task, grid_map, {"auction": {}}) is not None


def test_bid_uses_astar_and_rejects_unreachable_entry_point() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for y in range(5):
        grid_map.set_cell(Position(2, y), {"cell_type": CellType.OBSTACLE})
    task = _task(Position(4, 4))

    assert calculate_bid(
        _uav("uav_01", Position(0, 0)),
        task,
        grid_map,
        {"auction": {"use_astar_for_bid": True}},
    ) is None


def test_bid_uses_astar_distance_when_configured() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    grid_map.set_cell(Position(1, 0), {"cell_type": CellType.OBSTACLE})
    task = _task(Position(2, 0))

    manhattan_bid = calculate_bid(_uav("uav_01", Position(0, 0)), task, grid_map, {"auction": {"use_astar_for_bid": False}})
    astar_bid = calculate_bid(_uav("uav_01", Position(0, 0)), task, grid_map, {"auction": {"use_astar_for_bid": True}})

    assert astar_bid is not None and manhattan_bid is not None
    assert astar_bid > manhattan_bid
