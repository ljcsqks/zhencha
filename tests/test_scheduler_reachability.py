from __future__ import annotations

from uav_search.core.data_types import CellType, CommandType, Position, Task, TaskStatus, TaskType, UAVState, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager
from uav_search.uav.uav_model import UAV


def _config() -> dict:
    return {
        "uav": {
            "count": 1,
            "battery_threshold": 0.2,
            "sensor_radius_cells": 1,
            "max_speed_mps": 10.0,
            "endurance_s": 1000.0,
        },
        "search": {
            "coverage_complete_threshold": 0.95,
            "mission_complete_coverage_threshold": 0.95,
            "priority_complete_threshold": 0.98,
            "supplemental_task_max_cells": 20,
            "supplemental_cluster_max_cells": 20,
            "supplemental_cluster_radius_cells": 0,
            "blocked_region_cache_ttl_s": 60.0,
            "min_supplemental_cells": 1,
            "min_supplemental_score": 0.0,
            "large_supplemental_region_cells": 1,
            "post_goal_ordinary_min_cells": 1,
            "distance_cost_weight": 1.0,
            "uncovered_value_weight": 1.0,
            "priority_value_weight": 2.0,
            "priority_cell_weight": 3.0,
        },
        "auction": {"use_astar_for_bid": True, "w_distance": 1.0, "w_battery": 0.3, "w_balance": 0.2},
        "planning": {
            "obstacle_proximity_penalty": 0.5,
            "priority_area_bonus": -0.2,
            "safety_distance_cells": 2,
            "conflict_time_horizon_steps": 10,
        },
        "scheduler": {"event_debounce_s": 0.0},
        "simulation": {"time_step_s": 1.0},
    }


def _fleet(position: Position = Position(0, 0)) -> FleetManager:
    state = UAVState(
        id="uav_01",
        position=position,
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=UAVStatus.IDLE,
        home_position=position,
    )
    return FleetManager([UAV(state, endurance_s=1000.0)])


def _wall(grid_map: GridMap) -> None:
    for y in range(grid_map.height_cells):
        grid_map.set_cell(Position(2, y), {"cell_type": CellType.OBSTACLE})


def test_initial_tasks_exclude_cells_unreachable_by_all_uavs() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    _wall(grid_map)
    scheduler = Scheduler(grid_map, _fleet(), _config())

    scheduler._ensure_initial_tasks(now=0.0)

    task_cells = {cell for task in scheduler.task_manager.tasks.values() for cell in task.target_cells}
    assert Position(1, 4) in task_cells
    assert Position(4, 4) not in task_cells
    diagnostics = scheduler.reachability_diagnostics()
    assert diagnostics["unreachable_cells_count"] > 0
    assert diagnostics["unreachable_components_count"] == 1


def test_blocked_region_cache_suppresses_repeated_supplemental_candidate() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    scheduler = Scheduler(grid_map, _fleet(), _config())
    region = {Position(1, 1), Position(1, 2)}
    task = next(iter(scheduler.task_manager.tasks.values()), None)
    if task is None:
        from uav_search.core.data_types import Task, TaskType

        task = Task(
            id="blocked",
            type=TaskType.SEARCH,
            priority=1.0,
            target_cells=set(region),
            entry_point=Position(1, 1),
            coverage_waypoints=[Position(1, 1), Position(1, 2)],
        )
    scheduler._remember_blocked_region(task, now=10.0)
    scheduler._last_decision_time = 11.0

    assert scheduler._is_region_blocked(region, now=11.0)
    scheduler._last_decision_time = 80.0
    assert not scheduler._is_region_blocked(region, now=80.0)


def test_supplemental_tasks_wait_until_pending_search_tasks_are_allocated() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    scheduler = Scheduler(grid_map, _fleet(), _config())
    scheduler._ensure_initial_tasks(now=0.0)
    initial_count = len(scheduler.task_manager.tasks)

    scheduler._ensure_supplemental_search_tasks(now=0.0)

    assert len(scheduler.task_manager.tasks) == initial_count


def test_post_goal_does_not_create_ordinary_supplemental_and_cancels_pending() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for cell in grid_map.get_searchable_cells():
        grid_map.set_cell(cell, {"search_confidence": 1.0})
    scheduler = Scheduler(grid_map, _fleet(), _config())
    pending = Task(
        id="supplemental_001",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(1, 1)},
        entry_point=Position(1, 1),
        coverage_waypoints=[Position(1, 1)],
        metadata={"supplemental": True},
    )
    scheduler.task_manager.add_tasks([pending])

    scheduler._ensure_supplemental_search_tasks(now=5.0)

    assert scheduler.task_manager.tasks["supplemental_001"].status == TaskStatus.CANCELLED
    assert scheduler.diagnostics_snapshot()["skipped_post_goal_supplemental_tasks"] >= 1


def test_post_goal_active_ordinary_search_gets_hold() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for cell in grid_map.get_searchable_cells():
        grid_map.set_cell(cell, {"search_confidence": 1.0})
    fleet = _fleet(Position(0, 0))
    uav = fleet.get_uav("uav_01").state
    uav.status = UAVStatus.SEARCHING
    uav.current_task_id = "search_001"
    uav.path = [Position(0, 0), Position(1, 0)]
    scheduler = Scheduler(grid_map, fleet, _config())
    task = Task(
        id="search_001",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(1, 0)},
        entry_point=Position(1, 0),
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="uav_01",
        coverage_waypoints=[Position(1, 0)],
    )
    scheduler.task_manager.add_tasks([task])

    commands = scheduler._stop_search_tasks_after_coverage_goal(now=5.0)

    assert commands
    assert commands[0].command == CommandType.HOLD
    assert scheduler.diagnostics_snapshot()["post_goal_active_search_cancel_count"] == 1


def test_post_goal_keeps_priority_remaining_task() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for cell in grid_map.get_searchable_cells():
        grid_map.set_cell(cell, {"search_confidence": 1.0})
    priority_cell = Position(2, 2)
    grid_map.set_cell(priority_cell, {"search_priority": 3.0, "search_confidence": 0.0})
    scheduler = Scheduler(grid_map, _fleet(), _config())
    task = Task(
        id="priority_001",
        type=TaskType.SEARCH,
        priority=3.0,
        target_cells={priority_cell},
        entry_point=priority_cell,
        coverage_waypoints=[priority_cell],
    )
    scheduler.task_manager.add_tasks([task])

    tasks = scheduler._allocatable_pending_tasks()

    assert [item.id for item in tasks] == ["priority_001"]
