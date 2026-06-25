from __future__ import annotations

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
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
