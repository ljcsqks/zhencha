from __future__ import annotations

from uav_search.core.contracts import ControlCommand
from uav_search.core.data_types import CommandType, Position, Task, TaskStatus, TaskType, UAVState, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.grid_map import GridMap
from uav_search.simulation.command_applier import CommandApplier
from uav_search.uav.fleet_manager import FleetManager
from uav_search.uav.uav_model import UAV


def _config() -> dict:
    return {
        "algorithm": {"version": "baseline_sparse_boustrophedon"},
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
            "active_replan_min_interval_s": 0.0,
            "active_replan_low_gain_ratio": 0.95,
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


def _fleet() -> FleetManager:
    state = UAVState(
        id="uav_01",
        position=Position(0, 0),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=UAVStatus.SEARCHING,
        home_position=Position(0, 0),
        current_task_id="task_trim",
        path=[
            Position(0, 0),
            Position(0, 1),
            Position(1, 1),
            Position(2, 1),
            Position(3, 1),
            Position(4, 1),
            Position(4, 0),
        ],
        path_index=0,
        available=False,
    )
    return FleetManager([UAV(state, endurance_s=1000.0)])


def test_active_trim_returns_executable_replan_command_and_applier_updates_path() -> None:
    grid_map = GridMap(width_m=80, height_m=40, resolution_m=10)
    fleet = _fleet()
    scheduler = Scheduler(grid_map, fleet, _config())
    task = Task(
        id="task_trim",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(x, 0) for x in range(5)},
        entry_point=Position(0, 0),
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="uav_01",
        coverage_waypoints=[Position(x, 0) for x in range(5)],
        waypoints=[Position(x, 0) for x in range(5)],
    )
    scheduler.task_manager.add_tasks([task])
    for x in range(3):
        grid_map.mark_covered(Position(x, 0), radius_cells=1, timestamp=0.0)

    commands = scheduler._trim_redundant_active_search_paths(10.0, 0.95, force_replan=True)

    assert len(commands) == 1
    command = commands[0]
    assert command.command == CommandType.REPLAN
    assert command.reason == "active_search_trim_replan"
    assert command.path
    assert command.metadata["logical_waypoints"]
    assert command.path != fleet.get_uav("uav_01").state.path

    control = ControlCommand.from_decision(command, issued_at=10.0)
    ack = CommandApplier(fleet, grid_map).apply([control], now=10.0)[0]

    assert ack.status.value == "accepted"
    assert fleet.get_uav("uav_01").state.path == command.path
