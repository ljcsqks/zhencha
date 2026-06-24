from __future__ import annotations

from dataclasses import asdict
from typing import Any

from uav_search.evaluation.metrics import compute_metrics
from uav_search.maps.grid_map import GridMap
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager


def build_state(
    *,
    simulator: Simulator,
    grid_map: GridMap,
    fleet: FleetManager,
    scheduler,
    scenario_name: str,
    running: bool,
    include_map: bool = True,
) -> dict[str, Any]:
    latest = simulator.snapshots[-1] if simulator.snapshots else {}
    state = {
        "time_s": simulator.time_s,
        "tick": simulator._tick,
        "running": running,
        "scenario_name": scenario_name,
        "global_coverage": grid_map.coverage_rate(),
        "priority_coverage": grid_map.coverage_rate(priority_only=True),
        "uavs": latest.get("uavs", _uavs_from_fleet(fleet)),
        "commands": latest.get("commands", []),
        "command_acks": latest.get("command_acks", []),
        "events": latest.get("events", []),
        "advisory_summary": latest.get("advisory_summary", {}),
        "tasks": scheduler.task_status_snapshot(),
        "targets": scheduler.target_metrics_snapshot(),
        "changed_cells": latest.get("changed_cells", []),
        "metrics": metrics_summary(simulator, grid_map, fleet, scenario_name),
    }
    if include_map:
        state["map"] = map_state(grid_map)
    return state


def metrics_summary(simulator: Simulator, grid_map: GridMap, fleet: FleetManager, scenario_name: str) -> dict[str, Any]:
    if not simulator.snapshots:
        return {
            "global_coverage": grid_map.coverage_rate(),
            "priority_coverage": grid_map.coverage_rate(priority_only=True),
            "total_distance_m": sum(state.total_distance_m for state in fleet.get_all_states()),
            "no_fly_violations": 0,
        }
    metrics = compute_metrics(
        scenario_name,
        grid_map,
        fleet,
        simulator.snapshots,
        mission_complete_coverage_threshold=0.95,
    )
    return asdict(metrics)


def map_state(grid_map: GridMap) -> dict[str, Any]:
    terrain: list[list[str]] = []
    passable: list[list[bool]] = []
    coverage_count: list[list[int]] = []
    search_confidence: list[list[float]] = []
    search_priority: list[list[float]] = []
    for y in range(grid_map.height_cells):
        terrain_row: list[str] = []
        passable_row: list[bool] = []
        coverage_row: list[int] = []
        confidence_row: list[float] = []
        priority_row: list[float] = []
        for x in range(grid_map.width_cells):
            terrain_row.append(str(grid_map.terrain[y, x]))
            passable_row.append(bool(grid_map.passable[y, x]))
            coverage_row.append(int(grid_map.coverage_count[y, x]))
            confidence_row.append(float(grid_map.search_confidence[y, x]))
            priority_row.append(float(grid_map.search_priority[y, x]))
        terrain.append(terrain_row)
        passable.append(passable_row)
        coverage_count.append(coverage_row)
        search_confidence.append(confidence_row)
        search_priority.append(priority_row)
    return {
        "width_cells": grid_map.width_cells,
        "height_cells": grid_map.height_cells,
        "resolution_m": grid_map.resolution_m,
        "terrain": terrain,
        "passable": passable,
        "coverage_count": coverage_count,
        "search_confidence": search_confidence,
        "search_priority": search_priority,
    }


def _uavs_from_fleet(fleet: FleetManager) -> list[dict[str, Any]]:
    return [
        {
            "id": state.id,
            "position": asdict(state.position),
            "status": state.status.value,
            "battery": state.battery,
            "task_id": state.current_task_id,
            "total_distance_m": state.total_distance_m,
            "effective_search_distance_m": state.effective_search_distance_m,
        }
        for state in fleet.get_all_states()
    ]
