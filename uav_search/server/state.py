from __future__ import annotations

from dataclasses import asdict
from typing import Any

from uav_search.server.algorithms import DEFAULT_ALGORITHM_VERSION, algorithms_payload
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
    config: dict[str, Any],
    scenario_name: str,
    running: bool,
    run_id: str,
    include_map: bool = True,
    state_level: str = "full",
    pending_events: list[dict[str, Any]] | None = None,
    recent_events: list[dict[str, Any]] | None = None,
    event_log: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    latest = simulator.snapshots[-1] if simulator.snapshots else {}
    include_full_map = include_map and state_level == "full"
    algorithm_version = str(config.get("algorithm", {}).get("version", DEFAULT_ALGORITHM_VERSION))
    state = {
        "time_s": simulator.time_s,
        "tick": simulator._tick,
        "running": running,
        "run_id": run_id,
        "scenario_name": scenario_name,
        "algorithm_version": algorithm_version,
        "available_algorithm_versions": [item["version"] for item in algorithms_payload()["algorithms"]],
        "global_coverage": grid_map.coverage_rate(),
        "priority_coverage": grid_map.coverage_rate(priority_only=True),
        "uavs": latest.get("uavs", _uavs_from_fleet(fleet)),
        "commands": latest.get("commands", []),
        "command_acks": latest.get("command_acks", []),
        "events": latest.get("events", []),
        "advisory_summary": latest.get("advisory_summary", {}),
        "tasks": scheduler.task_status_snapshot(),
        "targets": scheduler.target_metrics_snapshot(),
        "diagnostics": {"reachability": scheduler.reachability_diagnostics()},
        "changed_cells": latest.get("changed_cells", []),
        "coverage_changed_cells": latest.get("coverage_changed_cells", []),
        "active_commands": latest.get("active_commands", []),
        "pending_events": list(pending_events or []),
        "recent_events": list(recent_events or []),
        "event_log": list(event_log or []),
        "metrics": metrics_summary(simulator, grid_map, fleet, scenario_name, config, state_level),
    }
    if include_full_map:
        state["map"] = map_state(grid_map)
    return state


def metrics_summary(
    simulator: Simulator,
    grid_map: GridMap,
    fleet: FleetManager,
    scenario_name: str,
    config: dict[str, Any],
    state_level: str = "full",
) -> dict[str, Any]:
    if state_level != "full":
        summary = lightweight_metrics_summary(
            simulator,
            grid_map,
            fleet,
            mission_complete_coverage_threshold=float(
                config.get("search", {}).get("mission_complete_coverage_threshold", 0.95)
            ),
        )
        summary["algorithm_version"] = str(config.get("algorithm", {}).get("version", DEFAULT_ALGORITHM_VERSION))
        return summary
    if not simulator.snapshots:
        return {
            "global_coverage": grid_map.coverage_rate(),
            "priority_coverage": grid_map.coverage_rate(priority_only=True),
            "total_distance_m": sum(state.total_distance_m for state in fleet.get_all_states()),
            "no_fly_violations": 0,
            "algorithm_version": str(config.get("algorithm", {}).get("version", DEFAULT_ALGORITHM_VERSION)),
        }
    metrics = compute_metrics(
        scenario_name,
        grid_map,
        fleet,
        simulator.snapshots,
        mission_complete_coverage_threshold=float(
            config.get("search", {}).get("mission_complete_coverage_threshold", 0.95)
        ),
        config=config,
    )
    return asdict(metrics)


def lightweight_metrics_summary(
    simulator: Simulator,
    grid_map: GridMap,
    fleet: FleetManager,
    mission_complete_coverage_threshold: float,
) -> dict[str, Any]:
    latest_metrics = _latest_target_metrics(simulator)
    global_coverage = grid_map.coverage_rate()
    priority_coverage = grid_map.coverage_rate(priority_only=True)
    return {
        "global_coverage": global_coverage,
        "priority_coverage": priority_coverage,
        "coverage_goal_met": global_coverage >= mission_complete_coverage_threshold,
        "total_distance_m": sum(state.total_distance_m for state in fleet.get_all_states()),
        "no_fly_violations": 0,
        "confirm_success_rate": latest_metrics.get("confirm_success_rate", 0.0),
        "target_response_time_s": latest_metrics.get("target_response_time_s"),
        "target_confirm_duration_s": latest_metrics.get("target_confirm_duration_s"),
        "running_command_count": len(simulator.command_applier.active_commands_snapshot()),
    }


def _latest_target_metrics(simulator: Simulator) -> dict[str, Any]:
    if not simulator.snapshots:
        return {}
    target_metrics = simulator.snapshots[-1].get("target_metrics", {})
    if not target_metrics:
        return {}
    successes = [item for item in target_metrics.values() if item.get("success") is True]
    response_times = [
        item.get("response_time_s")
        for item in target_metrics.values()
        if isinstance(item.get("response_time_s"), (int, float))
    ]
    durations = [
        item.get("confirm_duration_s")
        for item in target_metrics.values()
        if isinstance(item.get("confirm_duration_s"), (int, float))
    ]
    return {
        "confirm_success_rate": len(successes) / len(target_metrics) if target_metrics else 0.0,
        "target_response_time_s": min(response_times) if response_times else None,
        "target_confirm_duration_s": min(durations) if durations else None,
    }


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
