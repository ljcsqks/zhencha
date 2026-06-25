from __future__ import annotations

import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any

from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager


def algorithm_version(config: dict[str, Any] | None = None, override: str | None = None) -> str:
    if override:
        return override
    return str((config or {}).get("algorithm", {}).get("version", "baseline_sparse_boustrophedon"))


def code_version() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=Path(__file__).resolve().parents[2],
            capture_output=True,
            text=True,
            check=True,
            timeout=2,
        )
        return result.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def config_hash(config: dict[str, Any] | None = None) -> str:
    payload = json.dumps(config or {}, sort_keys=True, ensure_ascii=False, default=str)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]


def compute_diagnostics(
    grid_map: GridMap,
    fleet: FleetManager,
    snapshots: list[dict[str, Any]],
) -> dict[str, Any]:
    per_uav = _per_uav_diagnostics(fleet, snapshots)
    route_quality = _route_quality(snapshots)
    coverage_quality = _coverage_quality(grid_map, snapshots)
    allocation_quality = _allocation_quality(per_uav, snapshots)
    return {
        "per_uav": per_uav,
        "route_quality": route_quality,
        "coverage_quality": coverage_quality,
        "allocation_quality": allocation_quality,
    }


def _per_uav_diagnostics(fleet: FleetManager, snapshots: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    ids = [state.id for state in fleet.get_all_states()]
    result: dict[str, dict[str, Any]] = {}
    coverage_by_uav: dict[str, set[tuple[int, int]]] = {uav_id: set() for uav_id in ids}
    repeat_by_uav: dict[str, int] = {uav_id: 0 for uav_id in ids}
    active_time: dict[str, float] = {uav_id: 0.0 for uav_id in ids}
    idle_time: dict[str, float] = {uav_id: 0.0 for uav_id in ids}
    confirm_time: dict[str, float] = {uav_id: 0.0 for uav_id in ids}
    turn_count: dict[str, int] = {uav_id: 0 for uav_id in ids}
    previous_pos: dict[str, tuple[int, int]] = {}
    previous_vec: dict[str, tuple[int, int]] = {}

    for snapshot in snapshots:
        dt = 1.0
        for cell in snapshot.get("coverage_changed_cells", []):
            owners = _covering_uavs(snapshot, int(cell.get("x", 0)), int(cell.get("y", 0)))
            if not owners:
                continue
            owner = owners[0]
            key = (int(cell.get("x", 0)), int(cell.get("y", 0)))
            if key in coverage_by_uav.setdefault(owner, set()):
                repeat_by_uav[owner] = repeat_by_uav.get(owner, 0) + 1
            coverage_by_uav[owner].add(key)
        for uav in snapshot.get("uavs", []):
            uav_id = str(uav.get("id"))
            status = str(uav.get("status", ""))
            if status == "IDLE":
                idle_time[uav_id] = idle_time.get(uav_id, 0.0) + dt
            elif status == "CONFIRMING":
                confirm_time[uav_id] = confirm_time.get(uav_id, 0.0) + dt
                active_time[uav_id] = active_time.get(uav_id, 0.0) + dt
            elif status != "OFFLINE":
                active_time[uav_id] = active_time.get(uav_id, 0.0) + dt
            pos = uav.get("position", {})
            current = (int(pos.get("x", 0)), int(pos.get("y", 0)))
            if uav_id in previous_pos:
                vec = (current[0] - previous_pos[uav_id][0], current[1] - previous_pos[uav_id][1])
                if vec != (0, 0):
                    old_vec = previous_vec.get(uav_id)
                    if old_vec is not None and _norm(vec) != _norm(old_vec):
                        turn_count[uav_id] = turn_count.get(uav_id, 0) + 1
                    previous_vec[uav_id] = vec
            previous_pos[uav_id] = current

    final_distances = _final_distances(snapshots)
    final_replans = max((int(snapshot.get("replan_count", 0)) for snapshot in snapshots), default=0)
    for state in fleet.get_all_states():
        uav_id = state.id
        distance_m = float(final_distances.get(uav_id, state.total_distance_m))
        newly = len(coverage_by_uav.get(uav_id, set()))
        result[uav_id] = {
            "assigned_area_cells": newly,
            "newly_covered_cells": newly,
            "repeated_covered_cells": repeat_by_uav.get(uav_id, 0),
            "distance_m": distance_m,
            "active_time_s": active_time.get(uav_id, 0.0),
            "idle_time_s": idle_time.get(uav_id, 0.0),
            "confirm_time_s": confirm_time.get(uav_id, 0.0),
            "replan_count": final_replans,
            "turn_count": turn_count.get(uav_id, 0),
            "average_coverage_gain_per_meter": newly / distance_m if distance_m > 0 else 0.0,
        }
    return result


def _route_quality(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    connectors: list[float] = []
    total_turns = 0
    total_distance = 0.0
    for snapshot in snapshots:
        total_distance = max(total_distance, _snapshot_total_distance(snapshot))
        for command in snapshot.get("commands", []):
            path = command.get("path") or []
            if len(path) < 2:
                continue
            connectors.extend(_segment_lengths(path))
    long_connectors = [value for value in connectors if value > 5.0]
    total_turns = _turn_count(snapshots)
    turn_rate = total_turns / total_distance if total_distance > 0 else 0.0
    return {
        "approximate_crossing_count": 0,
        "crossing_count": 0,
        "long_connector_count": len(long_connectors),
        "average_connector_length": sum(connectors) / len(connectors) if connectors else 0.0,
        "max_connector_length": max(connectors, default=0.0),
        "path_smoothness_score": 1.0 / (1.0 + turn_rate),
        "turn_rate": turn_rate,
    }


def _coverage_quality(grid_map: GridMap, snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    at_95 = next((snapshot for snapshot in snapshots if float(snapshot.get("global_coverage", 0.0)) >= 0.95), None)
    post_95_cells: set[tuple[int, int]] = set()
    post_95_distance = 0.0
    if at_95 is not None and snapshots:
        threshold_time = float(at_95.get("time_s", 0.0))
        threshold_distance = _snapshot_total_distance(at_95)
        post_95_distance = max(0.0, _snapshot_total_distance(snapshots[-1]) - threshold_distance)
        for snapshot in snapshots:
            if float(snapshot.get("time_s", 0.0)) > threshold_time:
                post_95_cells.update((int(cell["x"]), int(cell["y"])) for cell in snapshot.get("coverage_changed_cells", []))
    components = _uncovered_components(grid_map)
    return {
        "uncovered_components_count_at_95": len(components),
        "largest_uncovered_component_at_95": max((len(component) for component in components), default=0),
        "priority_uncovered_cells": _priority_uncovered(grid_map),
        "post_95_new_coverage_cells": len(post_95_cells),
        "post_95_distance_m": post_95_distance,
    }


def _allocation_quality(per_uav: dict[str, dict[str, Any]], snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    actual_costs = {uav_id: float(data.get("distance_m", 0.0)) for uav_id, data in per_uav.items()}
    active_finish_times = _finish_times(snapshots)
    return {
        "route_cost_estimate_per_uav": dict(actual_costs),
        "actual_cost_per_uav": actual_costs,
        "workload_balance": _workload_balance(list(actual_costs.values())),
        "max_uav_finish_time": max(active_finish_times.values(), default=0.0),
        "min_uav_finish_time": min(active_finish_times.values(), default=0.0),
        "idle_uav_count_after_50_percent_coverage": _idle_count_after(snapshots, 0.5),
        "idle_uav_count_after_80_percent_coverage": _idle_count_after(snapshots, 0.8),
    }


def _covering_uavs(snapshot: dict[str, Any], x: int, y: int) -> list[str]:
    owners = []
    for uav in snapshot.get("uavs", []):
        pos = uav.get("position", {})
        if abs(int(pos.get("x", 0)) - x) <= 2 and abs(int(pos.get("y", 0)) - y) <= 2:
            owners.append(str(uav.get("id")))
    return owners


def _final_distances(snapshots: list[dict[str, Any]]) -> dict[str, float]:
    if not snapshots:
        return {}
    return {str(uav.get("id")): float(uav.get("total_distance_m", 0.0)) for uav in snapshots[-1].get("uavs", [])}


def _segment_lengths(path: list[dict[str, Any]]) -> list[float]:
    values: list[float] = []
    for a, b in zip(path, path[1:]):
        values.append(math.hypot(float(b.get("x", 0)) - float(a.get("x", 0)), float(b.get("y", 0)) - float(a.get("y", 0))))
    return values


def _turn_count(snapshots: list[dict[str, Any]]) -> int:
    previous_pos: dict[str, tuple[int, int]] = {}
    previous_vec: dict[str, tuple[int, int]] = {}
    turns = 0
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            uav_id = str(uav.get("id"))
            pos = uav.get("position", {})
            current = (int(pos.get("x", 0)), int(pos.get("y", 0)))
            if uav_id in previous_pos:
                vec = (current[0] - previous_pos[uav_id][0], current[1] - previous_pos[uav_id][1])
                if vec != (0, 0):
                    old = previous_vec.get(uav_id)
                    if old is not None and _norm(vec) != _norm(old):
                        turns += 1
                    previous_vec[uav_id] = vec
            previous_pos[uav_id] = current
    return turns


def _norm(vector: tuple[int, int]) -> tuple[int, int]:
    return (
        0 if vector[0] == 0 else int(math.copysign(1, vector[0])),
        0 if vector[1] == 0 else int(math.copysign(1, vector[1])),
    )


def _snapshot_total_distance(snapshot: dict[str, Any]) -> float:
    return sum(float(uav.get("total_distance_m", 0.0)) for uav in snapshot.get("uavs", []))


def _uncovered_components(grid_map: GridMap) -> list[set[tuple[int, int]]]:
    unvisited = {(cell.x, cell.y) for cell in grid_map.get_unsearched_cells(threshold=0.95)}
    components: list[set[tuple[int, int]]] = []
    while unvisited:
        seed = unvisited.pop()
        component = {seed}
        stack = [seed]
        while stack:
            x, y = stack.pop()
            for neighbor in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components


def _priority_uncovered(grid_map: GridMap) -> int:
    return sum(1 for cell in grid_map.get_priority_cells() if grid_map.get_cell(cell).search_confidence < 0.95)


def _finish_times(snapshots: list[dict[str, Any]]) -> dict[str, float]:
    finish: dict[str, float] = {}
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            if str(uav.get("status")) not in {"IDLE", "OFFLINE"}:
                finish[str(uav.get("id"))] = float(snapshot.get("time_s", 0.0))
    return finish


def _idle_count_after(snapshots: list[dict[str, Any]], threshold: float) -> int:
    relevant = next((snapshot for snapshot in snapshots if float(snapshot.get("global_coverage", 0.0)) >= threshold), None)
    if relevant is None:
        return 0
    return sum(1 for uav in relevant.get("uavs", []) if str(uav.get("status")) == "IDLE")


def _workload_balance(values: list[float]) -> float:
    active = [value for value in values if value > 0]
    if not active:
        return 1.0
    mean = sum(active) / len(active)
    if mean <= 0:
        return 1.0
    variance = sum((value - mean) ** 2 for value in active) / len(active)
    return 1.0 / (1.0 + math.sqrt(variance) / mean)
