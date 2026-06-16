from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager


@dataclass
class MetricsResult:
    run_id: str
    final_time_s: float
    global_coverage: float
    priority_coverage: float
    redundant_coverage_rate: float
    total_distance_m: float
    effective_search_distance_m: float
    path_efficiency: float
    min_battery: float
    event_count: int
    conflict_count: int
    no_fly_violations: int
    map_update_count: int
    target_found_count: int
    confirm_done_count: int
    time_to_95_coverage_s: float | None
    time_to_priority_coverage_s: float | None
    coverage_goal_met: bool
    priority_goal_met: bool
    supplemental_task_count: int
    ignored_uncovered_cells: int
    final_uncovered_cells: int
    final_priority_uncovered_cells: int
    post_95_extra_time_s: float | None
    post_95_extra_distance_m: float | None


def compute_metrics(
    run_id: str,
    grid_map: GridMap,
    fleet: FleetManager,
    snapshots: list[dict[str, Any]],
) -> MetricsResult:
    """Compute first-pass metrics from final state and recorded snapshots."""
    states = fleet.get_all_states()
    total_distance_m = sum(state.total_distance_m for state in states)
    effective_distance_m = sum(state.effective_search_distance_m for state in states)
    path_efficiency = effective_distance_m / total_distance_m if total_distance_m > 0 else 0.0
    event_ids = [event_id for snapshot in snapshots for event_id in snapshot.get("events", [])]
    time_to_95 = _first_time_reaching(snapshots, "global_coverage", 0.95)
    final_time_s = float(snapshots[-1]["time_s"]) if snapshots else 0.0
    final_uncovered_cells = len(grid_map.get_unsearched_cells(threshold=0.95))
    final_priority_uncovered_cells = _count_final_priority_uncovered(grid_map)
    supplemental_task_count = len(
        {
            uav.get("task_id")
            for snapshot in snapshots
            for uav in snapshot.get("uavs", [])
            if str(uav.get("task_id", "")).startswith("supplemental_")
        }
    )

    return MetricsResult(
        run_id=run_id,
        final_time_s=final_time_s,
        global_coverage=grid_map.coverage_rate(),
        priority_coverage=grid_map.coverage_rate(priority_only=True),
        redundant_coverage_rate=grid_map.redundant_coverage_rate(),
        total_distance_m=total_distance_m,
        effective_search_distance_m=effective_distance_m,
        path_efficiency=path_efficiency,
        min_battery=min((state.battery for state in states), default=0.0),
        event_count=len(event_ids),
        conflict_count=sum(1 for event_id in event_ids if "conflict" in event_id),
        no_fly_violations=_count_no_fly_violations(grid_map, snapshots),
        map_update_count=_count_events(event_ids, "scenario_map_update"),
        target_found_count=_count_events(event_ids, "scenario_target_found"),
        confirm_done_count=sum(1 for event_id in event_ids if event_id.startswith("confirm_done_")),
        time_to_95_coverage_s=time_to_95,
        time_to_priority_coverage_s=_first_time_reaching(snapshots, "priority_coverage", 0.95),
        coverage_goal_met=grid_map.coverage_rate() >= 0.92,
        priority_goal_met=grid_map.coverage_rate(priority_only=True) >= 0.98 or not grid_map.get_priority_cells(),
        supplemental_task_count=supplemental_task_count,
        ignored_uncovered_cells=final_uncovered_cells if grid_map.coverage_rate() >= 0.92 else 0,
        final_uncovered_cells=final_uncovered_cells,
        final_priority_uncovered_cells=final_priority_uncovered_cells,
        post_95_extra_time_s=None if time_to_95 is None else max(0.0, final_time_s - time_to_95),
        post_95_extra_distance_m=_post_threshold_distance(snapshots, time_to_95),
    )


def save_metrics(metrics: MetricsResult, path: str | Path) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(asdict(metrics), handle, ensure_ascii=False, indent=2)


def _first_time_reaching(snapshots: list[dict[str, Any]], field: str, threshold: float) -> float | None:
    for snapshot in snapshots:
        if float(snapshot.get(field, 0.0)) >= threshold:
            return float(snapshot["time_s"])
    return None


def _count_events(event_ids: list[str], prefix_or_id: str) -> int:
    return sum(1 for event_id in event_ids if event_id == prefix_or_id or event_id.startswith(prefix_or_id))


def _count_no_fly_violations(grid_map: GridMap, snapshots: list[dict[str, Any]]) -> int:
    from uav_search.core.data_types import CellType, Position

    violations = 0
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            pos_data = uav["position"]
            pos = Position(int(pos_data["x"]), int(pos_data["y"]))
            if grid_map.in_bounds(pos) and grid_map.get_cell(pos).cell_type == CellType.NO_FLY:
                violations += 1
    return violations


def _count_final_priority_uncovered(grid_map: GridMap) -> int:
    return sum(
        1
        for cell in grid_map.get_priority_cells()
        if grid_map.get_cell(cell).search_confidence < 0.95
    )


def _post_threshold_distance(snapshots: list[dict[str, Any]], threshold_time_s: float | None) -> float | None:
    if threshold_time_s is None or not snapshots:
        return None
    threshold_snapshot = next((snapshot for snapshot in snapshots if float(snapshot["time_s"]) >= threshold_time_s), None)
    if threshold_snapshot is None:
        return None
    final_distance = _snapshot_total_distance(snapshots[-1])
    threshold_distance = _snapshot_total_distance(threshold_snapshot)
    return max(0.0, final_distance - threshold_distance)


def _snapshot_total_distance(snapshot: dict[str, Any]) -> float:
    return sum(float(uav.get("total_distance_m", 0.0)) for uav in snapshot.get("uavs", []))
