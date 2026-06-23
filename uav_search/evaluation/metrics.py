from __future__ import annotations

import json
import math
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
    turn_rate: float
    replan_count: int
    coverage_gain_per_meter: float
    per_uav_workload_balance: float
    target_response_time_s: float | None
    target_confirm_duration_s: float | None
    confirm_success_rate: float
    search_resume_delay_s: float | None
    coverage_gap_at_confirm_done: float
    interrupted_task_resume_rate: float
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
    mission_complete_coverage_threshold: float = 0.95,
) -> MetricsResult:
    """Compute first-pass metrics from final state and recorded snapshots."""
    states = fleet.get_all_states()
    total_distance_m = sum(state.total_distance_m for state in states)
    effective_distance_m = sum(state.effective_search_distance_m for state in states)
    path_efficiency = effective_distance_m / total_distance_m if total_distance_m > 0 else 0.0
    event_ids = [event_id for snapshot in snapshots for event_id in snapshot.get("events", [])]
    time_to_95 = _first_time_reaching(snapshots, "global_coverage", 0.95)
    final_time_s = float(snapshots[-1]["time_s"]) if snapshots else 0.0
    initial_coverage = float(snapshots[0].get("global_coverage", 0.0)) if snapshots else 0.0
    final_uncovered_cells = len(grid_map.get_unsearched_cells(threshold=0.95))
    final_priority_uncovered_cells = _count_final_priority_uncovered(grid_map)
    target_metrics = _latest_target_metrics(snapshots)
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
        target_found_count=_target_found_count(target_metrics, event_ids, snapshots),
        confirm_done_count=sum(1 for event_id in event_ids if event_id.startswith("confirm_done_")),
        time_to_95_coverage_s=time_to_95,
        time_to_priority_coverage_s=_first_time_reaching(snapshots, "priority_coverage", 0.95),
        coverage_goal_met=grid_map.coverage_rate() >= mission_complete_coverage_threshold,
        priority_goal_met=grid_map.coverage_rate(priority_only=True) >= 0.98 or not grid_map.get_priority_cells(),
        turn_rate=_turn_rate(snapshots, total_distance_m),
        replan_count=_replan_count(snapshots, event_ids),
        coverage_gain_per_meter=(grid_map.coverage_rate() - initial_coverage) / total_distance_m if total_distance_m > 0 else 0.0,
        per_uav_workload_balance=_workload_balance([state.total_distance_m for state in states]),
        target_response_time_s=_avg_target_delta(target_metrics, "found_time_s", "assigned_time_s"),
        target_confirm_duration_s=_avg_target_delta(target_metrics, "assigned_time_s", "done_time_s"),
        confirm_success_rate=_confirm_success_rate(target_metrics),
        search_resume_delay_s=_avg_target_delta(target_metrics, "done_time_s", "resumed_time_s", require_interrupted=True),
        coverage_gap_at_confirm_done=_coverage_gap_at_confirm_done(target_metrics, mission_complete_coverage_threshold),
        interrupted_task_resume_rate=_interrupted_task_resume_rate(target_metrics),
        supplemental_task_count=supplemental_task_count,
        ignored_uncovered_cells=final_uncovered_cells if grid_map.coverage_rate() >= mission_complete_coverage_threshold else 0,
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


def _target_found_count(
    target_metrics: dict[str, dict[str, Any]],
    event_ids: list[str],
    snapshots: list[dict[str, Any]],
) -> int:
    if target_metrics:
        return len(target_metrics)
    command_targets = {
        command.get("task_id")
        for snapshot in snapshots
        for command in snapshot.get("commands", [])
        if command.get("command") == "CONFIRM_TARGET"
    }
    if command_targets:
        return len(command_targets)
    return sum(1 for event_id in event_ids if "target_found" in event_id.lower())


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


def _replan_count(snapshots: list[dict[str, Any]], event_ids: list[str]) -> int:
    snapshot_count = max((int(snapshot.get("replan_count", 0)) for snapshot in snapshots), default=0)
    event_count = sum(1 for event_id in event_ids if "replan" in event_id or "map_update" in event_id)
    return max(snapshot_count, event_count)


def _turn_rate(snapshots: list[dict[str, Any]], total_distance_m: float) -> float:
    if total_distance_m <= 0:
        return 0.0
    previous_vectors: dict[str, tuple[int, int]] = {}
    previous_positions: dict[str, tuple[int, int]] = {}
    turns = 0
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            uav_id = str(uav.get("id"))
            pos_data = uav.get("position", {})
            current = (int(pos_data.get("x", 0)), int(pos_data.get("y", 0)))
            previous = previous_positions.get(uav_id)
            if previous is None:
                previous_positions[uav_id] = current
                continue
            vector = (current[0] - previous[0], current[1] - previous[1])
            if vector == (0, 0):
                continue
            old_vector = previous_vectors.get(uav_id)
            if old_vector is not None and _normalized_step(vector) != _normalized_step(old_vector):
                turns += 1
            previous_vectors[uav_id] = vector
            previous_positions[uav_id] = current
    return turns / total_distance_m


def _normalized_step(vector: tuple[int, int]) -> tuple[int, int]:
    dx, dy = vector
    return (
        0 if dx == 0 else int(math.copysign(1, dx)),
        0 if dy == 0 else int(math.copysign(1, dy)),
    )


def _workload_balance(distances: list[float]) -> float:
    active = [distance for distance in distances if distance > 0]
    if not active:
        return 1.0
    mean = sum(active) / len(active)
    if mean <= 0:
        return 1.0
    variance = sum((distance - mean) ** 2 for distance in active) / len(active)
    coefficient_of_variation = math.sqrt(variance) / mean
    return 1.0 / (1.0 + coefficient_of_variation)


def _latest_target_metrics(snapshots: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    for snapshot in reversed(snapshots):
        metrics = snapshot.get("target_metrics")
        if isinstance(metrics, dict):
            return {str(target_id): dict(record) for target_id, record in metrics.items() if isinstance(record, dict)}
    return {}


def _avg_target_delta(
    target_metrics: dict[str, dict[str, Any]],
    start_key: str,
    end_key: str,
    require_interrupted: bool = False,
) -> float | None:
    values: list[float] = []
    for record in target_metrics.values():
        if require_interrupted and not record.get("interrupted_task_id"):
            continue
        start = record.get(start_key)
        end = record.get(end_key)
        if start is None or end is None:
            continue
        values.append(max(0.0, float(end) - float(start)))
    if not values:
        return None
    return sum(values) / len(values)


def _confirm_success_rate(target_metrics: dict[str, dict[str, Any]]) -> float:
    if not target_metrics:
        return 0.0
    successes = sum(1 for record in target_metrics.values() if bool(record.get("success")))
    return successes / len(target_metrics)


def _coverage_gap_at_confirm_done(
    target_metrics: dict[str, dict[str, Any]],
    mission_threshold: float,
) -> float:
    gaps: list[float] = []
    for record in target_metrics.values():
        if not record.get("interrupted_task_id"):
            continue
        coverage_at_done = record.get("coverage_at_done")
        if coverage_at_done is None:
            continue
        gaps.append(max(0.0, mission_threshold - float(coverage_at_done)))
    if not gaps:
        return 0.0
    return sum(gaps) / len(gaps)


def _interrupted_task_resume_rate(target_metrics: dict[str, dict[str, Any]]) -> float:
    interrupted = [record for record in target_metrics.values() if record.get("interrupted_task_id")]
    if not interrupted:
        return 1.0
    resumed = sum(1 for record in interrupted if record.get("resumed_time_s") is not None)
    return resumed / len(interrupted)
