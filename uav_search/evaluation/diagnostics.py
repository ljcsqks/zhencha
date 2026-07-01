from __future__ import annotations

import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any

from uav_search.core.data_types import Position
from uav_search.maps.grid_map import GridMap
from uav_search.planning.reachability import build_reachability_index
from uav_search.planning.reachability import connected_components as reachability_components
from uav_search.uav.fleet_manager import FleetManager


def algorithm_version(config: dict[str, Any] | None = None, override: str | None = None) -> str:
    if override:
        return override
    return str((config or {}).get("algorithm", {}).get("version", "adaptive_component_sweep_v1"))


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
    reachability_quality = _reachability_quality(grid_map, fleet)
    coverage_quality.update(reachability_quality)
    allocation_quality = _allocation_quality(per_uav, snapshots)
    segment_quality = _segment_quality(snapshots)
    fleet_planned = _fleet_planned_coverage_quality(grid_map, snapshots)
    segment_quality.update(fleet_planned)
    planned_ratio = float(segment_quality.get("fleet_planned_coverage_ratio", 0.0) or segment_quality.get("planned_coverage_ratio", 0.0) or 0.0)
    actual_ratio = grid_map.coverage_rate()
    segment_quality["actual_final_coverage_ratio"] = actual_ratio
    segment_quality["planned_vs_actual_coverage_error"] = actual_ratio - planned_ratio if planned_ratio > 0 else 0.0
    segment_quality["fleet_planned_vs_actual_coverage_error"] = (
        actual_ratio - float(segment_quality.get("fleet_planned_coverage_ratio", 0.0))
        if float(segment_quality.get("fleet_planned_coverage_ratio", 0.0)) > 0
        else 0.0
    )
    planned_error = float(segment_quality.get("fleet_planned_vs_actual_coverage_error", 0.0) or 0.0)
    segment_quality["planned_actual_gap_abs"] = abs(planned_error)
    segment_quality["planned_vs_actual_explanation"] = _planned_vs_actual_explanation(planned_error, planned_ratio)
    command_quality = _command_quality(snapshots)
    scheduler_quality = _scheduler_quality(snapshots)
    return {
        "per_uav": per_uav,
        "route_quality": route_quality,
        "coverage_quality": coverage_quality,
        "allocation_quality": allocation_quality,
        "segment_quality": segment_quality,
        "command_quality": command_quality,
        "scheduler_quality": scheduler_quality,
    }


def _planned_vs_actual_explanation(error: float, planned_ratio: float) -> str:
    if planned_ratio <= 0:
        return "planned coverage unavailable; this run may use baseline-style tasks without adaptive metadata"
    if abs(error) <= 0.03:
        return "actual coverage is close to planned coverage"
    if error > 0:
        return "actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells"
    return "actual is below plan because cancellations, blocked paths, or dynamic updates can prevent planned cells from being visited"


def _reachability_quality(grid_map: GridMap, fleet: FleetManager) -> dict[str, Any]:
    index = build_reachability_index(grid_map, fleet.get_all_states())
    components = reachability_components(grid_map, index.unreachable_searchable_cells)
    return {
        "unreachable_cells_count": len(index.unreachable_searchable_cells),
        "unreachable_components_count": len(components),
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
        active_s = active_time.get(uav_id, 0.0)
        idle_s = idle_time.get(uav_id, 0.0)
        observed_s = active_s + idle_s
        result[uav_id] = {
            "assigned_area_cells": newly,
            "newly_covered_cells": newly,
            "repeated_covered_cells": repeat_by_uav.get(uav_id, 0),
            "distance_m": distance_m,
            "active_time_s": active_s,
            "idle_time_s": idle_s,
            "idle_time_ratio": idle_s / observed_s if observed_s > 0 else 0.0,
            "confirm_time_s": confirm_time.get(uav_id, 0.0),
            "replan_count": final_replans,
            "turn_count": turn_count.get(uav_id, 0),
            "average_coverage_gain_per_meter": newly / distance_m if distance_m > 0 else 0.0,
        }
    return result


def _route_quality(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    connectors: list[float] = []
    logical_connectors: list[float] = []
    total_turns = 0
    total_distance = 0.0
    for snapshot in snapshots:
        total_distance = max(total_distance, _snapshot_total_distance(snapshot))
        for command in snapshot.get("commands", []):
            path = command.get("path") or []
            if len(path) < 2:
                continue
            connectors.extend(_segment_lengths(path))
            logical_waypoints = _logical_waypoints(command)
            if len(logical_waypoints) >= 2:
                logical_connectors.extend(_segment_lengths(logical_waypoints))
    long_connectors = [value for value in connectors if value > 5.0]
    long_logical_connectors = [value for value in logical_connectors if value > 5.0]
    total_turns = _turn_count(snapshots)
    turn_rate = total_turns / total_distance if total_distance > 0 else 0.0
    return {
        "approximate_crossing_count": 0,
        "crossing_count": 0,
        "long_connector_count": len(long_connectors),
        "average_connector_length": sum(connectors) / len(connectors) if connectors else 0.0,
        "max_connector_length": max(connectors, default=0.0),
        "long_logical_connector_count": len(long_logical_connectors),
        "avg_logical_connector_length": sum(logical_connectors) / len(logical_connectors) if logical_connectors else 0.0,
        "max_logical_connector_length": max(logical_connectors, default=0.0),
        "path_smoothness_score": 1.0 / (1.0 + turn_rate),
        "turn_rate": turn_rate,
    }


def _segment_quality(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    segment_count_per_uav: dict[str, int] = {}
    connector_cost_per_uav: dict[str, float] = {}
    sweep_cost_per_uav: dict[str, float] = {}
    orientations: list[str] = []
    seen_segment_ids: set[tuple[str, str]] = set()
    planner_summary = {
        "generated_segment_count": 0,
        "selected_segment_count": 0,
        "dropped_low_gain_segment_count": 0,
        "dropped_short_segment_count": 0,
        "estimated_selected_coverage_cells": 0,
        "estimated_selected_priority_cells": 0,
        "local_cluster_count": 0,
        "astar_connector_cache_hits": 0,
        "astar_connector_cache_misses": 0,
        "unreachable_connector_count": 0,
        "bundle_exchange_attempts": 0,
        "bundle_exchange_accepted": 0,
    }
    planner_summary_float = {
        "max_bundle_cost_before_exchange": 0.0,
        "max_bundle_cost_after_exchange": 0.0,
        "total_bundle_cost_before_exchange": 0.0,
        "total_bundle_cost_after_exchange": 0.0,
    }
    seen_planner_summary_keys: set[tuple[str, tuple[str, ...]]] = set()
    for snapshot in snapshots:
        for command in snapshot.get("commands", []):
            metadata = command.get("metadata") or {}
            if metadata.get("planner_version") not in {"segment_sweep_v1", "adaptive_component_sweep_v1"}:
                continue
            uav_id = str(command.get("uav_id", "unknown"))
            segment_ids = [str(item) for item in metadata.get("segment_ids", []) if item is not None]
            if segment_ids:
                new_ids = [(uav_id, segment_id) for segment_id in segment_ids if (uav_id, segment_id) not in seen_segment_ids]
                seen_segment_ids.update(new_ids)
                segment_count = len(new_ids)
            else:
                segment_count = int(metadata.get("segment_count", 0) or 0)
            if segment_count <= 0:
                continue
            connector_cost = float(metadata.get("estimated_connector_cost_m", 0.0) or 0.0)
            sweep_cost = float(metadata.get("estimated_sweep_cost_m", 0.0) or 0.0)
            segment_count_per_uav[uav_id] = segment_count_per_uav.get(uav_id, 0) + segment_count
            connector_cost_per_uav[uav_id] = connector_cost_per_uav.get(uav_id, 0.0) + connector_cost
            sweep_cost_per_uav[uav_id] = sweep_cost_per_uav.get(uav_id, 0.0) + sweep_cost
            orientation = metadata.get("segment_orientation")
            if isinstance(orientation, str):
                orientations.append(orientation)
            summary_key = (uav_id, tuple(segment_ids))
            if segment_ids and summary_key not in seen_planner_summary_keys:
                seen_planner_summary_keys.add(summary_key)
                for key in planner_summary:
                    planner_summary[key] = max(planner_summary[key], int(float(metadata.get(key, 0) or 0)))
                for key in planner_summary_float:
                    planner_summary_float[key] = max(planner_summary_float[key], float(metadata.get(key, 0.0) or 0.0))

    bundle_costs = {
        uav_id: connector_cost_per_uav.get(uav_id, 0.0) + sweep_cost_per_uav.get(uav_id, 0.0)
        for uav_id in set(connector_cost_per_uav) | set(sweep_cost_per_uav)
    }
    segment_lengths = [
        float((command.get("metadata") or {}).get("estimated_sweep_cost_m", 0.0) or 0.0)
        / max(1, int((command.get("metadata") or {}).get("segment_count", 1) or 1))
        for snapshot in snapshots
        for command in snapshot.get("commands", [])
        if (command.get("metadata") or {}).get("planner_version") == "segment_sweep_v1"
    ]
    return {
        "segment_count_total": sum(segment_count_per_uav.values()),
        "unique_segment_count": sum(segment_count_per_uav.values()),
        "segment_count_per_uav": segment_count_per_uav,
        "estimated_connector_cost_per_uav": connector_cost_per_uav,
        "estimated_sweep_cost_per_uav": sweep_cost_per_uav,
        "segment_bundle_cost_per_uav": bundle_costs,
        "max_segment_bundle_cost": max(bundle_costs.values(), default=0.0),
        "segment_workload_balance": _workload_balance(list(bundle_costs.values()), include_zero=True),
        "average_segment_length": sum(segment_lengths) / len(segment_lengths) if segment_lengths else 0.0,
        "max_segment_length": max(segment_lengths, default=0.0),
        "segment_orientation": _dominant_value(orientations),
        **planner_summary,
        **planner_summary_float,
        **_cluster_quality_from_commands(snapshots),
    }


def _cluster_quality_from_commands(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    cluster_count_per_uav: dict[str, int] = {}
    cluster_bundle_cost_per_uav: dict[str, float] = {}
    summary: dict[str, Any] = {
        "cluster_count_total": 0,
        "avg_segments_per_cluster": 0.0,
        "max_segments_per_cluster": 0,
        "max_cluster_bundle_cost": 0.0,
        "cluster_workload_balance": 1.0,
        "cluster_exchange_attempts": 0,
        "cluster_exchange_accepted": 0,
        "max_cluster_cost_before_exchange": 0.0,
        "max_cluster_cost_after_exchange": 0.0,
        "total_cluster_cost_before_exchange": 0.0,
        "total_cluster_cost_after_exchange": 0.0,
        "cluster_assignment_connector_cost_per_uav": {},
        "cluster_assignment_total_cost_per_uav": {},
        "intra_component_connector_cost": 0.0,
        "inter_component_connector_cost": 0.0,
        "inter_component_jump_count": 0,
        "max_inter_component_jump_m": 0.0,
        "avg_inter_component_jump_m": 0.0,
        "planned_coverage_ratio": 0.0,
        "planned_priority_coverage_ratio": 0.0,
        "simple_component_count": 0,
        "complex_component_count": 0,
        "component_count_total": 0,
        "simple_frontload_enabled": False,
        "frontload_component_count": 0,
        "frontload_target_cells": 0,
        "frontload_coverage_target": 0.0,
        "frontload_priority_cells": 0,
        "frontload_uav_ids": [],
        "simple_guardrail_triggered_count": 0,
        "simple_guardrail_component_ids": [],
        "baseline_estimated_cost": 0.0,
        "adaptive_estimated_cost": 0.0,
        "estimated_connector_cost": 0.0,
        "chosen_component_planner": {},
        "simple_guardrail_max_cost_ratio": 0.0,
        "simple_guardrail_max_connector_ratio": 0.0,
        "complex_guardrail_triggered_count": 0,
        "complex_guardrail_component_ids": [],
        "complex_baseline_estimated_cost": 0.0,
        "complex_adaptive_estimated_cost": 0.0,
        "complex_baseline_max_cost": 0.0,
        "complex_adaptive_max_cost": 0.0,
        "complex_guardrail_max_bundle_cost_ratio": 0.0,
        "complex_guardrail_max_total_cost_ratio": 0.0,
        "complex_guardrail_max_complexity_score": 0.0,
        "complex_guardrail_observed_bundle_cost_ratio": 0.0,
        "complex_guardrail_observed_total_cost_ratio": 0.0,
        "threshold_phase_cluster_count": 0,
        "post_threshold_cluster_count": 0,
        "estimated_threshold_coverage_ratio": 0.0,
        "threshold_first_ordering_enabled": False,
        "low_gain_pre_threshold_cluster_count": 0,
        "far_pre_threshold_cluster_count": 0,
        "threshold_phase_inter_component_jump_count": 0,
        "clustered_launch_detected": False,
        "clustered_launch_uav_count": 0,
        "clustered_launch_bbox": {},
        "clustered_launch_reason": "",
        "clustered_sector_count": 0,
        "clustered_sector_orientation": "",
        "clustered_sector_entry_side": "",
        "clustered_sector_cost_per_uav": {},
        "clustered_sector_cells_per_uav": {},
        "clustered_sector_workload_balance": 1.0,
        "launch_profile": "",
        "launch_entry_side": "",
        "common_edge_staging_detected": False,
        "common_edge_reason": "",
        "common_edge_uav_projection_order": [],
        "common_edge_max_distance_to_side": 0.0,
        "common_edge_avg_distance_to_side": 0.0,
        "common_edge_distance_limit": 0.0,
        "sector_assignment_order": [],
        "sector_cells_per_uav": {},
        "sector_estimated_cost_per_uav": {},
        "sector_balance_score": 1.0,
    }
    seen: set[tuple[str, tuple[str, ...]]] = set()
    for snapshot in snapshots:
        for command in snapshot.get("commands", []):
            metadata = command.get("metadata") or {}
            if metadata.get("planner_version") != "adaptive_component_sweep_v1":
                continue
            uav_id = str(command.get("uav_id", "unknown"))
            cluster_ids = tuple(str(item) for item in metadata.get("cluster_ids", []) if item is not None)
            if cluster_ids and (uav_id, cluster_ids) not in seen:
                seen.add((uav_id, cluster_ids))
                cluster_count_per_uav[uav_id] = cluster_count_per_uav.get(uav_id, 0) + len(cluster_ids)
                cluster_bundle_cost_per_uav[uav_id] = cluster_bundle_cost_per_uav.get(uav_id, 0.0) + float(
                    metadata.get("estimated_connector_cost_m", 0.0) or 0.0
                ) + float(metadata.get("estimated_sweep_cost_m", 0.0) or 0.0)
            for key in summary:
                value = metadata.get(key)
                if value is None:
                    continue
                if isinstance(summary[key], bool):
                    summary[key] = bool(summary[key]) or bool(value)
                elif isinstance(summary[key], dict):
                    if isinstance(value, dict):
                        summary[key] = _merge_numeric_dicts(summary[key], value)
                elif isinstance(summary[key], list):
                    if isinstance(value, list) and len(value) > len(summary[key]):
                        summary[key] = list(value)
                elif isinstance(summary[key], str):
                    if value:
                        summary[key] = str(value)
                elif isinstance(summary[key], int):
                    summary[key] = max(summary[key], int(float(value)))
                else:
                    summary[key] = max(float(summary[key]), float(value))
    summary["cluster_count_per_uav"] = cluster_count_per_uav
    summary["cluster_bundle_cost_per_uav"] = cluster_bundle_cost_per_uav
    if cluster_count_per_uav:
        summary["cluster_count_total"] = max(int(summary["cluster_count_total"]), sum(cluster_count_per_uav.values()))
    return summary


def _merge_numeric_dicts(current: dict[str, Any], incoming: dict[str, Any]) -> dict[str, Any]:
    merged = dict(current)
    for key, value in incoming.items():
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            merged[str(key)] = value
            continue
        merged[str(key)] = max(float(merged.get(str(key), 0.0) or 0.0), numeric)
    return merged


def _fleet_planned_coverage_quality(grid_map: GridMap, snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    target_cells = set(grid_map.get_searchable_cells())
    priority_cells = {cell for cell in target_cells if grid_map.get_cell(cell).search_priority > 1.0}
    planned: set[Position] = set()
    seen_tasks: set[str] = set()
    for snapshot in snapshots:
        for command in snapshot.get("commands", []):
            metadata = command.get("metadata") or {}
            if metadata.get("planner_version") != "adaptive_component_sweep_v1":
                continue
            task_key = str(metadata.get("task_id") or command.get("task_id") or command.get("command_id") or "")
            if task_key and task_key in seen_tasks:
                continue
            waypoints = _metadata_positions(metadata.get("coverage_waypoints") or metadata.get("logical_waypoints") or [])
            if not waypoints:
                continue
            if task_key:
                seen_tasks.add(task_key)
            radius = int(metadata.get("sensor_radius_cells", 0) or 0)
            planned.update(_simulate_planned_coverage(waypoints, radius, target_cells))
    if not target_cells:
        coverage_ratio = 1.0
    else:
        coverage_ratio = len(planned & target_cells) / len(target_cells)
    if not priority_cells:
        priority_ratio = 1.0
    else:
        priority_ratio = len(planned & priority_cells) / len(priority_cells)
    return {
        "fleet_planned_coverage_ratio": coverage_ratio,
        "fleet_planned_priority_coverage_ratio": priority_ratio,
        "fleet_planned_covered_cells": len(planned & target_cells),
        "fleet_planned_priority_covered_cells": len(planned & priority_cells),
    }


def _metadata_positions(raw: Any) -> list[Position]:
    positions: list[Position] = []
    if not isinstance(raw, list):
        return positions
    for item in raw:
        if not isinstance(item, dict):
            continue
        positions.append(Position(int(item.get("x", 0)), int(item.get("y", 0))))
    return positions


def _simulate_planned_coverage(
    waypoints: list[Position],
    sensor_radius_cells: int,
    target_cells: set[Position],
) -> set[Position]:
    radius_sq = sensor_radius_cells * sensor_radius_cells
    return {
        cell
        for waypoint in waypoints
        for cell in target_cells
        if (cell.x - waypoint.x) ** 2 + (cell.y - waypoint.y) ** 2 <= radius_sq
    }


def _command_quality(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    seen_acks: set[tuple[str, str]] = set()
    rejected_reasons: dict[str, int] = {}
    rejected_count = 0
    for snapshot in snapshots:
        for ack in snapshot.get("command_acks", []):
            command_id = str(ack.get("command_id", ""))
            status = str(ack.get("status", "")).lower()
            key = (command_id, status)
            if not command_id or key in seen_acks:
                continue
            seen_acks.add(key)
            if status not in {"rejected", "failed"}:
                continue
            rejected_count += 1
            reason = str(ack.get("reason") or "unknown")
            rejected_reasons[reason] = rejected_reasons.get(reason, 0) + 1
    return {
        "command_rejected_count": rejected_count,
        "rejected_reasons": rejected_reasons,
    }


def _scheduler_quality(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    keys = {
        "cancelled_post_goal_tasks",
        "skipped_post_goal_supplemental_tasks",
        "post_goal_active_search_cancel_count",
        "skipped_low_gain_supplemental_count",
        "late_stage_supplemental_count",
        "idle_assist_attempts",
        "idle_assist_created_tasks",
        "idle_assist_accepted_tasks",
        "idle_assist_rejected_low_gain",
        "idle_assist_rejected_unreachable",
        "idle_assist_donor_replans",
        "idle_uav_wait_time_s",
        "idle_assist_cells_reassigned",
        "idle_assist_distance_m",
        "dynamic_route_repair_attempts",
        "dynamic_route_repair_success",
        "dynamic_route_repair_dropped_waypoints",
        "dynamic_route_repair_replanned_tasks",
        "dynamic_route_repair_fallback_to_supplemental",
        "modeling_jobs_total",
        "modeling_jobs_completed",
        "modeling_jobs_failed",
        "modeling_active_jobs",
        "modeling_assigned_uav_count",
        "modeling_facade_lane_count",
        "modeling_facade_progress_ratio",
        "modeling_distance_m",
        "modeling_interrupted_search_tasks",
        "modeling_resumed_search_tasks",
        "modeling_unreachable_facade_lanes",
        "modeling_no_fly_violations",
        "modeling_return_home_commands",
        "modeling_hold_after_done_count",
        "modeling_no_resume_return_home_count",
        "modeling_completed_without_interrupted_search_count",
        "modeling_uav_stuck_modeling_count",
    }
    result = {key: 0 for key in keys}
    latest_idle_reasons: dict[str, str] = {}
    for snapshot in snapshots:
        diagnostics = snapshot.get("scheduler_diagnostics") or {}
        if not isinstance(diagnostics, dict):
            continue
        for key in keys:
            result[key] = max(result[key], int(diagnostics.get(key, 0) or 0))
        reasons = diagnostics.get("idle_reason_per_uav", {})
        if isinstance(reasons, dict) and reasons:
            latest_idle_reasons = {str(key): str(value) for key, value in reasons.items()}
    result["idle_reason_per_uav"] = latest_idle_reasons
    return result


def _coverage_quality(grid_map: GridMap, snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    at_95 = next((snapshot for snapshot in snapshots if float(snapshot.get("global_coverage", 0.0)) >= 0.95), None)
    post_95_cells: set[tuple[int, int]] = set()
    post_95_distance = 0.0
    post_95_search_distance = 0.0
    post_95_return_distance = 0.0
    post_95_confirm_distance = 0.0
    if at_95 is not None and snapshots:
        threshold_time = float(at_95.get("time_s", 0.0))
        threshold_distance = _snapshot_total_distance(at_95)
        post_95_distance = max(0.0, _snapshot_total_distance(snapshots[-1]) - threshold_distance)
        post_by_mode = _post_threshold_distance_by_mode(snapshots, threshold_time)
        post_95_search_distance = post_by_mode["search"]
        post_95_return_distance = post_by_mode["return"]
        post_95_confirm_distance = post_by_mode["confirm"]
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
        "post_95_search_distance_m": post_95_search_distance,
        "post_95_return_distance_m": post_95_return_distance,
        "post_95_confirm_distance_m": post_95_confirm_distance,
    }


def _allocation_quality(per_uav: dict[str, dict[str, Any]], snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    actual_costs = {uav_id: float(data.get("distance_m", 0.0)) for uav_id, data in per_uav.items()}
    active_finish_times = _finish_times(snapshots)
    idle_ratios = [float(data.get("idle_time_ratio", 0.0)) for data in per_uav.values()]
    workload_balance_all = _workload_balance(list(actual_costs.values()), include_zero=True)
    workload_balance_active = _workload_balance(list(actual_costs.values()), include_zero=False)
    return {
        "route_cost_estimate_per_uav": dict(actual_costs),
        "actual_distance_per_uav": dict(actual_costs),
        "actual_cost_per_uav": actual_costs,
        "workload_balance": workload_balance_all,
        "workload_balance_all_uavs": workload_balance_all,
        "workload_balance_active_uavs": workload_balance_active,
        "fleet_idle_time_ratio": sum(idle_ratios) / len(idle_ratios) if idle_ratios else 0.0,
        "max_uav_finish_time": max(active_finish_times.values(), default=0.0),
        "min_uav_finish_time": min(active_finish_times.values(), default=0.0),
        "idle_uav_count_after_50_percent_coverage": _idle_count_after(snapshots, 0.5),
        "idle_uav_count_after_80_percent_coverage": _idle_count_after(snapshots, 0.8),
    }


def _covering_uavs(snapshot: dict[str, Any], x: int, y: int) -> list[str]:
    owners = []
    for uav in snapshot.get("uavs", []):
        pos = uav.get("position", {})
        radius = int(uav.get("sensor_radius_cells", 2))
        if abs(int(pos.get("x", 0)) - x) <= radius and abs(int(pos.get("y", 0)) - y) <= radius:
            owners.append(str(uav.get("id")))
    return owners


def _logical_waypoints(command: dict[str, Any]) -> list[dict[str, Any]]:
    metadata = command.get("metadata") or {}
    waypoints = metadata.get("logical_waypoints") or metadata.get("coverage_waypoints") or []
    return [point for point in waypoints if isinstance(point, dict)]


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


def _post_threshold_distance_by_mode(snapshots: list[dict[str, Any]], threshold_time: float) -> dict[str, float]:
    totals = {"search": 0.0, "return": 0.0, "confirm": 0.0}
    previous_by_uav: dict[str, float] = {}
    for snapshot in snapshots:
        time_s = float(snapshot.get("time_s", 0.0))
        for uav in snapshot.get("uavs", []):
            uav_id = str(uav.get("id"))
            current_distance = float(uav.get("total_distance_m", 0.0))
            previous_distance = previous_by_uav.get(uav_id)
            previous_by_uav[uav_id] = current_distance
            if previous_distance is None or time_s <= threshold_time:
                continue
            delta = max(0.0, current_distance - previous_distance)
            status = str(uav.get("status", ""))
            if status == "RETURNING":
                totals["return"] += delta
            elif status == "CONFIRMING":
                totals["confirm"] += delta
            else:
                totals["search"] += delta
    return totals


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


def _workload_balance(values: list[float], include_zero: bool = False) -> float:
    active = list(values) if include_zero else [value for value in values if value > 0]
    if not active:
        return 1.0
    mean = sum(active) / len(active)
    if mean <= 0:
        return 1.0
    variance = sum((value - mean) ** 2 for value in active) / len(active)
    return 1.0 / (1.0 + math.sqrt(variance) / mean)


def _dominant_value(values: list[str]) -> str | None:
    if not values:
        return None
    counts = {value: values.count(value) for value in set(values)}
    if len(counts) > 1:
        return "mixed"
    return max(counts, key=counts.get)
