from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import count
from typing import Protocol

from uav_search.core.data_types import Position, Task, TaskType, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.planning.astar import astar_search, path_cost
from uav_search.planning.reachability import ReachabilityIndex
from uav_search.planning.reachability import connected_components as reachability_components
from uav_search.task.task_generator import estimate_task_cost, generate_initial_tasks
from uav_search.task.task_generator import reorder_waypoints_for_uav


@dataclass(frozen=True)
class SweepSegment:
    id: str
    component_id: str
    orientation: str
    line_index: int
    start: Position
    end: Position
    sampled_waypoints: list[Position]
    coverage_cells: set[Position]
    priority_value: float
    uncovered_value: float
    allowed_uav_ids: set[str]
    sweep_cost_m: float


class SegmentConnectorCostCache:
    def __init__(self, planner_run_id: str = "default") -> None:
        self.planner_run_id = planner_run_id
        self._costs: dict[tuple[str, Position, Position], float] = {}
        self.hits = 0
        self.misses = 0
        self.unreachable_count = 0

    def cost(self, start: Position, end: Position, grid_map: GridMap) -> float:
        if start == end:
            return 0.0
        key = (self.planner_run_id, start, end)
        if key in self._costs:
            self.hits += 1
            return self._costs[key]
        self.misses += 1
        path = astar_search(grid_map, start, end)
        if path is None:
            self.unreachable_count += 1
            self._costs[key] = math.inf
            return math.inf
        cost = path_cost(path) * grid_map.resolution_m
        self._costs[key] = cost
        return cost


class CoveragePlanner(Protocol):
    version: str

    def plan_initial_tasks(
        self,
        *,
        grid_map: GridMap,
        uav_states: list[UAVState],
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        searchable_cells: set[Position],
    ) -> list[Task]:
        ...

    def plan_region_task(
        self,
        *,
        task_id: str,
        region: set[Position],
        origin: Position,
        grid_map: GridMap,
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        allowed_uav_ids: set[str] | None = None,
    ) -> Task | None:
        ...


class BaselineSparseBoustrophedonPlanner:
    version = "baseline_sparse_boustrophedon"

    def __init__(self, config: dict | None = None) -> None:
        self.config = config or {}

    def plan_initial_tasks(
        self,
        *,
        grid_map: GridMap,
        uav_states: list[UAVState],
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        searchable_cells: set[Position],
    ) -> list[Task]:
        if not uav_states:
            return []
        tasks = generate_initial_tasks(
            grid_map=grid_map,
            uav_count=max(1, len(uav_states)),
            sensor_radius_cells=sensor_radius_cells,
            home=uav_states[0].home_position,
            origins=[state.position for state in uav_states],
            created_at=created_at,
            searchable_cells=searchable_cells,
        )
        for task in tasks:
            task.metadata.setdefault("planner_version", self.version)
        return tasks

    def plan_region_task(
        self,
        *,
        task_id: str,
        region: set[Position],
        origin: Position,
        grid_map: GridMap,
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        allowed_uav_ids: set[str] | None = None,
    ) -> Task | None:
        from uav_search.task.task_generator import compute_region_value, generate_boustrophedon_path

        waypoints = generate_boustrophedon_path(region, sensor_radius_cells)
        if not waypoints:
            return None
        waypoints = reorder_waypoints_for_uav(waypoints, origin)
        uncovered_value, priority_value = compute_region_value(region, grid_map)
        estimated_cost_m = estimate_task_cost(waypoints, waypoints[0], grid_map.resolution_m)
        return Task(
            id=task_id,
            type=TaskType.SEARCH,
            priority=max(grid_map.get_cell(cell).search_priority for cell in region),
            target_cells=set(region),
            entry_point=waypoints[0],
            waypoints=waypoints,
            coverage_waypoints=list(waypoints),
            estimated_cost_m=estimated_cost_m,
            created_at=created_at,
            updated_at=created_at,
            uncovered_value=uncovered_value,
            priority_value=priority_value,
            score=(uncovered_value + priority_value) / max(estimated_cost_m, 1.0),
            allowed_uav_ids=set(allowed_uav_ids) if allowed_uav_ids else None,
            metadata={"planner_version": self.version},
        )


class SegmentSweepPlanner:
    version = "segment_sweep_v1"

    def __init__(self, config: dict | None = None) -> None:
        self.config = config or {}
        self.last_diagnostics: dict[str, object] = {}
        self._current_task_metadata: dict[str, object] = {}

    def plan_initial_tasks(
        self,
        *,
        grid_map: GridMap,
        uav_states: list[UAVState],
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        searchable_cells: set[Position],
    ) -> list[Task]:
        online = [state for state in uav_states if state.id in reachability.reachable_by_uav]
        segments = self.generate_segments(
            searchable_cells=searchable_cells,
            grid_map=grid_map,
            uav_states=online,
            sensor_radius_cells=sensor_radius_cells,
            reachability=reachability,
        )
        selected_segments = self.select_segments_for_coverage_goal(
            segments,
            grid_map,
            mission_complete_coverage_threshold=float(
                self.config.get("search", {}).get("mission_complete_coverage_threshold", 0.95)
            ),
            priority_complete_threshold=float(self.config.get("search", {}).get("priority_complete_threshold", 0.98)),
            reference_cells=searchable_cells,
        )
        bundles = self.assign_segments_to_uavs(selected_segments, online, grid_map)
        tasks: list[Task] = []
        task_seq = count(1)
        for uav in online:
            ordered = self.order_segments_for_uav(bundles.get(uav.id, []), uav.position, grid_map)
            task = self._task_from_segments(
                task_id=f"task_{next(task_seq):03d}",
                uav_id=uav.id,
                ordered_segments=ordered,
                origin=uav.position,
                grid_map=grid_map,
                created_at=created_at,
            )
            if task is not None:
                tasks.append(task)
        self._record_diagnostics(segments, selected_segments, bundles)
        return sorted(tasks, key=lambda task: (-task.priority, task.created_at, task.id))

    def plan_region_task(
        self,
        *,
        task_id: str,
        region: set[Position],
        origin: Position,
        grid_map: GridMap,
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
        allowed_uav_ids: set[str] | None = None,
    ) -> Task | None:
        uav_states = [
            UAVState(
                id=uav_id,
                position=origin,
                velocity_mps=1.0,
                heading_deg=0.0,
                battery=1.0,
                sensor_radius_cells=sensor_radius_cells,
                status=UAVStatus.IDLE,
                home_position=origin,
            )
            for uav_id in sorted(allowed_uav_ids or reachability.reachable_by_uav)
        ]
        segments = self.generate_segments(
            searchable_cells=region,
            grid_map=grid_map,
            uav_states=uav_states,
            sensor_radius_cells=sensor_radius_cells,
            reachability=reachability,
        )
        if not segments:
            return None
        segments = self.select_segments_for_coverage_goal(
            segments,
            grid_map,
            mission_complete_coverage_threshold=1.0,
            priority_complete_threshold=1.0,
            reference_cells=region,
        )
        ordered = self.order_segments_for_uav(segments, origin, grid_map)
        owner = sorted(allowed_uav_ids or set().union(*(segment.allowed_uav_ids for segment in ordered)) or {"unassigned"})[0]
        return self._task_from_segments(
            task_id=task_id,
            uav_id=owner,
            ordered_segments=ordered,
            origin=origin,
            grid_map=grid_map,
            created_at=created_at,
            force_allowed_uav_ids=allowed_uav_ids,
        )

    def generate_segments(
        self,
        *,
        searchable_cells: set[Position],
        grid_map: GridMap,
        uav_states: list[UAVState],
        sensor_radius_cells: int,
        reachability: ReachabilityIndex,
    ) -> list[SweepSegment]:
        reachable_cells = {cell for cell in searchable_cells if grid_map.is_passable(cell) and reachability.any_reachable(cell)}
        segments: list[SweepSegment] = []
        for component_index, component in enumerate(reachability_components(grid_map, reachable_cells), start=1):
            horizontal = self._segments_for_orientation(
                component, f"component_{component_index}", "horizontal", grid_map, sensor_radius_cells, reachability
            )
            vertical = self._segments_for_orientation(
                component, f"component_{component_index}", "vertical", grid_map, sensor_radius_cells, reachability
            )
            chosen = min((horizontal, vertical), key=lambda candidate: self._orientation_cost(candidate, grid_map))
            segments.extend(chosen)
        return segments

    def select_segments_for_coverage_goal(
        self,
        segments: list[SweepSegment],
        grid_map: GridMap,
        *,
        mission_complete_coverage_threshold: float,
        priority_complete_threshold: float,
        reference_cells: set[Position] | None = None,
    ) -> list[SweepSegment]:
        if not segments:
            self._current_task_metadata = {
                "generated_segment_count": 0,
                "selected_segment_count": 0,
                "dropped_low_gain_segment_count": 0,
                "dropped_short_segment_count": 0,
                "estimated_selected_coverage_cells": 0,
                "estimated_selected_priority_cells": 0,
                "local_cluster_count": 0,
            }
            return []

        config = self._segment_config()
        margin = float(config.get("coverage_margin", 0.015))
        max_target = float(config.get("max_initial_coverage_target", 0.97))
        target_ratio = min(max_target, mission_complete_coverage_threshold + margin)
        min_gain_cells = int(config.get("min_segment_gain_cells", 1))
        min_gain_per_meter = float(config.get("min_segment_gain_per_meter", 0.0))
        min_segment_length_m = float(config.get("min_segment_length_m", 0.0))
        drop_short_low_gain = bool(config.get("drop_short_low_gain_segments", False))

        segment_cells = set().union(*(segment.coverage_cells for segment in segments))
        searchable_cells = {
            cell
            for cell in (reference_cells or segment_cells)
            if grid_map.is_passable(cell)
        }
        priority_cells = {cell for cell in searchable_cells if grid_map.get_cell(cell).search_priority > 1.0}
        target_cells = math.ceil(len(searchable_cells) * target_ratio)
        target_priority_cells = math.ceil(len(priority_cells) * priority_complete_threshold)
        selected: list[SweepSegment] = []
        selected_ids: set[str] = set()
        covered: set[Position] = set()
        priority_covered: set[Position] = set()
        dropped_low_gain = 0
        dropped_short = 0

        def add(segment: SweepSegment) -> None:
            selected.append(segment)
            selected_ids.add(segment.id)
            covered.update(segment.coverage_cells)
            priority_covered.update(segment.coverage_cells & priority_cells)

        priority_segments = sorted(
            (segment for segment in segments if segment.coverage_cells & priority_cells),
            key=lambda segment: (
                -len(segment.coverage_cells & priority_cells),
                segment.sweep_cost_m / max(1, len(segment.coverage_cells & priority_cells)),
                segment.id,
            ),
        )
        for segment in priority_segments:
            if len(priority_covered) >= target_priority_cells:
                break
            add(segment)

        ordinary = [segment for segment in segments if segment.id not in selected_ids]
        while len(covered) < target_cells and ordinary:
            scored: list[tuple[float, float, str, SweepSegment, set[Position]]] = []
            for segment in ordinary:
                incremental = segment.coverage_cells - covered
                gain = len(incremental)
                if gain <= 0:
                    dropped_low_gain += 1
                    continue
                gain_per_meter = gain / max(segment.sweep_cost_m, grid_map.resolution_m)
                has_priority = bool(incremental & priority_cells)
                if drop_short_low_gain and segment.sweep_cost_m < min_segment_length_m and gain < min_gain_cells and not has_priority:
                    dropped_short += 1
                    continue
                if not has_priority and (gain < min_gain_cells or gain_per_meter < min_gain_per_meter):
                    dropped_low_gain += 1
                    continue
                scored.append((-(gain_per_meter + segment.priority_value), segment.sweep_cost_m, segment.id, segment, incremental))
            if not scored:
                break
            _, _, _, winner, _ = min(scored)
            add(winner)
            ordinary = [segment for segment in ordinary if segment.id != winner.id]

        local_clusters = len({(segment.component_id, segment.orientation, segment.line_index) for segment in selected})
        self._current_task_metadata = {
            "generated_segment_count": len(segments),
            "selected_segment_count": len(selected),
            "dropped_low_gain_segment_count": dropped_low_gain,
            "dropped_short_segment_count": dropped_short,
            "estimated_selected_coverage_cells": len(covered),
            "estimated_selected_priority_cells": len(priority_covered),
            "local_cluster_count": local_clusters,
        }
        self.last_diagnostics.update(self._current_task_metadata)
        return selected

    def assign_segments_to_uavs(
        self,
        segments: list[SweepSegment],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, list[SweepSegment]]:
        bundles: dict[str, list[SweepSegment]] = {state.id: [] for state in uav_states}
        route_ends = {state.id: state.position for state in uav_states}
        route_costs = {state.id: 0.0 for state in uav_states}
        cache = SegmentConnectorCostCache(planner_run_id=f"assign_{id(self)}_{len(segments)}")
        ordered_segments = sorted(
            segments,
            key=lambda segment: (-(segment.priority_value + segment.uncovered_value), segment.component_id, segment.line_index, segment.id),
        )
        for segment in ordered_segments:
            candidates: list[tuple[float, str, bool]] = []
            for uav_id in sorted(segment.allowed_uav_ids):
                if uav_id not in bundles:
                    continue
                end = route_ends[uav_id]
                forward = cache.cost(end, segment.start, grid_map)
                reverse = cache.cost(end, segment.end, grid_map)
                connector = min(forward, reverse)
                if math.isinf(connector):
                    continue
                projected = route_costs[uav_id] + connector + segment.sweep_cost_m
                candidates.append((projected, uav_id, reverse < forward))
            if not candidates:
                continue
            projected, winner, reversed_segment = min(candidates, key=lambda item: (item[0], item[1]))
            bundles[winner].append(_reverse_segment(segment) if reversed_segment else segment)
            route_ends[winner] = segment.start if reversed_segment else segment.end
            route_costs[winner] = projected
        improved = self.improve_segment_bundles(bundles, uav_states, grid_map)
        self.last_diagnostics.update(
            {
                "astar_connector_cache_hits": cache.hits,
                "astar_connector_cache_misses": cache.misses,
                "unreachable_connector_count": cache.unreachable_count,
            }
        )
        return improved

    def order_segments_for_uav(
        self,
        segments: list[SweepSegment],
        origin: Position,
        grid_map: GridMap,
        cache: SegmentConnectorCostCache | None = None,
    ) -> list[SweepSegment]:
        remaining = list(segments)
        ordered: list[SweepSegment] = []
        current = origin
        cache = cache or SegmentConnectorCostCache(planner_run_id=f"order_{id(self)}_{origin}_{len(segments)}")
        while remaining:
            index, reverse = min(
                (
                    (
                        idx,
                        cache.cost(current, segment.end, grid_map) < cache.cost(current, segment.start, grid_map),
                    )
                    for idx, segment in enumerate(remaining)
                ),
                key=lambda item: cache.cost(current, remaining[item[0]].end if item[1] else remaining[item[0]].start, grid_map),
            )
            segment = remaining.pop(index)
            if reverse:
                segment = _reverse_segment(segment)
            ordered.append(segment)
            current = segment.end
        return ordered

    def bundle_costs(
        self,
        bundles: dict[str, list[SweepSegment]],
        uav_states: list[UAVState],
        grid_map: GridMap,
        cache: SegmentConnectorCostCache | None = None,
    ) -> dict[str, float]:
        states = {state.id: state for state in uav_states}
        return {
            uav_id: self._bundle_cost(bundle, states[uav_id].position, grid_map, cache)
            for uav_id, bundle in bundles.items()
            if uav_id in states
        }

    def improve_segment_bundles(
        self,
        bundles: dict[str, list[SweepSegment]],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, list[SweepSegment]]:
        config = self._segment_config()
        max_iterations = int(config.get("bundle_exchange_iterations", 0))
        max_total_increase_ratio = float(config.get("bundle_exchange_max_total_cost_increase_ratio", 0.5))
        candidate_limit = int(config.get("bundle_exchange_candidate_limit", 6))
        if max_iterations <= 0:
            return {uav_id: list(bundle) for uav_id, bundle in bundles.items()}

        improved = {uav_id: list(bundle) for uav_id, bundle in bundles.items()}
        attempts = 0
        accepted = 0
        cache = SegmentConnectorCostCache(planner_run_id=f"exchange_{id(self)}_{sum(len(bundle) for bundle in bundles.values())}")
        before = self.bundle_costs(improved, uav_states, grid_map, cache)
        states = {state.id: state for state in uav_states}

        for _ in range(max_iterations):
            costs = self.bundle_costs(improved, uav_states, grid_map, cache)
            if not costs:
                break
            max_uav = max(costs, key=costs.get)
            current_max = costs[max_uav]
            current_total = sum(costs.values())
            best_move: tuple[float, float, dict[str, list[SweepSegment]]] | None = None

            max_bundle = list(improved.get(max_uav, []))
            movable = sorted(
                enumerate(max_bundle),
                key=lambda item: (
                    -item[1].sweep_cost_m,
                    -len(item[1].coverage_cells),
                    item[1].id,
                ),
            )[: max(1, candidate_limit)]
            for idx, segment in movable:
                for target_uav in sorted(improved):
                    if target_uav == max_uav or target_uav not in segment.allowed_uav_ids:
                        continue
                    attempts += 1
                    candidate = {uav_id: list(bundle) for uav_id, bundle in improved.items()}
                    candidate[max_uav].pop(idx)
                    candidate[target_uav].append(segment)
                    ordered = {
                        uav_id: self.order_segments_for_uav(bundle, states[uav_id].position, grid_map, cache)
                        for uav_id, bundle in candidate.items()
                        if uav_id in states
                    }
                    candidate_costs = self.bundle_costs(ordered, uav_states, grid_map, cache)
                    candidate_max = max(candidate_costs.values(), default=0.0)
                    candidate_total = sum(candidate_costs.values())
                    if candidate_max < current_max and candidate_total <= current_total * (1.0 + max_total_increase_ratio):
                        if best_move is None or (candidate_max, candidate_total) < (best_move[0], best_move[1]):
                            best_move = (candidate_max, candidate_total, ordered)

            if best_move is None:
                break
            improved = best_move[2]
            accepted += 1

        after = self.bundle_costs(improved, uav_states, grid_map, cache)
        self.last_diagnostics.update(
            {
                "bundle_exchange_attempts": attempts,
                "bundle_exchange_accepted": accepted,
                "max_bundle_cost_before_exchange": max(before.values(), default=0.0),
                "max_bundle_cost_after_exchange": max(after.values(), default=0.0),
                "total_bundle_cost_before_exchange": sum(before.values()),
                "total_bundle_cost_after_exchange": sum(after.values()),
            }
        )
        return improved

    def _segments_for_orientation(
        self,
        component: set[Position],
        component_id: str,
        orientation: str,
        grid_map: GridMap,
        sensor_radius_cells: int,
        reachability: ReachabilityIndex,
    ) -> list[SweepSegment]:
        line_step = max(1, 2 * sensor_radius_cells)
        sample_step = max(1, sensor_radius_cells)
        line_values = sorted({cell.y if orientation == "horizontal" else cell.x for cell in component})
        selected_lines = _sample_with_boundaries(line_values, line_step)
        segments: list[SweepSegment] = []
        seq = 1
        for line in selected_lines:
            axis_values = sorted(
                cell.x if orientation == "horizontal" else cell.y
                for cell in component
                if (cell.y if orientation == "horizontal" else cell.x) == line
            )
            for part in _contiguous(axis_values):
                sampled_axis = _sample_with_boundaries(part, sample_step)
                sampled = [
                    Position(value, line) if orientation == "horizontal" else Position(line, value)
                    for value in sampled_axis
                ]
                if not sampled or any(not grid_map.is_passable(point) for point in sampled):
                    continue
                coverage = self._segment_coverage_cells(sampled, component, sensor_radius_cells)
                allowed = {
                    uav_id
                    for cell in coverage
                    for uav_id in reachability.reachable_uavs(cell)
                }
                if not allowed:
                    continue
                priority_value = sum(max(0.0, grid_map.get_cell(cell).search_priority - 1.0) for cell in coverage)
                sweep_cost_m = _path_cost_cells(sampled) * grid_map.resolution_m
                segments.append(
                    SweepSegment(
                        id=f"{component_id}_{orientation}_{seq:03d}",
                        component_id=component_id,
                        orientation=orientation,
                        line_index=line,
                        start=sampled[0],
                        end=sampled[-1],
                        sampled_waypoints=sampled,
                        coverage_cells=coverage,
                        priority_value=priority_value,
                        uncovered_value=float(len(coverage)),
                        allowed_uav_ids=allowed,
                        sweep_cost_m=sweep_cost_m,
                    )
                )
                seq += 1
        return segments

    def _segment_coverage_cells(
        self,
        sampled: list[Position],
        component: set[Position],
        sensor_radius_cells: int,
    ) -> set[Position]:
        radius_sq = sensor_radius_cells * sensor_radius_cells
        coverage: set[Position] = set()
        for point in sampled:
            for cell in component:
                if (cell.x - point.x) ** 2 + (cell.y - point.y) ** 2 <= radius_sq:
                    coverage.add(cell)
        return coverage or set(sampled)

    def _task_from_segments(
        self,
        *,
        task_id: str,
        uav_id: str,
        ordered_segments: list[SweepSegment],
        origin: Position,
        grid_map: GridMap,
        created_at: float,
        force_allowed_uav_ids: set[str] | None = None,
    ) -> Task | None:
        if not ordered_segments:
            return None
        waypoints = [point for segment in ordered_segments for point in segment.sampled_waypoints]
        if not waypoints:
            return None
        target_cells: set[Position] = set()
        for segment in ordered_segments:
            target_cells.update(segment.coverage_cells)
        connector_cost_m = _connector_cost_m(origin, ordered_segments, grid_map)
        sweep_cost_m = sum(segment.sweep_cost_m for segment in ordered_segments)
        estimated_cost_m = connector_cost_m + sweep_cost_m
        priority_value = sum(segment.priority_value for segment in ordered_segments)
        uncovered_value = sum(segment.uncovered_value for segment in ordered_segments)
        allowed = set(force_allowed_uav_ids) if force_allowed_uav_ids else {uav_id}
        metadata = {
            "planner_version": self.version,
            "segment_ids": [segment.id for segment in ordered_segments],
            "segment_count": len(ordered_segments),
            "segment_endpoints": [
                {"start": {"x": segment.start.x, "y": segment.start.y}, "end": {"x": segment.end.x, "y": segment.end.y}}
                for segment in ordered_segments
            ],
            "logical_waypoints": _positions_to_dicts(waypoints),
            "estimated_connector_cost_m": connector_cost_m,
            "estimated_sweep_cost_m": sweep_cost_m,
            "segment_orientation": _dominant_orientation(ordered_segments),
        }
        metadata.update(self._current_task_metadata)
        metadata.update(
            {
                key: value
                for key, value in self.last_diagnostics.items()
                if key
                in {
                    "astar_connector_cache_hits",
                    "astar_connector_cache_misses",
                    "unreachable_connector_count",
                    "bundle_exchange_attempts",
                    "bundle_exchange_accepted",
                    "max_bundle_cost_before_exchange",
                    "max_bundle_cost_after_exchange",
                    "total_bundle_cost_before_exchange",
                    "total_bundle_cost_after_exchange",
                }
            }
        )
        return Task(
            id=task_id,
            type=TaskType.SEARCH,
            priority=max(grid_map.get_cell(cell).search_priority for cell in target_cells),
            target_cells=target_cells,
            entry_point=waypoints[0],
            waypoints=waypoints,
            coverage_waypoints=list(waypoints),
            estimated_cost_m=estimated_cost_m,
            created_at=created_at,
            updated_at=created_at,
            uncovered_value=uncovered_value,
            priority_value=priority_value,
            score=(uncovered_value + priority_value) / max(estimated_cost_m, 1.0),
            allowed_uav_ids=allowed,
            metadata=metadata,
        )

    def _orientation_cost(self, segments: list[SweepSegment], grid_map: GridMap) -> float:
        if not segments:
            return math.inf
        sweep = sum(segment.sweep_cost_m for segment in segments)
        connector = sum(
            _manhattan(first.end, second.start) * grid_map.resolution_m
            for first, second in zip(segments, segments[1:])
        )
        return sweep + connector + len(segments) * grid_map.resolution_m

    def _record_diagnostics(
        self,
        segments: list[SweepSegment],
        selected_segments: list[SweepSegment],
        bundles: dict[str, list[SweepSegment]],
    ) -> None:
        costs = {
            uav_id: sum(segment.sweep_cost_m for segment in bundle)
            for uav_id, bundle in bundles.items()
        }
        existing = dict(self.last_diagnostics)
        self.last_diagnostics = {
            "segment_count_total": len(segments),
            "generated_segment_count": len(segments),
            "selected_segment_count": len(selected_segments),
            "segment_count_per_uav": {uav_id: len(bundle) for uav_id, bundle in bundles.items()},
            "estimated_sweep_cost_per_uav": costs,
            "segment_bundle_cost_per_uav": costs,
            "max_segment_bundle_cost": max(costs.values(), default=0.0),
            "segment_workload_balance": _workload_balance(list(costs.values())),
            "average_segment_length": sum(segment.sweep_cost_m for segment in segments) / len(segments) if segments else 0.0,
            "max_segment_length": max((segment.sweep_cost_m for segment in segments), default=0.0),
            "segment_orientation": _dominant_orientation(segments),
        }
        self.last_diagnostics.update(existing)

    def _segment_config(self) -> dict:
        return dict(self.config.get("algorithm", {}).get("segment_sweep", {}))

    def _bundle_cost(
        self,
        segments: list[SweepSegment],
        origin: Position,
        grid_map: GridMap,
        cache: SegmentConnectorCostCache | None = None,
    ) -> float:
        ordered = self.order_segments_for_uav(segments, origin, grid_map, cache) if segments else []
        connector = _connector_cost_m(origin, ordered, grid_map)
        sweep = sum(segment.sweep_cost_m for segment in ordered)
        turn_penalty = max(0, len(ordered) - 1) * grid_map.resolution_m
        priority_lateness = sum(index * segment.priority_value for index, segment in enumerate(ordered))
        return connector + sweep + turn_penalty + priority_lateness


def create_coverage_planner(config: dict | None = None) -> CoveragePlanner:
    version = str((config or {}).get("algorithm", {}).get("version", "baseline_sparse_boustrophedon"))
    if version == BaselineSparseBoustrophedonPlanner.version:
        return BaselineSparseBoustrophedonPlanner(config)
    if version == SegmentSweepPlanner.version:
        return SegmentSweepPlanner(config)
    raise ValueError(f"unknown algorithm.version: {version}")


def _reverse_segment(segment: SweepSegment) -> SweepSegment:
    return SweepSegment(
        id=segment.id,
        component_id=segment.component_id,
        orientation=segment.orientation,
        line_index=segment.line_index,
        start=segment.end,
        end=segment.start,
        sampled_waypoints=list(reversed(segment.sampled_waypoints)),
        coverage_cells=set(segment.coverage_cells),
        priority_value=segment.priority_value,
        uncovered_value=segment.uncovered_value,
        allowed_uav_ids=set(segment.allowed_uav_ids),
        sweep_cost_m=segment.sweep_cost_m,
    )


def _sample_with_boundaries(values: list[int], step: int) -> list[int]:
    if not values:
        return []
    sampled = values[:: max(1, step)]
    if values[-1] not in sampled:
        sampled.append(values[-1])
    return sampled


def _contiguous(values: list[int]) -> list[list[int]]:
    if not values:
        return []
    groups = [[values[0]]]
    for value in values[1:]:
        if value == groups[-1][-1] + 1:
            groups[-1].append(value)
        else:
            groups.append([value])
    return groups


def _manhattan(a: Position, b: Position) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


def _path_cost_cells(points: list[Position]) -> float:
    return sum(math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(points, points[1:]))


def _connector_cost_m(origin: Position, segments: list[SweepSegment], grid_map: GridMap) -> float:
    cost = 0.0
    current = origin
    cache = SegmentConnectorCostCache(planner_run_id=f"connector_{origin}_{len(segments)}")
    for segment in segments:
        connector = cache.cost(current, segment.start, grid_map)
        if math.isinf(connector):
            connector = _manhattan(current, segment.start) * grid_map.resolution_m
        cost += connector
        current = segment.end
    return cost


def _positions_to_dicts(positions: list[Position]) -> list[dict[str, int]]:
    return [{"x": point.x, "y": point.y} for point in positions]


def _dominant_orientation(segments: list[SweepSegment]) -> str | None:
    if not segments:
        return None
    horizontal = sum(1 for segment in segments if segment.orientation == "horizontal")
    vertical = len(segments) - horizontal
    return "horizontal" if horizontal >= vertical else "vertical"


def _workload_balance(values: list[float]) -> float:
    if not values:
        return 1.0
    mean = sum(values) / len(values)
    if mean <= 0:
        return 1.0
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return 1.0 / (1.0 + math.sqrt(variance) / mean)
