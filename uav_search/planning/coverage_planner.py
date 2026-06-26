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


@dataclass(frozen=True)
class ComponentComplexity:
    component_id: str
    kind: str
    component_cell_count: int
    bounding_box_area: int
    fill_ratio: float
    obstacle_hole_ratio: float
    fragmented_line_count: int
    avg_segments_per_scanline: float
    max_segments_per_scanline: int
    component_aspect_ratio: float
    reachable_uav_count: int
    nearest_uav_distance: float
    priority_cell_count: int
    complexity_score: float


@dataclass(frozen=True)
class SweepCluster:
    id: str
    component_id: str
    segment_ids: list[str]
    segments: list[SweepSegment]
    coverage_cells: set[Position]
    priority_cells: set[Position]
    centroid: Position
    entry_candidates: list[Position]
    exit_candidates: list[Position]
    sweep_cost_m: float
    estimated_internal_connector_cost_m: float
    allowed_uav_ids: set[str]


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


class ComponentComplexityAnalyzer:
    def __init__(self, config: dict | None = None) -> None:
        self.config = config or {}

    def analyze(
        self,
        component: set[Position],
        component_id: str,
        grid_map: GridMap,
        uav_states: list[UAVState],
    ) -> ComponentComplexity:
        if not component:
            return ComponentComplexity(component_id, "tiny", 0, 0, 0.0, 0.0, 0, 0.0, 0, 1.0, 0, math.inf, 0, 0.0)
        min_x = min(cell.x for cell in component)
        max_x = max(cell.x for cell in component)
        min_y = min(cell.y for cell in component)
        max_y = max(cell.y for cell in component)
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        bbox_area = max(1, width * height)
        fill_ratio = len(component) / bbox_area
        line_segment_counts = []
        for y in range(min_y, max_y + 1):
            xs = sorted(cell.x for cell in component if cell.y == y)
            if xs:
                line_segment_counts.append(len(_contiguous(xs)))
        fragmented_line_count = sum(1 for count_value in line_segment_counts if count_value > 1)
        avg_segments = sum(line_segment_counts) / len(line_segment_counts) if line_segment_counts else 0.0
        max_segments = max(line_segment_counts, default=0)
        holes = sum(
            1
            for y in range(min_y, max_y + 1)
            for x in range(min_x, max_x + 1)
            if Position(x, y) not in component and not grid_map.is_passable(Position(x, y))
        )
        obstacle_hole_ratio = holes / bbox_area
        aspect = max(width, height) / max(1, min(width, height))
        reachable_uavs = {
            state.id
            for state in uav_states
            if any(_manhattan(state.position, cell) < math.inf for cell in component)
        }
        nearest = min(
            (_manhattan(state.position, cell) * grid_map.resolution_m for state in uav_states for cell in component),
            default=math.inf,
        )
        priority_count = sum(1 for cell in component if grid_map.get_cell(cell).search_priority > 1.0)
        score = (
            (1.0 - fill_ratio)
            + obstacle_hole_ratio
            + max(0.0, avg_segments - 1.0) * 0.6
            + fragmented_line_count / max(1, len(line_segment_counts))
            + (0.15 if priority_count else 0.0)
        )
        config = self.config.get("algorithm", {}).get("adaptive_component_sweep", self.config)
        tiny_limit = int(config.get("tiny_component_max_cells", 8))
        simple_fill = float(config.get("simple_fill_ratio", 0.65))
        simple_avg = float(config.get("simple_max_avg_segments_per_scanline", 2.2))
        simple_fragment_ratio = float(config.get("simple_max_fragmented_line_ratio", 0.55))
        simple_max_score = float(config.get("simple_max_complexity_score", 1.75))
        fragment_ratio = fragmented_line_count / max(1, len(line_segment_counts))
        if len(component) <= tiny_limit and priority_count == 0:
            kind = "tiny"
        elif (
            fill_ratio >= simple_fill
            and avg_segments <= simple_avg
            and (fragment_ratio <= simple_fragment_ratio or score <= simple_max_score)
        ):
            kind = "simple"
        else:
            kind = "complex"
        return ComponentComplexity(
            component_id=component_id,
            kind=kind,
            component_cell_count=len(component),
            bounding_box_area=bbox_area,
            fill_ratio=fill_ratio,
            obstacle_hole_ratio=obstacle_hole_ratio,
            fragmented_line_count=fragmented_line_count,
            avg_segments_per_scanline=avg_segments,
            max_segments_per_scanline=max_segments,
            component_aspect_ratio=aspect,
            reachable_uav_count=len(reachable_uavs),
            nearest_uav_distance=nearest,
            priority_cell_count=priority_count,
            complexity_score=score,
        )


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


class AdaptiveComponentSweepPlanner(SegmentSweepPlanner):
    version = "adaptive_component_sweep_v1"

    def __init__(self, config: dict | None = None) -> None:
        super().__init__(config)
        self.baseline = BaselineSparseBoustrophedonPlanner(config)
        self.analyzer = ComponentComplexityAnalyzer(config)

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
        reachable_cells = {cell for cell in searchable_cells if grid_map.is_passable(cell) and reachability.any_reachable(cell)}
        components = reachability_components(grid_map, reachable_cells)
        analyses = [
            self.analyzer.analyze(component, f"component_{idx}", grid_map, online)
            for idx, component in enumerate(components, start=1)
        ]
        component_by_id = {analysis.component_id: component for analysis, component in zip(analyses, components)}
        simple_cells = set().union(
            *(component_by_id[analysis.component_id] for analysis in analyses if analysis.kind in {"simple", "tiny"}),
            set(),
        )
        complex_components = [component_by_id[analysis.component_id] for analysis in analyses if analysis.kind == "complex"]
        tasks: list[Task] = []

        if simple_cells:
            simple_tasks = self.baseline.plan_initial_tasks(
                grid_map=grid_map,
                uav_states=online,
                sensor_radius_cells=sensor_radius_cells,
                created_at=created_at,
                reachability=reachability,
                searchable_cells=simple_cells,
            )
            for task in simple_tasks:
                self._mark_adaptive_task(
                    task,
                    component_ids=[analysis.component_id for analysis in analyses if analysis.kind in {"simple", "tiny"}],
                    simple_count=sum(1 for analysis in analyses if analysis.kind in {"simple", "tiny"}),
                    complex_count=sum(1 for analysis in analyses if analysis.kind == "complex"),
                    target_cells=reachable_cells,
                    sensor_radius_cells=sensor_radius_cells,
                )
            tasks.extend(simple_tasks)

        complex_segments: list[SweepSegment] = []
        for index, component in enumerate(complex_components, start=1):
            component_id = f"complex_{index}"
            horizontal = self._segments_for_orientation(component, component_id, "horizontal", grid_map, sensor_radius_cells, reachability)
            vertical = self._segments_for_orientation(component, component_id, "vertical", grid_map, sensor_radius_cells, reachability)
            complex_segments.extend(min((horizontal, vertical), key=lambda candidate: self._orientation_cost(candidate, grid_map)))
        if complex_segments and online:
            clusters = self.cluster_segments(complex_segments, grid_map)
            cluster_bundles = self.assign_clusters_to_uavs(clusters, online, grid_map)
            for sequence, uav in enumerate(online, start=1):
                ordered_clusters = self.order_clusters_for_uav(cluster_bundles.get(uav.id, []), uav.position, grid_map)
                ordered_segments = [segment for cluster in ordered_clusters for segment in cluster.segments]
                task = self._task_from_segments(
                    task_id=f"adaptive_complex_{sequence:03d}",
                    uav_id=uav.id,
                    ordered_segments=ordered_segments,
                    origin=uav.position,
                    grid_map=grid_map,
                    created_at=created_at,
                )
                if task is None:
                    continue
                self._mark_adaptive_task(
                    task,
                    component_ids=sorted({cluster.component_id for cluster in ordered_clusters}),
                    simple_count=sum(1 for analysis in analyses if analysis.kind in {"simple", "tiny"}),
                    complex_count=sum(1 for analysis in analyses if analysis.kind == "complex"),
                    target_cells=reachable_cells,
                    sensor_radius_cells=sensor_radius_cells,
                    clusters=ordered_clusters,
                )
                tasks.append(task)

        self.last_diagnostics.update(
            {
                "simple_component_count": sum(1 for analysis in analyses if analysis.kind in {"simple", "tiny"}),
                "complex_component_count": sum(1 for analysis in analyses if analysis.kind == "complex"),
                "component_count_total": len(analyses),
                "component_complexity": [analysis.__dict__ for analysis in analyses],
            }
        )
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
        task = super().plan_region_task(
            task_id=task_id,
            region=region,
            origin=origin,
            grid_map=grid_map,
            sensor_radius_cells=sensor_radius_cells,
            created_at=created_at,
            reachability=reachability,
            allowed_uav_ids=allowed_uav_ids,
        )
        if task is not None:
            self._mark_adaptive_task(
                task,
                component_ids=["supplemental"],
                simple_count=0,
                complex_count=1,
                target_cells=region,
                sensor_radius_cells=sensor_radius_cells,
            )
        return task

    def cluster_segments(self, segments: list[SweepSegment], grid_map: GridMap) -> list[SweepCluster]:
        config = self.config.get("algorithm", {}).get("adaptive_component_sweep", {})
        min_size = max(1, int(config.get("cluster_min_segments", 2)))
        max_size = max(min_size, int(config.get("cluster_max_segments", 6)))
        clusters: list[SweepCluster] = []
        by_component: dict[str, list[SweepSegment]] = {}
        for segment in segments:
            by_component.setdefault(segment.component_id, []).append(segment)
        for component_id, component_segments in sorted(by_component.items()):
            ordered = sorted(component_segments, key=lambda item: (item.orientation, item.line_index, item.start.x, item.start.y, item.id))
            chunk: list[SweepSegment] = []
            for segment in ordered:
                chunk.append(segment)
                if len(chunk) >= max_size:
                    clusters.append(self._cluster_from_segments(component_id, len(clusters) + 1, chunk, grid_map))
                    chunk = []
            if chunk:
                if clusters and len(chunk) < min_size and clusters[-1].component_id == component_id:
                    merged = list(clusters[-1].segments) + chunk
                    clusters[-1] = self._cluster_from_segments(component_id, len(clusters), merged, grid_map)
                else:
                    clusters.append(self._cluster_from_segments(component_id, len(clusters) + 1, chunk, grid_map))
        self.last_diagnostics.update(
            {
                "cluster_count_total": len(clusters),
                "avg_segments_per_cluster": sum(len(cluster.segments) for cluster in clusters) / len(clusters) if clusters else 0.0,
                "max_segments_per_cluster": max((len(cluster.segments) for cluster in clusters), default=0),
            }
        )
        return clusters

    def assign_clusters_to_uavs(
        self,
        clusters: list[SweepCluster],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, list[SweepCluster]]:
        bundles: dict[str, list[SweepCluster]] = {state.id: [] for state in uav_states}
        route_ends = {state.id: state.position for state in uav_states}
        route_costs = {state.id: 0.0 for state in uav_states}
        cache = SegmentConnectorCostCache(planner_run_id=f"cluster_assign_{id(self)}_{len(clusters)}")
        ordered_clusters = sorted(
            clusters,
            key=lambda cluster: (
                -(len(cluster.priority_cells) / max(1, len(cluster.coverage_cells))),
                -(len(cluster.coverage_cells) / max(cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m, 1.0)),
                cluster.id,
            ),
        )
        for cluster in ordered_clusters:
            candidates: list[tuple[float, str]] = []
            for uav_id in sorted(cluster.allowed_uav_ids):
                if uav_id not in bundles:
                    continue
                connector = min(cache.cost(route_ends[uav_id], entry, grid_map) for entry in cluster.entry_candidates)
                if math.isinf(connector):
                    continue
                projected = route_costs[uav_id] + connector + cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
                projected_max = max(projected, *(value for key, value in route_costs.items() if key != uav_id))
                candidates.append((projected_max, uav_id))
            if not candidates:
                continue
            _, winner = min(candidates, key=lambda item: (item[0], item[1]))
            bundles[winner].append(cluster)
            route_ends[winner] = cluster.exit_candidates[-1]
            route_costs[winner] += cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
        costs = self._cluster_bundle_costs(bundles, uav_states, grid_map)
        self.last_diagnostics.update(
            {
                "cluster_count_per_uav": {uav_id: len(bundle) for uav_id, bundle in bundles.items()},
                "cluster_bundle_cost_per_uav": costs,
                "max_cluster_bundle_cost": max(costs.values(), default=0.0),
                "cluster_workload_balance": _workload_balance(list(costs.values())),
                "cluster_exchange_attempts": 0,
                "cluster_exchange_accepted": 0,
                "max_cluster_cost_before_exchange": max(costs.values(), default=0.0),
                "max_cluster_cost_after_exchange": max(costs.values(), default=0.0),
            }
        )
        return bundles

    def order_clusters_for_uav(
        self,
        clusters: list[SweepCluster],
        origin: Position,
        grid_map: GridMap,
    ) -> list[SweepCluster]:
        remaining = list(clusters)
        ordered: list[SweepCluster] = []
        current = origin
        cache = SegmentConnectorCostCache(planner_run_id=f"cluster_order_{id(self)}_{origin}_{len(clusters)}")
        while remaining:
            idx, cluster = min(
                enumerate(remaining),
                key=lambda item: (
                    item[1].component_id != ordered[-1].component_id if ordered else False,
                    min(cache.cost(current, entry, grid_map) for entry in item[1].entry_candidates),
                    item[1].id,
                ),
            )
            ordered.append(cluster)
            current = cluster.exit_candidates[-1]
            remaining.pop(idx)
        self._record_component_jump_diagnostics(ordered, grid_map)
        return ordered

    def _cluster_from_segments(
        self,
        component_id: str,
        sequence: int,
        segments: list[SweepSegment],
        grid_map: GridMap,
    ) -> SweepCluster:
        ordered_segments = self.order_segments_for_uav(segments, segments[0].start, grid_map)
        coverage = set().union(*(segment.coverage_cells for segment in ordered_segments), set())
        priority = {cell for cell in coverage if grid_map.get_cell(cell).search_priority > 1.0}
        centroid = Position(
            round(sum(cell.x for cell in coverage) / len(coverage)),
            round(sum(cell.y for cell in coverage) / len(coverage)),
        ) if coverage else ordered_segments[0].start
        internal = _connector_cost_m(ordered_segments[0].start, ordered_segments, grid_map)
        return SweepCluster(
            id=f"{component_id}_cluster_{sequence:03d}",
            component_id=component_id,
            segment_ids=[segment.id for segment in ordered_segments],
            segments=ordered_segments,
            coverage_cells=coverage,
            priority_cells=priority,
            centroid=centroid,
            entry_candidates=[ordered_segments[0].start, ordered_segments[0].end],
            exit_candidates=[ordered_segments[-1].end, ordered_segments[-1].start],
            sweep_cost_m=sum(segment.sweep_cost_m for segment in ordered_segments),
            estimated_internal_connector_cost_m=internal,
            allowed_uav_ids=set.intersection(*(set(segment.allowed_uav_ids) for segment in ordered_segments)),
        )

    def _cluster_bundle_costs(
        self,
        bundles: dict[str, list[SweepCluster]],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, float]:
        states = {state.id: state for state in uav_states}
        costs: dict[str, float] = {}
        for uav_id, bundle in bundles.items():
            current = states[uav_id].position if uav_id in states else Position(0, 0)
            cost = 0.0
            for cluster in bundle:
                cost += min(_manhattan(current, entry) * grid_map.resolution_m for entry in cluster.entry_candidates)
                cost += cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
                current = cluster.exit_candidates[-1]
            costs[uav_id] = cost
        return costs

    def _record_component_jump_diagnostics(self, clusters: list[SweepCluster], grid_map: GridMap) -> None:
        inter_costs = [
            _manhattan(first.exit_candidates[-1], second.entry_candidates[0]) * grid_map.resolution_m
            for first, second in zip(clusters, clusters[1:])
            if first.component_id != second.component_id
        ]
        intra_costs = [
            _manhattan(first.exit_candidates[-1], second.entry_candidates[0]) * grid_map.resolution_m
            for first, second in zip(clusters, clusters[1:])
            if first.component_id == second.component_id
        ]
        self.last_diagnostics.update(
            {
                "intra_component_connector_cost": sum(intra_costs),
                "inter_component_connector_cost": sum(inter_costs),
                "inter_component_jump_count": len(inter_costs),
                "max_inter_component_jump_m": max(inter_costs, default=0.0),
                "avg_inter_component_jump_m": sum(inter_costs) / len(inter_costs) if inter_costs else 0.0,
            }
        )

    def _mark_adaptive_task(
        self,
        task: Task,
        *,
        component_ids: list[str],
        simple_count: int,
        complex_count: int,
        target_cells: set[Position],
        sensor_radius_cells: int,
        clusters: list[SweepCluster] | None = None,
    ) -> None:
        planned = simulate_planned_coverage(task.coverage_waypoints, sensor_radius_cells, target_cells)
        priority_cells = {cell for cell in target_cells if task.metadata.get("priority") or False}
        task.metadata.update(
            {
                "planner_version": self.version,
                "component_ids": list(component_ids),
                "simple_component_count": simple_count,
                "complex_component_count": complex_count,
                "cluster_ids": [cluster.id for cluster in clusters or []],
                "segment_ids": list(dict.fromkeys(task.metadata.get("segment_ids", []))),
                "planned_coverage_ratio": len(planned) / len(target_cells) if target_cells else 1.0,
                "planned_priority_coverage_ratio": (
                    len(planned & priority_cells) / len(priority_cells) if priority_cells else 1.0
                ),
            }
        )
        for key, value in self.last_diagnostics.items():
            if key.startswith("cluster_") or key in {
                "intra_component_connector_cost",
                "inter_component_connector_cost",
                "inter_component_jump_count",
                "max_inter_component_jump_m",
                "avg_inter_component_jump_m",
                "component_count_total",
            }:
                task.metadata[key] = value


def create_coverage_planner(config: dict | None = None) -> CoveragePlanner:
    version = str((config or {}).get("algorithm", {}).get("version", "baseline_sparse_boustrophedon"))
    if version == BaselineSparseBoustrophedonPlanner.version:
        return BaselineSparseBoustrophedonPlanner(config)
    if version == SegmentSweepPlanner.version:
        return SegmentSweepPlanner(config)
    if version == AdaptiveComponentSweepPlanner.version:
        return AdaptiveComponentSweepPlanner(config)
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


def simulate_planned_coverage(
    waypoints: list[Position],
    sensor_radius_cells: int,
    target_cells: set[Position],
) -> set[Position]:
    radius_sq = sensor_radius_cells * sensor_radius_cells
    covered: set[Position] = set()
    for waypoint in waypoints:
        for cell in target_cells:
            if (cell.x - waypoint.x) ** 2 + (cell.y - waypoint.y) ** 2 <= radius_sq:
                covered.add(cell)
    return covered


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
