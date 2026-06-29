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
from uav_search.task.task_generator import compute_region_value, estimate_task_cost, generate_boustrophedon_path, generate_initial_tasks
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


def is_clustered_launch(
    uav_states: list[UAVState],
    grid_map: GridMap,
    *,
    searchable_cells: set[Position],
    config: dict | None = None,
) -> tuple[bool, dict[str, object]]:
    cfg = config or {}
    online = [state for state in uav_states if state.status != UAVStatus.OFFLINE]
    bbox = _uav_bbox(online)
    diagnostics: dict[str, object] = {
        "clustered_launch_detected": False,
        "clustered_launch_uav_count": len(online),
        "clustered_launch_bbox": bbox,
        "clustered_launch_reason": "disabled" if not bool(cfg.get("clustered_launch_enabled", True)) else "not_clustered",
    }
    if not bool(cfg.get("clustered_launch_enabled", True)):
        return False, diagnostics
    if len(online) < 2:
        diagnostics["clustered_launch_reason"] = "too_few_online_uavs"
        return False, diagnostics
    min_cells = int(cfg.get("clustered_launch_min_searchable_cells", 300))
    if len(searchable_cells) < min_cells:
        diagnostics["clustered_launch_reason"] = "search_area_too_small"
        return False, diagnostics

    max_pairwise = max(_manhattan(first.position, second.position) for first in online for second in online)
    pairwise_limit = int(cfg.get("clustered_launch_max_pairwise_distance_cells", 8))
    bbox_ratio = float(cfg.get("clustered_launch_bbox_ratio", 0.18))
    bbox_match = (
        bbox["width"] <= max(1, math.ceil(grid_map.width_cells * bbox_ratio))
        and bbox["height"] <= max(1, math.ceil(grid_map.height_cells * bbox_ratio))
    )
    pairwise_match = max_pairwise <= pairwise_limit
    if pairwise_match or bbox_match:
        diagnostics["clustered_launch_detected"] = True
        diagnostics["clustered_launch_reason"] = "max_pairwise_distance" if pairwise_match else "bbox_ratio"
        return True, diagnostics
    diagnostics["clustered_launch_reason"] = "spread_out"
    return False, diagnostics


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
        *,
        reachability: ReachabilityIndex | None = None,
        reachable_uav_ids: set[str] | None = None,
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
        if reachable_uav_ids is not None:
            reachable_uavs = set(reachable_uav_ids)
        elif reachability is not None:
            reachable_uavs = {
                uav_id
                for cell in component
                for uav_id in reachability.reachable_uavs(cell)
            }
        else:
            reachable_uavs = {state.id for state in uav_states if state.status != UAVStatus.OFFLINE}
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
            self.analyzer.analyze(component, f"component_{idx}", grid_map, online, reachability=reachability)
            for idx, component in enumerate(components, start=1)
        ]
        component_by_id = {analysis.component_id: component for analysis, component in zip(analyses, components)}
        simple_cells = set().union(
            *(component_by_id[analysis.component_id] for analysis in analyses if analysis.kind in {"simple", "tiny"}),
            set(),
        )
        complex_components = [component_by_id[analysis.component_id] for analysis in analyses if analysis.kind == "complex"]
        tasks: list[Task] = []
        simple_guardrail_metadata: dict[str, object] = self._simple_guardrail_metadata(
            simple_cells=simple_cells,
            analyses=analyses,
            online=online,
        )
        adaptive_config = self._adaptive_config()
        clustered_detected, clustered_metadata = is_clustered_launch(
            online,
            grid_map,
            searchable_cells=reachable_cells,
            config=adaptive_config,
        )
        self.last_diagnostics.update(clustered_metadata)
        if simple_cells:
            if clustered_detected and len(online) >= 2:
                simple_tasks = self.plan_clustered_launch_sector_tasks(
                    searchable_cells=simple_cells,
                    grid_map=grid_map,
                    uav_states=online,
                    sensor_radius_cells=sensor_radius_cells,
                    created_at=created_at,
                    reachability=reachability,
                )
                frontload_metadata = self._apply_5uav_simple_frontload(
                    [],
                    simple_cells=simple_cells,
                    online=online,
                    grid_map=grid_map,
                )
                simple_guardrail_metadata = {
                    **simple_guardrail_metadata,
                    **clustered_metadata,
                    **frontload_metadata,
                    "simple_guardrail_triggered_count": 0,
                    "simple_guardrail_component_ids": [],
                    "chosen_component_planner": {"simple": "clustered_launch_sector_sweep"},
                }
            else:
                simple_tasks = self.baseline.plan_initial_tasks(
                    grid_map=grid_map,
                    uav_states=online,
                    sensor_radius_cells=sensor_radius_cells,
                    created_at=created_at,
                    reachability=reachability,
                    searchable_cells=simple_cells,
                )
                frontload_metadata = self._apply_5uav_simple_frontload(
                    simple_tasks,
                    simple_cells=simple_cells,
                    online=online,
                    grid_map=grid_map,
                )
                if frontload_metadata.get("simple_frontload_enabled"):
                    simple_guardrail_metadata = {
                        **simple_guardrail_metadata,
                        **clustered_metadata,
                        "simple_guardrail_triggered_count": 0,
                        "simple_guardrail_component_ids": [],
                        "chosen_component_planner": {"simple": "frontload_baseline"},
                    }
                else:
                    baseline_cost = sum(task.estimated_cost_m for task in simple_tasks)
                    simple_guardrail_metadata = {
                        **simple_guardrail_metadata,
                        **clustered_metadata,
                        "baseline_estimated_cost": baseline_cost,
                        "adaptive_estimated_cost": baseline_cost,
                        "estimated_connector_cost": sum(task.metadata.get("estimated_connector_cost_m", 0.0) for task in simple_tasks),
                    }
            self.last_diagnostics.update(simple_guardrail_metadata)
            for task in simple_tasks:
                self._mark_adaptive_task(
                    task,
                    component_ids=[analysis.component_id for analysis in analyses if analysis.kind in {"simple", "tiny"}],
                    simple_count=sum(1 for analysis in analyses if analysis.kind in {"simple", "tiny"}),
                    complex_count=sum(1 for analysis in analyses if analysis.kind == "complex"),
                    target_cells=reachable_cells,
                    sensor_radius_cells=sensor_radius_cells,
                    grid_map=grid_map,
                    extra_metadata={**clustered_metadata, **frontload_metadata},
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
            complex_cells = set().union(*complex_components, set())
            complex_guardrail_metadata, fallback_complex_tasks = self._complex_guardrail_result(
                complex_cells=complex_cells,
                complex_analyses=[analysis for analysis in analyses if analysis.kind == "complex"],
                cluster_bundles=cluster_bundles,
                grid_map=grid_map,
                online=online,
                sensor_radius_cells=sensor_radius_cells,
                created_at=created_at,
                reachability=reachability,
            )
            self.last_diagnostics.update(complex_guardrail_metadata)
            if fallback_complex_tasks is not None:
                for task in fallback_complex_tasks:
                    self._mark_adaptive_task(
                        task,
                        component_ids=[analysis.component_id for analysis in analyses if analysis.kind == "complex"],
                        simple_count=sum(1 for analysis in analyses if analysis.kind in {"simple", "tiny"}),
                        complex_count=sum(1 for analysis in analyses if analysis.kind == "complex"),
                        target_cells=complex_cells,
                        sensor_radius_cells=sensor_radius_cells,
                        grid_map=grid_map,
                        extra_metadata=complex_guardrail_metadata,
                )
                tasks.extend(fallback_complex_tasks)
            else:
                self.last_diagnostics.update(complex_guardrail_metadata)
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
                        grid_map=grid_map,
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
        if simple_guardrail_metadata.get("simple_guardrail_triggered_count", 0) and not complex_segments:
            return tasks
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
                grid_map=grid_map,
            )
        return task

    def plan_clustered_launch_sector_tasks(
        self,
        *,
        searchable_cells: set[Position],
        grid_map: GridMap,
        uav_states: list[UAVState],
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
    ) -> list[Task]:
        online = [state for state in uav_states if state.status != UAVStatus.OFFLINE]
        if not online or not searchable_cells:
            return []
        entry_side = _nearest_entry_side(searchable_cells, online)
        orientation = "horizontal" if entry_side in {"west", "east"} else "vertical"
        sectors = _build_clustered_sectors(searchable_cells, len(online), orientation)
        tasks: list[Task] = []
        costs: dict[str, float] = {}
        cells_per_uav: dict[str, int] = {}
        assigned: set[str] = set()
        launch_center = _centroid([state.position for state in online])
        for index, sector in enumerate(sectors, start=1):
            candidates = [state for state in online if state.id not in assigned] or online
            uav = min(
                candidates,
                key=lambda state: (
                    _sector_entry_cost(state.position, sector, entry_side),
                    state.id,
                ),
            )
            assigned.add(uav.id)
            waypoints = generate_boustrophedon_path(sector, sensor_radius_cells)
            if not waypoints:
                continue
            waypoints = _orient_waypoints_from_entry(waypoints, entry_side, launch_center)
            uncovered_value, priority_value = compute_region_value(sector, grid_map)
            connector = _manhattan(uav.position, waypoints[0]) * grid_map.resolution_m
            sweep = estimate_task_cost(waypoints, waypoints[0], grid_map.resolution_m)
            estimated = connector + sweep
            costs[uav.id] = estimated
            cells_per_uav[uav.id] = len(sector)
            task = Task(
                id=f"clustered_sector_{index:03d}",
                type=TaskType.SEARCH,
                priority=max(grid_map.get_cell(cell).search_priority for cell in sector),
                target_cells=set(sector),
                entry_point=waypoints[0],
                waypoints=waypoints,
                coverage_waypoints=list(waypoints),
                estimated_cost_m=estimated,
                created_at=created_at,
                updated_at=created_at,
                uncovered_value=uncovered_value,
                priority_value=priority_value,
                score=(uncovered_value + priority_value) / max(estimated, 1.0),
                allowed_uav_ids={uav.id},
                metadata={
                    "planner_version": self.version,
                    "clustered_launch_sector_task": True,
                    "clustered_sector_index": index,
                    "clustered_sector_orientation": orientation,
                    "clustered_sector_entry_side": entry_side,
                    "clustered_sector_cells": len(sector),
                    "estimated_connector_cost_m": connector,
                    "estimated_sweep_cost_m": sweep,
                    "coverage_waypoints": _positions_to_dicts(waypoints),
                },
            )
            self._mark_adaptive_task(
                task,
                component_ids=["clustered_launch_simple"],
                simple_count=1,
                complex_count=0,
                target_cells=searchable_cells,
                sensor_radius_cells=sensor_radius_cells,
                grid_map=grid_map,
                extra_metadata={"clustered_launch_sector_task": True},
            )
            tasks.append(task)

        diagnostics = {
            "clustered_sector_count": len(tasks),
            "clustered_sector_orientation": orientation,
            "clustered_sector_entry_side": entry_side,
            "clustered_sector_cost_per_uav": costs,
            "clustered_sector_cells_per_uav": cells_per_uav,
            "clustered_sector_workload_balance": _workload_balance(list(costs.values())),
        }
        self.last_diagnostics.update(diagnostics)
        for task in tasks:
            task.metadata.update(diagnostics)
        return tasks

    def _apply_5uav_simple_frontload(
        self,
        tasks: list[Task],
        *,
        simple_cells: set[Position],
        online: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, object]:
        config = self._adaptive_config()
        enabled = bool(config.get("enable_5uav_simple_frontload", False))
        min_uavs = int(config.get("frontload_min_uav_count", 5))
        min_cells = int(config.get("frontload_min_component_cells", 0))
        coverage_target = float(config.get("frontload_coverage_target", 0.70))
        priority_cells = {cell for cell in simple_cells if grid_map.get_cell(cell).search_priority > 1.0}
        active = enabled and len(online) >= min_uavs and len(simple_cells) >= min_cells
        metadata: dict[str, object] = {
            "simple_frontload_enabled": active,
            "frontload_component_count": 1 if active and simple_cells else 0,
            "frontload_target_cells": len(simple_cells) if active else 0,
            "frontload_coverage_target": coverage_target if active else 0.0,
            "frontload_priority_cells": len(priority_cells) if active else 0,
            "frontload_uav_ids": [state.id for state in online[:min_uavs]] if active else [],
        }
        self.last_diagnostics.update(metadata)
        if not active or not tasks:
            return metadata

        centroid = Position(
            round(sum(cell.x for cell in simple_cells) / len(simple_cells)),
            round(sum(cell.y for cell in simple_cells) / len(simple_cells)),
        )
        for task in tasks:
            if not task.coverage_waypoints:
                continue
            start_distance = _manhattan(task.coverage_waypoints[0], centroid)
            end_distance = _manhattan(task.coverage_waypoints[-1], centroid)
            if end_distance < start_distance:
                task.coverage_waypoints = list(reversed(task.coverage_waypoints))
                task.waypoints = list(reversed(task.waypoints))
                task.entry_point = task.coverage_waypoints[0]
                task.metadata["frontload_reversed"] = True
        return metadata

    def _simple_guardrail_metadata(
        self,
        *,
        simple_cells: set[Position],
        analyses: list[ComponentComplexity],
        online: list[UAVState],
    ) -> dict[str, object]:
        config = self._adaptive_config()
        enabled = bool(config.get("simple_guardrail_enabled", True))
        simple_ids = [analysis.component_id for analysis in analyses if analysis.kind in {"simple", "tiny"}]
        simple_scores = [analysis.complexity_score for analysis in analyses if analysis.kind in {"simple", "tiny"}]
        conservative_uav_count = len(online) <= 3
        obvious_complexity = max(simple_scores, default=0.0) > float(config.get("simple_guardrail_high_complexity_score", 2.0))
        triggered = bool(enabled and simple_cells and conservative_uav_count and not obvious_complexity)
        return {
            "simple_guardrail_triggered_count": 1 if triggered else 0,
            "simple_guardrail_component_ids": simple_ids if triggered else [],
            "baseline_estimated_cost": 0.0,
            "adaptive_estimated_cost": 0.0,
            "estimated_connector_cost": 0.0,
            "chosen_component_planner": {"simple": "baseline" if triggered else "adaptive"},
            "simple_guardrail_max_cost_ratio": float(config.get("simple_guardrail_max_cost_ratio", 1.03)),
            "simple_guardrail_max_connector_ratio": float(config.get("simple_guardrail_max_connector_ratio", 1.05)),
        }

    def _complex_guardrail_result(
        self,
        *,
        complex_cells: set[Position],
        complex_analyses: list[ComponentComplexity],
        cluster_bundles: dict[str, list[SweepCluster]],
        grid_map: GridMap,
        online: list[UAVState],
        sensor_radius_cells: int,
        created_at: float,
        reachability: ReachabilityIndex,
    ) -> tuple[dict[str, object], list[Task] | None]:
        config = self._adaptive_config()
        enabled = bool(config.get("complex_guardrail_enabled", True))
        baseline_tasks = self.baseline.plan_initial_tasks(
            grid_map=grid_map,
            uav_states=online,
            sensor_radius_cells=sensor_radius_cells,
            created_at=created_at,
            reachability=reachability,
            searchable_cells=complex_cells,
        ) if complex_cells else []
        baseline_costs = [task.estimated_cost_m for task in baseline_tasks]
        adaptive_costs = list(self._cluster_bundle_costs(cluster_bundles, online, grid_map).values())
        baseline_total = sum(baseline_costs)
        adaptive_total = sum(adaptive_costs)
        baseline_max = max(baseline_costs, default=0.0)
        adaptive_max = max(adaptive_costs, default=0.0)
        max_ratio = adaptive_max / max(baseline_max, grid_map.resolution_m)
        total_ratio = adaptive_total / max(baseline_total, grid_map.resolution_m)
        max_complexity = max((analysis.complexity_score for analysis in complex_analyses), default=0.0)
        max_cost_ratio = float(config.get("complex_guardrail_max_bundle_cost_ratio", 1.35))
        max_total_ratio = float(config.get("complex_guardrail_max_total_cost_ratio", 1.15))
        max_complexity_score = float(config.get("complex_guardrail_max_complexity_score", 2.4))
        triggered = bool(
            enabled
            and baseline_tasks
            and adaptive_costs
            and adaptive_max > baseline_max * max_cost_ratio
            and adaptive_total > baseline_total * max_total_ratio
            and max_complexity <= max_complexity_score
        )
        chosen = dict(self.last_diagnostics.get("chosen_component_planner", {}))
        chosen["complex"] = "baseline" if triggered else "cluster"
        metadata: dict[str, object] = {
            "complex_guardrail_triggered_count": 1 if triggered else 0,
            "complex_guardrail_component_ids": [analysis.component_id for analysis in complex_analyses] if triggered else [],
            "complex_baseline_estimated_cost": baseline_total,
            "complex_adaptive_estimated_cost": adaptive_total,
            "complex_baseline_max_cost": baseline_max,
            "complex_adaptive_max_cost": adaptive_max,
            "complex_guardrail_max_bundle_cost_ratio": max_cost_ratio,
            "complex_guardrail_max_total_cost_ratio": max_total_ratio,
            "complex_guardrail_max_complexity_score": max_complexity_score,
            "complex_guardrail_observed_bundle_cost_ratio": max_ratio,
            "complex_guardrail_observed_total_cost_ratio": total_ratio,
            "chosen_component_planner": chosen,
        }
        return metadata, baseline_tasks if triggered else None

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
            candidates: list[tuple[float, float, str, Position]] = []
            for uav_id in sorted(cluster.allowed_uav_ids):
                if uav_id not in bundles:
                    continue
                connector, _, exit_point = self._best_cluster_connection(route_ends[uav_id], cluster, grid_map, cache)
                if math.isinf(connector):
                    continue
                projected = route_costs[uav_id] + connector + cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
                other_costs = [value for key, value in route_costs.items() if key != uav_id]
                projected_max = max([projected, *other_costs])
                candidates.append((projected_max, projected, uav_id, exit_point))
            if not candidates:
                continue
            _, projected_cost, winner, exit_point = min(candidates, key=lambda item: (item[0], item[1], item[2]))
            bundles[winner].append(cluster)
            route_ends[winner] = exit_point
            route_costs[winner] = projected_cost
        before_costs = self._cluster_bundle_costs(bundles, uav_states, grid_map)
        bundles = self.improve_cluster_bundles(bundles, uav_states, grid_map)
        costs = self._cluster_bundle_costs(bundles, uav_states, grid_map)
        connector_costs = self._cluster_bundle_connector_costs(bundles, uav_states, grid_map)
        exchange = dict(self.last_diagnostics)
        self.last_diagnostics.update(
            {
                "cluster_count_per_uav": {uav_id: len(bundle) for uav_id, bundle in bundles.items()},
                "cluster_bundle_cost_per_uav": costs,
                "cluster_assignment_connector_cost_per_uav": connector_costs,
                "cluster_assignment_total_cost_per_uav": costs,
                "max_cluster_bundle_cost": max(costs.values(), default=0.0),
                "cluster_workload_balance": _workload_balance(list(costs.values())),
                "max_cluster_cost_before_exchange": exchange.get(
                    "max_cluster_cost_before_exchange", max(before_costs.values(), default=0.0)
                ),
                "max_cluster_cost_after_exchange": max(costs.values(), default=0.0),
            }
        )
        return bundles

    def improve_cluster_bundles(
        self,
        bundles: dict[str, list[SweepCluster]],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, list[SweepCluster]]:
        config = self._adaptive_config()
        max_iterations = int(config.get("cluster_exchange_iterations", 0))
        max_total_increase_ratio = float(config.get("cluster_exchange_max_total_cost_increase_ratio", 0.05))
        if max_iterations <= 0:
            copied = {uav_id: list(bundle) for uav_id, bundle in bundles.items()}
            costs = self._cluster_bundle_costs(copied, uav_states, grid_map)
            self.last_diagnostics.update(
                {
                    "cluster_exchange_attempts": 0,
                    "cluster_exchange_accepted": 0,
                    "max_cluster_cost_before_exchange": max(costs.values(), default=0.0),
                    "max_cluster_cost_after_exchange": max(costs.values(), default=0.0),
                    "total_cluster_cost_before_exchange": sum(costs.values()),
                    "total_cluster_cost_after_exchange": sum(costs.values()),
                }
            )
            return copied

        improved = {uav_id: list(bundle) for uav_id, bundle in bundles.items()}
        before = self._cluster_bundle_costs(improved, uav_states, grid_map)
        attempts = 0
        accepted = 0

        for _ in range(max_iterations):
            costs = self._cluster_bundle_costs(improved, uav_states, grid_map)
            if not costs:
                break
            max_uav = max(costs, key=costs.get)
            current_max = max(costs.values(), default=0.0)
            current_total = sum(costs.values())
            best: tuple[float, float, dict[str, list[SweepCluster]]] | None = None

            for idx, cluster in sorted(
                enumerate(improved.get(max_uav, [])),
                key=lambda item: (-item[1].sweep_cost_m, -len(item[1].coverage_cells), item[1].id),
            ):
                for target_uav in sorted(improved):
                    if target_uav == max_uav or target_uav not in cluster.allowed_uav_ids:
                        continue
                    attempts += 1
                    candidate = {uav_id: list(bundle) for uav_id, bundle in improved.items()}
                    candidate[max_uav].pop(idx)
                    candidate[target_uav].append(cluster)
                    candidate = self._order_cluster_bundles(candidate, uav_states, grid_map)
                    candidate_costs = self._cluster_bundle_costs(candidate, uav_states, grid_map)
                    candidate_max = max(candidate_costs.values(), default=0.0)
                    candidate_total = sum(candidate_costs.values())
                    if candidate_max < current_max and candidate_total <= current_total * (1.0 + max_total_increase_ratio):
                        if best is None or (candidate_max, candidate_total) < (best[0], best[1]):
                            best = (candidate_max, candidate_total, candidate)

            for source_uav in sorted(improved):
                if source_uav == max_uav:
                    continue
                for left_idx, left in enumerate(list(improved.get(max_uav, []))):
                    for right_idx, right in enumerate(list(improved.get(source_uav, []))):
                        if source_uav not in left.allowed_uav_ids or max_uav not in right.allowed_uav_ids:
                            continue
                        attempts += 1
                        candidate = {uav_id: list(bundle) for uav_id, bundle in improved.items()}
                        candidate[max_uav][left_idx] = right
                        candidate[source_uav][right_idx] = left
                        candidate = self._order_cluster_bundles(candidate, uav_states, grid_map)
                        candidate_costs = self._cluster_bundle_costs(candidate, uav_states, grid_map)
                        candidate_max = max(candidate_costs.values(), default=0.0)
                        candidate_total = sum(candidate_costs.values())
                        if candidate_max < current_max and candidate_total <= current_total * (1.0 + max_total_increase_ratio):
                            if best is None or (candidate_max, candidate_total) < (best[0], best[1]):
                                best = (candidate_max, candidate_total, candidate)

            if best is None:
                break
            improved = best[2]
            accepted += 1

        after = self._cluster_bundle_costs(improved, uav_states, grid_map)
        self.last_diagnostics.update(
            {
                "cluster_exchange_attempts": attempts,
                "cluster_exchange_accepted": accepted,
                "max_cluster_cost_before_exchange": max(before.values(), default=0.0),
                "max_cluster_cost_after_exchange": max(after.values(), default=0.0),
                "total_cluster_cost_before_exchange": sum(before.values()),
                "total_cluster_cost_after_exchange": sum(after.values()),
            }
        )
        return improved

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
        config = self._adaptive_config()
        threshold_first = bool(config.get("threshold_first_ordering_enabled", True))
        target_ratio = min(
            1.0,
            float(self.config.get("search", {}).get("mission_complete_coverage_threshold", 0.95))
            + float(config.get("threshold_first_coverage_margin", 0.015)),
        )
        all_cluster_cells = set().union(*(cluster.coverage_cells for cluster in remaining), set())
        threshold_target_cells = math.ceil(len(all_cluster_cells) * target_ratio)
        threshold_phase_count = 0
        low_gain_pre_threshold = 0
        far_pre_threshold = 0
        covered: set[Position] = set()
        threshold_covered: set[Position] = set()
        while remaining:
            before_threshold = threshold_first and len(covered) < threshold_target_cells
            if before_threshold:
                idx, cluster, score_data = self._select_threshold_first_cluster(
                    remaining,
                    current,
                    covered,
                    ordered[-1].component_id if ordered else None,
                    grid_map,
                    cache,
                )
                threshold_phase_count += 1
                if score_data["gain_per_meter"] < float(config.get("threshold_low_gain_per_meter", 0.03)):
                    low_gain_pre_threshold += 1
                if score_data["estimated_arrival_cost"] > max(grid_map.resolution_m, float(config.get("threshold_far_cluster_cost_m", 120.0))):
                    far_pre_threshold += 1
            else:
                idx, cluster = min(
                    enumerate(remaining),
                    key=lambda item: (
                        item[1].component_id != ordered[-1].component_id if ordered else False,
                        self._best_cluster_connection(current, item[1], grid_map, cache)[0],
                        item[1].id,
                    ),
                )
            ordered.append(cluster)
            covered.update(cluster.coverage_cells)
            if before_threshold:
                threshold_covered.update(cluster.coverage_cells)
            _, _, current = self._best_cluster_connection(current, cluster, grid_map, cache)
            remaining.pop(idx)
        estimated_threshold_ratio = len(threshold_covered & all_cluster_cells) / len(all_cluster_cells) if all_cluster_cells else 1.0
        threshold_jumps = sum(
            1
            for first, second in zip(ordered[:threshold_phase_count], ordered[1:threshold_phase_count])
            if first.component_id != second.component_id
        )
        self.last_diagnostics.update(
            {
                "threshold_phase_cluster_count": threshold_phase_count,
                "post_threshold_cluster_count": max(0, len(ordered) - threshold_phase_count),
                "estimated_threshold_coverage_ratio": min(1.0, estimated_threshold_ratio),
                "threshold_first_ordering_enabled": threshold_first,
                "low_gain_pre_threshold_cluster_count": low_gain_pre_threshold,
                "far_pre_threshold_cluster_count": far_pre_threshold,
                "threshold_phase_inter_component_jump_count": threshold_jumps,
            }
        )
        self._record_component_jump_diagnostics(ordered, grid_map)
        return ordered

    def _select_threshold_first_cluster(
        self,
        clusters: list[SweepCluster],
        current: Position,
        covered: set[Position],
        current_component_id: str | None,
        grid_map: GridMap,
        cache: SegmentConnectorCostCache,
    ) -> tuple[int, SweepCluster, dict[str, float]]:
        scored: list[tuple[float, float, int, str, int, SweepCluster, dict[str, float]]] = []
        priority_weight = float(self.config.get("search", {}).get("priority_cell_weight", 3.0))
        for index, cluster in enumerate(clusters):
            connector, _, _ = self._best_cluster_connection(current, cluster, grid_map, cache)
            if math.isinf(connector):
                connector = _manhattan(current, cluster.entry_candidates[0]) * grid_map.resolution_m
            marginal = cluster.coverage_cells - covered
            marginal_priority = cluster.priority_cells - covered
            weighted_gain = len(marginal) + priority_weight * len(marginal_priority)
            estimated_arrival = connector + cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
            gain_per_meter = weighted_gain / max(estimated_arrival, grid_map.resolution_m)
            score = gain_per_meter
            scored.append(
                (
                    score,
                    gain_per_meter,
                    -index,
                    cluster.id,
                    index,
                    cluster,
                    {
                        "marginal_coverage_cells": float(len(marginal)),
                        "marginal_priority_cells": float(len(marginal_priority)),
                        "estimated_arrival_cost": float(estimated_arrival),
                        "gain_per_meter": float(gain_per_meter),
                        "threshold_contribution_score": float(score),
                    },
                )
            )
        if not scored:
            raise ValueError("cannot order empty cluster list")
        best = max(scored, key=lambda item: (item[0], item[1], item[2], item[3]))
        if current_component_id is None:
            return best[4], best[5], best[6]
        same_component = [item for item in scored if item[5].component_id == current_component_id]
        if same_component and best[5].component_id != current_component_id:
            best_same = max(same_component, key=lambda item: (item[0], item[1], item[2], item[3]))
            switch_ratio = float(self._adaptive_config().get("component_switch_gain_ratio", 1.35))
            if best[1] < best_same[1] * switch_ratio:
                return best_same[4], best_same[5], best_same[6]
        return best[4], best[5], best[6]

    def _adaptive_config(self) -> dict:
        return dict(self.config.get("algorithm", {}).get("adaptive_component_sweep", {}))

    def _best_cluster_connection(
        self,
        current: Position,
        cluster: SweepCluster,
        grid_map: GridMap,
        cache: SegmentConnectorCostCache | None = None,
    ) -> tuple[float, Position, Position]:
        cache = cache or SegmentConnectorCostCache(planner_run_id=f"cluster_connect_{id(self)}")
        choices: list[tuple[float, int, Position, Position]] = []
        for index, entry in enumerate(cluster.entry_candidates):
            exit_point = cluster.exit_candidates[index] if index < len(cluster.exit_candidates) else cluster.exit_candidates[-1]
            choices.append((cache.cost(current, entry, grid_map), index, entry, exit_point))
        cost, _, entry, exit_point = min(choices, key=lambda item: (item[0], item[1]))
        return cost, entry, exit_point

    def _order_cluster_bundles(
        self,
        bundles: dict[str, list[SweepCluster]],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, list[SweepCluster]]:
        states = {state.id: state for state in uav_states}
        return {
            uav_id: self.order_clusters_for_uav(bundle, states[uav_id].position, grid_map) if uav_id in states else list(bundle)
            for uav_id, bundle in bundles.items()
        }

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
        cache = SegmentConnectorCostCache(planner_run_id=f"cluster_cost_{id(self)}_{sum(len(bundle) for bundle in bundles.values())}")
        for uav_id, bundle in bundles.items():
            current = states[uav_id].position if uav_id in states else Position(0, 0)
            cost = 0.0
            for cluster in bundle:
                connector, _, exit_point = self._best_cluster_connection(current, cluster, grid_map, cache)
                if math.isinf(connector):
                    connector = _manhattan(current, cluster.entry_candidates[0]) * grid_map.resolution_m
                    exit_point = cluster.exit_candidates[0]
                cost += connector
                cost += cluster.sweep_cost_m + cluster.estimated_internal_connector_cost_m
                current = exit_point
            costs[uav_id] = cost
        return costs

    def _cluster_bundle_connector_costs(
        self,
        bundles: dict[str, list[SweepCluster]],
        uav_states: list[UAVState],
        grid_map: GridMap,
    ) -> dict[str, float]:
        states = {state.id: state for state in uav_states}
        costs: dict[str, float] = {}
        cache = SegmentConnectorCostCache(
            planner_run_id=f"cluster_connector_cost_{id(self)}_{sum(len(bundle) for bundle in bundles.values())}"
        )
        for uav_id, bundle in bundles.items():
            current = states[uav_id].position if uav_id in states else Position(0, 0)
            connector_total = 0.0
            for cluster in bundle:
                connector, _, exit_point = self._best_cluster_connection(current, cluster, grid_map, cache)
                if math.isinf(connector):
                    connector = _manhattan(current, cluster.entry_candidates[0]) * grid_map.resolution_m
                    exit_point = cluster.exit_candidates[0]
                connector_total += connector
                current = exit_point
            costs[uav_id] = connector_total
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
        grid_map: GridMap,
        clusters: list[SweepCluster] | None = None,
        extra_metadata: dict[str, object] | None = None,
    ) -> None:
        planned = simulate_planned_coverage(task.coverage_waypoints, sensor_radius_cells, target_cells)
        priority_cells = {cell for cell in target_cells if grid_map.get_cell(cell).search_priority > 1.0}
        task.metadata.update(
            {
                "planner_version": self.version,
                "component_ids": list(component_ids),
                "simple_component_count": simple_count,
                "complex_component_count": complex_count,
                "cluster_ids": [cluster.id for cluster in clusters or []],
                "segment_ids": list(dict.fromkeys(task.metadata.get("segment_ids", []))),
                "sensor_radius_cells": sensor_radius_cells,
                "coverage_waypoints": _positions_to_dicts(task.coverage_waypoints),
                "planned_coverage_ratio": len(planned) / len(target_cells) if target_cells else 1.0,
                "planned_priority_coverage_ratio": (
                    len(planned & priority_cells) / len(priority_cells) if priority_cells else 1.0
                ),
            }
        )
        task.metadata.update(extra_metadata or {})
        for key, value in self.last_diagnostics.items():
            if key.startswith("cluster_") or key in {
                "intra_component_connector_cost",
                "inter_component_connector_cost",
                "inter_component_jump_count",
                "max_inter_component_jump_m",
                "avg_inter_component_jump_m",
                "component_count_total",
                "simple_frontload_enabled",
                "frontload_component_count",
                "frontload_target_cells",
                "frontload_coverage_target",
                "frontload_priority_cells",
                "frontload_uav_ids",
                "simple_guardrail_triggered_count",
                "simple_guardrail_component_ids",
                "baseline_estimated_cost",
                "adaptive_estimated_cost",
                "estimated_connector_cost",
                "chosen_component_planner",
                "simple_guardrail_max_cost_ratio",
                "simple_guardrail_max_connector_ratio",
                "complex_guardrail_triggered_count",
                "complex_guardrail_component_ids",
                "complex_baseline_estimated_cost",
                "complex_adaptive_estimated_cost",
                "complex_baseline_max_cost",
                "complex_adaptive_max_cost",
                "complex_guardrail_max_bundle_cost_ratio",
                "complex_guardrail_max_total_cost_ratio",
                "complex_guardrail_max_complexity_score",
                "complex_guardrail_observed_bundle_cost_ratio",
                "complex_guardrail_observed_total_cost_ratio",
                "threshold_phase_cluster_count",
                "post_threshold_cluster_count",
                "estimated_threshold_coverage_ratio",
                "threshold_first_ordering_enabled",
                "low_gain_pre_threshold_cluster_count",
                "far_pre_threshold_cluster_count",
                "threshold_phase_inter_component_jump_count",
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


def _uav_bbox(uavs: list[UAVState]) -> dict[str, int]:
    if not uavs:
        return {"min_x": 0, "min_y": 0, "max_x": 0, "max_y": 0, "width": 0, "height": 0}
    min_x = min(state.position.x for state in uavs)
    max_x = max(state.position.x for state in uavs)
    min_y = min(state.position.y for state in uavs)
    max_y = max(state.position.y for state in uavs)
    return {"min_x": min_x, "min_y": min_y, "max_x": max_x, "max_y": max_y, "width": max_x - min_x + 1, "height": max_y - min_y + 1}


def _centroid(points: list[Position]) -> Position:
    if not points:
        return Position(0, 0)
    return Position(round(sum(point.x for point in points) / len(points)), round(sum(point.y for point in points) / len(points)))


def _nearest_entry_side(cells: set[Position], uavs: list[UAVState]) -> str:
    min_x = min(cell.x for cell in cells)
    max_x = max(cell.x for cell in cells)
    min_y = min(cell.y for cell in cells)
    max_y = max(cell.y for cell in cells)
    center = _centroid([state.position for state in uavs])
    distances = {
        "west": abs(center.x - min_x),
        "east": abs(max_x - center.x),
        "north": abs(center.y - min_y),
        "south": abs(max_y - center.y),
    }
    return min(distances, key=lambda side: (distances[side], {"west": 0, "north": 1, "east": 2, "south": 3}[side]))


def _build_clustered_sectors(cells: set[Position], count: int, orientation: str) -> list[set[Position]]:
    if count <= 1:
        return [set(cells)]
    axis_values = sorted({cell.y if orientation == "horizontal" else cell.x for cell in cells})
    weighted: list[tuple[int, int]] = [
        (value, sum(1 for cell in cells if (cell.y if orientation == "horizontal" else cell.x) == value))
        for value in axis_values
    ]
    total = sum(weight for _, weight in weighted)
    target = total / count
    sectors: list[set[Position]] = []
    current_values: set[int] = set()
    current_weight = 0
    remaining_buckets = count
    for value, weight in weighted:
        current_values.add(value)
        current_weight += weight
        remaining_values = len([item for item, _ in weighted if item > value])
        if remaining_buckets > 1 and current_weight >= target and remaining_values >= remaining_buckets - 1:
            sectors.append(_cells_for_axis_values(cells, current_values, orientation))
            current_values = set()
            current_weight = 0
            remaining_buckets -= 1
    if current_values:
        sectors.append(_cells_for_axis_values(cells, current_values, orientation))
    while len(sectors) < count:
        largest = max(range(len(sectors)), key=lambda idx: len(sectors[idx]))
        first, second = _split_sector(sectors.pop(largest), orientation)
        sectors.insert(largest, second)
        sectors.insert(largest, first)
    return sectors[:count]


def _cells_for_axis_values(cells: set[Position], values: set[int], orientation: str) -> set[Position]:
    if orientation == "horizontal":
        return {cell for cell in cells if cell.y in values}
    return {cell for cell in cells if cell.x in values}


def _split_sector(sector: set[Position], orientation: str) -> tuple[set[Position], set[Position]]:
    values = sorted({cell.y if orientation == "horizontal" else cell.x for cell in sector})
    midpoint = max(1, len(values) // 2)
    first_values = set(values[:midpoint])
    first = _cells_for_axis_values(sector, first_values, orientation)
    second = set(sector) - first
    return first, second or first


def _sector_entry_cost(origin: Position, sector: set[Position], entry_side: str) -> int:
    if not sector:
        return 0
    if entry_side == "west":
        edge = min(cell.x for cell in sector)
        candidates = [cell for cell in sector if cell.x == edge]
    elif entry_side == "east":
        edge = max(cell.x for cell in sector)
        candidates = [cell for cell in sector if cell.x == edge]
    elif entry_side == "north":
        edge = min(cell.y for cell in sector)
        candidates = [cell for cell in sector if cell.y == edge]
    else:
        edge = max(cell.y for cell in sector)
        candidates = [cell for cell in sector if cell.y == edge]
    return min(_manhattan(origin, cell) for cell in candidates)


def _orient_waypoints_from_entry(waypoints: list[Position], entry_side: str, launch_center: Position) -> list[Position]:
    if not waypoints:
        return waypoints
    first_key = _entry_sort_key(waypoints[0], entry_side, launch_center)
    last_key = _entry_sort_key(waypoints[-1], entry_side, launch_center)
    return list(reversed(waypoints)) if last_key < first_key else waypoints


def _entry_sort_key(point: Position, entry_side: str, launch_center: Position) -> tuple[int, int]:
    if entry_side == "west":
        return (point.x, abs(point.y - launch_center.y))
    if entry_side == "east":
        return (-point.x, abs(point.y - launch_center.y))
    if entry_side == "north":
        return (point.y, abs(point.x - launch_center.x))
    return (-point.y, abs(point.x - launch_center.x))


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
