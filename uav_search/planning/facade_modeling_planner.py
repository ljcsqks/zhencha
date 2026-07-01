from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from uav_search.core.data_types import CellType, Position, UAVState
from uav_search.maps.grid_map import GridMap
from uav_search.planning.astar import AStarConfig, astar_search, path_cost


@dataclass(frozen=True)
class BuildingFootprint:
    building_id: str
    vertices: list[Position]


@dataclass(frozen=True)
class FacadeLane:
    lane_id: str
    side: str
    waypoints: list[Position]
    length_m: float


@dataclass
class ModelingPlan:
    uav_id: str
    building_id: str
    route: list[Position]
    facade_lane_ids: list[str]
    logical_waypoints: list[Position]
    estimated_distance_m: float
    metadata: dict[str, Any] = field(default_factory=dict)


class FacadeModelingPlanner:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or {}
        self.astar_config = AStarConfig(
            obstacle_proximity_penalty=float(self.config.get("obstacle_proximity_penalty", 0.5)),
            priority_area_bonus=0.0,
        )
        self.last_diagnostics: dict[str, Any] = self._empty_diagnostics()

    def footprint_cells(self, footprint: BuildingFootprint) -> set[Position]:
        min_x, max_x, min_y, max_y = self._bounds(footprint)
        return {Position(x, y) for x in range(min_x, max_x + 1) for y in range(min_y, max_y + 1)}

    def generate_facade_lanes(
        self,
        footprint: BuildingFootprint,
        grid_map: GridMap,
        standoff_cells: int | None = None,
    ) -> list[FacadeLane]:
        standoff = max(1, int(standoff_cells or self.config.get("default_standoff_cells", 3)))
        step = max(1, int(self.config.get("sample_step_cells", 2)))
        min_x, max_x, min_y, max_y = self._bounds(footprint)
        raw_lanes = [
            ("north", [Position(x, min_y - standoff) for x in range(min_x, max_x + 1)]),
            ("east", [Position(max_x + standoff, y) for y in range(min_y, max_y + 1)]),
            ("south", [Position(x, max_y + standoff) for x in range(max_x, min_x - 1, -1)]),
            ("west", [Position(min_x - standoff, y) for y in range(max_y, min_y - 1, -1)]),
        ]

        lanes: list[FacadeLane] = []
        for side, cells in raw_lanes:
            sampled = self._sample_lane([cell for cell in cells if grid_map.in_bounds(cell) and grid_map.is_passable(cell)], step)
            if not sampled:
                continue
            lanes.append(
                FacadeLane(
                    lane_id=f"{footprint.building_id}_{side}",
                    side=side,
                    waypoints=sampled,
                    length_m=path_cost(sampled) * grid_map.resolution_m,
                )
            )
        return lanes

    def plan_modeling(
        self,
        footprint: BuildingFootprint,
        grid_map: GridMap,
        uav_states: list[UAVState],
        uav_count: int,
        standoff_cells: int | None = None,
        laps: int = 1,
        created_at: float = 0.0,
        resume_search_after: bool = True,
    ) -> list[ModelingPlan]:
        self.last_diagnostics = self._empty_diagnostics()
        lanes = self.generate_facade_lanes(footprint, grid_map, standoff_cells)
        self.last_diagnostics["modeling_facade_lane_count"] = len(lanes)
        if len(lanes) < 4:
            self.last_diagnostics["modeling_unreachable_facade_lanes"] = 4 - len(lanes)
            return []

        selected = list(uav_states)[: max(1, min(int(uav_count), len(uav_states), 4))]
        if not selected:
            return []

        blocked = self._block_footprint(grid_map, self.footprint_cells(footprint))
        try:
            plans: list[ModelingPlan] = []
            chunks = self._split_lanes(lanes, len(selected))
            for uav, assigned_lanes in zip(selected, chunks):
                logical_waypoints = [point for _ in range(max(1, int(laps))) for lane in assigned_lanes for point in lane.waypoints]
                route = self._route_through(grid_map, uav.position, logical_waypoints)
                if not route:
                    self.last_diagnostics["modeling_unreachable_facade_lanes"] += len(assigned_lanes)
                    return []
                plans.append(
                    ModelingPlan(
                        uav_id=uav.id,
                        building_id=footprint.building_id,
                        route=route,
                        facade_lane_ids=[lane.lane_id for lane in assigned_lanes],
                        logical_waypoints=logical_waypoints,
                        estimated_distance_m=path_cost(route) * grid_map.resolution_m,
                        metadata={
                            "building_id": footprint.building_id,
                            "facade_lane_ids": [lane.lane_id for lane in assigned_lanes],
                            "logical_waypoints": self._positions_to_dicts(logical_waypoints),
                            "standoff_cells": int(standoff_cells or self.config.get("default_standoff_cells", 3)),
                            "laps": max(1, int(laps)),
                            "resume_search_after": bool(resume_search_after),
                            "created_at": created_at,
                        },
                    )
                )
        finally:
            self._restore_blocked(grid_map, blocked)

        self.last_diagnostics["modeling_assigned_uav_count"] = len(plans)
        self.last_diagnostics["modeling_distance_m"] = sum(plan.estimated_distance_m for plan in plans)
        return plans

    def _bounds(self, footprint: BuildingFootprint) -> tuple[int, int, int, int]:
        if len(footprint.vertices) < 4:
            raise ValueError("building footprint must contain at least four vertices")
        xs = [point.x for point in footprint.vertices]
        ys = [point.y for point in footprint.vertices]
        return min(xs), max(xs), min(ys), max(ys)

    def _sample_lane(self, cells: list[Position], step: int) -> list[Position]:
        if not cells:
            return []
        sampled = cells[::step]
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])
        return sampled

    def _split_lanes(self, lanes: list[FacadeLane], count: int) -> list[list[FacadeLane]]:
        chunks: list[list[FacadeLane]] = []
        lane_count = len(lanes)
        for idx in range(count):
            start = round(idx * lane_count / count)
            end = round((idx + 1) * lane_count / count)
            chunks.append(lanes[start:end])
        return [chunk for chunk in chunks if chunk]

    def _route_through(self, grid_map: GridMap, start: Position, waypoints: list[Position]) -> list[Position]:
        if not waypoints:
            return []
        route = [start]
        current = start
        for waypoint in waypoints:
            segment = astar_search(grid_map, current, waypoint, self.astar_config)
            if not segment:
                return []
            route.extend(segment[1:])
            current = waypoint
        return route

    def _block_footprint(self, grid_map: GridMap, cells: set[Position]) -> dict[Position, tuple[str, bool]]:
        previous: dict[Position, tuple[str, bool]] = {}
        for cell in cells:
            if not grid_map.in_bounds(cell):
                continue
            existing = grid_map.get_cell(cell)
            previous[cell] = (existing.cell_type.value, existing.passable)
            grid_map.set_cell(cell, {"cell_type": CellType.NO_FLY})
        return previous

    def _restore_blocked(self, grid_map: GridMap, previous: dict[Position, tuple[str, bool]]) -> None:
        for cell, (cell_type, passable) in previous.items():
            grid_map.set_cell(cell, {"cell_type": cell_type, "passable": passable})

    def _positions_to_dicts(self, positions: list[Position]) -> list[dict[str, int]]:
        return [{"x": point.x, "y": point.y} for point in positions]

    def _empty_diagnostics(self) -> dict[str, Any]:
        return {
            "modeling_facade_lane_count": 0,
            "modeling_assigned_uav_count": 0,
            "modeling_unreachable_facade_lanes": 0,
            "modeling_distance_m": 0.0,
            "modeling_no_fly_violations": 0,
        }
