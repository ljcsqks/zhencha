from __future__ import annotations

import time

from uav_search.core.data_types import PathPlan, Position, UAVState
from uav_search.maps.grid_map import GridMap
from uav_search.planning.astar import AStarConfig, astar_search, path_cost


class PathPlanner:
    def __init__(self, config: dict | None = None) -> None:
        config = config or {}
        self.astar_config = AStarConfig(
            obstacle_proximity_penalty=float(config.get("obstacle_proximity_penalty", 0.5)),
            priority_area_bonus=float(config.get("priority_area_bonus", -0.2)),
        )

    def plan_path(
        self,
        uav: UAVState,
        target: Position,
        grid_map: GridMap,
        task_id: str | None = None,
        now: float = 0.0,
    ) -> PathPlan:
        started = time.perf_counter()
        path = astar_search(grid_map, uav.position, target, self.astar_config)
        latency_ms = (time.perf_counter() - started) * 1000.0
        if path is None:
            return PathPlan(
                uav_id=uav.id,
                task_id=task_id,
                start=uav.position,
                goal=target,
                path=[],
                cost=float("inf"),
                valid=False,
                reason="path_not_found",
                planned_at=now,
                latency_ms=latency_ms,
            )

        return PathPlan(
            uav_id=uav.id,
            task_id=task_id,
            start=uav.position,
            goal=target,
            path=path,
            cost=path_cost(path),
            valid=True,
            planned_at=now,
            latency_ms=latency_ms,
        )

    def is_path_valid(self, path: list[Position], grid_map: GridMap) -> bool:
        return bool(path) and all(grid_map.is_passable(pos) for pos in path)
