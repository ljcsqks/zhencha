from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from itertools import count

from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap


@dataclass(frozen=True)
class AStarConfig:
    obstacle_proximity_penalty: float = 0.5
    priority_area_bonus: float = -0.2


def astar_search(
    grid_map: GridMap,
    start: Position,
    goal: Position,
    config: AStarConfig | None = None,
) -> list[Position] | None:
    if not grid_map.is_passable(start) or not grid_map.is_passable(goal):
        return None

    config = config or AStarConfig()
    open_heap: list[tuple[float, int, Position]] = []
    sequence = count()
    heapq.heappush(open_heap, (0.0, next(sequence), start))

    came_from: dict[Position, Position] = {}
    g_score: dict[Position, float] = {start: 0.0}
    closed: set[Position] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        # Neighbors come from GridMap so obstacle, no-fly, and diagonal corner
        # rules stay centralized in the map layer.
        for neighbor in grid_map.get_neighbors(current, mode=8):
            tentative_g = g_score[current] + _move_cost(current, neighbor) + _environment_cost(
                grid_map, neighbor, config
            )
            if tentative_g >= g_score.get(neighbor, math.inf):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + _diagonal_distance(neighbor, goal)
            heapq.heappush(open_heap, (f_score, next(sequence), neighbor))

    return None


def path_cost(path: list[Position]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(_move_cost(path[idx - 1], path[idx]) for idx in range(1, len(path)))


def _move_cost(a: Position, b: Position) -> float:
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return 1.41421356237 if dx == 1 and dy == 1 else 1.0


def _diagonal_distance(a: Position, b: Position) -> float:
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (1.41421356237 - 2.0) * min(dx, dy)


def _environment_cost(grid_map: GridMap, pos: Position, config: AStarConfig) -> float:
    cost = 0.0
    if _near_blocked_cell(grid_map, pos):
        cost += config.obstacle_proximity_penalty

    cell = grid_map.get_cell(pos)
    if cell.cell_type == CellType.PRIORITY and cell.search_confidence < 0.95:
        cost += config.priority_area_bonus
    return max(cost, -0.9)


def _near_blocked_cell(grid_map: GridMap, pos: Position) -> bool:
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            candidate = Position(pos.x + dx, pos.y + dy)
            if grid_map.in_bounds(candidate) and not grid_map.is_passable(candidate):
                return True
    return False


def _reconstruct_path(came_from: dict[Position, Position], current: Position) -> list[Position]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
