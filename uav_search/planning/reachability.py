from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap


@dataclass(frozen=True)
class ReachabilityIndex:
    reachable_by_uav: dict[str, set[Position]]
    unreachable_searchable_cells: set[Position]

    def reachable_uavs(self, cell: Position) -> list[str]:
        return sorted(
            uav_id
            for uav_id, cells in self.reachable_by_uav.items()
            if cell in cells
        )

    def is_reachable(self, uav_id: str, cell: Position) -> bool:
        return cell in self.reachable_by_uav.get(uav_id, set())

    def any_reachable(self, cell: Position) -> bool:
        return any(cell in cells for cells in self.reachable_by_uav.values())

    def reachable_cells_for(self, uav_id: str) -> set[Position]:
        return set(self.reachable_by_uav.get(uav_id, set()))


def build_reachability_index(
    grid_map: GridMap,
    uav_states: Iterable[UAVState],
) -> ReachabilityIndex:
    reachable_by_uav: dict[str, set[Position]] = {}
    for uav in uav_states:
        if uav.status == UAVStatus.OFFLINE:
            continue
        start = uav.position if grid_map.is_passable(uav.position) else uav.home_position
        if not grid_map.is_passable(start):
            reachable_by_uav[uav.id] = set()
            continue
        reachable_by_uav[uav.id] = _flood_fill(grid_map, start)

    searchable = set(grid_map.get_searchable_cells())
    reachable_union: set[Position] = set()
    for cells in reachable_by_uav.values():
        reachable_union.update(cells)
    return ReachabilityIndex(
        reachable_by_uav=reachable_by_uav,
        unreachable_searchable_cells=searchable - reachable_union,
    )


def connected_components(grid_map: GridMap, cells: Iterable[Position]) -> list[set[Position]]:
    remaining = set(cells)
    components: list[set[Position]] = []
    while remaining:
        seed = remaining.pop()
        component = {seed}
        queue: deque[Position] = deque([seed])
        while queue:
            current = queue.popleft()
            for neighbor in grid_map.get_neighbors(current, mode=4):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)
        components.append(component)
    return components


def _flood_fill(grid_map: GridMap, start: Position) -> set[Position]:
    visited = {start}
    queue: deque[Position] = deque([start])
    while queue:
        current = queue.popleft()
        for neighbor in grid_map.get_neighbors(current, mode=4):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append(neighbor)
    return visited
