from __future__ import annotations

from collections import deque
from itertools import count

from uav_search.core.data_types import Position, Task, TaskType
from uav_search.maps.grid_map import GridMap


def generate_initial_tasks(
    grid_map: GridMap,
    uav_count: int,
    sensor_radius_cells: int,
    home: Position,
    created_at: float = 0.0,
) -> list[Task]:
    regions = partition_search_area(grid_map, max(1, uav_count))
    task_ids = count(1)
    tasks: list[Task] = []
    for region in regions:
        if not region:
            continue
        waypoints = generate_boustrophedon_path(region, sensor_radius_cells)
        if not waypoints:
            continue
        entry_point = nearest_cell(waypoints, home)
        priority = max(grid_map.get_cell(cell).search_priority for cell in region)
        task_id = f"task_{next(task_ids):03d}"
        tasks.append(
            Task(
                id=task_id,
                type=TaskType.SEARCH,
                priority=priority,
                target_cells=set(region),
                entry_point=entry_point,
                waypoints=waypoints,
                created_at=created_at,
                updated_at=created_at,
            )
        )
    return tasks


def partition_search_area(grid_map: GridMap, region_count: int) -> list[set[Position]]:
    searchable = grid_map.get_searchable_cells()
    if not searchable:
        return []

    stripe_width = max(1, grid_map.width_cells // region_count)
    stripes: list[set[Position]] = [set() for _ in range(region_count)]
    for cell in searchable:
        stripe_index = min(cell.x // stripe_width, region_count - 1)
        stripes[stripe_index].add(cell)

    regions: list[set[Position]] = []
    for stripe in stripes:
        regions.extend(_connected_components(stripe, grid_map))
    return regions


def generate_boustrophedon_path(region: set[Position], sensor_radius_cells: int) -> list[Position]:
    if not region:
        return []
    row_step = max(1, sensor_radius_cells * 2)
    ys = sorted({cell.y for cell in region})
    selected_rows = ys[::row_step]
    if ys[-1] not in selected_rows:
        selected_rows.append(ys[-1])

    waypoints: list[Position] = []
    reverse = False
    for y in selected_rows:
        row_cells = sorted((cell for cell in region if cell.y == y), key=lambda cell: cell.x, reverse=reverse)
        if not row_cells:
            continue
        waypoints.extend(row_cells)
        reverse = not reverse
    return waypoints


def nearest_cell(cells: list[Position] | set[Position], origin: Position) -> Position:
    if not cells:
        raise ValueError("cells must not be empty")
    return min(cells, key=lambda cell: abs(cell.x - origin.x) + abs(cell.y - origin.y))


def _connected_components(cells: set[Position], grid_map: GridMap) -> list[set[Position]]:
    remaining = set(cells)
    components: list[set[Position]] = []
    while remaining:
        start = remaining.pop()
        component = {start}
        queue: deque[Position] = deque([start])
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
