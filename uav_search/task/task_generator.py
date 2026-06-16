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
        estimated_cost_m = estimate_task_cost(waypoints, entry_point, grid_map.resolution_m)
        uncovered_value, priority_value = compute_region_value(region, grid_map)
        task_id = f"task_{next(task_ids):03d}"
        tasks.append(
            Task(
                id=task_id,
                type=TaskType.SEARCH,
                priority=priority,
                target_cells=set(region),
                entry_point=entry_point,
                waypoints=waypoints,
                estimated_cost_m=estimated_cost_m,
                created_at=created_at,
                updated_at=created_at,
                uncovered_value=uncovered_value,
                priority_value=priority_value,
                score=(uncovered_value + priority_value) / max(estimated_cost_m, 1.0),
            )
        )
    return tasks


def partition_search_area(grid_map: GridMap, region_count: int) -> list[set[Position]]:
    searchable = grid_map.get_searchable_cells()
    if not searchable:
        return []

    components = _connected_components(set(searchable), grid_map)
    regions: list[set[Position]] = []
    for component, split_count in zip(components, _component_split_counts(components, region_count)):
        regions.extend(_split_component_balanced(component, split_count, grid_map))
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


def reorder_waypoints_for_uav(waypoints: list[Position], uav_position: Position) -> list[Position]:
    if len(waypoints) <= 1:
        return list(waypoints)

    candidates = [
        list(waypoints),
        list(reversed(waypoints)),
    ]
    nearest_index = min(range(len(waypoints)), key=lambda index: _manhattan(waypoints[index], uav_position))
    candidates.append([*waypoints[nearest_index:], *waypoints[:nearest_index]])
    reversed_waypoints = list(reversed(waypoints))
    reversed_nearest_index = min(
        range(len(reversed_waypoints)),
        key=lambda index: _manhattan(reversed_waypoints[index], uav_position),
    )
    candidates.append([*reversed_waypoints[reversed_nearest_index:], *reversed_waypoints[:reversed_nearest_index]])

    return min(candidates, key=lambda candidate: _route_cost_cells(candidate, start=uav_position))


def estimate_task_cost(waypoints: list[Position], entry_point: Position, resolution_m: float) -> float:
    if not waypoints:
        return 0.0
    return (_manhattan(entry_point, waypoints[0]) + _route_cost_cells(waypoints)) * resolution_m


def compute_region_value(region: set[Position], grid_map: GridMap) -> tuple[float, float]:
    uncovered_value = float(len(region))
    priority_value = sum(max(0.0, grid_map.get_cell(cell).search_priority - 1.0) for cell in region)
    return uncovered_value, priority_value


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
    return sorted(components, key=lambda component: (-len(component), min(component)))


def _component_split_counts(components: list[set[Position]], desired_count: int) -> list[int]:
    if not components:
        return []
    if desired_count <= len(components):
        return [1 for _ in components]

    counts = [1 for _ in components]
    remaining = desired_count - len(components)
    total_size = sum(len(component) for component in components)
    fractional: list[tuple[float, int]] = []
    for index, component in enumerate(components):
        raw_extra = remaining * (len(component) / total_size)
        extra = int(raw_extra)
        counts[index] += extra
        fractional.append((raw_extra - extra, index))

    assigned_extra = sum(counts) - len(components)
    for _, index in sorted(fractional, reverse=True):
        if assigned_extra >= remaining:
            break
        counts[index] += 1
        assigned_extra += 1
    return counts


def _split_component_balanced(component: set[Position], split_count: int, grid_map: GridMap) -> list[set[Position]]:
    if split_count <= 1 or len(component) <= 1:
        return [component]

    seeds = _choose_seeds(component, min(split_count, len(component)))
    regions = [{seed} for seed in seeds]
    remaining = set(component) - set(seeds)
    queues = [deque([seed]) for seed in seeds]
    target_size = max(1, len(component) // len(seeds))

    # Multi-source BFS keeps each grown region connected; the soft target size
    # prevents early seeds from swallowing most of a large component.
    while remaining and any(queues):
        progressed = False
        for index, queue in enumerate(queues):
            if not queue:
                continue
            if len(regions[index]) >= target_size and any(len(region) < target_size for region in regions):
                continue
            current = queue.popleft()
            for neighbor in grid_map.get_neighbors(current, mode=4):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                regions[index].add(neighbor)
                queue.append(neighbor)
                progressed = True
                break
        if not progressed:
            if any(queues):
                target_size = len(component)
                continue
            break

    while remaining:
        cell = min(remaining)
        owner = _nearest_region_index(cell, regions)
        remaining.remove(cell)
        regions[owner].add(cell)

    return [region for region in regions if region]


def _choose_seeds(component: set[Position], seed_count: int) -> list[Position]:
    seeds = [min(component)]
    while len(seeds) < seed_count:
        candidates = sorted(cell for cell in component if cell not in seeds)
        next_seed = max(candidates, key=lambda cell: min(_manhattan(cell, seed) for seed in seeds))
        seeds.append(next_seed)
    return seeds


def _nearest_region_index(cell: Position, regions: list[set[Position]]) -> int:
    return min(
        range(len(regions)),
        key=lambda index: (
            min(_manhattan(cell, existing) for existing in regions[index]),
            len(regions[index]),
            index,
        ),
    )


def _manhattan(a: Position, b: Position) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


def _route_cost_cells(waypoints: list[Position], start: Position | None = None) -> int:
    if not waypoints:
        return 0
    cost = _manhattan(start, waypoints[0]) if start is not None else 0
    cost += sum(_manhattan(a, b) for a, b in zip(waypoints, waypoints[1:]))
    return cost
