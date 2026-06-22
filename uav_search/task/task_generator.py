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
    origins: list[Position] | None = None,
    created_at: float = 0.0,
) -> list[Task]:
    task_origins = origins or [home]
    searchable_cells = set(grid_map.get_searchable_cells())
    assigned_regions = (
        _partition_cells_by_origins(searchable_cells, task_origins, grid_map, sensor_radius_cells)
        if task_origins
        else []
    )
    task_ids = count(1)
    tasks: list[Task] = []

    if origins is None:
        assigned_regions = [
            (region, home)
            for region in partition_search_area_from_cells(searchable_cells, max(1, uav_count), grid_map)
        ]

    for region, origin in assigned_regions:
        task = _build_search_task(
            task_id=f"task_{next(task_ids):03d}",
            region=region,
            origin=origin,
            grid_map=grid_map,
            sensor_radius_cells=sensor_radius_cells,
            created_at=created_at,
        )
        if task is not None:
            tasks.append(task)
    return sorted(tasks, key=lambda task: (-task.priority, task.created_at, task.id))


def partition_search_area(grid_map: GridMap, region_count: int) -> list[set[Position]]:
    searchable = grid_map.get_searchable_cells()
    return partition_search_area_from_cells(set(searchable), region_count, grid_map)


def partition_search_area_from_cells(
    cells: set[Position],
    region_count: int,
    grid_map: GridMap,
) -> list[set[Position]]:
    searchable = list(cells)
    if not searchable:
        return []

    components = _connected_components(set(searchable), grid_map)
    regions: list[set[Position]] = []
    for component, split_count in zip(components, _component_split_counts(components, region_count)):
        regions.extend(_split_component_balanced(component, split_count, grid_map))
    return regions


def connected_components(cells: set[Position], grid_map: GridMap) -> list[set[Position]]:
    return _connected_components(cells, grid_map)


def generate_boustrophedon_path(region: set[Position], sensor_radius_cells: int) -> list[Position]:
    if not region:
        return []
    sample_step = max(1, sensor_radius_cells * 2)
    ys = sorted({cell.y for cell in region})
    selected_rows = _sample_axis_with_boundaries(ys, sample_step)

    waypoints: list[Position] = []
    reverse = False
    for y in selected_rows:
        row_xs = sorted(cell.x for cell in region if cell.y == y)
        row_points: list[Position] = []
        for segment in _contiguous_segments(row_xs):
            sampled_xs = _sample_axis_with_boundaries(segment, sample_step)
            row_points.extend(Position(x, y) for x in sampled_xs)
        if not row_points:
            continue
        row_points.sort(key=lambda cell: cell.x, reverse=reverse)
        waypoints.extend(row_points)
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
    return min(candidates, key=lambda candidate: _route_cost_cells(candidate, start=uav_position))


def estimate_task_cost(waypoints: list[Position], entry_point: Position, resolution_m: float) -> float:
    if not waypoints:
        return 0.0
    return (_manhattan(entry_point, waypoints[0]) + _route_cost_cells(waypoints)) * resolution_m


def compute_region_value(region: set[Position], grid_map: GridMap) -> tuple[float, float]:
    uncovered_value = float(len(region))
    priority_value = sum(max(0.0, grid_map.get_cell(cell).search_priority - 1.0) for cell in region)
    return uncovered_value, priority_value


def _build_search_task(
    task_id: str,
    region: set[Position],
    origin: Position,
    grid_map: GridMap,
    sensor_radius_cells: int,
    created_at: float,
) -> Task | None:
    if not region:
        return None
    waypoints = generate_boustrophedon_path(region, sensor_radius_cells)
    if not waypoints:
        return None
    waypoints = reorder_waypoints_for_uav(waypoints, origin)
    entry_point = waypoints[0]
    priority = max(grid_map.get_cell(cell).search_priority for cell in region)
    estimated_cost_m = estimate_task_cost(waypoints, entry_point, grid_map.resolution_m)
    uncovered_value, priority_value = compute_region_value(region, grid_map)
    return Task(
        id=task_id,
        type=TaskType.SEARCH,
        priority=priority,
        target_cells=set(region),
        entry_point=entry_point,
        waypoints=waypoints,
        coverage_waypoints=list(waypoints),
        estimated_cost_m=estimated_cost_m,
        created_at=created_at,
        updated_at=created_at,
        uncovered_value=uncovered_value,
        priority_value=priority_value,
        score=(uncovered_value + priority_value) / max(estimated_cost_m, 1.0),
    )


def _partition_cells_by_origins(
    cells: set[Position],
    origins: list[Position],
    grid_map: GridMap,
    sensor_radius_cells: int,
) -> list[tuple[set[Position], Position]]:
    if not cells:
        return []
    if _should_use_horizontal_bands(origins, grid_map):
        return _partition_cells_by_horizontal_bands(cells, origins, grid_map, sensor_radius_cells)

    regions: list[tuple[set[Position], Position]] = []
    for component in _connected_components(cells, grid_map):
        regions.extend(_grow_connected_voronoi_regions(component, origins, grid_map, sensor_radius_cells))
    return sorted(regions, key=lambda item: (item[1].y, item[1].x, min(item[0])))


def _should_use_horizontal_bands(origins: list[Position], grid_map: GridMap) -> bool:
    if len(origins) <= 1:
        return False
    x_span = max(origin.x for origin in origins) - min(origin.x for origin in origins)
    return x_span <= max(2, grid_map.width_cells // 10)


def _partition_cells_by_horizontal_bands(
    cells: set[Position],
    origins: list[Position],
    grid_map: GridMap,
    sensor_radius_cells: int,
) -> list[tuple[set[Position], Position]]:
    sorted_origins = sorted(origins, key=lambda origin: (origin.y, origin.x))
    buckets: dict[Position, set[Position]] = {origin: set() for origin in sorted_origins}
    rows = sorted({cell.y for cell in cells})
    row_weights = [
        (y, _estimated_region_route_cells({cell for cell in cells if cell.y == y}, sensor_radius_cells))
        for y in rows
    ]
    total_weight = sum(weight for _, weight in row_weights)
    target_weight = total_weight / max(1, len(sorted_origins))
    origin_index = 0
    current_weight = 0.0
    for y, weight in row_weights:
        if (
            origin_index < len(sorted_origins) - 1
            and current_weight > 0
            and current_weight + (weight / 2.0) > target_weight
        ):
            origin_index += 1
            current_weight = 0.0
        owner = sorted_origins[origin_index]
        buckets[owner].update(cell for cell in cells if cell.y == y)
        current_weight += weight

    regions: list[tuple[set[Position], Position]] = []
    for origin in sorted_origins:
        for component in _connected_components(buckets[origin], grid_map):
            regions.append((component, origin))
    return regions


def _nearest_origin_to_region(region: set[Position], origins: list[Position]) -> Position:
    return min(
        origins,
        key=lambda origin: (
            min(_manhattan(cell, origin) for cell in region),
            origin.y,
            origin.x,
        ),
    )


def _grow_connected_voronoi_regions(
    component: set[Position],
    origins: list[Position],
    grid_map: GridMap,
    sensor_radius_cells: int,
) -> list[tuple[set[Position], Position]]:
    if not component:
        return []

    seed_pairs: list[tuple[Position, Position]] = []
    used_seeds: set[Position] = set()
    for origin in origins:
        seed = nearest_cell(component, origin)
        if seed in used_seeds:
            continue
        used_seeds.add(seed)
        seed_pairs.append((seed, origin))

    if not seed_pairs:
        return []
    if len(seed_pairs) == 1:
        return [(set(component), seed_pairs[0][1])]

    regions: list[set[Position]] = [{seed} for seed, _ in seed_pairs]
    queues: list[deque[Position]] = [deque([seed]) for seed, _ in seed_pairs]
    remaining = set(component) - {seed for seed, _ in seed_pairs}
    soft_target = max(1.0, _estimated_region_route_cells(component, sensor_radius_cells) / len(seed_pairs))

    while remaining and any(queues):
        progressed = False
        for index, queue in enumerate(queues):
            if not queue:
                continue
            region_load = _estimated_region_route_cells(regions[index], sensor_radius_cells)
            if region_load >= soft_target and any(
                _estimated_region_route_cells(region, sensor_radius_cells) < soft_target
                for region in regions
            ):
                continue
            current = queue.popleft()
            neighbors = sorted(
                (neighbor for neighbor in grid_map.get_neighbors(current, mode=4) if neighbor in remaining),
                key=lambda cell: (
                    _manhattan(cell, seed_pairs[index][1]),
                    cell.y,
                    cell.x,
                ),
            )
            for neighbor in neighbors:
                remaining.remove(neighbor)
                regions[index].add(neighbor)
                queue.append(neighbor)
                progressed = True
                break
        if not progressed:
            soft_target = _estimated_region_route_cells(component, sensor_radius_cells)

    while remaining:
        cell = min(remaining)
        owner = min(
            range(len(regions)),
            key=lambda index: (
                min(_manhattan(cell, existing) for existing in regions[index]),
                len(regions[index]),
                index,
            ),
        )
        remaining.remove(cell)
        regions[owner].add(cell)

    return [
        (region, seed_pairs[index][1])
        for index, region in enumerate(regions)
        if region
    ]


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


def _sample_axis_with_boundaries(values: list[int], step: int) -> list[int]:
    if not values:
        return []
    step = max(1, step)
    sampled = values[::step]
    if values[-1] not in sampled:
        sampled.append(values[-1])
    return sampled


def _contiguous_segments(values: list[int]) -> list[list[int]]:
    if not values:
        return []
    segments: list[list[int]] = []
    current = [values[0]]
    for value in values[1:]:
        if value == current[-1] + 1:
            current.append(value)
            continue
        segments.append(current)
        current = [value]
    segments.append(current)
    return segments


def _estimated_region_route_cells(region: set[Position], sensor_radius_cells: int) -> float:
    waypoints = generate_boustrophedon_path(region, sensor_radius_cells)
    if not waypoints:
        return 0.0
    return float(_route_cost_cells(waypoints) + len(waypoints))


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
