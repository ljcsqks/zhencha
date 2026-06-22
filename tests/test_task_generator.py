from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap
from uav_search.task.task_generator import (
    estimate_task_cost,
    generate_boustrophedon_path,
    generate_initial_tasks,
    partition_search_area,
    reorder_waypoints_for_uav,
)


def test_partition_search_area_returns_connected_regions() -> None:
    grid_map = GridMap(width_m=80, height_m=40, resolution_m=10)

    regions = partition_search_area(grid_map, region_count=2)

    assert len(regions) == 2
    assert sum(len(region) for region in regions) == len(grid_map.get_searchable_cells())
    assert all(_is_connected(region, grid_map) for region in regions)


def test_partition_splits_disconnected_stripe_components() -> None:
    grid_map = GridMap(width_m=60, height_m=40, resolution_m=10)
    for y in range(4):
        grid_map.set_cell(Position(1, y), {"cell_type": CellType.OBSTACLE})

    regions = partition_search_area(grid_map, region_count=1)

    assert len(regions) == 2
    assert all(_is_connected(region, grid_map) for region in regions)


def test_partition_search_area_balances_large_connected_component() -> None:
    grid_map = GridMap(width_m=120, height_m=120, resolution_m=10)

    regions = partition_search_area(grid_map, region_count=4)
    sizes = [len(region) for region in regions]

    assert len(regions) == 4
    assert max(sizes) - min(sizes) <= 12
    assert all(_is_connected(region, grid_map) for region in regions)


def test_boustrophedon_path_alternates_rows() -> None:
    region = {Position(x, y) for y in range(4) for x in range(4)}

    path = generate_boustrophedon_path(region, sensor_radius_cells=1)

    assert path[:3] == [Position(0, 0), Position(2, 0), Position(3, 0)]
    assert path[3:6] == [Position(3, 2), Position(2, 2), Position(0, 2)]


def test_boustrophedon_path_samples_rows_and_columns_by_sensor_radius() -> None:
    region = {Position(x, y) for y in range(5) for x in range(5)}

    path = generate_boustrophedon_path(region, sensor_radius_cells=2)

    assert path == [Position(0, 0), Position(4, 0), Position(4, 4), Position(0, 4)]


def test_generate_initial_tasks_creates_search_tasks() -> None:
    grid_map = GridMap(width_m=80, height_m=40, resolution_m=10)

    tasks = generate_initial_tasks(grid_map, uav_count=2, sensor_radius_cells=1, home=Position(0, 0))

    assert len(tasks) == 2
    assert all(task.waypoints for task in tasks)
    assert all(task.entry_point in task.waypoints for task in tasks)
    assert all(task.estimated_cost_m > 0 for task in tasks)
    assert all(task.uncovered_value == len(task.target_cells) for task in tasks)
    assert all(task.score > 0 for task in tasks)


def test_generate_initial_tasks_splits_priority_region_first() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    priority_cells = {Position(x, y) for y in range(7, 9) for x in range(7, 9)}
    for cell in priority_cells:
        grid_map.set_cell(cell, {"cell_type": CellType.PRIORITY, "search_priority": 3.0})

    tasks = generate_initial_tasks(
        grid_map,
        uav_count=2,
        sensor_radius_cells=1,
        home=Position(0, 0),
        origins=[Position(0, 0), Position(0, 9)],
    )

    assert tasks[0].priority == 3.0
    assert priority_cells.issubset(tasks[0].target_cells)
    assert all(cell not in task.target_cells for task in tasks[1:] for cell in priority_cells)


def test_generate_initial_tasks_uses_uav_origins_for_ordinary_regions() -> None:
    grid_map = GridMap(width_m=120, height_m=120, resolution_m=10)

    tasks = generate_initial_tasks(
        grid_map,
        uav_count=2,
        sensor_radius_cells=1,
        home=Position(0, 0),
        origins=[Position(0, 0), Position(0, 11)],
    )
    low_region, high_region = tasks

    assert max(cell.y for cell in low_region.target_cells) < min(cell.y for cell in high_region.target_cells)


def test_generate_initial_tasks_scales_horizontal_bands_for_tall_maps() -> None:
    grid_map = GridMap(width_m=500, height_m=500, resolution_m=10)
    origins = [Position(0, 0), Position(0, 8), Position(0, 16), Position(0, 23)]

    tasks = generate_initial_tasks(
        grid_map,
        uav_count=4,
        sensor_radius_cells=2,
        home=Position(0, 0),
        origins=origins,
    )
    y_ranges = sorted((min(cell.y for cell in task.target_cells), max(cell.y for cell in task.target_cells)) for task in tasks)

    assert len(tasks) == 4
    assert y_ranges[0][0] == 0
    assert y_ranges[-1][1] == 49
    assert all(previous[1] < current[0] for previous, current in zip(y_ranges, y_ranges[1:]))


def test_reorder_waypoints_starts_near_current_uav() -> None:
    waypoints = [Position(0, 0), Position(1, 0), Position(9, 0), Position(9, 1)]

    reordered = reorder_waypoints_for_uav(waypoints, Position(9, 1))

    assert reordered[0] == Position(9, 1)
    assert set(reordered) == set(waypoints)


def test_estimate_task_cost_includes_entry_to_route_distance() -> None:
    waypoints = [Position(1, 0), Position(3, 0)]

    cost = estimate_task_cost(waypoints, entry_point=Position(0, 0), resolution_m=10)

    assert cost == 30.0


def _is_connected(region: set[Position], grid_map: GridMap) -> bool:
    if not region:
        return True
    visited = set()
    stack = [next(iter(region))]
    while stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        for neighbor in grid_map.get_neighbors(current, mode=4):
            if neighbor in region and neighbor not in visited:
                stack.append(neighbor)
    return visited == region
