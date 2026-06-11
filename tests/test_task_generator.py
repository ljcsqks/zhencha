from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap
from uav_search.task.task_generator import generate_boustrophedon_path, generate_initial_tasks, partition_search_area


def test_partition_search_area_returns_connected_regions() -> None:
    grid_map = GridMap(width_m=80, height_m=40, resolution_m=10)

    regions = partition_search_area(grid_map, region_count=2)

    assert len(regions) == 2
    assert sum(len(region) for region in regions) == len(grid_map.get_searchable_cells())


def test_partition_splits_disconnected_stripe_components() -> None:
    grid_map = GridMap(width_m=60, height_m=40, resolution_m=10)
    for y in range(4):
        grid_map.set_cell(Position(1, y), {"cell_type": CellType.OBSTACLE})

    regions = partition_search_area(grid_map, region_count=1)

    assert len(regions) == 2


def test_boustrophedon_path_alternates_rows() -> None:
    region = {Position(x, y) for y in range(4) for x in range(4)}

    path = generate_boustrophedon_path(region, sensor_radius_cells=1)

    assert path[:4] == [Position(0, 0), Position(1, 0), Position(2, 0), Position(3, 0)]
    assert path[4:8] == [Position(3, 2), Position(2, 2), Position(1, 2), Position(0, 2)]


def test_generate_initial_tasks_creates_search_tasks() -> None:
    grid_map = GridMap(width_m=80, height_m=40, resolution_m=10)

    tasks = generate_initial_tasks(grid_map, uav_count=2, sensor_radius_cells=1, home=Position(0, 0))

    assert len(tasks) == 2
    assert all(task.waypoints for task in tasks)
    assert all(task.entry_point in task.waypoints for task in tasks)
