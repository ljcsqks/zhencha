from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap
from uav_search.planning.astar import astar_search


def test_astar_finds_path_on_open_grid() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)

    path = astar_search(grid_map, Position(0, 0), Position(5, 5))

    assert path is not None
    assert path[0] == Position(0, 0)
    assert path[-1] == Position(5, 5)


def test_astar_routes_around_obstacle_wall_with_gap() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    for y in range(10):
        if y != 5:
            grid_map.set_cell(Position(3, y), {"cell_type": CellType.OBSTACLE})

    path = astar_search(grid_map, Position(0, 5), Position(8, 5))

    assert path is not None
    assert Position(3, 5) in path
    assert all(grid_map.is_passable(pos) for pos in path)


def test_astar_returns_none_when_goal_blocked() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    grid_map.set_cell(Position(5, 5), {"cell_type": CellType.NO_FLY})

    path = astar_search(grid_map, Position(0, 0), Position(5, 5))

    assert path is None
