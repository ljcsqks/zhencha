from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap
from uav_search.maps.map_updater import MapUpdater


def test_map_updater_sets_cell_obstacle() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    updater = MapUpdater(grid_map)

    affected = updater.apply_update(
        {"operation": "SET_CELL", "position": {"x": 2, "y": 3}, "cell_type": "OBSTACLE"}
    )

    assert affected == [Position(2, 3)]
    assert grid_map.get_cell(Position(2, 3)).cell_type == CellType.OBSTACLE
    assert not grid_map.is_passable(Position(2, 3))


def test_map_updater_sets_rectangle_region() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    updater = MapUpdater(grid_map)

    affected = updater.apply_update(
        {"operation": "SET_REGION", "shape": "rectangle", "x": 1, "y": 1, "width": 2, "height": 2, "cell_type": "NO_FLY"}
    )

    assert len(affected) == 4
    assert not grid_map.is_passable(Position(1, 1))
    assert not grid_map.is_passable(Position(2, 2))
