from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap


def test_world_to_grid_uses_resolution() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)

    assert grid_map.world_to_grid(25.0, 35.0) == Position(2, 3)


def test_obstacle_cell_is_not_passable() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    pos = Position(3, 4)

    grid_map.set_cell(pos, {"cell_type": CellType.OBSTACLE})

    assert not grid_map.is_passable(pos)
    assert grid_map.get_cell(pos).cell_type == CellType.OBSTACLE


def test_mark_covered_updates_confidence_and_count() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)

    covered = grid_map.mark_covered(Position(5, 5), radius_cells=1, timestamp=10.0)

    assert Position(5, 5) in covered
    assert grid_map.get_cell(Position(5, 5)).search_confidence == 1.0
    assert grid_map.coverage_count[5, 5] == 1
