from __future__ import annotations

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.planning.reachability import build_reachability_index


def _uav(uav_id: str, position: Position, status: UAVStatus = UAVStatus.IDLE) -> UAVState:
    return UAVState(
        id=uav_id,
        position=position,
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=status,
        home_position=position,
    )


def test_reachability_index_identifies_unreachable_cells_behind_obstacle_wall() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for y in range(5):
        grid_map.set_cell(Position(2, y), {"cell_type": CellType.OBSTACLE})

    index = build_reachability_index(grid_map, [_uav("uav_01", Position(0, 0))])

    assert index.is_reachable("uav_01", Position(1, 4))
    assert not index.is_reachable("uav_01", Position(4, 4))
    assert Position(4, 4) in index.unreachable_searchable_cells


def test_reachability_index_does_not_cross_no_fly_and_ignores_offline_uavs() -> None:
    grid_map = GridMap(width_m=50, height_m=50, resolution_m=10)
    for y in range(5):
        grid_map.set_cell(Position(2, y), {"cell_type": CellType.NO_FLY})

    index = build_reachability_index(
        grid_map,
        [
            _uav("uav_01", Position(0, 0)),
            _uav("uav_02", Position(4, 4), UAVStatus.OFFLINE),
        ],
    )

    assert index.reachable_uavs(Position(1, 1)) == ["uav_01"]
    assert index.reachable_uavs(Position(4, 4)) == []
    assert not index.any_reachable(Position(4, 4))
