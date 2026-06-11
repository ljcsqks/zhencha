from uav_search.allocation.auction import SequentialAuction
from uav_search.core.data_types import Position, Task, TaskType, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap


def _uav(uav_id: str, position: Position, battery: float = 1.0) -> UAVState:
    return UAVState(
        id=uav_id,
        position=position,
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=battery,
        sensor_radius_cells=2,
        status=UAVStatus.IDLE,
        home_position=position,
    )


def _task(task_id: str, entry: Position, priority: float = 1.0) -> Task:
    return Task(
        id=task_id,
        type=TaskType.SEARCH,
        priority=priority,
        target_cells={entry},
        entry_point=entry,
        waypoints=[entry],
    )


def test_auction_assigns_nearest_uav() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    auction = SequentialAuction({"battery_threshold": 0.2, "auction": {}})

    assignments = auction.allocate(
        [_task("task_001", Position(8, 0))],
        [_uav("uav_01", Position(0, 0)), _uav("uav_02", Position(9, 0))],
        grid_map,
    )

    assert assignments[0].uav_id == "uav_02"


def test_auction_filters_low_battery_uav() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    auction = SequentialAuction({"battery_threshold": 0.2, "auction": {}})

    assignments = auction.allocate(
        [_task("task_001", Position(1, 0))],
        [_uav("uav_01", Position(1, 0), battery=0.1), _uav("uav_02", Position(9, 0))],
        grid_map,
    )

    assert assignments[0].uav_id == "uav_02"
