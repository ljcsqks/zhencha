from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.uav.uav_model import UAV


def test_uav_moves_along_path_and_consumes_battery() -> None:
    state = UAVState(
        id="uav_01",
        position=Position(0, 0),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=2,
        status=UAVStatus.IDLE,
        home_position=Position(0, 0),
    )
    uav = UAV(state, endurance_s=100.0)
    uav.assign_path([Position(0, 0), Position(1, 0), Position(2, 0)])

    traveled = uav.move_along_path(time_step_s=1.0, resolution_m=10.0)

    assert traveled == 10.0
    assert state.position == Position(1, 0)
    assert state.battery < 1.0


def test_can_reach_and_return_checks_reserve() -> None:
    state = UAVState(
        id="uav_01",
        position=Position(0, 0),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=0.5,
        sensor_radius_cells=2,
        status=UAVStatus.IDLE,
        home_position=Position(0, 0),
    )
    uav = UAV(state, endurance_s=100.0)

    assert uav.can_reach_and_return(Position(2, 0), Position(0, 0), resolution_m=10.0, reserve=0.2)


def test_diagonal_step_uses_movement_carry() -> None:
    state = UAVState(
        id="uav_01",
        position=Position(0, 0),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=2,
        status=UAVStatus.IDLE,
        home_position=Position(0, 0),
    )
    uav = UAV(state, endurance_s=100.0)
    uav.assign_path([Position(0, 0), Position(1, 1)])

    first_step = uav.move_along_path(time_step_s=1.0, resolution_m=10.0)
    second_step = uav.move_along_path(time_step_s=1.0, resolution_m=10.0)

    assert first_step == 0.0
    assert second_step > 14.0
    assert state.position == Position(1, 1)
