from uav_search.core.data_types import CommandType, Position, UAVState, UAVStatus
from uav_search.planning.conflict_resolver import detect_conflicts, resolve_conflicts


def _state(uav_id: str, path: list[Position], status: UAVStatus = UAVStatus.SEARCHING) -> UAVState:
    return UAVState(
        id=uav_id,
        position=path[0],
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=2,
        status=status,
        home_position=path[0],
        path=path,
    )


def test_detect_conflicts_finds_same_time_overlap() -> None:
    first = _state("uav_01", [Position(0, 0), Position(1, 0), Position(2, 0)])
    second = _state("uav_02", [Position(2, 0), Position(1, 0), Position(0, 0)])

    conflicts = detect_conflicts([first, second], safety_distance_cells=1.0, time_horizon_steps=3)

    assert conflicts
    assert conflicts[0].position_a == Position(1, 0)
    assert conflicts[0].position_b == Position(1, 0)


def test_resolve_conflicts_inserts_wait_for_lower_priority_uav() -> None:
    high_priority = _state("uav_01", [Position(0, 0), Position(1, 0), Position(2, 0)], UAVStatus.RETURNING)
    low_priority = _state("uav_02", [Position(2, 0), Position(1, 0), Position(0, 0)], UAVStatus.SEARCHING)

    commands = resolve_conflicts(
        detect_conflicts([high_priority, low_priority], safety_distance_cells=1.0, time_horizon_steps=3),
        [high_priority, low_priority],
        safety_distance_cells=1.0,
    )

    assert commands
    assert commands[0].uav_id == "uav_02"
    assert len(low_priority.path) == 3
    assert commands[0].command == CommandType.CONFLICT_YIELD
    assert commands[0].path == []
    assert commands[0].metadata["effect"] == "none"
    assert commands[0].metadata["advisory"] is True
    assert len(commands[0].metadata["suggested_path"]) > 3
