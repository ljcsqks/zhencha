from __future__ import annotations

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.planning.facade_modeling_planner import BuildingFootprint, FacadeModelingPlanner


def _uav(uav_id: str, position: Position) -> UAVState:
    return UAVState(
        id=uav_id,
        position=position,
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=2,
        status=UAVStatus.IDLE,
        home_position=position,
    )


def _footprint() -> BuildingFootprint:
    return BuildingFootprint(
        building_id="building_a",
        vertices=[Position(10, 10), Position(18, 10), Position(18, 18), Position(10, 18)],
    )


def test_rectangular_footprint_generates_four_facade_lanes() -> None:
    planner = FacadeModelingPlanner({"sample_step_cells": 2, "default_standoff_cells": 3})
    lanes = planner.generate_facade_lanes(_footprint(), GridMap(width_m=300, height_m=300, resolution_m=10), standoff_cells=3)

    assert {lane.side for lane in lanes} == {"north", "east", "south", "west"}
    assert len(lanes) == 4
    assert all(lane.waypoints for lane in lanes)
    assert all(lane.length_m > 0 for lane in lanes)


def test_standoff_path_does_not_enter_building_footprint_and_is_contiguous() -> None:
    grid_map = GridMap(width_m=300, height_m=300, resolution_m=10)
    planner = FacadeModelingPlanner({"sample_step_cells": 2, "default_standoff_cells": 3})

    plans = planner.plan_modeling(
        footprint=_footprint(),
        grid_map=grid_map,
        uav_states=[_uav("uav_01", Position(2, 2))],
        uav_count=1,
        standoff_cells=3,
        laps=1,
        created_at=0.0,
    )

    assert len(plans) == 1
    building_cells = planner.footprint_cells(_footprint())
    assert not any(point in building_cells for point in plans[0].route)
    assert all(grid_map.is_passable(point) for point in plans[0].route)
    assert _is_contiguous(plans[0].route)


def test_one_uav_receives_complete_perimeter() -> None:
    grid_map = GridMap(width_m=300, height_m=300, resolution_m=10)
    planner = FacadeModelingPlanner({"sample_step_cells": 2, "default_standoff_cells": 3})

    plans = planner.plan_modeling(
        footprint=_footprint(),
        grid_map=grid_map,
        uav_states=[_uav("uav_01", Position(2, 2))],
        uav_count=1,
        standoff_cells=3,
        laps=1,
        created_at=0.0,
    )

    assert plans[0].facade_lane_ids == ["building_a_north", "building_a_east", "building_a_south", "building_a_west"]


def test_two_three_and_four_uavs_split_facade_segments() -> None:
    grid_map = GridMap(width_m=300, height_m=300, resolution_m=10)
    planner = FacadeModelingPlanner({"sample_step_cells": 2, "default_standoff_cells": 3})
    uavs = [_uav(f"uav_{idx:02d}", Position(2 + idx, 2)) for idx in range(1, 5)]

    for count in (2, 3, 4):
        plans = planner.plan_modeling(
            footprint=_footprint(),
            grid_map=grid_map,
            uav_states=uavs,
            uav_count=count,
            standoff_cells=3,
            laps=1,
            created_at=0.0,
        )
        assert len(plans) == count
        assert sum(len(plan.facade_lane_ids) for plan in plans) >= 4
        assert {plan.uav_id for plan in plans} == {f"uav_{idx:02d}" for idx in range(1, count + 1)}
        assert all(_is_contiguous(plan.route) for plan in plans)


def test_unreachable_facade_lane_is_reported() -> None:
    grid_map = GridMap(width_m=300, height_m=300, resolution_m=10)
    for y in range(0, grid_map.height_cells):
        grid_map.set_cell(Position(6, y), {"cell_type": CellType.OBSTACLE})
    planner = FacadeModelingPlanner({"sample_step_cells": 2, "default_standoff_cells": 3})

    plans = planner.plan_modeling(
        footprint=_footprint(),
        grid_map=grid_map,
        uav_states=[_uav("uav_01", Position(2, 2))],
        uav_count=1,
        standoff_cells=3,
        laps=1,
        created_at=0.0,
    )

    assert plans == []
    assert planner.last_diagnostics["modeling_unreachable_facade_lanes"] > 0


def _is_contiguous(path: list[Position]) -> bool:
    return all(max(abs(a.x - b.x), abs(a.y - b.y)) <= 1 for a, b in zip(path, path[1:]))
