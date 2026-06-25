from __future__ import annotations

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.evaluation.diagnostics import compute_diagnostics
from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager
from uav_search.uav.uav_model import UAV


def _fleet() -> FleetManager:
    states = [
        UAVState(
            id="uav_01",
            position=Position(0, 0),
            velocity_mps=10.0,
            heading_deg=0.0,
            battery=1.0,
            sensor_radius_cells=1,
            status=UAVStatus.IDLE,
            home_position=Position(0, 0),
            total_distance_m=100.0,
        ),
        UAVState(
            id="uav_02",
            position=Position(0, 1),
            velocity_mps=10.0,
            heading_deg=0.0,
            battery=1.0,
            sensor_radius_cells=1,
            status=UAVStatus.IDLE,
            home_position=Position(0, 1),
            total_distance_m=0.0,
        ),
    ]
    return FleetManager([UAV(state, endurance_s=100.0) for state in states])


def test_workload_balance_all_uavs_exposes_idle_aircraft() -> None:
    diagnostics = compute_diagnostics(
        GridMap(width_m=50, height_m=50, resolution_m=10),
        _fleet(),
        [
            {
                "time_s": 1.0,
                "global_coverage": 0.2,
                "uavs": [
                    {"id": "uav_01", "status": "SEARCHING", "position": {"x": 1, "y": 0}, "total_distance_m": 100.0},
                    {"id": "uav_02", "status": "IDLE", "position": {"x": 0, "y": 1}, "total_distance_m": 0.0},
                ],
            }
        ],
    )

    allocation = diagnostics["allocation_quality"]
    assert allocation["workload_balance_active_uavs"] == 1.0
    assert allocation["workload_balance_all_uavs"] < 0.7
    assert diagnostics["per_uav"]["uav_02"]["idle_time_ratio"] == 1.0
    assert allocation["fleet_idle_time_ratio"] > 0.0


def test_logical_connector_metrics_detect_long_coverage_waypoint_jump() -> None:
    diagnostics = compute_diagnostics(
        GridMap(width_m=200, height_m=200, resolution_m=10),
        _fleet(),
        [
            {
                "time_s": 1.0,
                "global_coverage": 0.2,
                "commands": [
                    {
                        "command": "FOLLOW_PATH",
                        "metadata": {
                            "logical_waypoints": [
                                {"x": 0, "y": 0},
                                {"x": 15, "y": 0},
                            ]
                        },
                        "path": [{"x": x, "y": 0} for x in range(16)],
                    }
                ],
                "uavs": [{"id": "uav_01", "status": "SEARCHING", "position": {"x": 0, "y": 0}, "total_distance_m": 150.0}],
            }
        ],
    )

    route_quality = diagnostics["route_quality"]
    assert route_quality["max_connector_length"] <= 1.0
    assert route_quality["max_logical_connector_length"] == 15.0
    assert route_quality["long_logical_connector_count"] == 1


def test_post_95_search_distance_excludes_return_home() -> None:
    diagnostics = compute_diagnostics(
        GridMap(width_m=100, height_m=100, resolution_m=10),
        _fleet(),
        [
            {
                "time_s": 1.0,
                "global_coverage": 0.95,
                "uavs": [{"id": "uav_01", "status": "SEARCHING", "position": {"x": 1, "y": 0}, "total_distance_m": 100.0}],
            },
            {
                "time_s": 2.0,
                "global_coverage": 0.96,
                "uavs": [{"id": "uav_01", "status": "RETURNING", "position": {"x": 0, "y": 0}, "total_distance_m": 150.0}],
            },
        ],
    )

    coverage = diagnostics["coverage_quality"]
    assert coverage["post_95_distance_m"] == 50.0
    assert coverage["post_95_search_distance_m"] == 0.0
    assert coverage["post_95_return_distance_m"] == 50.0


def test_segment_diagnostics_read_command_metadata() -> None:
    diagnostics = compute_diagnostics(
        GridMap(width_m=200, height_m=200, resolution_m=10),
        _fleet(),
        [
            {
                "time_s": 1.0,
                "commands": [
                    {
                        "command": "FOLLOW_PATH",
                        "uav_id": "uav_01",
                        "metadata": {
                            "planner_version": "segment_sweep_v1",
                            "segment_count": 2,
                            "segment_ids": ["s1", "s2"],
                            "estimated_connector_cost_m": 30.0,
                            "estimated_sweep_cost_m": 100.0,
                            "segment_orientation": "horizontal",
                        },
                    },
                    {
                        "command": "FOLLOW_PATH",
                        "uav_id": "uav_02",
                        "metadata": {
                            "planner_version": "segment_sweep_v1",
                            "segment_count": 1,
                            "segment_ids": ["s3"],
                            "estimated_connector_cost_m": 10.0,
                            "estimated_sweep_cost_m": 40.0,
                            "segment_orientation": "vertical",
                        },
                    },
                ],
            }
        ],
    )

    segment_quality = diagnostics["segment_quality"]
    assert segment_quality["segment_count_total"] == 3
    assert segment_quality["unique_segment_count"] == 3
    assert segment_quality["segment_count_per_uav"] == {"uav_01": 2, "uav_02": 1}
    assert segment_quality["estimated_connector_cost_per_uav"] == {"uav_01": 30.0, "uav_02": 10.0}
    assert segment_quality["max_segment_bundle_cost"] == 130.0
    assert segment_quality["segment_orientation"] in {"horizontal", "mixed"}


def test_segment_diagnostics_deduplicate_replanned_segment_ids() -> None:
    snapshots = [
        {
            "time_s": 1.0,
            "commands": [
                {
                    "command": "FOLLOW_PATH",
                    "uav_id": "uav_01",
                    "metadata": {
                        "planner_version": "segment_sweep_v1",
                        "segment_count": 2,
                        "segment_ids": ["s1", "s2"],
                        "estimated_connector_cost_m": 10.0,
                        "estimated_sweep_cost_m": 20.0,
                    },
                }
            ],
        },
        {
            "time_s": 2.0,
            "commands": [
                {
                    "command": "REPLAN",
                    "uav_id": "uav_01",
                    "metadata": {
                        "planner_version": "segment_sweep_v1",
                        "segment_count": 2,
                        "segment_ids": ["s1", "s2"],
                        "estimated_connector_cost_m": 10.0,
                        "estimated_sweep_cost_m": 20.0,
                    },
                }
            ],
        },
    ]

    diagnostics = compute_diagnostics(GridMap(width_m=200, height_m=200, resolution_m=10), _fleet(), snapshots)

    assert diagnostics["segment_quality"]["segment_count_total"] == 2
    assert diagnostics["segment_quality"]["unique_segment_count"] == 2
