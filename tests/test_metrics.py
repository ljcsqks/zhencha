import json
from pathlib import Path

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.evaluation.metrics import compute_metrics, save_metrics
from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager
from uav_search.uav.uav_model import UAV


def test_compute_metrics_counts_events_and_coverage() -> None:
    grid_map = GridMap(width_m=20, height_m=10, resolution_m=10)
    grid_map.mark_covered(Position(0, 0), radius_cells=0, timestamp=1.0)
    grid_map.mark_covered(Position(1, 0), radius_cells=0, timestamp=2.0)
    state = UAVState(
        id="uav_01",
        position=Position(0, 0),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=0.8,
        sensor_radius_cells=1,
        status=UAVStatus.IDLE,
        home_position=Position(0, 0),
        total_distance_m=20.0,
        effective_search_distance_m=10.0,
    )
    fleet = FleetManager([UAV(state, endurance_s=100.0)])
    snapshots = [
        {
            "time_s": 1.0,
            "global_coverage": 0.5,
            "priority_coverage": 0.0,
            "events": ["target_found_manual"],
            "target_metrics": {
                "target_a": {
                    "found_time_s": 1.0,
                    "assigned_time_s": 1.5,
                    "done_time_s": 2.0,
                    "success": True,
                    "interrupted_task_id": "task_001",
                    "resumed_time_s": 2.5,
                    "coverage_at_done": 0.8,
                }
            },
            "uavs": [{"position": {"x": 0, "y": 0}, "total_distance_m": 5.0, "task_id": None}],
        },
        {
            "time_s": 2.0,
            "global_coverage": 1.0,
            "priority_coverage": 0.0,
            "events": ["confirm_done_confirm_target_001", "scenario_map_update_002"],
            "target_metrics": {
                "target_a": {
                    "found_time_s": 1.0,
                    "assigned_time_s": 1.5,
                    "done_time_s": 2.0,
                    "success": True,
                    "interrupted_task_id": "task_001",
                    "resumed_time_s": 2.5,
                    "coverage_at_done": 0.8,
                }
            },
            "uavs": [{"position": {"x": 0, "y": 0}, "total_distance_m": 20.0, "task_id": "supplemental_001"}],
        },
    ]

    metrics = compute_metrics("test_run", grid_map, fleet, snapshots)

    assert metrics.run_id == "test_run"
    assert metrics.event_count == 3
    assert metrics.conflict_count == 0
    assert metrics.no_fly_violations == 0
    assert metrics.target_found_count == 1
    assert metrics.map_update_count == 1
    assert metrics.confirm_done_count == 1
    assert metrics.path_efficiency == 0.5
    assert metrics.time_to_95_coverage_s == 2.0
    assert metrics.coverage_goal_met
    assert metrics.priority_goal_met
    assert metrics.supplemental_task_count == 1
    assert metrics.final_uncovered_cells == 0
    assert metrics.ignored_uncovered_cells == 0
    assert metrics.target_response_time_s == 0.5
    assert metrics.target_confirm_duration_s == 0.5
    assert metrics.confirm_success_rate == 1.0
    assert metrics.search_resume_delay_s == 0.5
    assert abs(metrics.coverage_gap_at_confirm_done - 0.15) < 1e-9
    assert metrics.interrupted_task_resume_rate == 1.0
    assert metrics.post_95_extra_time_s == 0.0
    assert metrics.post_95_extra_distance_m == 0.0
    assert metrics.algorithm_version == "baseline_sparse_boustrophedon"
    assert metrics.code_version
    assert metrics.config_hash
    assert set(metrics.diagnostics) == {
        "per_uav",
        "route_quality",
        "coverage_quality",
        "allocation_quality",
        "segment_quality",
        "command_quality",
        "scheduler_quality",
    }
    assert "uav_01" in metrics.diagnostics["per_uav"]
    assert "max_connector_length" in metrics.diagnostics["route_quality"]
    assert "uncovered_components_count_at_95" in metrics.diagnostics["coverage_quality"]
    assert "workload_balance" in metrics.diagnostics["allocation_quality"]


def test_save_metrics_writes_json(tmp_path: Path) -> None:
    grid_map = GridMap(width_m=10, height_m=10, resolution_m=10)
    fleet = FleetManager([])
    metrics = compute_metrics("empty", grid_map, fleet, [])
    output = tmp_path / "metrics.json"

    save_metrics(metrics, output)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["run_id"] == "empty"
    assert payload["algorithm_version"] == "baseline_sparse_boustrophedon"
    assert "diagnostics" in payload
