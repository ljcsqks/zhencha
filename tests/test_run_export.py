from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from uav_search.server.app import app


def test_export_empty_run_returns_clear_error() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    response = client.post("/api/sim/export")

    assert response.status_code == 400
    assert "snapshots" in response.json()["detail"]


def test_export_run_writes_required_files_and_summary() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )
    client.post("/api/sim/step", json={"steps": 3})

    response = client.post("/api/sim/export")

    assert response.status_code == 200
    payload = response.json()
    export_dir = Path(payload["export_dir"])
    assert export_dir.parts[-2:] == ("web_exports", payload["run_id"])
    expected_files = {
        "snapshots.json",
        "metrics.json",
        "final_state.json",
        "event_log.json",
        "command_log.json",
        "scenario.yaml",
        "config.yaml",
        "summary.json",
    }
    assert expected_files.issubset(set(payload["files"]))
    for filename in expected_files:
        assert (export_dir / filename).exists()

    summary = json.loads((export_dir / "summary.json").read_text(encoding="utf-8"))
    for field in (
        "run_id",
        "scenario_name",
        "final_time_s",
        "final_coverage",
        "priority_coverage",
        "time_to_95_coverage_s",
        "total_distance_m",
        "redundant_coverage_rate",
        "no_fly_violations",
        "target_found_count",
        "confirm_done_count",
        "confirm_success_rate",
        "interrupted_task_resume_rate",
        "algorithm_version",
        "code_version",
        "config_hash",
        "diagnostics",
        "workload_balance",
        "post_95_extra_distance_m",
        "max_connector_length",
        "idle_time_ratio",
        "exported_at",
    ):
        assert field in summary
    assert summary["algorithm_version"] == "adaptive_component_sweep_v1"


def test_export_summary_uses_reset_algorithm_override() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
            "algorithm_version": "adaptive_component_sweep_v1",
        },
    )
    client.post("/api/sim/step", json={"steps": 1})

    response = client.post("/api/sim/export")

    assert response.status_code == 200
    summary_path = Path(response.json()["export_dir"]) / "summary.json"
    snapshots_path = Path(response.json()["export_dir"]) / "snapshots.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    snapshots = json.loads(snapshots_path.read_text(encoding="utf-8"))
    assert summary["algorithm_version"] == "adaptive_component_sweep_v1"
    assert snapshots["algorithm_version"] == "adaptive_component_sweep_v1"


def test_export_does_not_change_runtime_state() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )
    client.post("/api/sim/step", json={"steps": 2})
    before = client.get("/api/sim/state", params={"include_map": False, "state_level": "lite"}).json()

    response = client.post("/api/sim/export", json={"export_dir": "../../outside"})

    assert response.status_code == 200
    after = client.get("/api/sim/state", params={"include_map": False, "state_level": "lite"}).json()
    assert after["run_id"] == before["run_id"]
    assert after["tick"] == before["tick"]
    assert after["time_s"] == before["time_s"]
    assert Path(response.json()["export_dir"]).parts[-2] == "web_exports"


def test_demo_scenarios_are_listed() -> None:
    client = TestClient(app)

    response = client.get("/api/scenarios")

    names = {item["name"] for item in response.json()["scenarios"]}
    assert {
        "demo_search_3uav",
        "demo_target_confirm",
        "demo_dynamic_obstacle",
        "demo_uav_offline_recover",
    }.issubset(names)
