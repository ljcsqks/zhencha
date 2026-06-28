from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import Mock

from fastapi.testclient import TestClient

from uav_search.server.app import app
from uav_search.server.runtime import SimulationRuntime
from uav_search.server.schemas import EventRequest


def test_health_returns_ok() -> None:
    client = TestClient(app)

    response = client.get("/api/health")

    assert response.status_code == 200
    assert response.json()["status"] == "ok"


def test_scenarios_lists_known_files() -> None:
    client = TestClient(app)

    response = client.get("/api/scenarios")

    assert response.status_code == 200
    names = {item["name"] for item in response.json()["scenarios"]}
    assert "area_search_2uav_target_confirm" in names


def test_algorithms_endpoint_lists_supported_versions() -> None:
    client = TestClient(app)

    response = client.get("/api/algorithms")

    assert response.status_code == 200
    payload = response.json()
    versions = {item["version"] for item in payload["algorithms"]}
    assert payload["default_version"] == "adaptive_component_sweep_v1"
    assert versions == {
        "baseline_sparse_boustrophedon",
        "segment_sweep_v1",
        "adaptive_component_sweep_v1",
    }


def test_reset_returns_state_with_map_uavs_and_coverage() -> None:
    client = TestClient(app)

    response = client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_2uav.yaml",
        },
    )

    assert response.status_code == 200
    state = response.json()
    assert state["run_id"]
    assert state["scenario_name"] == "area_search_2uav"
    assert state["map"]["width_cells"] > 0
    assert state["map"]["terrain"]
    assert len(state["uavs"]) == 2
    assert state["global_coverage"] >= 0.0
    assert state["algorithm_version"] == "adaptive_component_sweep_v1"


def test_reset_can_override_algorithm_without_modifying_default_config() -> None:
    client = TestClient(app)
    default_config = Path("config/default.yaml")
    before = default_config.read_text(encoding="utf-8")

    response = client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
            "algorithm_version": "adaptive_component_sweep_v1",
        },
    )

    assert response.status_code == 200
    state = response.json()
    assert state["algorithm_version"] == "adaptive_component_sweep_v1"
    assert state["metrics"]["algorithm_version"] == "adaptive_component_sweep_v1"
    assert default_config.read_text(encoding="utf-8") == before


def test_reset_rejects_unknown_algorithm_version() -> None:
    client = TestClient(app)

    response = client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
            "algorithm_version": "missing_algorithm",
        },
    )

    assert response.status_code == 400
    assert "unknown algorithm_version" in response.text


def test_step_advances_time_and_records_commands_and_acks() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    response = client.post("/api/sim/step", json={"steps": 1})

    assert response.status_code == 200
    state = response.json()
    assert "map" not in state
    assert state["run_id"]
    assert state["time_s"] > 0.0
    assert state["commands"]
    assert state["command_acks"]
    assert all(command["executable"] for command in state["commands"])


def test_state_endpoint_supports_full_and_lite_state() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    full = client.get("/api/sim/state", params={"include_map": True}).json()
    lite = client.get("/api/sim/state", params={"include_map": False, "state_level": "lite"}).json()

    assert "map" in full
    assert "map" not in lite
    assert full["run_id"] == lite["run_id"]


def test_cors_allows_vite_and_localhost_origins() -> None:
    client = TestClient(app)

    response = client.options(
        "/api/health",
        headers={
            "Origin": "http://localhost:5173",
            "Access-Control-Request-Method": "GET",
        },
    )

    assert response.status_code == 200
    assert response.headers["access-control-allow-origin"] == "http://localhost:5173"


def test_target_found_event_then_step_produces_confirm_or_failure() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    event_response = client.post(
        "/api/sim/event",
        json={
            "type": "TARGET_FOUND",
            "source_uav_id": None,
            "data": {
                "target_id": "server_target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.85,
                "target_type": "unknown",
                "orbit_radius_cells": 2,
                "orbit_laps": 1,
                "dwell_s": 1,
            },
        },
    )
    assert event_response.status_code == 200
    event_payload = event_response.json()
    assert event_payload["queued"] is True
    assert event_payload["event_id"].startswith("server_target_found_")
    assert any(item["event_id"] == event_payload["event_id"] for item in event_payload["state"]["pending_events"])

    state = client.post("/api/sim/step", json={"steps": 1}).json()

    assert any(item["event_id"] == event_payload["event_id"] for item in state["recent_events"])
    assert any(item["event_id"] == event_payload["event_id"] for item in state["event_log"])
    commands = {command["command"] for command in state["commands"]}
    target = state["targets"].get("server_target_001", {})
    assert "CONFIRM_TARGET" in commands or target.get("success") is False


def test_map_update_event_then_step_reports_changed_cells() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    event_response = client.post(
        "/api/sim/event",
        json={
            "type": "MAP_UPDATE",
            "data": {
                "operation": "add_obstacle",
                "x": 20,
                "y": 20,
                "width": 5,
                "height": 5,
            },
        },
    )
    assert event_response.json()["queued"] is True
    state = client.post("/api/sim/step", json={"steps": 1}).json()

    assert state["changed_cells"]
    assert "map" not in state
    full_state = client.get("/api/sim/state", params={"include_map": True}).json()
    assert full_state["changed_cells"]
    assert full_state["map"]["passable"][20][20] is False


def test_uav_offline_event_prevents_future_commands_for_uav() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_2uav.yaml",
        },
    )

    client.post(
        "/api/sim/event",
        json={"type": "UAV_OFFLINE", "source_uav_id": "uav_01", "data": {}},
    )
    state = client.post("/api/sim/step", json={"steps": 1}).json()

    uav_01 = next(uav for uav in state["uavs"] if uav["id"] == "uav_01")
    assert uav_01["status"] == "OFFLINE"
    assert all(command["uav_id"] != "uav_01" for command in state["commands"])


def test_websocket_receives_state_frame() -> None:
    client = TestClient(app)

    with client.websocket_connect("/ws/sim") as websocket:
        state = websocket.receive_json()

    assert "time_s" in state
    assert "uavs" in state
    assert "map" in state


def test_websocket_first_frame_full_then_broadcast_lite_state() -> None:
    client = TestClient(app)
    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )

    with client.websocket_connect("/ws/sim") as websocket:
        first = websocket.receive_json()
        response = client.post("/api/sim/step", json={"steps": 1})
        second = websocket.receive_json()

    assert response.status_code == 200
    assert "map" in first
    assert "map" not in second
    assert first["run_id"] == second["run_id"]


def test_start_reset_and_concurrent_event_step_do_not_corrupt_runtime() -> None:
    client = TestClient(app)
    reset_body = {
        "config_path": "config/default.yaml",
        "scenario_path": "config/scenarios/area_search_1uav.yaml",
    }
    assert client.post("/api/sim/reset", json=reset_body).status_code == 200
    assert client.post("/api/sim/start", json={"tick_interval_ms": 50}).status_code == 200
    assert client.post("/api/sim/reset", json=reset_body).status_code == 200

    def post_step() -> int:
        return client.post("/api/sim/step", json={"steps": 1}).status_code

    def post_event() -> int:
        return client.post(
            "/api/sim/event",
            json={
                "type": "TARGET_FOUND",
                "source_uav_id": None,
                "data": {
                    "target_id": "concurrent_target",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "unknown",
                },
            },
        ).status_code

    with ThreadPoolExecutor(max_workers=2) as executor:
        codes = [future.result() for future in (executor.submit(post_step), executor.submit(post_event))]

    assert codes == [200, 200]
    assert client.post("/api/sim/pause").status_code == 200


def test_reset_and_invalid_events_return_clear_client_errors() -> None:
    client = TestClient(app)

    missing = client.post(
        "/api/sim/reset",
        json={"config_path": "config/missing.yaml", "scenario_path": "config/scenarios/area_search_1uav.yaml"},
    )
    assert missing.status_code in (400, 404)
    assert "traceback" not in missing.text.lower()

    client.post(
        "/api/sim/reset",
        json={
            "config_path": "config/default.yaml",
            "scenario_path": "config/scenarios/area_search_1uav.yaml",
        },
    )
    bad_map = client.post(
        "/api/sim/event",
        json={"type": "MAP_UPDATE", "data": {"operation": "add_obstacle", "x": 9999, "y": 0, "width": 1, "height": 1}},
    )
    assert bad_map.status_code in (400, 422)
    assert "traceback" not in bad_map.text.lower()

    bad_uav = client.post(
        "/api/sim/event",
        json={"type": "UAV_OFFLINE", "source_uav_id": "missing_uav", "data": {}},
    )
    assert bad_uav.status_code in (400, 404)
    assert "traceback" not in bad_uav.text.lower()


def test_requirements_uses_httpx_not_httpx2() -> None:
    requirements = Path("requirements.txt").read_text(encoding="utf-8")

    assert "httpx2" not in requirements
    assert any(line.strip() == "httpx" for line in requirements.splitlines())


def test_state_metrics_summary_uses_configured_coverage_threshold() -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )
    runtime.step(1)
    runtime.config["search"]["mission_complete_coverage_threshold"] = 0.001

    state = runtime.get_state(include_map=False, state_level="lite")

    assert state["metrics"]["coverage_goal_met"] is True


def test_lite_state_includes_coverage_changed_cells_for_heatmap_updates() -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )

    state = runtime.step(1)

    assert "map" not in state
    assert state["coverage_changed_cells"]
    changed = state["coverage_changed_cells"][0]
    assert {"x", "y", "coverage_count"}.issubset(changed)
    assert "search_confidence" in changed


def test_lite_state_includes_active_commands_with_remaining_path() -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )

    state = runtime.step(1)

    assert state["active_commands"]
    active = state["active_commands"][0]
    assert active["command_id"]
    assert active["uav_id"] == "uav_01"
    assert active["command"] == "FOLLOW_PATH"
    assert active["remaining_path"]
    assert active["progress"] is not None


def test_lite_state_uses_lightweight_metrics_without_compute_metrics(monkeypatch) -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )
    runtime.step(1)
    compute = Mock(side_effect=AssertionError("full compute_metrics should not run for lite state"))
    monkeypatch.setattr("uav_search.server.state.compute_metrics", compute)

    state = runtime.get_state(include_map=False, state_level="lite")

    assert state["metrics"]["global_coverage"] >= 0.0
    assert "running_command_count" in state["metrics"]
    compute.assert_not_called()


def test_get_metrics_still_uses_full_compute_metrics(monkeypatch) -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )
    runtime.step(1)

    class DummyMetrics:
        def __init__(self) -> None:
            self.run_id = "dummy"
            self.global_coverage = 0.5

    compute = Mock(return_value=DummyMetrics())
    monkeypatch.setattr("uav_search.server.runtime.compute_metrics", compute)

    metrics = runtime.get_metrics()

    assert metrics["run_id"] == "dummy"
    compute.assert_called_once()


def test_multi_step_state_includes_intermediate_command_acks() -> None:
    runtime = SimulationRuntime(
        config_path="config/default.yaml",
        scenario_path="config/scenarios/area_search_1uav.yaml",
    )
    runtime.step(1)
    runtime.enqueue_event(
        EventRequest(
            type="TARGET_FOUND",
            source_uav_id=None,
            data={
                "target_id": "multi_step_target",
                "position": {"x": 2, "y": 2},
                "confidence": 0.85,
                "target_type": "unknown",
                "orbit_radius_cells": 2,
                "orbit_laps": 1,
                "dwell_s": 1,
            },
        )
    )

    state = runtime.step(40)

    confirm_command_ids = {
        command["command_id"]
        for command in state["commands"]
        if command["command"] == "CONFIRM_TARGET"
    }
    assert any(
        ack["command_id"] in confirm_command_ids and ack["status"] == "completed"
        for ack in state["command_acks"]
    )


def test_server_layer_does_not_call_internal_scheduler_or_fleet_methods() -> None:
    server_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in Path("uav_search/server").glob("*.py")
        if path.name != "runtime.py"
    )

    assert "scheduler.regular_cycle" not in server_sources
    assert "scheduler.event_manager.emit" not in server_sources
    assert "fleet.assign_path" not in server_sources
