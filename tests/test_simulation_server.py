from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient

from uav_search.server.app import app


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
    assert state["scenario_name"] == "area_search_2uav"
    assert state["map"]["width_cells"] > 0
    assert state["map"]["terrain"]
    assert len(state["uavs"]) == 2
    assert state["global_coverage"] >= 0.0


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
    assert state["time_s"] > 0.0
    assert state["commands"]
    assert state["command_acks"]
    assert all(command["executable"] for command in state["commands"])


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

    state = client.post("/api/sim/step", json={"steps": 1}).json()

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

    client.post(
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
    state = client.post("/api/sim/step", json={"steps": 1}).json()

    assert state["changed_cells"]
    assert state["map"]["passable"][20][20] is False


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


def test_server_layer_does_not_call_internal_scheduler_or_fleet_methods() -> None:
    server_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in Path("uav_search/server").glob("*.py")
        if path.name != "runtime.py"
    )

    assert "scheduler.regular_cycle" not in server_sources
    assert "scheduler.event_manager.emit" not in server_sources
    assert "fleet.assign_path" not in server_sources
