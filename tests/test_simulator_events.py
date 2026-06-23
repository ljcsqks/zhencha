from uav_search.core.config import load_config
from uav_search.core.data_types import CellType, Event, EventPriority, EventType, Position, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager
from uav_search.uav.uav_model import UAV


def test_simulator_injects_due_scenario_events() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 2
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.regular_cycle(now=0.0)
    injector = ScenarioEventInjector(
        [
            {
                "id": "event_map_update",
                "time_s": 1.0,
                "type": "MAP_UPDATE",
                "data": {"operation": "SET_CELL", "position": {"x": 1, "y": 1}, "cell_type": "OBSTACLE"},
            }
        ]
    )
    simulator = Simulator(grid_map, fleet, config)

    simulator.run(scheduler=scheduler, event_injector=injector)

    assert grid_map.get_cell(Position(1, 1)).cell_type == CellType.OBSTACLE
    assert any("event_map_update" in snapshot["events"] for snapshot in simulator.snapshots)


def test_snapshots_include_commands_and_task_summary() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    decision = scheduler.regular_cycle(now=0.0)
    simulator = Simulator(grid_map, fleet, config)

    simulator.record_snapshot(scheduler=scheduler, commands=decision.commands)

    snapshot = simulator.snapshots[-1]
    assert snapshot["commands"]
    assert {"command", "uav_id", "task_id", "target", "path", "reason"}.issubset(snapshot["commands"][0])
    assert snapshot["tasks"]["status_counts"]["in_progress"] >= 1
    assert "confirmations" in snapshot["tasks"]


def test_simulator_completes_target_confirmation() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["confirm_duration_steps"] = 1
    config["simulation"]["max_steps"] = 8
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.event_manager.emit(
        Event(
            id="target_found_at_home",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_home",
                "position": {"x": 0, "y": 0},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    simulator = Simulator(grid_map, fleet, config)

    simulator.run(scheduler=scheduler)

    assert fleet.get_uav("uav_01").state.status != UAVStatus.CONFIRMING
    assert any("confirm_done_confirm_target_home" in snapshot["events"] for snapshot in simulator.snapshots)


def test_simulator_resumes_search_after_confirmation_and_reaches_coverage_goal() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_2uav.yaml")
    config["search"]["confirm_duration_steps"] = 1
    config["simulation"]["max_steps"] = 260
    config["simulation"]["mission_grace_steps"] = 700
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.regular_cycle(now=0.0)
    injector = ScenarioEventInjector(
        [
            {
                "id": "target_found_no_source",
                "time_s": 10.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_resume",
                    "position": {"x": 20, "y": 20},
                    "confidence": 0.9,
                    "target_type": "person",
                    "dwell_s": 1.0,
                },
            }
        ]
    )
    simulator = Simulator(grid_map, fleet, config)

    simulator.run(scheduler=scheduler, event_injector=injector)

    latest_metrics = simulator.snapshots[-1].get("target_metrics", {})
    assert latest_metrics["target_resume"]["success"]
    assert latest_metrics["target_resume"]["resumed_time_s"] is not None
    assert grid_map.coverage_rate() >= float(config["search"]["mission_complete_coverage_threshold"])


def test_simulator_extends_for_active_return_home() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["time_step_s"] = 1.0
    config["simulation"]["return_home_grace_steps"] = 5
    grid_map = build_grid_map(config)
    uav_state = FleetManager.from_config(config, config["scenario"]).get_uav("uav_01").state
    uav_state.position = Position(3, 0)
    uav_state.home_position = Position(0, 0)
    uav = UAV(uav_state, endurance_s=100.0)
    uav.assign_path([Position(3, 0), Position(2, 0), Position(1, 0), Position(0, 0)], status=UAVStatus.RETURNING)
    fleet = FleetManager([uav])
    simulator = Simulator(grid_map, fleet, config)

    simulator.run(max_steps=1)

    assert simulator.snapshots[-1]["time_s"] == 3.0
    assert fleet.get_uav("uav_01").state.position == Position(0, 0)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.IDLE


def test_simulator_extends_when_search_is_not_finished() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["mission_grace_steps"] = 3
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.regular_cycle(now=0.0)
    simulator = Simulator(grid_map, fleet, config)

    simulator.run(max_steps=1, scheduler=scheduler)

    assert simulator.snapshots[-1]["time_s"] > 1.0
