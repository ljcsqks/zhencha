import re
from pathlib import Path

from uav_search.core.config import load_config
from uav_search.core.contracts import AckStatus, CommandAck, CommandStatusStore, ControlCommand
from uav_search.core.data_types import CommandType, DecisionCommand, Position, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.core.scheduler_adapter import SchedulerAlgorithmAdapter
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.command_applier import CommandApplier
from uav_search.simulation.observation_builder import ObservationBuilder
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager


def test_control_command_from_decision_has_command_id_issued_at_and_ttl() -> None:
    decision = DecisionCommand(
        uav_id="uav_01",
        command=CommandType.FOLLOW_PATH,
        task_id="task_001",
        target=Position(2, 0),
        path=[Position(0, 0), Position(1, 0), Position(2, 0)],
        reason="test",
    )

    command = ControlCommand.from_decision(decision, issued_at=12.5, ttl_s=3.0)

    assert command.command_id
    assert command.command == CommandType.FOLLOW_PATH
    assert command.issued_at == 12.5
    assert command.ttl_s == 3.0


def test_command_applier_cancel_command_cancels_specific_or_active_command() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    applier = CommandApplier(fleet, grid_map)
    start = fleet.get_uav("uav_01").state.position
    follow = ControlCommand(
        command_id="cmd_follow",
        command=CommandType.FOLLOW_PATH,
        uav_id="uav_01",
        task_id="task_001",
        target=Position(2, 0),
        path=[Position(0, 0), Position(1, 0), Position(2, 0)],
        issued_at=0.0,
    )
    applier.apply([follow], now=0.0)

    cancel_specific = ControlCommand(
        command_id="cmd_cancel_specific",
        command=CommandType.CANCEL_COMMAND,
        uav_id="uav_01",
        task_id=None,
        target=None,
        path=[],
        issued_at=1.0,
        metadata={"command_id": "cmd_follow"},
    )
    acks = applier.apply([cancel_specific], now=1.0)

    assert any(ack.command_id == "cmd_follow" and ack.status == AckStatus.CANCELLED for ack in acks)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.IDLE
    assert fleet.get_uav("uav_01").state.path == []

    applier.apply([follow], now=2.0)
    cancel_active = ControlCommand(
        command_id="cmd_cancel_active",
        command=CommandType.CANCEL_COMMAND,
        uav_id="uav_01",
        task_id=None,
        target=None,
        path=[],
        issued_at=3.0,
    )
    acks = applier.apply([cancel_active], now=3.0)

    assert any(ack.command_id == "cmd_follow" and ack.status == AckStatus.CANCELLED for ack in acks)


def test_observation_builder_emits_full_map_changed_cells_and_ack_window() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    ack_store = CommandStatusStore(max_count=200, max_age_s=30.0)
    for idx in range(205):
        ack_store.record(
            CommandAck(
                command_id=f"cmd_{idx}",
                uav_id="uav_01",
                status=AckStatus.RUNNING,
                issued_at=idx * 0.1,
                updated_at=idx * 0.1,
            )
        )
    ack_store.record(
        CommandAck(
            command_id="active_old",
            uav_id="uav_01",
            status=AckStatus.RUNNING,
            issued_at=0.0,
            updated_at=0.0,
        ),
        active=True,
    )
    changed = [Position(1, 1)]

    observation = ObservationBuilder(grid_map, fleet, config).build(
        tick=7,
        time_s=40.0,
        changed_cells=changed,
        command_acks=ack_store.recent(now=40.0),
    )

    assert observation.tick == 7
    assert len(observation.map.cells) == grid_map.height_cells
    assert len(observation.map.cells[0]) == grid_map.width_cells
    assert observation.changed_cells == changed
    assert len(observation.command_acks) <= 201
    assert any(ack.command_id == "active_old" for ack in observation.command_acks)
    assert all(ack.updated_at >= 10.0 or ack.command_id == "active_old" for ack in observation.command_acks)


def test_scheduler_algorithm_adapter_reuses_scheduler_state_between_ticks() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    adapter = SchedulerAlgorithmAdapter(scheduler)
    builder = ObservationBuilder(grid_map, fleet, config)

    first = adapter.decide(builder.build(tick=0, time_s=0.0))
    task_manager_id = id(scheduler.task_manager)
    second = adapter.decide(builder.build(tick=1, time_s=1.0))

    assert id(adapter.scheduler) == id(scheduler)
    assert id(scheduler.task_manager) == task_manager_id
    assert scheduler._initialized
    assert first.commands
    assert second.task_summary["status_counts"]


def test_initial_snapshot_commands_are_control_commands_with_matching_acks() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    simulator = Simulator(grid_map, fleet, config)

    simulator.run_initial_decision(scheduler)

    snapshot = simulator.snapshots[0]
    assert snapshot["commands"]
    command_ids = {command["command_id"] for command in snapshot["commands"]}
    assert all(command_id is not None for command_id in command_ids)
    assert all(command["issued_at"] is not None for command in snapshot["commands"])
    assert snapshot["command_acks"]
    assert {ack["command_id"] for ack in snapshot["command_acks"]}.issubset(command_ids)


def test_contracts_module_does_not_import_world_implementations() -> None:
    source = Path("uav_search/core/contracts.py").read_text(encoding="utf-8")

    assert "GridMap" not in source
    assert "FleetManager" not in source


def test_scenario_event_injector_returns_due_events_without_scheduler() -> None:
    injector = ScenarioEventInjector(
        [
            {
                "time_s": 1,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_queue",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    )

    assert injector.emit_due(0.0) == []
    assert len(injector.emit_due(1.0)) == 1
    assert injector.emit_due(2.0) == []


def test_target_found_reaches_adapter_through_observation_events() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    simulator = Simulator(grid_map, fleet, config)
    event = ScenarioEventInjector(
        [
            {
                "id": "target_found_via_queue",
                "time_s": 0.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_via_queue",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    ).emit_due(0.0)[0]

    simulator.enqueue_event(event)
    simulator.run(max_steps=1, scheduler=scheduler)

    assert any(
        command["command"] == CommandType.CONFIRM_TARGET.value
        for snapshot in simulator.snapshots
        for command in snapshot["commands"]
    )
    assert any("target_found_via_queue" in snapshot["events"] for snapshot in simulator.snapshots)


def test_map_update_is_applied_by_simulator_and_exposed_as_changed_cells() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    simulator = Simulator(grid_map, fleet, config)
    event = ScenarioEventInjector(
        [
            {
                "id": "map_update_via_queue",
                "time_s": 0.0,
                "type": "MAP_UPDATE",
                "data": {"operation": "SET_CELL", "position": {"x": 1, "y": 1}, "cell_type": "OBSTACLE"},
            }
        ]
    ).emit_due(0.0)[0]

    simulator.enqueue_event(event)
    simulator.run(max_steps=1, scheduler=scheduler)

    assert not grid_map.is_passable(Position(1, 1))
    assert any(snapshot["changed_cells"] for snapshot in simulator.snapshots)


def test_scheduler_no_longer_installs_paths_on_fleet_directly() -> None:
    source = Path("uav_search/core/scheduler.py").read_text(encoding="utf-8")

    assert "fleet.assign_path" not in source
    for forbidden in (
        r"\buav\.path\s*=(?!=)",
        r"\buav\.path_index\s*=(?!=)",
        r"\buav\.status\s*=(?!=)",
        r"\buav\.available\s*=(?!=)",
        r"\buav\.current_task_id\s*=(?!=)",
        r"\bfallback\.path\s*=(?!=)",
        r"\bfallback\.status\s*=(?!=)",
        r"\bfallback\.available\s*=(?!=)",
        r"\bfallback\.current_task_id\s*=(?!=)",
    ):
        assert re.search(forbidden, source) is None


def test_confirm_target_rejected_ack_marks_confirmation_failed() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    adapter = SchedulerAlgorithmAdapter(scheduler)
    event = ScenarioEventInjector(
        [
            {
                "id": "target_found_reject",
                "time_s": 0.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_reject",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    ).emit_due(0.0)[0]
    observation = ObservationBuilder(grid_map, fleet, config).build(tick=0, time_s=0.0, events=[event])
    output = adapter.decide(observation)
    confirm = next(command for command in output.commands if command.command == CommandType.CONFIRM_TARGET)

    rejected = CommandAck(
        command_id=confirm.command_id,
        uav_id=confirm.uav_id,
        status=AckStatus.REJECTED,
        issued_at=confirm.issued_at,
        updated_at=1.0,
        reason="path_not_passable",
    )
    adapter.decide(ObservationBuilder(grid_map, fleet, config).build(tick=1, time_s=1.0, command_acks=[rejected]))

    metrics = scheduler.target_metrics_snapshot()["target_reject"]
    assert metrics["success"] is False
    assert metrics["failed_time_s"] == 1.0
    assert not scheduler.task_status_snapshot()["confirmations"]


def test_simulator_tick_injects_target_found_through_observation_events() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    simulator = Simulator(grid_map, fleet, config)
    injector = ScenarioEventInjector(
        [
            {
                "id": "target_found_tick",
                "time_s": 0.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_tick",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    )

    simulator.tick(scheduler=scheduler, event_injector=injector)

    assert any("target_found_tick" in snapshot["events"] for snapshot in simulator.snapshots)
    assert any(
        command["command"] == CommandType.CONFIRM_TARGET.value
        for snapshot in simulator.snapshots
        for command in snapshot["commands"]
    )


def test_web_style_enqueue_event_then_tick_triggers_confirm_command() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    simulator = Simulator(grid_map, fleet, config)
    event = ScenarioEventInjector(
        [
            {
                "id": "target_found_step",
                "time_s": 0.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_step",
                    "position": {"x": 5, "y": 5},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    ).emit_due(0.0)[0]

    simulator.enqueue_event(event)
    simulator.tick(scheduler=scheduler)

    assert simulator.snapshots[-1]["events"] == ["target_found_step"]
    assert any(command["command"] == CommandType.CONFIRM_TARGET.value for command in simulator.snapshots[-1]["commands"])


def test_confirm_target_completed_ack_drives_confirm_done_and_search_resume() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_2uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    adapter = SchedulerAlgorithmAdapter(scheduler)
    builder = ObservationBuilder(grid_map, fleet, config)
    applier = CommandApplier(fleet, grid_map)

    initial = adapter.decide(builder.build(tick=0, time_s=0.0))
    applier.apply(initial.commands, now=0.0)
    target_event = ScenarioEventInjector(
        [
            {
                "id": "target_found_ack_done",
                "time_s": 1.0,
                "type": "TARGET_FOUND",
                "data": {
                    "target_id": "target_ack_done",
                    "position": {"x": 20, "y": 20},
                    "confidence": 0.9,
                    "target_type": "person",
                },
            }
        ]
    ).emit_due(1.0)[0]
    confirm_output = adapter.decide(builder.build(tick=1, time_s=1.0, events=[target_event]))
    confirm = next(command for command in confirm_output.commands if command.command == CommandType.CONFIRM_TARGET)
    completed = CommandAck(
        command_id=confirm.command_id,
        uav_id=confirm.uav_id,
        status=AckStatus.COMPLETED,
        issued_at=confirm.issued_at,
        updated_at=2.0,
        reason="path_completed",
        progress=1.0,
    )

    done_output = adapter.decide(builder.build(tick=2, time_s=2.0, command_acks=[completed]))

    metrics = scheduler.target_metrics_snapshot()["target_ack_done"]
    assert metrics["success"] is True
    assert metrics["done_time_s"] == 2.0
    assert not scheduler.task_status_snapshot()["confirmations"]
    assert any(command.command == CommandType.FOLLOW_PATH for command in done_output.commands)


def test_scheduler_adapter_no_longer_calls_update_after_step() -> None:
    source = Path("uav_search/core/scheduler_adapter.py").read_text(encoding="utf-8")

    assert "scheduler.update_after_step" not in source


def test_command_applier_rejects_bad_path_geometry() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    applier = CommandApplier(fleet, grid_map)
    start = fleet.get_uav("uav_01").state.position
    bad_start = ControlCommand(
        command_id="cmd_bad_start",
        command=CommandType.FOLLOW_PATH,
        uav_id="uav_01",
        task_id="task_bad",
        target=Position(3, 0),
        path=[Position(3, 0), Position(4, 0)],
        issued_at=0.0,
    )
    discontinuous = ControlCommand(
        command_id="cmd_jump",
        command=CommandType.FOLLOW_PATH,
        uav_id="uav_01",
        task_id="task_bad",
        target=Position(10, 10),
        path=[start, Position(start.x + 10, start.y)],
        issued_at=0.0,
    )

    start_ack = applier.apply([bad_start], now=0.0)[0]
    jump_ack = applier.apply([discontinuous], now=0.0)[0]

    assert start_ack.status == AckStatus.REJECTED
    assert start_ack.reason == "path_start_not_at_uav"
    assert jump_ack.status == AckStatus.REJECTED
    assert jump_ack.reason == "path_not_contiguous"


def test_snapshot_distinguishes_executable_commands_from_advisories() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    simulator = Simulator(grid_map, fleet, config)
    advisory = ControlCommand(
        command_id="cmd_advisory",
        command=CommandType.CONFLICT_YIELD,
        uav_id="uav_01",
        task_id=None,
        target=None,
        path=[],
        issued_at=0.0,
        reason="conflict_time_offset",
        metadata={"advisory": True, "effect": "none"},
    )
    executable = ControlCommand(
        command_id="cmd_hold",
        command=CommandType.HOLD,
        uav_id="uav_01",
        task_id=None,
        target=None,
        path=[],
        issued_at=0.0,
        reason="test_hold",
    )

    simulator.record_snapshot(commands=[advisory, executable])

    commands = simulator.snapshots[-1]["commands"]
    assert commands[0]["advisory"] is True
    assert commands[0]["executable"] is False
    assert commands[1]["advisory"] is False
    assert commands[1]["executable"] is True


def test_run_matches_repeated_tick_for_key_state() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["simulation"]["max_steps"] = 6
    config["simulation"]["mission_grace_steps"] = 0

    run_map = build_grid_map(config)
    run_fleet = FleetManager.from_config(config, config["scenario"])
    run_scheduler = Scheduler(run_map, run_fleet, config)
    run_simulator = Simulator(run_map, run_fleet, config)
    run_simulator.run(max_steps=6, scheduler=run_scheduler)

    tick_map = build_grid_map(config)
    tick_fleet = FleetManager.from_config(config, config["scenario"])
    tick_scheduler = Scheduler(tick_map, tick_fleet, config)
    tick_simulator = Simulator(tick_map, tick_fleet, config)
    for _ in range(6):
        tick_simulator.tick(scheduler=tick_scheduler)
        if all(state.status in (UAVStatus.IDLE, UAVStatus.OFFLINE) for state in tick_fleet.get_all_states()):
            break

    assert tick_simulator.snapshots[-1]["time_s"] == run_simulator.snapshots[-1]["time_s"]
    assert tick_map.coverage_rate() == run_map.coverage_rate()
    assert [state.position for state in tick_fleet.get_all_states()] == [
        state.position for state in run_fleet.get_all_states()
    ]
