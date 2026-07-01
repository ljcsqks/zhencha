from __future__ import annotations

from uav_search.core.config import load_config
from uav_search.core.contracts import AckStatus, CommandAck, ControlCommand
from uav_search.core.data_types import CommandType, Event, EventPriority, EventType, Position, Task, TaskStatus, TaskType, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.grid_map import GridMap
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.command_applier import CommandApplier
from uav_search.uav.fleet_manager import FleetManager


def test_modeling_path_completion_releases_uav_from_modeling_status() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    applier = CommandApplier(fleet, grid_map)
    command = ControlCommand(
        command_id="model_cmd_release",
        command=CommandType.MODEL_STRUCTURE,
        uav_id="uav_01",
        task_id="model_task_release",
        target=Position(2, 0),
        path=[Position(0, 0), Position(1, 0), Position(2, 0)],
        issued_at=0.0,
        reason="building_model_request",
    )

    applier.apply([command], now=0.0)
    _step_until_idle_or_stuck(fleet, grid_map, max_steps=5)

    state = fleet.get_uav("uav_01").state
    assert state.position == Position(2, 0)
    assert state.status == UAVStatus.IDLE
    assert state.available is True


def test_modeling_after_search_complete_returns_home_and_finishes_idle() -> None:
    config, grid_map, fleet, scheduler = _ready_scheduler("config/scenarios/area_search_1uav.yaml")
    _mark_mission_covered(grid_map)

    model_command = _request_modeling(scheduler, uav_count=1, now=0.0)
    scheduler.record_issued_commands([ControlCommand.from_decision(model_command, issued_at=0.0)])

    commands = _complete_modeling_ack(scheduler, fleet, model_command, updated_at=30.0)
    return_command = _only_command(commands, CommandType.RETURN_HOME)

    assert return_command.reason == "modeling_done_return_home"
    assert scheduler.task_manager.tasks[model_command.task_id or ""].status == TaskStatus.COMPLETED
    assert scheduler.diagnostics_snapshot()["modeling_no_resume_return_home_count"] == 1
    assert scheduler.diagnostics_snapshot()["modeling_return_home_commands"] == 1

    applier = CommandApplier(fleet, grid_map)
    applier.apply([ControlCommand.from_decision(return_command, issued_at=30.0)], now=30.0)
    _step_until_idle_or_stuck(fleet, grid_map, max_steps=80)
    state = fleet.get_uav(model_command.uav_id).state
    assert state.position == state.home_position
    assert state.status == UAVStatus.IDLE


def test_modeling_during_search_resumes_search_instead_of_returning_home() -> None:
    config, grid_map, fleet, scheduler = _ready_scheduler("config/scenarios/area_search_1uav.yaml")
    search_task = _active_search_task("search_task", "uav_01")
    scheduler.task_manager.add_tasks([search_task])
    uav = fleet.get_uav("uav_01").state
    uav.status = UAVStatus.SEARCHING
    uav.available = False
    uav.current_task_id = search_task.id
    uav.path = [uav.position, Position(5, 0), Position(6, 0)]
    uav.path_index = 0

    model_command = _request_modeling(scheduler, uav_count=1, now=1.0)
    scheduler.record_issued_commands([ControlCommand.from_decision(model_command, issued_at=1.0)])

    commands = _complete_modeling_ack(scheduler, fleet, model_command, updated_at=20.0)

    assert any(command.command == CommandType.FOLLOW_PATH and command.reason == "resume_interrupted_search" for command in commands)
    assert not any(command.command == CommandType.RETURN_HOME for command in commands)
    assert scheduler.diagnostics_snapshot()["modeling_resumed_search_tasks"] == 1


def test_modeling_done_at_home_holds_without_pointless_return() -> None:
    config, grid_map, fleet, scheduler = _ready_scheduler("config/scenarios/area_search_1uav.yaml")
    _mark_mission_covered(grid_map)

    model_command = _request_modeling(
        scheduler,
        uav_count=1,
        now=0.0,
        footprint=[
            {"x": 5, "y": 5},
            {"x": 9, "y": 5},
            {"x": 9, "y": 9},
            {"x": 5, "y": 9},
        ],
    )
    scheduler.record_issued_commands([ControlCommand.from_decision(model_command, issued_at=0.0)])
    fleet.get_uav(model_command.uav_id).state.position = fleet.get_uav(model_command.uav_id).state.home_position

    commands = _complete_modeling_ack(scheduler, fleet, model_command, updated_at=10.0, sync_position=False)
    hold_command = _only_command(commands, CommandType.HOLD)

    assert hold_command.reason == "modeling_done_at_home"
    assert not any(command.command == CommandType.RETURN_HOME for command in commands)
    assert scheduler.diagnostics_snapshot()["modeling_hold_after_done_count"] == 1


def test_resume_search_false_uses_post_modeling_behavior() -> None:
    config, grid_map, fleet, scheduler = _ready_scheduler("config/scenarios/area_search_1uav.yaml")
    _mark_mission_covered(grid_map)
    config["modeling"]["post_modeling_behavior"] = "hold"

    model_command = _request_modeling(scheduler, uav_count=1, now=0.0, resume_search_after=False)
    scheduler.record_issued_commands([ControlCommand.from_decision(model_command, issued_at=0.0)])

    commands = _complete_modeling_ack(scheduler, fleet, model_command, updated_at=10.0)

    assert _only_command(commands, CommandType.HOLD).reason == "modeling_done_wait_for_search_assignment"
    assert not any(command.command == CommandType.FOLLOW_PATH for command in commands)
    assert scheduler.diagnostics_snapshot()["modeling_hold_after_done_count"] == 1


def test_multi_uav_modeling_post_completion_is_per_uav_and_job_done_waits_for_all() -> None:
    config, grid_map, fleet, scheduler = _ready_scheduler("config/scenarios/area_search_3uav.yaml")
    _mark_mission_covered(grid_map)

    output = scheduler.regular_cycle(now=0.0)
    scheduler.task_manager.tasks.clear()
    model_commands = _request_modeling_all(scheduler, uav_count=2, now=1.0)
    scheduler.record_issued_commands([ControlCommand.from_decision(command, issued_at=1.0) for command in model_commands])

    first_commands = _complete_modeling_ack(scheduler, fleet, model_commands[0], updated_at=20.0)
    assert any(command.command == CommandType.RETURN_HOME for command in first_commands)
    assert scheduler.diagnostics_snapshot()["modeling_jobs_completed"] == 0

    second_commands = _complete_modeling_ack(scheduler, fleet, model_commands[1], updated_at=21.0)
    assert any(command.command == CommandType.RETURN_HOME for command in second_commands)
    assert scheduler.diagnostics_snapshot()["modeling_jobs_completed"] == 1


def _ready_scheduler(scenario_path: str) -> tuple[dict, GridMap, FleetManager, Scheduler]:
    config = load_config("config/default.yaml", scenario_path)
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    return config, grid_map, fleet, scheduler


def _mark_mission_covered(grid_map: GridMap) -> None:
    for cell in grid_map.get_searchable_cells():
        grid_map.set_cell(cell, {"search_confidence": 1.0})


def _request_modeling(
    scheduler: Scheduler,
    uav_count: int,
    now: float,
    resume_search_after: bool = True,
    footprint: list[dict[str, int]] | None = None,
) -> object:
    commands = _request_modeling_all(scheduler, uav_count, now, resume_search_after, footprint)
    assert len(commands) >= 1
    return commands[0]


def _request_modeling_all(
    scheduler: Scheduler,
    uav_count: int,
    now: float,
    resume_search_after: bool = True,
    footprint: list[dict[str, int]] | None = None,
) -> list:
    event = Event(
        id=f"building_model_post_{now}",
        type=EventType.BUILDING_MODEL_REQUEST,
        timestamp=now,
        priority=EventPriority.CRITICAL,
        data={
            "building_id": "building_post",
            "footprint": footprint
            or [{"x": 30, "y": 10}, {"x": 38, "y": 10}, {"x": 38, "y": 18}, {"x": 30, "y": 18}],
            "uav_count": uav_count,
            "standoff_cells": 3,
            "laps": 1,
            "resume_search_after": resume_search_after,
        },
    )
    commands = scheduler.handle_event(event)
    return [command for command in commands if command.command == CommandType.MODEL_STRUCTURE]


def _complete_modeling_ack(
    scheduler: Scheduler,
    fleet: FleetManager,
    model_command,
    updated_at: float,
    sync_position: bool = True,
) -> list:
    if sync_position and model_command.path:
        state = fleet.get_uav(model_command.uav_id).state
        state.position = model_command.path[-1]
        state.path = list(model_command.path)
        state.path_index = len(model_command.path) - 1
        state.status = UAVStatus.IDLE
        state.available = True
    return scheduler.handle_command_acks(
        [
            CommandAck(
                command_id=model_command.command_id or "",
                uav_id=model_command.uav_id,
                status=AckStatus.COMPLETED,
                issued_at=model_command.issued_at or 0.0,
                updated_at=updated_at,
                reason="path_completed",
                progress=1.0,
            )
        ]
    )


def _only_command(commands: list, command_type: CommandType):
    matches = [command for command in commands if command.command == command_type]
    assert len(matches) == 1
    return matches[0]


def _active_search_task(task_id: str, uav_id: str) -> Task:
    return Task(
        id=task_id,
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(5, 0), Position(6, 0)},
        entry_point=Position(5, 0),
        waypoints=[Position(5, 0), Position(6, 0)],
        coverage_waypoints=[Position(5, 0), Position(6, 0)],
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id=uav_id,
        created_at=0.0,
        updated_at=0.0,
    )


def _step_until_idle_or_stuck(fleet: FleetManager, grid_map: GridMap, max_steps: int) -> None:
    for _ in range(max_steps):
        fleet.step(time_step_s=1.0, resolution_m=grid_map.resolution_m)
