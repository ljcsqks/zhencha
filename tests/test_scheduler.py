from uav_search.core.config import load_config
from uav_search.core.contracts import ControlCommand
from uav_search.core.data_types import (
    CellType,
    CommandType,
    Event,
    EventPriority,
    EventType,
    Position,
    Task,
    TaskStatus,
    TaskType,
    UAVState,
    UAVStatus,
)
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.command_applier import CommandApplier
from uav_search.uav.uav_model import UAV
from uav_search.uav.fleet_manager import FleetManager


def test_scheduler_assigns_tasks_and_paths() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_3uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)

    assert output.assignments
    assert any(command.command == CommandType.FOLLOW_PATH for command in output.commands)
    assert any(state.path for state in fleet.get_all_states())


def test_scheduler_handles_low_battery_event() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_3uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.event_manager.emit(
        Event(
            id="low_battery_001",
            type=EventType.LOW_BATTERY,
            timestamp=0.0,
            priority=EventPriority.HIGH,
            source_uav_id="uav_01",
        )
    )

    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)

    assert "low_battery_001" in output.events_handled
    assert any(command.command == CommandType.RETURN_HOME for command in output.commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.RETURNING


def test_scheduler_replans_invalid_path_after_map_update() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial.commands, now=0.0)
    state = fleet.get_uav("uav_01").state
    blocked_pos = state.path[1]

    scheduler.event_manager.emit(
        Event(
            id="map_update_001",
            type=EventType.MAP_UPDATE,
            timestamp=1.0,
            priority=EventPriority.HIGH,
            data={
                "operation": "SET_CELL",
                "position": {"x": blocked_pos.x, "y": blocked_pos.y},
                "cell_type": "OBSTACLE",
            },
        )
    )
    output = scheduler.regular_cycle(now=1.0)

    assert "map_update_001" in output.events_handled
    assert any(command.command in (CommandType.REPLAN, CommandType.HOLD) for command in output.commands)
    assert not grid_map.is_passable(blocked_pos)


def test_scheduler_handles_target_found_event() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)

    assert "target_found_001" in output.events_handled
    assert any(command.command == CommandType.CONFIRM_TARGET for command in output.commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.CONFIRMING


def test_target_found_without_source_selects_lowest_cost_uav() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_2uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    scheduler.event_manager.emit(
        Event(
            id="target_found_no_source",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id=None,
            data={
                "target_id": "target_near_top",
                "position": {"x": 5, "y": 45},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )

    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)

    command = next(command for command in output.commands if command.command == CommandType.CONFIRM_TARGET)
    assert command.uav_id == "uav_02"
    assert fleet.get_uav("uav_02").state.status == UAVStatus.CONFIRMING


def test_target_found_interrupts_only_one_searching_uav() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_3uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial.commands, now=0.0)

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=1.0,
            priority=EventPriority.CRITICAL,
            source_uav_id=None,
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    target_output = scheduler.regular_cycle(now=1.0)
    _apply_commands(fleet, grid_map, target_output.commands, now=1.0)

    statuses = [state.status for state in fleet.get_all_states()]
    assert statuses.count(UAVStatus.CONFIRMING) == 1
    assert statuses.count(UAVStatus.SEARCHING) >= 1


def test_confirmation_done_resumes_interrupted_search_task() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["confirm_duration_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial_output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial_output.commands, now=0.0)
    interrupted_task_id = initial_output.assignments[0].task_id

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=1.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    target_output = scheduler.regular_cycle(now=1.0)
    _apply_commands(fleet, grid_map, target_output.commands, now=1.0)
    state = fleet.get_uav("uav_01").state
    state.position = state.path[-1]
    state.path_index = len(state.path) - 1

    commands, events = scheduler.update_after_step(now=2.0)
    _apply_commands(fleet, grid_map, commands, now=2.0)

    assert "confirm_done_confirm_target_001" in events
    assert any(command.reason == "resume_interrupted_search" for command in commands)
    assert scheduler.task_manager.tasks[interrupted_task_id].status == TaskStatus.ASSIGNED
    assert fleet.get_uav("uav_01").state.status == UAVStatus.SEARCHING


def test_duplicate_target_id_does_not_dispatch_twice() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    event = Event(
        id="target_found_001",
        type=EventType.TARGET_FOUND,
        timestamp=0.0,
        priority=EventPriority.CRITICAL,
        source_uav_id=None,
        data={
            "target_id": "target_001",
            "position": {"x": 5, "y": 5},
            "confidence": 0.9,
            "target_type": "person",
        },
    )

    first = scheduler.handle_event(event)
    second = scheduler.handle_event(event)

    assert sum(1 for command in first if command.command == CommandType.CONFIRM_TARGET) == 1
    assert second == []


def test_confirm_path_avoids_blocked_target_surroundings() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    target = Position(5, 5)
    grid_map.set_cell(target, {"cell_type": CellType.OBSTACLE})
    grid_map.set_cell(Position(5, 3), {"cell_type": CellType.NO_FLY})
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    commands = scheduler.handle_event(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id=None,
            data={
                "target_id": "target_001",
                "position": {"x": target.x, "y": target.y},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )

    command = next(command for command in commands if command.command == CommandType.CONFIRM_TARGET)
    assert all(grid_map.is_passable(point) for point in command.path)
    assert target not in command.path


def test_unreachable_target_confirmation_fails_without_sticking_confirming() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["confirm_orbit_max_extra_radius_cells"] = 1
    grid_map = build_grid_map(config)
    for y in range(0, 9):
        for x in range(0, 9):
            if (x, y) != (0, 0):
                grid_map.set_cell(Position(x, y), {"cell_type": CellType.OBSTACLE})
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    commands = scheduler.handle_event(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id=None,
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )

    assert any(command.reason == "CONFIRM_FAILED" for command in commands)
    assert all(state.status != UAVStatus.CONFIRMING for state in fleet.get_all_states())


def test_low_battery_uav_is_not_selected_for_confirmation() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_2uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    fleet.get_uav("uav_02").state.battery = 0.21
    scheduler = Scheduler(grid_map, fleet, config)

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_02",
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 45},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    output = scheduler.regular_cycle(now=0.0)

    command = next(command for command in output.commands if command.command == CommandType.CONFIRM_TARGET)
    assert command.uav_id != "uav_02"


def test_target_confirmation_uses_orbit_path() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["confirm_orbit_radius_cells"] = 2
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    target = {"x": 5, "y": 5}

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": target,
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    output = scheduler.regular_cycle(now=0.0)

    command = next(command for command in output.commands if command.command == CommandType.CONFIRM_TARGET)
    orbit_points = scheduler._confirmations["confirm_target_001"]["orbit_waypoints"]
    assert orbit_points
    assert command.path[-1] == orbit_points[-1]
    assert all(max(abs(point.x - target["x"]), abs(point.y - target["y"])) == 2 for point in orbit_points)
    assert all(point != command.target for point in orbit_points)


def test_target_found_requeues_interrupted_search_task() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial_output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial_output.commands, now=0.0)
    interrupted_task_id = initial_output.assignments[0].task_id

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=1.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    target_output = scheduler.regular_cycle(now=1.0)
    _apply_commands(fleet, grid_map, target_output.commands, now=1.0)

    assert scheduler.task_manager.tasks[interrupted_task_id].status == TaskStatus.PENDING


def test_interrupted_search_task_resumes_with_uncovered_waypoints_only() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial_output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial_output.commands, now=0.0)
    interrupted_task_id = initial_output.assignments[0].task_id
    task = scheduler.task_manager.tasks[interrupted_task_id]
    covered_waypoint = task.waypoints[0]
    grid_map.set_cell(covered_waypoint, {"search_confidence": 1.0})

    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=1.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": {"x": 5, "y": 5},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    target_output = scheduler.regular_cycle(now=1.0)
    _apply_commands(fleet, grid_map, target_output.commands, now=1.0)

    resumed_task = scheduler.task_manager.tasks[interrupted_task_id]
    assert resumed_task.status == TaskStatus.PENDING
    assert covered_waypoint not in resumed_task.waypoints


def test_scheduler_completes_confirmation_after_dwell_steps() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["confirm_duration_steps"] = 1
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.event_manager.emit(
        Event(
            id="target_found_001",
            type=EventType.TARGET_FOUND,
            timestamp=0.0,
            priority=EventPriority.CRITICAL,
            source_uav_id="uav_01",
            data={
                "target_id": "target_001",
                "position": {"x": 0, "y": 0},
                "confidence": 0.9,
                "target_type": "person",
            },
        )
    )
    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)
    state = fleet.get_uav("uav_01").state
    state.position = state.path[-1]
    state.path_index = len(state.path) - 1

    first_commands, first_events = scheduler.update_after_step(now=1.0)
    _apply_commands(fleet, grid_map, first_commands, now=1.0)

    assert "confirm_done_confirm_target_001" in first_events
    assert any(command.reason == "confirm_done" for command in first_commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.IDLE


def test_completed_search_dispatches_return_home() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)
    task_id = output.assignments[0].task_id
    task = scheduler.task_manager.tasks[task_id]
    _cover_all_cells_except(grid_map, set())
    state = fleet.get_uav("uav_01").state
    state.status = UAVStatus.IDLE
    state.available = True
    state.position = task.waypoints[-1]

    commands, _ = scheduler.update_after_step(now=10.0)
    _apply_commands(fleet, grid_map, commands, now=10.0)

    assert any(command.command == CommandType.RETURN_HOME and command.reason == "mission_complete" for command in commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.RETURNING


def test_finished_search_route_gets_supplemental_task_when_coverage_remains() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, output.commands, now=0.0)
    task_id = output.assignments[0].task_id
    state = fleet.get_uav("uav_01").state
    state.position = state.path[-1]
    state.path_index = len(state.path) - 1
    state.status = UAVStatus.IDLE
    state.available = True

    scheduler.update_after_step(now=10.0)
    assert scheduler.should_run_regular_cycle()
    output = scheduler.regular_cycle(now=10.0)
    _apply_commands(fleet, grid_map, output.commands, now=10.0)

    assert scheduler.task_manager.tasks[task_id].status == TaskStatus.COMPLETED
    assert any(assignment.task_id.startswith("supplemental_") for assignment in output.assignments)
    assert any(command.command == CommandType.FOLLOW_PATH for command in output.commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.SEARCHING


def test_idle_uav_takes_nonreserved_search_work() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_3uav.yaml")
    config["search"]["min_supplemental_cells"] = 1
    config["search"]["min_supplemental_score"] = 0.0
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial_output = scheduler.regular_cycle(now=0.0)
    _apply_commands(fleet, grid_map, initial_output.commands, now=0.0)
    finished_task_id = next(assignment.task_id for assignment in initial_output.assignments if assignment.uav_id == "uav_01")
    finished_state = fleet.get_uav("uav_01").state
    finished_state.position = finished_state.path[-1]
    finished_state.path_index = len(finished_state.path) - 1
    finished_state.status = UAVStatus.IDLE
    finished_state.available = True

    scheduler.update_after_step(now=10.0)
    reserved_by_active_uavs = scheduler._get_active_search_footprint()
    output = scheduler.regular_cycle(now=10.0)

    assigned_ids = {assignment.task_id for assignment in output.assignments}
    assert assigned_ids
    assert scheduler.task_manager.tasks[finished_task_id].status == TaskStatus.COMPLETED
    for task_id in assigned_ids:
        task = scheduler.task_manager.tasks[task_id]
        assert all(
            scheduler._point_adds_unreserved_search_coverage(
                waypoint,
                int(config["uav"]["sensor_radius_cells"]),
                float(config["search"]["coverage_complete_threshold"]),
                reserved_by_active_uavs,
            )
            for waypoint in task.waypoints
        )


def test_idle_uav_assists_active_search_task_and_donor_replans() -> None:
    config = load_config("config/default.yaml")
    config["map"] = {"width_m": 300, "height_m": 120, "resolution_m": 10}
    config["uav"]["sensor_radius_cells"] = 1
    config["search"].update(
        {
            "idle_assist_enabled": True,
            "idle_assist_min_remaining_cells": 4,
            "idle_assist_min_gain_per_meter": 0.0,
            "idle_assist_max_tasks_per_cycle": 1,
            "idle_assist_donor_keep_front_waypoints": 2,
            "idle_assist_replan_donor": True,
            "min_supplemental_score": 999.0,
            "max_supplemental_tasks_per_run": 0,
            "active_replan_min_interval_s": 0.0,
        }
    )
    grid_map = build_grid_map(config)
    donor = UAVState(
        id="uav_01",
        position=Position(0, 5),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=UAVStatus.SEARCHING,
        home_position=Position(0, 5),
        current_task_id="task_donor",
        path=[Position(x, 5) for x in range(16)],
        path_index=0,
        available=False,
    )
    helper = UAVState(
        id="uav_02",
        position=Position(0, 8),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=1,
        status=UAVStatus.IDLE,
        home_position=Position(0, 8),
        available=True,
    )
    fleet = FleetManager([UAV(donor, endurance_s=1000.0), UAV(helper, endurance_s=1000.0)])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler._initialized = True
    donor_task = Task(
        id="task_donor",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(x, 5) for x in range(16)},
        entry_point=Position(0, 5),
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="uav_01",
        waypoints=[Position(x, 5) for x in range(16)],
        coverage_waypoints=[Position(x, 5) for x in range(16)],
        created_at=0.0,
        updated_at=0.0,
    )
    scheduler.task_manager.add_tasks([donor_task])

    output = scheduler.regular_cycle(now=10.0)

    assist_commands = [
        command
        for command in output.commands
        if command.uav_id == "uav_02" and command.command == CommandType.FOLLOW_PATH and command.task_id
    ]
    donor_replans = [
        command
        for command in output.commands
        if command.uav_id == "uav_01" and command.command == CommandType.REPLAN and command.reason == "idle_assist_donor_replan"
    ]
    assert assist_commands
    assist_task = scheduler.task_manager.tasks[assist_commands[0].task_id]
    assert assist_task.metadata["assist_task"] is True
    assert assist_task.metadata["donor_task_id"] == "task_donor"
    assert assist_task.metadata["donor_uav_id"] == "uav_01"
    assert assist_task.metadata["helper_uav_id"] == "uav_02"
    assert assist_task.allowed_uav_ids == {"uav_02"}
    assert donor_replans
    assert len(scheduler.task_manager.tasks["task_donor"].coverage_waypoints) < 16
    diagnostics = scheduler.diagnostics_snapshot()
    assert diagnostics["idle_assist_created_tasks"] == 1
    assert diagnostics["idle_assist_donor_replans"] == 1


def test_small_ordinary_fragment_is_ignored_after_coverage_goal() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["min_supplemental_cells"] = 4
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    _cover_all_cells_except(grid_map, {Position(0, 0), Position(1, 0)})

    assert grid_map.coverage_rate() >= config["search"]["mission_complete_coverage_threshold"]
    assert scheduler._get_supplemental_candidates() == []


def test_large_ordinary_region_is_not_searched_after_coverage_goal() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["min_supplemental_score"] = 999.0
    grid_map = build_grid_map(config)
    uncovered = {Position(x, y) for y in range(7) for x in range(7)}
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    _cover_all_cells_except(grid_map, uncovered)

    candidates = scheduler._get_supplemental_candidates()

    assert grid_map.coverage_rate() >= config["search"]["mission_complete_coverage_threshold"]
    assert candidates == []
    assert scheduler._mission_goal_met()


def test_priority_fragment_still_gets_supplemental_candidate() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["min_supplemental_cells"] = 8
    config["search"]["priority_complete_threshold"] = 1.0
    grid_map = build_grid_map(config)
    priority_cell = Position(0, 0)
    grid_map.set_cell(priority_cell, {"cell_type": CellType.PRIORITY, "search_priority": 4.0})
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    _cover_all_cells_except(grid_map, {priority_cell})

    candidates = scheduler._get_supplemental_candidates()

    assert len(candidates) == 1
    assert priority_cell in candidates[0].cells
    assert candidates[0].priority_uncovered_cells == 1


def test_low_score_supplemental_region_is_ignored() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["min_supplemental_cells"] = 4
    config["search"]["min_supplemental_score"] = 999.0
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    uncovered = {Position(x, 0) for x in range(4)}
    _cover_all_cells_except(grid_map, uncovered)

    assert scheduler._get_supplemental_candidates() == []


def test_idle_uav_returns_when_only_ignored_fragments_remain() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["min_supplemental_cells"] = 4
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    _cover_all_cells_except(grid_map, {Position(0, 0), Position(1, 0)})
    state = fleet.get_uav("uav_01").state
    state.position = Position(5, 5)
    state.status = UAVStatus.IDLE
    state.available = True

    commands = scheduler._dispatch_completed_search_returns(now=10.0)
    _apply_commands(fleet, grid_map, commands, now=10.0)

    assert any(command.command == CommandType.RETURN_HOME and command.reason == "mission_complete" for command in commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.RETURNING


def test_assignment_reorders_task_waypoints_for_current_uav_position() -> None:
    config = load_config("config/default.yaml", "config/scenarios/area_search_1uav.yaml")
    config["search"]["mission_complete_coverage_threshold"] = 1.0
    grid_map = build_grid_map(config)
    _cover_all_cells_except(grid_map, {Position(0, 0), Position(9, 0)})
    fleet = FleetManager.from_config(config, config["scenario"])
    state = fleet.get_uav("uav_01").state
    state.position = Position(9, 0)
    state.home_position = Position(0, 0)
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler._initialized = True
    task = Task(
        id="manual_search",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(0, 0), Position(9, 0)},
        entry_point=Position(0, 0),
        waypoints=[Position(0, 0), Position(9, 0)],
        estimated_cost_m=90.0,
        uncovered_value=2.0,
    )
    scheduler.task_manager.add_tasks([task])

    output = scheduler.regular_cycle(now=0.0)

    assert output.assignments[0].task_id == "manual_search"
    assert scheduler.task_manager.tasks["manual_search"].waypoints[0] == Position(9, 0)


def _cover_all_cells_except(grid_map, uncovered: set[Position]) -> None:
    for cell in grid_map.get_searchable_cells():
        grid_map.set_cell(cell, {"search_confidence": 0.0 if cell in uncovered else 1.0})


def _apply_commands(fleet: FleetManager, grid_map, commands, now: float) -> None:
    applier = CommandApplier(fleet, grid_map)
    applier.apply([ControlCommand.from_decision(command, issued_at=now) for command in commands], now=now)

