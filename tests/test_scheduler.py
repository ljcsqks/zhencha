from uav_search.core.config import load_config
from uav_search.core.data_types import CommandType, Event, EventPriority, EventType, TaskStatus, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.uav.fleet_manager import FleetManager


def test_scheduler_assigns_tasks_and_paths() -> None:
    config = load_config("config/default.yaml", "config/scenarios/multi_basic.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)

    output = scheduler.regular_cycle(now=0.0)

    assert output.assignments
    assert any(command.command == CommandType.FOLLOW_PATH for command in output.commands)
    assert any(state.path for state in fleet.get_all_states())


def test_scheduler_handles_low_battery_event() -> None:
    config = load_config("config/default.yaml", "config/scenarios/multi_basic.yaml")
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

    assert "low_battery_001" in output.events_handled
    assert any(command.command == CommandType.RETURN_HOME for command in output.commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.RETURNING


def test_scheduler_replans_invalid_path_after_map_update() -> None:
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    scheduler.regular_cycle(now=0.0)
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
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
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

    assert "target_found_001" in output.events_handled
    assert any(command.command == CommandType.CONFIRM_TARGET for command in output.commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.CONFIRMING


def test_target_found_requeues_interrupted_search_task() -> None:
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    initial_output = scheduler.regular_cycle(now=0.0)
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
    scheduler.regular_cycle(now=1.0)

    assert scheduler.task_manager.tasks[interrupted_task_id].status == TaskStatus.PENDING


def test_scheduler_completes_confirmation_after_dwell_steps() -> None:
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
    config["search"]["confirm_duration_steps"] = 2
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
    scheduler.regular_cycle(now=0.0)

    first_commands, first_events = scheduler.update_after_step(now=1.0)
    second_commands, second_events = scheduler.update_after_step(now=2.0)

    assert not first_commands
    assert not first_events
    assert "confirm_done_confirm_target_001" in second_events
    assert any(command.reason == "confirm_done" for command in second_commands)
    assert fleet.get_uav("uav_01").state.status == UAVStatus.IDLE
