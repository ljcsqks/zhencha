from uav_search.core.config import load_config
from uav_search.core.data_types import CommandType
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
