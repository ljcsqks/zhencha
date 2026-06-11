from uav_search.core.config import load_config
from uav_search.core.data_types import CellType, Position
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager


def test_simulator_injects_due_scenario_events() -> None:
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
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
