from uav_search.core.config import load_config
from uav_search.core.data_types import EventType
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.uav.fleet_manager import FleetManager


def test_scenario_event_injector_emits_due_events_once() -> None:
    config = load_config("config/default.yaml", "config/scenarios/basic.yaml")
    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, config["scenario"])
    scheduler = Scheduler(grid_map, fleet, config)
    injector = ScenarioEventInjector(
        [
            {
                "time_s": 5,
                "type": "MAP_UPDATE",
                "data": {"operation": "SET_CELL", "position": {"x": 1, "y": 1}, "cell_type": "OBSTACLE"},
            }
        ]
    )

    assert injector.emit_due(4.0, scheduler) == []
    emitted = injector.emit_due(5.0, scheduler)
    emitted_again = injector.emit_due(6.0, scheduler)

    assert len(emitted) == 1
    assert emitted[0].type == EventType.MAP_UPDATE
    assert emitted_again == []
