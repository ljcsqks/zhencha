from uav_search.core.config import load_config
from uav_search.core.data_types import EventType
from uav_search.simulation.scenario_events import ScenarioEventInjector


def test_scenario_event_injector_emits_due_events_once() -> None:
    injector = ScenarioEventInjector(
        [
            {
                "time_s": 5,
                "type": "MAP_UPDATE",
                "data": {"operation": "SET_CELL", "position": {"x": 1, "y": 1}, "cell_type": "OBSTACLE"},
            }
        ]
    )

    assert injector.emit_due(4.0) == []
    emitted = injector.emit_due(5.0)
    emitted_again = injector.emit_due(6.0)

    assert len(emitted) == 1
    assert emitted[0].type == EventType.MAP_UPDATE
    assert emitted_again == []


def test_target_confirm_scenario_is_separate_from_base_2uav() -> None:
    base = load_config("config/default.yaml", "config/scenarios/area_search_2uav.yaml")
    target_confirm = load_config("config/default.yaml", "config/scenarios/area_search_2uav_target_confirm.yaml")

    assert base["scenario"].get("events", []) == []
    assert any(
        event["type"] == EventType.TARGET_FOUND.value
        for event in target_confirm["scenario"].get("events", [])
    )
