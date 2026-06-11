from uav_search.core.data_types import Event, EventPriority, EventType
from uav_search.core.event_manager import EventManager


def test_event_manager_polls_by_priority_then_time() -> None:
    manager = EventManager(debounce_s=0.0)
    manager.emit(Event(id="low", type=EventType.CONFIRM_DONE, timestamp=1.0, priority=EventPriority.NORMAL))
    manager.emit(Event(id="high", type=EventType.LOW_BATTERY, timestamp=2.0, priority=EventPriority.HIGH))

    events = manager.poll_events()

    assert [event.id for event in events] == ["high", "low"]


def test_event_manager_debounces_same_type_and_source() -> None:
    manager = EventManager(debounce_s=0.2)

    accepted = manager.emit(
        Event(
            id="first",
            type=EventType.MAP_UPDATE,
            timestamp=1.0,
            priority=EventPriority.HIGH,
            source_uav_id="uav_01",
        )
    )
    rejected = manager.emit(
        Event(
            id="second",
            type=EventType.MAP_UPDATE,
            timestamp=1.1,
            priority=EventPriority.HIGH,
            source_uav_id="uav_01",
        )
    )

    assert accepted
    assert not rejected
    assert [event.id for event in manager.poll_events()] == ["first"]
