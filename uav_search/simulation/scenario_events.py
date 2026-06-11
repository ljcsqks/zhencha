from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from uav_search.core.data_types import Event, EventPriority, EventType
from uav_search.core.scheduler import Scheduler


DEFAULT_PRIORITY = {
    EventType.TARGET_FOUND: EventPriority.CRITICAL,
    EventType.UAV_OFFLINE: EventPriority.CRITICAL,
    EventType.LOW_BATTERY: EventPriority.HIGH,
    EventType.MAP_UPDATE: EventPriority.HIGH,
}


@dataclass
class ScheduledScenarioEvent:
    time_s: float
    event: Event
    emitted: bool = False


class ScenarioEventInjector:
    """Convert scenario YAML events into scheduler events at simulation time."""

    def __init__(self, scenario_events: list[dict[str, Any]]) -> None:
        self.events = sorted(
            [self._build_event(index, item) for index, item in enumerate(scenario_events, start=1)],
            key=lambda item: item.time_s,
        )

    def emit_due(self, current_time_s: float, scheduler: Scheduler) -> list[Event]:
        emitted: list[Event] = []
        for scheduled in self.events:
            if scheduled.emitted or scheduled.time_s > current_time_s:
                continue
            scheduler.event_manager.emit(scheduled.event)
            scheduled.emitted = True
            emitted.append(scheduled.event)
        return emitted

    def has_pending(self) -> bool:
        return any(not item.emitted for item in self.events)

    def _build_event(self, index: int, item: dict[str, Any]) -> ScheduledScenarioEvent:
        event_type = EventType(item["type"])
        timestamp = float(item["time_s"])
        priority_value = item.get("priority")
        priority = EventPriority[priority_value] if priority_value else DEFAULT_PRIORITY.get(event_type, EventPriority.NORMAL)
        event = Event(
            id=str(item.get("id", f"scenario_event_{index:03d}")),
            type=event_type,
            timestamp=timestamp,
            priority=priority,
            source_uav_id=item.get("source_uav_id"),
            data=dict(item.get("data", {})),
        )
        return ScheduledScenarioEvent(time_s=timestamp, event=event)
