from __future__ import annotations

import heapq

from uav_search.core.data_types import Event


class EventManager:
    """Priority event queue with lightweight same-type debounce."""

    def __init__(self, debounce_s: float = 0.2) -> None:
        self.debounce_s = max(0.0, float(debounce_s))
        self._queue: list[Event] = []
        self._last_event_time: dict[tuple[str, str | None], float] = {}

    def emit(self, event: Event) -> bool:
        key = (event.type.value, event.source_uav_id)
        last_time = self._last_event_time.get(key)
        if last_time is not None and event.timestamp - last_time < self.debounce_s:
            return False
        self._last_event_time[key] = event.timestamp
        heapq.heappush(self._queue, event)
        return True

    def poll_events(self, current_time: float | None = None) -> list[Event]:
        events: list[Event] = []
        while self._queue:
            event = heapq.heappop(self._queue)
            if current_time is not None and event.timestamp > current_time:
                heapq.heappush(self._queue, event)
                break
            events.append(event)
        return events

    def has_events(self) -> bool:
        return bool(self._queue)
