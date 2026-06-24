from __future__ import annotations

import asyncio
from pathlib import Path
from threading import RLock
from typing import Any, Awaitable, Callable
from uuid import uuid4

from uav_search.core.config import load_config, load_yaml, validate_config
from uav_search.core.data_types import CellType, Event, EventPriority, EventType
from uav_search.core.scheduler import Scheduler
from uav_search.evaluation.metrics import compute_metrics
from uav_search.maps.map_loader import build_grid_map
from uav_search.server.schemas import EventRequest
from uav_search.server.state import build_state
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager

StateCallback = Callable[[dict[str, Any]], Awaitable[None]]


class SimulationRuntime:
    def __init__(
        self,
        config_path: str | Path = "config/default.yaml",
        scenario_path: str | Path = "config/scenarios/area_search_1uav.yaml",
    ) -> None:
        self.config_path = str(config_path)
        self.scenario_path = str(scenario_path)
        self.config: dict[str, Any] = {}
        self.scenario: dict[str, Any] = {}
        self.grid_map = None
        self.fleet = None
        self.scheduler: Scheduler | None = None
        self.simulator: Simulator | None = None
        self.event_injector: ScenarioEventInjector | None = None
        self.running = False
        self._loop_task: asyncio.Task | None = None
        self._state_callback: StateCallback | None = None
        self._lock = RLock()
        self.run_id = ""
        self._pending_event_records: dict[str, dict[str, Any]] = {}
        self._recent_events: list[dict[str, Any]] = []
        self._event_log: list[dict[str, Any]] = []
        self.reset(self.config_path, self.scenario_path)

    def set_state_callback(self, callback: StateCallback | None) -> None:
        self._state_callback = callback

    def reset(self, config_path: str | Path | None = None, scenario_path: str | Path | None = None) -> dict[str, Any]:
        with self._lock:
            self._pause_unlocked()
            next_config_path = Path(config_path or self.config_path)
            next_scenario_path = Path(scenario_path or self.scenario_path)
            if not next_config_path.exists():
                raise FileNotFoundError(f"config_path not found: {next_config_path}")
            if not next_scenario_path.exists():
                raise FileNotFoundError(f"scenario_path not found: {next_scenario_path}")
            self.config_path = str(next_config_path)
            self.scenario_path = str(next_scenario_path)
            self.run_id = f"run_{uuid4().hex[:12]}"
            self._pending_event_records = {}
            self._recent_events = []
            self._event_log = []
            self.config = load_config(self.config_path, self.scenario_path)
            validate_config(self.config)
            self.scenario = self.config.get("scenario") or load_yaml(self.scenario_path)
            self.grid_map = build_grid_map(self.config)
            self.fleet = FleetManager.from_config(self.config, self.scenario)
            self.scheduler = Scheduler(self.grid_map, self.fleet, self.config)
            self.simulator = Simulator(self.grid_map, self.fleet, self.config)
            self.event_injector = ScenarioEventInjector(self.scenario.get("events", []))
            return self.get_state(include_map=True, state_level="full")

    def step(self, steps: int = 1, include_map: bool = False, state_level: str = "lite") -> dict[str, Any]:
        with self._lock:
            bounded_steps = max(1, min(int(steps), 100))
            batch_snapshots: list[dict[str, Any]] = []
            for _ in range(bounded_steps):
                self._require_ready()
                self.simulator.tick(scheduler=self.scheduler, event_injector=self.event_injector)
                self._refresh_event_observability()
                if self.simulator.snapshots:
                    batch_snapshots.append(self.simulator.snapshots[-1])
            state = self.get_state(include_map=include_map, state_level=state_level)
            if bounded_steps > 1:
                _merge_batch_snapshots_into_state(state, batch_snapshots)
            return state

    def start(self, tick_interval_ms: int = 100) -> dict[str, Any]:
        with self._lock:
            if self.running:
                return self.get_state(include_map=False, state_level="lite")
            self.running = True
            try:
                loop = asyncio.get_running_loop()
            except RuntimeError:
                return self.get_state(include_map=False, state_level="lite")
            self._loop_task = loop.create_task(self._run_loop(max(10, int(tick_interval_ms)) / 1000.0))
            return self.get_state(include_map=False, state_level="lite")

    def pause(self) -> dict[str, Any]:
        with self._lock:
            self._pause_unlocked()
            if self.simulator is None:
                return {}
            return self.get_state(include_map=False, state_level="lite")

    def enqueue_event(self, event_request: EventRequest) -> dict[str, Any]:
        with self._lock:
            self._require_ready()
            event = self._event_from_request(event_request)
            self._validate_event(event)
            self.simulator.enqueue_event(event)
            record = self._event_record(event, status="queued")
            self._pending_event_records[event.id] = record
            self._append_event_log(record)
            return {
                "event_id": event.id,
                "queued": True,
                "state": self.get_state(include_map=False, state_level="lite"),
            }

    def _pause_unlocked(self) -> None:
        self.running = False
        if self._loop_task is not None and not self._loop_task.done():
            self._loop_task.cancel()
        self._loop_task = None

    def get_state(self, include_map: bool = True, state_level: str = "full") -> dict[str, Any]:
        with self._lock:
            self._require_ready()
            normalized_level = "full" if state_level == "full" else "lite"
            return build_state(
                simulator=self.simulator,
                grid_map=self.grid_map,
                fleet=self.fleet,
                scheduler=self.scheduler,
                config=self.config,
                scenario_name=str(self.scenario.get("name", Path(self.scenario_path).stem)),
                running=self.running,
                run_id=self.run_id,
                include_map=include_map,
                state_level=normalized_level,
                pending_events=list(self._pending_event_records.values()),
                recent_events=self._recent_events,
                event_log=self._event_log,
            )

    def get_metrics(self) -> dict[str, Any]:
        with self._lock:
            self._require_ready()
            if not self.simulator.snapshots:
                return self.get_state(include_map=False, state_level="lite")["metrics"]
            metrics = compute_metrics(
                str(self.scenario.get("name", Path(self.scenario_path).stem)),
                self.grid_map,
                self.fleet,
                self.simulator.snapshots,
                mission_complete_coverage_threshold=float(
                    self.config.get("search", {}).get("mission_complete_coverage_threshold", 0.95)
                ),
            )
            return metrics.__dict__.copy()

    def get_scenarios(self, scenario_dir: str | Path = "config/scenarios") -> list[dict[str, str]]:
        scenarios: list[dict[str, str]] = []
        for path in sorted(Path(scenario_dir).glob("*.yaml")):
            data = load_yaml(path)
            scenarios.append(
                {
                    "name": str(data.get("name", path.stem)),
                    "path": str(path).replace("\\", "/"),
                    "description": str(data.get("description", "")),
                }
            )
        return scenarios

    async def _run_loop(self, interval_s: float) -> None:
        try:
            while self.running:
                state = self.step(1, include_map=False, state_level="lite")
                if self._state_callback is not None:
                    await self._state_callback(state)
                await asyncio.sleep(interval_s)
        except asyncio.CancelledError:
            return

    def _event_from_request(self, request: EventRequest) -> Event:
        event_type = EventType(request.type)
        data = _normalize_event_data(event_type, request.data)
        priority = EventPriority.CRITICAL if event_type == EventType.TARGET_FOUND else EventPriority.HIGH
        return Event(
            id=f"server_{event_type.value.lower()}_{uuid4().hex[:8]}",
            type=event_type,
            timestamp=self.simulator.time_s if request.time_s is None else float(request.time_s),
            priority=priority,
            source_uav_id=request.source_uav_id,
            data=data,
        )

    def _validate_event(self, event: Event) -> None:
        if event.type in (EventType.UAV_OFFLINE, EventType.UAV_RECOVERED):
            uav_id = event.source_uav_id or event.data.get("uav_id")
            if not uav_id:
                raise ValueError(f"{event.type.value} requires source_uav_id or data.uav_id")
            known_uavs = {state.id for state in self.fleet.get_all_states()}
            if str(uav_id) not in known_uavs:
                raise KeyError(f"unknown UAV id: {uav_id}")
            return
        if event.type != EventType.MAP_UPDATE:
            return
        updates = event.data.get("updates", [])
        if not updates and "operation" in event.data:
            updates = [event.data]
        if not updates:
            raise ValueError("MAP_UPDATE requires at least one update")
        for update in updates:
            operation = str(update.get("operation", "")).upper()
            x = int(update.get("x", 0))
            y = int(update.get("y", 0))
            width = int(update.get("width", 1))
            height = int(update.get("height", 1))
            if width <= 0 or height <= 0:
                raise ValueError("MAP_UPDATE width and height must be positive")
            if operation in {"SET_REGION", "CLEAR_REGION", "ADD_OBSTACLE", "ADD_OBSTACLES", "REMOVE_OBSTACLE"}:
                if x < 0 or y < 0 or x + width > self.grid_map.width_cells or y + height > self.grid_map.height_cells:
                    raise ValueError("MAP_UPDATE region is outside map bounds")
            elif operation == "SET_CELL":
                if x < 0 or y < 0 or x >= self.grid_map.width_cells or y >= self.grid_map.height_cells:
                    raise ValueError("MAP_UPDATE cell is outside map bounds")

    def _refresh_event_observability(self) -> None:
        latest = self.simulator.snapshots[-1] if self.simulator.snapshots else {}
        handled_ids = list(latest.get("events", []))
        recent: list[dict[str, Any]] = []
        for event_id in handled_ids:
            queued_record = self._pending_event_records.pop(event_id, None)
            if queued_record is None:
                queued_record = {
                    "event_id": event_id,
                    "type": "SCENARIO_OR_INTERNAL",
                    "status": "queued",
                    "queued_at_s": None,
                    "handled_at_s": None,
                    "source": "scenario_or_internal",
                }
            handled_record = dict(queued_record)
            handled_record["status"] = "handled"
            handled_record["handled_at_s"] = self.simulator.time_s
            recent.append(handled_record)
            self._append_event_log(handled_record)
        self._recent_events = recent

    def _event_record(self, event: Event, status: str) -> dict[str, Any]:
        return {
            "event_id": event.id,
            "type": event.type.value,
            "status": status,
            "queued_at_s": self.simulator.time_s,
            "handled_at_s": None,
            "source_uav_id": event.source_uav_id,
            "data": event.data,
            "source": "api",
        }

    def _append_event_log(self, record: dict[str, Any]) -> None:
        self._event_log.append(dict(record))
        self._event_log = self._event_log[-200:]

    def _require_ready(self) -> None:
        if self.grid_map is None or self.fleet is None or self.scheduler is None or self.simulator is None:
            raise RuntimeError("simulation runtime is not initialized")


def _normalize_event_data(event_type: EventType, data: dict[str, Any]) -> dict[str, Any]:
    if event_type != EventType.MAP_UPDATE:
        return dict(data)
    operation = str(data.get("operation", "")).upper()
    if operation in {"ADD_OBSTACLE", "ADD_OBSTACLES"}:
        return {
            "operation": "SET_REGION",
            "shape": "rectangle",
            "frame": "grid",
            "x": int(data["x"]),
            "y": int(data["y"]),
            "width": int(data.get("width", 1)),
            "height": int(data.get("height", 1)),
            "cell_type": CellType.OBSTACLE.value,
        }
    if operation in {"REMOVE_OBSTACLE", "CLEAR_REGION"}:
        return {
            "operation": "CLEAR_REGION",
            "shape": "rectangle",
            "frame": "grid",
            "x": int(data["x"]),
            "y": int(data["y"]),
            "width": int(data.get("width", 1)),
            "height": int(data.get("height", 1)),
        }
    if operation == "SET_CELL":
        normalized = dict(data)
        normalized.setdefault("cell_type", CellType.OBSTACLE.value)
        return normalized
    if operation == "SET_REGION":
        normalized = dict(data)
        normalized.setdefault("shape", "rectangle")
        normalized.setdefault("frame", "grid")
        normalized.setdefault("cell_type", CellType.OBSTACLE.value)
        return normalized
    if str(data.get("operation")) == "add_obstacle":
        return {
            "operation": "SET_REGION",
            "shape": "rectangle",
            "frame": "grid",
            "x": int(data["x"]),
            "y": int(data["y"]),
            "width": int(data.get("width", 1)),
            "height": int(data.get("height", 1)),
            "cell_type": CellType.OBSTACLE.value,
        }
    return dict(data)


def _merge_batch_snapshots_into_state(state: dict[str, Any], snapshots: list[dict[str, Any]]) -> None:
    if not snapshots:
        return
    state["commands"] = _dedupe_by_key(
        [command for snapshot in snapshots for command in snapshot.get("commands", [])],
        key="command_id",
    )
    state["command_acks"] = _dedupe_by_key(
        [ack for snapshot in snapshots for ack in snapshot.get("command_acks", [])],
        key="command_id",
        prefer_latest=True,
    )
    state["events"] = list(dict.fromkeys(event for snapshot in snapshots for event in snapshot.get("events", [])))
    coverage_cells = [cell for snapshot in snapshots for cell in snapshot.get("coverage_changed_cells", [])]
    if coverage_cells:
        state["coverage_changed_cells"] = _dedupe_cells(coverage_cells)


def _dedupe_by_key(items: list[dict[str, Any]], key: str, prefer_latest: bool = False) -> list[dict[str, Any]]:
    ordered: dict[str, dict[str, Any]] = {}
    for item in items:
        item_key = str(item.get(key))
        if item_key == "None":
            continue
        if prefer_latest or item_key not in ordered:
            ordered[item_key] = item
    return list(ordered.values())


def _dedupe_cells(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    ordered: dict[tuple[int, int], dict[str, Any]] = {}
    for item in items:
        ordered[(int(item["x"]), int(item["y"]))] = item
    return list(ordered.values())
