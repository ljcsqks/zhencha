from __future__ import annotations

import asyncio
from pathlib import Path
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
        self.reset(self.config_path, self.scenario_path)

    def set_state_callback(self, callback: StateCallback | None) -> None:
        self._state_callback = callback

    def reset(self, config_path: str | Path | None = None, scenario_path: str | Path | None = None) -> dict[str, Any]:
        self.pause()
        self.config_path = str(config_path or self.config_path)
        self.scenario_path = str(scenario_path or self.scenario_path)
        self.config = load_config(self.config_path, self.scenario_path)
        validate_config(self.config)
        self.scenario = self.config.get("scenario") or load_yaml(self.scenario_path)
        self.grid_map = build_grid_map(self.config)
        self.fleet = FleetManager.from_config(self.config, self.scenario)
        self.scheduler = Scheduler(self.grid_map, self.fleet, self.config)
        self.simulator = Simulator(self.grid_map, self.fleet, self.config)
        self.event_injector = ScenarioEventInjector(self.scenario.get("events", []))
        return self.get_state(include_map=True)

    def step(self, steps: int = 1) -> dict[str, Any]:
        bounded_steps = max(1, min(int(steps), 100))
        for _ in range(bounded_steps):
            self._require_ready()
            self.simulator.tick(scheduler=self.scheduler, event_injector=self.event_injector)
        return self.get_state(include_map=True)

    def start(self, tick_interval_ms: int = 100) -> dict[str, Any]:
        if self.running:
            return self.get_state(include_map=False)
        self.running = True
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return self.get_state(include_map=False)
        self._loop_task = loop.create_task(self._run_loop(max(10, int(tick_interval_ms)) / 1000.0))
        return self.get_state(include_map=False)

    def pause(self) -> dict[str, Any]:
        self.running = False
        if self._loop_task is not None and not self._loop_task.done():
            self._loop_task.cancel()
        self._loop_task = None
        if self.simulator is None:
            return {}
        return self.get_state(include_map=False)

    def enqueue_event(self, event_request: EventRequest) -> dict[str, Any]:
        self._require_ready()
        self.simulator.enqueue_event(self._event_from_request(event_request))
        return self.get_state(include_map=False)

    def get_state(self, include_map: bool = True) -> dict[str, Any]:
        self._require_ready()
        return build_state(
            simulator=self.simulator,
            grid_map=self.grid_map,
            fleet=self.fleet,
            scheduler=self.scheduler,
            scenario_name=str(self.scenario.get("name", Path(self.scenario_path).stem)),
            running=self.running,
            include_map=include_map,
        )

    def get_metrics(self) -> dict[str, Any]:
        self._require_ready()
        if not self.simulator.snapshots:
            return self.get_state(include_map=False)["metrics"]
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
                state = self.step(1)
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
