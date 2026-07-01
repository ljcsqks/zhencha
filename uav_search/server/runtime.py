from __future__ import annotations

import asyncio
import json
import shutil
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from threading import RLock
from typing import Any, Awaitable, Callable
from uuid import uuid4

from uav_search.core.config import load_config, load_yaml, validate_config
from uav_search.core.data_types import CellType, Event, EventPriority, EventType
from uav_search.core.scheduler import Scheduler
from uav_search.evaluation.metrics import compute_metrics
from uav_search.maps.map_loader import build_grid_map
from uav_search.server.algorithms import algorithms_payload, validate_algorithm_version
from uav_search.server.schemas import EventRequest, MissionDraft
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

    def reset(
        self,
        config_path: str | Path | None = None,
        scenario_path: str | Path | None = None,
        algorithm_version: str | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            self._pause_unlocked()
            algorithm_override = validate_algorithm_version(algorithm_version)
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
            if algorithm_override:
                self.config.setdefault("algorithm", {})["version"] = algorithm_override
            validate_config(self.config)
            self.scenario = self.config.get("scenario") or load_yaml(self.scenario_path)
            self.grid_map = build_grid_map(self.config)
            self.fleet = FleetManager.from_config(self.config, self.scenario)
            self.scheduler = Scheduler(self.grid_map, self.fleet, self.config)
            self.simulator = Simulator(self.grid_map, self.fleet, self.config)
            self.event_injector = ScenarioEventInjector(self.scenario.get("events", []))
            return self.get_state(include_map=True, state_level="full")

    def reset_custom(
        self,
        config_path: str | Path | None = None,
        scenario_path: str | Path | None = None,
        mission: MissionDraft | None = None,
        algorithm_version: str | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            self._pause_unlocked()
            algorithm_override = validate_algorithm_version(algorithm_version)
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
            self.config, self.scenario = _config_with_mission(next_config_path, next_scenario_path, mission or MissionDraft())
            if algorithm_override:
                self.config.setdefault("algorithm", {})["version"] = algorithm_override
            validate_config(self.config)
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
                config=self.config,
            )
            return metrics.__dict__.copy()

    def export_run(self) -> dict[str, Any]:
        with self._lock:
            self._require_ready()
            if not self.simulator.snapshots:
                raise RuntimeError("cannot export run without snapshots")

            export_root = Path("runs") / "web_exports"
            export_dir = export_root / self.run_id
            export_dir.mkdir(parents=True, exist_ok=True)

            metrics = self.get_metrics()
            final_state = self.get_state(include_map=True, state_level="full")
            summary = _export_summary(
                run_id=self.run_id,
                scenario_name=str(self.scenario.get("name", Path(self.scenario_path).stem)),
                metrics=metrics,
                final_state=final_state,
            )
            files: dict[str, Any] = {
                "snapshots.json": {
                    "run_id": self.run_id,
                    "scenario_name": str(self.scenario.get("name", Path(self.scenario_path).stem)),
                    "algorithm_version": self.config.get("algorithm", {}).get("version"),
                    "summary": summary,
                    "map": final_state.get("map"),
                    "steps": self.simulator.snapshots,
                },
                "metrics.json": metrics,
                "final_state.json": final_state,
                "event_log.json": self._event_log,
                "command_log.json": _command_log_from_snapshots(self.simulator.snapshots),
                "summary.json": summary,
            }
            for filename, payload in files.items():
                _write_json(export_dir / filename, payload)

            shutil.copyfile(Path(self.scenario_path), export_dir / "scenario.yaml")
            shutil.copyfile(Path(self.config_path), export_dir / "config.yaml")

            return {
                "run_id": self.run_id,
                "export_dir": str(export_dir),
                "files": sorted([*files.keys(), "scenario.yaml", "config.yaml"]),
            }

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

    def get_algorithms(self) -> dict[str, Any]:
        return algorithms_payload()

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
        priority = EventPriority.CRITICAL if event_type in (EventType.TARGET_FOUND, EventType.BUILDING_MODEL_REQUEST) else EventPriority.HIGH
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


def _config_with_mission(
    config_path: str | Path,
    scenario_path: str | Path,
    mission: MissionDraft,
) -> tuple[dict[str, Any], dict[str, Any]]:
    config = load_config(config_path, scenario_path)
    scenario = deepcopy(config.get("scenario") or load_yaml(scenario_path))
    scenario["name"] = "mission_draft"
    scenario["description"] = "Mission draft generated by the Web simulation console."

    if mission.draft_map_config is not None:
        map_config = mission.draft_map_config
        resolution_m = float(map_config.resolution_m or config["map"]["resolution_m"])
        width_m = (
            float(map_config.width_m)
            if map_config.width_m is not None
            else float(map_config.width_cells or config["map"]["width_m"] / resolution_m) * resolution_m
        )
        height_m = (
            float(map_config.height_m)
            if map_config.height_m is not None
            else float(map_config.height_cells or config["map"]["height_m"] / resolution_m) * resolution_m
        )
        config.setdefault("map", {}).update(
            {
                "width_m": width_m,
                "height_m": height_m,
                "resolution_m": resolution_m,
            }
        )
        scenario.setdefault("overrides", {}).setdefault("map", {}).update(config["map"])

    map_features = deepcopy(scenario.get("map_features", {}))
    map_features["obstacles"] = [_draft_rect_to_feature(item, f"draft_obstacle_{idx}") for idx, item in enumerate(mission.draft_obstacles, start=1)]
    map_features["priority_zones"] = [
        {**_draft_rect_to_feature(item, f"draft_priority_{idx}"), "priority": float(item.priority)}
        for idx, item in enumerate(mission.draft_priority_regions, start=1)
    ]
    map_features.setdefault("no_fly_zones", [])
    if mission.draft_search_region is not None:
        map_features["obstacles"].extend(_outside_search_region_obstacles(config, mission.draft_search_region))
    scenario["map_features"] = map_features

    if mission.draft_uavs:
        scenario["uavs"] = [_draft_uav_to_scenario(item, idx) for idx, item in enumerate(mission.draft_uavs, start=1)]
        config.setdefault("uav", {})["count"] = len(scenario["uavs"])
        scenario.setdefault("overrides", {}).setdefault("uav", {})["count"] = len(scenario["uavs"])

    config["scenario"] = scenario
    return config, scenario


def _draft_rect_to_feature(rect, fallback_id: str) -> dict[str, Any]:
    return {
        "id": rect.id or fallback_id,
        "shape": "rectangle",
        "frame": "grid",
        "x": int(rect.x),
        "y": int(rect.y),
        "width": int(rect.width),
        "height": int(rect.height),
    }


def _outside_search_region_obstacles(config: dict[str, Any], region) -> list[dict[str, Any]]:
    resolution_m = float(config["map"]["resolution_m"])
    width_cells = int(round(float(config["map"]["width_m"]) / resolution_m))
    height_cells = int(round(float(config["map"]["height_m"]) / resolution_m))
    x0 = max(0, min(width_cells, int(region.x)))
    y0 = max(0, min(height_cells, int(region.y)))
    x1 = max(x0, min(width_cells, x0 + int(region.width)))
    y1 = max(y0, min(height_cells, y0 + int(region.height)))
    bands = [
        ("draft_search_north", 0, 0, width_cells, y0),
        ("draft_search_south", 0, y1, width_cells, height_cells - y1),
        ("draft_search_west", 0, y0, x0, y1 - y0),
        ("draft_search_east", x1, y0, width_cells - x1, y1 - y0),
    ]
    return [
        {
            "id": feature_id,
            "shape": "rectangle",
            "frame": "grid",
            "x": x,
            "y": y,
            "width": width,
            "height": height,
        }
        for feature_id, x, y, width, height in bands
        if width > 0 and height > 0
    ]


def _draft_uav_to_scenario(uav, index: int) -> dict[str, Any]:
    initial = uav.initial_position or uav.home_position
    return {
        "id": uav.id or f"uav_{index:02d}",
        "home_position": [int(uav.home_position.x), int(uav.home_position.y)],
        "initial_position": [int(initial.x), int(initial.y)],
        "sensor_radius_cells": int(uav.sensor_radius_cells),
        "speed_mps": float(uav.speed_mps),
        "battery": float(uav.battery),
    }


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


def _write_json(path: Path, payload: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def _command_log_from_snapshots(snapshots: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for snapshot in snapshots:
        time_s = snapshot.get("time_s")
        for command in snapshot.get("commands", []):
            rows.append({"kind": "command", "time_s": time_s, **dict(command)})
        for ack in snapshot.get("command_acks", []):
            rows.append({"kind": "ack", "time_s": time_s, **dict(ack)})
    return rows


def _export_summary(
    *,
    run_id: str,
    scenario_name: str,
    metrics: dict[str, Any],
    final_state: dict[str, Any],
) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "scenario_name": scenario_name,
        "final_time_s": metrics.get("final_time_s", final_state.get("time_s")),
        "final_coverage": metrics.get("global_coverage", final_state.get("global_coverage")),
        "priority_coverage": metrics.get("priority_coverage", final_state.get("priority_coverage")),
        "time_to_95_coverage_s": metrics.get("time_to_95_coverage_s"),
        "total_distance_m": metrics.get("total_distance_m"),
        "redundant_coverage_rate": metrics.get("redundant_coverage_rate"),
        "no_fly_violations": metrics.get("no_fly_violations"),
        "target_found_count": metrics.get("target_found_count"),
        "confirm_done_count": metrics.get("confirm_done_count"),
        "confirm_success_rate": metrics.get("confirm_success_rate"),
        "interrupted_task_resume_rate": metrics.get("interrupted_task_resume_rate"),
        "algorithm_version": metrics.get("algorithm_version"),
        "code_version": metrics.get("code_version"),
        "config_hash": metrics.get("config_hash"),
        "diagnostics": metrics.get("diagnostics", {}),
        "workload_balance": _nested_metric(metrics, ["diagnostics", "allocation_quality", "workload_balance"], metrics.get("per_uav_workload_balance")),
        "workload_balance_all_uavs": _nested_metric(metrics, ["diagnostics", "allocation_quality", "workload_balance_all_uavs"], metrics.get("per_uav_workload_balance")),
        "workload_balance_active_uavs": _nested_metric(metrics, ["diagnostics", "allocation_quality", "workload_balance_active_uavs"], metrics.get("per_uav_workload_balance")),
        "fleet_idle_time_ratio": _nested_metric(metrics, ["diagnostics", "allocation_quality", "fleet_idle_time_ratio"], 0.0),
        "idle_assist_attempts": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_attempts"], 0),
        "idle_assist_created_tasks": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_created_tasks"], 0),
        "idle_assist_accepted_tasks": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_accepted_tasks"], 0),
        "idle_assist_rejected_low_gain": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_rejected_low_gain"], 0),
        "idle_assist_rejected_unreachable": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_rejected_unreachable"], 0),
        "idle_assist_donor_replans": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_donor_replans"], 0),
        "idle_uav_wait_time_s": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_uav_wait_time_s"], 0),
        "idle_assist_cells_reassigned": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_cells_reassigned"], 0),
        "idle_assist_distance_m": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_assist_distance_m"], 0),
        "idle_reason_per_uav": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "idle_reason_per_uav"], {}),
        "dynamic_route_repair_attempts": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "dynamic_route_repair_attempts"], 0),
        "dynamic_route_repair_success": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "dynamic_route_repair_success"], 0),
        "dynamic_route_repair_dropped_waypoints": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "dynamic_route_repair_dropped_waypoints"], 0),
        "dynamic_route_repair_replanned_tasks": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "dynamic_route_repair_replanned_tasks"], 0),
        "dynamic_route_repair_fallback_to_supplemental": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "dynamic_route_repair_fallback_to_supplemental"], 0),
        "modeling_jobs_total": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_jobs_total"], 0),
        "modeling_jobs_completed": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_jobs_completed"], 0),
        "modeling_jobs_failed": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_jobs_failed"], 0),
        "modeling_active_jobs": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_active_jobs"], 0),
        "modeling_assigned_uav_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_assigned_uav_count"], 0),
        "modeling_facade_lane_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_facade_lane_count"], 0),
        "modeling_facade_progress_ratio": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_facade_progress_ratio"], 0),
        "modeling_distance_m": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_distance_m"], 0),
        "modeling_interrupted_search_tasks": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_interrupted_search_tasks"], 0),
        "modeling_resumed_search_tasks": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_resumed_search_tasks"], 0),
        "modeling_unreachable_facade_lanes": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_unreachable_facade_lanes"], 0),
        "modeling_no_fly_violations": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_no_fly_violations"], 0),
        "modeling_return_home_commands": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_return_home_commands"], 0),
        "modeling_hold_after_done_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_hold_after_done_count"], 0),
        "modeling_no_resume_return_home_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_no_resume_return_home_count"], 0),
        "modeling_completed_without_interrupted_search_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_completed_without_interrupted_search_count"], 0),
        "modeling_uav_stuck_modeling_count": _nested_metric(metrics, ["diagnostics", "scheduler_quality", "modeling_uav_stuck_modeling_count"], 0),
        "post_95_extra_distance_m": metrics.get("post_95_extra_distance_m"),
        "post_95_search_distance_m": _nested_metric(metrics, ["diagnostics", "coverage_quality", "post_95_search_distance_m"], 0.0),
        "post_95_return_distance_m": _nested_metric(metrics, ["diagnostics", "coverage_quality", "post_95_return_distance_m"], 0.0),
        "max_connector_length": _nested_metric(metrics, ["diagnostics", "route_quality", "max_connector_length"], 0.0),
        "long_logical_connector_count": _nested_metric(metrics, ["diagnostics", "route_quality", "long_logical_connector_count"], 0),
        "max_logical_connector_length": _nested_metric(metrics, ["diagnostics", "route_quality", "max_logical_connector_length"], 0.0),
        "unreachable_cells_count": _nested_metric(metrics, ["diagnostics", "coverage_quality", "unreachable_cells_count"], 0),
        "unreachable_components_count": _nested_metric(metrics, ["diagnostics", "coverage_quality", "unreachable_components_count"], 0),
        "segment_count_total": _nested_metric(metrics, ["diagnostics", "segment_quality", "segment_count_total"], 0),
        "unique_segment_count": _nested_metric(metrics, ["diagnostics", "segment_quality", "unique_segment_count"], 0),
        "segment_workload_balance": _nested_metric(metrics, ["diagnostics", "segment_quality", "segment_workload_balance"], 1.0),
        "clustered_launch_detected": _nested_metric(metrics, ["diagnostics", "segment_quality", "clustered_launch_detected"], False),
        "clustered_sector_orientation": _nested_metric(metrics, ["diagnostics", "segment_quality", "clustered_sector_orientation"], ""),
        "clustered_sector_entry_side": _nested_metric(metrics, ["diagnostics", "segment_quality", "clustered_sector_entry_side"], ""),
        "clustered_sector_count": _nested_metric(metrics, ["diagnostics", "segment_quality", "clustered_sector_count"], 0),
        "clustered_sector_workload_balance": _nested_metric(metrics, ["diagnostics", "segment_quality", "clustered_sector_workload_balance"], 1.0),
        "launch_profile": _nested_metric(metrics, ["diagnostics", "segment_quality", "launch_profile"], ""),
        "launch_entry_side": _nested_metric(metrics, ["diagnostics", "segment_quality", "launch_entry_side"], ""),
        "common_edge_staging_detected": _nested_metric(metrics, ["diagnostics", "segment_quality", "common_edge_staging_detected"], False),
        "sector_balance_score": _nested_metric(metrics, ["diagnostics", "segment_quality", "sector_balance_score"], 1.0),
        "planned_coverage_ratio": _nested_metric(metrics, ["diagnostics", "segment_quality", "fleet_planned_coverage_ratio"], 0.0),
        "planned_actual_gap_abs": _nested_metric(metrics, ["diagnostics", "segment_quality", "planned_actual_gap_abs"], 0.0),
        "planned_vs_actual_explanation": _nested_metric(
            metrics,
            ["diagnostics", "segment_quality", "planned_vs_actual_explanation"],
            "planned coverage unavailable",
        ),
        "estimated_connector_cost_per_uav": _nested_metric(metrics, ["diagnostics", "segment_quality", "estimated_connector_cost_per_uav"], {}),
        "idle_time_ratio": _idle_time_ratio(metrics.get("diagnostics", {})),
        "exported_at": datetime.now(timezone.utc).isoformat(),
    }


def _nested_metric(payload: dict[str, Any], path: list[str], default: Any = None) -> Any:
    current: Any = payload
    for key in path:
        if not isinstance(current, dict):
            return default
        current = current.get(key)
    return default if current is None else current


def _idle_time_ratio(diagnostics: Any) -> float:
    if not isinstance(diagnostics, dict):
        return 0.0
    per_uav = diagnostics.get("per_uav", {})
    if not isinstance(per_uav, dict):
        return 0.0
    idle = sum(float(item.get("idle_time_s", 0.0)) for item in per_uav.values() if isinstance(item, dict))
    active = sum(float(item.get("active_time_s", 0.0)) for item in per_uav.values() if isinstance(item, dict))
    total = idle + active
    return idle / total if total > 0 else 0.0
