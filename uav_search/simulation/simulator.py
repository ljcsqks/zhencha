from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from uav_search.core.data_types import UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.maps.grid_map import GridMap
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.uav.fleet_manager import FleetManager


class Simulator:
    def __init__(self, grid_map: GridMap, fleet: FleetManager, config: dict[str, Any]) -> None:
        self.grid_map = grid_map
        self.fleet = fleet
        self.config = config
        self.time_s = 0.0
        self.snapshots: list[dict[str, Any]] = []
        self._last_events: list[str] = []

    def step(self) -> None:
        time_step_s = float(self.config["simulation"]["time_step_s"])
        self.time_s += time_step_s
        self.fleet.step(time_step_s, self.grid_map.resolution_m)

        for state in self.fleet.get_all_states():
            if state.status != UAVStatus.OFFLINE:
                self.grid_map.mark_covered(state.position, state.sensor_radius_cells, self.time_s)

        self.record_snapshot()

    def run(
        self,
        max_steps: int | None = None,
        scheduler: Scheduler | None = None,
        event_injector: ScenarioEventInjector | None = None,
    ) -> None:
        steps = int(max_steps if max_steps is not None else self.config["simulation"]["max_steps"])
        for _ in range(steps):
            self._last_events = []
            if scheduler is not None and event_injector is not None:
                emitted = event_injector.emit_due(self.time_s, scheduler)
                if emitted:
                    decision = scheduler.regular_cycle(now=self.time_s)
                    self._last_events = decision.events_handled
            self.step()
            if all(state.status in (UAVStatus.IDLE, UAVStatus.OFFLINE) for state in self.fleet.get_all_states()):
                break

    def record_snapshot(self) -> None:
        self.snapshots.append(
            {
                "time_s": self.time_s,
                "global_coverage": self.grid_map.coverage_rate(),
                "priority_coverage": self.grid_map.coverage_rate(priority_only=True),
                "uavs": [
                    {
                        "id": state.id,
                        "position": asdict(state.position),
                        "status": state.status.value,
                        "battery": state.battery,
                        "task_id": state.current_task_id,
                    }
                    for state in self.fleet.get_all_states()
                ],
                "events": self._last_events,
            }
        )

    def save_snapshots(self, path: str | Path, run_id: str = "manual_run") -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"run_id": run_id, "steps": self.snapshots}
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
