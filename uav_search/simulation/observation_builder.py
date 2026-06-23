from __future__ import annotations

from typing import Any

from uav_search.core.contracts import CommandAck, MapCellObservation, MapObservation, MissionSpec, Observation, UAVObservation
from uav_search.core.data_types import Event, Position
from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager


class ObservationBuilder:
    def __init__(self, grid_map: GridMap, fleet: FleetManager, config: dict[str, Any], mission_id: str = "manual_run") -> None:
        self.grid_map = grid_map
        self.fleet = fleet
        self.config = config
        self.mission_id = mission_id

    def build(
        self,
        tick: int,
        time_s: float,
        changed_cells: list[Position] | None = None,
        events: list[Event] | None = None,
        command_acks: list[CommandAck] | None = None,
        active_command_ids: dict[str, str] | None = None,
    ) -> Observation:
        return Observation(
            tick=tick,
            time_s=time_s,
            mission_id=self.mission_id,
            mission=self._mission_spec(),
            map=self._map_observation(),
            changed_cells=list(changed_cells or []),
            uavs=self._uav_observations(active_command_ids or {}),
            events=list(events or []),
            command_acks=list(command_acks or []),
            metrics_hint={
                "global_coverage": self.grid_map.coverage_rate(),
                "priority_coverage": self.grid_map.coverage_rate(priority_only=True),
            },
        )

    def _mission_spec(self) -> MissionSpec:
        return MissionSpec(
            mission_id=self.mission_id,
            width_cells=self.grid_map.width_cells,
            height_cells=self.grid_map.height_cells,
            resolution_m=self.grid_map.resolution_m,
            mission_complete_coverage_threshold=float(
                self.config["search"].get("mission_complete_coverage_threshold", 0.95)
            ),
            sensor_radius_cells=int(self.config["uav"]["sensor_radius_cells"]),
        )

    def _map_observation(self) -> MapObservation:
        cells: list[list[MapCellObservation]] = []
        for y in range(self.grid_map.height_cells):
            row: list[MapCellObservation] = []
            for x in range(self.grid_map.width_cells):
                row.append(
                    MapCellObservation(
                        cell_type=str(self.grid_map.terrain[y, x]),
                        passable=bool(self.grid_map.passable[y, x]),
                        search_confidence=float(self.grid_map.search_confidence[y, x]),
                        search_priority=float(self.grid_map.search_priority[y, x]),
                        coverage_count=int(self.grid_map.coverage_count[y, x]),
                    )
                )
            cells.append(row)
        return MapObservation(
            width_cells=self.grid_map.width_cells,
            height_cells=self.grid_map.height_cells,
            resolution_m=self.grid_map.resolution_m,
            cells=cells,
        )

    def _uav_observations(self, active_command_ids: dict[str, str]) -> list[UAVObservation]:
        observations: list[UAVObservation] = []
        for state in self.fleet.get_all_states():
            remaining = state.path[state.path_index :] if state.path else []
            observations.append(
                UAVObservation(
                    uav_id=state.id,
                    position=state.position,
                    status=state.status,
                    battery=state.battery,
                    home=state.home_position,
                    current_command_id=active_command_ids.get(state.id),
                    current_task_id=state.current_task_id,
                    remaining_path=list(remaining),
                )
            )
        return observations
