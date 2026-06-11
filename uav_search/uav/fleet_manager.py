from __future__ import annotations

from collections.abc import Iterable

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.uav.uav_model import UAV


class FleetManager:
    def __init__(self, uavs: Iterable[UAV]) -> None:
        self._uavs = {uav.state.id: uav for uav in uavs}

    @classmethod
    def from_config(cls, config: dict, scenario: dict | None = None) -> "FleetManager":
        uav_config = config["uav"]
        scenario_uavs = (scenario or {}).get("uavs", [])
        uavs: list[UAV] = []

        if scenario_uavs:
            for item in scenario_uavs:
                home = Position(int(item["home_position"][0]), int(item["home_position"][1]))
                initial = Position(int(item["initial_position"][0]), int(item["initial_position"][1]))
                state = UAVState(
                    id=str(item["id"]),
                    position=initial,
                    velocity_mps=float(uav_config["max_speed_mps"]),
                    heading_deg=0.0,
                    battery=float(item.get("battery", 1.0)),
                    sensor_radius_cells=int(uav_config["sensor_radius_cells"]),
                    status=UAVStatus.IDLE,
                    home_position=home,
                )
                uavs.append(UAV(state, endurance_s=float(uav_config["endurance_s"])))
        else:
            home = Position(int(uav_config["home_position"][0]), int(uav_config["home_position"][1]))
            for idx in range(int(uav_config["count"])):
                state = UAVState(
                    id=f"uav_{idx + 1:02d}",
                    position=home,
                    velocity_mps=float(uav_config["max_speed_mps"]),
                    heading_deg=0.0,
                    battery=1.0,
                    sensor_radius_cells=int(uav_config["sensor_radius_cells"]),
                    status=UAVStatus.IDLE,
                    home_position=home,
                )
                uavs.append(UAV(state, endurance_s=float(uav_config["endurance_s"])))

        return cls(uavs)

    def get_uav(self, uav_id: str) -> UAV:
        return self._uavs[uav_id]

    def get_all_uavs(self) -> list[UAV]:
        return list(self._uavs.values())

    def get_all_states(self) -> list[UAVState]:
        return [uav.state for uav in self._uavs.values()]

    def get_available_uavs(self) -> list[UAVState]:
        return [uav.state for uav in self._uavs.values() if uav.state.available and uav.state.status == UAVStatus.IDLE]

    def assign_path(self, uav_id: str, path: list[Position], status: UAVStatus = UAVStatus.SEARCHING) -> None:
        self.get_uav(uav_id).assign_path(path, status=status)

    def set_status(self, uav_id: str, status: UAVStatus) -> None:
        state = self.get_uav(uav_id).state
        state.status = status
        state.available = status != UAVStatus.OFFLINE

    def step(self, time_step_s: float, resolution_m: float) -> None:
        for uav in self._uavs.values():
            uav.move_along_path(time_step_s, resolution_m)
