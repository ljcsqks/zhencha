from __future__ import annotations

import math

from uav_search.core.data_types import Position, UAVState, UAVStatus


class UAV:
    def __init__(self, state: UAVState, endurance_s: float) -> None:
        if endurance_s <= 0:
            raise ValueError("endurance_s must be greater than 0")
        self.state = state
        self.endurance_s = float(endurance_s)
        self._movement_carry_m = 0.0

    def assign_path(self, path: list[Position], status: UAVStatus = UAVStatus.SEARCHING) -> None:
        self.state.path = path
        self.state.path_index = 0
        self.state.status = status
        self.state.available = status == UAVStatus.IDLE

    def move_along_path(self, time_step_s: float, resolution_m: float) -> float:
        if self.state.status == UAVStatus.OFFLINE or not self.state.path:
            return 0.0
        if self.state.path_index >= len(self.state.path) - 1:
            self.state.position = self.state.path[-1]
            self._finish_completed_path()
            return 0.0

        movement_budget_m = self._movement_carry_m + self.state.velocity_mps * time_step_s
        traveled_m = 0.0
        while self.state.path_index < len(self.state.path) - 1:
            current = self.state.path[self.state.path_index]
            next_pos = self.state.path[self.state.path_index + 1]
            step_distance_m = _grid_distance(current, next_pos) * resolution_m
            if traveled_m + step_distance_m > movement_budget_m:
                break
            self.state.path_index += 1
            self.state.position = next_pos
            traveled_m += step_distance_m

        self._movement_carry_m = max(0.0, movement_budget_m - traveled_m)
        if traveled_m > 0:
            self.consume_battery(traveled_m)
            self.state.total_distance_m += traveled_m
            if self.state.status == UAVStatus.SEARCHING:
                self.state.effective_search_distance_m += traveled_m
            self._finish_completed_path()
        return traveled_m

    def _finish_completed_path(self) -> None:
        if self.state.path_index < len(self.state.path) - 1:
            return
        if self.state.status == UAVStatus.SEARCHING:
            self.state.status = UAVStatus.IDLE
            self.state.available = True
        elif self.state.status == UAVStatus.RETURNING:
            # Returning is a terminal transit state; once home is reached the UAV can accept future missions.
            self.state.status = UAVStatus.IDLE
            self.state.available = True
            self.state.current_task_id = None

    def consume_battery(self, distance_m: float) -> None:
        if self.state.velocity_mps <= 0:
            return
        duration_s = distance_m / self.state.velocity_mps
        self.state.battery = max(0.0, self.state.battery - duration_s / self.endurance_s)

    def can_reach_and_return(self, target: Position, home: Position, resolution_m: float, reserve: float) -> bool:
        required_m = (_grid_distance(self.state.position, target) + _grid_distance(target, home)) * resolution_m
        required_ratio = (required_m / self.state.velocity_mps) / self.endurance_s
        return self.state.battery - required_ratio >= reserve

    def is_low_battery(self, threshold: float) -> bool:
        return self.state.battery <= threshold


def _grid_distance(a: Position, b: Position) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
