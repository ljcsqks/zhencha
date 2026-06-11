from __future__ import annotations

from uav_search.core.data_types import Task, UAVState
from uav_search.maps.grid_map import GridMap


def calculate_bid(uav: UAVState, task: Task, grid_map: GridMap, config: dict) -> float | None:
    battery_threshold = float(config.get("battery_threshold", 0.2))
    if uav.battery <= battery_threshold:
        return None

    auction_config = config.get("auction", {})
    w_distance = float(auction_config.get("w_distance", 1.0))
    w_battery = float(auction_config.get("w_battery", 0.3))
    w_priority = float(auction_config.get("w_priority", 0.5))
    w_balance = float(auction_config.get("w_balance", 0.2))

    distance_cells = abs(uav.position.x - task.entry_point.x) + abs(uav.position.y - task.entry_point.y)
    distance_cost = distance_cells * grid_map.resolution_m
    estimated_distance_m = distance_cost + task.estimated_cost_m
    battery_cost = estimated_distance_m / max(uav.battery, 0.01)
    priority_bonus = -task.priority
    load_balance_penalty = uav.assigned_task_count

    return (
        w_distance * distance_cost
        + w_battery * battery_cost
        + w_priority * priority_bonus
        + w_balance * load_balance_penalty
    )
