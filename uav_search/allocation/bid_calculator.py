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
    w_balance = float(auction_config.get("w_balance", 0.2))
    search_config = config.get("search", {})
    distance_cost_weight = float(search_config.get("distance_cost_weight", 1.0))
    value_weight = float(search_config.get("uncovered_value_weight", 1.0))
    priority_value_weight = float(search_config.get("priority_value_weight", 2.0))
    redundant_penalty_weight = float(search_config.get("redundant_penalty_weight", 0.5))

    distance_cells = abs(uav.position.x - task.entry_point.x) + abs(uav.position.y - task.entry_point.y)
    distance_cost = distance_cells * grid_map.resolution_m
    estimated_distance_m = distance_cost + task.estimated_cost_m
    battery_cost = estimated_distance_m / max(uav.battery, 0.01)
    load_balance_penalty = uav.assigned_task_count
    task_value = (
        value_weight * max(task.uncovered_value, float(len(task.target_cells)))
        + priority_value_weight * task.priority_value
        + max(task.priority - 1.0, 0.0) * 10.0
    )
    redundant_penalty = redundant_penalty_weight * max(0.0, 1.0 - task.progress)
    cost = (
        distance_cost_weight * (w_distance * distance_cost + task.estimated_cost_m)
        + w_battery * battery_cost
        + w_balance * load_balance_penalty
        + redundant_penalty
    )

    return cost / max(task_value, 1.0)
