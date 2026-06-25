from __future__ import annotations

from uav_search.core.data_types import Task, UAVState
from uav_search.maps.grid_map import GridMap
from uav_search.planning.astar import AStarConfig, astar_search, path_cost


def calculate_bid(uav: UAVState, task: Task, grid_map: GridMap, config: dict) -> float | None:
    if task.allowed_uav_ids is not None and uav.id not in task.allowed_uav_ids:
        return None

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

    distance_cost = _entry_distance_m(uav, task, grid_map, config)
    if distance_cost is None:
        return None
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


def _entry_distance_m(uav: UAVState, task: Task, grid_map: GridMap, config: dict) -> float | None:
    auction_config = config.get("auction", {})
    if bool(auction_config.get("use_astar_for_bid", False)):
        planning_config = config.get("planning", {})
        astar_config = AStarConfig(
            obstacle_proximity_penalty=float(planning_config.get("obstacle_proximity_penalty", 0.5)),
            priority_area_bonus=float(planning_config.get("priority_area_bonus", -0.2)),
        )
        path = astar_search(grid_map, uav.position, task.entry_point, astar_config)
        if path is None:
            return None
        return path_cost(path) * grid_map.resolution_m

    distance_cells = abs(uav.position.x - task.entry_point.x) + abs(uav.position.y - task.entry_point.y)
    return distance_cells * grid_map.resolution_m
