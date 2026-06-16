from __future__ import annotations

import math
from typing import Any

from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap


def build_grid_map(config: dict[str, Any]) -> GridMap:
    map_config = config["map"]
    grid_map = GridMap(
        width_m=map_config["width_m"],
        height_m=map_config["height_m"],
        resolution_m=map_config["resolution_m"],
    )

    scenario = config.get("scenario", {})
    features = scenario.get("map_features", {})
    for obstacle in features.get("obstacles", []):
        _apply_feature(grid_map, obstacle, CellType.OBSTACLE, search_priority=1.0)
    for no_fly_zone in features.get("no_fly_zones", []):
        _apply_feature(grid_map, no_fly_zone, CellType.NO_FLY, search_priority=1.0)
    for priority_zone in features.get("priority_zones", []):
        _apply_feature(
            grid_map,
            priority_zone,
            CellType.PRIORITY,
            search_priority=float(priority_zone.get("priority", 2.0)),
        )

    return grid_map


def _apply_feature(
    grid_map: GridMap,
    feature: dict[str, Any],
    cell_type: CellType,
    search_priority: float,
) -> None:
    shape = feature.get("shape")
    if shape == "rectangle":
        cells = _rectangle_cells(grid_map, feature)
    elif shape == "circle":
        cells = _circle_cells(grid_map, feature)
    elif shape == "polygon":
        cells = _polygon_cells(grid_map, feature)
    else:
        raise ValueError(f"Unsupported map feature shape: {shape}")

    grid_map.update_region(
        cells,
        {
            "cell_type": cell_type,
            "search_priority": search_priority,
        },
    )


def _rectangle_cells(grid_map: GridMap, feature: dict[str, Any]) -> list[Position]:
    frame = feature.get("frame", "world")
    if frame == "world":
        start = grid_map.world_to_grid(float(feature["x_m"]), float(feature["y_m"]))
        end = grid_map.world_to_grid(
            float(feature["x_m"]) + float(feature["width_m"]),
            float(feature["y_m"]) + float(feature["height_m"]),
        )
    elif frame == "grid":
        start = Position(int(feature["x"]), int(feature["y"]))
        end = Position(int(feature["x"]) + int(feature["width"]), int(feature["y"]) + int(feature["height"]))
    else:
        raise ValueError(f"Unsupported frame: {frame}")

    cells: list[Position] = []
    for y in range(start.y, end.y):
        for x in range(start.x, end.x):
            pos = Position(x, y)
            if grid_map.in_bounds(pos):
                cells.append(pos)
    return cells


def _circle_cells(grid_map: GridMap, feature: dict[str, Any]) -> list[Position]:
    frame = feature.get("frame", "world")
    if frame == "world":
        center = grid_map.world_to_grid(float(feature["center_x_m"]), float(feature["center_y_m"]))
        radius_cells = int(math.ceil(float(feature["radius_m"]) / grid_map.resolution_m))
    elif frame == "grid":
        center = Position(int(feature["center_x"]), int(feature["center_y"]))
        radius_cells = int(feature["radius"])
    else:
        raise ValueError(f"Unsupported frame: {frame}")

    cells: list[Position] = []
    radius_sq = radius_cells * radius_cells
    for y in range(center.y - radius_cells, center.y + radius_cells + 1):
        for x in range(center.x - radius_cells, center.x + radius_cells + 1):
            pos = Position(x, y)
            if not grid_map.in_bounds(pos):
                continue
            if (x - center.x) ** 2 + (y - center.y) ** 2 <= radius_sq:
                cells.append(pos)
    return cells


def _polygon_cells(grid_map: GridMap, feature: dict[str, Any]) -> list[Position]:
    frame = feature.get("frame", "world")
    if frame == "world":
        points = [grid_map.world_to_grid(float(x), float(y)) for x, y in feature["points_m"]]
    elif frame == "grid":
        points = [Position(int(x), int(y)) for x, y in feature["points"]]
    else:
        raise ValueError(f"Unsupported frame: {frame}")

    min_x = max(min(point.x for point in points), 0)
    max_x = min(max(point.x for point in points), grid_map.width_cells - 1)
    min_y = max(min(point.y for point in points), 0)
    max_y = min(max(point.y for point in points), grid_map.height_cells - 1)

    cells: list[Position] = []
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            pos = Position(x, y)
            if _point_in_polygon(pos, points):
                cells.append(pos)
    return cells


def _point_in_polygon(pos: Position, polygon: list[Position]) -> bool:
    inside = False
    j = len(polygon) - 1
    px = pos.x + 0.5
    py = pos.y + 0.5
    for i, point_i in enumerate(polygon):
        point_j = polygon[j]
        if ((point_i.y > py) != (point_j.y > py)) and (
            px < (point_j.x - point_i.x) * (py - point_i.y) / ((point_j.y - point_i.y) or 1e-9) + point_i.x
        ):
            inside = not inside
        j = i
    return inside
