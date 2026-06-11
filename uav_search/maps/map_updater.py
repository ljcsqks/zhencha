from __future__ import annotations

from typing import Any

from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap


class MapUpdater:
    """Apply runtime map updates and return affected cells for replanning checks."""

    def __init__(self, grid_map: GridMap) -> None:
        self.grid_map = grid_map

    def apply_update(self, update: dict[str, Any]) -> list[Position]:
        operation = update.get("operation")
        if operation == "SET_CELL":
            return self._set_cell(update)
        if operation == "SET_REGION":
            return self._set_region(update)
        if operation == "CLEAR_REGION":
            cleared = dict(update)
            cleared["cell_type"] = CellType.FREE.value
            return self._set_region(cleared)
        raise ValueError(f"Unsupported map update operation: {operation}")

    def apply_updates(self, updates: list[dict[str, Any]]) -> list[Position]:
        affected: list[Position] = []
        for update in updates:
            affected.extend(self.apply_update(update))
        return affected

    def _set_cell(self, update: dict[str, Any]) -> list[Position]:
        pos = _position_from_mapping(update["position"])
        attrs = _attrs_from_update(update)
        if not self.grid_map.in_bounds(pos):
            return []
        self.grid_map.set_cell(pos, attrs)
        return [pos]

    def _set_region(self, update: dict[str, Any]) -> list[Position]:
        cells = _region_cells(self.grid_map, update)
        attrs = _attrs_from_update(update)
        return self.grid_map.update_region(cells, attrs)


def _attrs_from_update(update: dict[str, Any]) -> dict[str, Any]:
    cell_type = update.get("cell_type")
    attrs: dict[str, Any] = {}
    if cell_type is not None:
        attrs["cell_type"] = CellType(cell_type) if isinstance(cell_type, str) else cell_type
    if "search_priority" in update:
        attrs["search_priority"] = float(update["search_priority"])
    if "passable" in update:
        attrs["passable"] = bool(update["passable"])
    return attrs


def _region_cells(grid_map: GridMap, update: dict[str, Any]) -> list[Position]:
    shape = update.get("shape", update.get("region_type"))
    if shape != "rectangle":
        raise ValueError(f"Unsupported update region shape: {shape}")

    frame = update.get("frame", "grid")
    if frame == "world":
        start = grid_map.world_to_grid(float(update["x_m"]), float(update["y_m"]))
        end = grid_map.world_to_grid(
            float(update["x_m"]) + float(update["width_m"]),
            float(update["y_m"]) + float(update["height_m"]),
        )
    else:
        if "points" in update:
            first = _position_from_mapping(update["points"][0])
            second = _position_from_mapping(update["points"][1])
            start = Position(min(first.x, second.x), min(first.y, second.y))
            end = Position(max(first.x, second.x) + 1, max(first.y, second.y) + 1)
        else:
            start = Position(int(update["x"]), int(update["y"]))
            end = Position(start.x + int(update["width"]), start.y + int(update["height"]))

    return [
        Position(x, y)
        for y in range(start.y, end.y)
        for x in range(start.x, end.x)
        if grid_map.in_bounds(Position(x, y))
    ]


def _position_from_mapping(value: dict[str, Any] | list[int] | tuple[int, int]) -> Position:
    if isinstance(value, dict):
        return Position(int(value["x"]), int(value["y"]))
    return Position(int(value[0]), int(value[1]))
