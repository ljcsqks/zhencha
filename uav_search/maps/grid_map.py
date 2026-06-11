from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np

from uav_search.core.data_types import CellType, GridCell, Position


class GridMap:
    def __init__(self, width_m: float, height_m: float, resolution_m: float) -> None:
        if width_m <= 0 or height_m <= 0 or resolution_m <= 0:
            raise ValueError("width_m, height_m, and resolution_m must be greater than 0")

        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution_m = float(resolution_m)
        self.width_cells = int(math.ceil(width_m / resolution_m))
        self.height_cells = int(math.ceil(height_m / resolution_m))

        shape = (self.height_cells, self.width_cells)
        self.terrain = np.full(shape, CellType.FREE.value, dtype=object)
        self.passable = np.ones(shape, dtype=bool)
        self.search_confidence = np.zeros(shape, dtype=float)
        self.search_priority = np.ones(shape, dtype=float)
        self.last_search_time = np.full(shape, np.nan, dtype=float)
        self.coverage_count = np.zeros(shape, dtype=int)

    def in_bounds(self, pos: Position) -> bool:
        return 0 <= pos.x < self.width_cells and 0 <= pos.y < self.height_cells

    def is_passable(self, pos: Position) -> bool:
        return self.in_bounds(pos) and bool(self.passable[pos.y, pos.x])

    def get_cell(self, pos: Position) -> GridCell:
        if not self.in_bounds(pos):
            raise IndexError(f"Position out of bounds: {pos}")
        last_time = self.last_search_time[pos.y, pos.x]
        return GridCell(
            position=pos,
            cell_type=CellType(self.terrain[pos.y, pos.x]),
            passable=bool(self.passable[pos.y, pos.x]),
            search_confidence=float(self.search_confidence[pos.y, pos.x]),
            search_priority=float(self.search_priority[pos.y, pos.x]),
            last_search_time=None if np.isnan(last_time) else float(last_time),
        )

    def set_cell(self, pos: Position, attrs: dict[str, Any]) -> None:
        if not self.in_bounds(pos):
            raise IndexError(f"Position out of bounds: {pos}")

        cell_type = attrs.get("cell_type")
        if cell_type is not None:
            if isinstance(cell_type, str):
                cell_type = CellType(cell_type)
            self.terrain[pos.y, pos.x] = cell_type.value
            if cell_type in (CellType.OBSTACLE, CellType.NO_FLY):
                self.passable[pos.y, pos.x] = False
            elif cell_type in (CellType.FREE, CellType.PRIORITY):
                self.passable[pos.y, pos.x] = bool(attrs.get("passable", True))

        if "passable" in attrs:
            self.passable[pos.y, pos.x] = bool(attrs["passable"])
        if "search_confidence" in attrs:
            self.search_confidence[pos.y, pos.x] = float(attrs["search_confidence"])
        if "search_priority" in attrs:
            self.search_priority[pos.y, pos.x] = float(attrs["search_priority"])
        if "last_search_time" in attrs:
            value = attrs["last_search_time"]
            self.last_search_time[pos.y, pos.x] = np.nan if value is None else float(value)

    def update_region(self, cells: Iterable[Position], attrs: dict[str, Any]) -> list[Position]:
        affected: list[Position] = []
        for pos in cells:
            if self.in_bounds(pos):
                self.set_cell(pos, attrs)
                affected.append(pos)
        return affected

    def get_neighbors(self, pos: Position, mode: int = 8) -> list[Position]:
        if mode not in (4, 8):
            raise ValueError("mode must be 4 or 8")

        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if mode == 8:
            offsets.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        neighbors: list[Position] = []
        for dx, dy in offsets:
            candidate = Position(pos.x + dx, pos.y + dy)
            if not self.is_passable(candidate):
                continue
            if mode == 8 and dx != 0 and dy != 0:
                # Avoid cutting diagonally through blocked corners.
                if not self.is_passable(Position(pos.x + dx, pos.y)):
                    continue
                if not self.is_passable(Position(pos.x, pos.y + dy)):
                    continue
            neighbors.append(candidate)
        return neighbors

    def get_searchable_cells(self) -> list[Position]:
        ys, xs = np.where(self.passable)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def get_unsearched_cells(self, threshold: float = 0.01) -> list[Position]:
        mask = self.passable & (self.search_confidence < threshold)
        ys, xs = np.where(mask)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def get_priority_cells(self) -> list[Position]:
        mask = self.passable & (self.terrain == CellType.PRIORITY.value)
        ys, xs = np.where(mask)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def mark_covered(self, center: Position, radius_cells: int, timestamp: float) -> list[Position]:
        covered: list[Position] = []
        radius_sq = radius_cells * radius_cells
        for y in range(center.y - radius_cells, center.y + radius_cells + 1):
            for x in range(center.x - radius_cells, center.x + radius_cells + 1):
                pos = Position(x, y)
                if not self.is_passable(pos):
                    continue
                if (x - center.x) ** 2 + (y - center.y) ** 2 > radius_sq:
                    continue
                self.search_confidence[y, x] = 1.0
                self.last_search_time[y, x] = timestamp
                self.coverage_count[y, x] += 1
                covered.append(pos)
        return covered

    def decay_search_confidence(self, current_time: float, lambda_decay: float) -> None:
        if lambda_decay <= 0:
            return
        searched = self.passable & ~np.isnan(self.last_search_time)
        elapsed = current_time - self.last_search_time[searched]
        elapsed = np.maximum(elapsed, 0.0)
        self.search_confidence[searched] *= np.exp(-lambda_decay * elapsed)
        self.last_search_time[searched] = current_time

    def coverage_rate(self, priority_only: bool = False) -> float:
        mask = self.passable
        if priority_only:
            mask = mask & (self.terrain == CellType.PRIORITY.value)
        total = int(np.count_nonzero(mask))
        if total == 0:
            return 0.0
        covered = int(np.count_nonzero(mask & (self.search_confidence >= 0.95)))
        return covered / total

    def redundant_coverage_rate(self) -> float:
        searched = self.passable & (self.coverage_count > 0)
        total = int(np.count_nonzero(searched))
        if total == 0:
            return 0.0
        redundant = int(np.count_nonzero(searched & (self.coverage_count > 1)))
        return redundant / total

    def world_to_grid(self, x_m: float, y_m: float) -> Position:
        return Position(int(math.floor(x_m / self.resolution_m)), int(math.floor(y_m / self.resolution_m)))

    def grid_to_world(self, pos: Position) -> tuple[float, float]:
        return ((pos.x + 0.5) * self.resolution_m, (pos.y + 0.5) * self.resolution_m)
