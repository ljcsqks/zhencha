"""
栅格地图模块

实现了基于栅格的环境地图，用于无人机路径规划和搜索管理。
采用NumPy数组存储地图数据，支持高效的区域操作。

主要功能：
- 栅格地图的创建和初始化
- 地图属性的设置和查询
- 邻居节点获取（支持4邻域和8邻域）
- 搜索覆盖状态管理
- 覆盖率统计计算
"""
from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np

from uav_search.core.data_types import CellType, GridCell, Position


class GridMap:
    """栅格地图类

    使用二维栅格表示搜索区域，存储多层属性信息。

    属性：
        width_m: 地图宽度（米）
        height_m: 地图高度（米）
        resolution_m: 栅格分辨率（米）
        width_cells: 地图宽度（栅格数）
        height_cells: 地图高度（栅格数）
        terrain: 地形类型数组
        passable: 可通行性数组
        search_confidence: 搜索置信度数组
        search_priority: 搜索优先级数组
        last_search_time: 最后搜索时间数组
        coverage_count: 覆盖次数数组

    数据组织：
        - 使用NumPy数组存储，shape = (height_cells, width_cells)
        - 坐标系统：x为列索引，y为行索引
        - 支持高效的向量化操作
    """

    def __init__(self, width_m: float, height_m: float, resolution_m: float) -> None:
        """初始化栅格地图

        参数：
            width_m: 地图宽度（米）
            height_m: 地图高度（米）
            resolution_m: 栅格分辨率（米），即每个栅格的实际边长

        异常：
            ValueError: 如果参数不合法（非正数）

        初始化状态：
            - 所有栅格初始为FREE类型
            - 所有栅格初始可通行
            - 搜索置信度初始化为0
            - 搜索优先级初始化为1.0
        """
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
        """检查坐标是否在地图范围内

        参数：
            pos: 待检查的坐标

        返回：
            bool: 是否在范围内
        """
        return 0 <= pos.x < self.width_cells and 0 <= pos.y < self.height_cells

    def is_passable(self, pos: Position) -> bool:
        """检查坐标是否可通行

        综合考虑地图边界和障碍物/禁飞区。

        参数：
            pos: 待检查的坐标

        返回：
            bool: 是否可通行
        """
        return self.in_bounds(pos) and bool(self.passable[pos.y, pos.x])

    def get_cell(self, pos: Position) -> GridCell:
        """获取指定位置的栅格信息

        参数：
            pos: 栅格坐标

        返回：
            GridCell: 包含所有属性的栅格对象

        异常：
            IndexError: 如果坐标超出范围
        """
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
        """设置指定位置栅格的属性

        参数：
            pos: 栅格坐标
            attrs: 属性字典，可包含：
                - cell_type: 地形类型
                - passable: 可通行性
                - search_confidence: 搜索置信度
                - search_priority: 搜索优先级
                - last_search_time: 最后搜索时间

        异常：
            IndexError: 如果坐标超出范围

        注意：
            设置cell_type时会自动更新passable属性
        """
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
        """批量更新区域属性

        对多个栅格应用相同的属性更新。

        参数：
            cells: 栅格坐标的可迭代集合
            attrs: 属性字典

        返回：
            list[Position]: 成功更新的栅格坐标列表
        """
        affected: list[Position] = []
        for pos in cells:
            if self.in_bounds(pos):
                self.set_cell(pos, attrs)
                affected.append(pos)
        return affected

    def get_neighbors(self, pos: Position, mode: int = 8) -> list[Position]:
        """获取邻居节点

        返回指定位置的可行邻居节点。

        参数：
            pos: 中心位置
            mode: 邻域模式，4表示四邻域，8表示八邻域

        返回：
            list[Position]: 可通行的邻居节点列表

        异常：
            ValueError: 如果mode不是4或8

        注意：
            - 八邻域模式下，对角移动需要检查拐角是否被阻挡
            - 避免无人机"穿墙"移动（对角线穿过障碍物）
        """
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
                # 对角移动时，检查拐角是否被阻挡，避免"穿墙"
                if not self.is_passable(Position(pos.x + dx, pos.y)):
                    continue
                if not self.is_passable(Position(pos.x, pos.y + dy)):
                    continue
            neighbors.append(candidate)
        return neighbors

    def get_searchable_cells(self) -> list[Position]:
        """获取所有可搜索栅格

        返回所有可通行栅格的坐标列表。

        返回：
            list[Position]: 可搜索栅格坐标列表

        用途：
            用于任务生成时确定搜索区域
        """
        ys, xs = np.where(self.passable)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def get_unsearched_cells(self, threshold: float = 0.01) -> list[Position]:
        """获取未搜索栅格

        返回搜索置信度低于阈值的可通行栅格。

        参数：
            threshold: 搜索置信度阈值，默认0.01

        返回：
            list[Position]: 未搜索栅格坐标列表

        用途：
            用于动态生成新的搜索任务
        """
        mask = self.passable & (self.search_confidence < threshold)
        ys, xs = np.where(mask)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def get_priority_cells(self) -> list[Position]:
        """获取重点区域栅格

        返回所有重点区域的可通行栅格坐标。

        返回：
            list[Position]: 重点区域栅格坐标列表

        用途：
            用于生成高优先级搜索任务
        """
        mask = self.passable & (self.terrain == CellType.PRIORITY.value)
        ys, xs = np.where(mask)
        return [Position(int(x), int(y)) for y, x in zip(ys, xs)]

    def mark_covered(self, center: Position, radius_cells: int, timestamp: float) -> list[Position]:
        """标记传感器覆盖区域

        当无人机飞越某位置时，标记其传感器覆盖范围内的栅格。

        参数：
            center: 无人机中心位置
            radius_cells: 传感器覆盖半径（栅格数）
            timestamp: 当前时间戳

        返回：
            list[Position]: 被标记的栅格坐标列表

        更新内容：
            - search_confidence 设为 1.0
            - last_search_time 设为当前时间戳
            - coverage_count 加1

        注意：
            使用圆形区域近似传感器覆盖范围
        """
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
        """搜索置信度衰减

        对已搜索栅格应用时间衰减，适用于移动目标场景。

        参数：
            current_time: 当前时间戳
            lambda_decay: 衰减系数，λ=0表示不衰减

        衰减公式：
            confidence(t) = confidence(t0) × exp(-λ × (t - t0))

        用途：
            模拟搜索信息的时间有效性衰减
            促使系统重复搜索重要区域
        """
        if lambda_decay <= 0:
            return
        searched = self.passable & ~np.isnan(self.last_search_time)
        elapsed = current_time - self.last_search_time[searched]
        elapsed = np.maximum(elapsed, 0.0)
        self.search_confidence[searched] *= np.exp(-lambda_decay * elapsed)
        self.last_search_time[searched] = current_time

    def coverage_rate(self, priority_only: bool = False) -> float:
        """计算覆盖率

        计算全局或重点区域的搜索覆盖率。

        参数：
            priority_only: 是否仅计算重点区域覆盖率

        返回：
            float: 覆盖率 [0,1]

        计算方式：
            覆盖率 = 搜索置信度≥0.95的栅格数 / 可通行栅格总数
        """
        mask = self.passable
        if priority_only:
            mask = mask & (self.terrain == CellType.PRIORITY.value)
        total = int(np.count_nonzero(mask))
        if total == 0:
            return 0.0
        covered = int(np.count_nonzero(mask & (self.search_confidence >= 0.95)))
        return covered / total

    def redundant_coverage_rate(self) -> float:
        """计算重复覆盖率

        计算被多次搜索的栅格比例，用于评估协同效率。

        返回：
            float: 重复覆盖率 [0,1]

        计算方式：
            重复覆盖率 = coverage_count > 1的栅格数 / coverage_count > 0的栅格数

        用途：
            评估任务分配的合理性，避免过多重复搜索
        """
        searched = self.passable & (self.coverage_count > 0)
        total = int(np.count_nonzero(searched))
        if total == 0:
            return 0.0
        redundant = int(np.count_nonzero(searched & (self.coverage_count > 1)))
        return redundant / total

    def world_to_grid(self, x_m: float, y_m: float) -> Position:
        """世界坐标转栅格坐标

        将真实世界的米制坐标转换为栅格索引。

        参数：
            x_m: 世界坐标x（米）
            y_m: 世界坐标y（米）

        返回：
            Position: 栅格坐标
        """
        return Position(int(math.floor(x_m / self.resolution_m)), int(math.floor(y_m / self.resolution_m)))

    def grid_to_world(self, pos: Position) -> tuple[float, float]:
        """栅格坐标转世界坐标

        将栅格索引转换为真实世界的米制坐标。
        返回栅格中心点的世界坐标。

        参数：
            pos: 栅格坐标

        返回：
            tuple[float, float]: 世界坐标(x_m, y_m)（米）
        """
        return ((pos.x + 0.5) * self.resolution_m, (pos.y + 0.5) * self.resolution_m)
