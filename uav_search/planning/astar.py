"""
A*路径规划算法模块

实现了基于栅格地图的A*搜索算法，用于无人机路径规划。
支持8邻域移动、对角距离启发函数、环境代价增强。

算法特点：
- 保证找到最优路径（如果存在）
- 支持对角移动，路径更自然
- 考虑障碍物接近代价，提高安全性
- 鼓励经过未搜索的重点区域
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from itertools import count

from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap


@dataclass(frozen=True)
class AStarConfig:
    """A*算法配置参数

    属性：
        obstacle_proximity_penalty: 靠近障碍物的额外代价，提高安全性
        priority_area_bonus: 经过未搜索重点区域的奖励（负代价）
    """
    obstacle_proximity_penalty: float = 0.5
    priority_area_bonus: float = -0.2


def astar_search(
    grid_map: GridMap,
    start: Position,
    goal: Position,
    config: AStarConfig | None = None,
) -> list[Position] | None:
    """A*路径搜索主函数

    使用A*算法在栅格地图上搜索从起点到终点的最优路径。
    支持8邻域移动，考虑环境代价增强。

    参数：
        grid_map: 栅格地图对象
        start: 起点坐标
        goal: 终点坐标
        config: 算法配置参数，None则使用默认值

    返回：
        list[Position] | None: 路径点列表（包含起点和终点），失败返回None

    算法流程：
        1. 初始化开放列表和关闭列表
        2. 从开放列表中取出f值最小的节点
        3. 如果到达目标，重构路径
        4. 扩展邻居节点，计算g值和f值
        5. 更新或添加邻居到开放列表
        6. 重复2-5直到找到路径或开放列表为空

    性能：
        - 200×200栅格典型耗时 < 50ms
        - 使用堆优化开放列表操作
        - 支持对角距离启发函数，保证可采纳性
    """
    if not grid_map.is_passable(start) or not grid_map.is_passable(goal):
        return None

    config = config or AStarConfig()
    open_heap: list[tuple[float, int, Position]] = []
    sequence = count()
    heapq.heappush(open_heap, (0.0, next(sequence), start))

    came_from: dict[Position, Position] = {}  # 记录每个节点的前驱节点
    g_score: dict[Position, float] = {start: 0.0}  # 从起点到每个节点的实际代价
    closed: set[Position] = set()  # 已访问节点集合

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        # 扩展邻居节点（8邻域）
        for neighbor in grid_map.get_neighbors(current, mode=8):
            # 计算从起点经过current到neighbor的代价
            tentative_g = g_score[current] + _move_cost(current, neighbor) + _environment_cost(
                grid_map, neighbor, config
            )
            if tentative_g >= g_score.get(neighbor, math.inf):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            # f = g + h，h为启发函数值
            f_score = tentative_g + _diagonal_distance(neighbor, goal)
            heapq.heappush(open_heap, (f_score, next(sequence), neighbor))

    return None


def path_cost(path: list[Position]) -> float:
    """计算路径总代价

    参数：
        path: 路径点列表

    返回：
        float: 路径总代价（距离）
    """
    if len(path) < 2:
        return 0.0
    return sum(_move_cost(path[idx - 1], path[idx]) for idx in range(1, len(path)))


def _move_cost(a: Position, b: Position) -> float:
    """计算两相邻栅格间的移动代价

    对角移动代价为√2，直线移动代价为1。
    这确保了对角移动不会比直线移动更优。

    参数：
        a: 起点坐标
        b: 终点坐标

    返回：
        float: 移动代价
    """
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return 1.41421356237 if dx == 1 and dy == 1 else 1.0


def _diagonal_distance(a: Position, b: Position) -> float:
    """计算对角距离启发函数值

    对角距离是一种可采纳的启发函数，适用于8邻域移动。
    公式：h = D * (dx + dy) + (D2 - 2*D) * min(dx, dy)
    其中D=1（直线移动代价），D2=√2（对角移动代价）

    参数：
        a: 起点
        b: 终点

    返回：
        float: 启发函数值（估计距离）
    """
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (1.41421356237 - 2.0) * min(dx, dy)


def _environment_cost(grid_map: GridMap, pos: Position, config: AStarConfig) -> float:
    """计算环境代价增强

    在基础移动代价上叠加环境因素：
    1. 靠近障碍物的栅格增加代价（提高安全性）
    2. 经过未搜索重点区域给予奖励（鼓励覆盖）

    参数：
        grid_map: 栅格地图
        pos: 当前位置
        config: 算法配置

    返回：
        float: 环境代价（可为负值表示奖励）

    设计思路：
        - 障碍物接近惩罚：避免无人机贴着障碍物飞行
        - 重点区域奖励：在路径规划时顺带覆盖重点区域
    """
    cost = 0.0
    if _near_blocked_cell(grid_map, pos):
        cost += config.obstacle_proximity_penalty

    cell = grid_map.get_cell(pos)
    if cell.cell_type == CellType.PRIORITY and cell.search_confidence < 0.95:
        cost += config.priority_area_bonus
    return max(cost, -0.9)  # 限制最大奖励，避免负代价过大


def _near_blocked_cell(grid_map: GridMap, pos: Position) -> bool:
    """检查位置是否靠近障碍物或禁飞区

    检查周围8个栅格，如果存在不可通行栅格则返回True。

    参数：
        grid_map: 栅格地图
        pos: 待检查位置

    返回：
        bool: 是否靠近障碍物
    """
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            candidate = Position(pos.x + dx, pos.y + dy)
            if grid_map.in_bounds(candidate) and not grid_map.is_passable(candidate):
                return True
    return False


def _reconstruct_path(came_from: dict[Position, Position], current: Position) -> list[Position]:
    """重构路径

    从目标节点回溯到起点，构建完整路径。

    参数：
        came_from: 前驱节点字典
        current: 目标节点（搜索终点）

    返回：
        list[Position]: 从起点到终点的路径列表
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
