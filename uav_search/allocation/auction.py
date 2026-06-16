"""
顺序单物品拍卖任务分配模块

实现了基于拍卖机制的任务分配算法，用于多无人机协同任务分配。
采用顺序单物品拍卖（Sequential Single-Item Auction）策略。

算法特点：
- 分布式决策思想，每个无人机独立竞标
- 考虑距离代价、电量约束、任务优先级
- 支持负载均衡，避免任务分配不均
- 可扩展性好，易于添加新的竞标策略
"""
from __future__ import annotations

from uav_search.allocation.bid_calculator import calculate_bid
from uav_search.core.data_types import Assignment, Task, TaskType, UAVState
from uav_search.maps.grid_map import GridMap


class SequentialAuction:
    """顺序单物品拍卖分配器

    实现顺序单物品拍卖算法，逐个任务进行竞标分配。

    拍卖流程：
        1. 按优先级对待分配任务排序
        2. 对每个任务，所有可用无人机提交竞标值
        3. 选择竞标值最优的无人机获得任务
        4. 已分配任务的无人机不参与后续竞标
        5. 重复2-4直到所有任务分配完毕或无可用无人机

    属性：
        config: 配置字典，包含电量阈值等参数
    """

    def __init__(self, config: dict) -> None:
        """初始化拍卖分配器

        参数：
            config: 配置字典，包含：
                - battery_threshold: 电量阈值，低于此值不参与竞标
                - 其他竞标权重参数
        """
        self.config = config

    def allocate(
        self,
        pending_tasks: list[Task],
        available_uavs: list[UAVState],
        grid_map: GridMap,
        now: float = 0.0,
    ) -> list[Assignment]:
        """执行任务分配

        对待分配任务列表执行顺序拍卖算法，返回分配结果。

        参数：
            pending_tasks: 待分配任务列表
            available_uavs: 可用无人机列表
            grid_map: 栅格地图对象
            now: 当前时间戳

        返回：
            list[Assignment]: 任务分配结果列表

        分配策略：
            1. 任务排序：按优先级降序、创建时间升序、ID升序
            2. CONFIRM类型任务：直接分配给指定无人机
            3. 普通任务：竞标值最小者获胜
            4. 每个无人机每轮最多获得一个任务

        注意：
            - 已分配任务的无人机从可用列表中移除
            - 无无人机竞标的任务会被跳过
        """
        assignments: list[Assignment] = []
        available_by_id = {uav.id: uav for uav in available_uavs}

        # 顺序单物品拍卖：每个任务只拍卖一次，每个无人机最多赢得一个任务
        for task in sorted(pending_tasks, key=lambda item: (-item.priority, item.created_at, item.id)):
            if not available_by_id:
                break

            # CONFIRM类型任务直接分配给指定无人机（通常是发现者）
            if task.type == TaskType.CONFIRM and task.assigned_uav_id in available_by_id:
                winner = available_by_id.pop(task.assigned_uav_id)
                winner.assigned_task_count += 1
                assignments.append(Assignment(task.id, winner.id, 0.0, now))
                continue

            # 收集所有无人机的竞标值
            bids: list[tuple[float, UAVState]] = []
            for uav in available_by_id.values():
                bid = calculate_bid(uav, task, grid_map, self.config)
                if bid is not None:
                    bids.append((bid, uav))

            if not bids:
                continue

            # 选择竞标值最小的无人机获胜
            bid_value, winner = min(bids, key=lambda item: (item[0], item[1].id))
            winner.assigned_task_count += 1
            available_by_id.pop(winner.id)
            assignments.append(Assignment(task.id, winner.id, bid_value, now))

        return assignments
