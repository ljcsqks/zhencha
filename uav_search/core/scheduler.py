"""
主控调度器模块

该模块实现了系统的核心决策循环，协调各个模块完成：
1. 事件处理（高优先级事件立即响应）
2. 任务生成与分配
3. 路径规划
4. 冲突检测与消解

调度器采用混合触发模式：固定周期决策 + 事件驱动响应。
"""
from __future__ import annotations

import time
from dataclasses import replace
from typing import Any

from uav_search.allocation.auction import SequentialAuction
from uav_search.core.data_types import CommandType, DecisionCommand, DecisionOutput, Event, EventPriority, EventType, Position
from uav_search.core.data_types import UAVState, UAVStatus
from uav_search.core.event_manager import EventManager
from uav_search.maps.grid_map import GridMap
from uav_search.maps.map_updater import MapUpdater
from uav_search.planning.conflict_resolver import detect_conflicts, resolve_conflicts
from uav_search.planning.path_planner import PathPlanner
from uav_search.task.task_generator import generate_initial_tasks
from uav_search.task.task_manager import TaskManager
from uav_search.uav.fleet_manager import FleetManager


class Scheduler:
    """主控调度器类

    负责协调系统的整体决策流程，是系统的核心控制中心。

    主要职责：
    - 管理固定周期的决策循环
    - 处理高优先级事件的即时响应
    - 协调任务分配、路径规划、冲突消解等模块
    - 维护系统全局状态

    属性：
        grid_map: 栅格地图对象
        fleet: 无人机编队管理器
        config: 系统配置字典
        planner: 路径规划器
        map_updater: 地图更新器
        auction: 拍卖分配器
        task_manager: 任务管理器
        event_manager: 事件管理器
        _confirmations: 目标确认任务追踪字典
        _initialized: 是否已初始化任务

    设计模式：
    - 采用策略模式：不同事件类型由不同的处理方法处理
    - 采用观察者模式：通过事件管理器实现事件驱动
    """

    def __init__(self, grid_map: GridMap, fleet: FleetManager, config: dict[str, Any]) -> None:
        """初始化调度器

        参数：
            grid_map: 栅格地图对象
            fleet: 无人机编队管理器
            config: 系统配置字典，包含各模块参数
        """
        self.grid_map = grid_map
        self.fleet = fleet
        self.config = config
        self.planner = PathPlanner(config.get("planning", {}))
        self.map_updater = MapUpdater(grid_map)
        self.auction = SequentialAuction({**config, "battery_threshold": config["uav"]["battery_threshold"]})
        self.task_manager = TaskManager()
        self.event_manager = EventManager(config["scheduler"].get("event_debounce_s", 0.2))
        self._confirmations: dict[str, dict[str, Any]] = {}  # 追踪目标确认任务状态
        self._initialized = False  # 标记是否已生成初始任务

    def regular_cycle(self, now: float = 0.0) -> DecisionOutput:
        """执行一次完整的决策周期

        这是调度器的核心方法，执行完整的决策流程：
        1. 处理待处理的事件（高优先级优先）
        2. 确保初始任务已生成
        3. 执行任务分配（拍卖算法）
        4. 规划路径
        5. 检测并消解冲突
        6. 返回决策输出

        参数：
            now: 当前时间戳（秒）

        返回：
            DecisionOutput: 包含指令、分配、覆盖率等信息的决策输出

        性能要求：
            单次决策延迟应 < 1000ms，满足实时性要求
        """
        started = time.perf_counter()
        events_handled: list[str] = []
        commands: list[DecisionCommand] = []

        # 步骤1: 处理待处理的事件（高优先级事件优先处理）
        urgent_commands, urgent_event_ids = self.handle_urgent_events(self.event_manager.poll_events(now))
        commands.extend(urgent_commands)
        events_handled.extend(urgent_event_ids)

        # 步骤2: 确保初始任务已生成（仅首次执行）
        self._ensure_initial_tasks(now)
        self._refresh_task_progress(now)

        # 步骤3: 执行任务分配
        assignments = []
        proposed_assignments = self.auction.allocate(
            self.task_manager.get_pending_tasks(),
            self.fleet.get_available_uavs(),
            self.grid_map,
            now=now,
        )

        # 步骤4: 为每个分配的任务规划路径并下发指令
        for proposed in proposed_assignments:
            task = self.task_manager.tasks[proposed.task_id]
            assignment = self.task_manager.assign_task(task.id, proposed.uav_id, now=now, bid_value=proposed.bid_value)
            uav_state = self.fleet.get_uav(proposed.uav_id).state

            # 规划经过所有航路点的路径
            route = self._plan_route_through_waypoints(uav_state, task.waypoints)
            if not route:
                # 路径规划失败，标记任务为阻塞状态
                self.task_manager.mark_blocked(task.id, now=now)
                commands.append(
                    DecisionCommand(
                        uav_id=uav_state.id,
                        command=CommandType.HOLD,
                        task_id=task.id,
                        target=task.entry_point,
                        path=[],
                        reason="task_route_not_found",
                    )
                )
                continue

            # 路径规划成功，更新任务和无人机状态
            self.task_manager.start_task(task.id, now=now)
            uav_state.current_task_id = task.id
            self.fleet.assign_path(uav_state.id, route, status=UAVStatus.SEARCHING)
            assignments.append(assignment)
            commands.append(
                DecisionCommand(
                    uav_id=uav_state.id,
                    command=CommandType.FOLLOW_PATH,
                    task_id=task.id,
                    target=task.waypoints[-1],
                    path=route,
                    reason="auction_search_task",
                )
            )

        # 步骤5: 检测并消解冲突
        # 冲突检测：检查所有无人机路径是否存在碰撞风险
        conflicts = detect_conflicts(
            self.fleet.get_all_states(),
            safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            time_horizon_steps=int(self.config["planning"]["conflict_time_horizon_steps"]),
        )
        # 冲突消解：为低优先级无人机添加等待指令
        commands.extend(
            resolve_conflicts(
                conflicts,
                self.fleet.get_all_states(),
                safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            )
        )

        # 计算决策延迟并返回结果
        latency_ms = (time.perf_counter() - started) * 1000.0
        return DecisionOutput(
            timestamp=now,
            commands=commands,
            assignments=assignments,
            events_handled=events_handled,
            global_coverage=self.grid_map.coverage_rate(),
            priority_coverage=self.grid_map.coverage_rate(priority_only=True),
            decision_latency_ms=latency_ms,
        )

    def handle_event(self, event: Event) -> list[DecisionCommand]:
        """处理单个事件

        根据事件类型调用相应的处理方法。
        不同事件类型有不同的响应策略。

        参数：
            event: 待处理的事件对象

        返回：
            list[DecisionCommand]: 事件触发的决策指令列表

        事件处理映射：
            - LOW_BATTERY: 触发返航
            - UAV_OFFLINE: 标记离线并回收任务
            - MAP_UPDATE: 更新地图并重规划受影响路径
            - TARGET_FOUND: 触发目标确认任务
            - CONFIRM_DONE: 完成确认，恢复IDLE状态
        """
        if event.type == EventType.LOW_BATTERY:
            return self._handle_low_battery(event)
        if event.type == EventType.UAV_OFFLINE:
            return self._handle_uav_offline(event)
        if event.type == EventType.MAP_UPDATE:
            return self._handle_map_update(event)
        if event.type == EventType.TARGET_FOUND:
            return self._handle_target_found(event)
        if event.type == EventType.CONFIRM_DONE:
            return self._handle_confirm_done(event)
        return []

    def handle_urgent_events(self, events: list[Event]) -> tuple[list[DecisionCommand], list[str]]:
        """批量处理紧急事件

        对事件列表中的每个事件调用handle_event，并收集结果。

        参数：
            events: 待处理的事件列表

        返回：
            tuple: (决策指令列表, 已处理的事件ID列表)
        """
        commands: list[DecisionCommand] = []
        handled_ids: list[str] = []
        for event in events:
            commands.extend(self.handle_event(event))
            handled_ids.append(event.id)
        return commands, handled_ids

    def update_after_step(self, now: float) -> tuple[list[DecisionCommand], list[str]]:
        """仿真步进后更新任务状态

        在每个仿真步进后调用，检查目标确认任务的完成条件。
        当无人机到达目标位置并停留足够时间后，触发确认完成事件。

        参数：
            now: 当前时间戳（秒）

        返回：
            tuple: (决策指令列表, 已处理的事件ID列表)

        设计思路：
            - 目标确认需要无人机抵近并停留一段时间
            - 停留时间由配置参数 confirm_duration_steps 决定
            - 完成后触发 CONFIRM_DONE 事件
        """
        commands: list[DecisionCommand] = []
        handled_ids: list[str] = []
        self._refresh_task_progress(now)
        for task_id, confirmation in list(self._confirmations.items()):
            uav = self.fleet.get_uav(confirmation["uav_id"]).state
            target = confirmation["target"]

            # 检查无人机是否仍在确认状态且到达目标位置
            if uav.status != UAVStatus.CONFIRMING or uav.position != target:
                continue

            # 增加停留计数
            confirmation["dwell_steps"] += 1

            # 检查是否达到确认所需停留时间
            if confirmation["dwell_steps"] < int(self.config["search"]["confirm_duration_steps"]):
                continue

            # 触发确认完成事件
            event = Event(
                id=f"confirm_done_{task_id}",
                type=EventType.CONFIRM_DONE,
                timestamp=now,
                priority=EventPriority.NORMAL,
                source_uav_id=uav.id,
                data={"task_id": task_id, "target_id": confirmation["target_id"]},
            )
            commands.extend(self.handle_event(event))
            handled_ids.append(event.id)
        return commands, handled_ids

    def _ensure_initial_tasks(self, now: float) -> None:
        """确保初始任务已生成

        在首次决策周期生成初始搜索任务。
        仅执行一次，之后通过 _initialized 标志避免重复生成。

        参数：
            now: 当前时间戳（秒）

        异常：
            ValueError: 如果编队中没有无人机
        """
        if self._initialized:
            return
        states = self.fleet.get_all_states()
        if not states:
            raise ValueError("fleet must contain at least one UAV")
        tasks = generate_initial_tasks(
            grid_map=self.grid_map,
            uav_count=int(self.config["uav"]["count"]),
            sensor_radius_cells=int(self.config["uav"]["sensor_radius_cells"]),
            home=states[0].home_position,
            created_at=now,
        )
        self.task_manager.add_tasks(tasks)
        self._initialized = True

    def _refresh_task_progress(self, now: float) -> None:
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        self.task_manager.update_progress(self.grid_map, now=now, coverage_threshold=coverage_threshold)
        self.task_manager.refresh_pending_waypoints(self.grid_map, now=now, coverage_threshold=coverage_threshold)

    def _plan_route_through_waypoints(self, uav_state: UAVState, waypoints: list[Position]) -> list[Position]:
        """规划经过多个航路点的路径

        为无人机规划一条经过所有航路点的连续路径。
        对每段航路分别调用A*规划，确保避开障碍物。

        参数：
            uav_state: 无人机当前状态
            waypoints: 航路点列表

        返回：
            list[Position]: 完整路径点列表，失败返回空列表

        设计思路：
            - 将复杂路径分解为多段，逐段规划
            - 每段使用A*算法确保避障
            - 合并路径时去除重复点
        """
        route: list[Position] = []
        current = uav_state.position

        # 逐段规划路径
        for waypoint in waypoints:
            if waypoint == current:
                continue
            segment_uav = replace(uav_state, position=current)
            plan = self.planner.plan_path(segment_uav, waypoint, self.grid_map)
            if not plan.valid:
                return []
            if not route:
                route.extend(plan.path)
            else:
                route.extend(plan.path[1:])  # 去除重复的连接点
            current = waypoint

        return route

    def _handle_low_battery(self, event: Event) -> list[DecisionCommand]:
        """处理低电量事件

        当无人机电量低于阈值时触发，强制无人机返航。

        参数：
            event: 低电量事件对象

        返回：
            list[DecisionCommand]: 包含返航指令的列表

        处理流程：
            1. 将无人机状态切换为RETURNING
            2. 标记为不可用
            3. 规划返回起飞点的路径
            4. 如果规划失败，发送HOLD指令
        """
        if event.source_uav_id is None:
            return []
        uav = self.fleet.get_uav(event.source_uav_id).state
        uav.status = UAVStatus.RETURNING
        uav.available = False
        plan = self.planner.plan_path(uav, uav.home_position, self.grid_map, task_id=uav.current_task_id)
        if not plan.valid:
            return [
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.HOLD,
                    task_id=uav.current_task_id,
                    target=uav.home_position,
                    path=[],
                    reason="return_path_not_found",
                )
            ]

        self.fleet.assign_path(uav.id, plan.path, status=UAVStatus.RETURNING)
        return [
            DecisionCommand(
                uav_id=uav.id,
                command=CommandType.RETURN_HOME,
                task_id=uav.current_task_id,
                target=uav.home_position,
                path=plan.path,
                reason="low_battery",
            )
        ]

    def _handle_uav_offline(self, event: Event) -> list[DecisionCommand]:
        """处理无人机离线事件

        当无人机出现故障或通信中断时触发，标记为离线状态。

        参数：
            event: 离线事件对象

        返回：
            list[DecisionCommand]: 包含HOLD指令的列表

        处理流程：
            1. 将无人机状态切换为OFFLINE
            2. 标记为不可用
            3. 清空当前路径
            4. 其未完成任务会在下一轮拍卖中重新分配
        """
        if event.source_uav_id is None:
            return []
        uav = self.fleet.get_uav(event.source_uav_id).state
        uav.status = UAVStatus.OFFLINE
        uav.available = False
        uav.path = []
        return [
            DecisionCommand(
                uav_id=uav.id,
                command=CommandType.HOLD,
                task_id=uav.current_task_id,
                target=None,
                path=[],
                reason="uav_offline",
            )
        ]

    def _handle_map_update(self, event: Event) -> list[DecisionCommand]:
        """处理地图更新事件

        当检测到新障碍物或区域变化时触发，更新地图并重规划受影响路径。

        参数：
            event: 地图更新事件对象

        返回：
            list[DecisionCommand]: 包含重规划指令的列表

        处理流程：
            1. 应用地图更新
            2. 检查所有无人机的路径是否受影响
            3. 对受影响路径重新规划
            4. 如果规划失败，发送HOLD指令
        """
        updates = event.data.get("updates", [])
        if not updates and "operation" in event.data:
            updates = [event.data]
        self.map_updater.apply_updates(updates)

        commands: list[DecisionCommand] = []
        for state in self.fleet.get_all_states():
            if state.status == UAVStatus.OFFLINE or not state.path:
                continue
            # 检查剩余路径是否仍然有效
            if self.planner.is_path_valid(state.path[state.path_index :], self.grid_map):
                continue

            # 路径失效，重新规划
            goal = state.path[-1]
            plan = self.planner.plan_path(state, goal, self.grid_map, task_id=state.current_task_id)
            if not plan.valid:
                commands.append(
                    DecisionCommand(
                        uav_id=state.id,
                        command=CommandType.HOLD,
                        task_id=state.current_task_id,
                        target=goal,
                        path=[],
                        reason="map_update_replan_failed",
                    )
                )
                continue
            self.fleet.assign_path(state.id, plan.path, status=state.status)
            commands.append(
                DecisionCommand(
                    uav_id=state.id,
                    command=CommandType.REPLAN,
                    task_id=state.current_task_id,
                    target=goal,
                    path=plan.path,
                    reason="map_update",
                )
            )
        return commands

    def _handle_target_found(self, event: Event) -> list[DecisionCommand]:
        """处理目标发现事件

        当无人机发现目标时触发，发现者切换为确认状态并抵近目标。

        参数：
            event: 目标发现事件对象

        返回：
            list[DecisionCommand]: 包含确认指令的列表

        处理流程：
            1. 将发现者状态切换为CONFIRMING
            2. 中断其当前搜索任务
            3. 创建目标确认任务
            4. 规划到目标的路径
            5. 追踪确认任务状态（用于后续判断完成）
        """
        if event.source_uav_id is None:
            return []
        target_data = event.data
        target_pos_data = target_data.get("position")
        if target_pos_data is None:
            return []

        target = Position(int(target_pos_data["x"]), int(target_pos_data["y"]))
        uav = self.fleet.get_uav(event.source_uav_id).state
        interrupted_task_id = uav.current_task_id

        # 将被中断的任务重新放回任务池
        if interrupted_task_id in self.task_manager.tasks:
            self.task_manager.requeue_task(interrupted_task_id, now=event.timestamp)
            self._refresh_task_progress(event.timestamp)

        uav.status = UAVStatus.CONFIRMING
        uav.available = False
        confirm_task_id = f"confirm_{target_data.get('target_id', event.id)}"
        uav.current_task_id = confirm_task_id

        # 记录确认任务状态，用于后续判断完成
        self._confirmations[confirm_task_id] = {
            "uav_id": uav.id,
            "target": target,
            "target_id": target_data.get("target_id", event.id),
            "dwell_steps": 0,
        }

        plan = self.planner.plan_path(uav, target, self.grid_map, task_id=confirm_task_id)
        if not plan.valid:
            return [
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.HOLD,
                    task_id=confirm_task_id,
                    target=target,
                    path=[],
                    reason="target_confirm_path_not_found",
                )
            ]

        self.fleet.assign_path(uav.id, plan.path, status=UAVStatus.CONFIRMING)
        return [
            DecisionCommand(
                uav_id=uav.id,
                command=CommandType.CONFIRM_TARGET,
                task_id=confirm_task_id,
                target=target,
                path=plan.path,
                reason="target_found",
            )
        ]

    def _handle_confirm_done(self, event: Event) -> list[DecisionCommand]:
        """处理目标确认完成事件

        当无人机完成目标确认后触发，恢复为IDLE状态。

        参数：
            event: 确认完成事件对象

        返回：
            list[DecisionCommand]: 包含HOLD指令的列表

        处理流程：
            1. 清除确认任务追踪记录
            2. 将无人机状态恢复为IDLE
            3. 标记为可用
            4. 清空任务和路径
            5. 在下一轮拍卖中会重新分配新任务
        """
        task_id = event.data.get("task_id")
        if task_id:
            self._confirmations.pop(task_id, None)
        if event.source_uav_id is None:
            return []
        uav = self.fleet.get_uav(event.source_uav_id).state
        uav.status = UAVStatus.IDLE
        uav.available = True
        uav.current_task_id = None
        uav.path = []
        uav.path_index = 0
        return [
            DecisionCommand(
                uav_id=uav.id,
                command=CommandType.HOLD,
                task_id=task_id,
                target=uav.position,
                path=[],
                reason="confirm_done",
            )
        ]
