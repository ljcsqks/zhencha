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
from dataclasses import dataclass
from dataclasses import replace
from typing import Any

from uav_search.core.contracts import AckStatus, CommandAck, ControlCommand
from uav_search.allocation.auction import SequentialAuction
from uav_search.core.data_types import CommandType, DecisionCommand, DecisionOutput, Event, EventPriority, EventType, Position
from uav_search.core.data_types import Task
from uav_search.core.data_types import TaskStatus, TaskType, UAVState, UAVStatus
from uav_search.core.event_manager import EventManager
from uav_search.maps.grid_map import GridMap
from uav_search.maps.map_updater import MapUpdater
from uav_search.planning.conflict_resolver import detect_conflicts, resolve_conflicts
from uav_search.planning.coverage_planner import create_coverage_planner
from uav_search.planning.facade_modeling_planner import BuildingFootprint, FacadeModelingPlanner, ModelingPlan
from uav_search.planning.path_planner import PathPlanner
from uav_search.planning.reachability import ReachabilityIndex, build_reachability_index
from uav_search.planning.reachability import connected_components as reachability_components
from uav_search.task.task_generator import estimate_task_cost
from uav_search.task.task_generator import connected_components, generate_boustrophedon_path, nearest_cell
from uav_search.task.task_generator import reorder_waypoints_for_uav
from uav_search.task.task_manager import TaskManager
from uav_search.uav.fleet_manager import FleetManager


@dataclass
class SupplementalCandidate:
    cells: set[Position]
    uncovered_cells: int
    priority_uncovered_cells: int
    uncovered_value: float
    priority_value: float
    nearest_uav_distance: float
    estimated_cost_m: float
    gain_per_meter: float
    score: float
    entry_point: Position
    waypoints: list[Position]
    allowed_uav_ids: set[str] | None = None


@dataclass
class ConfirmPlan:
    uav: UAVState
    target: Position
    orbit_waypoints: list[Position]
    route: list[Position]
    cost_m: float
    interrupted_task_id: str | None = None
    interrupt_cost_m: float = 0.0


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
        self.coverage_planner = create_coverage_planner(config)
        self.map_updater = MapUpdater(grid_map)
        self.auction = SequentialAuction({**config, "battery_threshold": config["uav"]["battery_threshold"]})
        self.task_manager = TaskManager()
        self.event_manager = EventManager(config["scheduler"].get("event_debounce_s", 0.2))
        self._confirmations: dict[str, dict[str, Any]] = {}  # 追踪目标确认任务状态
        self._confirmed_targets: set[str] = set()
        self._target_metrics: dict[str, dict[str, Any]] = {}
        self._confirmed_targets: set[str] = set()
        self._modeling_jobs: dict[str, dict[str, Any]] = {}
        self._modeling_job_seq = 0
        self._issued_commands: dict[str, ControlCommand] = {}
        self._handled_command_ack_keys: set[tuple[str, str, float]] = set()
        self._ack_events_handled: list[str] = []
        self._supplemental_task_seq = 0
        self._initialized = False  # 标记是否已生成初始任务
        self._supplemental_task_seq = 0
        self._diagnostics: dict[str, int] = {
            "cancelled_post_goal_tasks": 0,
            "skipped_post_goal_supplemental_tasks": 0,
            "post_goal_active_search_cancel_count": 0,
            "skipped_low_gain_supplemental_count": 0,
            "late_stage_supplemental_count": 0,
            "idle_assist_attempts": 0,
            "idle_assist_created_tasks": 0,
            "idle_assist_accepted_tasks": 0,
            "idle_assist_rejected_low_gain": 0,
            "idle_assist_rejected_unreachable": 0,
            "idle_assist_donor_replans": 0,
            "idle_uav_wait_time_s": 0,
            "idle_assist_cells_reassigned": 0,
            "idle_assist_distance_m": 0,
            "dynamic_route_repair_attempts": 0,
            "dynamic_route_repair_success": 0,
            "dynamic_route_repair_dropped_waypoints": 0,
            "dynamic_route_repair_replanned_tasks": 0,
            "dynamic_route_repair_fallback_to_supplemental": 0,
            "modeling_jobs_total": 0,
            "modeling_jobs_completed": 0,
            "modeling_jobs_failed": 0,
            "modeling_active_jobs": 0,
            "modeling_assigned_uav_count": 0,
            "modeling_facade_lane_count": 0,
            "modeling_facade_progress_ratio": 0,
            "modeling_distance_m": 0,
            "modeling_interrupted_search_tasks": 0,
            "modeling_resumed_search_tasks": 0,
            "modeling_unreachable_facade_lanes": 0,
            "modeling_no_fly_violations": 0,
        }
        self._idle_reason_per_uav: dict[str, str] = {}
        self._idle_since_by_uav: dict[str, float] = {}
        self._assist_task_seq = 0
        self._reachability: ReachabilityIndex = build_reachability_index(self.grid_map, self.fleet.get_all_states())
        self._unreachable_components: list[set[Position]] = reachability_components(
            self.grid_map,
            self._reachability.unreachable_searchable_cells,
        )
        self._blocked_region_cache: dict[tuple[tuple[int, int], ...], float] = {}
        self._last_decision_time = 0.0
        self.replan_count = 0

    def target_metrics_snapshot(self) -> dict[str, dict[str, Any]]:
        return {target_id: dict(record) for target_id, record in self._target_metrics.items()}

    def reachability_diagnostics(self) -> dict[str, Any]:
        return {
            "unreachable_cells_count": len(self._reachability.unreachable_searchable_cells),
            "unreachable_components_count": len(self._unreachable_components),
            "unreachable_components": [
                {"size": len(component), "sample": {"x": min(component).x, "y": min(component).y}}
                for component in sorted(self._unreachable_components, key=lambda item: (-len(item), min(item)))[:20]
                if component
            ],
        }

    def task_status_snapshot(self) -> dict[str, Any]:
        status_counts = {
            "pending": 0,
            "assigned": 0,
            "in_progress": 0,
            "completed": 0,
            "blocked": 0,
            "cancelled": 0,
        }
        for task in self.task_manager.tasks.values():
            key = task.status.value.lower()
            status_counts[key] = status_counts.get(key, 0) + 1
        confirmations = [
            {
                "task_id": task_id,
                "target_id": str(record.get("target_id")),
                "uav_id": str(record.get("uav_id")),
                "status": str(record.get("status", "CONFIRMING")),
                "interrupted_task_id": record.get("interrupted_task_id"),
            }
            for task_id, record in sorted(self._confirmations.items())
        ]
        assist_tasks = [
            {
                "task_id": task.id,
                "status": task.status.value,
                "donor_task_id": task.metadata.get("donor_task_id"),
                "donor_uav_id": task.metadata.get("donor_uav_id"),
                "helper_uav_id": task.metadata.get("helper_uav_id"),
                "assist_region_cell_count": len(task.metadata.get("assist_region_cells", []) or []),
            }
            for task in sorted(self.task_manager.tasks.values(), key=lambda item: item.id)
            if task.metadata.get("assist_task") is True
        ]
        modeling_tasks = [
            {
                "task_id": task.id,
                "status": task.status.value,
                "building_id": task.metadata.get("building_id"),
                "job_id": task.metadata.get("job_id"),
                "uav_id": task.assigned_uav_id,
                "facade_lane_ids": list(task.metadata.get("facade_lane_ids", []) or []),
                "progress": task.progress,
                "resume_search_after": bool(task.metadata.get("resume_search_after", True)),
                "interrupted_task_id": task.metadata.get("interrupted_task_id"),
                "footprint": task.metadata.get("footprint", []),
                "logical_waypoints": task.metadata.get("logical_waypoints", []),
            }
            for task in sorted(self.task_manager.tasks.values(), key=lambda item: item.id)
            if task.type == TaskType.MODELING
        ]
        modeling_jobs = [dict(job) for _, job in sorted(self._modeling_jobs.items())]
        return {
            "status_counts": status_counts,
            "confirmations": confirmations,
            "assist_tasks": assist_tasks,
            "modeling_tasks": modeling_tasks,
            "modeling_jobs": modeling_jobs,
        }

    def diagnostics_snapshot(self) -> dict[str, Any]:
        return {**self._diagnostics, "idle_reason_per_uav": dict(self._idle_reason_per_uav)}

    def remember_control_commands(self, commands: list[ControlCommand]) -> None:
        for command in commands:
            self._issued_commands[command.command_id] = command

    def record_issued_commands(self, commands: list[ControlCommand]) -> None:
        self.remember_control_commands(commands)

    def handle_command_acks(self, command_acks: list[CommandAck]) -> list[DecisionCommand]:
        commands: list[DecisionCommand] = []
        self._ack_events_handled = []
        for ack in command_acks:
            key = (ack.command_id, ack.status.value, ack.updated_at)
            if key in self._handled_command_ack_keys:
                continue
            self._handled_command_ack_keys.add(key)
            command = self._issued_commands.get(ack.command_id)
            if command is None:
                continue
            if ack.status in (AckStatus.REJECTED, AckStatus.FAILED, AckStatus.CANCELLED):
                self._handle_command_not_executed(command, ack)
            elif ack.status in (AckStatus.ACCEPTED, AckStatus.RUNNING):
                self._handle_command_started(command, ack)
            elif ack.status == AckStatus.COMPLETED:
                commands.extend(self._handle_command_completed(command, ack))
        return commands

    def pop_ack_events_handled(self) -> list[str]:
        events = list(self._ack_events_handled)
        self._ack_events_handled = []
        return events

    def _handle_command_started(self, command: ControlCommand, ack: CommandAck) -> None:
        if command.command == CommandType.CONFIRM_TARGET and command.task_id in self._confirmations:
            self._confirmations[command.task_id]["status"] = "CONFIRMING"
            return
        if command.task_id is None or command.task_id not in self.task_manager.tasks:
            return
        task = self.task_manager.tasks[command.task_id]
        if command.command == CommandType.MODEL_STRUCTURE and task.type == TaskType.MODELING:
            if task.status == TaskStatus.ASSIGNED:
                self.task_manager.start_task(task.id, now=ack.updated_at)
            return
        if command.command in (CommandType.FOLLOW_PATH, CommandType.REPLAN) and task.type == TaskType.SEARCH:
            if task.status == TaskStatus.ASSIGNED:
                self.task_manager.start_task(task.id, now=ack.updated_at)

    def _handle_command_not_executed(self, command: ControlCommand, ack: CommandAck) -> None:
        if command.command == CommandType.CONFIRM_TARGET:
            self._fail_confirmation_from_ack(command, ack)
            return
        if command.command == CommandType.MODEL_STRUCTURE:
            self._fail_modeling_from_ack(command, ack)
            return
        if command.task_id is None or command.task_id not in self.task_manager.tasks:
            return
        task = self.task_manager.tasks[command.task_id]
        if task.type == TaskType.SEARCH and task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
            task.assigned_uav_id = None
            task.status = TaskStatus.PENDING
            task.updated_at = ack.updated_at
        elif task.status == TaskStatus.ASSIGNED:
            self.task_manager.mark_blocked(task.id, now=ack.updated_at)

    def _handle_command_completed(self, command: ControlCommand, ack: CommandAck) -> list[DecisionCommand]:
        if command.command == CommandType.CONFIRM_TARGET:
            return self._complete_confirmation_from_ack(command, ack)
        if command.command == CommandType.MODEL_STRUCTURE:
            return self._complete_modeling_from_ack(command, ack)
        if command.command == CommandType.RETURN_HOME:
            return []
        if command.task_id is None or command.task_id not in self.task_manager.tasks:
            return []
        task = self.task_manager.tasks[command.task_id]
        if task.type == TaskType.SEARCH and task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
            self.task_manager.complete_task(task.id, now=ack.updated_at)
        return []

    def _complete_modeling_from_ack(self, command: ControlCommand, ack: CommandAck) -> list[DecisionCommand]:
        if command.task_id is None or command.task_id not in self.task_manager.tasks:
            return []
        task = self.task_manager.tasks[command.task_id]
        if task.type != TaskType.MODELING:
            return []
        if task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
            self.task_manager.complete_task(task.id, now=ack.updated_at)

        job_id = str(task.metadata.get("job_id", ""))
        job = self._modeling_jobs.get(job_id)
        if not job:
            return []
        completed = sum(
            1
            for task_id in job.get("task_ids", [])
            if task_id in self.task_manager.tasks and self.task_manager.tasks[task_id].status == TaskStatus.COMPLETED
        )
        total = max(1, len(job.get("task_ids", [])))
        self._diagnostics["modeling_facade_progress_ratio"] = int((completed / total) * 100)
        if completed < total:
            return []

        if job.get("status") != "COMPLETED":
            job["status"] = "COMPLETED"
            job["completed_at"] = ack.updated_at
            self._diagnostics["modeling_jobs_completed"] += 1
            self._diagnostics["modeling_active_jobs"] = max(0, self._diagnostics["modeling_active_jobs"] - 1)
            event_id = f"modeling_done_{job_id}"
            self._ack_events_handled.append(event_id)

        if not bool(job.get("resume_search_after", True)):
            return [
                DecisionCommand(
                    uav_id=command.uav_id,
                    command=CommandType.HOLD,
                    task_id=command.task_id,
                    target=ack and task.entry_point,
                    path=[],
                    reason="modeling_done",
                )
            ]

        resume_commands: list[DecisionCommand] = []
        interrupted_by_uav = job.get("interrupted_task_by_uav", {})
        for uav_id, interrupted_task_id in interrupted_by_uav.items():
            if interrupted_task_id is None:
                continue
            uav = self.fleet.get_uav(str(uav_id)).state
            resume = self._resume_interrupted_search_after_confirm(
                {"interrupted_task_id": interrupted_task_id, "target_id": job.get("building_id", job_id)},
                uav,
                ack.updated_at,
            )
            if resume is not None:
                self._diagnostics["modeling_resumed_search_tasks"] += 1
                resume_commands.append(resume)
        return resume_commands

    def _fail_modeling_from_ack(self, command: ControlCommand, ack: CommandAck) -> None:
        if command.task_id is None or command.task_id not in self.task_manager.tasks:
            return
        task = self.task_manager.tasks[command.task_id]
        if task.type != TaskType.MODELING:
            return
        if task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
            self.task_manager.mark_blocked(task.id, now=ack.updated_at)
        job_id = str(task.metadata.get("job_id", ""))
        job = self._modeling_jobs.get(job_id)
        if job and job.get("status") != "FAILED":
            job["status"] = "FAILED"
            job["failed_at"] = ack.updated_at
            job["failure_reason"] = ack.reason or ack.status.value
            self._diagnostics["modeling_jobs_failed"] += 1
            self._diagnostics["modeling_active_jobs"] = max(0, self._diagnostics["modeling_active_jobs"] - 1)
            self._ack_events_handled.append(f"modeling_failed_{job_id}")

    def _complete_confirmation_from_ack(self, command: ControlCommand, ack: CommandAck) -> list[DecisionCommand]:
        task_id = command.task_id or ""
        confirmation = self._confirmations.get(task_id)
        target_id = str(
            (confirmation or {}).get("target_id")
            or command.metadata.get("target_id")
            or task_id.removeprefix("confirm_")
        )
        event = Event(
            id=f"confirm_done_{task_id}",
            type=EventType.CONFIRM_DONE,
            timestamp=ack.updated_at,
            priority=EventPriority.NORMAL,
            source_uav_id=command.uav_id,
            data={"task_id": task_id, "target_id": target_id},
        )
        self._ack_events_handled.append(event.id)
        return self._handle_confirm_done(event)

    def _fail_confirmation_from_ack(self, command: ControlCommand, ack: CommandAck) -> None:
        confirmation = self._confirmations.pop(command.task_id or "", None)
        target_id = str(
            (confirmation or {}).get("target_id")
            or (command.metadata or {}).get("target_id")
            or (command.task_id or "").removeprefix("confirm_")
        )
        if target_id:
            record = self._target_metrics.setdefault(target_id, {"target_id": target_id, "found_time_s": ack.updated_at})
            record.update(
                {
                    "failed_time_s": ack.updated_at,
                    "success": False,
                    "failure_reason": ack.reason or ack.status.value,
                }
            )
        interrupted_task_id = (confirmation or {}).get("interrupted_task_id")
        if interrupted_task_id in self.task_manager.tasks:
            task = self.task_manager.tasks[interrupted_task_id]
            if task.status == TaskStatus.IN_PROGRESS:
                task.status = TaskStatus.PENDING
                task.assigned_uav_id = None

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
        self._last_decision_time = now
        events_handled: list[str] = []
        commands: list[DecisionCommand] = []

        # 步骤1: 处理待处理的事件（高优先级事件优先处理）
        urgent_commands, urgent_event_ids = self.handle_urgent_events(self.event_manager.poll_events(now))
        commands.extend(urgent_commands)
        events_handled.extend(urgent_event_ids)

        # 步骤2: 确保初始任务已生成（仅首次执行）
        self._ensure_initial_tasks(now)
        commands.extend(self._refresh_task_progress(now))
        self._ensure_supplemental_search_tasks(now)
        commands.extend(self._try_assign_idle_assist_tasks(now))

        # 步骤3: 执行任务分配
        assignments = []
        reserved_uav_ids = {
            command.uav_id
            for command in commands
            if command.command in (CommandType.CONFIRM_TARGET, CommandType.MODEL_STRUCTURE, CommandType.RETURN_HOME, CommandType.HOLD)
        }
        available_uavs = [
            state for state in self.fleet.get_available_uavs()
            if state.id not in reserved_uav_ids
        ]
        proposed_assignments = self.auction.allocate(
            self._allocatable_pending_tasks(),
            available_uavs,
            self.grid_map,
            now=now,
        )

        # 步骤4: 为每个分配的任务规划路径并下发指令
        for proposed in proposed_assignments:
            task = self.task_manager.tasks[proposed.task_id]
            assignment = self.task_manager.assign_task(task.id, proposed.uav_id, now=now, bid_value=proposed.bid_value)
            uav_state = self.fleet.get_uav(proposed.uav_id).state

            coverage_waypoints = reorder_waypoints_for_uav(
                self._task_coverage_waypoints(task),
                uav_state.position,
            )
            task.coverage_waypoints = coverage_waypoints
            task.waypoints = list(coverage_waypoints)
            task.entry_point = coverage_waypoints[0]
            route = self._plan_route_through_waypoints(uav_state, coverage_waypoints)
            if not route:
                self._remember_blocked_region(task, now)
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
            assignments.append(assignment)
            self._clear_idle_reason(uav_state.id)
            if task.metadata.get("assist_task") is True:
                self._diagnostics["idle_assist_accepted_tasks"] += 1
            commands.append(
                DecisionCommand(
                    uav_id=uav_state.id,
                    command=CommandType.FOLLOW_PATH,
                    task_id=task.id,
                    target=coverage_waypoints[-1],
                    path=route,
                    reason="auction_search_task",
                    metadata=self._command_metadata_for_task(task, coverage_waypoints),
                )
            )

        # 步骤5: 检测并消解冲突
        # 冲突检测：检查所有无人机路径是否存在碰撞风险
        planning_states = self._states_with_pending_commands(commands)
        conflicts = detect_conflicts(
            planning_states,
            safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            time_horizon_steps=int(self.config["planning"]["conflict_time_horizon_steps"]),
        )
        # 冲突消解：为低优先级无人机添加等待指令
        commands.extend(
            resolve_conflicts(
                conflicts,
                planning_states,
                safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            )
        )

        commands.extend(self._dispatch_completed_search_returns(now))

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
        if event.type == EventType.UAV_RECOVERED:
            self._refresh_reachability()
            return []
        if event.type == EventType.MAP_UPDATE:
            return self._handle_map_update(event)
        if event.type == EventType.TARGET_FOUND:
            return self._handle_target_found(event)
        if event.type == EventType.BUILDING_MODEL_REQUEST:
            return self._handle_building_model_request(event)
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

        在每个仿真步进后调用，检查目标确认任务和返航任务的完成条件。
        当无人机完成目标周边盘旋航线并满足确认步数后，触发确认完成事件。

        参数：
            now: 当前时间戳（秒）

        返回：
            tuple: (决策指令列表, 已处理的事件ID列表)

        设计思路：
            - 目标确认需要无人机抵近到目标周边并完成盘旋航线
            - 确认步数由配置参数 confirm_duration_steps 决定
            - 完成后触发 CONFIRM_DONE 事件
        """
        commands: list[DecisionCommand] = []
        handled_ids: list[str] = []
        self._refresh_task_progress(now)
        for task_id, confirmation in list(self._confirmations.items()):
            uav = self.fleet.get_uav(confirmation["uav_id"]).state

            # 盘旋确认以“完成目标周边航线”为完成条件，而不是停在目标格上。
            if uav.status != UAVStatus.CONFIRMING or uav.path_index < len(uav.path) - 1:
                continue

            if confirmation.get("dwell_started_at") is None:
                step_s = float(self.config.get("simulation", {}).get("time_step_s", 1.0))
                confirmation["dwell_started_at"] = now - step_s
            dwell_s = float(confirmation.get("dwell_s", 0.0))
            if now - float(confirmation["dwell_started_at"]) < dwell_s:
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
        commands.extend(self._dispatch_completed_search_returns(now))
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
        self._refresh_reachability()
        active_states = [state for state in states if state.status != UAVStatus.OFFLINE]
        searchable_cells = set(self.grid_map.get_searchable_cells()) - self._reachability.unreachable_searchable_cells
        tasks = self.coverage_planner.plan_initial_tasks(
            grid_map=self.grid_map,
            uav_states=active_states,
            sensor_radius_cells=int(self.config["uav"]["sensor_radius_cells"]),
            created_at=now,
            reachability=self._reachability,
            searchable_cells=searchable_cells,
        )
        for task in tasks:
            if task.allowed_uav_ids is None:
                self._set_allowed_uavs_for_task(task)
        self.task_manager.add_tasks(tasks)
        self._initialized = True

    def _refresh_reachability(self) -> None:
        self._reachability = build_reachability_index(self.grid_map, self.fleet.get_all_states())
        self._unreachable_components = reachability_components(
            self.grid_map,
            self._reachability.unreachable_searchable_cells,
        )

    def _set_allowed_uavs_for_task(self, task: Task) -> None:
        allowed = set(self._reachability.reachable_uavs(task.entry_point))
        task.allowed_uav_ids = allowed or None

    def _region_signature(self, cells: set[Position]) -> tuple[tuple[int, int], ...]:
        return tuple(sorted((cell.x, cell.y) for cell in cells))

    def _blocked_region_ttl_s(self) -> float:
        return float(self.config.get("search", {}).get("blocked_region_cache_ttl_s", 60.0))

    def _latest_snapshot_time(self) -> float:
        return self._last_decision_time

    def _remember_blocked_region(self, task: Task, now: float) -> None:
        if task.type != TaskType.SEARCH or not task.target_cells:
            return
        self._blocked_region_cache[self._region_signature(set(task.target_cells))] = now

    def _prune_blocked_region_cache(self, now: float) -> None:
        ttl_s = self._blocked_region_ttl_s()
        self._blocked_region_cache = {
            signature: blocked_at
            for signature, blocked_at in self._blocked_region_cache.items()
            if now - blocked_at <= ttl_s
        }

    def _is_region_blocked(self, region: set[Position], now: float) -> bool:
        self._prune_blocked_region_cache(now)
        return self._region_signature(region) in self._blocked_region_cache

    def _positions_to_dicts(self, positions: list[Position]) -> list[dict[str, int]]:
        return [{"x": point.x, "y": point.y} for point in positions]

    def _command_metadata_for_task(self, task: Task, logical_waypoints: list[Position]) -> dict[str, Any]:
        metadata = dict(task.metadata)
        metadata["logical_waypoints"] = self._positions_to_dicts(logical_waypoints)
        return metadata

    def _refresh_task_progress(self, now: float) -> list[DecisionCommand]:
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        self.task_manager.update_progress(self.grid_map, now=now, coverage_threshold=coverage_threshold)
        self.task_manager.refresh_pending_waypoints(self.grid_map, now=now, coverage_threshold=coverage_threshold)
        self._trim_pending_search_waypoints_against_reserved(now, coverage_threshold)
        commands = self._stop_search_tasks_after_coverage_goal(now)
        commands.extend(self._trim_redundant_active_search_paths(now, coverage_threshold))
        self._complete_finished_search_tasks(now)
        return commands

    def _stop_search_tasks_after_coverage_goal(self, now: float) -> list[DecisionCommand]:
        if not self._coverage_goal_reached():
            return []
        commands: list[DecisionCommand] = []
        commanded_uavs: set[str] = set()
        for task in list(self.task_manager.tasks.values()):
            if task.type != TaskType.SEARCH or task.status in (TaskStatus.COMPLETED, TaskStatus.CANCELLED):
                continue
            if self._task_has_priority_remaining(task):
                continue
            uav: UAVState | None = None
            if task.assigned_uav_id is not None:
                uav = self.fleet.get_uav(task.assigned_uav_id).state
            self.task_manager.complete_task(task.id, now=now)
            self._diagnostics["cancelled_post_goal_tasks"] += 1
            if uav is None or uav.status != UAVStatus.SEARCHING:
                continue
            commanded_uavs.add(uav.id)
            self._diagnostics["post_goal_active_search_cancel_count"] += 1
            commands.append(
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.HOLD,
                    task_id=task.id,
                    target=uav.position,
                    path=[],
                    reason="mission_coverage_goal_met",
                    metadata={"post_goal_stop": True},
                )
            )
        for uav in self.fleet.get_all_states():
            if uav.id in commanded_uavs or uav.status != UAVStatus.SEARCHING:
                continue
            commands.append(
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.HOLD,
                    task_id=uav.current_task_id,
                    target=uav.position,
                    path=[],
                    reason="mission_coverage_goal_met",
                )
            )
        return commands

    def should_run_regular_cycle(self) -> bool:
        return self.event_manager.has_events() or (
            bool(self._allocatable_pending_tasks()) and bool(self.fleet.get_available_uavs())
        ) or (
            bool(self.fleet.get_available_uavs()) and bool(self._get_supplemental_candidates())
        ) or (
            self._has_idle_assist_opportunity()
        ) or (
            self._coverage_goal_reached() and self._has_active_search_uavs()
        )

    def _has_active_search_uavs(self) -> bool:
        return any(state.status == UAVStatus.SEARCHING for state in self.fleet.get_all_states())

    def _allocatable_pending_tasks(self) -> list[Task]:
        tasks: list[Task] = []
        states_by_id = {state.id: state for state in self.fleet.get_all_states()}
        for task in self.task_manager.get_pending_tasks():
            if self._coverage_goal_reached() and task.type == TaskType.SEARCH and not self._task_has_priority_remaining(task):
                continue
            if task.resume_owner_id is None:
                tasks.append(task)
                continue
            owner = states_by_id.get(task.resume_owner_id)
            if owner is not None and owner.available and owner.status == UAVStatus.IDLE:
                tasks.append(task)
        return tasks

    def _states_with_pending_commands(self, commands: list[DecisionCommand]) -> list[UAVState]:
        states = {state.id: replace(state, path=list(state.path)) for state in self.fleet.get_all_states()}
        for command in commands:
            if command.uav_id not in states or not command.path:
                continue
            if command.command == CommandType.FOLLOW_PATH:
                status = UAVStatus.SEARCHING
            elif command.command == CommandType.CONFIRM_TARGET:
                status = UAVStatus.CONFIRMING
            elif command.command == CommandType.MODEL_STRUCTURE:
                status = UAVStatus.MODELING
            elif command.command == CommandType.RETURN_HOME:
                status = UAVStatus.RETURNING
            elif command.command == CommandType.REPLAN:
                status = UAVStatus(str(command.metadata.get("status", states[command.uav_id].status.value)))
            elif command.command == CommandType.CONFLICT_YIELD:
                status = states[command.uav_id].status
            else:
                continue
            states[command.uav_id].path = list(command.path)
            states[command.uav_id].path_index = 0
            states[command.uav_id].status = status
        return list(states.values())

    def _complete_finished_search_tasks(self, now: float) -> None:
        for task in self.task_manager.get_active_tasks():
            if task.type != TaskType.SEARCH or task.assigned_uav_id is None:
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            path_finished = bool(uav.path) and uav.path_index >= len(uav.path) - 1
            if uav.current_task_id == task.id and uav.status == UAVStatus.IDLE and path_finished:
                self.task_manager.complete_task(task.id, now=now)

    def _ensure_supplemental_search_tasks(self, now: float) -> None:
        available_uavs = self.fleet.get_available_uavs()
        if not available_uavs:
            return
        if self._coverage_goal_reached():
            self._cancel_pending_ordinary_supplemental(now)
            self._diagnostics["skipped_post_goal_supplemental_tasks"] += 1
            return
        if self._supplemental_task_seq >= int(self.config["search"].get("max_supplemental_tasks_per_run", 1_000_000)):
            self._diagnostics["skipped_low_gain_supplemental_count"] += 1
            return
        if any(task.type == TaskType.SEARCH for task in self._allocatable_pending_tasks()):
            return

        candidates = self._get_supplemental_candidates()
        if not candidates:
            return

        tasks: list[Task] = []
        for candidate in candidates:
            if len(tasks) >= len(available_uavs):
                break
            self._supplemental_task_seq += 1
            task_id = f"supplemental_{self._supplemental_task_seq:03d}"
            large_region = candidate.uncovered_cells >= int(self.config["search"].get("large_supplemental_region_cells", 16))
            planned_task = None
            if large_region:
                planned_task = self.coverage_planner.plan_region_task(
                    task_id=task_id,
                    region=set(candidate.cells),
                    origin=candidate.entry_point,
                    grid_map=self.grid_map,
                    sensor_radius_cells=int(self.config["uav"]["sensor_radius_cells"]),
                    created_at=now,
                    reachability=self._reachability,
                    allowed_uav_ids=set(candidate.allowed_uav_ids) if candidate.allowed_uav_ids else None,
                )
                if planned_task is not None:
                    planned_task.metadata.update(
                        {
                            "supplemental": True,
                            "gain_per_meter": candidate.gain_per_meter,
                            "new_coverage_gain": candidate.uncovered_cells,
                            "estimated_connector_cost_m": candidate.nearest_uav_distance,
                        }
                    )
            tasks.append(
                planned_task
                or Task(
                        id=task_id,
                        type=TaskType.SEARCH,
                        priority=max(self.grid_map.get_cell(cell).search_priority for cell in candidate.cells),
                        target_cells=set(candidate.cells),
                        entry_point=candidate.entry_point,
                        waypoints=candidate.waypoints,
                        coverage_waypoints=list(candidate.waypoints),
                        estimated_cost_m=candidate.estimated_cost_m,
                        created_at=now,
                        updated_at=now,
                        uncovered_value=candidate.uncovered_value,
                        priority_value=candidate.priority_value,
                        score=candidate.score,
                        allowed_uav_ids=set(candidate.allowed_uav_ids) if candidate.allowed_uav_ids else None,
                        metadata={
                            "supplemental": True,
                            "gain_per_meter": candidate.gain_per_meter,
                            "new_coverage_gain": candidate.uncovered_cells,
                            "estimated_connector_cost_m": candidate.nearest_uav_distance,
                        },
                    )
            )
        self.task_manager.add_tasks(tasks)

    def _try_assign_idle_assist_tasks(self, now: float) -> list[DecisionCommand]:
        if not bool(self.config["search"].get("idle_assist_enabled", True)):
            return []
        idle_uavs = [uav for uav in self.fleet.get_available_uavs() if uav.status == UAVStatus.IDLE]
        if not idle_uavs or self._mission_goal_met():
            return []
        pending_count = sum(1 for task in self._allocatable_pending_tasks() if task.type == TaskType.SEARCH)
        if pending_count >= len(idle_uavs):
            for uav in idle_uavs:
                self._set_idle_reason(uav.id, "waiting_for_other_uavs", now)
            return []

        commands: list[DecisionCommand] = []
        created = 0
        for idle_uav in idle_uavs:
            if created >= int(self.config["search"].get("idle_assist_max_tasks_per_cycle", 2)):
                break
            self._diagnostics["idle_assist_attempts"] += 1
            plan = self._build_idle_assist_plan(idle_uav, now)
            if plan is None:
                self._set_idle_reason(idle_uav.id, "no_valuable_assist_region", now)
                continue
            assist_task, donor_task, donor_uav, donor_waypoints = plan
            self.task_manager.add_tasks([assist_task])
            self._diagnostics["idle_assist_created_tasks"] += 1
            self._diagnostics["idle_assist_cells_reassigned"] += len(assist_task.target_cells)
            self._idle_reason_per_uav.pop(idle_uav.id, None)
            created += 1
            if bool(self.config["search"].get("idle_assist_replan_donor", True)) and donor_waypoints:
                self._set_task_coverage_waypoints(donor_task, donor_waypoints, now)
                donor_route = self._plan_route_through_waypoints(donor_uav, donor_waypoints)
                if donor_route:
                    donor_task.last_replan_time = now
                    donor_task.replan_count += 1
                    self.replan_count += 1
                    self._diagnostics["idle_assist_donor_replans"] += 1
                    commands.append(
                        DecisionCommand(
                            uav_id=donor_uav.id,
                            command=CommandType.REPLAN,
                            task_id=donor_task.id,
                            target=donor_waypoints[-1],
                            path=donor_route,
                            reason="idle_assist_donor_replan",
                            metadata={
                                **self._command_metadata_for_task(donor_task, donor_waypoints),
                                "status": UAVStatus.SEARCHING.value,
                                "assist_task_id": assist_task.id,
                            },
                        )
                    )
        if created == 0 and idle_uavs:
            for uav in idle_uavs:
                self._idle_reason_per_uav.setdefault(uav.id, "no_pending_tasks")
        return commands

    def _build_idle_assist_plan(
        self,
        idle_uav: UAVState,
        now: float,
    ) -> tuple[Task, Task, UAVState, list[Position]] | None:
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        min_remaining = int(self.config["search"].get("idle_assist_min_remaining_cells", 40))
        min_gain = float(self.config["search"].get("idle_assist_min_gain_per_meter", 0.02))
        keep_front = max(0, int(self.config["search"].get("idle_assist_donor_keep_front_waypoints", 5)))
        reserved = self._reserved_search_cells_for_idle_assist()
        active_tasks = sorted(
            (
                task
                for task in self.task_manager.get_active_tasks()
                if task.type == TaskType.SEARCH and task.assigned_uav_id is not None and not task.metadata.get("assist_task")
            ),
            key=lambda task: (task.updated_at, task.id),
        )
        for donor_task in active_tasks:
            donor_uav = self.fleet.get_uav(donor_task.assigned_uav_id).state
            if donor_uav.status != UAVStatus.SEARCHING or not donor_uav.path:
                continue
            donor_reserved = set(reserved)
            donor_reserved.difference_update(self._task_remaining_cells_from_path(donor_task, donor_uav, coverage_threshold, keep_front))
            remaining = self._task_remaining_cells_from_path(donor_task, donor_uav, coverage_threshold, keep_front)
            remaining = {cell for cell in remaining if cell not in donor_reserved}
            if len(remaining) < min_remaining:
                continue
            component = self._select_assist_component(remaining, idle_uav, min_remaining)
            if not component:
                continue
            if not all(idle_uav.id in self._reachability.reachable_uavs(cell) for cell in component):
                self._diagnostics["idle_assist_rejected_unreachable"] += 1
                continue
            assist_task = self._build_assist_task(component, idle_uav, donor_task, donor_uav, now)
            if assist_task is None:
                self._diagnostics["idle_assist_rejected_unreachable"] += 1
                continue
            helper_route = self._plan_route_through_waypoints(idle_uav, assist_task.coverage_waypoints)
            if not helper_route:
                self._diagnostics["idle_assist_rejected_unreachable"] += 1
                continue
            gain_per_meter = len(component) / max(self._route_distance_m(helper_route), 1.0)
            if gain_per_meter < min_gain:
                self._diagnostics["idle_assist_rejected_low_gain"] += 1
                continue
            assist_task.metadata["gain_per_meter"] = gain_per_meter
            helper_distance_m = self._route_distance_m(helper_route)
            assist_task.metadata["estimated_connector_cost_m"] = helper_distance_m
            assist_task.score = gain_per_meter
            self._diagnostics["idle_assist_distance_m"] += int(helper_distance_m)
            donor_waypoints = self._donor_waypoints_after_assist(donor_task, component, donor_uav, keep_front)
            if donor_waypoints:
                return assist_task, donor_task, donor_uav, donor_waypoints
        return None

    def _build_assist_task(
        self,
        region: set[Position],
        idle_uav: UAVState,
        donor_task: Task,
        donor_uav: UAVState,
        now: float,
    ) -> Task | None:
        self._assist_task_seq += 1
        task_id = f"assist_{self._assist_task_seq:03d}"
        task = self.coverage_planner.plan_region_task(
            task_id=task_id,
            region=region,
            origin=idle_uav.position,
            grid_map=self.grid_map,
            sensor_radius_cells=idle_uav.sensor_radius_cells,
            created_at=now,
            reachability=self._reachability,
            allowed_uav_ids={idle_uav.id},
        )
        if task is None:
            return None
        task.allowed_uav_ids = {idle_uav.id}
        task.metadata.update(
            {
                "assist_task": True,
                "donor_task_id": donor_task.id,
                "donor_uav_id": donor_uav.id,
                "helper_uav_id": idle_uav.id,
                "assist_region_cells": self._positions_to_dicts(sorted(region)),
            }
        )
        return task

    def _task_remaining_cells_from_path(
        self,
        task: Task,
        uav: UAVState,
        coverage_threshold: float,
        skip_front_waypoints: int,
    ) -> set[Position]:
        path = uav.path[uav.path_index + skip_front_waypoints :] if uav.path else []
        cells = self._path_coverage_footprint(path, uav.sensor_radius_cells)
        return {
            cell
            for cell in cells
            if cell in task.target_cells
            and self.grid_map.is_passable(cell)
            and self.grid_map.get_cell(cell).search_confidence < coverage_threshold
        }

    def _reserved_search_cells_for_idle_assist(self) -> set[Position]:
        reserved: set[Position] = set()
        for task in self.task_manager.get_pending_tasks():
            if task.type == TaskType.SEARCH:
                reserved.update(task.target_cells)
        for task in self.task_manager.get_active_tasks():
            if task.type != TaskType.SEARCH or task.assigned_uav_id is None or task.metadata.get("assist_task"):
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            if uav.status == UAVStatus.SEARCHING and uav.path:
                reserved.update(self._path_coverage_footprint(uav.path[uav.path_index :], uav.sensor_radius_cells))
        return reserved

    def _select_assist_component(
        self,
        remaining: set[Position],
        idle_uav: UAVState,
        min_cells: int,
    ) -> set[Position]:
        components = [component for component in connected_components(remaining, self.grid_map) if len(component) >= min_cells]
        if not components:
            return set()
        component = min(
            components,
            key=lambda component: (
                min(abs(cell.x - idle_uav.position.x) + abs(cell.y - idle_uav.position.y) for cell in component),
                -len(component),
            ),
        )
        max_cells = max(
            min_cells,
            int(
                self.config["search"].get(
                    "idle_assist_max_region_cells",
                    self.config["search"].get(
                        "supplemental_cluster_max_cells",
                        self.config["search"].get("supplemental_task_max_cells", 80),
                    ),
                )
            ),
        )
        if len(component) <= max_cells:
            return component
        seed = min(component, key=lambda cell: abs(cell.x - idle_uav.position.x) + abs(cell.y - idle_uav.position.y))
        region = self._collect_supplemental_region(seed, set(component), max_cells)
        return region if len(region) >= min_cells else component

    def _donor_waypoints_after_assist(
        self,
        donor_task: Task,
        assist_region: set[Position],
        donor_uav: UAVState,
        keep_front: int,
    ) -> list[Position]:
        waypoints = list(donor_uav.path[donor_uav.path_index :]) if donor_uav.path else self._task_coverage_waypoints(donor_task)
        front = waypoints[:keep_front]
        radius = donor_uav.sensor_radius_cells
        kept = [
            waypoint
            for index, waypoint in enumerate(waypoints)
            if index < keep_front or not self._point_intersects_cells(waypoint, radius, assist_region)
        ]
        if not kept:
            return front
        return kept

    def _set_idle_reason(self, uav_id: str, reason: str, now: float) -> None:
        self._idle_reason_per_uav[uav_id] = reason
        started_at = self._idle_since_by_uav.setdefault(uav_id, now)
        self._diagnostics["idle_uav_wait_time_s"] = int(
            max(float(self._diagnostics.get("idle_uav_wait_time_s", 0)), now - started_at)
        )

    def _clear_idle_reason(self, uav_id: str) -> None:
        self._idle_reason_per_uav.pop(uav_id, None)
        self._idle_since_by_uav.pop(uav_id, None)

    def _has_idle_assist_opportunity(self) -> bool:
        return (
            bool(self.config["search"].get("idle_assist_enabled", True))
            and not self._mission_goal_met()
            and any(uav.status == UAVStatus.IDLE for uav in self.fleet.get_available_uavs())
            and any(task.type == TaskType.SEARCH and task.assigned_uav_id is not None for task in self.task_manager.get_active_tasks())
        )

    def _collect_supplemental_region(self, seed: Position, candidates: set[Position], max_cells: int) -> set[Position]:
        region = {seed}
        queue = [seed]
        max_radius = int(self.config["search"].get("supplemental_cluster_radius_cells", 0))
        while queue and len(region) < max_cells:
            current = queue.pop(0)
            for neighbor in self.grid_map.get_neighbors(current, mode=4):
                if neighbor not in candidates or neighbor in region:
                    continue
                if max_radius > 0 and abs(neighbor.x - seed.x) + abs(neighbor.y - seed.y) > max_radius:
                    continue
                region.add(neighbor)
                queue.append(neighbor)
                if len(region) >= max_cells:
                    break
        return region

    def _get_unsearched_cells(self) -> list[Position]:
        threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        return self.grid_map.get_unsearched_cells(threshold=threshold)

    def _get_supplemental_candidates(self) -> list[SupplementalCandidate]:
        available_uavs = self.fleet.get_available_uavs()
        if not available_uavs:
            return []

        self._refresh_reachability()
        raw_candidates = {
            cell
            for cell in set(self._get_unsearched_cells()) - self._get_reserved_search_cells()
            if self._reachability.any_reachable(cell)
        }
        if not raw_candidates:
            return []
        now = self._latest_snapshot_time()
        self._prune_blocked_region_cache(now)

        max_cells = max(
            1,
            int(
                self.config["search"].get(
                    "supplemental_cluster_max_cells",
                    self.config["search"].get("supplemental_task_max_cells", 80),
                )
            ),
        )
        candidates: list[SupplementalCandidate] = []
        remaining = set(raw_candidates)
        while remaining:
            seed = min(remaining)
            region = self._collect_supplemental_region(seed, remaining, max_cells)
            remaining.difference_update(region)
            if self._is_region_blocked(region, now):
                continue
            candidate = self._build_supplemental_candidate(region, available_uavs)
            if candidate is not None and self._is_valuable_supplemental_candidate(candidate):
                candidates.append(candidate)

        return sorted(
            candidates,
            key=lambda item: (-item.score, -item.priority_value, item.nearest_uav_distance, -len(item.cells)),
        )

    def _build_supplemental_candidate(
        self,
        region: set[Position],
        available_uavs: list[UAVState],
    ) -> SupplementalCandidate | None:
        if not region:
            return None
        allowed_uav_ids = {
            uav_id
            for cell in region
            for uav_id in self._reachability.reachable_uavs(cell)
        }
        reachable_uavs = [state for state in available_uavs if state.id in allowed_uav_ids]
        if not reachable_uavs:
            return None
        raw_waypoints = generate_boustrophedon_path(region, int(self.config["uav"]["sensor_radius_cells"]))
        if not raw_waypoints:
            return None

        nearest_uav = min(
            reachable_uavs,
            key=lambda state: min(abs(cell.x - state.position.x) + abs(cell.y - state.position.y) for cell in region),
        )
        waypoints = reorder_waypoints_for_uav(raw_waypoints, nearest_uav.position)
        entry_point = waypoints[0]
        nearest_distance = (abs(entry_point.x - nearest_uav.position.x) + abs(entry_point.y - nearest_uav.position.y)) * self.grid_map.resolution_m
        internal_cost = estimate_task_cost(waypoints, entry_point, self.grid_map.resolution_m)
        estimated_cost_m = nearest_distance + internal_cost
        uncovered_value, priority_value = self._weighted_region_value(region)
        value = uncovered_value + priority_value
        cost = float(self.config["search"].get("distance_cost_weight", 1.0)) * max(estimated_cost_m, 1.0)
        score = value / cost
        gain_per_meter = len(region) / max(estimated_cost_m, 1.0)
        priority_uncovered_cells = sum(1 for cell in region if self.grid_map.get_cell(cell).search_priority > 1.0)
        return SupplementalCandidate(
            cells=set(region),
            uncovered_cells=len(region),
            priority_uncovered_cells=priority_uncovered_cells,
            uncovered_value=uncovered_value,
            priority_value=priority_value,
            nearest_uav_distance=nearest_distance,
            estimated_cost_m=estimated_cost_m,
            gain_per_meter=gain_per_meter,
            score=score,
            entry_point=entry_point,
            waypoints=waypoints,
            allowed_uav_ids=allowed_uav_ids,
        )

    def _weighted_region_value(self, region: set[Position]) -> tuple[float, float]:
        search_config = self.config["search"]
        uncovered_weight = float(search_config.get("uncovered_value_weight", 1.0))
        priority_cell_weight = float(search_config.get("priority_cell_weight", 3.0))
        priority_value_weight = float(search_config.get("priority_value_weight", 2.0))
        uncovered_value = float(len(region)) * uncovered_weight
        raw_priority_value = sum(max(0.0, self.grid_map.get_cell(cell).search_priority - 1.0) for cell in region)
        priority_value = raw_priority_value * priority_cell_weight * priority_value_weight
        return uncovered_value, priority_value

    def _is_valuable_supplemental_candidate(self, candidate: SupplementalCandidate) -> bool:
        search_config = self.config["search"]
        priority_exception = candidate.priority_uncovered_cells > 0 and not self._priority_goal_met()
        if priority_exception:
            return True
        if self._coverage_goal_reached():
            self._diagnostics["skipped_post_goal_supplemental_tasks"] += 1
            return False
        if self.grid_map.coverage_rate() >= float(search_config.get("late_stage_coverage_threshold", 0.93)):
            if candidate.priority_uncovered_cells > 0:
                self._diagnostics["late_stage_supplemental_count"] += 1
                return True
            if candidate.uncovered_cells < int(search_config.get("late_stage_min_supplemental_cells", 24)):
                self._diagnostics["skipped_low_gain_supplemental_count"] += 1
                return False
            if candidate.gain_per_meter < float(search_config.get("late_stage_min_gain_per_meter", 0.05)):
                self._diagnostics["skipped_low_gain_supplemental_count"] += 1
                return False
            self._diagnostics["late_stage_supplemental_count"] += 1
        early_min_cells = 1 if self.config.get("algorithm", {}).get("version") == "segment_sweep_v1" else int(search_config.get("min_supplemental_cells", 8))
        if candidate.uncovered_cells < early_min_cells:
            self._diagnostics["skipped_low_gain_supplemental_count"] += 1
            return False
        if candidate.uncovered_cells >= int(search_config.get("large_supplemental_region_cells", 16)):
            return True
        return candidate.score >= float(search_config.get("min_supplemental_score", 0.15))

    def _priority_goal_met(self) -> bool:
        threshold = float(self.config["search"].get("priority_complete_threshold", 0.98))
        priority_cells = self.grid_map.get_priority_cells()
        return not priority_cells or self.grid_map.coverage_rate(priority_only=True) >= threshold

    def _mission_goal_met(self) -> bool:
        if not self._coverage_goal_reached():
            return False
        return not self._has_valuable_supplemental_candidates()

    def _coverage_goal_reached(self) -> bool:
        global_threshold = float(self.config["search"].get("mission_complete_coverage_threshold", 0.95))
        return self.grid_map.coverage_rate() >= global_threshold and self._priority_goal_met()

    def _post_goal_ordinary_min_cells(self) -> int:
        return max(
            int(self.config["search"].get("large_supplemental_region_cells", 16)),
            int(self.config["search"].get("post_goal_ordinary_min_cells", 40)),
        )

    def _has_valuable_supplemental_candidates(self) -> bool:
        return bool(self._get_supplemental_candidates())

    def _task_has_priority_remaining(self, task: Task) -> bool:
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        return any(
            self.grid_map.get_cell(cell).search_priority > 1.0
            and self.grid_map.get_cell(cell).search_confidence < coverage_threshold
            for cell in task.target_cells
            if self.grid_map.is_passable(cell)
        )

    def _cancel_pending_ordinary_supplemental(self, now: float) -> None:
        for task in self.task_manager.get_pending_tasks():
            if task.type != TaskType.SEARCH or not str(task.id).startswith("supplemental_"):
                continue
            if self._task_has_priority_remaining(task):
                continue
            task.status = TaskStatus.CANCELLED
            task.updated_at = now
            self._diagnostics["cancelled_post_goal_tasks"] += 1

    def _get_reserved_search_cells(self) -> set[Position]:
        reserved: set[Position] = set()
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        for task in self.task_manager.tasks.values():
            if task.type != TaskType.SEARCH:
                continue
            if task.status == TaskStatus.PENDING:
                reserved.update(
                    cell
                    for cell in task.target_cells
                    if self.grid_map.get_cell(cell).search_confidence < coverage_threshold
                )
                continue
            if task.status not in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS) or task.assigned_uav_id is None:
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            if uav.status != UAVStatus.SEARCHING or not uav.path:
                continue
            # Reserve the sensor footprint of already committed remaining search paths so helpers do not duplicate it.
            reserved.update(self._path_coverage_footprint(uav.path[uav.path_index :], uav.sensor_radius_cells))
        return reserved

    def _path_coverage_footprint(self, path: list[Position], radius_cells: int) -> set[Position]:
        footprint: set[Position] = set()
        radius_sq = radius_cells * radius_cells
        for center in path:
            for y in range(center.y - radius_cells, center.y + radius_cells + 1):
                for x in range(center.x - radius_cells, center.x + radius_cells + 1):
                    pos = Position(x, y)
                    if not self.grid_map.is_passable(pos):
                        continue
                    if (x - center.x) ** 2 + (y - center.y) ** 2 > radius_sq:
                        continue
                    footprint.add(pos)
        return footprint

    def _task_coverage_waypoints(self, task: Task) -> list[Position]:
        waypoints = task.coverage_waypoints or task.waypoints
        return [point for point in waypoints if self.grid_map.is_passable(point)]

    def _set_task_coverage_waypoints(self, task: Task, waypoints: list[Position], now: float) -> None:
        task.coverage_waypoints = list(waypoints)
        task.waypoints = list(waypoints)
        if waypoints:
            task.entry_point = waypoints[0]
            task.estimated_cost_m = estimate_task_cost(waypoints, task.entry_point, self.grid_map.resolution_m)
        task.updated_at = now

    def _trim_redundant_active_search_paths(
        self,
        now: float,
        coverage_threshold: float,
        force_replan: bool = False,
    ) -> list[DecisionCommand]:
        commands: list[DecisionCommand] = []
        for task in self.task_manager.get_active_tasks():
            if task.type != TaskType.SEARCH or task.assigned_uav_id is None:
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            if uav.status != UAVStatus.SEARCHING or not uav.path:
                continue

            coverage_points = self._task_coverage_waypoints(task)
            useful_waypoints = [
                point
                for point in coverage_points
                if self._point_adds_search_coverage(point, uav.sensor_radius_cells, coverage_threshold)
            ]
            useful_waypoints = self._filter_post_goal_waypoints(task, useful_waypoints, uav.sensor_radius_cells, coverage_threshold)
            if len(useful_waypoints) == len(coverage_points) and not force_replan:
                continue

            if not useful_waypoints:
                self.task_manager.complete_task(task.id, now=now)
                commands.append(
                    DecisionCommand(
                        uav_id=uav.id,
                        command=CommandType.HOLD,
                        task_id=task.id,
                        target=uav.position,
                        path=[],
                        reason="search_task_no_remaining_coverage",
                    )
                )
                continue

            self._set_task_coverage_waypoints(task, useful_waypoints, now)
            if not self._should_replan_active_search(task, uav, coverage_points, useful_waypoints, now, force_replan):
                continue
            route = self._plan_route_through_waypoints(uav, useful_waypoints)
            if not route:
                self.task_manager.mark_blocked(task.id, now=now)
                continue
            task.last_replan_time = now
            task.replan_count += 1
            self.replan_count += 1
            commands.append(
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.REPLAN,
                    task_id=task.id,
                    target=useful_waypoints[-1],
                    path=route,
                    reason="active_search_trim_replan",
                    metadata={**self._command_metadata_for_task(task, useful_waypoints), "status": UAVStatus.SEARCHING.value},
                )
            )
        return commands

    def _should_replan_active_search(
        self,
        task: Task,
        uav: UAVState,
        previous_waypoints: list[Position],
        useful_waypoints: list[Position],
        now: float,
        force_replan: bool,
    ) -> bool:
        if force_replan:
            return True
        if not self.planner.is_path_valid(uav.path[uav.path_index :], self.grid_map):
            return True
        min_interval = float(self.config["search"].get("active_replan_min_interval_s", 60.0))
        if now - task.last_replan_time < min_interval:
            return False
        previous_count = max(1, len(previous_waypoints))
        if len(useful_waypoints) / previous_count <= float(self.config["search"].get("active_replan_low_gain_ratio", 0.35)):
            return True
        return self._is_near_scanline_end(uav.position, previous_waypoints)

    def _is_near_scanline_end(self, position: Position, waypoints: list[Position]) -> bool:
        row_points = [point for point in waypoints if point.y == position.y]
        if not row_points:
            return False
        nearest_end_distance = min(
            abs(min(point.x for point in row_points) - position.x),
            abs(max(point.x for point in row_points) - position.x),
        )
        return nearest_end_distance <= max(1, int(self.config["uav"]["sensor_radius_cells"]))

    def _trim_pending_search_waypoints_against_reserved(self, now: float, coverage_threshold: float) -> None:
        reserved = self._get_active_search_footprint()
        if not reserved:
            return
        for task in self.task_manager.get_pending_tasks():
            if task.type != TaskType.SEARCH:
                continue
            coverage_waypoints = self._task_coverage_waypoints(task)
            useful_waypoints = [
                waypoint
                for waypoint in coverage_waypoints
                if self._point_adds_unreserved_search_coverage(
                    waypoint,
                    int(self.config["uav"]["sensor_radius_cells"]),
                    coverage_threshold,
                    reserved,
                )
            ]
            useful_waypoints = self._filter_post_goal_waypoints(
                task,
                useful_waypoints,
                int(self.config["uav"]["sensor_radius_cells"]),
                coverage_threshold,
            )
            if len(useful_waypoints) == len(coverage_waypoints):
                continue
            if useful_waypoints:
                self._set_task_coverage_waypoints(task, useful_waypoints, now)
            else:
                self.task_manager.complete_task(task.id, now=now)

    def _get_active_search_footprint(self) -> set[Position]:
        reserved: set[Position] = set()
        for task in self.task_manager.get_active_tasks():
            if task.type != TaskType.SEARCH or task.assigned_uav_id is None:
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            if uav.status != UAVStatus.SEARCHING or not uav.path:
                continue
            reserved.update(self._path_coverage_footprint(uav.path[uav.path_index :], uav.sensor_radius_cells))
        return reserved

    def _point_adds_search_coverage(self, point: Position, radius_cells: int, coverage_threshold: float) -> bool:
        radius_sq = radius_cells * radius_cells
        for y in range(point.y - radius_cells, point.y + radius_cells + 1):
            for x in range(point.x - radius_cells, point.x + radius_cells + 1):
                pos = Position(x, y)
                if not self.grid_map.is_passable(pos):
                    continue
                if (x - point.x) ** 2 + (y - point.y) ** 2 > radius_sq:
                    continue
                if self.grid_map.get_cell(pos).search_confidence < coverage_threshold:
                    return True
        return False

    def _point_adds_unreserved_search_coverage(
        self,
        point: Position,
        radius_cells: int,
        coverage_threshold: float,
        reserved: set[Position],
    ) -> bool:
        radius_sq = radius_cells * radius_cells
        for y in range(point.y - radius_cells, point.y + radius_cells + 1):
            for x in range(point.x - radius_cells, point.x + radius_cells + 1):
                pos = Position(x, y)
                if pos in reserved or not self.grid_map.is_passable(pos):
                    continue
                if (x - point.x) ** 2 + (y - point.y) ** 2 > radius_sq:
                    continue
                if self.grid_map.get_cell(pos).search_confidence < coverage_threshold:
                    return True
        return False

    def _filter_post_goal_waypoints(
        self,
        task: Task,
        waypoints: list[Position],
        radius_cells: int,
        coverage_threshold: float,
    ) -> list[Position]:
        if not self._coverage_goal_reached() or task.type != TaskType.SEARCH:
            return waypoints
        valuable_cells = self._valuable_remaining_cells_after_goal(task, coverage_threshold)
        if not valuable_cells:
            return []
        return [
            waypoint
            for waypoint in waypoints
            if self._point_intersects_cells(waypoint, radius_cells, valuable_cells)
        ]

    def _valuable_remaining_cells_after_goal(self, task: Task, coverage_threshold: float) -> set[Position]:
        remaining = {
            cell
            for cell in task.target_cells
            if self.grid_map.is_passable(cell)
            and self.grid_map.get_cell(cell).search_confidence < coverage_threshold
        }
        if not remaining:
            return set()
        valuable: set[Position] = set()
        ordinary_min = self._post_goal_ordinary_min_cells()
        for component in connected_components(remaining, self.grid_map):
            has_priority = any(self.grid_map.get_cell(cell).search_priority > 1.0 for cell in component)
            if has_priority or len(component) >= ordinary_min:
                valuable.update(component)
        return valuable

    def _point_intersects_cells(self, point: Position, radius_cells: int, cells: set[Position]) -> bool:
        radius_sq = radius_cells * radius_cells
        for cell in cells:
            if abs(cell.x - point.x) > radius_cells or abs(cell.y - point.y) > radius_cells:
                continue
            if (cell.x - point.x) ** 2 + (cell.y - point.y) ** 2 <= radius_sq:
                return True
        return False

    def _dispatch_completed_search_returns(self, now: float) -> list[DecisionCommand]:
        if not bool(self.config["search"].get("allow_early_return", True)):
            if not self._search_tasks_finished() or self._get_unsearched_cells():
                return []
        if self.task_manager.get_pending_tasks() or not self._mission_goal_met():
            return []

        commands: list[DecisionCommand] = []
        for uav in self.fleet.get_all_states():
            if uav.status != UAVStatus.IDLE or uav.position == uav.home_position:
                continue
            plan = self.planner.plan_path(uav, uav.home_position, self.grid_map, task_id=uav.current_task_id)
            if not plan.valid:
                commands.append(
                    DecisionCommand(
                        uav_id=uav.id,
                        command=CommandType.HOLD,
                        task_id=uav.current_task_id,
                        target=uav.home_position,
                        path=[],
                        reason="mission_complete_return_path_not_found",
                    )
                )
                continue
            commands.append(
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.RETURN_HOME,
                    task_id=uav.current_task_id,
                    target=uav.home_position,
                    path=plan.path,
                    reason="mission_complete",
                )
            )
        return commands

    def _search_tasks_finished(self) -> bool:
        if not self.task_manager.tasks:
            return False
        active_statuses = {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        return all(
            task.type != TaskType.SEARCH or task.status not in active_statuses
            for task in self.task_manager.tasks.values()
        )

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

        物理离线状态由 Simulator 在构建 Observation 前应用；算法层不再向离线 UAV 下发 HOLD。
        """
        self._refresh_reachability()
        return []

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
        if not event.data.get("_applied_by_simulator"):
            self.map_updater.apply_updates(updates)
        self._blocked_region_cache.clear()
        self._refresh_reachability()
        changed_tasks = self._clean_impassable_search_tasks(event.timestamp)
        changed_tasks.update(self._split_active_search_tasks_after_map_update(event.timestamp))

        commands: list[DecisionCommand] = []
        for state in self.fleet.get_all_states():
            if state.status == UAVStatus.OFFLINE or not state.path:
                continue
            task = self.task_manager.tasks.get(state.current_task_id or "")
            path_valid = self.planner.is_path_valid(state.path[state.path_index :], self.grid_map)
            if path_valid and state.current_task_id not in changed_tasks:
                continue

            if task is not None and task.type == TaskType.SEARCH:
                repair = self._try_repair_dynamic_segment_route(task, state, event.timestamp)
                if repair is not None:
                    commands.append(repair)
                    continue
                waypoints = self._task_coverage_waypoints(task)
                if not waypoints:
                    self.task_manager.complete_task(task.id, now=event.timestamp)
                    continue
                route = self._plan_route_through_waypoints(state, waypoints)
                goal = waypoints[-1]
            else:
                goal = state.path[-1]
                plan = self.planner.plan_path(state, goal, self.grid_map, task_id=state.current_task_id)
                route = plan.path if plan.valid else []

            if not route:
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
            self.replan_count += 1
            if task is not None:
                task.replan_count += 1
                task.last_replan_time = event.timestamp
            commands.append(
                DecisionCommand(
                    uav_id=state.id,
                    command=CommandType.REPLAN,
                    task_id=state.current_task_id,
                    target=goal,
                    path=route,
                    reason="map_update",
                    metadata={**(self._command_metadata_for_task(task, waypoints) if task is not None else {}), "status": state.status.value},
                )
            )
        return commands

    def _clean_impassable_search_tasks(self, now: float) -> set[str]:
        changed: set[str] = set()
        for task in self.task_manager.tasks.values():
            if task.type != TaskType.SEARCH or task.status in (TaskStatus.COMPLETED, TaskStatus.CANCELLED):
                continue
            target_cells = {cell for cell in task.target_cells if self.grid_map.is_passable(cell)}
            coverage_waypoints = [point for point in self._task_coverage_waypoints(task) if self.grid_map.is_passable(point)]
            if target_cells != task.target_cells or len(coverage_waypoints) != len(task.coverage_waypoints or task.waypoints):
                changed.add(task.id)
                if self._is_segment_route_repair_task(task):
                    previous_count = len(task.coverage_waypoints or task.waypoints)
                    self._diagnostics["dynamic_route_repair_dropped_waypoints"] += max(0, previous_count - len(coverage_waypoints))
            task.target_cells = target_cells
            self._set_task_coverage_waypoints(task, coverage_waypoints, now)
            self._set_allowed_uavs_for_task(task)
            if not task.target_cells or not task.coverage_waypoints:
                self.task_manager.complete_task(task.id, now=now)
        return changed

    def _split_active_search_tasks_after_map_update(self, now: float) -> set[str]:
        changed: set[str] = set()
        new_tasks: list[Task] = []
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        for task in list(self.task_manager.get_active_tasks()):
            if task.type != TaskType.SEARCH or task.assigned_uav_id is None:
                continue
            if self._is_segment_route_repair_task(task):
                continue
            uav = self.fleet.get_uav(task.assigned_uav_id).state
            remaining_cells = {
                cell
                for cell in task.target_cells
                if self.grid_map.is_passable(cell)
                and self.grid_map.get_cell(cell).search_confidence < coverage_threshold
            }
            components = connected_components(remaining_cells, self.grid_map)
            if len(components) <= 1:
                continue

            keep = min(
                components,
                key=lambda component: min(abs(cell.x - uav.position.x) + abs(cell.y - uav.position.y) for cell in component),
            )
            task.target_cells = set(keep)
            kept_waypoints = [point for point in self._task_coverage_waypoints(task) if point in keep]
            if not kept_waypoints:
                kept_waypoints = generate_boustrophedon_path(keep, uav.sensor_radius_cells)
                kept_waypoints = reorder_waypoints_for_uav(kept_waypoints, uav.position)
            self._set_task_coverage_waypoints(task, kept_waypoints, now)
            self._set_allowed_uavs_for_task(task)
            changed.add(task.id)

            for component in components:
                if component == keep:
                    continue
                supplemental = self._build_search_task_from_region(component, uav.position, now, "map_update")
                if supplemental is not None:
                    new_tasks.append(supplemental)

        if new_tasks:
            self.task_manager.add_tasks(new_tasks)
        return changed

    def _is_segment_route_repair_task(self, task: Task) -> bool:
        return (
            bool(self.config.get("algorithm", {}).get("segment_sweep", {}).get("dynamic_route_repair_enabled", True))
            and task.metadata.get("planner_version") == "segment_sweep_v1"
            and task.type == TaskType.SEARCH
        )

    def _try_repair_dynamic_segment_route(self, task: Task, uav: UAVState, now: float) -> DecisionCommand | None:
        if not self._is_segment_route_repair_task(task):
            return None
        self._diagnostics["dynamic_route_repair_attempts"] += 1
        segment_config = self.config.get("algorithm", {}).get("segment_sweep", {})
        min_remaining = int(segment_config.get("dynamic_route_repair_min_remaining_waypoints", 4))
        max_connector_m = float(segment_config.get("dynamic_route_repair_max_connector_m", 200.0))
        previous = self._task_coverage_waypoints(task)
        if not previous:
            self._diagnostics["dynamic_route_repair_fallback_to_supplemental"] += 1
            return None
        valid = [
            point
            for point in previous
            if self.grid_map.is_passable(point)
            and uav.id in self._reachability.reachable_uavs(point)
        ]
        dropped = len(previous) - len(valid)
        self._diagnostics["dynamic_route_repair_dropped_waypoints"] += max(0, dropped)
        if len(valid) < min_remaining:
            self._diagnostics["dynamic_route_repair_fallback_to_supplemental"] += 1
            return None
        connector_plan = self.planner.plan_path(uav, valid[0], self.grid_map, task_id=task.id)
        if not connector_plan.valid:
            self._diagnostics["dynamic_route_repair_fallback_to_supplemental"] += 1
            return None
        connector_m = max(0, len(connector_plan.path) - 1) * self.grid_map.resolution_m
        if connector_m > max_connector_m:
            self._diagnostics["dynamic_route_repair_fallback_to_supplemental"] += 1
            return None
        route = self._plan_route_through_waypoints(uav, valid)
        if not route:
            self._diagnostics["dynamic_route_repair_fallback_to_supplemental"] += 1
            return None
        self._set_task_coverage_waypoints(task, valid, now)
        task.target_cells = {cell for cell in task.target_cells if self.grid_map.is_passable(cell)}
        task.replan_count += 1
        task.last_replan_time = now
        self.replan_count += 1
        self._diagnostics["dynamic_route_repair_success"] += 1
        self._diagnostics["dynamic_route_repair_replanned_tasks"] += 1
        return DecisionCommand(
            uav_id=uav.id,
            command=CommandType.REPLAN,
            task_id=task.id,
            target=valid[-1],
            path=route,
            reason="dynamic_obstacle_segment_route_repair",
            metadata={**self._command_metadata_for_task(task, valid), "status": uav.status.value},
        )

    def _build_search_task_from_region(
        self,
        region: set[Position],
        origin: Position,
        now: float,
        prefix: str,
    ) -> Task | None:
        allowed_uav_ids = {
            uav_id
            for cell in region
            for uav_id in self._reachability.reachable_uavs(cell)
        }
        if not allowed_uav_ids:
            return None
        self._supplemental_task_seq += 1
        return self.coverage_planner.plan_region_task(
            task_id=f"{prefix}_{self._supplemental_task_seq:03d}",
            region=region,
            origin=origin,
            grid_map=self.grid_map,
            sensor_radius_cells=int(self.config["uav"]["sensor_radius_cells"]),
            created_at=now,
            reachability=self._reachability,
            allowed_uav_ids=allowed_uav_ids,
        )

    def _handle_building_model_request(self, event: Event) -> list[DecisionCommand]:
        modeling_config = self.config.get("modeling", {})
        if not bool(modeling_config.get("enabled", True)):
            self._diagnostics["modeling_jobs_failed"] += 1
            return []

        footprint = self._building_footprint_from_event(event)
        if footprint is None:
            self._diagnostics["modeling_jobs_failed"] += 1
            return []

        max_uavs = max(1, int(modeling_config.get("max_uav_count", 4)))
        requested = int(event.data.get("uav_count", modeling_config.get("default_uav_count", 2)))
        uav_count = max(1, min(requested, max_uavs))
        standoff = int(event.data.get("standoff_cells", modeling_config.get("default_standoff_cells", 3)))
        standoff = max(int(modeling_config.get("min_standoff_cells", 1)), standoff)
        standoff = min(int(modeling_config.get("max_standoff_cells", standoff)), standoff)
        laps = max(1, int(event.data.get("laps", modeling_config.get("default_laps", 1))))
        resume_search_after = bool(event.data.get("resume_search_after", modeling_config.get("resume_search_after", True)))

        selected_uavs = self._select_modeling_uavs(uav_count, modeling_config)
        if not selected_uavs:
            self._diagnostics["modeling_jobs_failed"] += 1
            return []

        planner = FacadeModelingPlanner(modeling_config)
        plans = planner.plan_modeling(
            footprint=footprint,
            grid_map=self.grid_map,
            uav_states=selected_uavs,
            uav_count=len(selected_uavs),
            standoff_cells=standoff,
            laps=laps,
            created_at=event.timestamp,
            resume_search_after=resume_search_after,
        )
        self._merge_modeling_diagnostics(planner.last_diagnostics)
        if not plans:
            self._diagnostics["modeling_jobs_failed"] += 1
            return []

        self._modeling_job_seq += 1
        job_id = f"modeling_{footprint.building_id}_{self._modeling_job_seq:03d}"
        interrupted_by_uav: dict[str, str | None] = {}
        tasks: list[Task] = []
        commands: list[DecisionCommand] = []
        for plan in plans:
            uav = self.fleet.get_uav(plan.uav_id).state
            interrupted_task_id = self._pause_search_for_modeling(uav, event.timestamp, modeling_config)
            interrupted_by_uav[uav.id] = interrupted_task_id
            if interrupted_task_id is not None:
                self._diagnostics["modeling_interrupted_search_tasks"] += 1
            task_id = f"{job_id}_{uav.id}"
            metadata = {
                **plan.metadata,
                "job_id": job_id,
                "event_id": event.id,
                "footprint": event.data.get("footprint", []),
                "interrupted_task_id": interrupted_task_id,
                "resume_search_after": resume_search_after,
                "modeling_task": True,
            }
            task = Task(
                id=task_id,
                type=TaskType.MODELING,
                priority=10.0,
                target_cells=set(plan.logical_waypoints),
                entry_point=plan.logical_waypoints[0],
                status=TaskStatus.ASSIGNED,
                assigned_uav_id=uav.id,
                waypoints=list(plan.route),
                coverage_waypoints=list(plan.logical_waypoints),
                estimated_cost_m=plan.estimated_distance_m,
                created_at=event.timestamp,
                updated_at=event.timestamp,
                source_event_id=event.id,
                allowed_uav_ids={uav.id},
                metadata=metadata,
            )
            tasks.append(task)
            commands.append(
                DecisionCommand(
                    uav_id=uav.id,
                    command=CommandType.MODEL_STRUCTURE,
                    task_id=task_id,
                    target=plan.route[-1],
                    path=list(plan.route),
                    reason="building_model_request",
                    command_id=f"cmd_{task_id}",
                    metadata=metadata,
                )
            )

        self.task_manager.add_tasks(tasks)
        self._modeling_jobs[job_id] = {
            "job_id": job_id,
            "building_id": footprint.building_id,
            "task_ids": [task.id for task in tasks],
            "uav_ids": [str(task.assigned_uav_id) for task in tasks],
            "resume_search_after": resume_search_after,
            "interrupted_task_by_uav": interrupted_by_uav,
            "status": "ACTIVE",
            "created_at": event.timestamp,
            "completed_at": None,
        }
        self._diagnostics["modeling_jobs_total"] += 1
        self._diagnostics["modeling_active_jobs"] += 1
        self._diagnostics["modeling_assigned_uav_count"] += len(tasks)
        self._diagnostics["modeling_facade_progress_ratio"] = 0
        return commands

    def _building_footprint_from_event(self, event: Event) -> BuildingFootprint | None:
        vertices_data = event.data.get("footprint")
        building_id = str(event.data.get("building_id", event.id))
        if not isinstance(vertices_data, list) or len(vertices_data) < 4:
            return None
        vertices: list[Position] = []
        for item in vertices_data:
            if isinstance(item, Position):
                vertices.append(item)
            elif isinstance(item, dict) and "x" in item and "y" in item:
                vertices.append(Position(int(item["x"]), int(item["y"])))
        if len(vertices) < 4:
            return None
        return BuildingFootprint(building_id=building_id, vertices=vertices)

    def _select_modeling_uavs(self, uav_count: int, modeling_config: dict[str, Any]) -> list[UAVState]:
        allow_interrupt = bool(modeling_config.get("allow_interrupt_search", True))
        min_battery = float(modeling_config.get("min_battery_margin", 0.2))
        candidates: list[UAVState] = []
        for state in self.fleet.get_all_states():
            if state.status in (UAVStatus.OFFLINE, UAVStatus.RETURNING, UAVStatus.CONFIRMING):
                continue
            if state.status == UAVStatus.IDLE or (allow_interrupt and state.status == UAVStatus.SEARCHING):
                if state.battery >= min_battery:
                    candidates.append(state)
        return sorted(candidates, key=lambda state: (state.status != UAVStatus.IDLE, state.assigned_task_count, state.id))[:uav_count]

    def _pause_search_for_modeling(
        self,
        uav: UAVState,
        now: float,
        modeling_config: dict[str, Any],
    ) -> str | None:
        if not bool(modeling_config.get("allow_interrupt_search", True)):
            return None
        return self._pause_search_for_confirmation(uav, now)

    def _merge_modeling_diagnostics(self, diagnostics: dict[str, Any]) -> None:
        for key in (
            "modeling_facade_lane_count",
            "modeling_unreachable_facade_lanes",
            "modeling_no_fly_violations",
        ):
            if key in diagnostics:
                self._diagnostics[key] += int(diagnostics[key])
        self._diagnostics["modeling_distance_m"] += int(diagnostics.get("modeling_distance_m", 0))

    def _handle_target_found(self, event: Event) -> list[DecisionCommand]:
        target_data = event.data
        target_pos_data = target_data.get("position")
        if target_pos_data is None:
            return []

        target = Position(int(target_pos_data["x"]), int(target_pos_data["y"]))
        confirm_task_id = f"confirm_{target_data.get('target_id', event.id)}"
        target_id = str(target_data.get("target_id", event.id))
        if target_id in self._confirmed_targets or confirm_task_id in self._confirmations:
            return []

        self._target_metrics[target_id] = {
            "target_id": target_id,
            "found_time_s": event.timestamp,
            "assigned_time_s": None,
            "done_time_s": None,
            "failed_time_s": None,
            "success": False,
            "interrupted_task_id": None,
            "resumed_time_s": None,
            "coverage_at_found": self.grid_map.coverage_rate(),
            "coverage_at_done": None,
        }

        plan = self._select_confirm_uav(target, event)
        if plan is None:
            return self._mark_confirm_failed(target_id, confirm_task_id, target, event.timestamp, "confirm_uav_not_available")

        uav = plan.uav
        interrupted_task_id = self._pause_search_for_confirmation(uav, event.timestamp)
        plan.interrupted_task_id = interrupted_task_id
        self._confirmations[confirm_task_id] = {
            "uav_id": uav.id,
            "target": target,
            "orbit_waypoints": plan.orbit_waypoints,
            "target_id": target_id,
            "target_type": target_data.get("target_type"),
            "confidence": float(target_data.get("confidence", 0.0)),
            "status": "CONFIRMING",
            "assigned_at": event.timestamp,
            "dwell_s": self._confirm_dwell_s(event),
            "dwell_started_at": None,
            "interrupted_task_id": interrupted_task_id,
            "resume_owner_id": uav.id if interrupted_task_id else None,
        }
        self._target_metrics[target_id].update(
            {
                "assigned_time_s": event.timestamp,
                "uav_id": uav.id,
                "interrupted_task_id": interrupted_task_id,
            }
        )
        return [
            DecisionCommand(
                uav_id=uav.id,
                command=CommandType.CONFIRM_TARGET,
                task_id=confirm_task_id,
                target=target,
                path=plan.route,
                reason="target_found",
            )
        ]

    def _select_confirm_uav(self, target: Position, event: Event) -> ConfirmPlan | None:
        candidates: list[ConfirmPlan] = []
        source_bonus_m = float(self.config["search"].get("confirm_source_preference_bonus_m", 120.0))
        for state in self.fleet.get_all_states():
            if state.status == UAVStatus.OFFLINE or state.status in (UAVStatus.CONFIRMING, UAVStatus.RETURNING):
                continue
            if state.status != UAVStatus.IDLE and state.status != UAVStatus.SEARCHING:
                continue
            plan = self._build_confirm_plan(state, target, event)
            if plan is None:
                continue
            interrupt_cost_m = self._confirm_interrupt_cost(state)
            route_distance_m = self._route_distance_m(plan.route)
            return_distance_m = self._return_distance_m_from(plan.route[-1], state)
            if return_distance_m == float("inf"):
                continue
            if not self._battery_supports_confirmation(state, route_distance_m + return_distance_m):
                continue
            reserve = float(self.config["uav"].get("battery_threshold", 0.2))
            available_m = max(0.0, state.battery - reserve) * state.velocity_mps * float(self.config["uav"]["endurance_s"])
            margin_m = available_m - route_distance_m - return_distance_m
            battery_risk_m = max(0.0, self.grid_map.resolution_m * 10.0 - margin_m)
            source_bonus = source_bonus_m if event.source_uav_id == state.id else 0.0
            plan.cost_m = route_distance_m + interrupt_cost_m + battery_risk_m - source_bonus
            plan.interrupt_cost_m = interrupt_cost_m
            candidates.append(plan)
        if not candidates:
            return None
        return min(candidates, key=lambda item: (item.cost_m, item.uav.id))

    def _build_confirm_plan(self, uav: UAVState, target: Position, event: Event) -> ConfirmPlan | None:
        radius = max(1, int(event.data.get("orbit_radius_cells", self.config["search"].get("confirm_orbit_radius_cells", 2))))
        laps = max(1, int(event.data.get("orbit_laps", self.config["search"].get("confirm_orbit_laps", 1))))
        max_extra_radius = max(0, int(self.config["search"].get("confirm_orbit_max_extra_radius_cells", 5)))

        for current_radius in range(radius, radius + max_extra_radius + 1):
            orbit = self._build_confirm_orbit(target, current_radius, laps, require_complete=True)
            if not orbit:
                continue
            route = self._plan_route_through_waypoints(uav, orbit)
            if route:
                return ConfirmPlan(uav=uav, target=target, orbit_waypoints=orbit, route=route, cost_m=0.0)

        hover = self._nearest_observable_confirm_point(uav, target, radius, radius + max_extra_radius)
        if hover is None:
            return None
        route = self._plan_route_through_waypoints(uav, [hover])
        if not route:
            return None
        return ConfirmPlan(uav=uav, target=target, orbit_waypoints=[hover], route=route, cost_m=0.0)

    def _build_confirm_orbit(
        self,
        target: Position,
        radius: int | None = None,
        laps: int | None = None,
        require_complete: bool = False,
    ) -> list[Position]:
        radius = max(1, int(radius if radius is not None else self.config["search"].get("confirm_orbit_radius_cells", 2)))
        laps = max(1, int(laps if laps is not None else self.config["search"].get("confirm_orbit_laps", 1)))
        return self._square_orbit_candidates(target, radius, require_complete=require_complete) * laps

    def _square_orbit_candidates(self, target: Position, radius: int, require_complete: bool = False) -> list[Position]:
        candidates: list[Position] = []
        offsets: list[tuple[int, int]] = []
        offsets.extend((dx, -radius) for dx in range(-radius, radius + 1))
        offsets.extend((radius, dy) for dy in range(-radius + 1, radius + 1))
        offsets.extend((dx, radius) for dx in range(radius - 1, -radius - 1, -1))
        offsets.extend((-radius, dy) for dy in range(radius - 1, -radius, -1))
        expected_count = len(dict.fromkeys(offsets))
        seen: set[Position] = set()
        for dx, dy in offsets:
            pos = Position(target.x + dx, target.y + dy)
            if pos in seen:
                continue
            seen.add(pos)
            if not self.grid_map.is_passable(pos):
                if require_complete:
                    return []
                continue
            candidates.append(pos)
        if require_complete and len(candidates) != expected_count:
            return []
        return candidates

    def _nearest_observable_confirm_point(
        self,
        uav: UAVState,
        target: Position,
        min_radius: int,
        max_radius: int,
    ) -> Position | None:
        candidates: list[Position] = []
        for radius in range(max(1, min_radius), max(min_radius, max_radius) + 1):
            candidates.extend(self._square_orbit_candidates(target, radius, require_complete=False))
        candidates = sorted(set(candidates), key=lambda pos: (abs(pos.x - uav.position.x) + abs(pos.y - uav.position.y), pos.y, pos.x))
        for candidate in candidates:
            plan = self.planner.plan_path(uav, candidate, self.grid_map)
            if plan.valid:
                return candidate
        return None

    def _pause_search_for_confirmation(self, uav: UAVState, now: float) -> str | None:
        task_id = uav.current_task_id
        if uav.status != UAVStatus.SEARCHING or task_id not in self.task_manager.tasks:
            return None
        task = self.task_manager.tasks[task_id]
        if task.type != TaskType.SEARCH:
            return None
        coverage_threshold = float(self.config["search"].get("coverage_complete_threshold", 0.95))
        remaining = [
            waypoint
            for waypoint in self._task_coverage_waypoints(task)
            if self._point_adds_search_coverage(waypoint, uav.sensor_radius_cells, coverage_threshold)
        ]
        task.resume_owner_id = uav.id
        task.assigned_uav_id = None
        task.status = TaskStatus.PENDING
        self._set_task_coverage_waypoints(task, remaining, now)
        if not remaining:
            self.task_manager.complete_task(task.id, now=now)
            return None
        return task.id

    def _confirm_interrupt_cost(self, state: UAVState) -> float:
        if state.status != UAVStatus.SEARCHING or state.current_task_id not in self.task_manager.tasks:
            return 0.0
        task = self.task_manager.tasks[state.current_task_id]
        remaining = len(self._task_coverage_waypoints(task))
        return remaining * self.grid_map.resolution_m * float(self.config["search"].get("confirm_interrupt_waypoint_cost_weight", 0.5))

    def _route_distance_m(self, route: list[Position]) -> float:
        if len(route) < 2:
            return 0.0
        return sum(
            ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5 * self.grid_map.resolution_m
            for a, b in zip(route, route[1:])
        )

    def _return_distance_m_from(self, position: Position, state: UAVState) -> float:
        return_state = replace(state, position=position)
        plan = self.planner.plan_path(return_state, state.home_position, self.grid_map, task_id=state.current_task_id)
        if not plan.valid:
            return float("inf")
        return self._route_distance_m(plan.path)

    def _battery_supports_confirmation(self, state: UAVState, required_distance_m: float) -> bool:
        reserve = float(self.config["uav"].get("battery_threshold", 0.2))
        available_distance_m = max(0.0, state.battery - reserve) * state.velocity_mps * float(self.config["uav"]["endurance_s"])
        return available_distance_m >= required_distance_m

    def _confirm_dwell_s(self, event: Event) -> float:
        if "dwell_s" in event.data:
            return max(0.0, float(event.data["dwell_s"]))
        step_s = float(self.config.get("simulation", {}).get("time_step_s", 1.0))
        return max(0.0, int(self.config["search"].get("confirm_duration_steps", 1)) * step_s)

    def _mark_confirm_failed(
        self,
        target_id: str,
        confirm_task_id: str,
        target: Position,
        now: float,
        reason: str,
    ) -> list[DecisionCommand]:
        record = self._target_metrics.setdefault(target_id, {"target_id": target_id, "found_time_s": now})
        record.update({"failed_time_s": now, "success": False, "failure_reason": reason})
        fallback = next((state for state in self.fleet.get_all_states() if state.status != UAVStatus.OFFLINE), None)
        if fallback is None:
            return []
        return [
            DecisionCommand(
                uav_id=fallback.id,
                command=CommandType.HOLD,
                task_id=confirm_task_id,
                target=target,
                path=[],
                reason="CONFIRM_FAILED",
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
        confirmation = self._confirmations.pop(task_id, None) if task_id else None
        if event.source_uav_id is None:
            return []
        uav = self.fleet.get_uav(event.source_uav_id).state
        target_id = str(event.data.get("target_id", confirmation.get("target_id") if confirmation else task_id))
        self._confirmed_targets.add(target_id)
        if target_id in self._target_metrics:
            self._target_metrics[target_id].update(
                {
                    "done_time_s": event.timestamp,
                    "success": True,
                    "coverage_at_done": self.grid_map.coverage_rate(),
                }
            )

        resume_command = self._resume_interrupted_search_after_confirm(confirmation, uav, event.timestamp)
        if resume_command is not None:
            return [resume_command]

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

    def _resume_interrupted_search_after_confirm(
        self,
        confirmation: dict[str, Any] | None,
        uav: UAVState,
        now: float,
    ) -> DecisionCommand | None:
        if not confirmation:
            return None
        interrupted_task_id = confirmation.get("interrupted_task_id")
        if interrupted_task_id not in self.task_manager.tasks:
            return None
        task = self.task_manager.tasks[interrupted_task_id]
        if task.status != TaskStatus.PENDING or not self._task_coverage_waypoints(task):
            return None
        if task.resume_owner_id not in (None, uav.id):
            return None

        waypoints = reorder_waypoints_for_uav(self._task_coverage_waypoints(task), uav.position)
        if not waypoints:
            return None
        route = self._plan_route_through_waypoints(uav, waypoints)
        if not route:
            task.resume_owner_id = None
            return None

        self._set_task_coverage_waypoints(task, waypoints, now)
        assignment = self.task_manager.assign_task(task.id, uav.id, now=now, bid_value=0.0)
        task.resume_owner_id = None
        target_id = str(confirmation.get("target_id", ""))
        if target_id in self._target_metrics:
            self._target_metrics[target_id]["resumed_time_s"] = now
            self._target_metrics[target_id]["resume_assignment"] = assignment.task_id
        return DecisionCommand(
            uav_id=uav.id,
            command=CommandType.FOLLOW_PATH,
            task_id=task.id,
            target=waypoints[-1],
            path=route,
            reason="resume_interrupted_search",
            metadata=self._command_metadata_for_task(task, waypoints),
        )
