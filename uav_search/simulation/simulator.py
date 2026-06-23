"""
仿真引擎模块

实现了时间步进的仿真引擎，负责：
1. 推进仿真时间
2. 更新无人机位置
3. 更新传感器覆盖
4. 触发决策循环
5. 记录仿真快照

仿真采用固定时间步长方式，每个时间步执行完整的"决策-执行"循环。
"""
from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from uav_search.core.contracts import CommandAck, ControlCommand
from uav_search.core.data_types import DecisionCommand, Event, EventType, Position, UAVStatus
from uav_search.core.scheduler import Scheduler
from uav_search.core.scheduler_adapter import SchedulerAlgorithmAdapter
from uav_search.maps.grid_map import GridMap
from uav_search.maps.map_updater import MapUpdater
from uav_search.simulation.command_applier import CommandApplier
from uav_search.simulation.observation_builder import ObservationBuilder
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.uav.fleet_manager import FleetManager


class Simulator:
    """仿真引擎类

    负责推进整个系统的仿真循环，协调决策和执行。

    属性：
        grid_map: 栅格地图对象
        fleet: 无人机编队管理器
        config: 系统配置字典
        time_s: 当前仿真时间（秒）
        snapshots: 仿真快照列表，记录每个时间步的状态
        _last_events: 上一时间步处理的事件ID列表

    仿真模式：
        - 固定时间步长（time_step_s）
        - 每步执行：状态更新 → 覆盖标记 → 决策 → 快照记录
        - 支持事件注入和动态决策
    """

    def __init__(self, grid_map: GridMap, fleet: FleetManager, config: dict[str, Any]) -> None:
        """初始化仿真引擎

        参数：
            grid_map: 栅格地图对象
            fleet: 无人机编队管理器
            config: 系统配置字典
        """
        self.grid_map = grid_map
        self.fleet = fleet
        self.config = config
        self.time_s = 0.0  # 仿真时间从0开始
        self.snapshots: list[dict[str, Any]] = []  # 存储所有时间步的快照
        self._last_events: list[str] = []  # 追踪最近处理的事件
        self._last_commands: list[DecisionCommand | ControlCommand] = []
        self._last_command_acks: list[CommandAck] = []
        self._tick = 0
        self.observation_builder = ObservationBuilder(grid_map, fleet, config)
        self.command_applier = CommandApplier(fleet, grid_map)
        self._scheduler_adapter: SchedulerAlgorithmAdapter | None = None
        self._pending_events: list[Event] = []
        self._current_events: list[Event] = []
        self._last_changed_cells: list[Position] = []
        self.map_updater = MapUpdater(grid_map)

    def enqueue_event(self, event: Event) -> None:
        self._pending_events.append(event)

    def run_initial_decision(self, scheduler: Scheduler) -> None:
        self._last_events = []
        self._last_commands = []
        self._last_command_acks = []
        self._prepare_external_events()
        algorithm = self._adapter_for(scheduler)
        decision = algorithm.decide(self._build_observation())
        self._last_events = list(decision.debug.get("events_handled", [])) or [event.id for event in self._current_events]
        self._last_commands = list(decision.commands)
        self._last_command_acks = self.command_applier.apply(decision.commands, now=self.time_s)
        self.record_snapshot(scheduler=scheduler, commands=decision.commands)
        self._current_events = []
        self._last_changed_cells = []
        self._tick += 1

    def step(self, scheduler: Scheduler | None = None, algorithm: SchedulerAlgorithmAdapter | None = None) -> None:
        """执行一个仿真时间步

        这是仿真的核心步骤，执行：
        1. 时间推进
        2. 无人机运动更新
        3. 传感器覆盖标记
        4. 决策后处理（如目标确认完成）

        参数：
            scheduler: 调度器对象，None表示仅仿真不决策

        执行流程：
            time_s += time_step_s
            → fleet.step() 更新所有无人机位置
            → grid_map.mark_covered() 标记传感器覆盖
            → scheduler.update_after_step() 处理决策后逻辑
            → record_snapshot() 记录当前状态
        """
        time_step_s = float(self.config["simulation"]["time_step_s"])
        self.time_s += time_step_s

        # 更新所有无人机的位置（沿路径移动）
        self.fleet.step(time_step_s, self.grid_map.resolution_m)

        # 标记传感器覆盖区域
        # 每架无人机飞过的地方，其传感器覆盖范围内的栅格被标记为已搜索
        revisit_interval_s = float(self.config["search"].get("redundant_revisit_interval_s", 0.0))
        for state in self.fleet.get_all_states():
            if state.status != UAVStatus.OFFLINE:
                self.grid_map.mark_covered(
                    state.position,
                    state.sensor_radius_cells,
                    self.time_s,
                    redundant_revisit_interval_s=revisit_interval_s,
                )

        # 决策后处理
        # 检查是否有需要更新状态的任务（如目标确认完成）
        if scheduler is not None:
            algorithm = algorithm or self._adapter_for(scheduler)
            post_step_output = algorithm.update_after_step(self.time_s)
            self._last_commands.extend(post_step_output.commands)
            self._last_events.extend(post_step_output.debug.get("events_handled", []))
            self._last_command_acks.extend(self.command_applier.apply(post_step_output.commands, now=self.time_s))
            if scheduler.should_run_regular_cycle():
                decision = algorithm.decide(self._build_observation())
                self._last_commands.extend(decision.commands)
                self._last_events.extend(decision.debug.get("events_handled", []))
                self._last_command_acks.extend(self.command_applier.apply(decision.commands, now=self.time_s))

        self._last_command_acks.extend(self.command_applier.refresh(self.time_s))

        # 记录当前时间步的快照
        self.record_snapshot(scheduler=scheduler)
        self._tick += 1

    def run(
        self,
        max_steps: int | None = None,
        scheduler: Scheduler | None = None,
        event_injector: ScenarioEventInjector | None = None,
    ) -> None:
        """运行仿真主循环

        这是仿真的主循环，执行多个时间步直到达到最大步数或终止条件。

        参数：
            max_steps: 最大时间步数，None则使用配置中的值
            scheduler: 调度器对象，用于每个时间步的决策
            event_injector: 场景事件注入器，用于注入预设事件

        主循环流程：
            for each step:
                1. 注入场景事件（如目标发现、地图更新等）
                2. 执行决策循环（处理事件、任务分配、路径规划）
                3. 推进仿真一步
                4. 检查终止条件（所有无人机空闲或离线）

        终止条件：
            - 所有无人机状态为 IDLE 或 OFFLINE
            - 达到最大时间步数

        决策触发：
            - 每步都检查是否有待处理事件
            - 有事件则触发完整决策循环
            - 决策包括：事件处理、任务分配、路径规划、冲突消解
        """
        steps = int(max_steps if max_steps is not None else self.config["simulation"]["max_steps"])
        mission_grace_steps = int(self.config["simulation"].get("mission_grace_steps", 0))
        return_grace_steps = int(self.config["simulation"].get("return_home_grace_steps", 0))
        steps_run = 0
        mission_grace_used = 0
        return_grace_used = 0

        while (
            steps_run < steps
            or self._should_extend_for_activity(mission_grace_used, mission_grace_steps)
            or self._should_extend_for_return(return_grace_used, return_grace_steps)
        ):
            steps_run += 1
            if steps_run > steps:
                if self._should_extend_for_activity(mission_grace_used, mission_grace_steps):
                    mission_grace_used += 1
                elif self._should_extend_for_return(return_grace_used, return_grace_steps):
                    return_grace_used += 1
            self._last_events = []
            self._last_commands = []
            self._last_command_acks = []

            # 决策循环（如果提供了调度器）
            if scheduler is not None:
                algorithm = self._adapter_for(scheduler)
                # 步骤1: 注入场景事件
                # 按时间将预设事件注入到事件管理器
                if event_injector is not None:
                    for event in event_injector.emit_due(self.time_s):
                        self.enqueue_event(event)
                self._prepare_external_events()

                # 步骤2: 执行决策循环
                # 如果有待处理事件，执行完整的决策流程
                if self._current_events or scheduler.should_run_regular_cycle():
                    decision = algorithm.decide(self._build_observation())
                    self._last_events = list(decision.debug.get("events_handled", [])) or [
                        event.id for event in self._current_events
                    ]
                    self._last_commands = list(decision.commands)
                    self._last_command_acks = self.command_applier.apply(decision.commands, now=self.time_s)
                self._current_events = []

            # 步骤3: 推进仿真一个时间步
            self.step(scheduler=scheduler, algorithm=self._scheduler_adapter)

            # 步骤4: 检查终止条件
            # 如果所有无人机都空闲或离线，提前结束
            if all(state.status in (UAVStatus.IDLE, UAVStatus.OFFLINE) for state in self.fleet.get_all_states()):
                break

    def _should_extend_for_activity(self, grace_used: int, grace_limit: int) -> bool:
        if grace_used >= grace_limit:
            return False
        return any(state.status not in (UAVStatus.IDLE, UAVStatus.OFFLINE) for state in self.fleet.get_all_states())

    def _should_extend_for_return(self, grace_used: int, grace_limit: int) -> bool:
        if grace_used >= grace_limit:
            return False
        return any(state.status == UAVStatus.RETURNING for state in self.fleet.get_all_states())

    def _prepare_external_events(self) -> None:
        self._current_events = list(self._pending_events)
        self._pending_events = []
        self._last_changed_cells = []
        for event in self._current_events:
            self._last_changed_cells.extend(self._apply_physical_event(event))

    def _apply_physical_event(self, event: Event) -> list[Position]:
        if event.type == EventType.MAP_UPDATE:
            updates = event.data.get("updates", [])
            if not updates and "operation" in event.data:
                updates = [event.data]
            changed = self.map_updater.apply_updates(updates)
            event.data["_applied_by_simulator"] = True
            event.data["changed_cells"] = [{"x": cell.x, "y": cell.y} for cell in changed]
            return changed
        if event.type == EventType.UAV_OFFLINE:
            uav_id = event.source_uav_id or event.data.get("uav_id")
            if uav_id is not None:
                state = self.fleet.get_uav(str(uav_id)).state
                state.status = UAVStatus.OFFLINE
                state.available = False
                state.path = []
                state.path_index = 0
            return []
        if event.type == EventType.UAV_RECOVERED:
            uav_id = event.source_uav_id or event.data.get("uav_id")
            if uav_id is not None:
                state = self.fleet.get_uav(str(uav_id)).state
                state.status = UAVStatus.IDLE
                state.available = True
            return []
        return []

    def apply_commands(
        self,
        commands: list[DecisionCommand | ControlCommand],
        now: float | None = None,
    ) -> list[CommandAck]:
        applied_at = self.time_s if now is None else now
        control_commands = [
            command if isinstance(command, ControlCommand) else ControlCommand.from_decision(command, issued_at=applied_at)
            for command in commands
        ]
        acks = self.command_applier.apply(control_commands, now=applied_at)
        self._last_command_acks.extend(acks)
        return acks

    def _adapter_for(self, scheduler: Scheduler) -> SchedulerAlgorithmAdapter:
        if self._scheduler_adapter is None or self._scheduler_adapter.scheduler is not scheduler:
            self._scheduler_adapter = SchedulerAlgorithmAdapter(scheduler)
        return self._scheduler_adapter

    def _build_observation(self):
        return self.observation_builder.build(
            tick=self._tick,
            time_s=self.time_s,
            changed_cells=self._last_changed_cells,
            events=self._current_events,
            command_acks=self.command_applier.recent_acks(self.time_s),
            active_command_ids=self.command_applier.active_command_ids,
        )

    def record_snapshot(
        self,
        scheduler: Scheduler | None = None,
        commands: list[DecisionCommand | ControlCommand] | None = None,
    ) -> None:
        """记录当前时间步的快照

        将当前时刻的系统状态保存为快照，包括：
        - 仿真时间
        - 全局覆盖率
        - 重点区域覆盖率
        - 所有无人机的状态
        - 本步处理的事件

        快照用于后续的可视化和分析。
        """
        self.snapshots.append(
            {
                "time_s": self.time_s,
                "global_coverage": self.grid_map.coverage_rate(),
                "priority_coverage": self.grid_map.coverage_rate(priority_only=True),
                "replan_count": scheduler.replan_count if scheduler is not None else 0,
                "target_metrics": scheduler.target_metrics_snapshot() if scheduler is not None else {},
                "tasks": scheduler.task_status_snapshot() if scheduler is not None else {},
                "commands": [
                    _command_to_snapshot(command)
                    for command in (commands if commands is not None else self._last_commands)
                ],
                "command_acks": [_ack_to_snapshot(ack) for ack in self._last_command_acks],
                "changed_cells": [asdict(cell) for cell in self._last_changed_cells],
                "uavs": [
                    {
                        "id": state.id,
                        "position": asdict(state.position),
                        "status": state.status.value,
                        "battery": state.battery,
                        "task_id": state.current_task_id,
                        "total_distance_m": state.total_distance_m,
                        "effective_search_distance_m": state.effective_search_distance_m,
                    }
                    for state in self.fleet.get_all_states()
                ],
                "events": self._last_events,
            }
        )

    def save_snapshots(self, path: str | Path, run_id: str = "manual_run") -> None:
        """保存所有快照到JSON文件

        将仿真过程中记录的所有快照保存为JSON格式，用于后续分析和可视化。

        参数：
            path: 输出文件路径
            run_id: 运行标识符

        输出格式：
            {
                "run_id": "场景名称",
                "steps": [
                    {
                        "time_s": 0.0,
                        "global_coverage": 0.0,
                        "priority_coverage": 0.0,
                        "uavs": [...],
                        "events": [...]
                    },
                    ...
                ]
            }
        """
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"run_id": run_id, "steps": self.snapshots}
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)


def _command_to_snapshot(command: DecisionCommand | ControlCommand) -> dict[str, Any]:
    return {
        "command_id": command.command_id,
        "command": command.command.value,
        "uav_id": command.uav_id,
        "task_id": command.task_id,
        "target": asdict(command.target) if command.target is not None else None,
        "path": [asdict(point) for point in command.path],
        "reason": command.reason,
        "issued_at": command.issued_at,
        "ttl_s": command.ttl_s,
        "metadata": dict(command.metadata),
    }


def _ack_to_snapshot(ack: CommandAck) -> dict[str, Any]:
    return {
        "command_id": ack.command_id,
        "uav_id": ack.uav_id,
        "status": ack.status.value,
        "issued_at": ack.issued_at,
        "updated_at": ack.updated_at,
        "reason": ack.reason,
        "progress": ack.progress,
    }
