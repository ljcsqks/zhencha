"""
核心数据类型定义模块

该模块定义了整个无人机协同搜索系统中使用的核心数据结构，包括：
- 枚举类型：定义系统中的各种状态和类型
- 数据类：定义系统中的核心实体数据结构

这些数据类型是整个系统的基础，用于模块间的数据传递和状态管理。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class CellType(Enum):
    """栅格单元类型枚举

    定义地图中每个栅格的基本类型：
    - FREE: 自由区域，无人机可飞越
    - OBSTACLE: 障碍物，不可飞越（如建筑物）
    - NO_FLY: 禁飞区，不可飞越（如政府机关、学校上空）
    - PRIORITY: 重点区域，搜索优先级高（如事发地周边）
    """
    FREE = "FREE"
    OBSTACLE = "OBSTACLE"
    NO_FLY = "NO_FLY"
    PRIORITY = "PRIORITY"


class UAVStatus(Enum):
    """无人机状态枚举

    定义无人机在系统中的运行状态：
    - IDLE: 待命状态，空闲可接受任务
    - SEARCHING: 执行搜索任务中
    - CONFIRMING: 抵近确认目标中（发现目标后的响应状态）
    - RETURNING: 返航中（低电量或任务完成）
    - AVOIDING: 临时避障中
    - OFFLINE: 离线状态（故障或通信中断）
    """
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    CONFIRMING = "CONFIRMING"
    RETURNING = "RETURNING"
    AVOIDING = "AVOIDING"
    OFFLINE = "OFFLINE"


class TaskType(Enum):
    """任务类型枚举

    定义系统中的任务类型：
    - SEARCH: 区域搜索任务，覆盖指定区域
    - CONFIRM: 目标确认任务，抵近确认发现的目标
    - RETURN: 返航任务，返回起飞点
    """
    SEARCH = "SEARCH"
    CONFIRM = "CONFIRM"
    RETURN = "RETURN"


class TaskStatus(Enum):
    """任务状态枚举

    定义任务在生命周期中的状态：
    - PENDING: 待分配，在任务池中等待
    - ASSIGNED: 已分配，已指定执行无人机但未开始
    - IN_PROGRESS: 执行中，无人机正在执行
    - COMPLETED: 已完成，任务成功结束
    - BLOCKED: 阻塞，无法执行（如路径规划失败）
    - CANCELLED: 已取消，任务被撤销
    """
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    BLOCKED = "BLOCKED"
    CANCELLED = "CANCELLED"


class EventType(Enum):
    """事件类型枚举

    定义系统中可能发生的各种事件：
    - TARGET_FOUND: 发现目标，触发确认任务
    - TARGET_APPEAR: 目标出现（场景预设事件）
    - CONFIRM_DONE: 目标确认完成
    - UAV_OFFLINE: 无人机离线（故障或通信中断）
    - UAV_RECOVERED: 无人机恢复在线
    - MAP_UPDATE: 地图更新（新障碍物、区域变化）
    - LOW_BATTERY: 低电量告警，触发返航
    - TASK_BLOCKED: 任务阻塞事件
    - CONFLICT_DETECTED: 检测到冲突
    """
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_APPEAR = "TARGET_APPEAR"
    CONFIRM_DONE = "CONFIRM_DONE"
    CONFIRM_FAILED = "CONFIRM_FAILED"
    UAV_OFFLINE = "UAV_OFFLINE"
    UAV_RECOVERED = "UAV_RECOVERED"
    MAP_UPDATE = "MAP_UPDATE"
    LOW_BATTERY = "LOW_BATTERY"
    TASK_BLOCKED = "TASK_BLOCKED"
    CONFLICT_DETECTED = "CONFLICT_DETECTED"


class EventPriority(Enum):
    """事件优先级枚举

    定义事件处理的优先级顺序（数值越大优先级越高）：
    - LOW: 低优先级，常规处理
    - NORMAL: 普通优先级
    - HIGH: 高优先级，优先处理
    - CRITICAL: 关键优先级，立即处理
    """
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


class CommandType(Enum):
    """指令类型枚举

    定义决策输出给无人机的指令类型：
    - HOLD: 悬停等待，原地待命
    - FOLLOW_PATH: 跟随路径，执行给定路径
    - CONFIRM_TARGET: 确认目标，抵近目标确认
    - RETURN_HOME: 返航，返回起飞点
    - REPLAN: 重规划，使用新路径替代原路径
    """
    HOLD = "HOLD"
    FOLLOW_PATH = "FOLLOW_PATH"
    CONFIRM_TARGET = "CONFIRM_TARGET"
    RETURN_HOME = "RETURN_HOME"
    REPLAN = "REPLAN"
    CANCEL_COMMAND = "CANCEL_COMMAND"
    CONFLICT_YIELD = "CONFLICT_YIELD"


@dataclass(frozen=True, order=True)
class Position:
    """位置坐标数据类

    表示栅格地图中的二维坐标位置。
    使用栅格索引而非真实坐标，便于地图操作。

    属性：
        x: 横向栅格索引（列号）
        y: 纵向栅格索引（行号）

    注意：
        - frozen=True 使其不可变，可作为字典键和集合元素
        - order=True 支持比较操作，便于排序
    """
    x: int
    y: int


@dataclass
class GridCell:
    """栅格单元数据类

    表示地图中单个栅格的所有属性信息。
    每个栅格存储多层状态信息，支持动态更新。

    属性：
        position: 栅格坐标位置
        cell_type: 栅格类型（自由/障碍/禁飞/重点区域）
        passable: 是否可通行（综合考虑障碍物和禁飞区）
        search_confidence: 搜索置信度 [0,1]，0=未搜索，1=已完全覆盖
        search_priority: 搜索优先级，重点区域优先级高
        last_search_time: 上次被搜索的时间戳，None表示从未搜索
        target_ids: 该栅格内发现的目标ID列表

    注意：
        search_confidence 会随时间衰减（适用于移动目标场景）
    """
    position: Position
    cell_type: CellType
    passable: bool
    search_confidence: float = 0.0
    search_priority: float = 1.0
    last_search_time: float | None = None
    target_ids: list[str] = field(default_factory=list)


@dataclass
class UAVState:
    """无人机状态数据类

    完整描述单架无人机的当前状态，包括位置、电量、任务等信息。
    这是系统中最重要的状态数据结构之一。

    属性：
        id: 无人机唯一标识符
        position: 当前栅格坐标位置
        velocity_mps: 当前飞行速度（米/秒）
        heading_deg: 航向角（度）
        battery: 剩余电量百分比 [0,100]
        sensor_radius_cells: 传感器覆盖半径（栅格数）
        status: 当前状态（IDLE/SEARCHING/CONFIRMING等）
        home_position: 起飞点位置，用于返航
        current_task_id: 当前执行的任务ID，None表示无任务
        path: 当前路径点序列（栅格坐标列表）
        path_index: 当前路径执行到的索引位置
        available: 是否可接受新任务
        assigned_task_count: 已分配任务数量，用于负载均衡
        total_distance_m: 总飞行距离（米）
        effective_search_distance_m: 有效搜索飞行距离（米）

    注意：
        available=False 表示无人机正在执行任务或处于特殊状态
    """
    id: str
    position: Position
    velocity_mps: float
    heading_deg: float
    battery: float
    sensor_radius_cells: int
    status: UAVStatus
    home_position: Position
    current_task_id: str | None = None
    path: list[Position] = field(default_factory=list)
    path_index: int = 0
    available: bool = True
    assigned_task_count: int = 0
    total_distance_m: float = 0.0
    effective_search_distance_m: float = 0.0


@dataclass
class Task:
    """任务数据类

    描述搜索任务的完整信息，包括目标区域、状态、分配等。
    任务是系统调度的基本单位。

    属性：
        id: 任务唯一标识符
        type: 任务类型（SEARCH/CONFIRM/RETURN）
        priority: 任务优先级，数值越大优先级越高
        target_cells: 目标栅格集合，需要覆盖或确认的位置
        entry_point: 建议进入点坐标，优化路径规划
        status: 任务状态（PENDING/ASSIGNED/IN_PROGRESS等）
        assigned_uav_id: 分配的无人机ID，None表示未分配
        waypoints: 航路点序列，具体的飞行路径点
        estimated_cost_m: 预估完成代价（飞行距离，米）
        created_at: 任务创建时间戳
        updated_at: 任务最后更新时间戳
        progress: 任务完成进度 [0,1]
        source_event_id: 触发该任务的事件ID，用于追溯

    注意：
        CONFIRM 类型任务通常由 TARGET_FOUND 事件触发
    """
    id: str
    type: TaskType
    priority: float
    target_cells: set[Position]
    entry_point: Position
    status: TaskStatus = TaskStatus.PENDING
    assigned_uav_id: str | None = None
    waypoints: list[Position] = field(default_factory=list)
    estimated_cost_m: float = 0.0
    created_at: float = 0.0
    updated_at: float = 0.0
    progress: float = 0.0
    source_event_id: str | None = None
    uncovered_value: float = 0.0
    priority_value: float = 0.0
    score: float = 0.0
    coverage_waypoints: list[Position] = field(default_factory=list)
    last_replan_time: float = 0.0
    replan_count: int = 0
    resume_owner_id: str | None = None
    allowed_uav_ids: set[str] | None = None

    def __post_init__(self) -> None:
        if not self.coverage_waypoints and self.waypoints:
            self.coverage_waypoints = list(self.waypoints)
        elif self.coverage_waypoints and not self.waypoints:
            self.waypoints = list(self.coverage_waypoints)


@dataclass
class Target:
    """目标数据类

    描述搜索任务中发现的目标信息。

    属性：
        id: 目标唯一标识符
        position: 目标栅格坐标位置
        target_type: 目标类型（如"人员"、"车辆"等）
        confidence: 目标识别置信度 [0,1]
        first_seen_time: 首次发现时间戳
        discovered_by: 发现该目标的无人机ID
        confirmed: 是否已确认
        confirmed_time: 确认时间戳，None表示未确认

    注意：
        确认后置信度通常更高，可作为真实目标处理
    """
    id: str
    position: Position
    target_type: str
    confidence: float
    first_seen_time: float
    discovered_by: str | None = None
    confirmed: bool = False
    confirmed_time: float | None = None


@dataclass(order=True)
class Event:
    """事件数据类

    表示系统中发生的各类事件，用于驱动动态决策。
    事件按优先级和时间戳排序，高优先级事件优先处理。

    属性：
        sort_key: 排序键（内部使用，自动计算）
        id: 事件唯一标识符
        type: 事件类型（TARGET_FOUND/UAV_OFFLINE等）
        timestamp: 事件发生时间戳
        priority: 事件优先级
        source_uav_id: 触发该事件的无人机ID，None表示系统事件
        data: 事件详细数据字典，存储事件特定信息

    注意：
        - sort_key 自动计算为 (-priority, timestamp)，确保高优先级先处理
        - order=True 支持排序，使其可用于优先队列
    """
    sort_key: tuple[int, float] = field(init=False, repr=False)
    id: str
    type: EventType
    timestamp: float
    priority: EventPriority
    source_uav_id: str | None = None
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """初始化后自动计算排序键"""
        self.sort_key = (-self.priority.value, self.timestamp)


@dataclass
class Assignment:
    """任务分配记录数据类

    记录任务分配的结果，用于追踪和评估。

    属性：
        task_id: 被分配的任务ID
        uav_id: 接受任务的无人机ID
        bid_value: 竞标值，数值越低表示代价越小
        assigned_at: 分配时间戳

    注意：
        bid_value 来自拍卖算法，可用于评估分配质量
    """
    task_id: str
    uav_id: str
    bid_value: float
    assigned_at: float


@dataclass
class PathPlan:
    """路径规划结果数据类

    存储路径规划的完整结果，包括路径、代价、有效性等。

    属性：
        uav_id: 无人机ID
        task_id: 关联的任务ID，None表示无任务路径
        start: 起点
        goal: 终点
        path: 规划出的路径点序列
        cost: 路径代价（通常是飞行距离或时间）
        valid: 路径是否有效
        reason: 无效原因或规划说明
        planned_at: 规划时间戳
        latency_ms: 规划耗时（毫秒）

    注意：
        valid=False 时 path 可能为空，reason 会说明失败原因
    """
    uav_id: str
    task_id: str | None
    start: Position
    goal: Position
    path: list[Position]
    cost: float
    valid: bool
    reason: str | None = None
    planned_at: float = 0.0
    latency_ms: float = 0.0


@dataclass
class Conflict:
    """冲突记录数据类

    记录检测到的无人机冲突信息，用于冲突消解。

    属性：
        uav_id_a: 冲突无人机A的ID
        uav_id_b: 冲突无人机B的ID
        time: 冲突发生时间戳
        position_a: 无人机A在冲突时的位置
        position_b: 无人机B在冲突时的位置
        distance_cells: 两无人机间的距离（栅格数）
        severity: 冲突严重程度

    注意：
        distance_cells < safety_distance_cells 时视为冲突
    """
    uav_id_a: str
    uav_id_b: str
    time: float
    position_a: Position
    position_b: Position
    distance_cells: float
    severity: EventPriority


@dataclass
class DecisionCommand:
    """决策指令数据类

    表示决策模块输出给无人机的具体指令。
    这是决策层与执行层的接口数据结构。

    属性：
        uav_id: 目标无人机ID
        command: 指令类型（HOLD/FOLLOW_PATH等）
        task_id: 关联的任务ID，None表示无任务指令
        target: 目标位置，None表示无特定目标
        path: 路径点序列，空列表表示无路径
        reason: 指令原因说明

    注意：
        这是决策输出的核心数据结构，仿真引擎据此更新无人机状态
    """
    uav_id: str
    command: CommandType
    task_id: str | None
    target: Position | None
    path: list[Position]
    reason: str | None = None
    command_id: str | None = None
    issued_at: float | None = None
    ttl_s: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class DecisionOutput:
    """决策输出数据类

    单次决策周期的完整输出结果，包含指令、分配、事件处理等。

    属性：
        timestamp: 决策时间戳
        commands: 决策指令列表
        assignments: 任务分配记录列表
        events_handled: 已处理的事件ID列表
        global_coverage: 全局覆盖率 [0,1]
        priority_coverage: 重点区域覆盖率 [0,1]
        decision_latency_ms: 决策耗时（毫秒）

    注意：
        decision_latency_ms 应控制在1000ms以内，满足实时性要求
    """
    timestamp: float
    commands: list[DecisionCommand]
    assignments: list[Assignment]
    events_handled: list[str]
    global_coverage: float
    priority_coverage: float
    decision_latency_ms: float
