from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class CellType(Enum):
    FREE = "FREE"
    OBSTACLE = "OBSTACLE"
    NO_FLY = "NO_FLY"
    PRIORITY = "PRIORITY"


class UAVStatus(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    CONFIRMING = "CONFIRMING"
    RETURNING = "RETURNING"
    AVOIDING = "AVOIDING"
    OFFLINE = "OFFLINE"


class TaskType(Enum):
    SEARCH = "SEARCH"
    CONFIRM = "CONFIRM"
    RETURN = "RETURN"


class TaskStatus(Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    BLOCKED = "BLOCKED"
    CANCELLED = "CANCELLED"


class EventType(Enum):
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_APPEAR = "TARGET_APPEAR"
    CONFIRM_DONE = "CONFIRM_DONE"
    UAV_OFFLINE = "UAV_OFFLINE"
    UAV_RECOVERED = "UAV_RECOVERED"
    MAP_UPDATE = "MAP_UPDATE"
    LOW_BATTERY = "LOW_BATTERY"
    TASK_BLOCKED = "TASK_BLOCKED"
    CONFLICT_DETECTED = "CONFLICT_DETECTED"


class EventPriority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


class CommandType(Enum):
    HOLD = "HOLD"
    FOLLOW_PATH = "FOLLOW_PATH"
    CONFIRM_TARGET = "CONFIRM_TARGET"
    RETURN_HOME = "RETURN_HOME"
    REPLAN = "REPLAN"


@dataclass(frozen=True, order=True)
class Position:
    x: int
    y: int


@dataclass
class GridCell:
    position: Position
    cell_type: CellType
    passable: bool
    search_confidence: float = 0.0
    search_priority: float = 1.0
    last_search_time: float | None = None
    target_ids: list[str] = field(default_factory=list)


@dataclass
class UAVState:
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


@dataclass
class Target:
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
    sort_key: tuple[int, float] = field(init=False, repr=False)
    id: str
    type: EventType
    timestamp: float
    priority: EventPriority
    source_uav_id: str | None = None
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.sort_key = (-self.priority.value, self.timestamp)


@dataclass
class Assignment:
    task_id: str
    uav_id: str
    bid_value: float
    assigned_at: float


@dataclass
class PathPlan:
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
    uav_id_a: str
    uav_id_b: str
    time: float
    position_a: Position
    position_b: Position
    distance_cells: float
    severity: EventPriority


@dataclass
class DecisionCommand:
    uav_id: str
    command: CommandType
    task_id: str | None
    target: Position | None
    path: list[Position]
    reason: str | None = None


@dataclass
class DecisionOutput:
    timestamp: float
    commands: list[DecisionCommand]
    assignments: list[Assignment]
    events_handled: list[str]
    global_coverage: float
    priority_coverage: float
    decision_latency_ms: float
