from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any
from uuid import uuid4

from uav_search.core.data_types import CommandType, DecisionCommand, Event, Position, UAVStatus


class AckStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    REJECTED = "rejected"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class CommandAck:
    command_id: str
    uav_id: str
    status: AckStatus
    issued_at: float
    updated_at: float
    reason: str | None = None
    progress: float | None = None


@dataclass(frozen=True)
class ControlCommand:
    command_id: str
    command: CommandType
    uav_id: str
    task_id: str | None
    target: Position | None
    path: list[Position]
    issued_at: float
    ttl_s: float | None = None
    reason: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_decision(
        cls,
        command: DecisionCommand,
        issued_at: float,
        ttl_s: float | None = None,
    ) -> "ControlCommand":
        return cls(
            command_id=command.command_id or f"cmd_{uuid4().hex}",
            command=command.command,
            uav_id=command.uav_id,
            task_id=command.task_id,
            target=command.target,
            path=list(command.path),
            issued_at=command.issued_at if command.issued_at is not None else issued_at,
            ttl_s=command.ttl_s if command.ttl_s is not None else ttl_s,
            reason=command.reason,
            metadata=dict(command.metadata),
        )


@dataclass(frozen=True)
class MissionSpec:
    mission_id: str
    width_cells: int
    height_cells: int
    resolution_m: float
    mission_complete_coverage_threshold: float
    sensor_radius_cells: int


@dataclass(frozen=True)
class MapCellObservation:
    cell_type: str
    passable: bool
    search_confidence: float
    search_priority: float
    coverage_count: int


@dataclass(frozen=True)
class MapObservation:
    width_cells: int
    height_cells: int
    resolution_m: float
    cells: list[list[MapCellObservation]]


@dataclass(frozen=True)
class UAVObservation:
    uav_id: str
    position: Position
    status: UAVStatus
    battery: float
    home: Position
    current_command_id: str | None
    current_task_id: str | None
    remaining_path: list[Position]
    last_error: str | None = None


@dataclass(frozen=True)
class Observation:
    tick: int
    time_s: float
    mission_id: str
    mission: MissionSpec
    map: MapObservation
    changed_cells: list[Position]
    uavs: list[UAVObservation]
    events: list[Event]
    command_acks: list[CommandAck]
    metrics_hint: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class DecisionOutput:
    commands: list[ControlCommand]
    task_summary: dict[str, Any]
    target_summary: dict[str, Any]
    metrics_updates: dict[str, Any] = field(default_factory=dict)
    debug: dict[str, Any] = field(default_factory=dict)


class CommandStatusStore:
    def __init__(self, max_count: int = 200, max_age_s: float = 30.0) -> None:
        self.max_count = int(max_count)
        self.max_age_s = float(max_age_s)
        self._history: list[CommandAck] = []
        self._active_by_uav: dict[str, CommandAck] = {}

    def record(self, ack: CommandAck, active: bool = False) -> None:
        self._history.append(ack)
        if active and ack.status in (AckStatus.ACCEPTED, AckStatus.RUNNING):
            self._active_by_uav[ack.uav_id] = ack
        elif ack.uav_id in self._active_by_uav and self._active_by_uav[ack.uav_id].command_id == ack.command_id:
            self._active_by_uav.pop(ack.uav_id, None)

    def recent(self, now: float) -> list[CommandAck]:
        cutoff = now - self.max_age_s
        recent = [ack for ack in self._history if ack.updated_at >= cutoff]
        recent = recent[-self.max_count :]
        by_id = {ack.command_id: ack for ack in recent}
        for ack in self._active_by_uav.values():
            by_id.setdefault(ack.command_id, ack)
        return sorted(by_id.values(), key=lambda ack: (ack.updated_at, ack.command_id))
