from __future__ import annotations

from dataclasses import replace
from typing import Any

from uav_search.core.contracts import AckStatus, CommandAck, CommandStatusStore, ControlCommand
from uav_search.core.data_types import CommandType, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.uav.fleet_manager import FleetManager


class CommandApplier:
    def __init__(self, fleet: FleetManager, grid_map: GridMap, ack_store: CommandStatusStore | None = None) -> None:
        self.fleet = fleet
        self.grid_map = grid_map
        self.ack_store = ack_store or CommandStatusStore()
        self._active_by_uav: dict[str, ControlCommand] = {}

    @property
    def active_command_ids(self) -> dict[str, str]:
        return {uav_id: command.command_id for uav_id, command in self._active_by_uav.items()}

    def recent_acks(self, now: float) -> list[CommandAck]:
        return self.ack_store.recent(now)

    def apply(self, commands: list[ControlCommand], now: float) -> list[CommandAck]:
        acks: list[CommandAck] = []
        for command in commands:
            ack = self._apply_one(command, now)
            acks.append(ack)
            active = ack.status in (AckStatus.ACCEPTED, AckStatus.RUNNING)
            self.ack_store.record(ack, active=active)
        return acks

    def refresh(self, now: float) -> list[CommandAck]:
        acks: list[CommandAck] = []
        for uav_id, command in list(self._active_by_uav.items()):
            try:
                state = self.fleet.get_uav(uav_id).state
            except KeyError:
                ack = self._ack(command, AckStatus.FAILED, now, "uav_missing")
                self._active_by_uav.pop(uav_id, None)
                acks.append(ack)
                self.ack_store.record(ack)
                continue

            if state.status == UAVStatus.OFFLINE:
                ack = self._ack(command, AckStatus.FAILED, now, "uav_offline")
                self._active_by_uav.pop(uav_id, None)
            elif command.path and state.path_index >= len(state.path) - 1 and state.position == command.path[-1]:
                ack = self._ack(command, AckStatus.COMPLETED, now, "path_completed", progress=1.0)
                self._active_by_uav.pop(uav_id, None)
            else:
                progress = self._progress(state.path_index, len(state.path))
                ack = self._ack(command, AckStatus.RUNNING, now, progress=progress)
            acks.append(ack)
            self.ack_store.record(ack, active=ack.status == AckStatus.RUNNING)
        return acks

    def _apply_one(self, command: ControlCommand, now: float) -> CommandAck:
        if command.ttl_s is not None and now - command.issued_at > command.ttl_s:
            return self._ack(command, AckStatus.REJECTED, now, "ttl_expired")
        try:
            state = self.fleet.get_uav(command.uav_id).state
        except KeyError:
            return self._ack(command, AckStatus.REJECTED, now, "uav_missing")
        if state.status == UAVStatus.OFFLINE:
            return self._ack(command, AckStatus.REJECTED, now, "uav_offline")

        if command.command == CommandType.CANCEL_COMMAND:
            return self._cancel(command, now)
        if command.command == CommandType.REPLAN:
            status = self._status_from_metadata(command) or UAVStatus.SEARCHING
            return self._assign_path(command, status, now)
        if command.command == CommandType.FOLLOW_PATH:
            return self._assign_path(command, UAVStatus.SEARCHING, now)
        if command.command == CommandType.CONFLICT_YIELD:
            if command.metadata.get("advisory") is True or command.metadata.get("effect") == "none":
                return self._ack(command, AckStatus.ACCEPTED, now, "conflict_yield_advisory")
            current_status = self.fleet.get_uav(command.uav_id).state.status
            return self._assign_path(command, current_status, now)
        if command.command == CommandType.CONFIRM_TARGET:
            return self._assign_path(command, UAVStatus.CONFIRMING, now)
        if command.command == CommandType.RETURN_HOME:
            return self._assign_path(command, UAVStatus.RETURNING, now)
        if command.command == CommandType.HOLD:
            if command.metadata.get("advisory") is True or command.metadata.get("effect") == "none":
                return self._ack(command, AckStatus.ACCEPTED, now, "hold_advisory")
            state.path = []
            state.path_index = 0
            state.status = UAVStatus.IDLE
            state.available = True
            self._active_by_uav.pop(command.uav_id, None)
            return self._ack(command, AckStatus.ACCEPTED, now, "hold")
        return self._ack(command, AckStatus.REJECTED, now, "unsupported_command")

    def _assign_path(self, command: ControlCommand, status: UAVStatus, now: float) -> CommandAck:
        if not command.path:
            return self._ack(command, AckStatus.REJECTED, now, "empty_path")
        invalid = [point for point in command.path if not self.grid_map.is_passable(point)]
        if invalid:
            return self._ack(command, AckStatus.REJECTED, now, "path_not_passable")
        state = self.fleet.get_uav(command.uav_id).state
        if self._distance_cells(command.path[0], state.position) > 1.0:
            return self._ack(command, AckStatus.REJECTED, now, "path_start_not_at_uav")
        if not self._is_contiguous(command.path):
            return self._ack(command, AckStatus.REJECTED, now, "path_not_contiguous")
        state.current_task_id = command.task_id
        self.fleet.assign_path(command.uav_id, list(command.path), status=status)
        self._active_by_uav[command.uav_id] = command
        return self._ack(command, AckStatus.ACCEPTED, now, "path_assigned", progress=0.0)

    def _cancel(self, command: ControlCommand, now: float) -> CommandAck:
        target_command_id = command.metadata.get("command_id")
        target_uav_id = command.uav_id
        target = self._active_by_uav.get(target_uav_id)
        if target_command_id is not None:
            target = next(
                (
                    active
                    for active in self._active_by_uav.values()
                    if active.command_id == str(target_command_id)
                ),
                None,
            )
            if target is not None:
                target_uav_id = target.uav_id
        if target is None:
            return self._ack(command, AckStatus.CANCELLED, now, "no_active_command")

        state = self.fleet.get_uav(target_uav_id).state
        state.path = []
        state.path_index = 0
        state.status = UAVStatus.IDLE
        state.available = True
        state.current_task_id = None
        self._active_by_uav.pop(target_uav_id, None)
        cancelled = self._ack(target, AckStatus.CANCELLED, now, "cancelled_by_command")
        self.ack_store.record(cancelled)
        return cancelled

    def _ack(
        self,
        command: ControlCommand,
        status: AckStatus,
        now: float,
        reason: str | None = None,
        progress: float | None = None,
    ) -> CommandAck:
        return CommandAck(
            command_id=command.command_id,
            uav_id=command.uav_id,
            status=status,
            issued_at=command.issued_at,
            updated_at=now,
            reason=reason,
            progress=progress,
        )

    @staticmethod
    def _progress(path_index: int, path_length: int) -> float | None:
        if path_length <= 1:
            return None
        return max(0.0, min(1.0, path_index / (path_length - 1)))

    @staticmethod
    def _distance_cells(first, second) -> float:
        return max(abs(first.x - second.x), abs(first.y - second.y))

    @classmethod
    def _is_contiguous(cls, path) -> bool:
        return all(cls._distance_cells(first, second) <= 1.0 for first, second in zip(path, path[1:]))

    @staticmethod
    def _status_from_metadata(command: ControlCommand) -> UAVStatus | None:
        value = command.metadata.get("status")
        if value is None:
            return None
        try:
            return UAVStatus(str(value))
        except ValueError:
            return None
