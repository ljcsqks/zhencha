from __future__ import annotations

import math
from dataclasses import replace

from uav_search.core.data_types import Conflict, DecisionCommand, EventPriority, Position, UAVState, UAVStatus
from uav_search.core.data_types import CommandType


STATUS_PRIORITY = {
    UAVStatus.CONFIRMING: 5,
    UAVStatus.RETURNING: 4,
    UAVStatus.SEARCHING: 3,
    UAVStatus.AVOIDING: 2,
    UAVStatus.IDLE: 1,
    UAVStatus.OFFLINE: 0,
}


def detect_conflicts(
    uav_states: list[UAVState],
    safety_distance_cells: float,
    time_horizon_steps: int,
) -> list[Conflict]:
    """Expand planned paths into time steps and find pairwise spacing violations."""
    conflicts: list[Conflict] = []
    active_states = [state for state in uav_states if state.status != UAVStatus.OFFLINE and state.path]

    for index, first in enumerate(active_states):
        for second in active_states[index + 1 :]:
            for step in range(time_horizon_steps + 1):
                pos_a = _position_at_step(first, step)
                pos_b = _position_at_step(second, step)
                distance = _distance(pos_a, pos_b)
                if distance < safety_distance_cells:
                    conflicts.append(
                        Conflict(
                            uav_id_a=first.id,
                            uav_id_b=second.id,
                            time=float(step),
                            position_a=pos_a,
                            position_b=pos_b,
                            distance_cells=distance,
                            severity=EventPriority.HIGH,
                        )
                    )
                    break
    return conflicts


def resolve_conflicts(
    conflicts: list[Conflict],
    uav_states: list[UAVState],
    safety_distance_cells: float,
    max_iterations: int = 20,
) -> list[DecisionCommand]:
    """Resolve conflicts by inserting wait steps for the lower-priority UAV.

    This is intentionally conservative for the first implementation: it avoids
    replanning around temporary path reservations and gives us deterministic,
    easy-to-test behavior. Later stages can add path-offset replanning here.
    """
    states_by_id = {state.id: replace(state, path=list(state.path)) for state in uav_states}
    commands: list[DecisionCommand] = []

    for _ in range(max_iterations):
        current_conflicts = detect_conflicts(
            list(states_by_id.values()),
            safety_distance_cells=safety_distance_cells,
            time_horizon_steps=max((len(state.path) for state in states_by_id.values()), default=0),
        )
        if not current_conflicts:
            return commands

        conflict = current_conflicts[0]
        first = states_by_id[conflict.uav_id_a]
        second = states_by_id[conflict.uav_id_b]
        waiting_state = _lower_priority(first, second)
        _insert_wait_before_conflict(waiting_state, int(conflict.time))
        commands.append(
            DecisionCommand(
                uav_id=waiting_state.id,
                command=CommandType.CONFLICT_YIELD,
                task_id=waiting_state.current_task_id,
                target=waiting_state.path[-1] if waiting_state.path else None,
                path=[],
                reason="conflict_time_offset",
                metadata={
                    "effect": "none",
                    "advisory": True,
                    "suggested_effect": "path_time_offset",
                    "suggested_path": [{"x": point.x, "y": point.y} for point in waiting_state.path],
                },
            )
        )

    return commands


def _position_at_step(state: UAVState, step: int) -> Position:
    if not state.path:
        return state.position
    index = min(state.path_index + step, len(state.path) - 1)
    return state.path[index]


def _insert_wait_before_conflict(state: UAVState, conflict_step: int) -> None:
    if not state.path:
        return
    insert_index = max(state.path_index, min(state.path_index + conflict_step, len(state.path) - 1))
    wait_position = state.path[max(insert_index - 1, state.path_index)]
    state.path.insert(insert_index, wait_position)


def _lower_priority(first: UAVState, second: UAVState) -> UAVState:
    first_key = (STATUS_PRIORITY[first.status], -first.battery, first.id)
    second_key = (STATUS_PRIORITY[second.status], -second.battery, second.id)
    return first if first_key < second_key else second


def _distance(a: Position, b: Position) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
