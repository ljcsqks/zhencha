from __future__ import annotations

import time
from dataclasses import replace
from typing import Any

from uav_search.allocation.auction import SequentialAuction
from uav_search.core.data_types import CommandType, DecisionCommand, DecisionOutput, Event, EventType, Position
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
    """Fixed-cycle decision coordinator for the first multi-UAV implementation."""

    def __init__(self, grid_map: GridMap, fleet: FleetManager, config: dict[str, Any]) -> None:
        self.grid_map = grid_map
        self.fleet = fleet
        self.config = config
        self.planner = PathPlanner(config.get("planning", {}))
        self.map_updater = MapUpdater(grid_map)
        self.auction = SequentialAuction({**config, "battery_threshold": config["uav"]["battery_threshold"]})
        self.task_manager = TaskManager()
        self.event_manager = EventManager(config["scheduler"].get("event_debounce_s", 0.2))
        self._initialized = False

    def regular_cycle(self, now: float = 0.0) -> DecisionOutput:
        """Run task generation, allocation, path planning, and conflict handling once."""
        started = time.perf_counter()
        events_handled: list[str] = []
        commands: list[DecisionCommand] = []

        urgent_commands, urgent_event_ids = self.handle_urgent_events(self.event_manager.poll_events(now))
        commands.extend(urgent_commands)
        events_handled.extend(urgent_event_ids)

        self._ensure_initial_tasks(now)

        assignments = []
        proposed_assignments = self.auction.allocate(
            self.task_manager.get_pending_tasks(),
            self.fleet.get_available_uavs(),
            self.grid_map,
            now=now,
        )

        for proposed in proposed_assignments:
            task = self.task_manager.tasks[proposed.task_id]
            assignment = self.task_manager.assign_task(task.id, proposed.uav_id, now=now, bid_value=proposed.bid_value)
            uav_state = self.fleet.get_uav(proposed.uav_id).state
            route = self._plan_route_through_waypoints(uav_state, task.waypoints)
            if not route:
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

        # Conflict resolution mutates affected UAV paths by adding wait steps.
        conflicts = detect_conflicts(
            self.fleet.get_all_states(),
            safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            time_horizon_steps=int(self.config["planning"]["conflict_time_horizon_steps"]),
        )
        commands.extend(
            resolve_conflicts(
                conflicts,
                self.fleet.get_all_states(),
                safety_distance_cells=float(self.config["planning"]["safety_distance_cells"]),
            )
        )

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
        if event.type == EventType.LOW_BATTERY:
            return self._handle_low_battery(event)
        if event.type == EventType.UAV_OFFLINE:
            return self._handle_uav_offline(event)
        if event.type == EventType.MAP_UPDATE:
            return self._handle_map_update(event)
        return []

    def handle_urgent_events(self, events: list[Event]) -> tuple[list[DecisionCommand], list[str]]:
        commands: list[DecisionCommand] = []
        handled_ids: list[str] = []
        for event in events:
            commands.extend(self.handle_event(event))
            handled_ids.append(event.id)
        return commands, handled_ids

    def _ensure_initial_tasks(self, now: float) -> None:
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

    def _plan_route_through_waypoints(self, uav_state: UAVState, waypoints: list[Position]) -> list[Position]:
        route: list[Position] = []
        current = uav_state.position

        # Plan each leg separately so obstacle-aware A* can connect sparse coverage waypoints.
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
                route.extend(plan.path[1:])
            current = waypoint

        return route

    def _handle_low_battery(self, event: Event) -> list[DecisionCommand]:
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
        updates = event.data.get("updates", [])
        if not updates and "operation" in event.data:
            updates = [event.data]
        self.map_updater.apply_updates(updates)

        commands: list[DecisionCommand] = []
        for state in self.fleet.get_all_states():
            if state.status == UAVStatus.OFFLINE or not state.path:
                continue
            if self.planner.is_path_valid(state.path[state.path_index :], self.grid_map):
                continue

            # Replan only the affected UAV's current route, keeping task ownership stable.
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
