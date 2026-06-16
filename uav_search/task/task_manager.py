from __future__ import annotations

from uav_search.core.data_types import Assignment, Task, TaskStatus
from uav_search.maps.grid_map import GridMap


class TaskManager:
    def __init__(self, tasks: list[Task] | None = None) -> None:
        self.tasks: dict[str, Task] = {task.id: task for task in tasks or []}

    def add_tasks(self, tasks: list[Task]) -> None:
        for task in tasks:
            self.tasks[task.id] = task

    def get_pending_tasks(self) -> list[Task]:
        return sorted(
            # Empty-waypoint tasks are considered complete or stale and should not enter allocation.
            (task for task in self.tasks.values() if task.status == TaskStatus.PENDING and task.waypoints),
            key=lambda task: (-task.priority, task.created_at, task.id),
        )

    def get_active_tasks(self) -> list[Task]:
        return [
            task
            for task in self.tasks.values()
            if task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS)
        ]

    def assign_task(self, task_id: str, uav_id: str, now: float = 0.0, bid_value: float = 0.0) -> Assignment:
        task = self.tasks[task_id]
        task.assigned_uav_id = uav_id
        task.status = TaskStatus.ASSIGNED
        task.updated_at = now
        return Assignment(task_id=task_id, uav_id=uav_id, bid_value=bid_value, assigned_at=now)

    def start_task(self, task_id: str, now: float = 0.0) -> None:
        task = self.tasks[task_id]
        task.status = TaskStatus.IN_PROGRESS
        task.updated_at = now

    def complete_task(self, task_id: str, now: float = 0.0) -> None:
        task = self.tasks[task_id]
        task.status = TaskStatus.COMPLETED
        task.progress = 1.0
        task.updated_at = now

    def mark_blocked(self, task_id: str, now: float = 0.0) -> None:
        task = self.tasks[task_id]
        task.status = TaskStatus.BLOCKED
        task.updated_at = now

    def requeue_task(self, task_id: str, now: float = 0.0) -> None:
        task = self.tasks[task_id]
        task.status = TaskStatus.PENDING
        task.assigned_uav_id = None
        task.updated_at = now

    def update_progress(
        self,
        grid_map: GridMap,
        now: float = 0.0,
        coverage_threshold: float = 0.95,
    ) -> None:
        for task in self.tasks.values():
            if task.status in (TaskStatus.COMPLETED, TaskStatus.CANCELLED, TaskStatus.BLOCKED):
                continue
            if not task.target_cells:
                continue

            # A task is complete only when every target cell reaches the configured confidence threshold.
            covered = sum(
                1 for cell in task.target_cells if grid_map.get_cell(cell).search_confidence >= coverage_threshold
            )
            task.progress = covered / len(task.target_cells)
            task.updated_at = now
            if task.progress >= 1.0:
                self.complete_task(task.id, now=now)

    def refresh_pending_waypoints(
        self,
        grid_map: GridMap,
        now: float = 0.0,
        coverage_threshold: float = 0.95,
    ) -> None:
        for task in self.tasks.values():
            if task.status != TaskStatus.PENDING:
                continue
            # When an interrupted task returns to the queue, keep only waypoints that still need search coverage.
            task.waypoints = [
                waypoint
                for waypoint in task.waypoints
                if grid_map.get_cell(waypoint).search_confidence < coverage_threshold
            ]
            task.updated_at = now
            if not task.waypoints:
                self.complete_task(task.id, now=now)
