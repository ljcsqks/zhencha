from __future__ import annotations

from uav_search.core.data_types import Assignment, Task, TaskStatus


class TaskManager:
    def __init__(self, tasks: list[Task] | None = None) -> None:
        self.tasks: dict[str, Task] = {task.id: task for task in tasks or []}

    def add_tasks(self, tasks: list[Task]) -> None:
        for task in tasks:
            self.tasks[task.id] = task

    def get_pending_tasks(self) -> list[Task]:
        return sorted(
            (task for task in self.tasks.values() if task.status == TaskStatus.PENDING),
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
