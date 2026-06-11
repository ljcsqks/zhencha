from __future__ import annotations

from uav_search.allocation.bid_calculator import calculate_bid
from uav_search.core.data_types import Assignment, Task, TaskType, UAVState
from uav_search.maps.grid_map import GridMap


class SequentialAuction:
    def __init__(self, config: dict) -> None:
        self.config = config

    def allocate(
        self,
        pending_tasks: list[Task],
        available_uavs: list[UAVState],
        grid_map: GridMap,
        now: float = 0.0,
    ) -> list[Assignment]:
        assignments: list[Assignment] = []
        available_by_id = {uav.id: uav for uav in available_uavs}

        for task in sorted(pending_tasks, key=lambda item: (-item.priority, item.created_at, item.id)):
            if not available_by_id:
                break

            if task.type == TaskType.CONFIRM and task.assigned_uav_id in available_by_id:
                winner = available_by_id.pop(task.assigned_uav_id)
                winner.assigned_task_count += 1
                assignments.append(Assignment(task.id, winner.id, 0.0, now))
                continue

            bids: list[tuple[float, UAVState]] = []
            for uav in available_by_id.values():
                bid = calculate_bid(uav, task, grid_map, self.config)
                if bid is not None:
                    bids.append((bid, uav))

            if not bids:
                continue

            bid_value, winner = min(bids, key=lambda item: (item[0], item[1].id))
            winner.assigned_task_count += 1
            available_by_id.pop(winner.id)
            assignments.append(Assignment(task.id, winner.id, bid_value, now))

        return assignments
