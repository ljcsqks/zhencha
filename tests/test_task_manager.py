from uav_search.core.data_types import Position, Task, TaskStatus, TaskType
from uav_search.maps.grid_map import GridMap
from uav_search.task.task_manager import TaskManager


def test_update_progress_completes_fully_covered_task() -> None:
    grid_map = GridMap(width_m=30, height_m=10, resolution_m=10)
    cells = {Position(0, 0), Position(1, 0)}
    for cell in cells:
        grid_map.set_cell(cell, {"search_confidence": 1.0})
    task = Task(
        id="task_001",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells=cells,
        entry_point=Position(0, 0),
        waypoints=[Position(0, 0), Position(1, 0)],
    )
    manager = TaskManager([task])

    manager.update_progress(grid_map, now=3.0)

    assert task.status == TaskStatus.COMPLETED
    assert task.progress == 1.0


def test_refresh_pending_waypoints_removes_covered_points() -> None:
    grid_map = GridMap(width_m=30, height_m=10, resolution_m=10)
    grid_map.set_cell(Position(0, 0), {"search_confidence": 1.0})
    task = Task(
        id="task_001",
        type=TaskType.SEARCH,
        priority=1.0,
        target_cells={Position(0, 0), Position(1, 0)},
        entry_point=Position(0, 0),
        waypoints=[Position(0, 0), Position(1, 0)],
    )
    manager = TaskManager([task])

    manager.refresh_pending_waypoints(grid_map, now=4.0)

    assert task.status == TaskStatus.PENDING
    assert task.waypoints == [Position(1, 0)]
