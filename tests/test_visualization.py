from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import to_hex

from uav_search.core.data_types import CellType, Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.visualization.realtime_viewer import build_realtime_animation
from uav_search.visualization.static_viewer import TERRAIN_CMAP, TERRAIN_NORM, TERRAIN_TO_VALUE, render_static_map


def test_render_static_map_creates_png(tmp_path: Path) -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    grid_map.mark_covered(Position(1, 1), radius_cells=1, timestamp=1.0)
    state = UAVState(
        id="uav_01",
        position=Position(1, 1),
        velocity_mps=10.0,
        heading_deg=0.0,
        battery=1.0,
        sensor_radius_cells=2,
        status=UAVStatus.SEARCHING,
        home_position=Position(0, 0),
        path=[Position(0, 0), Position(1, 1)],
    )

    output = render_static_map(grid_map, [state], tmp_path / "view.png")

    assert output.exists()
    assert output.stat().st_size > 0


def test_build_realtime_animation_draws_first_frame() -> None:
    grid_map = GridMap(width_m=100, height_m=100, resolution_m=10)
    snapshots = [
        {
            "time_s": 0.0,
            "global_coverage": 0.0,
            "priority_coverage": 0.0,
            "uavs": [{"id": "uav_01", "position": {"x": 0, "y": 0}}],
            "events": [],
        },
        {
            "time_s": 1.0,
            "global_coverage": 0.1,
            "priority_coverage": 0.0,
            "uavs": [{"id": "uav_01", "position": {"x": 1, "y": 1}}],
            "events": ["scenario_target_found_001"],
        },
    ]

    fig, animation = build_realtime_animation(grid_map, snapshots, sensor_radius_cells=1, interval_ms=50)
    fig.canvas.draw()

    assert animation is not None
    assert fig.axes
    plt.close(fig)


def test_terrain_colors_do_not_shift_when_types_are_absent() -> None:
    mapped_colors = [
        to_hex(TERRAIN_CMAP(TERRAIN_NORM(TERRAIN_TO_VALUE[cell_type.value])))
        for cell_type in (CellType.FREE, CellType.PRIORITY, CellType.OBSTACLE, CellType.NO_FLY)
    ]

    assert mapped_colors == ["#f8fafc", "#fde68a", "#334155", "#ef4444"]
