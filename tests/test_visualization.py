from pathlib import Path

from uav_search.core.data_types import Position, UAVState, UAVStatus
from uav_search.maps.grid_map import GridMap
from uav_search.visualization.static_viewer import render_static_map


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
