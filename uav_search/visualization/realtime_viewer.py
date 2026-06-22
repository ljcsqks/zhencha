from __future__ import annotations

from collections import defaultdict
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Patch

from uav_search.core.data_types import CellType, Position
from uav_search.maps.grid_map import GridMap
from uav_search.visualization.static_viewer import TERRAIN_CMAP, TERRAIN_NORM, TERRAIN_TO_VALUE


def build_realtime_animation(
    grid_map: GridMap,
    snapshots: list[dict[str, Any]],
    sensor_radius_cells: int,
    interval_ms: int = 160,
    repeat: bool = False,
) -> tuple[plt.Figure, FuncAnimation]:
    """Build a matplotlib animation for recorded simulation snapshots.

    The animation replays recorded UAV positions and reconstructs cumulative
    sensor coverage from those positions. Terrain is taken from the final map
    state, which keeps the player simple and works well for post-run review.
    """
    if not snapshots:
        raise ValueError("snapshots must not be empty")

    terrain_values = np.vectorize(TERRAIN_TO_VALUE.get)(grid_map.terrain)
    coverage = np.zeros((grid_map.height_cells, grid_map.width_cells), dtype=float)
    tracks = _tracks_by_uav(snapshots)

    fig, ax = plt.subplots(figsize=(10, 7), dpi=110)
    ax.imshow(
        terrain_values,
        origin="lower",
        cmap=TERRAIN_CMAP,
        norm=TERRAIN_NORM,
        interpolation="nearest",
        alpha=0.95,
    )
    coverage_image = ax.imshow(
        np.ma.masked_where(coverage <= 0.0, coverage),
        origin="lower",
        cmap="Blues",
        interpolation="nearest",
        alpha=0.42,
        vmin=0.0,
        vmax=1.0,
    )

    line_by_uav = {}
    point_by_uav = {}
    label_by_uav = {}
    for index, uav_id in enumerate(sorted(tracks)):
        color = f"C{index % 10}"
        (line,) = ax.plot([], [], color=color, linewidth=1.2, alpha=0.85)
        point = ax.scatter([], [], color=color, s=42, edgecolors="black", linewidths=0.6)
        label = ax.text(0, 0, uav_id, fontsize=7, color="black")
        line_by_uav[uav_id] = line
        point_by_uav[uav_id] = point
        label_by_uav[uav_id] = label

    event_text = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "#cbd5e1"},
    )

    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")
    ax.set_xlim(-0.5, grid_map.width_cells - 0.5)
    ax.set_ylim(-0.5, grid_map.height_cells - 0.5)
    ax.set_aspect("equal")
    ax.grid(color="#cbd5e1", linewidth=0.25, alpha=0.35)
    ax.legend(handles=_legend_handles(), loc="upper right", fontsize=7, framealpha=0.85)

    def update(frame_index: int):
        _rebuild_coverage(grid_map, snapshots[: frame_index + 1], sensor_radius_cells, coverage)
        coverage_image.set_data(np.ma.masked_where(coverage <= 0.0, coverage))

        snapshot = snapshots[frame_index]
        for uav_id, points in tracks.items():
            visible_points = points[: frame_index + 1]
            if not visible_points:
                continue
            xs = [point[0] for point in visible_points]
            ys = [point[1] for point in visible_points]
            line_by_uav[uav_id].set_data(xs, ys)
            point_by_uav[uav_id].set_offsets([[xs[-1], ys[-1]]])
            label_by_uav[uav_id].set_position((xs[-1] + 0.25, ys[-1] + 0.25))

        events = ", ".join(snapshot.get("events", [])) or "none"
        ax.set_title(
            f"Simulation Playback | t={float(snapshot['time_s']):.1f}s | "
            f"coverage={float(snapshot.get('global_coverage', 0.0)):.3f}"
        )
        event_text.set_text(f"events: {events}")
        return [coverage_image, event_text, *line_by_uav.values(), *point_by_uav.values(), *label_by_uav.values()]

    animation = FuncAnimation(
        fig,
        update,
        frames=len(snapshots),
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    update(0)
    fig.tight_layout()
    return fig, animation


def play_snapshots(
    grid_map: GridMap,
    snapshots: list[dict[str, Any]],
    sensor_radius_cells: int,
    interval_ms: int = 160,
    repeat: bool = False,
) -> None:
    fig, animation = build_realtime_animation(grid_map, snapshots, sensor_radius_cells, interval_ms, repeat)
    # Keep a reference alive while the window is open; otherwise FuncAnimation
    # may be garbage-collected before matplotlib starts the GUI loop.
    fig._uav_search_animation = animation  # type: ignore[attr-defined]
    plt.show()


def _tracks_by_uav(snapshots: list[dict[str, Any]]) -> dict[str, list[tuple[int, int]]]:
    tracks: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for snapshot in snapshots:
        positions_by_id = {
            uav["id"]: (int(uav["position"]["x"]), int(uav["position"]["y"]))
            for uav in snapshot.get("uavs", [])
        }
        for uav_id in sorted(positions_by_id):
            tracks[uav_id].append(positions_by_id[uav_id])
    return tracks


def _rebuild_coverage(
    grid_map: GridMap,
    snapshots: list[dict[str, Any]],
    sensor_radius_cells: int,
    coverage: np.ndarray,
) -> None:
    coverage.fill(0.0)
    radius_sq = sensor_radius_cells * sensor_radius_cells
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            center = Position(int(uav["position"]["x"]), int(uav["position"]["y"]))
            for y in range(center.y - sensor_radius_cells, center.y + sensor_radius_cells + 1):
                for x in range(center.x - sensor_radius_cells, center.x + sensor_radius_cells + 1):
                    pos = Position(x, y)
                    if not grid_map.is_passable(pos):
                        continue
                    if (x - center.x) ** 2 + (y - center.y) ** 2 <= radius_sq:
                        coverage[y, x] = 1.0


def _legend_handles() -> list[Patch]:
    return [
        Patch(facecolor="#f8fafc", edgecolor="#94a3b8", label="free"),
        Patch(facecolor="#fde68a", edgecolor="#94a3b8", label="priority"),
        Patch(facecolor="#334155", edgecolor="#94a3b8", label="obstacle"),
        Patch(facecolor="#ef4444", edgecolor="#94a3b8", label="no-fly"),
        Patch(facecolor="#60a5fa", alpha=0.42, label="covered"),
    ]
