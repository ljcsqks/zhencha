from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Patch

from uav_search.core.data_types import CellType, UAVState
from uav_search.maps.grid_map import GridMap


TERRAIN_TO_VALUE = {
    CellType.FREE.value: 0,
    CellType.PRIORITY.value: 1,
    CellType.OBSTACLE.value: 2,
    CellType.NO_FLY.value: 3,
}
TERRAIN_COLORS = ["#f8fafc", "#fde68a", "#334155", "#ef4444"]
TERRAIN_CMAP = ListedColormap(TERRAIN_COLORS)
TERRAIN_NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], TERRAIN_CMAP.N)


def render_static_map(
    grid_map: GridMap,
    uav_states: list[UAVState],
    output_path: str | Path,
    title: str = "UAV Search Simulation",
    snapshots: list[dict[str, Any]] | None = None,
) -> Path:
    """Render terrain, coverage, final UAV positions, and flown trajectories."""
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    terrain_values = np.vectorize(TERRAIN_TO_VALUE.get)(grid_map.terrain)
    fig, ax = plt.subplots(figsize=(10, 7), dpi=140)

    ax.imshow(
        terrain_values,
        origin="lower",
        cmap=TERRAIN_CMAP,
        norm=TERRAIN_NORM,
        interpolation="nearest",
        alpha=0.95,
    )

    # Coverage is drawn as a translucent blue layer so terrain remains readable.
    coverage = np.ma.masked_where(grid_map.search_confidence <= 0.0, grid_map.search_confidence)
    ax.imshow(coverage, origin="lower", cmap="Blues", interpolation="nearest", alpha=0.42, vmin=0.0, vmax=1.0)

    tracks = _tracks_from_snapshots(snapshots or [])
    for index, state in enumerate(uav_states):
        color = f"C{index % 10}"
        track = tracks.get(state.id, [])
        if track:
            xs, ys = zip(*track)
            ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.9)
        elif state.path:
            xs = [pos.x for pos in state.path]
            ys = [pos.y for pos in state.path]
            ax.plot(xs, ys, color=color, linewidth=1.3, alpha=0.85)
        ax.scatter([state.position.x], [state.position.y], color=color, s=42, edgecolors="black", linewidths=0.6)
        ax.text(state.position.x + 0.25, state.position.y + 0.25, state.id, fontsize=7, color="black")

    ax.set_title(title)
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")
    ax.set_xlim(-0.5, grid_map.width_cells - 0.5)
    ax.set_ylim(-0.5, grid_map.height_cells - 0.5)
    ax.set_aspect("equal")
    ax.grid(color="#cbd5e1", linewidth=0.25, alpha=0.35)
    ax.legend(
        handles=[
            Patch(facecolor="#f8fafc", edgecolor="#94a3b8", label="free"),
            Patch(facecolor="#fde68a", edgecolor="#94a3b8", label="priority"),
            Patch(facecolor="#334155", edgecolor="#94a3b8", label="obstacle"),
            Patch(facecolor="#ef4444", edgecolor="#94a3b8", label="no-fly"),
            Patch(facecolor="#60a5fa", alpha=0.42, label="covered"),
        ],
        loc="upper right",
        fontsize=7,
        framealpha=0.85,
    )

    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)
    return output


def _tracks_from_snapshots(snapshots: list[dict[str, Any]]) -> dict[str, list[tuple[int, int]]]:
    tracks: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            pos = uav.get("position", {})
            point = (int(pos.get("x", 0)), int(pos.get("y", 0)))
            if not tracks[str(uav.get("id"))] or tracks[str(uav.get("id"))][-1] != point:
                tracks[str(uav.get("id"))].append(point)
    return tracks
