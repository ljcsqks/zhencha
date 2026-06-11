from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


def generate_report_charts(snapshots: list[dict[str, Any]], output_dir: str | Path) -> list[Path]:
    """Generate basic report charts from simulation snapshots."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if not snapshots:
        return []

    paths = [
        _plot_coverage(snapshots, output / "coverage_curve.png"),
        _plot_trajectories(snapshots, output / "uav_trajectories.png"),
        _plot_event_timeline(snapshots, output / "event_timeline.png"),
    ]
    return paths


def _plot_coverage(snapshots: list[dict[str, Any]], output_path: Path) -> Path:
    times = [float(snapshot["time_s"]) for snapshot in snapshots]
    global_coverage = [float(snapshot.get("global_coverage", 0.0)) for snapshot in snapshots]
    priority_coverage = [float(snapshot.get("priority_coverage", 0.0)) for snapshot in snapshots]

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=140)
    ax.plot(times, global_coverage, label="global coverage", linewidth=2)
    ax.plot(times, priority_coverage, label="priority coverage", linewidth=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage")
    ax.set_ylim(0.0, 1.05)
    ax.grid(True, alpha=0.25)
    ax.legend()
    ax.set_title("Coverage Over Time")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def _plot_trajectories(snapshots: list[dict[str, Any]], output_path: Path) -> Path:
    tracks: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for snapshot in snapshots:
        for uav in snapshot.get("uavs", []):
            pos = uav["position"]
            tracks[uav["id"]].append((int(pos["x"]), int(pos["y"])))

    fig, ax = plt.subplots(figsize=(7, 5), dpi=140)
    for uav_id, points in sorted(tracks.items()):
        if not points:
            continue
        xs, ys = zip(*points)
        ax.plot(xs, ys, marker="o", markersize=2, linewidth=1.2, label=uav_id)
        ax.scatter([xs[-1]], [ys[-1]], s=35)
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")
    ax.set_title("UAV Trajectories")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def _plot_event_timeline(snapshots: list[dict[str, Any]], output_path: Path) -> Path:
    event_points: list[tuple[float, str]] = []
    for snapshot in snapshots:
        for event_id in snapshot.get("events", []):
            event_points.append((float(snapshot["time_s"]), event_id))

    fig, ax = plt.subplots(figsize=(8, 3.5), dpi=140)
    if event_points:
        times = [point[0] for point in event_points]
        labels = [point[1] for point in event_points]
        y_positions = list(range(len(event_points)))
        ax.scatter(times, y_positions, s=38)
        for time_s, y_pos, label in zip(times, y_positions, labels):
            ax.text(time_s, y_pos + 0.08, label, fontsize=7, rotation=15)
        ax.set_yticks(y_positions)
        ax.set_yticklabels([str(index + 1) for index in y_positions])
    else:
        ax.text(0.5, 0.5, "No events recorded", ha="center", va="center", transform=ax.transAxes)
        ax.set_yticks([])

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Event")
    ax.set_title("Event Timeline")
    ax.grid(True, axis="x", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path
