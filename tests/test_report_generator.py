from pathlib import Path

from uav_search.visualization.report_generator import generate_report_charts


def test_generate_report_charts_creates_pngs(tmp_path: Path) -> None:
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
            "global_coverage": 0.5,
            "priority_coverage": 1.0,
            "uavs": [{"id": "uav_01", "position": {"x": 1, "y": 1}}],
            "events": ["scenario_target_found_001"],
        },
    ]

    outputs = generate_report_charts(snapshots, tmp_path)

    assert {path.name for path in outputs} == {
        "coverage_curve.png",
        "uav_trajectories.png",
        "event_timeline.png",
    }
    assert all(path.exists() and path.stat().st_size > 0 for path in outputs)
