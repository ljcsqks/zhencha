import json
import csv
from pathlib import Path

from uav_search.experiments.run_batch import run_batch


def test_run_batch_outputs_per_scenario_artifacts(tmp_path: Path) -> None:
    rows = run_batch(
        default_config=Path("config/default.yaml"),
        scenario_paths=[Path("config/scenarios/area_search_4uav.yaml"), Path("config/scenarios/area_search_5uav.yaml")],
        output_dir=tmp_path,
    )

    assert len(rows) == 2
    assert (tmp_path / "summary.json").exists()
    assert (tmp_path / "summary.csv").exists()
    for scenario_name in ("area_search_4uav", "area_search_5uav"):
        scenario_dir = tmp_path / scenario_name
        assert (scenario_dir / "snapshots.json").exists()
        assert (scenario_dir / "metrics.json").exists()
        assert (scenario_dir / "final_view.png").exists()
        assert (scenario_dir / "report" / "coverage_curve.png").exists()
        assert (scenario_dir / "report" / "uav_trajectories.png").exists()
        assert (scenario_dir / "report" / "event_timeline.png").exists()

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert {row["run_id"] for row in summary} == {"area_search_4uav", "area_search_5uav"}
    with (tmp_path / "summary.csv").open("r", encoding="utf-8") as handle:
        header = next(csv.reader(handle))
    assert "supplemental_task_count" in header
    assert "ignored_uncovered_cells" in header
    assert "algorithm_version" in header
    assert "config_hash" in header
