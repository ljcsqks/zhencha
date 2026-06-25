from __future__ import annotations

import json
from pathlib import Path

from uav_search.tools.compare_algorithms import compare_algorithms


def test_compare_algorithms_baseline_vs_baseline_outputs_near_zero_deltas(tmp_path: Path) -> None:
    result = compare_algorithms(
        baseline_version="baseline_sparse_boustrophedon",
        candidate_version="baseline_sparse_boustrophedon",
        scenarios=["area_search_1uav"],
        output_dir=tmp_path,
        default_config=Path("config/default.yaml"),
        scenario_dir=Path("config/scenarios"),
    )

    assert (tmp_path / "baseline_metrics.json").exists()
    assert (tmp_path / "candidate_metrics.json").exists()
    assert (tmp_path / "comparison.json").exists()
    assert (tmp_path / "comparison.md").exists()

    comparison = json.loads((tmp_path / "comparison.json").read_text(encoding="utf-8"))
    deltas = comparison["scenarios"]["area_search_1uav"]
    for key in (
        "final_coverage_delta",
        "time_to_95_delta_pct",
        "total_distance_delta_pct",
        "redundant_coverage_delta_pct",
        "post_95_extra_distance_delta_pct",
        "workload_balance_delta",
        "turn_rate_delta",
        "no_fly_violations_delta",
        "confirm_success_rate_delta",
        "interrupted_task_resume_rate_delta",
    ):
        assert abs(float(deltas[key])) < 1e-9
    assert result["baseline_version"] == "baseline_sparse_boustrophedon"
