from __future__ import annotations

import json
from pathlib import Path

from uav_search.tools.compare_algorithms import compare_algorithms
from uav_search.tools.compare_algorithms import compare_metric_rows


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


def test_compare_metric_rows_preserves_absolute_delta_when_baseline_is_zero() -> None:
    deltas = compare_metric_rows(
        {
            "post_95_extra_distance_m": 0.0,
            "supplemental_task_count": 0,
            "diagnostics": {
                "coverage_quality": {"post_95_search_distance_m": 0.0},
                "command_quality": {"command_rejected_count": 0, "rejected_reasons": {"task_route_not_found": 0}},
            },
        },
        {
            "post_95_extra_distance_m": 120.0,
            "supplemental_task_count": 3,
            "diagnostics": {
                "coverage_quality": {"post_95_search_distance_m": 80.0},
                "command_quality": {"command_rejected_count": 2, "rejected_reasons": {"task_route_not_found": 1}},
            },
        },
    )

    assert deltas["post_95_extra_distance_delta_pct"] is None
    assert deltas["post_95_extra_distance_delta_abs"] == 120.0
    assert deltas["post_95_search_distance_delta_pct"] is None
    assert deltas["post_95_search_distance_delta_abs"] == 80.0
    assert deltas["supplemental_task_count_delta_abs"] == 3.0
    assert deltas["task_route_not_found_delta_abs"] == 1.0
    assert deltas["command_rejected_count_delta_abs"] == 2.0
