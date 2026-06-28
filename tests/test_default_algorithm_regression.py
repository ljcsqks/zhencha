from __future__ import annotations

import json
from pathlib import Path

from uav_search.tools.default_algorithm_regression import (
    DEFAULT_REGRESSION_SCENARIOS,
    evaluate_regression,
    run_default_algorithm_regression,
)


def _row(
    scenario: str,
    *,
    coverage: float = 0.96,
    priority_goal: bool = True,
    no_fly: int = 0,
    t95: float = 100.0,
    distance: float = 1000.0,
    redundancy: float = 0.2,
    confirm_success: float = 0.0,
    target_found: int = 0,
    route_not_found: int = 0,
    planned_error: float = 0.01,
) -> dict:
    return {
        "scenario_file": f"config/scenarios/{scenario}.yaml",
        "global_coverage": coverage,
        "priority_goal_met": priority_goal,
        "no_fly_violations": no_fly,
        "time_to_95_coverage_s": t95,
        "total_distance_m": distance,
        "redundant_coverage_rate": redundancy,
        "target_found_count": target_found,
        "confirm_success_rate": confirm_success,
        "interrupted_task_resume_rate": 1.0,
        "algorithm_version": "adaptive_component_sweep_v1",
        "diagnostics": {
            "allocation_quality": {
                "workload_balance_all_uavs": 0.93,
                "workload_balance_active_uavs": 0.94,
            },
            "command_quality": {"rejected_reasons": {"task_route_not_found": route_not_found}},
            "segment_quality": {
                "segment_workload_balance": 0.95,
                "fleet_planned_coverage_ratio": coverage - planned_error,
                "actual_final_coverage_ratio": coverage,
                "planned_vs_actual_coverage_error": planned_error,
                "planned_vs_actual_explanation": "actual coverage is close to planned coverage",
            },
        },
    }


def test_default_regression_evaluates_cross_algorithm_guardrails() -> None:
    baseline = [
        _row("area_search_3uav", t95=100.0, distance=1000.0, redundancy=0.20),
        _row("area_search_5uav", t95=100.0, distance=1000.0, redundancy=0.20),
        _row("stress_obstacle_maze_3uav", t95=100.0, distance=1000.0),
        _row("area_search_2uav_target_confirm", target_found=1, confirm_success=1.0),
    ]
    candidate = [
        _row("area_search_3uav", t95=102.0, distance=1020.0, redundancy=0.22),
        _row("area_search_5uav", t95=95.0, distance=950.0, redundancy=0.18),
        _row("stress_obstacle_maze_3uav", t95=80.0, distance=900.0),
        _row("area_search_2uav_target_confirm", target_found=1, confirm_success=1.0),
    ]

    report = evaluate_regression(baseline, candidate)

    assert report["overall_status"] == "PASS"
    assert report["scenarios"]["area_search_3uav"]["status"] == "PASS"
    assert report["scenarios"]["area_search_5uav"]["checks"]["five_uav_not_worse"]["status"] == "PASS"
    assert report["scenarios"]["stress_obstacle_maze_3uav"]["checks"]["maze_better_than_baseline"]["status"] == "PASS"
    assert "planned_vs_actual" in report["scenarios"]["area_search_3uav"]
    assert report["scenarios"]["area_search_5uav"]["workload_balance"]["all_uavs"] == 0.93


def test_default_regression_marks_failures_and_writes_artifacts(tmp_path: Path) -> None:
    baseline = [_row("area_search_5uav", t95=100.0, distance=1000.0, redundancy=0.20)]
    candidate = [_row("area_search_5uav", t95=110.0, distance=1010.0, redundancy=0.21, route_not_found=1)]

    report = evaluate_regression(baseline, candidate)

    assert report["overall_status"] == "FAIL"
    assert report["scenarios"]["area_search_5uav"]["checks"]["five_uav_not_worse"]["status"] == "FAIL"
    assert report["scenarios"]["area_search_5uav"]["checks"]["task_route_not_found_not_increased"]["status"] == "FAIL"

    output_dir = run_default_algorithm_regression(
        output_dir=tmp_path,
        baseline_rows=baseline,
        candidate_rows=candidate,
    )

    assert output_dir == tmp_path
    assert (tmp_path / "regression_summary.json").exists()
    assert (tmp_path / "regression_summary.md").exists()
    saved = json.loads((tmp_path / "regression_summary.json").read_text(encoding="utf-8"))
    assert saved["overall_status"] == "FAIL"


def test_default_regression_scenario_set_is_fixed() -> None:
    assert DEFAULT_REGRESSION_SCENARIOS == [
        "area_search_1uav",
        "area_search_2uav",
        "area_search_3uav",
        "area_search_4uav",
        "area_search_5uav",
        "area_search_2uav_target_confirm",
        "stress_obstacle_maze_3uav",
        "stress_fragmented_area_4uav_reachable",
        "stress_5uav_balance",
        "stress_dynamic_obstacle_mid_route",
    ]
