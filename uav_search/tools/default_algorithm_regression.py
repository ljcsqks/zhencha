from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from uav_search.experiments.run_batch import run_batch


DEFAULT_ALGORITHM_VERSION = "adaptive_component_sweep_v1"
BASELINE_ALGORITHM_VERSION = "baseline_sparse_boustrophedon"

DEFAULT_REGRESSION_SCENARIOS = [
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


def run_default_algorithm_regression(
    *,
    output_dir: Path | None = None,
    default_config: Path = Path("config/default.yaml"),
    scenario_dir: Path = Path("config/scenarios"),
    baseline_rows: list[dict[str, Any]] | None = None,
    candidate_rows: list[dict[str, Any]] | None = None,
) -> Path:
    output_dir = output_dir or _timestamped_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)
    scenarios = [_scenario_path(name, scenario_dir) for name in DEFAULT_REGRESSION_SCENARIOS]

    if baseline_rows is None:
        baseline_config = _config_with_algorithm(
            default_config,
            BASELINE_ALGORITHM_VERSION,
            output_dir / "_configs" / "baseline.yaml",
        )
        baseline_rows = run_batch(baseline_config, scenarios, output_dir / "baseline_runs")
    if candidate_rows is None:
        candidate_config = _config_with_algorithm(
            default_config,
            DEFAULT_ALGORITHM_VERSION,
            output_dir / "_configs" / "default_adaptive.yaml",
        )
        candidate_rows = run_batch(candidate_config, scenarios, output_dir / "default_runs")

    report = evaluate_regression(baseline_rows, candidate_rows)
    _write_json(output_dir / "baseline_metrics.json", baseline_rows)
    _write_json(output_dir / "default_metrics.json", candidate_rows)
    _write_json(output_dir / "regression_summary.json", report)
    _write_markdown(output_dir / "regression_summary.md", report)
    return output_dir


def evaluate_regression(
    baseline_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    baseline_by_name = {_scenario_name(row): row for row in baseline_rows}
    candidate_by_name = {_scenario_name(row): row for row in candidate_rows}
    scenario_reports = {
        name: _evaluate_scenario(name, baseline_by_name.get(name, {}), candidate_by_name[name])
        for name in sorted(candidate_by_name)
    }
    overall = _combine_status(report["status"] for report in scenario_reports.values())
    return {
        "default_algorithm_version": DEFAULT_ALGORITHM_VERSION,
        "baseline_algorithm_version": BASELINE_ALGORITHM_VERSION,
        "overall_status": overall,
        "scenarios": scenario_reports,
    }


def _evaluate_scenario(name: str, baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    checks = {
        "final_coverage": _check(
            _num(candidate, "global_coverage") >= 0.95,
            f"final coverage {_num(candidate, 'global_coverage'):.4f} >= 0.95",
        ),
        "priority_coverage": _check(
            bool(candidate.get("priority_goal_met", True)),
            "priority coverage goal met",
        ),
        "no_fly": _check(
            _num(candidate, "no_fly_violations") == 0,
            f"no_fly_violations={_num(candidate, 'no_fly_violations'):.0f}",
        ),
        "task_route_not_found_not_increased": _check(
            _nested(candidate, ["diagnostics", "command_quality", "rejected_reasons", "task_route_not_found"])
            <= _nested(baseline, ["diagnostics", "command_quality", "rejected_reasons", "task_route_not_found"]),
            "task_route_not_found did not increase vs baseline",
        ),
    }
    if _num(candidate, "target_found_count") > 0 or _num(candidate, "confirm_done_count") > 0:
        checks["target_confirmation"] = _check(
            _num(candidate, "confirm_success_rate") >= 1.0
            and _num(candidate, "interrupted_task_resume_rate", 1.0) >= 1.0,
            "target confirmation and interrupted search resume are complete",
        )
    if name == "area_search_3uav":
        checks["three_uav_within_3pct"] = _check(
            _pct_delta(baseline, candidate, "time_to_95_coverage_s") <= 0.03
            and _pct_delta(baseline, candidate, "total_distance_m") <= 0.03
            and _num(candidate, "redundant_coverage_rate") - _num(baseline, "redundant_coverage_rate") <= 0.03,
            "3UAV time, distance, and redundancy stay within the 3% regression budget",
        )
    if name == "area_search_5uav":
        checks["five_uav_not_worse"] = _check(
            _num(candidate, "time_to_95_coverage_s") <= _num(baseline, "time_to_95_coverage_s")
            and _num(candidate, "total_distance_m") <= _num(baseline, "total_distance_m")
            and _num(candidate, "redundant_coverage_rate") <= _num(baseline, "redundant_coverage_rate"),
            "5UAV time_to_95, distance, and redundancy are not worse than baseline",
        )
    if name == "stress_obstacle_maze_3uav":
        checks["maze_better_than_baseline"] = _check(
            _num(candidate, "time_to_95_coverage_s") < _num(baseline, "time_to_95_coverage_s")
            and _num(candidate, "total_distance_m") < _num(baseline, "total_distance_m"),
            "maze time_to_95 and distance remain better than baseline",
        )
    status = _combine_status(check["status"] for check in checks.values())
    return {
        "status": status,
        "algorithm_version": candidate.get("algorithm_version"),
        "final_coverage": candidate.get("global_coverage"),
        "priority_coverage": candidate.get("priority_coverage"),
        "time_to_95_coverage_s": candidate.get("time_to_95_coverage_s"),
        "total_distance_m": candidate.get("total_distance_m"),
        "redundant_coverage_rate": candidate.get("redundant_coverage_rate"),
        "workload_balance": _workload_balance_summary(candidate),
        "checks": checks,
        "planned_vs_actual": _planned_vs_actual(candidate),
    }


def _workload_balance_summary(row: dict[str, Any]) -> dict[str, float]:
    allocation = row.get("diagnostics", {}).get("allocation_quality", {})
    segment = row.get("diagnostics", {}).get("segment_quality", {})
    return {
        "all_uavs": float(allocation.get("workload_balance_all_uavs", row.get("per_uav_workload_balance", 0.0)) or 0.0),
        "active_uavs": float(allocation.get("workload_balance_active_uavs", row.get("per_uav_workload_balance", 0.0)) or 0.0),
        "segment": float(segment.get("segment_workload_balance", 0.0) or 0.0),
    }


def _planned_vs_actual(row: dict[str, Any]) -> dict[str, Any]:
    segment = row.get("diagnostics", {}).get("segment_quality", {})
    planned = float(segment.get("fleet_planned_coverage_ratio", 0.0) or segment.get("planned_coverage_ratio", 0.0) or 0.0)
    actual = float(segment.get("actual_final_coverage_ratio", row.get("global_coverage", 0.0)) or 0.0)
    error = float(segment.get("fleet_planned_vs_actual_coverage_error", actual - planned if planned else 0.0) or 0.0)
    explanation = str(segment.get("planned_vs_actual_explanation") or _explain_planned_gap(error, planned))
    return {
        "planned_coverage_ratio": planned,
        "actual_coverage_ratio": actual,
        "error": error,
        "explanation": explanation,
    }


def _explain_planned_gap(error: float, planned: float) -> str:
    if planned <= 0:
        return "planned coverage unavailable; scenario may use baseline-style tasks without adaptive metadata"
    if abs(error) <= 0.03:
        return "actual coverage is close to planned coverage"
    if error > 0:
        return "actual exceeds plan because connectors, supplemental tasks, and post-goal motion can cover extra cells"
    return "actual is below plan because cancellations, blocked paths, or dynamic updates can prevent planned cells from being visited"


def _check(passed: bool, detail: str, warn: bool = False) -> dict[str, str]:
    return {"status": "PASS" if passed else ("WARN" if warn else "FAIL"), "detail": detail}


def _combine_status(statuses: Any) -> str:
    values = list(statuses)
    if any(value == "FAIL" for value in values):
        return "FAIL"
    if any(value == "WARN" for value in values):
        return "WARN"
    return "PASS"


def _scenario_name(row: dict[str, Any]) -> str:
    if row.get("run_id"):
        return str(row["run_id"])
    return Path(str(row.get("scenario_file", "unknown"))).stem


def _pct_delta(baseline: dict[str, Any], candidate: dict[str, Any], field: str) -> float:
    base = _num(baseline, field)
    if abs(base) < 1e-12:
        return 0.0
    return (_num(candidate, field) - base) / abs(base)


def _num(row: dict[str, Any], field: str, default: float = 0.0) -> float:
    value = row.get(field, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _nested(row: dict[str, Any], path: list[str], default: float = 0.0) -> float:
    value: Any = row
    for key in path:
        if not isinstance(value, dict):
            return default
        value = value.get(key)
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return default


def _timestamped_output_dir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("runs") / f"default_algorithm_regression_{stamp}"


def _config_with_algorithm(default_config: Path, version: str, output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with default_config.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    config.setdefault("algorithm", {})["version"] = version
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=True)
    return output_path


def _scenario_path(name: str, scenario_dir: Path) -> Path:
    return scenario_dir / f"{name}.yaml"


def _write_json(path: Path, payload: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def _write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Default Algorithm Regression",
        "",
        f"- Default: `{report['default_algorithm_version']}`",
        f"- Baseline: `{report['baseline_algorithm_version']}`",
        f"- Overall: **{report['overall_status']}**",
        "",
        "| Scenario | Status | Coverage | Time95 | Distance | Redundant | Workload | Planned vs actual |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for name, item in report["scenarios"].items():
        planned = item["planned_vs_actual"]
        workload = item.get("workload_balance", {})
        lines.append(
            f"| {name} | {item['status']} | {_fmt(item.get('final_coverage'))} | "
            f"{_fmt(item.get('time_to_95_coverage_s'))} | {_fmt(item.get('total_distance_m'))} | "
            f"{_fmt(item.get('redundant_coverage_rate'))} | {_fmt(workload.get('all_uavs'))} | "
            f"{planned['explanation']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.4f}"
    except (TypeError, ValueError):
        return str(value)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the adaptive default algorithm regression suite.")
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--scenario-dir", type=Path, default=Path("config/scenarios"))
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output_dir = run_default_algorithm_regression(
        output_dir=args.output,
        default_config=args.config,
        scenario_dir=args.scenario_dir,
    )
    print(f"default algorithm regression -> {output_dir}")


if __name__ == "__main__":
    main()
