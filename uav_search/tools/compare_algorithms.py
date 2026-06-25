from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from uav_search.experiments.run_batch import run_batch


DELTA_FIELDS = {
    "final_coverage_delta": ("global_coverage", "diff"),
    "time_to_95_delta_pct": ("time_to_95_coverage_s", "pct"),
    "total_distance_delta_pct": ("total_distance_m", "pct"),
    "redundant_coverage_delta_pct": ("redundant_coverage_rate", "pct"),
    "post_95_extra_distance_delta_pct": ("post_95_extra_distance_m", "pct"),
    "post_95_search_distance_delta_pct": ("diagnostics.coverage_quality.post_95_search_distance_m", "pct"),
    "workload_balance_delta": ("diagnostics.allocation_quality.workload_balance_all_uavs", "diff"),
    "workload_balance_active_delta": ("diagnostics.allocation_quality.workload_balance_active_uavs", "diff"),
    "segment_count_delta": ("diagnostics.segment_quality.segment_count_total", "diff"),
    "unique_segment_count_delta": ("diagnostics.segment_quality.unique_segment_count", "diff"),
    "segment_workload_balance_delta": ("diagnostics.segment_quality.segment_workload_balance", "diff"),
    "turn_rate_delta": ("turn_rate", "diff"),
    "no_fly_violations_delta": ("no_fly_violations", "diff"),
    "confirm_success_rate_delta": ("confirm_success_rate", "diff"),
    "interrupted_task_resume_rate_delta": ("interrupted_task_resume_rate", "diff"),
}


def compare_algorithms(
    *,
    baseline_version: str,
    candidate_version: str,
    scenarios: list[str],
    output_dir: Path,
    default_config: Path = Path("config/default.yaml"),
    scenario_dir: Path = Path("config/scenarios"),
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    baseline_config = _config_with_algorithm(default_config, baseline_version, output_dir / "_configs" / "baseline.yaml")
    candidate_config = _config_with_algorithm(default_config, candidate_version, output_dir / "_configs" / "candidate.yaml")
    scenario_paths = [_scenario_path(item, scenario_dir) for item in scenarios]

    baseline_rows = run_batch(baseline_config, scenario_paths, output_dir / "baseline_runs")
    candidate_rows = run_batch(candidate_config, scenario_paths, output_dir / "candidate_runs")

    baseline_by_name = {Path(row["scenario_file"]).stem: row for row in baseline_rows}
    candidate_by_name = {Path(row["scenario_file"]).stem: row for row in candidate_rows}
    comparison = {
        "baseline_version": baseline_version,
        "candidate_version": candidate_version,
        "scenarios": {
            name: _compare_rows(baseline_by_name[name], candidate_by_name[name])
            for name in sorted(baseline_by_name)
        },
    }

    _write_json(output_dir / "baseline_metrics.json", baseline_rows)
    _write_json(output_dir / "candidate_metrics.json", candidate_rows)
    _write_json(output_dir / "comparison.json", comparison)
    _write_markdown(output_dir / "comparison.md", comparison)
    return comparison


def _compare_rows(baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, float]:
    return {
        output_field: _delta(_value(baseline, source_field), _value(candidate, source_field), mode)
        for output_field, (source_field, mode) in DELTA_FIELDS.items()
    }


def _delta(baseline: float | None, candidate: float | None, mode: str) -> float:
    base = 0.0 if baseline is None else float(baseline)
    cand = 0.0 if candidate is None else float(candidate)
    if mode == "pct":
        return 0.0 if abs(base) < 1e-12 else (cand - base) / abs(base)
    return cand - base


def _value(row: dict[str, Any], field: str) -> float | None:
    value: Any = row
    for part in field.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(part)
    if value in ("", None):
        return None
    return float(value)


def _config_with_algorithm(default_config: Path, version: str, output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with default_config.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    config.setdefault("algorithm", {})["version"] = version
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=True)
    return output_path


def _scenario_path(item: str, scenario_dir: Path) -> Path:
    path = Path(item)
    if path.suffix:
        return path
    return scenario_dir / f"{item}.yaml"


def _write_json(path: Path, payload: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def _write_markdown(path: Path, comparison: dict[str, Any]) -> None:
    lines = [
        "# Algorithm Comparison",
        "",
        f"- Baseline: `{comparison['baseline_version']}`",
        f"- Candidate: `{comparison['candidate_version']}`",
        "",
        "| Scenario | Coverage delta | Time95 delta % | Distance delta % | Redundant delta % | Workload delta | Unique segments delta | No-fly delta |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for scenario, deltas in comparison["scenarios"].items():
        lines.append(
            f"| {scenario} | {deltas['final_coverage_delta']:.6f} | "
            f"{deltas['time_to_95_delta_pct']:.6f} | "
            f"{deltas['total_distance_delta_pct']:.6f} | "
            f"{deltas['redundant_coverage_delta_pct']:.6f} | "
            f"{deltas['workload_balance_delta']:.6f} | "
            f"{deltas['unique_segment_count_delta']:.6f} | "
            f"{deltas['no_fly_violations_delta']:.6f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare UAV search algorithm versions across scenarios.")
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--scenarios", nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--scenario-dir", type=Path, default=Path("config/scenarios"))
    args = parser.parse_args()
    result = compare_algorithms(
        baseline_version=args.baseline,
        candidate_version=args.candidate,
        scenarios=args.scenarios,
        output_dir=args.output,
        default_config=args.config,
        scenario_dir=args.scenario_dir,
    )
    print(f"compared {len(result['scenarios'])} scenarios -> {args.output}")


if __name__ == "__main__":
    main()
