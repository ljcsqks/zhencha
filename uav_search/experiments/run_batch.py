from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from uav_search.evaluation.metrics import MetricsResult
from uav_search.main import run


SUMMARY_FIELDS = [
    "run_id",
    "final_time_s",
    "global_coverage",
    "priority_coverage",
    "redundant_coverage_rate",
    "total_distance_m",
    "effective_search_distance_m",
    "path_efficiency",
    "min_battery",
    "event_count",
    "conflict_count",
    "no_fly_violations",
    "map_update_count",
    "target_found_count",
    "confirm_done_count",
    "time_to_95_coverage_s",
    "time_to_priority_coverage_s",
    "coverage_goal_met",
    "priority_goal_met",
    "turn_rate",
    "replan_count",
    "coverage_gain_per_meter",
    "post_95_extra_distance_m",
    "per_uav_workload_balance",
    "supplemental_task_count",
    "ignored_uncovered_cells",
    "final_uncovered_cells",
    "final_priority_uncovered_cells",
    "post_95_extra_time_s",
]


def run_batch(default_config: Path, scenario_paths: list[Path], output_dir: Path) -> list[dict[str, Any]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []

    for scenario_path in scenario_paths:
        scenario_name = scenario_path.stem
        scenario_output_dir = output_dir / scenario_name
        scenario_output_dir.mkdir(parents=True, exist_ok=True)

        snapshots_path = scenario_output_dir / "snapshots.json"
        image_path = scenario_output_dir / "final_view.png"
        metrics_path = scenario_output_dir / "metrics.json"
        report_dir = scenario_output_dir / "report"

        run(
            default_config=default_config,
            scenario_path=scenario_path,
            output_path=snapshots_path,
            image_path=image_path,
            metrics_path=metrics_path,
            report_dir=report_dir,
        )
        rows.append(_load_summary_row(metrics_path, scenario_path))

    _write_summary_json(rows, output_dir / "summary.json")
    _write_summary_csv(rows, output_dir / "summary.csv")
    return rows


def _load_summary_row(metrics_path: Path, scenario_path: Path) -> dict[str, Any]:
    with metrics_path.open("r", encoding="utf-8") as handle:
        metrics = json.load(handle)
    metrics["scenario_file"] = str(scenario_path)
    return metrics


def _write_summary_json(rows: list[dict[str, Any]], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(rows, handle, ensure_ascii=False, indent=2)


def _write_summary_csv(rows: list[dict[str, Any]], output_path: Path) -> None:
    fieldnames = ["scenario_file", *SUMMARY_FIELDS]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _scenario_paths(names_or_paths: list[str], scenario_dir: Path) -> list[Path]:
    paths: list[Path] = []
    for item in names_or_paths:
        path = Path(item)
        if path.suffix:
            paths.append(path)
        else:
            paths.append(scenario_dir / f"{item}.yaml")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Run multiple UAV search scenarios and summarize metrics.")
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--scenario-dir", type=Path, default=Path("config/scenarios"))
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=[
            "area_search_1uav",
            "area_search_2uav",
            "area_search_3uav",
            "area_search_4uav",
            "area_search_5uav",
        ],
    )
    parser.add_argument("--output-dir", type=Path, default=Path("runs/batch"))
    args = parser.parse_args()

    rows = run_batch(args.config, _scenario_paths(args.scenarios, args.scenario_dir), args.output_dir)
    print(f"finished {len(rows)} scenarios -> {args.output_dir}")


if __name__ == "__main__":
    main()
