from __future__ import annotations

import argparse
from pathlib import Path

from uav_search.core.config import load_config, validate_config
from uav_search.core.data_types import DecisionOutput
from uav_search.core.scheduler import Scheduler
from uav_search.evaluation.metrics import compute_metrics, save_metrics
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.scenario_events import ScenarioEventInjector
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager
from uav_search.visualization.report_generator import generate_report_charts
from uav_search.visualization.static_viewer import render_static_map


def run(
    default_config: Path,
    scenario_path: Path,
    output_path: Path,
    image_path: Path | None = None,
    metrics_path: Path | None = None,
    report_dir: Path | None = None,
    play: bool = False,
    play_interval_ms: int = 160,
    play_repeat: bool = False,
) -> DecisionOutput:
    config = load_config(default_config, scenario_path)
    validate_config(config)
    scenario = config.get("scenario", {})

    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, scenario)
    scheduler = Scheduler(grid_map, fleet, config)
    decision_output = scheduler.regular_cycle(now=0.0)

    simulator = Simulator(grid_map, fleet, config)
    simulator.record_snapshot()
    event_injector = ScenarioEventInjector(scenario.get("events", []))
    simulator.run(scheduler=scheduler, event_injector=event_injector)
    simulator.save_snapshots(output_path, run_id=scenario.get("name", "manual_run"))
    if metrics_path is not None:
        metrics = compute_metrics(scenario.get("name", "manual_run"), grid_map, fleet, simulator.snapshots)
        save_metrics(metrics, metrics_path)
    if image_path is not None:
        render_static_map(
            grid_map,
            fleet.get_all_states(),
            image_path,
            title=f"{scenario.get('name', 'UAV Search')} final state",
        )
    if report_dir is not None:
        generate_report_charts(simulator.snapshots, report_dir)
    if play:
        from uav_search.visualization.realtime_viewer import play_snapshots

        play_snapshots(
            grid_map,
            simulator.snapshots,
            sensor_radius_cells=int(config["uav"]["sensor_radius_cells"]),
            interval_ms=play_interval_ms,
            repeat=play_repeat,
        )

    return DecisionOutput(
        timestamp=simulator.time_s,
        commands=decision_output.commands,
        assignments=decision_output.assignments,
        events_handled=decision_output.events_handled,
        global_coverage=grid_map.coverage_rate(),
        priority_coverage=grid_map.coverage_rate(priority_only=True),
        decision_latency_ms=decision_output.decision_latency_ms,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the first-loop UAV search simulation.")
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--scenario", type=Path, default=Path("config/scenarios/basic.yaml"))
    parser.add_argument("--output", type=Path, default=Path("runs/basic_snapshots.json"))
    parser.add_argument("--image", type=Path, default=None, help="Optional PNG path for a static visualization.")
    parser.add_argument("--metrics", type=Path, default=None, help="Optional JSON path for evaluation metrics.")
    parser.add_argument("--report-dir", type=Path, default=None, help="Optional directory for report charts.")
    parser.add_argument("--play", action="store_true", help="Open a realtime matplotlib playback window after running.")
    parser.add_argument("--play-interval-ms", type=int, default=160, help="Playback frame interval in milliseconds.")
    parser.add_argument("--play-repeat", action="store_true", help="Loop playback until the window is closed.")
    args = parser.parse_args()

    output = run(
        args.config,
        args.scenario,
        args.output,
        args.image,
        args.metrics,
        args.report_dir,
        args.play,
        args.play_interval_ms,
        args.play_repeat,
    )
    print(
        f"finished timestamp={output.timestamp:.1f}s "
        f"coverage={output.global_coverage:.3f} "
        f"priority_coverage={output.priority_coverage:.3f} "
        f"decision_latency_ms={output.decision_latency_ms:.2f}"
    )


if __name__ == "__main__":
    main()
