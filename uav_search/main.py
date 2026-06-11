from __future__ import annotations

import argparse
from pathlib import Path

from uav_search.core.config import load_config, validate_config
from uav_search.core.data_types import DecisionOutput
from uav_search.core.scheduler import Scheduler
from uav_search.maps.map_loader import build_grid_map
from uav_search.simulation.simulator import Simulator
from uav_search.uav.fleet_manager import FleetManager


def run(default_config: Path, scenario_path: Path, output_path: Path) -> DecisionOutput:
    config = load_config(default_config, scenario_path)
    validate_config(config)
    scenario = config.get("scenario", {})

    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, scenario)
    scheduler = Scheduler(grid_map, fleet, config)
    decision_output = scheduler.regular_cycle(now=0.0)

    simulator = Simulator(grid_map, fleet, config)
    simulator.record_snapshot()
    simulator.run()
    simulator.save_snapshots(output_path, run_id=scenario.get("name", "manual_run"))

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
    args = parser.parse_args()

    output = run(args.config, args.scenario, args.output)
    print(
        f"finished timestamp={output.timestamp:.1f}s "
        f"coverage={output.global_coverage:.3f} "
        f"priority_coverage={output.priority_coverage:.3f} "
        f"decision_latency_ms={output.decision_latency_ms:.2f}"
    )


if __name__ == "__main__":
    main()
