from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from uav_search.core.config import load_config, validate_config
from uav_search.core.data_types import CommandType, DecisionCommand, DecisionOutput, Position, UAVStatus
from uav_search.allocation.auction import SequentialAuction
from uav_search.maps.map_loader import build_grid_map
from uav_search.planning.path_planner import PathPlanner
from uav_search.simulation.simulator import Simulator
from uav_search.task.task_generator import generate_initial_tasks
from uav_search.task.task_manager import TaskManager
from uav_search.uav.fleet_manager import FleetManager


def run(default_config: Path, scenario_path: Path, output_path: Path) -> DecisionOutput:
    config = load_config(default_config, scenario_path)
    validate_config(config)
    scenario = config.get("scenario", {})

    grid_map = build_grid_map(config)
    fleet = FleetManager.from_config(config, scenario)
    planner = PathPlanner(config.get("planning", {}))
    auction = SequentialAuction({**config, "battery_threshold": config["uav"]["battery_threshold"]})

    states = fleet.get_all_states()
    first_uav = states[0]
    tasks = generate_initial_tasks(
        grid_map=grid_map,
        uav_count=int(config["uav"]["count"]),
        sensor_radius_cells=int(config["uav"]["sensor_radius_cells"]),
        home=first_uav.home_position,
        created_at=0.0,
    )
    task_manager = TaskManager(tasks)
    started = time.perf_counter()
    proposed_assignments = auction.allocate(task_manager.get_pending_tasks(), fleet.get_available_uavs(), grid_map)
    commands: list[DecisionCommand] = []

    assignments = []
    for proposed in proposed_assignments:
        task = task_manager.tasks[proposed.task_id]
        assignment = task_manager.assign_task(task.id, proposed.uav_id, now=0.0, bid_value=proposed.bid_value)
        uav_state = fleet.get_uav(proposed.uav_id).state
        route = _plan_route_through_waypoints(uav_state, task.waypoints, grid_map, planner)
        if route:
            task_manager.start_task(task.id, now=0.0)
            uav_state.current_task_id = task.id
            fleet.assign_path(uav_state.id, route, status=UAVStatus.SEARCHING)
            commands.append(
                DecisionCommand(
                    uav_id=uav_state.id,
                    command=CommandType.FOLLOW_PATH,
                    task_id=task.id,
                    target=task.waypoints[-1],
                    path=route,
                    reason="auction_search_task",
                )
            )
            assignments.append(assignment)
            continue

        task_manager.mark_blocked(task.id, now=0.0)
        commands.append(
            DecisionCommand(
                uav_id=uav_state.id,
                command=CommandType.HOLD,
                task_id=task.id,
                target=task.entry_point,
                path=[],
                reason="task_route_not_found",
            )
        )

    latency_ms = (time.perf_counter() - started) * 1000.0

    simulator = Simulator(grid_map, fleet, config)
    simulator.record_snapshot()
    simulator.run()
    simulator.save_snapshots(output_path, run_id=scenario.get("name", "manual_run"))

    return DecisionOutput(
        timestamp=simulator.time_s,
        commands=commands,
        assignments=assignments,
        events_handled=[],
        global_coverage=grid_map.coverage_rate(),
        priority_coverage=grid_map.coverage_rate(priority_only=True),
        decision_latency_ms=latency_ms,
    )


def _plan_route_through_waypoints(
    uav_state,
    waypoints: list[Position],
    grid_map,
    planner: PathPlanner,
) -> list[Position]:
    route: list[Position] = []
    current = uav_state.position
    for waypoint in waypoints:
        if waypoint == current:
            continue
        segment_uav = replace(uav_state, position=current)
        plan = planner.plan_path(segment_uav, waypoint, grid_map)
        if not plan.valid:
            return []
        if not route:
            route.extend(plan.path)
        else:
            route.extend(plan.path[1:])
        current = waypoint
    return route


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
