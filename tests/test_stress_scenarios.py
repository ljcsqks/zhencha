from __future__ import annotations

from pathlib import Path

from uav_search.core.config import load_config
from uav_search.server.runtime import SimulationRuntime


STRESS_SCENARIOS = {
    "stress_obstacle_maze_3uav",
    "stress_fragmented_area_4uav",
    "stress_5uav_balance",
    "stress_target_confirm_mid_search",
    "stress_dynamic_obstacle_mid_route",
}


def test_stress_scenarios_exist_and_load() -> None:
    for scenario_name in STRESS_SCENARIOS:
        path = Path("config/scenarios") / f"{scenario_name}.yaml"
        assert path.exists()
        config = load_config("config/default.yaml", path)
        assert config["scenario"]["name"] == scenario_name


def test_runtime_lists_stress_scenarios() -> None:
    runtime = SimulationRuntime()
    names = {item["name"] for item in runtime.get_scenarios()}

    assert STRESS_SCENARIOS.issubset(names)
