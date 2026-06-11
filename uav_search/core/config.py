from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a mapping: {path}")
    return data


def load_config(default_path: str | Path, scenario_path: str | Path | None = None) -> dict[str, Any]:
    config = load_yaml(default_path)
    if scenario_path is None:
        return config

    scenario = load_yaml(scenario_path)
    config = deep_merge(config, scenario.get("overrides", {}))
    config["scenario"] = scenario
    return config


def validate_config(config: dict[str, Any]) -> None:
    map_config = config["map"]
    if map_config["width_m"] <= 0 or map_config["height_m"] <= 0:
        raise ValueError("map width_m and height_m must be greater than 0")
    if map_config["resolution_m"] <= 0:
        raise ValueError("map resolution_m must be greater than 0")

    uav_config = config["uav"]
    if not 0.0 <= uav_config["battery_threshold"] <= 1.0:
        raise ValueError("uav battery_threshold must be within [0.0, 1.0]")
    if uav_config["count"] <= 0:
        raise ValueError("uav count must be greater than 0")
    if uav_config["max_speed_mps"] <= 0:
        raise ValueError("uav max_speed_mps must be greater than 0")

    simulation_config = config["simulation"]
    if simulation_config["time_step_s"] <= 0:
        raise ValueError("simulation time_step_s must be greater than 0")
