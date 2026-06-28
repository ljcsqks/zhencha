from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator


class ResetRequest(BaseModel):
    config_path: str = "config/default.yaml"
    scenario_path: str = "config/scenarios/area_search_1uav.yaml"
    algorithm_version: str | None = None


class StepRequest(BaseModel):
    steps: int = Field(default=1, ge=1, le=100)


class StartRequest(BaseModel):
    tick_interval_ms: int = Field(default=100, ge=10, le=10000)


class EventRequest(BaseModel):
    type: Literal["TARGET_FOUND", "MAP_UPDATE", "UAV_OFFLINE", "UAV_RECOVERED"]
    time_s: float | None = None
    source_uav_id: str | None = None
    data: dict[str, Any] = Field(default_factory=dict)

    @field_validator("data")
    @classmethod
    def validate_event_data(cls, value: dict[str, Any], info) -> dict[str, Any]:
        event_type = info.data.get("type")
        if event_type == "TARGET_FOUND":
            required = {"target_id", "position", "confidence", "target_type"}
            missing = required.difference(value)
            if missing:
                raise ValueError(f"TARGET_FOUND missing fields: {sorted(missing)}")
        if event_type == "MAP_UPDATE" and "operation" not in value:
            raise ValueError("MAP_UPDATE requires operation")
        return value
