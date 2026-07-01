from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


class GridPositionModel(BaseModel):
    x: int = Field(ge=0)
    y: int = Field(ge=0)


class DraftRectangle(BaseModel):
    id: str | None = None
    x: int = Field(ge=0)
    y: int = Field(ge=0)
    width: int = Field(ge=1)
    height: int = Field(ge=1)


class DraftPriorityRegion(DraftRectangle):
    priority: float = Field(default=3.0, gt=0)


class DraftMapConfig(BaseModel):
    width_cells: int | None = Field(default=None, ge=1)
    height_cells: int | None = Field(default=None, ge=1)
    width_m: float | None = Field(default=None, gt=0)
    height_m: float | None = Field(default=None, gt=0)
    resolution_m: float = Field(default=10.0, gt=0)


class DraftUav(BaseModel):
    id: str | None = None
    home_position: GridPositionModel
    initial_position: GridPositionModel | None = None
    sensor_radius_cells: int = Field(default=2, ge=1)
    speed_mps: float = Field(default=10.0, gt=0)
    battery: float = Field(default=1.0, ge=0.0, le=1.0)


class MissionDraft(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    draft_uavs: list[DraftUav] = Field(default_factory=list, alias="draftUavs")
    draft_obstacles: list[DraftRectangle] = Field(default_factory=list, alias="draftObstacles")
    draft_search_region: DraftRectangle | None = Field(default=None, alias="draftSearchRegion")
    draft_priority_regions: list[DraftPriorityRegion] = Field(default_factory=list, alias="draftPriorityRegions")
    draft_map_config: DraftMapConfig | None = Field(default=None, alias="draftMapConfig")


class ResetRequest(BaseModel):
    config_path: str = "config/default.yaml"
    scenario_path: str = "config/scenarios/area_search_1uav.yaml"
    algorithm_version: str | None = None


class ResetCustomRequest(ResetRequest):
    mission: MissionDraft = Field(default_factory=MissionDraft)


class StepRequest(BaseModel):
    steps: int = Field(default=1, ge=1, le=100)


class StartRequest(BaseModel):
    tick_interval_ms: int = Field(default=100, ge=10, le=10000)


class EventRequest(BaseModel):
    type: Literal["TARGET_FOUND", "MAP_UPDATE", "UAV_OFFLINE", "UAV_RECOVERED", "BUILDING_MODEL_REQUEST"]
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
        if event_type == "BUILDING_MODEL_REQUEST":
            required = {"building_id", "footprint"}
            missing = required.difference(value)
            if missing:
                raise ValueError(f"BUILDING_MODEL_REQUEST missing fields: {sorted(missing)}")
            footprint = value.get("footprint")
            if not isinstance(footprint, list) or len(footprint) < 4:
                raise ValueError("BUILDING_MODEL_REQUEST footprint must contain at least four points")
            behavior = value.get("post_modeling_behavior")
            if behavior is not None and behavior not in {"return_home_when_no_resume", "hold", "return_home", "resume_or_idle"}:
                raise ValueError("BUILDING_MODEL_REQUEST post_modeling_behavior is invalid")
        return value
