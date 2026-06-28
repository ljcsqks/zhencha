from __future__ import annotations

from typing import Any


SUPPORTED_ALGORITHMS: tuple[dict[str, str], ...] = (
    {
        "version": "baseline_sparse_boustrophedon",
        "label": "Baseline Sparse Boustrophedon",
        "description": "Stable internal comparison baseline with sparse boustrophedon coverage.",
    },
    {
        "version": "segment_sweep_v1",
        "label": "Segment Sweep v1",
        "description": "Scanline segment planner for obstacle-heavy scenes; useful for algorithm diagnostics.",
    },
    {
        "version": "adaptive_component_sweep_v1",
        "label": "Adaptive Component Sweep v1",
        "description": "Current default planner: baseline-style simple regions, clustered sweeps for complex regions.",
    },
)

DEFAULT_ALGORITHM_VERSION = "adaptive_component_sweep_v1"
SUPPORTED_ALGORITHM_VERSIONS = {item["version"] for item in SUPPORTED_ALGORITHMS}


def algorithms_payload() -> dict[str, Any]:
    return {
        "algorithms": [dict(item) for item in SUPPORTED_ALGORITHMS],
        "default_version": DEFAULT_ALGORITHM_VERSION,
    }


def validate_algorithm_version(version: str | None) -> str | None:
    if version is None or version == "":
        return None
    if version not in SUPPORTED_ALGORITHM_VERSIONS:
        supported = ", ".join(sorted(SUPPORTED_ALGORITHM_VERSIONS))
        raise ValueError(f"unknown algorithm_version: {version}. supported: {supported}")
    return version
