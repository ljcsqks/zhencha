from __future__ import annotations

from typing import Any


SUPPORTED_ALGORITHMS: tuple[dict[str, str], ...] = (
    {
        "version": "baseline_sparse_boustrophedon",
        "label": "Baseline Sparse Boustrophedon",
        "description": "稳定基线：普通区域连续割草，当前默认算法。",
    },
    {
        "version": "segment_sweep_v1",
        "label": "Segment Sweep v1",
        "description": "扫描线片段规划：复杂障碍场景有优势，但普通场景可能过度切分。",
    },
    {
        "version": "adaptive_component_sweep_v1",
        "label": "Adaptive Component Sweep v1",
        "description": "自适应组件规划：简单区域走 baseline，复杂区域走 cluster segment。",
    },
)

DEFAULT_ALGORITHM_VERSION = "baseline_sparse_boustrophedon"
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
