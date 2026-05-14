"""Shared path helpers for runtime artifacts."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def discover_project_root() -> Optional[Path]:
    """Best-effort discovery of the simulator project root."""

    env_root = os.environ.get('TERRAIN_MAPPING_PROJECT_ROOT', '').strip()
    if env_root:
        candidate = Path(env_root).expanduser().resolve()
        if _looks_like_project_root(candidate):
            return candidate

    current = Path(__file__).resolve()
    for candidate in current.parents:
        if _looks_like_project_root(candidate):
            return candidate
    return None


def default_results_root() -> str:
    """Return the preferred root for run artifacts."""

    project_root = discover_project_root()
    if project_root is None:
        return '/tmp/terrain_mapping_results'
    return str(project_root / 'artifacts' / 'terrain_mapping_results')


def _looks_like_project_root(path: Path) -> bool:
    return (
        path.name == 'gazebo_harmonic_ardupilot'
        and (path / 'models').is_dir()
        and (path / 'worlds').is_dir()
        and (path / 'ros2_ws').is_dir()
    )
