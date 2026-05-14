"""Helpers for loading canonical terrain metadata."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Tuple

from terrain_mapping_system.mission.planner import RectBounds


def load_manifest(manifest_path: str) -> Dict[str, Any]:
    path = Path(manifest_path)
    if not path.is_file():
        raise FileNotFoundError(f'terrain manifest not found: {manifest_path}')
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def terrain_extents_from_manifest(manifest: Dict[str, Any]) -> Dict[str, float]:
    extents = manifest.get('terrain_extents')
    if isinstance(extents, dict):
        return {
            'x_min': float(extents['x_min']),
            'x_max': float(extents['x_max']),
            'y_min': float(extents['y_min']),
            'y_max': float(extents['y_max']),
            'z_min': float(extents.get('z_min', 0.0)),
            'z_max': float(extents.get('z_max', manifest.get('terrain_z_scale_m', 0.0))),
        }

    xy_extents = manifest.get('terrain_xy_extents_m')
    if isinstance(xy_extents, dict):
        x_size = float(xy_extents['x'])
        y_size = float(xy_extents['y'])
        z_scale = float(manifest.get('terrain_z_scale_m', 0.0))
        return {
            'x_min': -x_size / 2.0,
            'x_max': x_size / 2.0,
            'y_min': -y_size / 2.0,
            'y_max': y_size / 2.0,
            'z_min': 0.0,
            'z_max': z_scale,
        }

    raise KeyError("terrain manifest missing 'terrain_extents' or 'terrain_xy_extents_m'")


def grid_shape_from_manifest(manifest: Dict[str, Any]) -> Dict[str, int]:
    grid_shape = manifest.get('grid_shape_raw')
    if isinstance(grid_shape, dict):
        rows = int(grid_shape.get('rows', 0))
        cols = int(grid_shape.get('cols', 0))
        if rows > 0 and cols > 0:
            return {'rows': rows, 'cols': cols}

    generator_params = manifest.get('generator_params')
    if isinstance(generator_params, dict):
        size = int(generator_params.get('size', 0))
        if size > 0:
            return {'rows': size, 'cols': size}

    return {}


def world_name_from_manifest(manifest: Dict[str, Any]) -> Any:
    world_name = manifest.get('world_name')
    if world_name:
        return world_name
    generator_params = manifest.get('generator_params')
    if isinstance(generator_params, dict):
        return generator_params.get('world')
    return None


def canonical_file_paths_from_manifest(manifest: Dict[str, Any]) -> Dict[str, Any]:
    canonical = manifest.get('canonical_file_paths', {})
    if not isinstance(canonical, dict):
        return {}

    normalized = dict(canonical)
    if 'stl_mesh' not in normalized:
        stl_candidate = (
            normalized.get('collision_stl')
            or normalized.get('collision_stl_mesh')
            or normalized.get('visual_stl_mesh')
        )
        if stl_candidate:
            normalized['stl_mesh'] = stl_candidate
    return normalized


def bounds_from_manifest(manifest: Dict[str, Any]) -> Tuple[RectBounds, Dict[str, Any]]:
    extents = terrain_extents_from_manifest(manifest)

    bounds = RectBounds(
        x_min=float(extents['x_min']),
        x_max=float(extents['x_max']),
        y_min=float(extents['y_min']),
        y_max=float(extents['y_max']),
    )
    metadata = {
        'world_name': world_name_from_manifest(manifest),
        'generated_at': manifest.get('generated_at'),
        'seed': manifest.get('seed'),
        'vehicle_spawn': manifest.get('vehicle_spawn'),
        'terrain_extents': extents,
        'grid_shape_raw': grid_shape_from_manifest(manifest),
        'canonical_file_paths': canonical_file_paths_from_manifest(manifest),
    }
    return bounds, metadata
