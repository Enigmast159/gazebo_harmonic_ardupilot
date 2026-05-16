"""Evaluate one completed terrain-mapping run against terrain ground truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .common import file_fingerprint, read_json, software_context, utc_now_iso, write_json, write_text


DEFAULT_OUTPUT_BASENAME = 'run_evaluation'
EVALUATION_SCHEMA_VERSION = 3
METRIC_CHOICES = ('mae_m', 'rmse_m', 'p95_abs_error_m', 'max_abs_error_m')
ABS_ERROR_REPORT_THRESHOLD_M = 0.2
ABS_ERROR_REPORT_THRESHOLD_EPSILON_M = 1e-6
DEFAULT_PLATFORM_EXCLUSION_SIZE_M = 4.0


@dataclass(frozen=True)
class RegularGrid:
    x_coordinates_m: np.ndarray
    y_coordinates_m: np.ndarray
    height_m: np.ndarray

    def bounds(self) -> Dict[str, float]:
        return {
            'x_min': float(self.x_coordinates_m[0]),
            'x_max': float(self.x_coordinates_m[-1]),
            'y_min': float(self.y_coordinates_m[0]),
            'y_max': float(self.y_coordinates_m[-1]),
        }

    def shape_dict(self) -> Dict[str, int]:
        return {
            'rows': int(self.height_m.shape[0]),
            'cols': int(self.height_m.shape[1]),
        }


@dataclass(frozen=True)
class GroundTruthSource:
    grid: RegularGrid
    source_kind: str
    data_path: Path
    manifest_path: Optional[Path]
    manifest: Dict[str, Any]


@dataclass(frozen=True)
class PredictedDEM:
    height_mean_m: np.ndarray
    coverage_mask: np.ndarray
    x_coordinates_m: np.ndarray
    y_coordinates_m: np.ndarray
    metadata: Dict[str, Any]
    metadata_path: Optional[Path]
    summary_path: Optional[Path]


def _as_path(value: Optional[str]) -> Optional[Path]:
    if value is None:
        return None
    stripped = str(value).strip()
    if not stripped:
        return None
    return Path(stripped).expanduser().resolve()


def _load_optional_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    return read_json(path)


def _load_jsonl_head(path: Path, *, limit: int = 5) -> List[Dict[str, Any]]:
    if not path.is_file():
        return []
    items: List[Dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for index, line in enumerate(stream):
            if index >= limit:
                break
            stripped = line.strip()
            if not stripped:
                continue
            items.append(json.loads(stripped))
    return items


def _load_predicted_dem(run_dir: Path) -> PredictedDEM:
    dem_npz_path = run_dir / 'dem_map.npz'
    if not dem_npz_path.is_file():
        raise FileNotFoundError(f'predicted DEM artifact not found: {dem_npz_path}')

    with np.load(dem_npz_path, allow_pickle=False) as payload:
        metadata_raw = payload['metadata_json']
        metadata_json = str(metadata_raw.item() if metadata_raw.shape == () else metadata_raw.tolist())
        metadata = json.loads(metadata_json)
        return PredictedDEM(
            height_mean_m=np.asarray(payload['height_mean_m'], dtype=np.float64),
            coverage_mask=np.asarray(payload['coverage_mask'], dtype=bool),
            x_coordinates_m=np.asarray(payload['x_coordinates_m'], dtype=np.float64),
            y_coordinates_m=np.asarray(payload['y_coordinates_m'], dtype=np.float64),
            metadata=metadata,
            metadata_path=dem_npz_path.parent / 'dem_metadata.json',
            summary_path=dem_npz_path.parent / 'dem_mapper_summary.json',
        )


def _load_truth_grid_from_csv(path: Path) -> RegularGrid:
    rows: List[int] = []
    cols: List[int] = []
    x_values: List[float] = []
    y_values: List[float] = []
    z_values: List[float] = []

    with path.open('r', encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'row', 'col', 'x_m', 'y_m', 'z_m'}
        if reader.fieldnames is None or not required.issubset(set(reader.fieldnames)):
            raise ValueError(f'ground-truth CSV has unexpected columns: {path}')
        for record in reader:
            rows.append(int(record['row']))
            cols.append(int(record['col']))
            x_values.append(float(record['x_m']))
            y_values.append(float(record['y_m']))
            z_values.append(float(record['z_m']))

    if not rows:
        raise ValueError(f'ground-truth CSV is empty: {path}')

    row_count = max(rows) + 1
    col_count = max(cols) + 1
    grid = np.full((row_count, col_count), np.nan, dtype=np.float64)
    x_coordinates = np.full(col_count, np.nan, dtype=np.float64)
    y_coordinates = np.full(row_count, np.nan, dtype=np.float64)

    for row, col, x_m, y_m, z_m in zip(rows, cols, x_values, y_values, z_values):
        grid[row, col] = z_m
        x_coordinates[col] = x_m
        y_coordinates[row] = y_m

    if np.isnan(grid).any():
        raise ValueError(f'ground-truth grid has missing cells: {path}')
    if np.isnan(x_coordinates).any() or np.isnan(y_coordinates).any():
        raise ValueError(f'ground-truth coordinates are incomplete: {path}')

    return RegularGrid(
        x_coordinates_m=x_coordinates,
        y_coordinates_m=y_coordinates,
        height_m=grid,
    )


def _load_truth_grid_from_stl(path: Path) -> RegularGrid:
    with path.open('rb') as stream:
        header = stream.read(84)
    if len(header) < 84:
        raise ValueError(f'STL file is too small: {path}')

    face_count = struct.unpack('<I', header[80:84])[0]
    expected_size = 84 + face_count * 50
    if path.stat().st_size != expected_size:
        raise ValueError(f'only binary STL exported by the terrain generator is supported: {path}')

    face_dtype = np.dtype(
        [
            ('normal', '<f4', (3,)),
            ('vertices', '<f4', (3, 3)),
            ('attribute', '<u2'),
        ]
    )
    faces = np.fromfile(path, dtype=face_dtype, offset=84, count=face_count)
    vertices = faces['vertices'].reshape(-1, 3).astype(np.float64, copy=False)

    z_by_xy: Dict[Tuple[float, float], List[float]] = {}
    for x_m, y_m, z_m in vertices:
        key = (round(float(x_m), 6), round(float(y_m), 6))
        z_by_xy.setdefault(key, []).append(float(z_m))

    if not z_by_xy:
        raise ValueError(f'no vertices parsed from STL: {path}')

    x_coordinates = np.asarray(sorted({key[0] for key in z_by_xy}), dtype=np.float64)
    y_coordinates = np.asarray(sorted({key[1] for key in z_by_xy}), dtype=np.float64)
    x_index = {value: index for index, value in enumerate(x_coordinates.tolist())}
    y_index = {value: index for index, value in enumerate(y_coordinates.tolist())}
    height_m = np.full((y_coordinates.size, x_coordinates.size), np.nan, dtype=np.float64)

    for (x_m, y_m), samples in z_by_xy.items():
        height_m[y_index[y_m], x_index[x_m]] = float(np.mean(samples))

    if np.isnan(height_m).any():
        raise ValueError(f'could not reconstruct a dense terrain grid from STL: {path}')

    return RegularGrid(
        x_coordinates_m=x_coordinates,
        y_coordinates_m=y_coordinates,
        height_m=height_m,
    )


def _resolve_ground_truth_source(run_dir: Path, predicted: PredictedDEM) -> GroundTruthSource:
    experiment_request = _load_optional_json(run_dir / 'experiment_request.json') or {}
    planned_path = _load_optional_json(run_dir / 'planned_path.json') or {}
    dem_metadata_json = _load_optional_json(run_dir / 'dem_metadata.json') or predicted.metadata

    local_manifest_path = run_dir / 'terrain_manifest.snapshot.json'
    local_truth_csv_path = run_dir / 'terrain_ground_truth.snapshot.csv'
    local_stl_path = run_dir / 'terrain_model.snapshot.stl'

    manifest_candidates: List[Path] = []
    ground_truth_candidates: List[Path] = []
    stl_candidates: List[Path] = []

    if local_manifest_path.is_file():
        manifest_candidates.append(local_manifest_path)
    if local_truth_csv_path.is_file():
        ground_truth_candidates.append(local_truth_csv_path)
    if local_stl_path.is_file():
        stl_candidates.append(local_stl_path)

    terrain_request = experiment_request.get('terrain', {}) if isinstance(experiment_request, dict) else {}
    for value in (
        terrain_request.get('manifest_snapshot'),
        terrain_request.get('manifest_path'),
        dem_metadata_json.get('manifest', {}).get('manifest_path'),
        planned_path.get('bounds_metadata', {}).get('manifest_path'),
    ):
        candidate = _as_path(value)
        if candidate is not None:
            manifest_candidates.append(candidate)

    manifest_payload: Dict[str, Any] = {}
    manifest_path: Optional[Path] = None
    for candidate in manifest_candidates:
        if candidate.is_file():
            manifest_payload = read_json(candidate)
            manifest_path = candidate
            break

    def extend_from_manifest(manifest: Dict[str, Any]) -> None:
        canonical = manifest.get('canonical_file_paths', {})
        if not isinstance(canonical, dict):
            return
        truth_candidate = _as_path(canonical.get('ground_truth_csv'))
        stl_candidate = _as_path(
            canonical.get('stl_mesh')
            or canonical.get('collision_stl')
            or canonical.get('collision_stl_mesh')
            or canonical.get('visual_stl_mesh')
        )
        if truth_candidate is not None:
            ground_truth_candidates.append(truth_candidate)
        if stl_candidate is not None:
            stl_candidates.append(stl_candidate)

    if manifest_payload:
        extend_from_manifest(manifest_payload)

    canonical_from_metadata = dem_metadata_json.get('manifest', {}).get('canonical_file_paths', {})
    if isinstance(canonical_from_metadata, dict):
        extend_from_manifest({'canonical_file_paths': canonical_from_metadata})
    canonical_from_plan = planned_path.get('bounds_metadata', {}).get('canonical_file_paths', {})
    if isinstance(canonical_from_plan, dict):
        extend_from_manifest({'canonical_file_paths': canonical_from_plan})

    for value in (
        terrain_request.get('ground_truth_snapshot'),
        terrain_request.get('ground_truth_csv'),
    ):
        candidate = _as_path(value)
        if candidate is not None:
            ground_truth_candidates.append(candidate)
    for value in (
        terrain_request.get('terrain_mesh_snapshot'),
        terrain_request.get('stl_mesh'),
    ):
        candidate = _as_path(value)
        if candidate is not None:
            stl_candidates.append(candidate)

    for candidate in ground_truth_candidates:
        if candidate.is_file():
            return GroundTruthSource(
                grid=_load_truth_grid_from_csv(candidate),
                source_kind='terrain_truth_csv',
                data_path=candidate,
                manifest_path=manifest_path,
                manifest=manifest_payload,
            )

    for candidate in stl_candidates:
        if candidate.is_file():
            return GroundTruthSource(
                grid=_load_truth_grid_from_stl(candidate),
                source_kind='active_terrain_geometry_stl',
                data_path=candidate,
                manifest_path=manifest_path,
                manifest=manifest_payload,
            )

    raise FileNotFoundError(
        'could not resolve terrain ground truth. Expected a snapshot CSV, canonical terrain-truth CSV, or active STL mesh.'
    )


def _is_regular_axis(values: np.ndarray, *, atol: float = 1e-6) -> bool:
    if values.size < 2:
        return True
    deltas = np.diff(values)
    return bool(np.allclose(deltas, deltas[0], atol=atol, rtol=0.0))


def _truth_on_predicted_grid(truth: RegularGrid, predicted: PredictedDEM) -> Tuple[np.ndarray, np.ndarray]:
    pred_x = predicted.x_coordinates_m
    pred_y = predicted.y_coordinates_m

    if (
        truth.x_coordinates_m.shape == pred_x.shape
        and truth.y_coordinates_m.shape == pred_y.shape
        and np.allclose(truth.x_coordinates_m, pred_x, atol=1e-6, rtol=0.0)
        and np.allclose(truth.y_coordinates_m, pred_y, atol=1e-6, rtol=0.0)
    ):
        return truth.height_m.astype(np.float64, copy=False), np.ones(predicted.height_mean_m.shape, dtype=bool)

    if not _is_regular_axis(truth.x_coordinates_m) or not _is_regular_axis(truth.y_coordinates_m):
        raise ValueError('ground-truth raster is not regular, so bilinear interpolation is undefined')

    x_mesh, y_mesh = np.meshgrid(pred_x, pred_y)
    comparison_domain = (
        (x_mesh >= truth.x_coordinates_m[0] - 1e-9)
        & (x_mesh <= truth.x_coordinates_m[-1] + 1e-9)
        & (y_mesh >= truth.y_coordinates_m[0] - 1e-9)
        & (y_mesh <= truth.y_coordinates_m[-1] + 1e-9)
    )

    truth_on_grid = np.full(predicted.height_mean_m.shape, np.nan, dtype=np.float64)
    if not np.any(comparison_domain):
        return truth_on_grid, comparison_domain

    dx = float(truth.x_coordinates_m[1] - truth.x_coordinates_m[0]) if truth.x_coordinates_m.size > 1 else 1.0
    dy = float(truth.y_coordinates_m[1] - truth.y_coordinates_m[0]) if truth.y_coordinates_m.size > 1 else 1.0

    u = np.clip((x_mesh - truth.x_coordinates_m[0]) / dx, 0.0, float(truth.x_coordinates_m.size - 1))
    v = np.clip((y_mesh - truth.y_coordinates_m[0]) / dy, 0.0, float(truth.y_coordinates_m.size - 1))

    x0 = np.floor(u).astype(np.int64, copy=False)
    y0 = np.floor(v).astype(np.int64, copy=False)
    x1 = np.clip(x0 + 1, 0, truth.x_coordinates_m.size - 1)
    y1 = np.clip(y0 + 1, 0, truth.y_coordinates_m.size - 1)
    tx = u - x0
    ty = v - y0

    z00 = truth.height_m[y0, x0]
    z10 = truth.height_m[y0, x1]
    z01 = truth.height_m[y1, x0]
    z11 = truth.height_m[y1, x1]
    z0 = z00 * (1.0 - tx) + z10 * tx
    z1 = z01 * (1.0 - tx) + z11 * tx
    truth_on_grid[comparison_domain] = (z0 * (1.0 - ty) + z1 * ty)[comparison_domain]
    return truth_on_grid, comparison_domain


def _platform_exclusion_config(manifest: Dict[str, Any]) -> Optional[Dict[str, float]]:
    if not isinstance(manifest, dict):
        return None
    platform_spawn = manifest.get('platform_spawn')
    if not isinstance(platform_spawn, dict):
        return None

    x_m = platform_spawn.get('x_m')
    y_m = platform_spawn.get('y_m')
    if x_m is None or y_m is None:
        return None

    size_x_m = DEFAULT_PLATFORM_EXCLUSION_SIZE_M
    size_y_m = DEFAULT_PLATFORM_EXCLUSION_SIZE_M
    platform_size = manifest.get('platform_size_m')
    if isinstance(platform_size, (int, float)):
        size_x_m = float(platform_size)
        size_y_m = float(platform_size)
    elif isinstance(platform_size, dict):
        size_x_m = float(platform_size.get('x_m', DEFAULT_PLATFORM_EXCLUSION_SIZE_M))
        size_y_m = float(platform_size.get('y_m', DEFAULT_PLATFORM_EXCLUSION_SIZE_M))

    return {
        'center_x_m': float(x_m),
        'center_y_m': float(y_m),
        'size_x_m': float(size_x_m),
        'size_y_m': float(size_y_m),
    }


def _predicted_map_axis_signs(predicted: PredictedDEM) -> Tuple[float, float, float]:
    raw = predicted.metadata.get('map_axis_signs_xyz')
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            pass
    return (1.0, 1.0, 1.0)


def _platform_exclusion_mask(
    predicted: PredictedDEM,
    manifest: Dict[str, Any],
) -> Tuple[np.ndarray, Optional[Dict[str, float]]]:
    config = _platform_exclusion_config(manifest)
    if config is None:
        return np.zeros(predicted.height_mean_m.shape, dtype=bool), None

    map_axis_signs = _predicted_map_axis_signs(predicted)
    center_x_m = config['center_x_m'] * map_axis_signs[0]
    center_y_m = config['center_y_m'] * map_axis_signs[1]
    x_mesh, y_mesh = np.meshgrid(predicted.x_coordinates_m, predicted.y_coordinates_m)
    half_size_x_m = 0.5 * config['size_x_m']
    half_size_y_m = 0.5 * config['size_y_m']
    mask = (
        (x_mesh >= center_x_m - half_size_x_m)
        & (x_mesh <= center_x_m + half_size_x_m)
        & (y_mesh >= center_y_m - half_size_y_m)
        & (y_mesh <= center_y_m + half_size_y_m)
    )
    config = dict(config)
    config['center_x_m'] = center_x_m
    config['center_y_m'] = center_y_m
    return mask, config


def _read_latency_values(path: Path) -> np.ndarray:
    if not path.is_file():
        return np.empty((0,), dtype=np.float64)
    values: List[float] = []
    with path.open('r', encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        for record in reader:
            raw = (record.get('processing_ms') or '').strip()
            if not raw:
                continue
            values.append(float(raw))
    return np.asarray(values, dtype=np.float64)


def _percentile_or_none(values: np.ndarray, percentile: float) -> Optional[float]:
    if values.size == 0:
        return None
    return float(np.percentile(values, percentile))


def _rounded(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), 6)


def _write_error_visualization(path: Path, abs_error_m: np.ndarray, evaluation_mask: np.ndarray, scale_max_m: float) -> None:
    output = np.zeros(abs_error_m.shape + (3,), dtype=np.uint8)
    valid = evaluation_mask & np.isfinite(abs_error_m)
    if np.any(valid):
        normalized = np.clip(abs_error_m / max(scale_max_m, 1e-9), 0.0, 1.0)
        red = np.clip((normalized - 0.5) * 2.0, 0.0, 1.0)
        blue = np.clip((0.5 - normalized) * 2.0, 0.0, 1.0)
        green = 1.0 - np.abs(normalized - 0.5) * 2.0
        output[..., 0][valid] = np.round(red[valid] * 255.0).astype(np.uint8)
        output[..., 1][valid] = np.round(green[valid] * 255.0).astype(np.uint8)
        output[..., 2][valid] = np.round(blue[valid] * 255.0).astype(np.uint8)

    flipped = np.flipud(output)
    with path.open('wb') as stream:
        header = f'P6\n{flipped.shape[1]} {flipped.shape[0]}\n255\n'.encode('ascii')
        stream.write(header)
        stream.write(flipped.tobytes())


def _copy_if_resolved(source_path: Optional[Path], destination: Path) -> Optional[Path]:
    if source_path is None or not source_path.is_file() or destination.is_file():
        return destination if destination.is_file() else None
    destination.write_bytes(source_path.read_bytes())
    return destination


def _snapshot_resolved_inputs(run_dir: Path, truth: GroundTruthSource) -> Dict[str, Optional[str]]:
    snapshot_manifest_path = run_dir / 'terrain_manifest.snapshot.json'
    snapshot_truth_path = run_dir / 'terrain_ground_truth.snapshot.csv'
    snapshot_stl_path = run_dir / 'terrain_model.snapshot.stl'

    if truth.source_kind == 'terrain_truth_csv':
        _copy_if_resolved(truth.data_path, snapshot_truth_path)
    elif truth.source_kind == 'active_terrain_geometry_stl':
        _copy_if_resolved(truth.data_path, snapshot_stl_path)

    if truth.manifest_path is not None and truth.manifest_path.is_file() and not snapshot_manifest_path.is_file():
        manifest_payload = dict(truth.manifest) if isinstance(truth.manifest, dict) else {}
        canonical = manifest_payload.get('canonical_file_paths')
        if isinstance(canonical, dict):
            canonical = dict(canonical)
            canonical['manifest'] = str(snapshot_manifest_path)
            if snapshot_truth_path.is_file():
                canonical['ground_truth_csv'] = str(snapshot_truth_path)
            if snapshot_stl_path.is_file():
                canonical['stl_mesh'] = str(snapshot_stl_path)
            manifest_payload['canonical_file_paths'] = canonical
        write_json(snapshot_manifest_path, manifest_payload)

    return {
        'manifest_snapshot': str(snapshot_manifest_path) if snapshot_manifest_path.is_file() else None,
        'ground_truth_snapshot': str(snapshot_truth_path) if snapshot_truth_path.is_file() else None,
        'terrain_mesh_snapshot': str(snapshot_stl_path) if snapshot_stl_path.is_file() else None,
    }


def _build_markdown_summary(summary: Dict[str, Any]) -> str:
    metrics = summary['metrics']
    acceptance = summary['acceptance']
    reproducibility = summary['reproducibility']
    lines = [
        f'# Run Evaluation: {summary["run_id"]}',
        '',
        '## Verdict',
        '',
        f'- Overall: {"PASS" if acceptance["overall_pass"] else "FAIL"}',
        f'- Vertical accuracy: {"PASS" if acceptance["vertical_accuracy_pass"] else "FAIL"}',
        f'- Processing time: {"PASS" if acceptance["processing_time_pass"] else "FAIL"}',
        '',
        '## Vertical Metrics',
        '',
        f'- Mean signed error: {metrics["vertical"]["mean_signed_error_m"]}',
        f'- MAE: {metrics["vertical"]["mae_m"]}',
        f'- Median absolute error: {metrics["vertical"]["median_abs_error_m"]}',
        f'- RMSE: {metrics["vertical"]["rmse_m"]}',
        f'- p95 absolute error: {metrics["vertical"]["p95_abs_error_m"]}',
        f'- Max absolute error: {metrics["vertical"]["max_abs_error_m"]}',
        (
        f'- Fraction within {metrics["vertical"]["abs_error_within_threshold_m"]} m: '
        f'{metrics["vertical"]["abs_error_within_threshold_fraction"]}'
        ),
        f'- Observed-area fraction: {metrics["vertical"]["observed_area_fraction"]}',
        f'- Compared cells: {metrics["vertical"]["observed_cells"]} / {metrics["vertical"]["comparison_domain_cells"]}',
        f'- Excluded platform cells: {metrics["vertical"]["excluded_platform_cells"]}',
        '',
        '## Latency Metrics',
        '',
        f'- Frames: {metrics["latency"]["frame_count"]}',
        f'- Mean processing time (ms): {metrics["latency"]["mean_processing_ms"]}',
        f'- p95 processing time (ms): {metrics["latency"]["p95_processing_ms"]}',
        f'- Max processing time (ms): {metrics["latency"]["max_processing_ms"]}',
        '',
        '## Acceptance Criteria',
        '',
        f'- Vertical primary metric: {acceptance["criteria"]["vertical_accuracy"]["primary_metric"]}',
        f'- Vertical threshold (m): {acceptance["criteria"]["vertical_accuracy"]["threshold_m"]}',
        f'- Latency threshold (ms): {acceptance["criteria"]["processing_time"]["per_frame_threshold_ms"]}',
        f'- Coverage gate enabled: {acceptance["criteria"]["observed_area"]["enabled"]}',
    ]
    if acceptance['criteria']['observed_area']['enabled']:
        lines.append(
            f'- Minimum observed-area fraction: {acceptance["criteria"]["observed_area"]["minimum_fraction"]}'
        )
    lines.extend(
        [
            '',
            '## Provenance',
            '',
            f'- Ground truth source: {summary["ground_truth"]["source_kind"]}',
            f'- Ground truth path: {summary["ground_truth"]["data_path"]}',
            f'- Terrain seed: {reproducibility["terrain"].get("seed")}',
            f'- Manifest path: {reproducibility["terrain"].get("manifest_path")}',
            f'- Mission summary path: {reproducibility["mission"].get("mission_summary_path")}',
            f'- DEM metadata path: {reproducibility["mapping"].get("dem_metadata_path")}',
        ]
    )
    return '\n'.join(lines) + '\n'


def evaluate_run_directory(
    run_dir: Path,
    *,
    vertical_threshold_m: float = 1.0,
    latency_threshold_ms: float = 100.0,
    primary_vertical_metric: str = 'rmse_m',
    min_observed_area_fraction: Optional[float] = None,
    output_basename: str = DEFAULT_OUTPUT_BASENAME,
    snapshot_inputs: bool = True,
) -> Dict[str, Any]:
    if primary_vertical_metric not in METRIC_CHOICES:
        raise ValueError(f'unsupported primary_vertical_metric: {primary_vertical_metric}')

    run_dir = run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(f'run directory not found: {run_dir}')

    predicted = _load_predicted_dem(run_dir)
    truth = _resolve_ground_truth_source(run_dir, predicted)
    if snapshot_inputs:
        _snapshot_resolved_inputs(run_dir, truth)

    truth_on_grid, comparison_domain = _truth_on_predicted_grid(truth.grid, predicted)
    platform_exclusion_mask, platform_exclusion = _platform_exclusion_mask(predicted, truth.manifest)
    coverage_mask = predicted.coverage_mask.astype(bool, copy=False)
    comparison_domain_mask = comparison_domain & np.isfinite(truth_on_grid) & ~platform_exclusion_mask
    evaluation_mask = comparison_domain_mask & coverage_mask & np.isfinite(predicted.height_mean_m)

    signed_error_m = np.full(predicted.height_mean_m.shape, np.nan, dtype=np.float64)
    abs_error_m = np.full(predicted.height_mean_m.shape, np.nan, dtype=np.float64)
    signed_error_m[evaluation_mask] = predicted.height_mean_m[evaluation_mask] - truth_on_grid[evaluation_mask]
    abs_error_m[evaluation_mask] = np.abs(signed_error_m[evaluation_mask])

    comparison_domain_cells = int(comparison_domain_mask.sum())
    excluded_platform_cells = int((comparison_domain & platform_exclusion_mask).sum())
    observed_cells = int(evaluation_mask.sum())
    observed_area_fraction = (
        float(observed_cells) / float(comparison_domain_cells) if comparison_domain_cells > 0 else 0.0
    )

    evaluated_errors = abs_error_m[evaluation_mask]
    evaluated_signed_errors = signed_error_m[evaluation_mask]
    mean_signed_error_m = float(np.mean(evaluated_signed_errors)) if evaluated_signed_errors.size else None
    mae_m = float(np.mean(evaluated_errors)) if evaluated_errors.size else None
    median_abs_error_m = _percentile_or_none(evaluated_errors, 50.0)
    rmse_m = float(np.sqrt(np.mean(np.square(evaluated_errors)))) if evaluated_errors.size else None
    p95_abs_error_m = _percentile_or_none(evaluated_errors, 95.0)
    max_abs_error_m = float(np.max(evaluated_errors)) if evaluated_errors.size else None
    within_threshold_mask = evaluated_errors <= (
        ABS_ERROR_REPORT_THRESHOLD_M + ABS_ERROR_REPORT_THRESHOLD_EPSILON_M
    )
    abs_error_within_threshold_fraction = float(np.mean(within_threshold_mask)) if evaluated_errors.size else None
    abs_error_within_threshold_cells = (
        int(np.count_nonzero(within_threshold_mask))
        if evaluated_errors.size
        else 0
    )

    latency_csv_path = run_dir / 'dem_latency.csv'
    latencies_ms = _read_latency_values(latency_csv_path)
    latency_mean_ms = float(latencies_ms.mean()) if latencies_ms.size else None
    latency_p95_ms = _percentile_or_none(latencies_ms, 95.0)
    latency_max_ms = float(latencies_ms.max()) if latencies_ms.size else None

    primary_value = {
        'mae_m': mae_m,
        'rmse_m': rmse_m,
        'p95_abs_error_m': p95_abs_error_m,
        'max_abs_error_m': max_abs_error_m,
    }[primary_vertical_metric]

    observed_area_gate_enabled = min_observed_area_fraction is not None
    observed_area_pass = (
        observed_area_fraction >= float(min_observed_area_fraction)
        if observed_area_gate_enabled
        else True
    )
    vertical_accuracy_pass = (
        primary_value is not None
        and observed_cells > 0
        and observed_area_pass
        and primary_value <= vertical_threshold_m
    )
    processing_time_pass = (
        latency_p95_ms is not None
        and latency_max_ms is not None
        and latency_p95_ms <= latency_threshold_ms
        and latency_max_ms <= latency_threshold_ms
    )

    mission_summary = _load_optional_json(run_dir / 'mission_summary.json') or {}
    planned_path = _load_optional_json(run_dir / 'planned_path.json') or {}
    dem_metadata = _load_optional_json(run_dir / 'dem_metadata.json') or predicted.metadata
    dem_summary = _load_optional_json(run_dir / 'dem_mapper_summary.json') or {}
    experiment_request = _load_optional_json(run_dir / 'experiment_request.json') or {}

    error_map_path = run_dir / f'{output_basename}_error_map.npz'
    heatmap_path = run_dir / f'{output_basename}_error_heatmap.ppm'
    summary_json_path = run_dir / f'{output_basename}.json'
    summary_md_path = run_dir / f'{output_basename}.md'

    heatmap_scale_max_m = max(
        vertical_threshold_m,
        p95_abs_error_m or 0.0,
        max_abs_error_m or 0.0,
        1e-6,
    )
    np.savez_compressed(
        error_map_path,
        predicted_height_m=predicted.height_mean_m.astype(np.float32),
        truth_height_m=truth_on_grid.astype(np.float32),
        signed_error_m=signed_error_m.astype(np.float32),
        absolute_error_m=abs_error_m.astype(np.float32),
        comparison_domain_mask=comparison_domain_mask,
        evaluation_mask=evaluation_mask,
        platform_exclusion_mask=platform_exclusion_mask,
        x_coordinates_m=predicted.x_coordinates_m.astype(np.float32),
        y_coordinates_m=predicted.y_coordinates_m.astype(np.float32),
    )
    _write_error_visualization(heatmap_path, abs_error_m, evaluation_mask, heatmap_scale_max_m)

    summary: Dict[str, Any] = {
        'schema_version': EVALUATION_SCHEMA_VERSION,
        'generated_at': utc_now_iso(),
        'run_id': mission_summary.get('run_id') or dem_summary.get('run_id') or predicted.metadata.get('run_id') or run_dir.name,
        'run_dir': str(run_dir),
        'ground_truth': {
            'source_kind': truth.source_kind,
            'data_path': str(truth.data_path),
            'manifest_path': str(truth.manifest_path) if truth.manifest_path is not None else None,
            'grid_shape': truth.grid.shape_dict(),
            'grid_bounds': truth.grid.bounds(),
            'manifest_generated_at': truth.manifest.get('generated_at') if isinstance(truth.manifest, dict) else None,
            'seed': truth.manifest.get('seed') if isinstance(truth.manifest, dict) else None,
            'platform_exclusion': {
                'enabled': bool(platform_exclusion is not None),
                'center_x_m': _rounded(platform_exclusion.get('center_x_m')) if platform_exclusion else None,
                'center_y_m': _rounded(platform_exclusion.get('center_y_m')) if platform_exclusion else None,
                'size_x_m': _rounded(platform_exclusion.get('size_x_m')) if platform_exclusion else None,
                'size_y_m': _rounded(platform_exclusion.get('size_y_m')) if platform_exclusion else None,
                'excluded_cells': excluded_platform_cells,
            },
        },
        'metrics': {
            'vertical': {
                'mean_signed_error_m': _rounded(mean_signed_error_m),
                'mae_m': _rounded(mae_m),
                'median_abs_error_m': _rounded(median_abs_error_m),
                'rmse_m': _rounded(rmse_m),
                'p95_abs_error_m': _rounded(p95_abs_error_m),
                'max_abs_error_m': _rounded(max_abs_error_m),
                'abs_error_within_threshold_m': _rounded(ABS_ERROR_REPORT_THRESHOLD_M),
                'abs_error_within_threshold_fraction': _rounded(abs_error_within_threshold_fraction),
                'abs_error_within_threshold_cells': abs_error_within_threshold_cells,
                'observed_area_fraction': _rounded(observed_area_fraction),
                'comparison_domain_cells': comparison_domain_cells,
                'observed_cells': observed_cells,
                'excluded_platform_cells': excluded_platform_cells,
            },
            'latency': {
                'frame_count': int(latencies_ms.size),
                'mean_processing_ms': _rounded(latency_mean_ms),
                'p95_processing_ms': _rounded(latency_p95_ms),
                'max_processing_ms': _rounded(latency_max_ms),
            },
        },
        'acceptance': {
            'criteria': {
                'vertical_accuracy': {
                    'primary_metric': primary_vertical_metric,
                    'threshold_m': _rounded(vertical_threshold_m),
                },
                'processing_time': {
                    'per_frame_threshold_ms': _rounded(latency_threshold_ms),
                    'require_p95_below_threshold': True,
                    'require_max_below_threshold': True,
                },
                'observed_area': {
                    'enabled': observed_area_gate_enabled,
                    'minimum_fraction': _rounded(min_observed_area_fraction) if observed_area_gate_enabled else None,
                },
            },
            'vertical_accuracy_pass': bool(vertical_accuracy_pass),
            'processing_time_pass': bool(processing_time_pass),
            'overall_pass': bool(vertical_accuracy_pass and processing_time_pass),
        },
        'artifacts': {
            'summary_json': str(summary_json_path),
            'summary_markdown': str(summary_md_path),
            'error_map_npz': str(error_map_path),
            'error_heatmap_ppm': str(heatmap_path),
        },
        'reproducibility': {
            'terrain': {
                'seed': truth.manifest.get('seed') if isinstance(truth.manifest, dict) else None,
                'manifest_path': str(truth.manifest_path) if truth.manifest_path is not None else None,
                'manifest_fingerprint': file_fingerprint(truth.manifest_path),
                'ground_truth_fingerprint': file_fingerprint(truth.data_path),
            },
            'mission': {
                'mission_summary_path': str((run_dir / 'mission_summary.json').resolve()) if (run_dir / 'mission_summary.json').is_file() else None,
                'planned_path_path': str((run_dir / 'planned_path.json').resolve()) if (run_dir / 'planned_path.json').is_file() else None,
                'planner': mission_summary.get('planner') or planned_path.get('planner'),
                'bounds': mission_summary.get('bounds') or planned_path.get('bounds'),
                'config_fingerprint': file_fingerprint(_as_path((experiment_request.get('mission') or {}).get('config_file'))),
            },
            'mapping': {
                'dem_metadata_path': str((run_dir / 'dem_metadata.json').resolve()) if (run_dir / 'dem_metadata.json').is_file() else None,
                'dem_summary_path': str((run_dir / 'dem_mapper_summary.json').resolve()) if (run_dir / 'dem_mapper_summary.json').is_file() else None,
                'grid': dem_metadata.get('grid'),
                'filters': dem_metadata.get('filters'),
                'manifest': dem_metadata.get('manifest'),
                'config_fingerprint': file_fingerprint(_as_path((experiment_request.get('mapping') or {}).get('config_file'))),
            },
            'software': {
                **software_context(),
                'experiment_request_path': str((run_dir / 'experiment_request.json').resolve()) if (run_dir / 'experiment_request.json').is_file() else None,
                'experiment_request_fingerprint': file_fingerprint(run_dir / 'experiment_request.json'),
            },
        },
        'runtime_status': {
            'mission_summary': mission_summary,
            'dem_mapper_summary': dem_summary,
            'trajectory_head': _load_jsonl_head(run_dir / 'actual_trajectory.jsonl'),
            'mission_events_head': _load_jsonl_head(run_dir / 'mission_events.jsonl'),
        },
    }

    write_json(summary_json_path, summary)
    write_text(summary_md_path, _build_markdown_summary(summary))
    return summary


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Evaluate one completed terrain-mapping run.')
    parser.add_argument('run_dir', help='Completed run directory containing DEM and mission artifacts.')
    parser.add_argument('--vertical-threshold-m', type=float, default=1.0, help='Primary vertical-accuracy threshold in meters.')
    parser.add_argument('--latency-threshold-ms', type=float, default=100.0, help='Per-frame processing-time threshold in milliseconds.')
    parser.add_argument(
        '--primary-vertical-metric',
        choices=METRIC_CHOICES,
        default='rmse_m',
        help='Metric used for the vertical pass/fail verdict.',
    )
    parser.add_argument(
        '--min-observed-area-fraction',
        type=float,
        default=None,
        help='Optional minimum observed-area fraction gate. If omitted, coverage is reported but not used as a pass/fail gate.',
    )
    parser.add_argument('--output-basename', default=DEFAULT_OUTPUT_BASENAME, help='Basename for evaluation artifacts written into the run directory.')
    parser.add_argument('--no-snapshot-inputs', action='store_true', help='Do not snapshot resolved truth inputs into the run directory.')
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parse_args(argv)
    summary = evaluate_run_directory(
        Path(args.run_dir),
        vertical_threshold_m=args.vertical_threshold_m,
        latency_threshold_ms=args.latency_threshold_ms,
        primary_vertical_metric=args.primary_vertical_metric,
        min_observed_area_fraction=args.min_observed_area_fraction,
        output_basename=args.output_basename,
        snapshot_inputs=not args.no_snapshot_inputs,
    )
    verdict = 'PASS' if summary['acceptance']['overall_pass'] else 'FAIL'
    vertical = summary['metrics']['vertical']
    latency = summary['metrics']['latency']
    print(
        f'{summary["run_id"]}: {verdict} '
        f'RMSE={vertical["rmse_m"]}m '
        f'Obs={vertical["observed_area_fraction"]} '
        f'LatencyP95={latency["p95_processing_ms"]}ms'
    )


if __name__ == '__main__':
    main()
