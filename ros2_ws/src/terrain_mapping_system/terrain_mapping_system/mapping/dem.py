"""Core DEM mapping utilities shared by the ROS node and offline tests."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from terrain_mapping_system.mission.terrain_manifest import (
    canonical_file_paths_from_manifest,
    grid_shape_from_manifest,
    load_manifest,
    terrain_extents_from_manifest,
    world_name_from_manifest,
)


def resolve_run_id(run_id: str) -> str:
    """Return an existing run id or generate one in the established timestamp format."""

    return run_id or time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())


@dataclass(frozen=True)
class GridSpec:
    """Map-fixed DEM lattice definition."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float
    resolution_x_m: float
    resolution_y_m: float
    rows: int
    cols: int

    def as_dict(self) -> Dict[str, Any]:
        return {
            'x_min': self.x_min,
            'x_max': self.x_max,
            'y_min': self.y_min,
            'y_max': self.y_max,
            'resolution_x_m': self.resolution_x_m,
            'resolution_y_m': self.resolution_y_m,
            'rows': self.rows,
            'cols': self.cols,
        }

    def x_coordinates(self) -> np.ndarray:
        return self.x_min + np.arange(self.cols, dtype=np.float64) * self.resolution_x_m

    def y_coordinates(self) -> np.ndarray:
        return self.y_min + np.arange(self.rows, dtype=np.float64) * self.resolution_y_m


@dataclass(frozen=True)
class LidarExtrinsics:
    """Fixed sensor pose relative to base_link."""

    translation_xyz: Tuple[float, float, float]
    rpy_rad: Tuple[float, float, float]

    def rotation_sensor_to_base(self) -> np.ndarray:
        roll, pitch, yaw = self.rpy_rad
        return rpy_to_rotation_matrix(roll, pitch, yaw)

    def as_dict(self) -> Dict[str, Any]:
        return {
            'translation_xyz': list(self.translation_xyz),
            'rpy_rad': list(self.rpy_rad),
        }


def create_grid_spec(
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    resolution_m: float,
) -> GridSpec:
    """Create a DEM lattice with endpoints included."""

    if resolution_m <= 0.0:
        raise ValueError('resolution_m must be positive')
    width = float(x_max) - float(x_min)
    height = float(y_max) - float(y_min)
    cols = int(np.round(width / resolution_m)) + 1
    rows = int(np.round(height / resolution_m)) + 1
    if cols < 2 or rows < 2:
        raise ValueError('grid extents are too small for the requested resolution')
    return GridSpec(
        x_min=float(x_min),
        y_min=float(y_min),
        x_max=float(x_max),
        y_max=float(y_max),
        resolution_x_m=width / float(cols - 1),
        resolution_y_m=height / float(rows - 1),
        rows=rows,
        cols=cols,
    )


def grid_spec_from_manifest(manifest: Dict[str, Any], resolution_m: float = 0.0) -> GridSpec:
    """Derive DEM extents from the terrain manifest."""

    extents = terrain_extents_from_manifest(manifest)
    x_min = float(extents['x_min'])
    x_max = float(extents['x_max'])
    y_min = float(extents['y_min'])
    y_max = float(extents['y_max'])

    if resolution_m > 0.0:
        return create_grid_spec(
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            resolution_m=resolution_m,
        )

    grid_shape = grid_shape_from_manifest(manifest)
    if isinstance(grid_shape, dict):
        rows = int(grid_shape.get('rows', 0))
        cols = int(grid_shape.get('cols', 0))
        if rows >= 2 and cols >= 2:
            return GridSpec(
                x_min=x_min,
                y_min=y_min,
                x_max=x_max,
                y_max=y_max,
                resolution_x_m=(x_max - x_min) / float(cols - 1),
                resolution_y_m=(y_max - y_min) / float(rows - 1),
                rows=rows,
                cols=cols,
            )

    return create_grid_spec(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        resolution_m=1.0,
    )


def pointcloud2_has_xyz_fields(message: PointCloud2) -> bool:
    names = {field.name for field in message.fields}
    return {'x', 'y', 'z'}.issubset(names)


def _pointfield_dtype(field: PointField, byte_order: str) -> np.dtype:
    datatype_map = {
        PointField.INT8: 'i1',
        PointField.UINT8: 'u1',
        PointField.INT16: 'i2',
        PointField.UINT16: 'u2',
        PointField.INT32: 'i4',
        PointField.UINT32: 'u4',
        PointField.FLOAT32: 'f4',
        PointField.FLOAT64: 'f8',
    }
    if field.datatype not in datatype_map:
        raise ValueError(f'unsupported PointField datatype: {field.datatype}')
    return np.dtype(byte_order + datatype_map[field.datatype])


def extract_xyz_from_pointcloud2(message: PointCloud2) -> np.ndarray:
    """Extract x/y/z columns from PointCloud2 without per-point Python loops."""

    if message.point_step <= 0:
        return np.empty((0, 3), dtype=np.float32)

    field_map = {field.name: field for field in message.fields}
    missing_fields = {'x', 'y', 'z'} - set(field_map)
    if missing_fields:
        raise ValueError(f'PointCloud2 missing fields: {sorted(missing_fields)}')

    byte_order = '>' if message.is_bigendian else '<'
    dtype = np.dtype(
        {
            'names': ['x', 'y', 'z'],
            'formats': [
                _pointfield_dtype(field_map['x'], byte_order),
                _pointfield_dtype(field_map['y'], byte_order),
                _pointfield_dtype(field_map['z'], byte_order),
            ],
            'offsets': [
                int(field_map['x'].offset),
                int(field_map['y'].offset),
                int(field_map['z'].offset),
            ],
            'itemsize': int(message.point_step),
        }
    )
    point_count = int(message.width) * int(message.height)
    if point_count <= 0:
        return np.empty((0, 3), dtype=np.float32)

    structured = np.frombuffer(message.data, dtype=dtype, count=point_count)
    xyz = np.empty((point_count, 3), dtype=np.float32)
    xyz[:, 0] = structured['x'].astype(np.float32, copy=False)
    xyz[:, 1] = structured['y'].astype(np.float32, copy=False)
    xyz[:, 2] = structured['z'].astype(np.float32, copy=False)
    return xyz


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for intrinsic XYZ roll/pitch/yaw."""

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a ROS quaternion."""

    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class DEMGridAccumulator:
    """Accumulates DEM statistics on a fixed 2.5D grid."""

    def __init__(self, grid_spec: GridSpec) -> None:
        self.grid_spec = grid_spec
        shape = (grid_spec.rows, grid_spec.cols)
        self.sample_count = np.zeros(shape, dtype=np.int32)
        self.height_sum = np.zeros(shape, dtype=np.float64)
        self.height_sq_sum = np.zeros(shape, dtype=np.float64)
        self.height_min = np.full(shape, np.inf, dtype=np.float32)
        self.height_max = np.full(shape, -np.inf, dtype=np.float32)

    def integrate_points(self, points_map: np.ndarray) -> Dict[str, int]:
        """Fuse map-frame points into the DEM."""

        if points_map.size == 0:
            return {'points_used': 0, 'cells_updated': 0}

        x = points_map[:, 0]
        y = points_map[:, 1]
        z = points_map[:, 2].astype(np.float32, copy=False)

        col = np.rint((x - self.grid_spec.x_min) / self.grid_spec.resolution_x_m).astype(np.int64, copy=False)
        row = np.rint((y - self.grid_spec.y_min) / self.grid_spec.resolution_y_m).astype(np.int64, copy=False)

        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & (row >= 0)
            & (row < self.grid_spec.rows)
            & (col >= 0)
            & (col < self.grid_spec.cols)
        )
        if not np.any(valid):
            return {'points_used': 0, 'cells_updated': 0}

        row = row[valid]
        col = col[valid]
        z = z[valid]
        flat_index = row * self.grid_spec.cols + col

        count_flat = self.sample_count.reshape(-1)
        sum_flat = self.height_sum.reshape(-1)
        sq_sum_flat = self.height_sq_sum.reshape(-1)
        min_flat = self.height_min.reshape(-1)
        max_flat = self.height_max.reshape(-1)

        np.add.at(count_flat, flat_index, 1)
        np.add.at(sum_flat, flat_index, z.astype(np.float64))
        np.add.at(sq_sum_flat, flat_index, np.square(z, dtype=np.float64))
        np.minimum.at(min_flat, flat_index, z)
        np.maximum.at(max_flat, flat_index, z)

        return {
            'points_used': int(z.shape[0]),
            'cells_updated': int(np.unique(flat_index).shape[0]),
        }

    def height_mean(self) -> np.ndarray:
        mean = np.full(self.sample_count.shape, np.nan, dtype=np.float32)
        valid = self.sample_count > 0
        mean[valid] = (self.height_sum[valid] / self.sample_count[valid]).astype(np.float32)
        return mean

    def height_std(self) -> np.ndarray:
        std = np.full(self.sample_count.shape, np.nan, dtype=np.float32)
        valid = self.sample_count > 0
        variance = np.zeros(self.sample_count.shape, dtype=np.float64)
        variance[valid] = (
            self.height_sq_sum[valid] / self.sample_count[valid]
            - np.square(self.height_sum[valid] / self.sample_count[valid])
        )
        variance = np.maximum(variance, 0.0)
        std[valid] = np.sqrt(variance[valid]).astype(np.float32)
        return std

    def coverage_mask(self) -> np.ndarray:
        return self.sample_count > 0


def accumulate_cloud_into_dem(
    *,
    sensor_points: np.ndarray,
    vehicle_translation_xyz: Tuple[float, float, float],
    vehicle_quaternion_xyzw: Tuple[float, float, float, float],
    extrinsics: LidarExtrinsics,
    accumulator: DEMGridAccumulator,
    min_range_m: float,
    max_range_m: float,
    self_filter_base_z_max_m: float,
    z_bounds_m: Optional[Tuple[float, float]],
    sensor_axis_signs_xyz: Tuple[float, float, float] = (1.0, 1.0, 1.0),
    map_axis_signs_xyz: Tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> Dict[str, int]:
    """Transform one cloud into map frame, filter it, and fuse it into the DEM."""

    if sensor_points.size == 0:
        return {'points_in': 0, 'points_after_range': 0, 'points_after_self_filter': 0, 'points_used': 0, 'cells_updated': 0}

    finite_mask = np.all(np.isfinite(sensor_points), axis=1)
    points_sensor = sensor_points[finite_mask].astype(np.float64, copy=False)
    points_in = int(points_sensor.shape[0])
    if points_in == 0:
        return {'points_in': 0, 'points_after_range': 0, 'points_after_self_filter': 0, 'points_used': 0, 'cells_updated': 0}

    # Some bridges publish the cloud with one sensor axis mirrored relative to
    # the SDF pose convention. Apply sign correction before rigid extrinsics.
    axis_signs = np.asarray(sensor_axis_signs_xyz, dtype=np.float64)
    if axis_signs.shape != (3,):
        raise ValueError('sensor_axis_signs_xyz must contain exactly 3 values')
    points_sensor *= axis_signs

    ranges = np.linalg.norm(points_sensor, axis=1)
    range_mask = (ranges >= min_range_m) & (ranges <= max_range_m)
    points_sensor = points_sensor[range_mask]
    points_after_range = int(points_sensor.shape[0])
    if points_after_range == 0:
        return {'points_in': points_in, 'points_after_range': 0, 'points_after_self_filter': 0, 'points_used': 0, 'cells_updated': 0}

    rotation_sensor_to_base = extrinsics.rotation_sensor_to_base()
    translation_sensor_to_base = np.asarray(extrinsics.translation_xyz, dtype=np.float64)
    points_base = points_sensor @ rotation_sensor_to_base.T + translation_sensor_to_base

    self_mask = points_base[:, 2] <= self_filter_base_z_max_m
    points_base = points_base[self_mask]
    points_after_self_filter = int(points_base.shape[0])
    if points_after_self_filter == 0:
        return {
            'points_in': points_in,
            'points_after_range': points_after_range,
            'points_after_self_filter': 0,
            'points_used': 0,
            'cells_updated': 0,
        }

    rotation_base_to_map = quaternion_to_rotation_matrix(*vehicle_quaternion_xyzw)
    translation_base_to_map = np.asarray(vehicle_translation_xyz, dtype=np.float64)
    points_map = points_base @ rotation_base_to_map.T + translation_base_to_map

    map_axis_signs = np.asarray(map_axis_signs_xyz, dtype=np.float64)
    if map_axis_signs.shape != (3,):
        raise ValueError('map_axis_signs_xyz must contain exactly 3 values')
    points_map *= map_axis_signs

    if z_bounds_m is not None:
        z_min, z_max = z_bounds_m
        z_mask = (points_map[:, 2] >= z_min) & (points_map[:, 2] <= z_max)
        points_map = points_map[z_mask]

    fused = accumulator.integrate_points(points_map)
    return {
        'points_in': points_in,
        'points_after_range': points_after_range,
        'points_after_self_filter': points_after_self_filter,
        'points_used': int(fused['points_used']),
        'cells_updated': int(fused['cells_updated']),
    }


def flatten_dem_cells(
    accumulator: DEMGridAccumulator,
    *,
    min_samples: int = 1,
) -> Dict[str, np.ndarray]:
    """Return one flat record per observed DEM cell."""

    threshold = max(int(min_samples), 1)
    sample_count = accumulator.sample_count
    mask = sample_count >= threshold
    if not np.any(mask):
        empty_float = np.empty((0,), dtype=np.float32)
        empty_int = np.empty((0,), dtype=np.int32)
        return {
            'row': empty_int,
            'col': empty_int,
            'x_m': empty_float,
            'y_m': empty_float,
            'height_mean_m': empty_float,
            'height_std_m': empty_float,
            'height_min_m': empty_float,
            'height_max_m': empty_float,
            'sample_count': empty_int,
        }

    x_coordinates = accumulator.grid_spec.x_coordinates().astype(np.float32)
    y_coordinates = accumulator.grid_spec.y_coordinates().astype(np.float32)
    x_mesh, y_mesh = np.meshgrid(x_coordinates, y_coordinates)
    row, col = np.nonzero(mask)
    height_mean = accumulator.height_mean()
    height_std = accumulator.height_std()

    return {
        'row': row.astype(np.int32, copy=False),
        'col': col.astype(np.int32, copy=False),
        'x_m': x_mesh[mask].astype(np.float32, copy=False),
        'y_m': y_mesh[mask].astype(np.float32, copy=False),
        'height_mean_m': height_mean[mask].astype(np.float32, copy=False),
        'height_std_m': height_std[mask].astype(np.float32, copy=False),
        'height_min_m': accumulator.height_min[mask].astype(np.float32, copy=False),
        'height_max_m': accumulator.height_max[mask].astype(np.float32, copy=False),
        'sample_count': sample_count[mask].astype(np.int32, copy=False),
    }


def build_dem_pointcloud2(
    accumulator: DEMGridAccumulator,
    *,
    frame_id: str = 'map',
    stamp: Any = None,
    min_samples: int = 1,
) -> PointCloud2:
    """Convert observed DEM cells into a PointCloud2 message for RViz."""

    cells = flatten_dem_cells(accumulator, min_samples=min_samples)
    point_count = int(cells['x_m'].shape[0])

    dtype = np.dtype(
        [
            ('x', '<f4'),
            ('y', '<f4'),
            ('z', '<f4'),
            ('intensity', '<f4'),
        ]
    )
    payload = np.empty((point_count,), dtype=dtype)
    payload['x'] = cells['x_m']
    payload['y'] = cells['y_m']
    payload['z'] = cells['height_mean_m']
    payload['intensity'] = cells['height_mean_m']

    message = PointCloud2()
    message.header.frame_id = frame_id
    if stamp is not None:
        message.header.stamp = stamp
    message.height = 1
    message.width = point_count
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = int(dtype.itemsize)
    message.row_step = int(dtype.itemsize) * point_count
    message.is_dense = True
    message.data = payload.tobytes()
    return message


class DEMArtifactWriter:
    """Writes DEM arrays, quicklooks, latency logs, and summary metadata."""

    def __init__(self, results_root: str, run_id: str) -> None:
        self.results_dir = Path(results_root).expanduser().resolve() / run_id
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.dem_npz_path = self.results_dir / 'dem_map.npz'
        self.dem_cells_csv_path = self.results_dir / 'dem_cells.csv'
        self.dem_quicklook_path = self.results_dir / 'dem_quicklook.pgm'
        self.coverage_quicklook_path = self.results_dir / 'coverage_quicklook.pgm'
        self.latency_csv_path = self.results_dir / 'dem_latency.csv'
        self.summary_path = self.results_dir / 'dem_mapper_summary.json'
        self.metadata_path = self.results_dir / 'dem_metadata.json'
        self._latency_header_written = self.latency_csv_path.exists() and self.latency_csv_path.stat().st_size > 0
        if not self._latency_header_written:
            self._write_latency_header()

    def append_latency_row(self, payload: Dict[str, Any]) -> None:
        fieldnames = [
            'frame_index',
            'stamp_sec',
            'processing_ms',
            'points_in',
            'points_after_range',
            'points_after_self_filter',
            'points_used',
            'cells_updated',
        ]
        with self.latency_csv_path.open('a', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writerow({name: payload.get(name) for name in fieldnames})

    def write_snapshot(
        self,
        *,
        accumulator: DEMGridAccumulator,
        metadata: Dict[str, Any],
        summary: Dict[str, Any],
    ) -> None:
        height_mean = accumulator.height_mean()
        height_std = accumulator.height_std()
        coverage_mask = accumulator.coverage_mask()

        np.savez_compressed(
            self.dem_npz_path,
            height_mean_m=height_mean,
            height_std_m=height_std,
            height_min_m=np.where(coverage_mask, accumulator.height_min, np.nan).astype(np.float32),
            height_max_m=np.where(coverage_mask, accumulator.height_max, np.nan).astype(np.float32),
            sample_count=accumulator.sample_count,
            coverage_mask=coverage_mask,
            x_coordinates_m=accumulator.grid_spec.x_coordinates().astype(np.float32),
            y_coordinates_m=accumulator.grid_spec.y_coordinates().astype(np.float32),
            metadata_json=np.array(json.dumps(metadata, sort_keys=True)),
        )
        self._write_quicklook(self.dem_quicklook_path, height_mean)
        self._write_quicklook(self.coverage_quicklook_path, np.log1p(accumulator.sample_count.astype(np.float32)))
        self._write_cells_csv(self.dem_cells_csv_path, accumulator)
        self._write_json(self.metadata_path, metadata)
        self._write_json(self.summary_path, summary)

    def snapshot(self) -> Dict[str, str]:
        return {
            'results_dir': str(self.results_dir),
            'dem_map': str(self.dem_npz_path),
            'dem_cells_csv': str(self.dem_cells_csv_path),
            'dem_quicklook': str(self.dem_quicklook_path),
            'coverage_quicklook': str(self.coverage_quicklook_path),
            'latency_csv': str(self.latency_csv_path),
            'metadata_json': str(self.metadata_path),
            'summary_json': str(self.summary_path),
        }

    def _write_json(self, path: Path, payload: Dict[str, Any]) -> None:
        with path.open('w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')

    def _write_latency_header(self) -> None:
        fieldnames = [
            'frame_index',
            'stamp_sec',
            'processing_ms',
            'points_in',
            'points_after_range',
            'points_after_self_filter',
            'points_used',
            'cells_updated',
        ]
        with self.latency_csv_path.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
        self._latency_header_written = True

    def _write_quicklook(self, path: Path, image_data: np.ndarray) -> None:
        finite = np.isfinite(image_data)
        output = np.zeros(image_data.shape, dtype=np.uint8)
        if np.any(finite):
            valid = image_data[finite].astype(np.float64)
            value_min = float(valid.min())
            value_max = float(valid.max())
            if value_max > value_min:
                normalized = (image_data - value_min) / (value_max - value_min)
                output[finite] = np.clip(normalized[finite] * 255.0, 0.0, 255.0).astype(np.uint8)
            else:
                output[finite] = 255
        flipped = np.flipud(output)
        with path.open('wb') as stream:
            header = f'P5\n{flipped.shape[1]} {flipped.shape[0]}\n255\n'.encode('ascii')
            stream.write(header)
            stream.write(flipped.tobytes())

    def _write_cells_csv(self, path: Path, accumulator: DEMGridAccumulator) -> None:
        fieldnames = [
            'row',
            'col',
            'x_m',
            'y_m',
            'height_mean_m',
            'height_std_m',
            'height_min_m',
            'height_max_m',
            'sample_count',
        ]
        cells = flatten_dem_cells(accumulator)
        row_count = int(cells['row'].shape[0])
        with path.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for index in range(row_count):
                writer.writerow(
                    {
                        'row': int(cells['row'][index]),
                        'col': int(cells['col'][index]),
                        'x_m': round(float(cells['x_m'][index]), 6),
                        'y_m': round(float(cells['y_m'][index]), 6),
                        'height_mean_m': round(float(cells['height_mean_m'][index]), 6),
                        'height_std_m': round(float(cells['height_std_m'][index]), 6),
                        'height_min_m': round(float(cells['height_min_m'][index]), 6),
                        'height_max_m': round(float(cells['height_max_m'][index]), 6),
                        'sample_count': int(cells['sample_count'][index]),
                    }
                )


def manifest_metadata_from_path(manifest_path: str) -> Dict[str, Any]:
    manifest = load_manifest(manifest_path)
    return {
        'path': str(Path(manifest_path).expanduser().resolve()),
        'world_name': world_name_from_manifest(manifest),
        'generated_at': manifest.get('generated_at'),
        'seed': manifest.get('seed'),
        'terrain_extents': terrain_extents_from_manifest(manifest),
        'grid_shape_raw': grid_shape_from_manifest(manifest),
        'canonical_file_paths': canonical_file_paths_from_manifest(manifest),
    }
