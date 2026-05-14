"""ROS 2 node that fuses LiDAR clouds into a fixed-grid DEM."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from terrain_mapping_system.mapping.dem import (
    DEMArtifactWriter,
    DEMGridAccumulator,
    LidarExtrinsics,
    accumulate_cloud_into_dem,
    build_dem_pointcloud2,
    create_grid_spec,
    extract_xyz_from_pointcloud2,
    grid_spec_from_manifest,
    pointcloud2_has_xyz_fields,
    resolve_run_id,
)
from terrain_mapping_system.mission.terrain_manifest import (
    canonical_file_paths_from_manifest,
    grid_shape_from_manifest,
    load_manifest,
    terrain_extents_from_manifest,
    world_name_from_manifest,
)
from terrain_mapping_system.paths import default_results_root
from terrain_mapping_system.validation.evaluator import DEFAULT_OUTPUT_BASENAME, evaluate_run_directory


@dataclass
class VehiclePose:
    stamp_sec: float
    translation_xyz: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]


class DemMapper(Node):
    """Consumes bridged LiDAR and pose topics to build a live DEM."""

    def __init__(self) -> None:
        super().__init__('dem_mapper')

        self._declare_parameters()

        self._point_topic = str(self.get_parameter('point_topic').value)
        self._pose_topic = str(self.get_parameter('pose_topic').value)
        self._run_id = resolve_run_id(str(self.get_parameter('run_id').value))
        self._results_root = str(self.get_parameter('results_root').value)
        self._manifest_path = str(self.get_parameter('manifest_path').value).strip()
        self._use_manifest = bool(self.get_parameter('use_terrain_manifest').value)
        self._grid_resolution_m = float(self.get_parameter('grid_resolution_m').value)
        self._snapshot_interval_s = float(self.get_parameter('snapshot_interval_s').value)
        self._publish_dem_pointcloud = bool(self.get_parameter('publish_dem_pointcloud').value)
        self._dem_pointcloud_topic = str(self.get_parameter('dem_pointcloud_topic').value)
        self._dem_frame_id = str(self.get_parameter('dem_frame_id').value)
        self._dem_min_samples = int(self.get_parameter('dem_min_samples').value)
        self._min_range_m = float(self.get_parameter('min_range_m').value)
        self._max_range_m = float(self.get_parameter('max_range_m').value)
        self._self_filter_base_z_max_m = float(self.get_parameter('self_filter_base_z_max_m').value)
        self._z_margin_below_m = float(self.get_parameter('z_margin_below_m').value)
        self._z_margin_above_m = float(self.get_parameter('z_margin_above_m').value)
        self._write_run_evaluation_on_shutdown = bool(
            self.get_parameter('evaluation.write_on_shutdown').value
        )
        self._evaluation_vertical_threshold_m = float(
            self.get_parameter('evaluation.vertical_threshold_m').value
        )
        self._evaluation_latency_threshold_ms = float(
            self.get_parameter('evaluation.latency_threshold_ms').value
        )
        self._evaluation_primary_vertical_metric = str(
            self.get_parameter('evaluation.primary_vertical_metric').value
        )
        self._evaluation_output_basename = str(
            self.get_parameter('evaluation.output_basename').value
        )
        self._sensor_axis_signs_xyz = (
            float(self.get_parameter('sensor_axis_signs.x').value),
            float(self.get_parameter('sensor_axis_signs.y').value),
            float(self.get_parameter('sensor_axis_signs.z').value),
        )
        self._map_axis_signs_xyz = (
            float(self.get_parameter('map_axis_signs.x').value),
            float(self.get_parameter('map_axis_signs.y').value),
            float(self.get_parameter('map_axis_signs.z').value),
        )

        self._extrinsics = LidarExtrinsics(
            translation_xyz=(
                float(self.get_parameter('lidar_extrinsics.translation_xyz.x').value),
                float(self.get_parameter('lidar_extrinsics.translation_xyz.y').value),
                float(self.get_parameter('lidar_extrinsics.translation_xyz.z').value),
            ),
            rpy_rad=(
                float(self.get_parameter('lidar_extrinsics.rpy_rad.roll').value),
                float(self.get_parameter('lidar_extrinsics.rpy_rad.pitch').value),
                float(self.get_parameter('lidar_extrinsics.rpy_rad.yaw').value),
            ),
        )

        self._manifest: Dict[str, object] = {}
        if self._use_manifest and self._manifest_path:
            self._manifest = load_manifest(self._manifest_path)

        self._grid_spec = self._build_grid_spec()
        self._accumulator = DEMGridAccumulator(self._grid_spec)
        self._artifact_writer = DEMArtifactWriter(self._results_root, self._run_id)

        extents = terrain_extents_from_manifest(self._manifest) if self._manifest else {}
        if isinstance(extents, dict) and 'z_min' in extents and 'z_max' in extents:
            self._z_bounds_m: Optional[Tuple[float, float]] = (
                float(extents['z_min']) - self._z_margin_below_m,
                float(extents['z_max']) + self._z_margin_above_m,
            )
        else:
            self._z_bounds_m = None

        self._last_pose: Optional[VehiclePose] = None
        self._frames_processed = 0
        self._frames_skipped_no_pose = 0
        self._cumulative_points_used = 0
        self._latencies_ms: List[float] = []
        self._last_snapshot_time = 0.0

        self._pose_subscription = self.create_subscription(
            PoseStamped,
            self._pose_topic,
            self._pose_callback,
            20,
        )
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            self._point_topic,
            self._cloud_callback,
            10,
        )
        self._dem_pointcloud_publisher = None
        if self._publish_dem_pointcloud:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._dem_pointcloud_publisher = self.create_publisher(
                PointCloud2,
                self._dem_pointcloud_topic,
                qos,
            )
        self._flush_timer = self.create_timer(self._snapshot_interval_s, self._flush_snapshot)

        self.get_logger().info(
            'DEM mapper ready '
            f'point_topic={self._point_topic} '
            f'pose_topic={self._pose_topic} '
            f'dem_pointcloud_topic={self._dem_pointcloud_topic if self._publish_dem_pointcloud else "disabled"} '
            f'run_id={self._run_id} '
            f'results_dir={self._artifact_writer.results_dir}'
        )
        self.get_logger().info(
            'Using fixed LiDAR extrinsics derived from SDF: '
            f'{json.dumps(self._extrinsics.as_dict(), sort_keys=True)}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('point_topic', '/terrain_mapping/lidar/points')
        self.declare_parameter('pose_topic', '/terrain_mapping/vehicle/pose')
        self.declare_parameter('run_id', '')
        self.declare_parameter('results_root', default_results_root())
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('use_terrain_manifest', True)
        self.declare_parameter('grid_resolution_m', 0.0)
        self.declare_parameter('fallback_bounds.x_min', -50.0)
        self.declare_parameter('fallback_bounds.x_max', 50.0)
        self.declare_parameter('fallback_bounds.y_min', -50.0)
        self.declare_parameter('fallback_bounds.y_max', 50.0)
        self.declare_parameter('fallback_bounds.resolution_m', 1.0)
        self.declare_parameter('snapshot_interval_s', 5.0)
        self.declare_parameter('publish_dem_pointcloud', True)
        self.declare_parameter('dem_pointcloud_topic', '/terrain_mapping/dem/points')
        self.declare_parameter('dem_frame_id', 'map')
        self.declare_parameter('dem_min_samples', 1)
        self.declare_parameter('min_range_m', 0.75)
        self.declare_parameter('max_range_m', 120.0)
        self.declare_parameter('self_filter_base_z_max_m', -0.25)
        self.declare_parameter('z_margin_below_m', 5.0)
        self.declare_parameter('z_margin_above_m', 5.0)
        self.declare_parameter('evaluation.write_on_shutdown', True)
        self.declare_parameter('evaluation.vertical_threshold_m', 1.0)
        self.declare_parameter('evaluation.latency_threshold_ms', 100.0)
        self.declare_parameter('evaluation.primary_vertical_metric', 'rmse_m')
        self.declare_parameter('evaluation.output_basename', DEFAULT_OUTPUT_BASENAME)
        self.declare_parameter('sensor_axis_signs.x', 1.0)
        self.declare_parameter('sensor_axis_signs.y', 1.0)
        self.declare_parameter('sensor_axis_signs.z', 1.0)
        self.declare_parameter('map_axis_signs.x', 1.0)
        self.declare_parameter('map_axis_signs.y', -1.0)
        self.declare_parameter('map_axis_signs.z', 1.0)
        self.declare_parameter('lidar_extrinsics.translation_xyz.x', 0.0)
        self.declare_parameter('lidar_extrinsics.translation_xyz.y', 0.0)
        self.declare_parameter('lidar_extrinsics.translation_xyz.z', -0.15)
        self.declare_parameter('lidar_extrinsics.rpy_rad.roll', 1.5707963)
        self.declare_parameter('lidar_extrinsics.rpy_rad.pitch', 0.0)
        self.declare_parameter('lidar_extrinsics.rpy_rad.yaw', 0.0)

    def _build_grid_spec(self):
        if self._manifest:
            return grid_spec_from_manifest(self._manifest, resolution_m=self._grid_resolution_m)

        fallback_resolution = float(self.get_parameter('fallback_bounds.resolution_m').value)
        return create_grid_spec(
            x_min=float(self.get_parameter('fallback_bounds.x_min').value),
            x_max=float(self.get_parameter('fallback_bounds.x_max').value),
            y_min=float(self.get_parameter('fallback_bounds.y_min').value),
            y_max=float(self.get_parameter('fallback_bounds.y_max').value),
            resolution_m=self._grid_resolution_m if self._grid_resolution_m > 0.0 else fallback_resolution,
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        stamp_sec = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        self._last_pose = VehiclePose(
            stamp_sec=stamp_sec,
            translation_xyz=(
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ),
            quaternion_xyzw=(
                float(message.pose.orientation.x),
                float(message.pose.orientation.y),
                float(message.pose.orientation.z),
                float(message.pose.orientation.w),
            ),
        )

    def _cloud_callback(self, message: PointCloud2) -> None:
        if self._last_pose is None:
            self._frames_skipped_no_pose += 1
            return
        if not pointcloud2_has_xyz_fields(message):
            self.get_logger().warning('Skipping PointCloud2 without x/y/z fields')
            return

        started = time.perf_counter()
        sensor_points = extract_xyz_from_pointcloud2(message)
        stats = accumulate_cloud_into_dem(
            sensor_points=sensor_points,
            vehicle_translation_xyz=self._last_pose.translation_xyz,
            vehicle_quaternion_xyzw=self._last_pose.quaternion_xyzw,
            extrinsics=self._extrinsics,
            accumulator=self._accumulator,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
            self_filter_base_z_max_m=self._self_filter_base_z_max_m,
            z_bounds_m=self._z_bounds_m,
            sensor_axis_signs_xyz=self._sensor_axis_signs_xyz,
            map_axis_signs_xyz=self._map_axis_signs_xyz,
        )
        processing_ms = (time.perf_counter() - started) * 1000.0

        self._frames_processed += 1
        self._cumulative_points_used += stats['points_used']
        self._latencies_ms.append(processing_ms)

        stamp_sec = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        self._artifact_writer.append_latency_row(
            {
                'frame_index': self._frames_processed,
                'stamp_sec': round(stamp_sec, 6),
                'processing_ms': round(processing_ms, 3),
                **stats,
            }
        )

        if self._frames_processed == 1 or self._frames_processed % 10 == 0:
            self.get_logger().info(
                f'frame={self._frames_processed} '
                f'processing_ms={processing_ms:.2f} '
                f'points_in={stats["points_in"]} '
                f'points_used={stats["points_used"]} '
                f'cells_updated={stats["cells_updated"]}'
            )

    def _metadata_payload(self) -> Dict[str, object]:
        manifest_metadata = {}
        if self._manifest:
            manifest_metadata = {
                'manifest_path': self._manifest_path,
                'world_name': world_name_from_manifest(self._manifest),
                'generated_at': self._manifest.get('generated_at'),
                'seed': self._manifest.get('seed'),
                'terrain_extents': terrain_extents_from_manifest(self._manifest),
                'grid_shape_raw': grid_shape_from_manifest(self._manifest),
                'canonical_file_paths': canonical_file_paths_from_manifest(self._manifest),
            }

        return {
            'run_id': self._run_id,
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'point_topic': self._point_topic,
            'pose_topic': self._pose_topic,
            'grid': self._grid_spec.as_dict(),
            'lidar_extrinsics': self._extrinsics.as_dict(),
            'manifest': manifest_metadata,
            'filters': {
                'min_range_m': self._min_range_m,
                'max_range_m': self._max_range_m,
                'self_filter_base_z_max_m': self._self_filter_base_z_max_m,
                'z_bounds_m': list(self._z_bounds_m) if self._z_bounds_m is not None else None,
            },
            'sensor_axis_signs_xyz': list(self._sensor_axis_signs_xyz),
            'map_axis_signs_xyz': list(self._map_axis_signs_xyz),
        }

    def _summary_payload(self) -> Dict[str, object]:
        coverage_mask = self._accumulator.coverage_mask()
        coverage_ratio = float(coverage_mask.sum()) / float(coverage_mask.size) if coverage_mask.size else 0.0
        latencies = np.asarray(self._latencies_ms, dtype=np.float64)
        latency_stats = {
            'frames_processed': self._frames_processed,
            'frames_skipped_no_pose': self._frames_skipped_no_pose,
            'mean_processing_ms': round(float(latencies.mean()), 3) if latencies.size else None,
            'p95_processing_ms': round(float(np.percentile(latencies, 95)), 3) if latencies.size else None,
            'max_processing_ms': round(float(latencies.max()), 3) if latencies.size else None,
        }
        return {
            'run_id': self._run_id,
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'results': self._artifact_writer.snapshot(),
            'grid': self._grid_spec.as_dict(),
            'coverage_cells': int(coverage_mask.sum()),
            'coverage_ratio': round(coverage_ratio, 6),
            'sample_count_total': int(self._accumulator.sample_count.sum()),
            'cumulative_points_used': self._cumulative_points_used,
            'latency': latency_stats,
        }

    def _flush_snapshot(self) -> None:
        now = time.monotonic()
        if self._frames_processed == 0 and self._last_snapshot_time != 0.0:
            return
        self._artifact_writer.write_snapshot(
            accumulator=self._accumulator,
            metadata=self._metadata_payload(),
            summary=self._summary_payload(),
        )
        self._publish_snapshot_pointcloud()
        self._last_snapshot_time = now

    def _publish_snapshot_pointcloud(self) -> None:
        if self._dem_pointcloud_publisher is None:
            return
        message = build_dem_pointcloud2(
            self._accumulator,
            frame_id=self._dem_frame_id,
            stamp=self.get_clock().now().to_msg(),
            min_samples=self._dem_min_samples,
        )
        self._dem_pointcloud_publisher.publish(message)
        self.get_logger().info(
            f'published_dem_pointcloud points={message.width} topic={self._dem_pointcloud_topic}'
        )

    def destroy_node(self) -> bool:
        try:
            self._flush_snapshot()
            self._write_run_evaluation()
        finally:
            return super().destroy_node()

    def _write_run_evaluation(self) -> None:
        if not self._write_run_evaluation_on_shutdown:
            return
        try:
            summary = evaluate_run_directory(
                self._artifact_writer.results_dir,
                vertical_threshold_m=self._evaluation_vertical_threshold_m,
                latency_threshold_ms=self._evaluation_latency_threshold_ms,
                primary_vertical_metric=self._evaluation_primary_vertical_metric,
                output_basename=self._evaluation_output_basename,
                snapshot_inputs=True,
            )
            vertical = summary.get('metrics', {}).get('vertical', {})
            self.get_logger().info(
                'Wrote run evaluation '
                f'output={self._artifact_writer.results_dir / (self._evaluation_output_basename + ".json")} '
                f'rmse_m={vertical.get("rmse_m")} '
                f'mae_m={vertical.get("mae_m")}'
            )
        except Exception as exc:
            self.get_logger().warning(f'Failed to write run evaluation summary: {exc}')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DemMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('DEM mapper interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
