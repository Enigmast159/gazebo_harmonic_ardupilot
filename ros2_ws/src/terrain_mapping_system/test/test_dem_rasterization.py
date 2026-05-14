import csv
import json

import numpy as np

from terrain_mapping_system.mapping.dem import (
    DEMArtifactWriter,
    DEMGridAccumulator,
    LidarExtrinsics,
    accumulate_cloud_into_dem,
    build_dem_pointcloud2,
    create_grid_spec,
    flatten_dem_cells,
)


def test_dem_rasterization_accumulates_per_cell_statistics():
    grid_spec = create_grid_spec(
        x_min=0.0,
        x_max=2.0,
        y_min=0.0,
        y_max=2.0,
        resolution_m=1.0,
    )
    accumulator = DEMGridAccumulator(grid_spec)

    points_map = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.2, 0.1, 3.0],
            [1.0, 1.0, 2.5],
            [2.0, 2.0, 4.0],
        ],
        dtype=np.float32,
    )
    stats = accumulator.integrate_points(points_map)

    assert stats == {'points_used': 4, 'cells_updated': 3}
    assert accumulator.sample_count[0, 0] == 2
    assert np.isclose(accumulator.height_mean()[0, 0], 2.0)
    assert np.isclose(accumulator.height_min[0, 0], 1.0)
    assert np.isclose(accumulator.height_max[0, 0], 3.0)
    assert accumulator.sample_count[1, 1] == 1
    assert accumulator.coverage_mask()[2, 2]


def test_cloud_accumulation_applies_extrinsics_and_vehicle_pose():
    grid_spec = create_grid_spec(
        x_min=-2.0,
        x_max=2.0,
        y_min=-2.0,
        y_max=2.0,
        resolution_m=1.0,
    )
    accumulator = DEMGridAccumulator(grid_spec)
    extrinsics = LidarExtrinsics(
        translation_xyz=(0.0, 0.0, -0.15),
        rpy_rad=(0.0, 0.0, 0.0),
    )

    sensor_points = np.array(
        [
            [0.0, 0.0, -1.0],
            [1.0, 0.0, -1.0],
        ],
        dtype=np.float32,
    )
    stats = accumulate_cloud_into_dem(
        sensor_points=sensor_points,
        vehicle_translation_xyz=(1.0, 2.0, 10.0),
        vehicle_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        extrinsics=extrinsics,
        accumulator=accumulator,
        min_range_m=0.1,
        max_range_m=10.0,
        self_filter_base_z_max_m=0.0,
        z_bounds_m=(0.0, 20.0),
    )

    assert stats['points_used'] == 2
    mean = accumulator.height_mean()
    assert np.isclose(mean[4, 3], 8.85)
    assert np.isclose(mean[4, 4], 8.85)


def test_cloud_accumulation_can_flip_map_y_without_disturbing_vertical_filtering():
    grid_spec = create_grid_spec(
        x_min=-2.0,
        x_max=2.0,
        y_min=-2.0,
        y_max=2.0,
        resolution_m=1.0,
    )
    extrinsics = LidarExtrinsics(
        translation_xyz=(0.0, 0.0, 0.0),
        rpy_rad=(np.pi / 2.0, 0.0, 0.0),
    )
    sensor_points = np.array([[0.0, -1.0, 1.0]], dtype=np.float32)

    reflected = DEMGridAccumulator(grid_spec)
    reflected_stats = accumulate_cloud_into_dem(
        sensor_points=sensor_points,
        vehicle_translation_xyz=(0.0, 0.0, 0.0),
        vehicle_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        extrinsics=extrinsics,
        accumulator=reflected,
        min_range_m=0.1,
        max_range_m=10.0,
        self_filter_base_z_max_m=0.0,
        z_bounds_m=(-2.0, 2.0),
    )

    corrected = DEMGridAccumulator(grid_spec)
    corrected_stats = accumulate_cloud_into_dem(
        sensor_points=sensor_points,
        vehicle_translation_xyz=(0.0, 0.0, 0.0),
        vehicle_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        extrinsics=extrinsics,
        accumulator=corrected,
        min_range_m=0.1,
        max_range_m=10.0,
        self_filter_base_z_max_m=0.0,
        z_bounds_m=(-2.0, 2.0),
        map_axis_signs_xyz=(1.0, -1.0, 1.0),
    )

    assert reflected_stats['points_used'] == 1
    assert corrected_stats['points_used'] == 1
    reflected_row, reflected_col = np.argwhere(reflected.coverage_mask())[0].tolist()
    corrected_row, corrected_col = np.argwhere(corrected.coverage_mask())[0].tolist()
    assert reflected_col == corrected_col == 2
    assert reflected_row == 1
    assert corrected_row == 3


def test_flatten_dem_cells_and_build_pointcloud_message():
    grid_spec = create_grid_spec(
        x_min=0.0,
        x_max=1.0,
        y_min=0.0,
        y_max=1.0,
        resolution_m=1.0,
    )
    accumulator = DEMGridAccumulator(grid_spec)
    accumulator.integrate_points(
        np.array(
            [
                [0.0, 0.0, 1.5],
                [1.0, 1.0, 2.5],
            ],
            dtype=np.float32,
        )
    )

    cells = flatten_dem_cells(accumulator)
    assert cells['row'].tolist() == [0, 1]
    assert cells['col'].tolist() == [0, 1]
    assert np.allclose(cells['x_m'], [0.0, 1.0])
    assert np.allclose(cells['y_m'], [0.0, 1.0])
    assert np.allclose(cells['height_mean_m'], [1.5, 2.5])

    message = build_dem_pointcloud2(accumulator, frame_id='map')
    assert message.header.frame_id == 'map'
    assert message.width == 2
    assert message.height == 1
    assert message.point_step == 16
    payload = np.frombuffer(
        message.data,
        dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')]),
    )
    assert np.allclose(payload['x'], [0.0, 1.0])
    assert np.allclose(payload['y'], [0.0, 1.0])
    assert np.allclose(payload['z'], [1.5, 2.5])
    assert np.allclose(payload['intensity'], [1.5, 2.5])


def test_artifact_writer_persists_dem_cells_csv(tmp_path):
    grid_spec = create_grid_spec(
        x_min=-1.0,
        x_max=1.0,
        y_min=-1.0,
        y_max=1.0,
        resolution_m=1.0,
    )
    accumulator = DEMGridAccumulator(grid_spec)
    accumulator.integrate_points(
        np.array(
            [
                [-1.0, -1.0, 0.5],
                [0.0, 0.0, 1.0],
                [1.0, 1.0, 1.5],
            ],
            dtype=np.float32,
        )
    )
    writer = DEMArtifactWriter(str(tmp_path), 'dem_writer_test')
    writer.write_snapshot(
        accumulator=accumulator,
        metadata={'run_id': 'dem_writer_test'},
        summary={'run_id': 'dem_writer_test'},
    )

    csv_path = tmp_path / 'dem_writer_test' / 'dem_cells.csv'
    assert csv_path.is_file()
    with csv_path.open('r', encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3
    assert rows[0]['x_m'] == '-1.0'
    assert rows[0]['y_m'] == '-1.0'
    assert rows[0]['height_mean_m'] == '0.5'

    npz_path = tmp_path / 'dem_writer_test' / 'dem_map.npz'
    with np.load(npz_path, allow_pickle=False) as payload:
        metadata = json.loads(str(payload['metadata_json'].item()))
    assert metadata['run_id'] == 'dem_writer_test'
