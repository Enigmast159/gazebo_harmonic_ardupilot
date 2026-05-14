import csv
import json
from pathlib import Path

import numpy as np

from terrain_mapping_system.validation.evaluator import evaluate_run_directory


def _write_json(path: Path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding='utf-8')


def test_run_evaluator_writes_metrics_and_visualization(tmp_path):
    run_dir = tmp_path / 'synthetic_run'
    run_dir.mkdir()

    x_coordinates = np.array([-1.0, 0.0, 1.0], dtype=np.float32)
    y_coordinates = np.array([-1.0, 0.0, 1.0], dtype=np.float32)
    truth_height = np.array(
        [
            [0.0, 1.0, 2.0],
            [1.0, 2.0, 3.0],
            [2.0, 3.0, 4.0],
        ],
        dtype=np.float32,
    )
    predicted_height = np.array(
        [
            [0.0, 1.5, np.nan],
            [1.2, 1.8, 3.4],
            [np.nan, 3.0, 4.0],
        ],
        dtype=np.float32,
    )
    coverage_mask = np.array(
        [
            [True, True, False],
            [True, True, True],
            [False, True, True],
        ],
        dtype=bool,
    )

    metadata = {
        'run_id': 'synthetic_run',
        'grid': {
            'x_min': -1.0,
            'x_max': 1.0,
            'y_min': -1.0,
            'y_max': 1.0,
            'resolution_x_m': 1.0,
            'resolution_y_m': 1.0,
            'rows': 3,
            'cols': 3,
        },
        'manifest': {
            'manifest_path': str(run_dir / 'terrain_manifest.snapshot.json'),
            'seed': 123,
            'canonical_file_paths': {
                'ground_truth_csv': str(run_dir / 'terrain_ground_truth.snapshot.csv'),
            },
        },
        'filters': {
            'min_range_m': 0.75,
            'max_range_m': 100.0,
        },
    }
    np.savez_compressed(
        run_dir / 'dem_map.npz',
        height_mean_m=predicted_height,
        height_std_m=np.zeros_like(predicted_height),
        height_min_m=np.where(coverage_mask, predicted_height, np.nan),
        height_max_m=np.where(coverage_mask, predicted_height, np.nan),
        sample_count=np.where(coverage_mask, 1, 0).astype(np.int32),
        coverage_mask=coverage_mask,
        x_coordinates_m=x_coordinates,
        y_coordinates_m=y_coordinates,
        metadata_json=np.array(json.dumps(metadata, sort_keys=True)),
    )
    _write_json(run_dir / 'dem_metadata.json', metadata)
    _write_json(
        run_dir / 'dem_mapper_summary.json',
        {
            'run_id': 'synthetic_run',
            'coverage_ratio': 0.777778,
            'latency': {
                'frames_processed': 3,
                'mean_processing_ms': 20.0,
                'p95_processing_ms': 29.0,
                'max_processing_ms': 30.0,
            },
        },
    )
    _write_json(
        run_dir / 'mission_summary.json',
        {
            'run_id': 'synthetic_run',
            'final_status': 'completed',
            'planner': {
                'mapping_altitude_m': 15.0,
                'lane_spacing_m': 12.0,
                'boundary_margin_m': 5.0,
                'sweep_direction': 'x',
                'waypoint_tolerance_m': 3.0,
            },
            'bounds': {
                'x_min': -1.0,
                'x_max': 1.0,
                'y_min': -1.0,
                'y_max': 1.0,
            },
        },
    )
    _write_json(
        run_dir / 'planned_path.json',
        {
            'run_id': 'synthetic_run',
            'planner': {
                'mapping_altitude_m': 15.0,
                'lane_spacing_m': 12.0,
            },
            'bounds': {
                'x_min': -1.0,
                'x_max': 1.0,
                'y_min': -1.0,
                'y_max': 1.0,
            },
            'bounds_metadata': {
                'seed': 123,
                'manifest_path': str(run_dir / 'terrain_manifest.snapshot.json'),
                'canonical_file_paths': {
                    'ground_truth_csv': str(run_dir / 'terrain_ground_truth.snapshot.csv'),
                },
            },
        },
    )

    with (run_dir / 'terrain_ground_truth.snapshot.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['row', 'col', 'x_m', 'y_m', 'z_m'])
        for row in range(truth_height.shape[0]):
            for col in range(truth_height.shape[1]):
                writer.writerow([row, col, x_coordinates[col], y_coordinates[row], truth_height[row, col]])

    _write_json(
        run_dir / 'terrain_manifest.snapshot.json',
        {
            'seed': 123,
            'generated_at': '2026-04-28T00:00:00Z',
            'canonical_file_paths': {
                'manifest': str(run_dir / 'terrain_manifest.snapshot.json'),
                'ground_truth_csv': str(run_dir / 'terrain_ground_truth.snapshot.csv'),
            },
            'terrain_extents': {
                'x_min': -1.0,
                'x_max': 1.0,
                'y_min': -1.0,
                'y_max': 1.0,
                'z_min': 0.0,
                'z_max': 4.0,
            },
        },
    )

    with (run_dir / 'dem_latency.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                'frame_index',
                'stamp_sec',
                'processing_ms',
                'points_in',
                'points_after_range',
                'points_after_self_filter',
                'points_used',
                'cells_updated',
            ],
        )
        writer.writeheader()
        writer.writerow({'frame_index': 1, 'stamp_sec': 1.0, 'processing_ms': 10.0, 'points_in': 100, 'points_after_range': 90, 'points_after_self_filter': 80, 'points_used': 80, 'cells_updated': 4})
        writer.writerow({'frame_index': 2, 'stamp_sec': 2.0, 'processing_ms': 20.0, 'points_in': 100, 'points_after_range': 90, 'points_after_self_filter': 80, 'points_used': 80, 'cells_updated': 4})
        writer.writerow({'frame_index': 3, 'stamp_sec': 3.0, 'processing_ms': 30.0, 'points_in': 100, 'points_after_range': 90, 'points_after_self_filter': 80, 'points_used': 80, 'cells_updated': 4})

    summary = evaluate_run_directory(
        run_dir,
        vertical_threshold_m=0.3,
        latency_threshold_ms=35.0,
        primary_vertical_metric='rmse_m',
        output_basename='run_evaluation',
    )

    assert summary['acceptance']['overall_pass'] is True
    assert summary['acceptance']['vertical_accuracy_pass'] is True
    assert summary['acceptance']['processing_time_pass'] is True

    vertical = summary['metrics']['vertical']
    assert vertical['observed_cells'] == 7
    assert vertical['comparison_domain_cells'] == 9
    assert vertical['observed_area_fraction'] == 0.777778
    assert vertical['mean_signed_error_m'] == 0.128571
    assert vertical['mae_m'] == 0.185714
    assert vertical['median_abs_error_m'] == 0.2
    assert vertical['rmse_m'] == 0.264575
    assert vertical['max_abs_error_m'] == 0.5
    assert vertical['abs_error_within_threshold_m'] == 0.2
    assert vertical['abs_error_within_threshold_cells'] == 5
    assert vertical['abs_error_within_threshold_fraction'] == 0.714286

    latency = summary['metrics']['latency']
    assert latency['frame_count'] == 3
    assert latency['mean_processing_ms'] == 20.0
    assert latency['max_processing_ms'] == 30.0

    assert (run_dir / 'run_evaluation.json').is_file()
    assert (run_dir / 'run_evaluation.md').is_file()
    assert (run_dir / 'run_evaluation_error_map.npz').is_file()
    assert (run_dir / 'run_evaluation_error_heatmap.ppm').is_file()
    markdown = (run_dir / 'run_evaluation.md').read_text(encoding='utf-8')
    assert 'Mean signed error' in markdown
    assert 'Median absolute error' in markdown
    assert 'Fraction within 0.2 m' in markdown
