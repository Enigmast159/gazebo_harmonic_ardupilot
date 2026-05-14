from terrain_mapping_system.mapping.dem import grid_spec_from_manifest
from terrain_mapping_system.mission.terrain_manifest import (
    bounds_from_manifest,
    canonical_file_paths_from_manifest,
    grid_shape_from_manifest,
    terrain_extents_from_manifest,
    world_name_from_manifest,
)


def test_heightmap_manifest_helpers_support_native_schema():
    manifest = {
        'generated_at': '2026-05-05T00:00:00Z',
        'seed': 5001,
        'terrain_xy_extents_m': {'x': 100.0, 'y': 100.0},
        'terrain_z_scale_m': 8.0,
        'generator_params': {
            'world': 'random_terrain_ardupilot',
            'size': 257,
        },
        'canonical_file_paths': {
            'ground_truth_csv': '/tmp/terrain_ground_truth.csv',
            'collision_stl': '/tmp/collision.stl',
        },
    }

    extents = terrain_extents_from_manifest(manifest)
    assert extents == {
        'x_min': -50.0,
        'x_max': 50.0,
        'y_min': -50.0,
        'y_max': 50.0,
        'z_min': 0.0,
        'z_max': 8.0,
    }

    assert grid_shape_from_manifest(manifest) == {'rows': 257, 'cols': 257}
    assert world_name_from_manifest(manifest) == 'random_terrain_ardupilot'
    assert canonical_file_paths_from_manifest(manifest)['stl_mesh'] == '/tmp/collision.stl'


def test_bounds_and_grid_spec_accept_heightmap_manifest_schema():
    manifest = {
        'terrain_xy_extents_m': {'x': 100.0, 'y': 80.0},
        'terrain_z_scale_m': 8.0,
        'generator_params': {
            'world': 'random_terrain_ardupilot',
            'size': 257,
        },
        'canonical_file_paths': {
            'ground_truth_csv': '/tmp/terrain_ground_truth.csv',
        },
    }

    bounds, metadata = bounds_from_manifest(manifest)
    assert bounds.x_min == -50.0
    assert bounds.x_max == 50.0
    assert bounds.y_min == -40.0
    assert bounds.y_max == 40.0
    assert metadata['world_name'] == 'random_terrain_ardupilot'

    grid_spec = grid_spec_from_manifest(manifest)
    assert grid_spec.rows == 257
    assert grid_spec.cols == 257
    assert grid_spec.x_min == -50.0
    assert grid_spec.x_max == 50.0
    assert grid_spec.y_min == -40.0
    assert grid_spec.y_max == 40.0
