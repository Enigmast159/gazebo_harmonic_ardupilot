from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


def test_canonical_world_uses_heightmap_terrain():
    world_path = REPO_ROOT / 'worlds' / 'random_terrain_ardupilot.sdf'
    world_text = world_path.read_text(encoding='utf-8')

    assert 'model://terrain_heightmap' in world_text
    assert 'model://terrain_mesh' not in world_text


def test_canonical_world_declares_sensor_systems():
    world_path = REPO_ROOT / 'worlds' / 'random_terrain_ardupilot.sdf'
    world_text = world_path.read_text(encoding='utf-8')

    assert 'gz-sim-physics-system' in world_text
    assert 'gz-sim-user-commands-system' in world_text
    assert 'gz-sim-scene-broadcaster-system' in world_text
    assert 'gz-sim-sensors-system' in world_text
    assert '<render_engine>ogre2</render_engine>' in world_text


def test_heightmap_generator_defaults_to_canonical_world():
    script_path = REPO_ROOT / 'scripts' / 'generate_heightmap_world.py'
    script_text = script_path.read_text(encoding='utf-8')

    assert 'parser.add_argument("--world", default="random_terrain_ardupilot")' in script_text
    assert 'gz-sim-sensors-system' in script_text


def test_sitl_script_defaults_match_ros_autonomy_flow():
    script_path = REPO_ROOT / 'run_ardupilot_sitl.sh'
    script_text = script_path.read_text(encoding='utf-8')

    assert 'SITL_USE_EXTRA_PARAMS="${SITL_USE_EXTRA_PARAMS:-1}"' in script_text
    assert 'SITL_USE_MAVPROXY="${SITL_USE_MAVPROXY:-0}"' in script_text
    assert 'ARGS+=(--no-mavproxy)' in script_text
    assert 'ARGS+=(--no-rebuild)' in script_text


def test_bridge_defaults_use_lidar_pointcloud_subtopic():
    defaults_path = REPO_ROOT / 'ros2_ws' / 'src' / 'terrain_mapping_system' / 'config' / 'bridge_defaults.yaml'
    defaults_text = defaults_path.read_text(encoding='utf-8')

    assert 'lidar_points: /lidar/points/points' in defaults_text


def test_mapper_config_gates_dem_integration_to_sweep_state():
    config_path = REPO_ROOT / 'ros2_ws' / 'src' / 'terrain_mapping_system' / 'config' / 'dem_mapper.yaml'
    config_text = config_path.read_text(encoding='utf-8')

    assert 'integrate_only_during_sweep: true' in config_text
    assert 'mission_state_topic: /terrain_mapping/mission/state' in config_text


def test_mission_controller_config_publishes_state_topic():
    config_path = REPO_ROOT / 'ros2_ws' / 'src' / 'terrain_mapping_system' / 'config' / 'mission_controller.yaml'
    config_text = config_path.read_text(encoding='utf-8')

    assert 'state_topic: /terrain_mapping/mission/state' in config_text
