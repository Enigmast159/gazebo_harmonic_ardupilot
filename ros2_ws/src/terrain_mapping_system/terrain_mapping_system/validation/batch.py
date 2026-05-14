"""Practical batch harness for terrain-mapping experiments."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shlex
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .common import file_fingerprint, read_json, read_yaml, software_context, utc_now_iso, write_json, write_text, write_yaml
from .evaluator import DEFAULT_OUTPUT_BASENAME, evaluate_run_directory


SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ROS_WS_ROOT = SOURCE_PACKAGE_ROOT.parents[1]
SIM_ROOT = ROS_WS_ROOT.parent
DEFAULT_RESULTS_ROOT = Path('/tmp/terrain_mapping_results')
DEFAULT_EXPERIMENTS_ROOT = Path('/tmp/terrain_mapping_experiments')
DEFAULT_MISSION_CONFIG = SOURCE_PACKAGE_ROOT / 'config' / 'mission_controller.yaml'
DEFAULT_MAPPER_CONFIG = SOURCE_PACKAGE_ROOT / 'config' / 'dem_mapper.yaml'
DEFAULT_GENERATOR = SIM_ROOT / 'scripts' / 'generate_heightmap_world.py'
DEFAULT_WORLD = SIM_ROOT / 'worlds' / 'random_terrain_ardupilot.sdf'
DEFAULT_MANIFEST = SIM_ROOT / 'models' / 'terrain_heightmap' / 'terrain_manifest.json'
DEFAULT_GROUND_TRUTH = SIM_ROOT / 'models' / 'terrain_heightmap' / 'terrain_ground_truth.csv'
DEFAULT_TERRAIN_STL = SIM_ROOT / 'models' / 'terrain_heightmap' / 'collision.stl'
DEFAULT_RUN_WITH_ARDUPILOT = SIM_ROOT / 'run_with_ardupilot.sh'
DEFAULT_RUN_SITL = SIM_ROOT / 'run_ardupilot_sitl.sh'
DEFAULT_FULL_STACK_LAUNCH = SOURCE_PACKAGE_ROOT / 'launch' / 'full_stack.launch.py'
DEFAULT_SIM_BRIDGE_LAUNCH = SOURCE_PACKAGE_ROOT / 'launch' / 'sim_bridge.launch.py'
DEFAULT_MISSION_CONTROL_LAUNCH = SOURCE_PACKAGE_ROOT / 'launch' / 'mission_control.launch.py'
DEFAULT_ROS_SETUP = Path('/opt/ros/humble/setup.bash')
FINAL_MISSION_STATES = {'completed', 'failed'}


@dataclass
class RunPlan:
    run_id: str
    terrain_args: Dict[str, Any]
    mission_parameters: Dict[str, Any]
    mapping_parameters: Dict[str, Any]
    mission_dry_run: bool


@dataclass
class ManagedProcess:
    process: subprocess.Popen
    log_handle: Any
    log_path: Path
    label: str

    def close_log(self) -> None:
        try:
            self.log_handle.close()
        except Exception:
            pass


def _comma_list(raw: str) -> List[str]:
    return [item.strip() for item in raw.split(',') if item.strip()]


def _sanitize_token(value: str) -> str:
    sanitized = ''.join(ch if ch.isalnum() or ch in {'-', '_'} else '_' for ch in value.strip())
    return sanitized or 'run'


def _build_run_id(prefix: str, seed: Optional[int], index: int) -> str:
    if seed is None:
        return f'{_sanitize_token(prefix)}_{index:02d}'
    return f'{_sanitize_token(prefix)}_seed{int(seed):04d}'


def _format_shell_command(parts: Sequence[str]) -> str:
    return ' '.join(shlex.quote(str(part)) for part in parts)


def _load_plan_file(path: Path) -> List[Dict[str, Any]]:
    payload = read_json(path)
    runs = payload.get('runs')
    if not isinstance(runs, list) or not runs:
        raise ValueError(f'plan file must contain a non-empty "runs" list: {path}')
    return runs


def _generator_args_from_namespace(args: argparse.Namespace) -> Dict[str, Any]:
    return {
        'size': args.terrain_size,
        'scale': args.terrain_scale,
        'octaves': args.terrain_octaves,
        'persistence': args.terrain_persistence,
        'lacunarity': args.terrain_lacunarity,
        'smooth_passes': args.terrain_smooth_passes,
        'terrain_z': args.terrain_z,
        'tree_count': args.tree_count,
        'tree_exclusion_radius': args.tree_exclusion_radius,
        'world': 'random_terrain_ardupilot',
    }


def _build_run_plans(args: argparse.Namespace) -> List[RunPlan]:
    run_plans: List[RunPlan] = []
    if args.plan_file:
        for index, item in enumerate(_load_plan_file(Path(args.plan_file)), start=1):
            terrain_args = dict(_generator_args_from_namespace(args))
            terrain_args.update(item.get('terrain', {}))
            seed = terrain_args.get('seed')
            run_id = item.get('run_id') or _build_run_id(item.get('name') or args.run_prefix, seed, index)
            run_plans.append(
                RunPlan(
                    run_id=str(run_id),
                    terrain_args=terrain_args,
                    mission_parameters=dict(item.get('mission', {}).get('parameters', {})),
                    mapping_parameters=dict(item.get('mapping', {}).get('parameters', {})),
                    mission_dry_run=bool(item.get('mission', {}).get('dry_run', args.mission_dry_run)),
                )
            )
        return run_plans

    seeds: List[Optional[int]] = [int(seed) for seed in args.seed]
    if args.seed_start is not None and args.seed_count is not None:
        seeds.extend(range(int(args.seed_start), int(args.seed_start) + int(args.seed_count)))
    if not seeds and args.run_id:
        for run_id in args.run_id:
            run_plans.append(
                RunPlan(
                    run_id=str(run_id),
                    terrain_args={},
                    mission_parameters={},
                    mapping_parameters={},
                    mission_dry_run=bool(args.mission_dry_run),
                )
            )
        return run_plans
    if not seeds:
        seeds = [None]

    generator_defaults = _generator_args_from_namespace(args)
    for index, seed in enumerate(seeds, start=1):
        terrain_args = dict(generator_defaults)
        if seed is not None:
            terrain_args['seed'] = int(seed)
        run_plans.append(
            RunPlan(
                run_id=_build_run_id(args.run_prefix, seed, index),
                terrain_args=terrain_args,
                mission_parameters={},
                mapping_parameters={},
                mission_dry_run=bool(args.mission_dry_run),
            )
        )
    return run_plans


def _copy_file_if_exists(source: Path, destination: Path) -> Optional[Path]:
    if not source.is_file():
        return None
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return destination


def _snapshot_active_terrain_inputs(run_dir: Path) -> Dict[str, Optional[str]]:
    snapshot_manifest = run_dir / 'terrain_manifest.snapshot.json'
    snapshot_truth = run_dir / 'terrain_ground_truth.snapshot.csv'
    snapshot_stl = run_dir / 'terrain_model.snapshot.stl'
    snapshot_world = run_dir / 'world.snapshot.sdf'
    snapshot_model_sdf = run_dir / 'terrain_model.snapshot.sdf'
    snapshot_model_config = run_dir / 'terrain_model.snapshot.config'

    _copy_file_if_exists(DEFAULT_GROUND_TRUTH, snapshot_truth)
    _copy_file_if_exists(DEFAULT_TERRAIN_STL, snapshot_stl)
    _copy_file_if_exists(DEFAULT_WORLD, snapshot_world)

    manifest_payload = read_json(DEFAULT_MANIFEST)
    canonical = dict(manifest_payload.get('canonical_file_paths', {}))
    canonical['manifest'] = str(snapshot_manifest)
    if snapshot_truth.is_file():
        canonical['ground_truth_csv'] = str(snapshot_truth)
    if snapshot_stl.is_file():
        canonical['stl_mesh'] = str(snapshot_stl)

    model_sdf = Path(str(canonical.get('model_sdf', '')))
    model_config = Path(str(canonical.get('model_config', '')))
    copied_model_sdf = _copy_file_if_exists(model_sdf, snapshot_model_sdf) if model_sdf.is_file() else None
    copied_model_config = _copy_file_if_exists(model_config, snapshot_model_config) if model_config.is_file() else None
    if copied_model_sdf is not None:
        canonical['model_sdf'] = str(copied_model_sdf)
    if copied_model_config is not None:
        canonical['model_config'] = str(copied_model_config)

    manifest_payload['canonical_file_paths'] = canonical
    write_json(snapshot_manifest, manifest_payload)
    return {
        'manifest_snapshot': str(snapshot_manifest),
        'ground_truth_snapshot': str(snapshot_truth) if snapshot_truth.is_file() else None,
        'terrain_mesh_snapshot': str(snapshot_stl) if snapshot_stl.is_file() else None,
        'world_snapshot': str(snapshot_world) if snapshot_world.is_file() else None,
    }


def _resolved_ros_parameters(base_path: Path, overrides: Dict[str, Any]) -> Dict[str, Any]:
    payload = read_yaml(base_path)
    root = payload.get('/**')
    if not isinstance(root, dict):
        raise ValueError(f'expected "/**" root in ROS parameters file: {base_path}')
    parameters = root.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValueError(f'expected ros__parameters block in {base_path}')
    merged = dict(parameters)
    merged.update(overrides)
    return {'/**': {'ros__parameters': merged}}


def _materialize_run_inputs(
    plan: RunPlan,
    *,
    run_dir: Path,
    results_root: Path,
    mission_config_source: Path,
    mapper_config_source: Path,
    mavlink_connection_string: str,
    ros_setup: Path,
    workspace_setup: Path,
    dry_run: bool,
    startup_delays_s: Dict[str, float],
) -> Dict[str, Any]:
    run_dir.mkdir(parents=True, exist_ok=True)
    terrain_snapshot = _snapshot_active_terrain_inputs(run_dir) if not dry_run else {
        'manifest_snapshot': str(run_dir / 'terrain_manifest.snapshot.json'),
        'ground_truth_snapshot': str(run_dir / 'terrain_ground_truth.snapshot.csv'),
        'terrain_mesh_snapshot': str(run_dir / 'terrain_model.snapshot.stl'),
        'world_snapshot': str(run_dir / 'world.snapshot.sdf'),
    }

    mission_config_path = run_dir / 'mission_config.snapshot.yaml'
    mapper_config_path = run_dir / 'dem_mapper_config.snapshot.yaml'

    mission_overrides = dict(plan.mission_parameters)
    mission_overrides.update(
        {
            'run_id': plan.run_id,
            'results_root': str(results_root),
            'manifest_path': terrain_snapshot['manifest_snapshot'],
            'use_terrain_manifest': True,
            'dry_run': plan.mission_dry_run,
            'mavlink.connection_string': mavlink_connection_string,
        }
    )
    mapping_overrides = dict(plan.mapping_parameters)
    mapping_overrides.update(
        {
            'run_id': plan.run_id,
            'results_root': str(results_root),
            'manifest_path': terrain_snapshot['manifest_snapshot'],
            'use_terrain_manifest': True,
        }
    )

    write_yaml(mission_config_path, _resolved_ros_parameters(mission_config_source, mission_overrides))
    write_yaml(mapper_config_path, _resolved_ros_parameters(mapper_config_source, mapping_overrides))

    generator_command = _build_generator_command(plan.terrain_args)
    gazebo_command = [
        '/bin/bash',
        '-lc',
        (
            'unset DISPLAY WAYLAND_DISPLAY QT_QPA_PLATFORM && '
            f'cd {shlex.quote(str(SIM_ROOT))} && '
            f'{shlex.quote(str(DEFAULT_RUN_WITH_ARDUPILOT))}'
        ),
    ]
    sitl_command = [
        '/bin/bash',
        '-lc',
        (
            'unset DISPLAY WAYLAND_DISPLAY QT_QPA_PLATFORM && '
            'export SITL_WITH_CONSOLE=0 && '
            f'cd {shlex.quote(str(SIM_ROOT))} && '
            f'{shlex.quote(str(DEFAULT_RUN_SITL))} --no-mavproxy --no-rebuild'
        ),
    ]
    ros_command = [
        '/bin/bash',
        '-lc',
        (
            'export ROS_LOG_DIR=/tmp/ros_logs && mkdir -p "$ROS_LOG_DIR" && '
            f'source {shlex.quote(str(ros_setup))} && '
            f'cd {shlex.quote(str(ROS_WS_ROOT))} && '
            f'source {shlex.quote(str(workspace_setup))} && '
            'ros2 launch terrain_mapping_system full_stack.launch.py '
            f'run_id:={shlex.quote(plan.run_id)} '
            f'results_root:={shlex.quote(str(results_root))} '
            f'mission_config_file:={shlex.quote(str(mission_config_path))} '
            f'mapper_config_file:={shlex.quote(str(mapper_config_path))} '
            f'mavlink_connection_string:={shlex.quote(mavlink_connection_string)} '
            f'dry_run:={"true" if plan.mission_dry_run else "false"}'
        ),
    ]

    request = {
        'schema_version': 1,
        'generated_at': utc_now_iso(),
        'run_id': plan.run_id,
        'run_dir': str(run_dir),
        'terrain': {
            'seed': plan.terrain_args.get('seed'),
            'generator_args': plan.terrain_args,
            **terrain_snapshot,
        },
        'mission': {
            'dry_run': plan.mission_dry_run,
            'config_file': str(mission_config_path),
            'parameter_overrides': plan.mission_parameters,
            'mavlink_connection_string': mavlink_connection_string,
        },
        'mapping': {
            'config_file': str(mapper_config_path),
            'parameter_overrides': plan.mapping_parameters,
        },
        'commands': {
            'terrain_generator': _format_shell_command(generator_command),
            'gazebo': _format_shell_command(gazebo_command),
            'sitl': _format_shell_command(sitl_command),
            'ros_launch': _format_shell_command(ros_command),
        },
        'command_argv': {
            'terrain_generator': generator_command,
            'gazebo': gazebo_command,
            'sitl': sitl_command,
            'ros_launch': ros_command,
        },
        'startup_delays_s': startup_delays_s,
        'software': {
            **software_context(),
            'source_files': {
                'generator_script': file_fingerprint(DEFAULT_GENERATOR),
                'run_with_ardupilot': file_fingerprint(DEFAULT_RUN_WITH_ARDUPILOT),
                'run_ardupilot_sitl': file_fingerprint(DEFAULT_RUN_SITL),
                'full_stack_launch': file_fingerprint(DEFAULT_FULL_STACK_LAUNCH),
                'sim_bridge_launch': file_fingerprint(DEFAULT_SIM_BRIDGE_LAUNCH),
                'mission_control_launch': file_fingerprint(DEFAULT_MISSION_CONTROL_LAUNCH),
                'mission_config_source': file_fingerprint(mission_config_source),
                'mapper_config_source': file_fingerprint(mapper_config_source),
                'ros_setup': file_fingerprint(ros_setup),
                'workspace_setup': file_fingerprint(workspace_setup),
            },
        },
        'logs': {
            'gazebo': str(run_dir / 'gazebo.log'),
            'sitl': str(run_dir / 'sitl.log'),
            'ros_launch': str(run_dir / 'ros_launch.log'),
        },
    }
    write_json(run_dir / 'experiment_request.json', request)
    return request


def _build_generator_command(terrain_args: Dict[str, Any]) -> List[str]:
    command = ['python3', str(DEFAULT_GENERATOR)]
    ordered_flags = [
        ('seed', '--seed'),
        ('size', '--size'),
        ('scale', '--scale'),
        ('octaves', '--octaves'),
        ('persistence', '--persistence'),
        ('lacunarity', '--lacunarity'),
        ('smooth_passes', '--smooth-passes'),
        ('terrain_z', '--terrain-z'),
        ('tree_count', '--tree-count'),
        ('tree_exclusion_radius', '--tree-exclusion-radius'),
        ('world', '--world'),
    ]
    for key, flag in ordered_flags:
        value = terrain_args.get(key)
        if value is None:
            continue
        command.extend([flag, str(value)])
    return command


def _start_process(command: Sequence[str], *, log_path: Path, cwd: Path) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        list(command),
        cwd=str(cwd),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return ManagedProcess(process=process, log_handle=log_handle, log_path=log_path, label=log_path.stem)


def _stop_process(process: ManagedProcess, *, grace_s: float = 10.0) -> None:
    if process.process.poll() is not None:
        process.close_log()
        return
    try:
        os.killpg(process.process.pid, signal.SIGINT)
    except ProcessLookupError:
        process.close_log()
        return
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if process.process.poll() is not None:
            process.close_log()
            return
        time.sleep(0.25)
    try:
        os.killpg(process.process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.process.wait(timeout=5.0)
    process.close_log()


def _wait_for_final_mission_summary(run_dir: Path, *, timeout_s: float) -> Dict[str, Any]:
    mission_summary_path = run_dir / 'mission_summary.json'
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if mission_summary_path.is_file():
            summary = read_json(mission_summary_path)
            if str(summary.get('final_status', '')).lower() in FINAL_MISSION_STATES:
                return summary
        time.sleep(2.0)
    raise TimeoutError(f'mission summary did not reach a final state within {timeout_s:.1f}s')


def _planned_commands_text(run_plans: Iterable[RunPlan], *, experiment_root: Path, results_root: Path, mission_dry_run: bool) -> str:
    lines = [
        '#!/bin/bash',
        '# Planned experiment commands',
        f'# Generated at {utc_now_iso()}',
        '',
    ]
    for plan in run_plans:
        run_dir = results_root / plan.run_id
        lines.append(f'# Run {plan.run_id}')
        lines.append(_format_shell_command(_build_generator_command(plan.terrain_args)))
        lines.append(
            '# ros2 launch command will use '
            f'{run_dir / "mission_config.snapshot.yaml"} and {run_dir / "dem_mapper_config.snapshot.yaml"} '
            f'(mission dry-run={mission_dry_run or plan.mission_dry_run})'
        )
        lines.append('')
    return '\n'.join(lines) + '\n'


def _metric_row_from_summary(summary: Dict[str, Any]) -> Dict[str, Any]:
    vertical = summary.get('metrics', {}).get('vertical', {})
    latency = summary.get('metrics', {}).get('latency', {})
    acceptance = summary.get('acceptance', {})
    ground_truth = summary.get('ground_truth', {})
    mission = summary.get('reproducibility', {}).get('mission', {})
    return {
        'run_id': summary.get('run_id'),
        'seed': ground_truth.get('seed'),
        'ground_truth_source_kind': ground_truth.get('source_kind'),
        'mae_m': vertical.get('mae_m'),
        'rmse_m': vertical.get('rmse_m'),
        'p95_abs_error_m': vertical.get('p95_abs_error_m'),
        'max_abs_error_m': vertical.get('max_abs_error_m'),
        'observed_area_fraction': vertical.get('observed_area_fraction'),
        'comparison_domain_cells': vertical.get('comparison_domain_cells'),
        'observed_cells': vertical.get('observed_cells'),
        'latency_frame_count': latency.get('frame_count'),
        'latency_mean_processing_ms': latency.get('mean_processing_ms'),
        'latency_p95_processing_ms': latency.get('p95_processing_ms'),
        'latency_max_processing_ms': latency.get('max_processing_ms'),
        'vertical_accuracy_pass': acceptance.get('vertical_accuracy_pass'),
        'processing_time_pass': acceptance.get('processing_time_pass'),
        'overall_pass': acceptance.get('overall_pass'),
        'mission_planner': json.dumps(mission.get('planner'), sort_keys=True),
    }


def _write_aggregate_outputs(experiment_root: Path, summaries: List[Dict[str, Any]]) -> Dict[str, Any]:
    rows = [_metric_row_from_summary(summary) for summary in summaries]
    csv_path = experiment_root / 'aggregate_metrics.csv'
    json_path = experiment_root / 'aggregate_metrics.json'
    md_path = experiment_root / 'aggregate_summary.md'

    fieldnames = [
        'run_id',
        'seed',
        'ground_truth_source_kind',
        'mae_m',
        'rmse_m',
        'p95_abs_error_m',
        'max_abs_error_m',
        'observed_area_fraction',
        'comparison_domain_cells',
        'observed_cells',
        'latency_frame_count',
        'latency_mean_processing_ms',
        'latency_p95_processing_ms',
        'latency_max_processing_ms',
        'vertical_accuracy_pass',
        'processing_time_pass',
        'overall_pass',
        'mission_planner',
    ]
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    def _numeric(values: Iterable[Any]) -> List[float]:
        output: List[float] = []
        for value in values:
            if value is None or value == '':
                continue
            output.append(float(value))
        return output

    rmse_values = _numeric(row['rmse_m'] for row in rows)
    latency_p95_values = _numeric(row['latency_p95_processing_ms'] for row in rows)
    aggregate = {
        'schema_version': 1,
        'generated_at': utc_now_iso(),
        'run_count': len(rows),
        'pass_count': sum(1 for row in rows if row['overall_pass']),
        'rows': rows,
        'summary': {
            'rmse_m_mean': round(sum(rmse_values) / len(rmse_values), 6) if rmse_values else None,
            'rmse_m_max': round(max(rmse_values), 6) if rmse_values else None,
            'latency_p95_processing_ms_mean': round(sum(latency_p95_values) / len(latency_p95_values), 6) if latency_p95_values else None,
            'latency_p95_processing_ms_max': round(max(latency_p95_values), 6) if latency_p95_values else None,
        },
        'artifacts': {
            'aggregate_metrics_csv': str(csv_path),
            'aggregate_metrics_json': str(json_path),
            'aggregate_summary_markdown': str(md_path),
        },
    }
    write_json(json_path, aggregate)

    lines = [
        '# Aggregate Experiment Summary',
        '',
        f'- Runs evaluated: {aggregate["run_count"]}',
        f'- Overall pass count: {aggregate["pass_count"]}',
        f'- Mean RMSE (m): {aggregate["summary"]["rmse_m_mean"]}',
        f'- Max RMSE (m): {aggregate["summary"]["rmse_m_max"]}',
        f'- Mean latency p95 (ms): {aggregate["summary"]["latency_p95_processing_ms_mean"]}',
        f'- Max latency p95 (ms): {aggregate["summary"]["latency_p95_processing_ms_max"]}',
        '',
        '| run_id | seed | rmse_m | observed_area_fraction | latency_p95_ms | overall_pass |',
        '|---|---:|---:|---:|---:|---|',
    ]
    for row in rows:
        lines.append(
            f'| {row["run_id"]} | {row["seed"]} | {row["rmse_m"]} | {row["observed_area_fraction"]} | {row["latency_p95_processing_ms"]} | {row["overall_pass"]} |'
        )
    write_text(md_path, '\n'.join(lines) + '\n')
    return aggregate


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Run or plan batch terrain-mapping experiments.')
    parser.add_argument('--experiment-name', default='terrain_validation_batch', help='Experiment name used for output organization.')
    parser.add_argument('--experiment-root', default=None, help='Directory for aggregate experiment artifacts.')
    parser.add_argument('--results-root', default=str(DEFAULT_RESULTS_ROOT), help='Run-artifact root used by the runtime stack.')
    parser.add_argument('--run-prefix', default='terrain_eval', help='Prefix used to build run_id values when a plan file is not supplied.')
    parser.add_argument('--run-id', action='append', default=[], help='Existing run_id to evaluate without generating new terrain.')
    parser.add_argument('--plan-file', default=None, help='Optional JSON plan file with explicit per-run terrain and parameter overrides.')
    parser.add_argument('--seed', action='append', default=[], help='Terrain seed for one run. Repeat to queue multiple runs.')
    parser.add_argument('--seed-start', type=int, default=None, help='First terrain seed in an auto-generated range.')
    parser.add_argument('--seed-count', type=int, default=None, help='Number of terrain seeds in the auto-generated range.')
    parser.add_argument('--stages', default='generate,launch,evaluate,aggregate', help='Comma-separated subset of generate, launch, evaluate, aggregate.')
    parser.add_argument('--dry-run', action='store_true', help='Write the experiment plan and planned commands without executing the full stack.')
    parser.add_argument('--mission-dry-run', action='store_true', help='Pass dry_run:=true to the mission controller.')
    parser.add_argument('--vertical-threshold-m', type=float, default=1.0, help='Primary vertical-accuracy threshold in meters.')
    parser.add_argument('--latency-threshold-ms', type=float, default=100.0, help='Conservative per-frame processing-time threshold in milliseconds.')
    parser.add_argument('--primary-vertical-metric', default='rmse_m', choices=('mae_m', 'rmse_m', 'p95_abs_error_m', 'max_abs_error_m'))
    parser.add_argument('--min-observed-area-fraction', type=float, default=None, help='Optional minimum observed-area fraction gate.')
    parser.add_argument('--output-basename', default=DEFAULT_OUTPUT_BASENAME, help='Basename used for per-run evaluation artifacts.')
    parser.add_argument('--mavlink-connection-string', default='tcp:127.0.0.1:5760', help='pymavlink connection string.')
    parser.add_argument('--ros-setup', default=str(DEFAULT_ROS_SETUP), help='ROS environment setup script.')
    parser.add_argument('--workspace-setup', default=str(ROS_WS_ROOT / 'install' / 'setup.bash'), help='Workspace setup script for ros2 launch.')
    parser.add_argument('--mission-config-source', default=str(DEFAULT_MISSION_CONFIG), help='Base mission-controller parameter file.')
    parser.add_argument('--mapper-config-source', default=str(DEFAULT_MAPPER_CONFIG), help='Base DEM-mapper parameter file.')
    parser.add_argument('--sitl-startup-delay-s', type=float, default=8.0, help='Delay after starting SITL before continuing.')
    parser.add_argument('--gazebo-startup-delay-s', type=float, default=8.0, help='Delay after starting Gazebo before launching ROS.')
    parser.add_argument('--flush-delay-s', type=float, default=5.0, help='Grace delay before shutting the stack down after a final mission summary appears.')
    parser.add_argument('--mission-timeout-s', type=float, default=1200.0, help='Maximum wall-clock wait for one launched mission.')
    parser.add_argument('--terrain-size', type=int, default=257)
    parser.add_argument('--terrain-scale', type=float, default=28.0)
    parser.add_argument('--terrain-octaves', type=int, default=4)
    parser.add_argument('--terrain-persistence', type=float, default=0.42)
    parser.add_argument('--terrain-lacunarity', type=float, default=2.0)
    parser.add_argument('--terrain-smooth-passes', type=int, default=3)
    parser.add_argument('--terrain-mesh-downsample-skip', type=int, default=2)
    parser.add_argument('--terrain-z', type=float, default=8.0)
    parser.add_argument('--tree-count', type=int, default=18)
    parser.add_argument('--tree-exclusion-radius', type=float, default=10.0)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parse_args(argv)
    stages = set(_comma_list(args.stages))
    run_plans = _build_run_plans(args)
    if not run_plans:
        raise SystemExit('no runs planned')

    results_root = Path(args.results_root).expanduser().resolve()
    experiment_root = Path(args.experiment_root).expanduser().resolve() if args.experiment_root else (
        DEFAULT_EXPERIMENTS_ROOT / _sanitize_token(args.experiment_name)
    )
    experiment_root.mkdir(parents=True, exist_ok=True)

    startup_delays_s = {
        'sitl': float(args.sitl_startup_delay_s),
        'gazebo': float(args.gazebo_startup_delay_s),
        'flush': float(args.flush_delay_s),
    }

    plan_payload = {
        'schema_version': 1,
        'generated_at': utc_now_iso(),
        'experiment_name': args.experiment_name,
        'dry_run': bool(args.dry_run),
        'stages': sorted(stages),
        'results_root': str(results_root),
        'run_count': len(run_plans),
        'runs': [
            {
                'run_id': plan.run_id,
                'terrain_args': plan.terrain_args,
                'mission_parameters': plan.mission_parameters,
                'mapping_parameters': plan.mapping_parameters,
                'mission_dry_run': plan.mission_dry_run,
            }
            for plan in run_plans
        ],
    }
    write_json(experiment_root / 'experiment_plan.json', plan_payload)
    write_text(
        experiment_root / 'planned_commands.sh',
        _planned_commands_text(run_plans, experiment_root=experiment_root, results_root=results_root, mission_dry_run=args.mission_dry_run),
    )

    mission_config_source = Path(args.mission_config_source).expanduser().resolve()
    mapper_config_source = Path(args.mapper_config_source).expanduser().resolve()
    ros_setup = Path(args.ros_setup).expanduser().resolve()
    workspace_setup = Path(args.workspace_setup).expanduser().resolve()

    summaries: List[Dict[str, Any]] = []
    for plan in run_plans:
        run_dir = results_root / plan.run_id
        needs_materialization = args.dry_run or 'generate' in stages or 'launch' in stages
        request: Optional[Dict[str, Any]] = None
        if needs_materialization:
            request = _materialize_run_inputs(
                plan,
                run_dir=run_dir,
                results_root=results_root,
                mission_config_source=mission_config_source,
                mapper_config_source=mapper_config_source,
                mavlink_connection_string=args.mavlink_connection_string,
                ros_setup=ros_setup,
                workspace_setup=workspace_setup,
                dry_run=args.dry_run,
                startup_delays_s=startup_delays_s,
            )

        if args.dry_run:
            assert request is not None
            print(f'[dry-run] {plan.run_id}')
            print(f'  generator: {request["commands"]["terrain_generator"]}')
            print(f'  gazebo:    {request["commands"]["gazebo"]}')
            print(f'  sitl:      {request["commands"]["sitl"]}')
            print(f'  ros:       {request["commands"]["ros_launch"]}')
            continue

        if 'generate' in stages and plan.terrain_args:
            subprocess.run(_build_generator_command(plan.terrain_args), cwd=str(SIM_ROOT), check=True)
            request = _materialize_run_inputs(
                plan,
                run_dir=run_dir,
                results_root=results_root,
                mission_config_source=mission_config_source,
                mapper_config_source=mapper_config_source,
                mavlink_connection_string=args.mavlink_connection_string,
                ros_setup=ros_setup,
                workspace_setup=workspace_setup,
                dry_run=False,
                startup_delays_s=startup_delays_s,
            )

        if 'launch' in stages:
            if request is None:
                raise RuntimeError('launch stage requires a materialized run request')
            gazebo_process = _start_process(
                request['command_argv']['gazebo'],
                log_path=Path(request['logs']['gazebo']),
                cwd=SIM_ROOT,
            )
            sitl_process = _start_process(
                request['command_argv']['sitl'],
                log_path=Path(request['logs']['sitl']),
                cwd=SIM_ROOT,
            )
            ros_process: Optional[ManagedProcess] = None
            try:
                time.sleep(float(startup_delays_s['sitl']))
                time.sleep(float(startup_delays_s['gazebo']))
                ros_process = _start_process(
                    request['command_argv']['ros_launch'],
                    log_path=Path(request['logs']['ros_launch']),
                    cwd=SIM_ROOT,
                )
                _wait_for_final_mission_summary(run_dir, timeout_s=float(args.mission_timeout_s))
                time.sleep(float(startup_delays_s['flush']))
            finally:
                if ros_process is not None:
                    _stop_process(ros_process)
                _stop_process(gazebo_process)
                _stop_process(sitl_process)

        if 'evaluate' in stages:
            summaries.append(
                evaluate_run_directory(
                    run_dir,
                    vertical_threshold_m=float(args.vertical_threshold_m),
                    latency_threshold_ms=float(args.latency_threshold_ms),
                    primary_vertical_metric=args.primary_vertical_metric,
                    min_observed_area_fraction=args.min_observed_area_fraction,
                    output_basename=args.output_basename,
                    snapshot_inputs=True,
                )
            )

    if args.dry_run:
        print(f'[dry-run] experiment plan: {experiment_root / "experiment_plan.json"}')
        print(f'[dry-run] planned commands: {experiment_root / "planned_commands.sh"}')
        return

    if 'aggregate' in stages:
        if not summaries:
            for plan in run_plans:
                summary_path = results_root / plan.run_id / f'{args.output_basename}.json'
                if summary_path.is_file():
                    summaries.append(read_json(summary_path))
        aggregate = _write_aggregate_outputs(experiment_root, summaries)
        print(
            f'aggregate: runs={aggregate["run_count"]} '
            f'pass_count={aggregate["pass_count"]} '
            f'csv={aggregate["artifacts"]["aggregate_metrics_csv"]}'
        )


if __name__ == '__main__':
    main()
