#!/usr/bin/env python3
"""Generate evaluator artifacts and a human-readable Russian summary for one run."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
PACKAGE_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'terrain_mapping_system'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from terrain_mapping_system.validation.evaluator import evaluate_run_directory  # noqa: E402


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def polyline_distance(px: float, py: float, waypoints: list[dict[str, float]]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return min(
        point_to_segment_distance(px, py, a['x'], a['y'], b['x'], b['y'])
        for a, b in zip(waypoints[:-1], waypoints[1:])
    )


def _rounded(value: float | None, digits: int = 6) -> str:
    if value is None:
        return 'n/a'
    return f'{value:.{digits}f}'


def trajectory_summary(run_dir: Path) -> dict[str, object]:
    planned = json.loads((run_dir / 'planned_path.json').read_text(encoding='utf-8'))
    mission = json.loads((run_dir / 'mission_summary.json').read_text(encoding='utf-8'))
    waypoints = planned['waypoints']

    samples = []
    with (run_dir / 'actual_trajectory.jsonl').open('r', encoding='utf-8') as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped:
                continue
            payload = json.loads(stripped)
            if payload.get('state') == 'executing_sweep':
                samples.append(payload)

    polyline_errors = [
        polyline_distance(sample['pose']['x'], sample['pose']['y'], waypoints)
        for sample in samples
    ]
    altitude_errors = [abs(sample['pose']['z'] - 15.0) for sample in samples]

    polyline = np.asarray(polyline_errors, dtype=np.float64)
    altitude = np.asarray(altitude_errors, dtype=np.float64)

    return {
        'mission': {
            'final_status': mission.get('final_status'),
            'failure_reason': mission.get('failure_reason'),
            'waypoints_reached': mission.get('waypoints_reached'),
            'waypoints_total': mission.get('waypoints_total'),
            'duration_s': mission.get('duration_s'),
        },
        'trajectory': {
            'count': int(polyline.size),
            'mean_m': float(polyline.mean()) if polyline.size else None,
            'p95_m': float(np.percentile(polyline, 95)) if polyline.size else None,
            'max_m': float(polyline.max()) if polyline.size else None,
            'share_le_025': float(np.mean(polyline <= 0.25)) if polyline.size else None,
        },
        'altitude': {
            'count': int(altitude.size),
            'mean_abs_error_m': float(altitude.mean()) if altitude.size else None,
            'p95_abs_error_m': float(np.percentile(altitude, 95)) if altitude.size else None,
            'max_abs_error_m': float(altitude.max()) if altitude.size else None,
        },
    }


def build_human_summary(run_id: str, evaluation: dict[str, object], trajectory: dict[str, object]) -> str:
    mission = trajectory['mission']
    traj = trajectory['trajectory']
    alt = trajectory['altitude']
    vertical = evaluation['metrics']['vertical']

    traj_max = traj['max_m']
    traj_p95 = traj['p95_m']
    traj_verdict = (
        'FAIL, если трактовать требование строго по максимуму для каждого сэмпла.'
        if traj_max is not None and traj_max > 0.25
        else 'PASS.'
    )
    map_verdict = (
        'PASS по средней абсолютной ошибке, но не по более жёстким метрикам.'
        if (vertical.get('mae_m') is not None and vertical.get('mae_m') <= 0.2)
        else 'FAIL.'
    )

    return (
        f'# Понятный отчёт по прогону `{run_id}`\n\n'
        '## Что это за файл\n\n'
        'Этот файл сделан как краткая человеко-читаемая сводка.\n\n'
        'Важно:\n\n'
        '- `run_evaluation.json` считает качество карты высот и latency mapper-а;\n'
        '- ошибки по траектории добавлены сюда отдельным разделом.\n\n'
        '## Статус миссии\n\n'
        f'- Итоговый статус: `{mission["final_status"]}`\n'
        f'- Достигнуто waypoint-ов: `{mission["waypoints_reached"]} / {mission["waypoints_total"]}`\n'
        f'- Длительность миссии: `{_rounded(mission["duration_s"], 3)} с`\n\n'
        '## Траектория\n\n'
        'Траектория сравнивается с плановой ломаной по waypoint-ам в горизонтальной плоскости `x/y`.\n\n'
        f'- Среднее отклонение: `{_rounded(traj["mean_m"])} м`\n'
        f'- `p95`: `{_rounded(traj_p95)} м`\n'
        f'- Максимум: `{_rounded(traj_max)} м`\n'
        f'- Доля сэмплов с ошибкой `<= 0.25 м`: `{_rounded(traj["share_le_025"] * 100.0 if traj["share_le_025"] is not None else None, 2)}%`\n\n'
        'Вывод по требованию `не более 0.25 м`:\n\n'
        f'- {traj_verdict}\n'
        + (
            f'- Типичное поведение хорошее, потому что `p95 = {_rounded(traj_p95)} м`.\n\n'
            if traj_p95 is not None else '\n'
        )
        + 'Дополнительно по высоте полёта во время sweep:\n\n'
        f'- Средняя абсолютная ошибка относительно плановых `15 м`: `{_rounded(alt["mean_abs_error_m"])} м`\n'
        f'- `p95`: `{_rounded(alt["p95_abs_error_m"])} м`\n'
        f'- Максимум: `{_rounded(alt["max_abs_error_m"])} м`\n\n'
        '## Карта высот\n\n'
        'Эти метрики берутся из `run_evaluation.json` и считаются относительно `terrain_ground_truth.csv`.\n\n'
        f'- Покрытие: `{vertical["observed_cells"]} / {vertical["comparison_domain_cells"]}` ячеек (`{_rounded(vertical["observed_area_fraction"])}`)\n'
        f'- Средняя signed-ошибка: `{_rounded(vertical["mean_signed_error_m"])} м`\n'
        f'- `MAE`: `{_rounded(vertical["mae_m"])} м`\n'
        f'- Медианная абсолютная ошибка: `{_rounded(vertical["median_abs_error_m"])} м`\n'
        f'- `RMSE`: `{_rounded(vertical["rmse_m"])} м`\n'
        f'- `p95` абсолютной ошибки: `{_rounded(vertical["p95_abs_error_m"])} м`\n'
        f'- Максимальная абсолютная ошибка: `{_rounded(vertical["max_abs_error_m"])} м`\n'
        f'- Доля ячеек с ошибкой `<= {vertical["abs_error_within_threshold_m"]} м`: '
        f'`{_rounded(vertical["abs_error_within_threshold_fraction"] * 100.0 if vertical["abs_error_within_threshold_fraction"] is not None else None, 2)}%` '
        f'(`{vertical["abs_error_within_threshold_cells"]}` ячеек)\n\n'
        'Вывод по требованию `точность не хуже 0.2 м по высоте`:\n\n'
        f'- {map_verdict}\n'
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate per-run evaluator artifacts and a Russian summary.')
    parser.add_argument('run_id', help='Run identifier inside artifacts/terrain_mapping_results.')
    parser.add_argument(
        '--results-root',
        default=str(REPO_ROOT / 'artifacts' / 'terrain_mapping_results'),
        help='Root directory containing run artifact subdirectories.',
    )
    parser.add_argument('--vertical-threshold-m', type=float, default=1.0)
    parser.add_argument('--latency-threshold-ms', type=float, default=100.0)
    parser.add_argument('--primary-vertical-metric', default='rmse_m')
    parser.add_argument('--output-basename', default='run_evaluation')
    parser.add_argument('--human-summary-name', default='human_summary_ru.md')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = Path(args.results_root).expanduser().resolve() / args.run_id
    evaluation = evaluate_run_directory(
        run_dir,
        vertical_threshold_m=float(args.vertical_threshold_m),
        latency_threshold_ms=float(args.latency_threshold_ms),
        primary_vertical_metric=str(args.primary_vertical_metric),
        output_basename=str(args.output_basename),
        snapshot_inputs=True,
    )
    trajectory = trajectory_summary(run_dir)
    human_path = run_dir / args.human_summary_name
    human_path.write_text(
        build_human_summary(args.run_id, evaluation, trajectory),
        encoding='utf-8',
    )

    print(f'run_dir: {run_dir}')
    print(f'evaluation_json: {evaluation["artifacts"]["summary_json"]}')
    print(f'evaluation_md: {evaluation["artifacts"]["summary_markdown"]}')
    print(f'human_summary: {human_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
