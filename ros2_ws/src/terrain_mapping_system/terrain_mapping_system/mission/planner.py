"""Deterministic rectangular lawnmower planner in map/ENU coordinates."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, List


@dataclass(frozen=True)
class RectBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def inset(self, margin_m: float) -> "RectBounds":
        if margin_m < 0.0:
            raise ValueError('boundary margin must be non-negative')
        x_min = self.x_min + margin_m
        x_max = self.x_max - margin_m
        y_min = self.y_min + margin_m
        y_max = self.y_max - margin_m
        if x_min >= x_max or y_min >= y_max:
            raise ValueError('boundary margin collapsed the sweep bounds')
        return RectBounds(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max)

    def as_dict(self) -> Dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class PlannerConfig:
    mapping_altitude_m: float
    lane_spacing_m: float
    waypoint_tolerance_m: float
    sweep_direction: str = 'x'
    boundary_margin_m: float = 0.0
    return_home_x_m: float | None = None
    return_home_y_m: float | None = None
    return_home_altitude_m: float | None = None

    def normalized_direction(self) -> str:
        direction = self.sweep_direction.lower().strip()
        if direction not in {'x', 'y'}:
            raise ValueError("sweep_direction must be 'x' or 'y'")
        return direction


def _axis_samples(axis_min: float, axis_max: float, lane_spacing_m: float) -> List[float]:
    if lane_spacing_m <= 0.0:
        raise ValueError('lane spacing must be positive')
    if axis_max <= axis_min:
        raise ValueError('invalid sweep axis bounds')

    samples = [axis_min]
    cursor = axis_min
    while cursor + lane_spacing_m < axis_max:
        cursor += lane_spacing_m
        samples.append(round(cursor, 6))
    if samples[-1] != axis_max:
        samples.append(axis_max)
    return samples


def plan_lawnmower_path(bounds: RectBounds, config: PlannerConfig) -> List[Dict[str, float]]:
    """Build a snake-pattern sweep over rectangular ENU bounds."""
    sweep_bounds = bounds.inset(config.boundary_margin_m)
    direction = config.normalized_direction()

    if direction == 'x':
        lane_positions = _axis_samples(sweep_bounds.y_min, sweep_bounds.y_max, config.lane_spacing_m)
        waypoints: List[Dict[str, float]] = []
        travel_forward = True
        for lane_y in lane_positions:
            x_start = sweep_bounds.x_min if travel_forward else sweep_bounds.x_max
            x_end = sweep_bounds.x_max if travel_forward else sweep_bounds.x_min
            waypoints.append({'x': x_start, 'y': lane_y, 'z': config.mapping_altitude_m})
            waypoints.append({'x': x_end, 'y': lane_y, 'z': config.mapping_altitude_m})
            travel_forward = not travel_forward
        return _append_return_home(_deduplicate_consecutive(waypoints), config)

    lane_positions = _axis_samples(sweep_bounds.x_min, sweep_bounds.x_max, config.lane_spacing_m)
    waypoints = []
    travel_forward = True
    for lane_x in lane_positions:
        y_start = sweep_bounds.y_min if travel_forward else sweep_bounds.y_max
        y_end = sweep_bounds.y_max if travel_forward else sweep_bounds.y_min
        waypoints.append({'x': lane_x, 'y': y_start, 'z': config.mapping_altitude_m})
        waypoints.append({'x': lane_x, 'y': y_end, 'z': config.mapping_altitude_m})
        travel_forward = not travel_forward
    return _append_return_home(_deduplicate_consecutive(waypoints), config)


def _deduplicate_consecutive(waypoints: List[Dict[str, float]]) -> List[Dict[str, float]]:
    if not waypoints:
        return []
    deduplicated = [waypoints[0]]
    for waypoint in waypoints[1:]:
        previous = deduplicated[-1]
        if (
            previous['x'] == waypoint['x']
            and previous['y'] == waypoint['y']
            and previous['z'] == waypoint['z']
        ):
            continue
        deduplicated.append(waypoint)
    return deduplicated


def _append_return_home(waypoints: List[Dict[str, float]], config: PlannerConfig) -> List[Dict[str, float]]:
    if config.return_home_x_m is None or config.return_home_y_m is None:
        return waypoints

    home_waypoint = {
        'x': float(config.return_home_x_m),
        'y': float(config.return_home_y_m),
        'z': float(
            config.return_home_altitude_m
            if config.return_home_altitude_m is not None
            else config.mapping_altitude_m
        ),
    }
    return _deduplicate_consecutive([*waypoints, home_waypoint])
