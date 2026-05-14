from terrain_mapping_system.mission.planner import PlannerConfig, RectBounds, plan_lawnmower_path


def test_lawnmower_x_direction_snakes_across_y_lanes():
    bounds = RectBounds(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0)
    config = PlannerConfig(
        mapping_altitude_m=12.0,
        lane_spacing_m=10.0,
        waypoint_tolerance_m=2.0,
        sweep_direction='x',
        boundary_margin_m=0.0,
    )

    waypoints = plan_lawnmower_path(bounds, config)

    assert waypoints == [
        {'x': -10.0, 'y': -10.0, 'z': 12.0},
        {'x': 10.0, 'y': -10.0, 'z': 12.0},
        {'x': 10.0, 'y': 0.0, 'z': 12.0},
        {'x': -10.0, 'y': 0.0, 'z': 12.0},
        {'x': -10.0, 'y': 10.0, 'z': 12.0},
        {'x': 10.0, 'y': 10.0, 'z': 12.0},
    ]


def test_lawnmower_y_direction_respects_margin_and_endcaps():
    bounds = RectBounds(x_min=-20.0, x_max=20.0, y_min=-10.0, y_max=10.0)
    config = PlannerConfig(
        mapping_altitude_m=15.0,
        lane_spacing_m=12.0,
        waypoint_tolerance_m=2.0,
        sweep_direction='y',
        boundary_margin_m=2.0,
    )

    waypoints = plan_lawnmower_path(bounds, config)

    assert waypoints[0] == {'x': -18.0, 'y': -8.0, 'z': 15.0}
    assert waypoints[-1] == {'x': 18.0, 'y': -8.0, 'z': 15.0}
    assert [waypoint['x'] for waypoint in waypoints[::2]] == [-18.0, -6.0, 6.0, 18.0]


def test_lawnmower_appends_return_home_waypoint_when_configured():
    bounds = RectBounds(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0)
    config = PlannerConfig(
        mapping_altitude_m=12.0,
        lane_spacing_m=10.0,
        waypoint_tolerance_m=2.0,
        sweep_direction='x',
        boundary_margin_m=0.0,
        return_home_x_m=-44.0,
        return_home_y_m=-44.0,
    )

    waypoints = plan_lawnmower_path(bounds, config)

    assert waypoints[-1] == {'x': -44.0, 'y': -44.0, 'z': 12.0}
    assert waypoints[-2] == {'x': 10.0, 'y': 10.0, 'z': 12.0}
