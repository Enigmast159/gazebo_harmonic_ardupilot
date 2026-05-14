"""Frame conversion helpers that keep ROS-side logic in ENU."""

from typing import Tuple


def enu_to_local_ned(x_enu: float, y_enu: float, z_enu: float) -> Tuple[float, float, float]:
    """Convert an ENU position into a LOCAL_NED position."""
    north_m = y_enu
    east_m = x_enu
    down_m = -z_enu
    return north_m, east_m, down_m


def local_ned_to_enu(north_m: float, east_m: float, down_m: float) -> Tuple[float, float, float]:
    """Convert a LOCAL_NED position into ENU."""
    x_enu = east_m
    y_enu = north_m
    z_enu = -down_m
    return x_enu, y_enu, z_enu
