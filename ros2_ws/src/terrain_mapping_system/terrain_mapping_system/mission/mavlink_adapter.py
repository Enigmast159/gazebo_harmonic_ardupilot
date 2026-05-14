"""Small pymavlink adapter used by the mission controller node."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from terrain_mapping_system.mission.conversions import enu_to_local_ned

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - exercised only when dependency is missing
    mavutil = None


@dataclass
class MavlinkConfig:
    connection_string: str
    baudrate: int = 115200
    heartbeat_timeout_s: float = 30.0


class MavlinkAdapter:
    """Minimal ArduPilot command adapter with explicit command helpers."""

    def __init__(self, config: MavlinkConfig):
        self._config = config
        self._master = None
        self._last_heartbeat_wall_time = None
        self._system_id = None
        self._component_id = None

    def connect(self) -> None:
        if mavutil is None:
            raise RuntimeError('pymavlink is not installed; install python3-pymavlink to enable mission control')
        if self._master is not None:
            return
        self._master = mavutil.mavlink_connection(
            self._config.connection_string,
            baud=self._config.baudrate,
            autoreconnect=True,
        )

    def wait_heartbeat(self, timeout_s: Optional[float] = None) -> bool:
        if self._master is None:
            raise RuntimeError('MAVLink connection not established')
        timeout = timeout_s if timeout_s is not None else self._config.heartbeat_timeout_s
        heartbeat = self._master.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            return False
        self._system_id = self._master.target_system
        self._component_id = self._master.target_component
        self._last_heartbeat_wall_time = time.monotonic()
        return True

    def poll(self) -> None:
        if self._master is None:
            return
        while True:
            message = self._master.recv_match(blocking=False)
            if message is None:
                break
            if message.get_type() == 'HEARTBEAT':
                self._last_heartbeat_wall_time = time.monotonic()

    def heartbeat_age_s(self) -> float:
        if self._last_heartbeat_wall_time is None:
            return math.inf
        return time.monotonic() - self._last_heartbeat_wall_time

    def set_mode(self, mode_name: str) -> None:
        self._require_master()
        self._master.set_mode_apm(mode_name)

    def current_mode(self) -> Optional[str]:
        self._require_master()
        mode = getattr(self._master, 'flightmode', None)
        if mode is None:
            return None
        mode_text = str(mode).strip()
        return mode_text or None

    def arm(self) -> None:
        self._require_master()
        self._master.arducopter_arm()

    def disarm(self) -> None:
        self._require_master()
        self._master.arducopter_disarm()

    def motors_armed(self) -> bool:
        self._require_master()
        return bool(self._master.motors_armed())

    def send_takeoff(self, altitude_m: float) -> None:
        self._require_master()
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )

    def send_land(self) -> None:
        self._require_master()
        self._master.set_mode_apm('LAND')

    def send_position_target_enu(self, x_enu: float, y_enu: float, z_enu: float, yaw_rad: float = 0.0) -> None:
        self._require_master()
        north_m, east_m, down_m = enu_to_local_ned(x_enu, y_enu, z_enu)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._master.mav.set_position_target_local_ned_send(
            0,
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north_m,
            east_m,
            down_m,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rad,
            0.0,
        )

    def _require_master(self) -> None:
        if self._master is None:
            raise RuntimeError('MAVLink connection not established')
