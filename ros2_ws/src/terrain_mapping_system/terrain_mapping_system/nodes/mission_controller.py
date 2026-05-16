"""High-level mission controller for lawnmower coverage through ArduPilot SITL."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from terrain_mapping_system.mission.artifacts import MissionArtifactWriter, utc_now_iso
from terrain_mapping_system.mission.mavlink_adapter import MavlinkAdapter, MavlinkConfig
from terrain_mapping_system.mission.planner import PlannerConfig, RectBounds, plan_lawnmower_path
from terrain_mapping_system.mission.terrain_manifest import bounds_from_manifest, load_manifest
from terrain_mapping_system.paths import default_results_root


class MissionState(str, Enum):
    INITIALIZING = 'initializing'
    CONNECTING = 'connecting'
    WAITING_FOR_HEARTBEAT = 'waiting_for_heartbeat'
    SETTING_GUIDED_MODE = 'setting_guided_mode'
    ARMING = 'arming'
    TAKING_OFF = 'taking_off'
    EXECUTING_SWEEP = 'executing_sweep'
    LANDING = 'landing'
    COMPLETED = 'completed'
    ABORTING = 'aborting'
    FAILED = 'failed'


@dataclass
class MissionPose:
    stamp_sec: float
    received_wall_time: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class MissionController(Node):
    """Runs a simple map-frame lawnmower mission and commands ArduPilot via MAVLink."""

    def __init__(self) -> None:
        super().__init__('mission_controller')

        self._declare_parameters()
        self._run_id = self.get_parameter('run_id').value or time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        self._results_root = self.get_parameter('results_root').value
        self._artifact_writer = MissionArtifactWriter(self._results_root, self._run_id)

        self._bounds, self._bounds_metadata = self._load_bounds()
        vehicle_spawn = self._bounds_metadata.get('vehicle_spawn')
        return_home_x_m = None
        return_home_y_m = None
        if isinstance(vehicle_spawn, dict):
            x_value = vehicle_spawn.get('x_m')
            y_value = vehicle_spawn.get('y_m')
            if x_value is not None and y_value is not None:
                return_home_x_m = float(x_value)
                return_home_y_m = float(y_value)

        self._planner_config = PlannerConfig(
            mapping_altitude_m=float(self.get_parameter('mapping_altitude_m').value),
            lane_spacing_m=float(self.get_parameter('lane_spacing_m').value),
            waypoint_tolerance_m=float(self.get_parameter('waypoint_tolerance_m').value),
            sweep_direction=str(self.get_parameter('sweep_direction').value),
            boundary_margin_m=float(self.get_parameter('boundary_margin_m').value),
            return_home_x_m=return_home_x_m,
            return_home_y_m=return_home_y_m,
        )
        self._waypoint_horizontal_tolerance_m = float(
            self.get_parameter('waypoint_horizontal_tolerance_m').value
        )
        self._waypoint_vertical_tolerance_m = float(
            self.get_parameter('waypoint_vertical_tolerance_m').value
        )

        self._mission_timeout_s = float(self.get_parameter('mission_timeout_s').value)
        self._command_interval_s = float(self.get_parameter('command_interval_s').value)
        self._arrival_timeout_s = float(self.get_parameter('arrival_timeout_s').value)
        self._pose_timeout_s = float(self.get_parameter('pose_timeout_s').value)
        self._takeoff_altitude_tolerance_m = float(self.get_parameter('takeoff_altitude_tolerance_m').value)
        self._guided_mode_timeout_s = float(self.get_parameter('guided_mode_timeout_s').value)
        self._arming_timeout_s = float(self.get_parameter('arming_timeout_s').value)
        self._takeoff_timeout_s = float(self.get_parameter('takeoff_timeout_s').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._state_topic = str(self.get_parameter('state_topic').value)

        self._pose_topic = str(self.get_parameter('pose_topic').value)
        self._last_pose: Optional[MissionPose] = None
        self._trajectory_samples = 0
        self._trajectory_log_period_s = float(self.get_parameter('trajectory_log_period_s').value)
        self._last_trajectory_log_time = 0.0
        self._state_log_period_s = float(self.get_parameter('state_log_period_s').value)
        self._last_state_log_time = 0.0

        self._subscription = self.create_subscription(
            PoseStamped,
            self._pose_topic,
            self._pose_callback,
            20,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(String, self._state_topic, state_qos)

        self._mavlink = MavlinkAdapter(
            MavlinkConfig(
                connection_string=str(self.get_parameter('mavlink.connection_string').value),
                baudrate=int(self.get_parameter('mavlink.baudrate').value),
                heartbeat_timeout_s=float(self.get_parameter('mavlink.heartbeat_timeout_s').value),
            )
        )

        self._mission_start_wall_time = time.monotonic()
        self._state = MissionState.INITIALIZING
        self._state_since = self._mission_start_wall_time
        self._last_command_time = 0.0
        self._current_waypoint_index = 0
        self._current_waypoint_started_at = self._mission_start_wall_time
        self._mission_completed = False
        self._failure_reason: Optional[str] = None
        self._landing_requested = False
        self._shutdown_requested = False

        self._planned_waypoints = plan_lawnmower_path(self._bounds, self._planner_config)
        self._write_plan_artifact()
        self._record_event('mission_initialized', details={'results': self._artifact_writer.snapshot()})

        self._set_state(MissionState.CONNECTING, 'starting mission controller')
        self._timer = self.create_timer(0.2, self._on_timer)

    def _declare_parameters(self) -> None:
        self.declare_parameter('pose_topic', '/terrain_mapping/vehicle/pose')
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('use_terrain_manifest', True)
        self.declare_parameter('fallback_bounds.x_min', -40.0)
        self.declare_parameter('fallback_bounds.x_max', 40.0)
        self.declare_parameter('fallback_bounds.y_min', -40.0)
        self.declare_parameter('fallback_bounds.y_max', 40.0)
        self.declare_parameter('mapping_altitude_m', 15.0)
        self.declare_parameter('lane_spacing_m', 10.0)
        self.declare_parameter('waypoint_tolerance_m', 0.25)
        self.declare_parameter('waypoint_horizontal_tolerance_m', 0.25)
        self.declare_parameter('waypoint_vertical_tolerance_m', 0.0)
        self.declare_parameter('sweep_direction', 'x')
        self.declare_parameter('boundary_margin_m', 3.0)
        self.declare_parameter('run_id', '')
        self.declare_parameter('results_root', default_results_root())
        self.declare_parameter('mission_timeout_s', 900.0)
        self.declare_parameter('command_interval_s', 0.2)
        self.declare_parameter('arrival_timeout_s', 120.0)
        self.declare_parameter('pose_timeout_s', 5.0)
        self.declare_parameter('takeoff_altitude_tolerance_m', 0.3)
        self.declare_parameter('guided_mode_timeout_s', 30.0)
        self.declare_parameter('arming_timeout_s', 30.0)
        self.declare_parameter('takeoff_timeout_s', 60.0)
        self.declare_parameter('trajectory_log_period_s', 0.5)
        self.declare_parameter('state_log_period_s', 5.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('state_topic', '/terrain_mapping/mission/state')
        self.declare_parameter('mavlink.connection_string', 'tcp:127.0.0.1:5760')
        self.declare_parameter('mavlink.baudrate', 115200)
        self.declare_parameter('mavlink.heartbeat_timeout_s', 30.0)

    def _load_bounds(self) -> tuple[RectBounds, Dict[str, object]]:
        use_manifest = bool(self.get_parameter('use_terrain_manifest').value)
        manifest_path = str(self.get_parameter('manifest_path').value).strip()
        if use_manifest and manifest_path:
            manifest = load_manifest(manifest_path)
            bounds, metadata = bounds_from_manifest(manifest)
            metadata['source'] = 'terrain_manifest'
            metadata['manifest_path'] = manifest_path
            return bounds, metadata

        bounds = RectBounds(
            x_min=float(self.get_parameter('fallback_bounds.x_min').value),
            x_max=float(self.get_parameter('fallback_bounds.x_max').value),
            y_min=float(self.get_parameter('fallback_bounds.y_min').value),
            y_max=float(self.get_parameter('fallback_bounds.y_max').value),
        )
        return bounds, {'source': 'fallback_bounds'}

    def _write_plan_artifact(self) -> None:
        payload = {
            'run_id': self._run_id,
            'generated_at': utc_now_iso(),
            'frame': 'map_enu',
            'planner': asdict(self._planner_config),
            'bounds': self._bounds.as_dict(),
            'bounds_metadata': self._bounds_metadata,
            'waypoints': [
                {'seq': index, **waypoint}
                for index, waypoint in enumerate(self._planned_waypoints)
            ],
        }
        self._artifact_writer.write_plan(payload)
        self._write_status()

    def _pose_callback(self, msg: PoseStamped) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec == 0.0:
            stamp_sec = time.time()
        pose = MissionPose(
            stamp_sec=stamp_sec,
            received_wall_time=time.monotonic(),
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            qx=float(msg.pose.orientation.x),
            qy=float(msg.pose.orientation.y),
            qz=float(msg.pose.orientation.z),
            qw=float(msg.pose.orientation.w),
        )
        self._last_pose = pose

        now = time.monotonic()
        if now - self._last_trajectory_log_time >= self._trajectory_log_period_s:
            self._last_trajectory_log_time = now
            self._trajectory_samples += 1
            self._artifact_writer.append_trajectory(
                {
                    'timestamp': utc_now_iso(),
                    'state': self._state.value,
                    'pose': asdict(pose),
                }
            )

    def _on_timer(self) -> None:
        try:
            self._step_mission()
        except Exception as exc:
            if self._state not in {MissionState.ABORTING, MissionState.FAILED, MissionState.COMPLETED}:
                self.get_logger().error(f'mission controller exception: {exc}')
                self._fail(f'mission controller exception: {exc}')

    def _step_mission(self) -> None:
        now = time.monotonic()
        self._mavlink.poll()

        if now - self._mission_start_wall_time > self._mission_timeout_s and not self._is_terminal():
            self._abort(f'mission timeout after {self._mission_timeout_s:.1f}s')
            return

        if self._state in {MissionState.TAKING_OFF, MissionState.EXECUTING_SWEEP, MissionState.LANDING}:
            if self._pose_age_s() > self._pose_timeout_s:
                self._abort(f'pose timeout exceeded {self._pose_timeout_s:.1f}s')
                return
            if self._mavlink.heartbeat_age_s() > float(self.get_parameter('mavlink.heartbeat_timeout_s').value):
                self._abort('MAVLink heartbeat timeout during mission execution')
                return

        if self._state == MissionState.CONNECTING:
            self._mavlink.connect()
            self._set_state(MissionState.WAITING_FOR_HEARTBEAT, 'MAVLink transport opened')
            return

        if self._state == MissionState.WAITING_FOR_HEARTBEAT:
            if self._mavlink.wait_heartbeat(timeout_s=0.1):
                self._record_event('heartbeat_received')
                if self._dry_run:
                    self._record_event('dry_run_complete', details={'waypoints': len(self._planned_waypoints)})
                    self._complete()
                else:
                    self._set_state(MissionState.SETTING_GUIDED_MODE, 'heartbeat received')
            return

        if self._state == MissionState.SETTING_GUIDED_MODE:
            if self._command_due(now):
                self._mavlink.set_mode('GUIDED')
                self._record_event('set_mode_guided')
            current_mode = self._mavlink.current_mode()
            if current_mode == 'GUIDED':
                self._set_state(MissionState.ARMING, 'guided mode confirmed')
                return
            if now - self._state_since > self._guided_mode_timeout_s:
                self._abort(
                    f'GUIDED mode was not confirmed within {self._guided_mode_timeout_s:.1f}s '
                    f'(current_mode={current_mode!r})'
                )
            return

        if self._state == MissionState.ARMING:
            if self._command_due(now):
                self._mavlink.arm()
                self._record_event('arm_command_sent')
            if self._mavlink.motors_armed():
                self._set_state(MissionState.TAKING_OFF, 'vehicle armed')
                self._current_waypoint_started_at = now
                self._last_state_log_time = 0.0
                return
            self._log_progress(
                now,
                'arming_progress',
                extra={
                    'mode': self._mavlink.current_mode(),
                    'motors_armed': self._mavlink.motors_armed(),
                },
            )
            if now - self._state_since > self._arming_timeout_s:
                self._abort(
                    f'arming timeout after {self._arming_timeout_s:.1f}s '
                    f'(mode={self._mavlink.current_mode()!r}, motors_armed={self._mavlink.motors_armed()})'
                )
            return

        if self._state == MissionState.TAKING_OFF:
            if self._command_due(now):
                self._mavlink.send_takeoff(self._planner_config.mapping_altitude_m)
                self._record_event(
                    'takeoff_command_sent',
                    details={'altitude_m': self._planner_config.mapping_altitude_m},
                )
            if self._last_pose is not None and self._last_pose.z >= (
                self._planner_config.mapping_altitude_m - self._takeoff_altitude_tolerance_m
            ):
                self._set_state(MissionState.EXECUTING_SWEEP, 'takeoff altitude reached')
                self._current_waypoint_started_at = now
                self._last_state_log_time = 0.0
                return
            self._log_progress(
                now,
                'takeoff_progress',
                extra={
                    'mode': self._mavlink.current_mode(),
                    'motors_armed': self._mavlink.motors_armed(),
                    'target_altitude_m': self._planner_config.mapping_altitude_m,
                    'current_altitude_m': None if self._last_pose is None else round(self._last_pose.z, 3),
                },
            )
            if now - self._state_since > self._takeoff_timeout_s:
                current_altitude = None if self._last_pose is None else round(self._last_pose.z, 3)
                self._abort(
                    f'takeoff timeout after {self._takeoff_timeout_s:.1f}s '
                    f'(mode={self._mavlink.current_mode()!r}, motors_armed={self._mavlink.motors_armed()}, '
                    f'altitude_m={current_altitude}, target_altitude_m={self._planner_config.mapping_altitude_m})'
                )
            return

        if self._state == MissionState.EXECUTING_SWEEP:
            if self._current_waypoint_index >= len(self._planned_waypoints):
                self._set_state(MissionState.LANDING, 'all sweep waypoints reached')
                return
            waypoint = self._planned_waypoints[self._current_waypoint_index]
            if self._command_due(now):
                self._mavlink.send_position_target_enu(
                    waypoint['x'],
                    waypoint['y'],
                    waypoint['z'],
                )
                self._record_event(
                    'waypoint_command_sent',
                    details={'index': self._current_waypoint_index, 'waypoint': waypoint},
                )
            if self._last_pose is not None and self._within_tolerance(self._last_pose, waypoint):
                self._record_event(
                    'waypoint_reached',
                    details={'index': self._current_waypoint_index, 'waypoint': waypoint},
                )
                self._current_waypoint_index += 1
                self._current_waypoint_started_at = now
            elif now - self._current_waypoint_started_at > self._arrival_timeout_s:
                self._abort(f'waypoint {self._current_waypoint_index} timeout after {self._arrival_timeout_s:.1f}s')
            return

        if self._state == MissionState.LANDING:
            if not self._landing_requested:
                self._mavlink.send_land()
                self._landing_requested = True
                self._record_event('land_command_sent')
            if not self._mavlink.motors_armed():
                self._complete()
            elif self._last_pose is not None and self._last_pose.z <= 0.5:
                self._complete()
            return

        if self._state == MissionState.ABORTING:
            if not self._landing_requested:
                try:
                    self._mavlink.send_land()
                    self._record_event('abort_land_command_sent')
                except Exception as exc:  # pragma: no cover - defensive cleanup
                    self.get_logger().warning(f'LAND during abort failed: {exc}')
                self._landing_requested = True
            if now - self._state_since > 5.0:
                self._fail(self._failure_reason or 'mission aborted')
            return

    def _within_tolerance(self, pose: MissionPose, waypoint: Dict[str, float]) -> bool:
        dx = pose.x - waypoint['x']
        dy = pose.y - waypoint['y']
        dz = pose.z - waypoint['z']
        horizontal_distance_m = math.sqrt(dx * dx + dy * dy)
        vertical_distance_m = abs(dz)
        vertical_gate_enabled = self._waypoint_vertical_tolerance_m > 0.0
        return (
            horizontal_distance_m <= self._waypoint_horizontal_tolerance_m
            and (
                not vertical_gate_enabled
                or vertical_distance_m <= self._waypoint_vertical_tolerance_m
            )
        )

    def _pose_age_s(self) -> float:
        if self._last_pose is None:
            return math.inf
        return max(0.0, time.monotonic() - self._last_pose.received_wall_time)

    def _command_due(self, now: float) -> bool:
        if now - self._last_command_time < self._command_interval_s:
            return False
        self._last_command_time = now
        return True

    def _set_state(self, state: MissionState, reason: str) -> None:
        self._state = state
        self._state_since = time.monotonic()
        self.get_logger().info(f'state={state.value} reason="{reason}"')
        self._publish_state()
        self._record_event('state_transition', details={'state': state.value, 'reason': reason})
        self._write_status()

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.value
        self._state_publisher.publish(message)

    def _record_event(self, event: str, details: Optional[Dict[str, object]] = None) -> None:
        payload = {
            'timestamp': utc_now_iso(),
            'event': event,
            'state': self._state.value,
            'details': details or {},
        }
        self._artifact_writer.append_event(payload)

    def _log_progress(self, now: float, event: str, *, extra: Optional[Dict[str, object]] = None) -> None:
        if now - self._last_state_log_time < self._state_log_period_s:
            return
        self._last_state_log_time = now
        details = extra or {}
        self.get_logger().info(f'{event} details={details}')
        self._record_event(event, details=details)
        self._write_status()

    def _write_status(self) -> None:
        payload = {
            'run_id': self._run_id,
            'timestamp': utc_now_iso(),
            'state': self._state.value,
            'current_waypoint_index': self._current_waypoint_index,
            'waypoint_count': len(self._planned_waypoints),
            'trajectory_samples': self._trajectory_samples,
            'dry_run': self._dry_run,
            'failure_reason': self._failure_reason,
        }
        self._artifact_writer.write_status(payload)

    def _complete(self) -> None:
        self._mission_completed = True
        self._set_state(MissionState.COMPLETED, 'mission finished')
        summary = self._summary_payload(final_status='completed')
        self._artifact_writer.write_summary(summary)
        self._write_status()
        self._record_event('mission_completed', details={'summary': summary})
        self._request_shutdown()

    def _abort(self, reason: str) -> None:
        self._failure_reason = reason
        self.get_logger().error(reason)
        self._set_state(MissionState.ABORTING, reason)

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._set_state(MissionState.FAILED, reason)
        summary = self._summary_payload(final_status='failed')
        self._artifact_writer.write_summary(summary)
        self._write_status()
        self._record_event('mission_failed', details={'summary': summary})
        self._request_shutdown()

    def _summary_payload(self, final_status: str) -> Dict[str, object]:
        return {
            'run_id': self._run_id,
            'final_status': final_status,
            'completed_at': utc_now_iso(),
            'duration_s': round(time.monotonic() - self._mission_start_wall_time, 3),
            'waypoints_total': len(self._planned_waypoints),
            'waypoints_reached': self._current_waypoint_index,
            'trajectory_samples': self._trajectory_samples,
            'failure_reason': self._failure_reason,
            'results': self._artifact_writer.snapshot(),
            'planner': asdict(self._planner_config),
            'bounds': self._bounds.as_dict(),
            'dry_run': self._dry_run,
        }

    def _is_terminal(self) -> bool:
        return self._state in {MissionState.COMPLETED, MissionState.FAILED}

    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.destroy_timer(self._timer)
        rclpy.shutdown()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('mission controller interrupted')
    finally:
        if not node._is_terminal() and node._mission_completed is False:
            node._artifact_writer.write_summary(node._summary_payload(final_status=node._state.value))
            node._write_status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
