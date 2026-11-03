"""Teach and visit named table service poses with existing Nav2 parking."""

import argparse
import ast
from contextlib import contextmanager
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from jdamr_cube_navigation.corridor_route import (
    _quaternion_yaw, AMCL_QOS, CorridorRoute, ODOM_TRAIL_STEP_M, PATH_BLOCKED_NAV2_CODES,
    spin_node)
from jdamr_cube_navigation.parking import (
    DOCK_POSITION_TOLERANCE_M, ENTRY_HEADING_TOLERANCE_DEG, load_parking_contract,
    MAXIMUM_CONTRACT_VALUES, ParkingHold,
    pose_errors,
)
from jdamr_cube_navigation.rear_collision_profile import rotation_collision_watch
from jdamr_cube_navigation.reverse_parking import (
    outline_intrusion, path_sweep_hit, rear_swing_clearance, remembered_blind_points,
    reverse_curve_waypoints, reverse_waypoints, scan_points, SERVICE_TRANSIT_MAX_MPS,
    static_corridor_clear, straight_sweep_hit, to_base_frame, trail_retrace_points)
from jdamr_cube_navigation.service_destinations import (
    add_pose, candidates, front_gap_evidence, grid_signature, home_pose, load_registry,
    map_grid_signature, new_registry, route_config, save_registry, set_home_pose, taught_pose,
    validate_gap_measurement,
    validate_registry, verify_identity,
)
from jdamr_cube_navigation.surface_approach import (
    FACE_ALIGNMENT_GAP_M, FACE_APPROACH_TIME_S, FACE_HEADING_TOLERANCE_RAD,
    TABLE_APPROACH_SPEED_MPS, face_approach_command, filtered_face_heading,
    lateral_face_approach_command,
)
from nav2_msgs.action import (
    ComputePathThroughPoses, FollowPath, NavigateThroughPoses, NavigateToPose, Spin)
from nav2_msgs.srv import IsPathValid, Toggle
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformException
import yaml


BLOCKED_PLAN_CODES = {
    ComputePathThroughPoses.Result.GOAL_OCCUPIED,
    ComputePathThroughPoses.Result.NO_VALID_PATH,
}
# AMCL publishes a pose only after one odom-frame axis exceeds update_min_d or
# update_min_a; test_t12 pins these to new_base_nav2_params.yaml.
AMCL_QUIET_MAX_AXIS_M = 0.05
AMCL_QUIET_MAX_YAW_RAD = 0.05
# AMCL stamps map->odom at every scan plus transform_tolerance, so an older
# map->base_link TF means AMCL or odometry has gone silent.
AMCL_QUIET_MAX_TF_AGE_S = 0.5
# A staging heading left within this goes to the curved pre-dock reverse as is
# (reverse_curve_waypoints accepts a 15 deg heading with a 10 cm offset).
STAGING_CORRECTION_MIN_RAD = math.radians(10.0)
# The heading is set this far in front of the dock and the rest is reversed
# straight. 2026-10-01: a curve ending at the dock left about 0.7 deg of heading
# per cm of staging offset (0.9-6.9 deg for 0.4-9.6 cm), turned in place inside
# the dock. 0.25 m keeps the curve wider than its 0.3 m radius for the 10 cm
# offsets seen from 0.70 m staging distances.
PRE_DOCK_DISTANCE_M = 0.25
# A staging offset up to this is reversed straight from the staging position.
STRAIGHT_DOCK_LATERAL_M = 0.01
# The straight entry, then one pull-out, heading set and entry again.
DOCK_ENTRY_ATTEMPTS = 2
# A dock heading off by up to 5 deg is turned where the robot stands (operator,
# 2026-10-02: the pull-out and re-entry shuttled back and forth). At 5 deg the
# rear corners (0.414 m from the turn centre) sweep about 3.6 cm.
DOCK_END_TURN_MAX_RAD = math.radians(5.0)
# There it is trimmed to 1 deg, not the 3 deg contract: 2.7 deg left the left
# front corner about 2 cm ahead of the right (operator, 2026-10-06).
DOCK_HEADING_TOLERANCE_RAD = math.radians(1.0)
DOCK_TRIM_ATTEMPTS = 3
# The configured global costmap updates at 1 Hz; recheck a rejected snapshot
# across two updates while stopped before reporting a blocked reverse path.
REVERSE_PATH_UPDATE_S = 1.0
REVERSE_PATH_CHECK_ATTEMPTS = 3
ENTRY_HEADING_CHECKER = 'entry_heading_checker'
ENTRY_HEADING_TOLERANCE_RAD = math.radians(ENTRY_HEADING_TOLERANCE_DEG)
# The PC refreshes the heartbeat on every 10 s poll; a missed poll stays inside this.
OPERATOR_LINK_TIMEOUT_S = 30.0
# A blocked base with an object right in front backs this far straight out before
# the hold, as Nav2's default tree backs up before it retries.
PATH_BLOCKED_BACKOFF_M = 0.10
# The band in front of the chassis front edge that counts as "blocked ahead".
PATH_BLOCKED_AHEAD_BAND_M = 0.40
# An in-place turn sweeps the rear about the axle near the front. Something beside
# the rear inside the 0.43 m rotation StopZone (+2 cm) blocks every turn, and an
# arc swings the rear toward it (2026-10-02 11:09: a left arc put a desk leg inside
# the right rear corner), so the base first drives straight out of that circle.
PATH_BLOCKED_TURN_RADIUS_M = 0.45
PATH_BLOCKED_FORWARD_MIN_M = 0.10
PATH_BLOCKED_FORWARD_MAX_M = 0.35
# Escapes drive with zero turn rate so the Collision Monitor uses its zero-turn
# StopZones; slow so its StopZone (5 mm beyond the body) still stops within one
# 10 Hz cycle.
STRAIGHT_ESCAPE_SPEED_MPS = 0.05
PRECISION_STRAIGHT_MIN_SPEED_MPS = 0.01
PRECISION_STRAIGHT_TARGET_S = 4.0
LIVE_APPROACH_MAX_M = 0.6
# A 1/s proportional correction removes a one-degree drift without snapping the
# chassis. 0.02 rad/s is below the 0.08 rad/s in-place trim and, on the configured
# 0.51 m wheel separation, keeps both wheels moving forward at the 0.01 m/s
# precision minimum instead of turning the final approach into a pivot.
PRECISION_HEADING_KP = 1.0
PRECISION_HEADING_MAX_RADPS = 0.02
STRAIGHT_ESCAPE_EXTRA_S = 5.0
STRAIGHT_ESCAPE_TOLERANCE_M = 0.01
# The velocity smoother's linear deceleration (max_decel) that ends a direct move.
STRAIGHT_DECEL_MPS2 = 0.5
# In-place trims (zero-turn final approach, dock heading). Their stop is sent
# TRIM_STOP_RAD early: anticipating 0.35 deg (smoother 1.5 rad/s^2 plus 50 ms), a
# 0.08 rad/s trim still went 0.52 deg past (2026-10-06); the base kept turning
# 70 ms after the stop and stopped over 140 ms, 0.87 deg in all.
TRIM_ANGULAR_RADPS = 0.08
STAGING_ANGULAR_RADPS = 0.3
STAGING_CRUISE_THRESHOLD_RAD = math.radians(10.0)
TRIM_STOP_RAD = math.radians(0.87)
TRIM_EXTRA_S = 4.0
# A direct move's stop is sent this much early on top of the deceleration: one
# 20 Hz smoother cycle plus odometry delay (reviewed 2026-10-06; not measured).
DIRECT_STOP_LATENCY_S = 0.05
# After the stop, wait at most this long for odometry to show the base at rest.
DIRECT_SETTLE_S = 0.5
DIRECT_STOP_CONFIRM_S = 0.08
DIRECT_STOP_CONFIRM_SAMPLES = 2
DIRECT_STOP_DECEL_GRACE_S = 0.5
# Direct moves re-check the fail-closed inputs (fresh scan and odom, no latched
# emergency stop) at this period and stop on the first failure.
DIRECT_GUARD_PERIOD_S = 0.1
# The Collision Monitor stops on this many returns inside one polygon.
COLLISION_MONITOR_MIN_POINTS = 3
# Backing out along the odom trail when something touches the body: 0.30 m, then
# 0.15 m more per hold, at most 0.60 m.
PATH_BLOCKED_RETRACE_M = 0.30
PATH_BLOCKED_RETRACE_STEP_M = 0.15
PATH_BLOCKED_RETRACE_MAX_M = 0.60
# The retrace follows the trail back only while each step leads backwards, is at
# most three trail steps long (an odom jump) and turns at most 20 deg (a turn on
# the spot that a reverse controller cannot undo).
RETRACE_MAX_GAP_M = 3.0 * ODOM_TRAIL_STEP_M
RETRACE_MAX_TURN_RAD = math.radians(20.0)
# Escape decisions use one scan no older than this (the G4 publishes at 7-10 Hz).
ESCAPE_SCAN_MAX_AGE_S = 0.5
# Scans kept, one per 5 cm or 10 deg of motion, for what the LiDAR cannot see within
# its range_min (0.28 m from the laser, up to 0.20 m ahead of the front edge):
# something the base drove up to vanishes from the live scan but not from these.
SCAN_MEMORY_KEYFRAMES = 20
SCAN_MEMORY_STEP_M = 0.05
SCAN_MEMORY_STEP_RAD = math.radians(10.0)
SCAN_MEMORY_MAX_AGE_S = 120.0
# Keyframes are 5 cm apart; a jump this large is an odom reset, not motion.
SCAN_MEMORY_RESET_M = 0.5
# While the Collision Monitor is paused for a straight escape, this band in the
# travel direction must stay clear on every fresh scan (0.05 m/s: 5 mm per check).
ESCAPE_WATCH_BAND_M = 0.10


# Transit and dock staging trees per planner + controller (behavior_trees/*.xml):
# navfn-mppi keeps MPPI on NavFn paths (2026-10-06: Lattice's waypoint headings
# planned a 5 m loop back to a waypoint MPPI had passed 0.31 m away).
TRANSIT_VARIANTS = {'lattice-mppi': '', 'navfn-mppi': '_navfn_mppi', 'navfn-rpp': '_rpp'}
# NavFn + RPP again since 2026-10-06 (lattice-mppi 10-05 to 10-06). One go table_02
# each: RPP 175.3 s, MPPI 284.2 s; MPPI's path critics turn off within 0.5 m
# straight-line distance of the final goal, so it skipped the pre-point 0.42 m from
# the observation pose and reversed in (evaluation/20261006_CONTROLLER_SELECTION_*).
DEFAULT_TRANSIT = 'navfn-rpp'
# A parked dwell is confirmed as a stationary window for at most this long; a longer
# dwell (an external gap measurement) then only has to stay put. One recovered input
# gap after the first seconds failed a 120 s window that held for 64 s (review
# 2026-10-06).
PARKED_CONFIRM_MAX_S = 5.0
PARKED_HOLD_MAX_MOVE_M = 0.02
PARKED_HOLD_MAX_TURN_RAD = math.radians(2.0)
# FollowPath TF_ERROR: RPP could not get map->odom at the robot pose stamp within
# its transform_tolerance. Nav2 Jazzy aborts on it at once (failure_tolerance covers
# only NoValidControl) and WouldAControllerRecoveryHelp does not retry it, so the
# transit tree ends (table_01_20261029_191747: map->odom 0.1115 s short).
NAV2_CONTROLLER_TF_ERROR = 102
NAV2_TF_ERROR_REASON = 'nav2 transform error'


def _ahead(pose, distance_m):
    """Return the pose distance_m in front of pose along its heading."""
    x, y, yaw = pose
    return (x + distance_m * math.cos(yaw), y + distance_m * math.sin(yaw), yaw)


def _lateral_offset(pose, axis):
    """Return the distance of pose from the line through axis along its heading."""
    return abs(-(pose[0] - axis[0]) * math.sin(axis[2])
               + (pose[1] - axis[1]) * math.cos(axis[2]))


def load_service_contract(path):
    """Load candidate localization bounds with distinct SI units."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('service confidence contract schema must be 1')
    for key in ('max_x_covariance_m2', 'max_y_covariance_m2', 'max_yaw_covariance_rad2',
                'minimum_start_battery_v', 'minimum_running_battery_v'):
        value = document.get(key)
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or value <= 0.0):
            raise ValueError(f'{key} must be finite and positive')
    if document['minimum_start_battery_v'] < document['minimum_running_battery_v']:
        raise ValueError('departure battery threshold must not be below running cutoff')
    for axis, unit in (('x', 'm2'), ('y', 'm2'), ('yaw', 'rad2')):
        key = f'intermediate_max_{axis}_covariance_{unit}'
        value = document.get(key)
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value < document[f'max_{axis}_covariance_{unit}']):
            raise ValueError(f'{key} must be finite and not below the strict limit')
    return document


def select_destination(poses, plan):
    """Try one alternate only for an occupied, unreachable or pathless goal."""
    attempts = []
    for pose in poses[:2]:
        result = plan(pose)
        attempts.append({'pose_id': pose['id'], **result})
        if result['ok']:
            return pose, attempts
        # NavFn tolerance ends an occupied goal short instead of failing it.
        if (result.get('error_code') not in BLOCKED_PLAN_CODES
                and result.get('reason') != 'goal_not_reachable_within_tolerance'):
            break
    return None, attempts


class BoxDwell:
    """Confirm one continuously fresh, stable box observation window."""

    def __init__(self, dwell_s, maximum_age_s=0.75):
        if (isinstance(dwell_s, bool) or not isinstance(dwell_s, (int, float))
                or not math.isfinite(dwell_s) or dwell_s <= 0.0):
            raise ValueError('box dwell must be finite and positive')
        if (isinstance(maximum_age_s, bool)
                or not isinstance(maximum_age_s, (int, float))
                or not math.isfinite(maximum_age_s) or maximum_age_s <= 0.0):
            raise ValueError('box observation age must be finite and positive')
        self.dwell_s = float(dwell_s)
        self.maximum_age_s = float(maximum_age_s)
        self.started_s = None

    def observe(self, now_s, received_s, status):
        """Return a fail-closed hold state for one parsed observer message."""
        values = (now_s, received_s)
        if (not isinstance(status, dict)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in values)
                or now_s < received_s or now_s - received_s > self.maximum_age_s
                or status.get('detected') is not True
                or status.get('stable') is not True
                or status.get('surface_kind') != 'front'):
            self.started_s = None
            return {'confirmed': False, 'hold_s': 0.0, 'reason': 'box_not_stable'}
        for key in ('front_distance_m', 'lateral_error_m', 'edge_angle_deg'):
            value = status.get(key)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                self.started_s = None
                return {'confirmed': False, 'hold_s': 0.0,
                        'reason': 'box_observation_invalid'}
        if self.started_s is None:
            self.started_s = float(now_s)
        hold_s = max(0.0, float(now_s) - self.started_s)
        return {
            'confirmed': hold_s >= self.dwell_s,
            'hold_s': hold_s,
            'reason': 'box_dwell_complete' if hold_s >= self.dwell_s else 'box_dwell_active',
        }


class ServiceRoute(CorridorRoute):
    """Reuse route protection, low-speed parking and post-goal pose checking."""

    def __init__(self, registry, contract, result_stream, home_contract=None):
        initial = {'id': 'capture', 'priority': 1,
                   'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0,
                   'approach_offset_m': 0.5}
        super().__init__(route_config(registry, initial),
                         navigation_profile='obstacle_base_candidate',
                         parking_contract=contract)
        # A named destination may be approached from anywhere on this map.
        self.start_check_pending = False
        # The dock acceptance contract may be looser than a precision session.
        self.home_contract = home_contract or contract
        self.amcl_odom_reference = None
        self.registry = registry
        self.result_stream = result_stream
        self.table_id = None
        self.pose_id = None
        self.confirmation = None
        self.selected_pose = None
        self.active_handle = None
        self.pending_goal = None
        self.navigation_result = None
        self.navigation_uuid = None
        self.cancel_navigation = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.query_navigation = self.create_client(
            NavigateToPose.Impl.GetResultService, 'navigate_to_pose/_action/get_result')
        self.follow_reverse = ActionClient(self, FollowPath, 'follow_path')
        self.cancel_reverse = self.create_client(
            CancelGoal, 'follow_path/_action/cancel_goal')
        self.query_reverse = self.create_client(
            FollowPath.Impl.GetResultService, 'follow_path/_action/get_result')
        self.navigate_through = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.cancel_through = self.create_client(
            CancelGoal, 'navigate_through_poses/_action/cancel_goal')
        self.query_through = self.create_client(
            NavigateThroughPoses.Impl.GetResultService,
            'navigate_through_poses/_action/get_result')
        self.spin_search = ActionClient(self, Spin, 'spin')
        self.cancel_spin = self.create_client(CancelGoal, 'spin/_action/cancel_goal')
        self.query_spin = self.create_client(
            Spin.Impl.GetResultService, 'spin/_action/get_result')
        self.validate_reverse_path = self.create_client(IsPathValid, 'is_path_valid')
        self.active_action_type = NavigateToPose
        self.amcl_yaw_covariance_rad2 = None
        package = Path(get_package_share_directory('jdamr_cube_navigation'))
        self.alignment_behavior_tree = str(
            package / 'behavior_trees/navigate_to_pose_alignment.xml')
        self.face_alignment_behavior_tree = str(
            package / 'behavior_trees/navigate_to_pose_face_alignment.xml')
        self.use_transit(DEFAULT_TRANSIT)
        # RPP for the box approach and the dock leg. Graceful (--graceful-final) checks
        # its own trajectory against the costmap in Nav2 1.3.12 with no switch, so a
        # 5 cm stop at a box ended in 105 (2026-10-01).
        self.final_approach_controller = 'Parking'
        self.dock_leg_controller = 'ParkingReverse'
        # Box search resume target; the staging turn shares the Spin helper and
        # ran in a process that never searched (2026-10-01 AttributeError).
        self._search_target_yaw = None
        self.last_search_failure_reason = None
        self.service_contract = load_service_contract(
            package / 'config/restaurant_service_contract.yaml')
        self.minimum_battery_v = self.service_contract['minimum_running_battery_v']
        self.max_amcl_covariance = (
            self.service_contract['max_x_covariance_m2'],
            self.service_contract['max_y_covariance_m2'])
        # Read only by _guard_failure; set through _localization_bound().
        self._intermediate_localization = False
        self.live_grids = {}
        self.expected_grids = {}
        self.map_mismatch = None
        self.box_status = None
        self.box_status_received_s = None
        self.create_subscription(
            String, '/box_parking/perception_status', self._box_callback, 1)
        for key, topic in (('map', '/map'), ('keepout', '/keepout_filter_mask')):
            self.create_subscription(
                OccupancyGrid, topic,
                lambda message, name=key: self._map_callback(name, message), AMCL_QOS)
        self.run_deadline_s = None
        self._input_recovery_paused = False
        self.create_timer(0.1, self._deadline_tick)

    def _box_callback(self, message):
        try:
            document = json.loads(message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            self.box_status = None
            self.box_status_received_s = None
            return
        if not isinstance(document, dict):
            self.box_status = None
            self.box_status_received_s = None
            return
        self.box_status = document
        self.box_status_received_s = time.monotonic()

    def _deadline_tick(self):
        if (self.run_deadline_s is not None
                and not getattr(self, '_input_recovery_paused', False)
                and time.monotonic() >= self.run_deadline_s):
            self.request_stop()

    def _amcl_callback(self, message):
        super()._amcl_callback(message)
        self.amcl_yaw_covariance_rad2 = float(message.pose.covariance[35])
        self.amcl_odom_reference = getattr(self, 'odom_last_pose', None)

    def _odom_callback(self, message):
        super()._odom_callback(message)
        # A latched AMCL pose may arrive before the first odometry sample.
        if (getattr(self, 'amcl_seen', None) is not None
                and getattr(self, 'amcl_odom_reference', None) is None):
            self.amcl_odom_reference = self.odom_last_pose

    def _quiet_amcl_denial(self):
        """Explain why an old AMCL pose is not merely quiet, or return None."""
        reference = getattr(self, 'amcl_odom_reference', None)
        current = getattr(self, 'odom_last_pose', None)
        if reference is None or current is None:
            return 'no odometry since AMCL pose'
        # Negate AMCL's own update rule per odom-frame axis, not a radius.
        dx = abs(current[0] - reference[0])
        dy = abs(current[1] - reference[1])
        dyaw = abs(math.atan2(math.sin(current[2] - reference[2]),
                              math.cos(current[2] - reference[2])))
        if (dx > AMCL_QUIET_MAX_AXIS_M or dy > AMCL_QUIET_MAX_AXIS_M
                or dyaw > AMCL_QUIET_MAX_YAW_RAD):
            return f'moved since AMCL pose: dx={dx:.3f} dy={dy:.3f} dyaw={dyaw:.3f}'
        try:
            transform = self.parking_tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as error:
            return f'map->base_link TF unavailable: {error}'
        stamp = transform.header.stamp
        age_s = (self.get_clock().now().nanoseconds * 1e-9
                 - stamp.sec - stamp.nanosec * 1e-9)
        if age_s < 0.0:
            return f'map->base_link TF ahead of ROS time: age={age_s:.3f}s'
        if age_s > AMCL_QUIET_MAX_TF_AGE_S:
            return ('map->base_link TF stale (AMCL silent > ~1.5 s or odom TF stale): '
                    f'age={age_s:.3f}s')
        return None

    def _map_callback(self, name, message):
        try:
            origin = message.info.origin
            if abs(origin.position.z) > 1e-9:
                raise ValueError('map origin must be planar')
            signature = grid_signature(
                message.info.width, message.info.height, message.info.resolution,
                (origin.position.x, origin.position.y, _quaternion_yaw(origin.orientation)),
                message.data, message.header.frame_id)
            self.live_grids[name] = signature
            if name in self.expected_grids and signature != self.expected_grids[name]:
                self.map_mismatch = f'live {name} changed or does not match registered data'
                self.request_stop()
        except ValueError as error:
            self.map_mismatch = str(error)
            self.request_stop()

    def _guard_failure(self, require_fresh_amcl=True):
        if self.map_mismatch:
            return self.map_mismatch
        # Intermediate legs use the lost-localization bound; the final approach,
        # its confirmation, escape and docking keep the strict limits.
        intermediate = getattr(self, '_intermediate_localization', False)
        prefix = 'intermediate_max' if intermediate else 'max'
        limits = ((self.service_contract['intermediate_max_x_covariance_m2'],
                   self.service_contract['intermediate_max_y_covariance_m2'])
                  if intermediate else None)
        bound = 'intermediate' if intermediate else 'strict'
        failure = super()._guard_failure(require_fresh_amcl, covariance_limits=limits)
        if (failure and require_fresh_amcl
                and failure.startswith('AMCL pose stale:')):
            # A stationary robot keeps AMCL quiet by design. Exempt only the
            # pose age; every other base check still applies.
            denial = self._quiet_amcl_denial()
            if denial is not None:
                return f'{failure}; quiet exemption denied: {denial}'
            failure = super()._guard_failure(False, covariance_limits=limits)
        if failure:
            if 'covariance high' in failure:
                # Three decimals hid 0.010213 against 0.01 in the 9/29 log.
                failure += (f' (x={self.amcl_covariance[0]:.6f} '
                            f'y={self.amcl_covariance[1]:.6f} bound={bound})')
            return failure
        yaw_covariance_rad2 = self.amcl_yaw_covariance_rad2
        if (yaw_covariance_rad2 is None or not math.isfinite(yaw_covariance_rad2)
                or yaw_covariance_rad2 < 0.0):
            return 'AMCL yaw covariance invalid'
        yaw_limit_rad2 = self.service_contract[f'{prefix}_yaw_covariance_rad2']
        if yaw_covariance_rad2 > yaw_limit_rad2:
            return (f'AMCL yaw covariance high: value={yaw_covariance_rad2:.6f} '
                    f'limit={yaw_limit_rad2:.6f} bound={bound}')
        if any(value < 0.0 for value in self.amcl_covariance):
            return 'AMCL position covariance invalid'
        moved = (self.amcl_motion_distance_m >= self.revisit_motion_min_distance_m
                 or self.amcl_motion_rotation_rad >= self.revisit_motion_min_rotation_rad)
        if moved and time.monotonic() - self.amcl_seen > self.revisit_motion_amcl_freshness_s:
            return 'AMCL stale after motion'
        return None

    def emit(self, event, **fields):
        """Flush structured evidence on each transition, including failures."""
        # wall_time_s lines events up with the PC bag and phone video.
        record = {'event': event, 'table_id': self.table_id,
                  'pose_id': self.pose_id, 'monotonic_s': time.monotonic(),
                  'wall_time_s': time.time(),
                  'physical_accuracy': 'NOT_MEASURED', **fields}
        record['front_gap'] = front_gap_evidence(getattr(self, 'selected_pose', None) or {})
        covariance = getattr(self, 'amcl_covariance', None)
        yaw_covariance = getattr(self, 'amcl_yaw_covariance_rad2', None)
        record['localization_covariance'] = {
            'xy_m2': ([value if math.isfinite(value) else None for value in covariance]
                      if covariance is not None else None),
            'yaw_rad2': (
                yaw_covariance if yaw_covariance is not None
                and math.isfinite(yaw_covariance) else None),
        }
        payload = json.dumps(record, ensure_ascii=False, allow_nan=False)
        self.result_stream.write(payload + '\n')
        self.result_stream.flush()
        self.get_logger().info('service_event ' + payload)

    def _route_event(self, event, route_index, handle, **fields):
        if handle is None:
            if event not in ('parking_estimate_confirmed', 'parking_not_confirmed'):
                raise ValueError('goal lifecycle events require a goal handle')
            # Dwell is an observation window, not another Nav2 action goal.
            self.confirmation = fields
            self.emit('parked_dwell_observation', observation_event=event,
                      goal_uuid=None, waypoint_id=self.waypoints[route_index]['id'],
                      **fields)
            return
        super()._route_event(event, route_index, handle, **fields)
        if event == 'accepted':
            self.active_handle = handle
            self._mission_started = True
        elif event == 'result':
            self.active_handle = None
        if event in ('parking_estimate_confirmed', 'parking_not_confirmed'):
            self.confirmation = fields
        yaw_covariance = getattr(self, 'amcl_yaw_covariance_rad2', None)
        self.emit(event, waypoint_id=self.waypoints[route_index]['id'],
                  amcl_yaw_covariance_rad2=(
                      yaw_covariance if yaw_covariance is not None
                      and math.isfinite(yaw_covariance) else None), **fields)

    def _wait(self, future, timeout_s):
        deadline_s = time.monotonic() + timeout_s
        while not future.done() and not self.stop_requested and time.monotonic() < deadline_s:
            spin_node(self, timeout_sec=0.05)
        if not future.done() or future.exception() is not None:
            # Which of the three, so a cycle log tells them apart (2026-10-06).
            detail = ('stop requested' if not future.done() and self.stop_requested
                      else f'no response in {timeout_s:.1f} s' if not future.done()
                      else f'failed: {future.exception()!r}')
            raise RuntimeError(f'request interrupted, failed or timed out ({detail})')
        return future.result()

    @staticmethod
    def _publisher_names(publishers):
        names = []
        for publisher in publishers:
            namespace = getattr(publisher, 'node_namespace', '') or '/'
            name = getattr(publisher, 'node_name', '<unknown>')
            names.append(
                f'/{name}' if namespace == '/'
                else f'{namespace.rstrip("/")}/{name}')
        return names

    def _startup_protection_ready(self, discovery_timeout_s=30.0,
                                  require_command_path=True):
        """
        Wait for unresolved DDS identities, never for a known conflict.

        A fresh process on the Pi missed the /cmd_vel publisher within 2.5 s twice
        while UDP receive buffers overflowed (2026-10-01); the live maps already
        get 30 s.
        """
        expected = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info':
                'keepout_costmap_filter_info_server',
        }
        deadline_s = time.monotonic() + discovery_timeout_s
        while True:
            if self.stop_requested:
                return 'publisher discovery interrupted before motion'
            missing = None
            for topic, node_name in expected.items():
                publishers = self.get_publishers_info_by_topic(topic)
                actual = self._publisher_names(publishers)
                if not publishers:
                    if topic == '/cmd_vel' and not require_command_path:
                        continue
                    missing = (topic, node_name, actual)
                    continue
                if len(publishers) != 1:
                    return (
                        f'{topic} publisher must be {node_name} only; '
                        f'actual={actual}')
                publisher = publishers[0]
                name = getattr(publisher, 'node_name', None)
                namespace = getattr(publisher, 'node_namespace', None)
                unresolved_name = name in (None, '', '_NODE_NAME_UNKNOWN_')
                unresolved_namespace = namespace in (
                    None, '', '_NODE_NAMESPACE_UNKNOWN_')
                if ((not unresolved_name and name != node_name)
                        or (not unresolved_namespace and namespace != '/')):
                    return (
                        f'{topic} publisher must be /{node_name} only; '
                        f'actual={actual}')
                if unresolved_name or unresolved_namespace:
                    missing = (topic, node_name, actual)
            if self.stop_requested:
                return 'publisher discovery interrupted before motion'
            if missing is None:
                return None
            if self.stop_requested or time.monotonic() >= deadline_s:
                topic, node_name, actual = missing
                return (
                    f'{topic} publisher must be {node_name} only; '
                    f'actual={actual}')
            spin_node(self, timeout_sec=0.05)

    def verify_live_maps(self, require_command_path=True):
        """Require the running map servers to name the registered assets."""
        registry_identity = json.dumps(
            {name: self.registry[name] for name in ('map', 'keepout')}, sort_keys=True)
        if getattr(self, '_verified_map_identity', None) == registry_identity:
            if self.map_mismatch or self.live_grids != self.expected_grids:
                raise RuntimeError(self.map_mismatch or 'live map/keepout changed')
            failure = self._startup_protection_ready(require_command_path=require_command_path)
            if failure:
                raise RuntimeError(failure)
            if require_command_path:
                self._confirm_collision_monitor_on()
            return
        validate_registry(self.registry)
        self.expected_grids = {
            name: map_grid_signature(self.registry[name]['yaml_path'])
            for name in ('map', 'keepout')}
        # Allow initial transient-local discovery without relaxing asset identity.
        # A fresh process on the loaded Pi took up to 6 s idle and more while driving.
        deadline_s = time.monotonic() + 30.0
        while (len(self.live_grids) != 2 and not self.map_mismatch and not self.stop_requested
               and time.monotonic() < deadline_s):
            spin_node(self, timeout_sec=0.05)
        missing = sorted(set(self.expected_grids) - set(self.live_grids))
        if missing and not self.map_mismatch:
            raise RuntimeError('live map data unavailable: ' + ', '.join(missing))
        if self.map_mismatch or self.live_grids != self.expected_grids:
            raise RuntimeError(
                self.map_mismatch or 'live map/keepout does not match registered data')
        for node_name, identity in (
                ('map_server', self.registry['map']),
                ('keepout_filter_mask_server', self.registry['keepout'])):
            response = self._read_parameters(node_name, ['yaml_filename'])
            if response is None:
                raise RuntimeError(f'{node_name} parameters unavailable')
            if len(response.values) != 1:
                raise RuntimeError(f'{node_name} yaml_filename unavailable')
            path = parameter_value_to_python(response.values[0])
            if not isinstance(path, str) or not path:
                raise RuntimeError(f'{node_name} yaml_filename is not a file')
            verify_identity(identity, path)
        protection_error = self._startup_protection_ready(
            require_command_path=require_command_path)
        if protection_error:
            raise RuntimeError(protection_error)
        if require_command_path:
            self._confirm_collision_monitor_on()
        # Cache the verified assets for this executor, not sensor freshness or
        # permission to drive. Map callbacks still stop on changed grid content.
        self._verified_map_identity = registry_identity

    def plan_pose(self, pose, single=False, end_tolerance_m=None):
        """
        Validate the entire approach before dispatching any motion goal.

        The plan must end within the goal checker's xy tolerance: the final
        contract by default, or end_tolerance_m for intermediate alignment.
        """
        self.pose_id = pose['id']
        self.config = route_config(self.registry, pose)
        if single:
            self.config['waypoints'] = self.config['waypoints'][-1:]
        self.waypoints = self.config['waypoints']
        if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
            return {'ok': False, 'reason': 'navigation_not_ready',
                    'guard_failure': self._guard_failure(False) or 'operator_stop'}
        if not self.compute.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'reason': 'planner_unavailable'}
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [self._pose(index, item) for index, item in enumerate(self.waypoints)]
        goal.planner_id = 'GridBased'
        goal.use_start = False
        handle = None
        try:
            handle = self._wait(self.compute.send_goal_async(goal), 3.0)
            if not handle.accepted:
                return {'ok': False, 'reason': 'planner_rejected'}
            wrapped = self._wait(handle.get_result_async(), 10.0)
        except RuntimeError as error:
            if handle is not None and handle.accepted:
                self._cancel(handle, 'service planning interrupted')
            return {'ok': False, 'reason': str(error)}
        result = wrapped.result
        ok = (wrapped.status == GoalStatus.STATUS_SUCCEEDED
              and result.error_code == 0 and bool(result.path.poses))
        planned = {'ok': ok, 'error_code': int(result.error_code),
                   'reason': result.error_msg or ('planned' if ok else 'planning_failed')}
        if not ok:
            return planned
        # NavFn ends an unreachable goal cell short without an error code, and
        # the controller then judges only that truncated end.
        end = result.path.poses[-1].pose.position
        goal = self.waypoints[-1]
        end_error_m = math.hypot(end.x - goal['x'], end.y - goal['y'])
        tolerance_m = (self.parking_contract['xy_tolerance_m'] if end_tolerance_m is None
                       else end_tolerance_m)
        if not math.isfinite(end_error_m) or end_error_m > tolerance_m:
            return {'ok': False, 'error_code': int(result.error_code),
                    'reason': 'goal_not_reachable_within_tolerance',
                    'end_error_m': end_error_m if math.isfinite(end_error_m) else None,
                    'end_tolerance_m': tolerance_m}
        planned['end_error_m'] = end_error_m
        return planned

    def _plan_with_input_recovery(self, pose, **kwargs):
        """Replan the same registry pose once after a recovered input gap."""
        planned = self.plan_pose(pose, **kwargs)
        reason = planned.get('guard_failure')
        if (planned.get('reason') == 'navigation_not_ready' and not self.stop_requested
                and self._input_gap_recoverable(reason)
                and self._wait_for_input_recovery(reason)):
            planned = self.plan_pose(pose, **kwargs)
        return planned

    def finish_navigation(self):
        """Resolve late acceptance and wait for terminal cancellation evidence."""
        if self.pending_goal is not None:
            deadline_s = time.monotonic() + 5.0
            while not self.pending_goal.done() and time.monotonic() < deadline_s:
                spin_node(self, timeout_sec=0.05)
            if not self.pending_goal.done() or self.pending_goal.exception():
                if self._cancel_navigation_uuid():
                    return True
                self._emergency_stop_unconfirmed('cancel_unconfirmed')
                return False
            handle = self.pending_goal.result()
            self.pending_goal = None
            if handle.accepted:
                self.active_handle = handle
        if self.active_handle is None:
            return True
        handle = self.active_handle
        result_future = self.navigation_result or handle.get_result_async()
        if not result_future.done():
            self._cancel(handle, 'service command interrupted')
            deadline_s = time.monotonic() + 5.0
            while not result_future.done() and time.monotonic() < deadline_s:
                spin_node(self, timeout_sec=0.05)
        terminal = (result_future.done() and result_future.exception() is None
                    and result_future.result().status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED))
        self.emit('navigation_terminal' if terminal else 'cancel_unconfirmed')
        if terminal:
            self.active_handle = None
            self.navigation_result = None
        else:
            # A goal that may still be driving is stopped below Nav2.
            self._emergency_stop_unconfirmed('cancel_unconfirmed')
        return terminal

    def _emergency_stop_unconfirmed(self, reason):
        self.engage_emergency_stop(reason)
        self.emit('emergency_stop_requested', reason=reason)

    def _cancel_navigation_uuid(self):
        """Cancel our goal even when its acceptance response was lost."""
        if self.navigation_uuid is None:
            self.emit('cancel_unconfirmed', reason='navigation_goal_id_unavailable')
            return False
        deadline_s = time.monotonic() + 5.0
        cancel_future = None
        query_future = None
        action_type = getattr(self, 'active_action_type', NavigateToPose)
        if action_type is Spin:
            cancel_client, query_client = self.cancel_spin, self.query_spin
        elif action_type is FollowPath:
            cancel_client, query_client = self.cancel_reverse, self.query_reverse
        elif action_type is NavigateThroughPoses:
            cancel_client, query_client = self.cancel_through, self.query_through
        else:
            cancel_client, query_client = self.cancel_navigation, self.query_navigation
        while time.monotonic() < deadline_s:
            if (cancel_client.service_is_ready()
                    and (cancel_future is None or cancel_future.done())):
                request = CancelGoal.Request()
                request.goal_info.goal_id = self.navigation_uuid
                cancel_future = cancel_client.call_async(request)
            if (query_client.service_is_ready()
                    and (query_future is None or query_future.done())):
                request = action_type.Impl.GetResultService.Request()
                request.goal_id = self.navigation_uuid
                query_future = query_client.call_async(request)
            spin_node(self, timeout_sec=0.1)
            if (query_future is not None and query_future.done()
                    and query_future.exception() is None):
                status = query_future.result().status
                if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                              GoalStatus.STATUS_ABORTED):
                    self.pending_goal = None
                    self.active_handle = None
                    self.navigation_result = None
                    self.emit('navigation_terminal', terminal_status_code=int(status))
                    return True
        self.emit('cancel_unconfirmed', reason='navigation_terminal_unavailable',
                  goal_uuid=bytes(self.navigation_uuid.uuid).hex())
        return False

    def _tag_blocked(self, error_code, final):
        """Mark a leg that could not move on for the hold-and-retry in the recovery loop."""
        if not self.stop_requested and not final and error_code in PATH_BLOCKED_NAV2_CODES:
            self._retry_blocked_reason = f'path blocked: nav2 error {error_code}'
            self._retry_blocked_code = error_code
        elif not self.stop_requested and error_code == NAV2_CONTROLLER_TF_ERROR:
            # The existing input-gap path: wait for fresh, stopped inputs, then
            # resend the current waypoint once; a second 102 still ends the leg.
            self._retry_guard_reason = f'{NAV2_TF_ERROR_REASON} {error_code}'

    def _path_open(self):
        """Plan once from the current pose to the waypoints not yet reached."""
        remaining = self.waypoints[getattr(self, '_resume_waypoint_index', 0):]
        if not remaining or not self.compute.wait_for_server(timeout_sec=1.0):
            return False
        goal = ComputePathThroughPoses.Goal()
        first = len(self.waypoints) - len(remaining)
        goal.goals = [self._pose(first + index, item) for index, item in enumerate(remaining)]
        goal.planner_id = 'GridBased'
        goal.use_start = False
        try:
            handle = self._wait(self.compute.send_goal_async(goal), 3.0)
            if not handle.accepted:
                return False
            wrapped = self._wait(handle.get_result_async(), 5.0)
        except RuntimeError:
            return False
        return (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and not wrapped.result.error_code and len(wrapped.result.path.poses) > 0)

    def use_transit(self, variant):
        """Select the transit and staging trees by variant (TRANSIT_VARIANTS)."""
        self._set_transit_trees(TRANSIT_VARIANTS[variant])
        # MPPI drives to the pre-point; RPP takes the straight leg to the observation
        # pose (see _execute_through_once).
        self.final_leg_behavior_tree = (
            str(Path(get_package_share_directory('jdamr_cube_navigation'))
                / 'behavior_trees/navigate_through_poses_transit_rpp.xml')
            if variant.endswith('mppi') else None)

    def use_rpp_transit(self):
        """
        Follow the transit and dock staging legs with RPP (DEFAULT_TRANSIT).

        Transit and staging plan with NavFn. Box approach, alignment and the dock
        leg keep their controllers.
        """
        self.use_transit('navfn-rpp')

    def _set_transit_trees(self, suffix):
        package = Path(get_package_share_directory('jdamr_cube_navigation'))
        self.through_behavior_tree = str(
            package / f'behavior_trees/navigate_through_poses_transit{suffix}.xml')
        self.staging_behavior_tree = str(
            package / f'behavior_trees/navigate_to_pose_staging{suffix}.xml')

    def _scan_in_base(self):
        """
        Return one snapshot for an escape decision, or None if any part is unavailable.

        (scan, laser pose in the base frame, outline, remembered returns inside the
        LiDAR blind range in the base frame). The scan must be fresh; every check of
        one decision uses the same snapshot.
        """
        scan = getattr(self, 'last_scan', None)
        received_s = (getattr(self, 'samples', None) or {}).get('scan')
        if (scan is None or received_s is None
                or time.monotonic() - received_s > ESCAPE_SCAN_MAX_AGE_S):
            return None
        try:
            transform = self.parking_tf.lookup_transform(
                self.parking_contract['robot_base_frame'], scan.header.frame_id,
                rclpy.time.Time())
            outline = self._footprint_outline()
            laser = (transform.transform.translation.x, transform.transform.translation.y,
                     _quaternion_yaw(transform.transform.rotation))
            memory = self._remembered_blind_points(laser, scan)
        except (TransformException, RuntimeError, ValueError, SyntaxError):
            return None
        return scan, laser, outline, memory

    def _remembered_blind_points(self, laser, scan, source_point_filter=None):
        """Remembered returns now inside the LiDAR range_min, in the base frame."""
        now_s = time.monotonic()
        keyframes = [(pose, frame) for stamp_s, pose, frame
                     in getattr(self, 'scan_memory', None) or ()
                     if now_s - stamp_s <= SCAN_MEMORY_MAX_AGE_S]
        cleared_sources = getattr(self, '_blind_point_clearances', None)
        if cleared_sources is None:
            cleared_sources = self._blind_point_clearances = set()
        clearance_failures = getattr(self, '_blind_point_clearance_failures', None)
        if clearance_failures is None:
            clearance_failures = self._blind_point_clearance_failures = {}
        self._blind_point_sources = []
        self._blind_point_diagnostics = []
        if not keyframes:
            cleared_sources.clear()
            clearance_failures.clear()
            return []
        current_pose = self._odom_pose()
        current_scan_pose = None
        current_evidence_failure = None
        try:
            stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            ros_now_s = self.get_clock().now().nanoseconds * 1e-9
            if not 0.0 <= ros_now_s - stamp_s <= ESCAPE_SCAN_MAX_AGE_S:
                current_evidence_failure = 'current_scan_timestamp_not_fresh'
                raise ValueError(current_evidence_failure)
            base = self.parking_contract['robot_base_frame']
            scan_transform = self.parking_tf.lookup_transform(
                'odom', base, rclpy.time.Time(seconds=stamp_s)).transform
            current_scan_pose = (
                scan_transform.translation.x, scan_transform.translation.y,
                _quaternion_yaw(scan_transform.rotation))
        except TransformException:
            current_evidence_failure = 'current_scan_stamped_tf_unavailable'
        except (AttributeError, TypeError, ValueError, OverflowError):
            # A missing stamped pose can never establish clearance. Existing
            # memory still uses the current pose and remains fail-closed.
            current_evidence_failure = (
                current_evidence_failure or 'current_scan_timestamp_invalid')
        return remembered_blind_points(
            keyframes, current_pose, laser, scan.range_min,
            source_point_filter=source_point_filter, source_records=self._blind_point_sources,
            current_scan=scan, current_scan_pose=current_scan_pose,
            cleared_sources=cleared_sources, clearance_failures=clearance_failures,
            source_diagnostics=self._blind_point_diagnostics,
            current_evidence_failure=current_evidence_failure)

    def _remember_scan(self, scan):
        """Keep a scan whenever the base moved SCAN_MEMORY_STEP_M or turned STEP_RAD."""
        memory = getattr(self, 'scan_memory', None)
        latest_pose = getattr(self, 'odom_last_pose', None)
        if memory is None or latest_pose is None:
            return
        if memory and math.dist(latest_pose[:2], memory[-1][1][:2]) > SCAN_MEMORY_RESET_M:
            memory.clear()
        if memory:
            _stamp_s, last, _scan = memory[-1]
            turned = abs(math.atan2(
                math.sin(latest_pose[2] - last[2]),
                math.cos(latest_pose[2] - last[2])))
            if (math.dist(latest_pose[:2], last[:2]) < SCAN_MEMORY_STEP_M
                    and turned < SCAN_MEMORY_STEP_RAD):
                return
        pose = latest_pose
        try:
            header = scan.header
        except AttributeError:
            # Headerless test/custom scans cannot establish persistent clearance.
            header = None
        if header is not None:
            try:
                stamp_s = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
                if not math.isfinite(stamp_s):
                    return
                transform = self.parking_tf.lookup_transform(
                    'odom', self.parking_contract['robot_base_frame'],
                    rclpy.time.Time(seconds=stamp_s)).transform
            except (AttributeError, TypeError, ValueError, OverflowError,
                    TransformException):
                return
            pose = (transform.translation.x, transform.translation.y,
                    _quaternion_yaw(transform.rotation))
            if memory:
                _stored_s, last, _stored_scan = memory[-1]
                turned = abs(math.atan2(
                    math.sin(pose[2] - last[2]), math.cos(pose[2] - last[2])))
                if (math.dist(pose[:2], last[:2]) < SCAN_MEMORY_STEP_M
                        and turned < SCAN_MEMORY_STEP_RAD):
                    return
        memory.append((time.monotonic(), pose, scan))

    def _blocked_ahead(self, band_m=PATH_BLOCKED_AHEAD_BAND_M, found=None):
        """
        Return whether driving band_m straight ahead would meet a return.

        None when no snapshot is available. found is a snapshot from _scan_in_base.
        """
        found = found or self._scan_in_base()
        if found is None:
            return None
        scan, laser, outline, memory = found
        return straight_sweep_hit(scan, laser, outline, band_m, extra_points=memory)

    def _blocked_behind(self, band_m, found=None):
        """Return whether reversing band_m straight would meet a return."""
        found = found or self._scan_in_base()
        if found is None:
            return None
        scan, laser, outline, memory = found
        return straight_sweep_hit(scan, laser, outline, -band_m, extra_points=memory)

    def _footprint_intrusion(self, found=None):
        """
        Which half of the footprint holds live returns: 'front', 'rear', 'both' or None.

        Live returns only: this decides whether the Collision Monitor is held and must
        be paused, and the monitor sees the same scan. Remembered returns inside the
        blind range cannot hold it (review 2026-10-06: a stale one could pause it).
        """
        found = found or self._scan_in_base()
        if found is None:
            return None
        scan, laser, outline, _memory = found
        return outline_intrusion(scan, laser, outline, COLLISION_MONITOR_MIN_POINTS)

    def _rear_swing_need(self, found=None):
        """Forward travel that frees the turn circle behind the axle (0.0 when free)."""
        found = found or self._scan_in_base()
        if found is None:
            return 0.0
        scan, laser, _outline, memory = found
        return rear_swing_clearance(scan, laser, PATH_BLOCKED_TURN_RADIUS_M,
                                    extra_points=memory)

    def _front_clear(self):
        """Whether the band ahead is empty on a fresh scan (an unreadable scan is not)."""
        return self._blocked_ahead() is False

    def _escape_blocked(self, wait=1):
        """
        Move away from what blocks the base before the hold.

        Every check of one decision uses one scan snapshot, which includes the
        remembered returns inside the LiDAR blind range.

        A live return inside the footprint (it touches the body and holds every
        Collision Monitor polygon): back out along the odom trail the base drove in
        with the monitor paused (operator 2026-10-02: "if it got in it can get out";
        a straight escape beside a touching object pushed it, 18:02). When that way
        back would push what touches the rear, or there is none, drive the rear off
        it straight ahead instead if the band ahead is clear (2026-10-02 11:09).

        Something in the band ahead: back PATH_BLOCKED_BACKOFF_M times the hold
        count straight out (0.10, 0.20, 0.30 m; a repeated 0.10 m back-off met the
        same object four times, 17:49-17:52), or back out along the trail when the
        straight way back is not clear; the monitor stays on for both, since nothing
        touches the body. Something behind the axle inside the turn circle: drive
        forward until it is out of the circle, since every turn swings the rear into it.
        """
        found = self._scan_in_base()
        if found is None:
            return False
        ahead = self._blocked_ahead(found=found)
        retrace_m = min(PATH_BLOCKED_RETRACE_M + PATH_BLOCKED_RETRACE_STEP_M * (max(1, wait) - 1),
                        PATH_BLOCKED_RETRACE_MAX_M)
        intrusion = self._footprint_intrusion(found=found)
        if intrusion is not None:
            done = self._retrace(retrace_m, pause_monitor=True, found=found)
            if done is None and intrusion == 'rear' and not ahead:
                return self._escape_forward(found, pause_monitor=True)
            return bool(done)
        if ahead:
            back_m = PATH_BLOCKED_BACKOFF_M * max(1, wait)
            if self._blocked_behind(band_m=back_m + 0.05, found=found) is not False:
                return bool(self._retrace(retrace_m, pause_monitor=False, found=found))
            return self._drive_straight(-back_m, 'path_blocked_back_off')
        return self._escape_forward(found)

    def _escape_forward(self, found, pause_monitor=False):
        """Drive straight forward until nothing behind the axle is in the turn circle."""
        needed_m = self._rear_swing_need(found=found)
        if needed_m <= 0.0:
            return False
        distance_m = min(max(needed_m + 0.03, PATH_BLOCKED_FORWARD_MIN_M),
                         PATH_BLOCKED_FORWARD_MAX_M)
        if self._blocked_ahead(band_m=distance_m + 0.05, found=found) is not False:
            self.emit('path_blocked_escape_forward', done=False,
                      reason='band ahead not clear', distance_m=distance_m)
            return False
        return self._drive_straight(distance_m, 'path_blocked_escape_forward',
                                    pause_monitor=pause_monitor, found=found)

    def _retrace(self, distance_m, event='path_blocked_retrace', pause_monitor=True,
                 found=None):
        """
        Reverse along the odom trail of the last distance_m the base drove forward.

        The trail is the way the base came in, walked back only while every step
        leads backwards (trail_retrace_points), and the whole way back must be clear
        of the snapshot's returns and must not push one already inside the body
        (path_sweep_hit). With pause_monitor (something touches the body and holds
        every Collision Monitor polygon) the monitor is switched off for this move
        alone and always switched back on; if it cannot be, the emergency stop is
        latched and the run stops.

        Returns None when it refused before moving, else whether the move finished.
        """
        try:
            current = self._odom_pose()
        except RuntimeError as error:
            self.emit(event, done=False, reason=str(error))
            return None
        points, length_m = trail_retrace_points(
            current, getattr(self, 'odom_trail', None) or (), distance_m,
            RETRACE_MAX_GAP_M, RETRACE_MAX_TURN_RAD)
        if len(points) < 2:
            self.emit(event, done=False, reason='no odom trail')
            return None
        found = found or self._scan_in_base()
        if found is None:
            self.emit(event, done=False, reason='no fresh scan')
            return None
        scan, laser, outline, memory = found
        if path_sweep_hit([*scan_points(scan, laser), *memory], outline,
                          [to_base_frame(current, point) for point in points]):
            self.emit(event, done=False, reason='trail not clear',
                      distance_m=round(length_m, 3))
            return None
        path = self._odom_path(points)
        if pause_monitor and not self._pause_collision_monitor(event):
            return None
        done, restored = False, True
        try:
            done = self._execute_reverse_once(
                path, goal_checker_id='dock_position_checker', final=False,
                controller_id='ParkingReverse', motion='retrace')
        finally:
            if pause_monitor:
                restored = self._resume_collision_monitor()
        self.emit(event, done=bool(done) and restored, distance_m=round(length_m, 3),
                  monitor_paused=pause_monitor, monitor_restored=restored)
        return bool(done) and restored

    def _pause_collision_monitor(self, event):
        """Switch the Collision Monitor off for one escape; on failure switch it back on."""
        self._monitor_confirmed_on = False
        if self._set_collision_monitor(False):
            return True
        # A timed-out request may still land late.
        restored = self._resume_collision_monitor()
        self.emit(event, done=False, reason='collision monitor not paused',
                  monitor_restored=restored)
        return False

    def _resume_collision_monitor(self):
        """
        Switch the Collision Monitor back on; latch the emergency stop if it stays off.

        A monitor that stays off would leave every later goal unprotected, so the
        run is stopped as well; the next run confirms the monitor on first.
        """
        restored = any(self._set_collision_monitor(True) for _attempt in range(3))
        if restored:
            self._monitor_confirmed_on = True
        else:
            self.engage_emergency_stop('Collision Monitor could not be switched back on')
            self.request_stop()
        return restored

    def _confirm_collision_monitor_on(self):
        """
        Switch the Collision Monitor on once per executor before its first motion.

        A process that died while the monitor was paused for a retrace would
        otherwise leave every later run without it.
        """
        if getattr(self, '_monitor_confirmed_on', False):
            return
        if not self._set_collision_monitor(True):
            raise RuntimeError('Collision Monitor could not be confirmed on')
        self._monitor_confirmed_on = True

    def _set_collision_monitor(self, enabled):
        """Switch the Collision Monitor on or off; True once it confirmed."""
        client = getattr(self, 'monitor_toggle', None)
        if client is None:
            client = self.create_client(Toggle, '/collision_monitor/toggle')
            self.monitor_toggle = client
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        request = Toggle.Request()
        request.enable = enabled
        # Not self._wait: switching back on must not give way to a stop request.
        future = client.call_async(request)
        deadline_s = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline_s:
            spin_node(self, timeout_sec=0.05)
        if not future.done():
            client.remove_pending_request(future)
            return False
        if future.exception() is not None:
            return False
        return bool(future.result().success)

    def _drive_straight(self, distance_m, event, pause_monitor=False, found=None):
        """
        Drive distance_m along the heading with zero turn rate (negative: reverse).

        With pause_monitor the Collision Monitor is off for this move alone, as in
        _retrace; the caller has checked the way on the snapshot found, and every
        fresh scan during the move must keep ESCAPE_WATCH_BAND_M in the travel
        direction clear in its place (review 2026-10-06: someone stepping in front
        during the paused move went unseen).
        """
        watch = None
        if pause_monitor:
            if found is None:
                self.emit(event, done=False, reason='no snapshot to watch the way')
                return False
            watch = self._band_watch(found, distance_m)
            if not self._pause_collision_monitor(event):
                return False
        travelled_m, restored = None, True
        try:
            travelled_m = self._drive_zero_turn(distance_m, watch=watch)
        finally:
            if pause_monitor:
                restored = self._resume_collision_monitor()
        done = (travelled_m is not None and restored
                and math.copysign(travelled_m, distance_m) == travelled_m
                and abs(travelled_m) >= abs(distance_m) - STRAIGHT_ESCAPE_TOLERANCE_M)
        fields = {'monitor_paused': True, 'monitor_restored': restored} if pause_monitor else {}
        self.emit(event, done=done, distance_m=abs(distance_m),
                  travelled_m=None if travelled_m is None else round(travelled_m, 3), **fields)
        return done

    def _band_watch(self, found, distance_m):
        """
        Return a check of the latest scan: a reason to stop, or None while the way is clear.

        The laser pose and outline come from the snapshot (no parameter reads inside
        the loop); the scan must stay fresh.
        """
        _scan, laser, outline, _memory = found

        def watch():
            scan = getattr(self, 'last_scan', None)
            received_s = (getattr(self, 'samples', None) or {}).get('scan')
            if (scan is None or received_s is None
                    or time.monotonic() - received_s > ESCAPE_SCAN_MAX_AGE_S):
                return 'no fresh scan while the monitor is paused'
            if straight_sweep_hit(scan, laser, outline,
                                  math.copysign(ESCAPE_WATCH_BAND_M, distance_m)):
                return 'way not clear while the monitor is paused'
            return None
        return watch

    def _escape_publisher(self):
        publisher = getattr(self, 'escape_velocity', None)
        if publisher is None:
            publisher = self.create_publisher(Twist, 'cmd_vel_nav', 10)
            self.escape_velocity = publisher
        return publisher

    def _direct_motion(self, command, done, measure, deadline_s, event, watch=None,
                       confirm_stopped=False, pause_on_failure=None):
        """
        Publish command until done(measure()) or a stop, then stop and let the base settle.

        The command goes in ahead of the velocity smoother and the Collision Monitor,
        like the controller's. It ends on a stop request, the deadline, a failed
        fail-closed input (stale scan or odom, latched emergency stop) or a reason
        from watch, both checked every DIRECT_GUARD_PERIOD_S. Returns the last
        measurement once the base is at rest (odom speed under 0.01) or
        DIRECT_SETTLE_S has passed.
        """
        setup_started_s = time.monotonic()
        publisher = self._escape_publisher()
        self._last_direct_motion_failure = None
        previous_pause = getattr(self, '_input_recovery_paused', False)
        # Publisher discovery may have blocked callbacks before the first
        # measurement. Drain pending inputs before judging their freshness.
        input_deadline_s = time.monotonic() + 0.02
        for _ in range(32):
            if self.stop_requested or time.monotonic() >= input_deadline_s:
                break
            spin_node(self, timeout_sec=0.0)
        value = measure()
        # DDS publisher creation precedes motion and can exceed a short move's
        # budget. Keep that budget for motion; the task deadline stays fixed.
        setup_s = time.monotonic() - setup_started_s
        deadline_s += setup_s
        task_deadline_s = getattr(self, 'run_deadline_s', None)
        if task_deadline_s is not None:
            deadline_s = min(deadline_s, task_deadline_s)
        if setup_s >= 0.1:
            self.emit('direct_motion_ready', setup_s=setup_s,
                      motion_budget_s=max(0.0, deadline_s - time.monotonic()))
        published_s, guarded_s = 0.0, -math.inf
        # The velocity smoother ramps to zero after velocity_timeout (0.5 s) without
        # a command, so a gap that long is a stop no event reports.
        max_publish_gap_s = 0.0
        settle_pause_started_s = None
        input_recovered = False

        def finish_settle_pause():
            nonlocal settle_pause_started_s
            if settle_pause_started_s is None:
                return
            settle_pause_s = max(0.0, time.monotonic() - settle_pause_started_s)
            task_deadline_s = getattr(self, 'run_deadline_s', None)
            if (isinstance(task_deadline_s, (int, float))
                    and not isinstance(task_deadline_s, bool)
                    and math.isfinite(task_deadline_s)):
                self.run_deadline_s = task_deadline_s + settle_pause_s
            self._input_recovery_paused = previous_pause
            settle_pause_started_s = None

        try:
            try:
                while (not done(value) and not self.stop_requested
                       and time.monotonic() < deadline_s):
                    now_s = time.monotonic()
                    if now_s - guarded_s >= DIRECT_GUARD_PERIOD_S:
                        guarded_s = now_s
                        failure = (self._guard_failure(False)
                                   or (None if self._operator_link_ready()
                                       else 'operator_link_lost')
                                   or (watch() if watch else None))
                        if failure:
                            if (not input_recovered and self._input_gap_recoverable(failure)
                                    and self._operator_link_ready()
                                    and not self.stop_requested):
                                publisher.publish(Twist())
                                paused_s = time.monotonic()
                                if self._wait_for_input_recovery(failure):
                                    pause_s = time.monotonic() - paused_s
                                    deadline_s += pause_s
                                    if getattr(self, 'run_deadline_s', None) is not None:
                                        self.run_deadline_s += pause_s
                                    input_recovered = True
                                    self.emit('direct_motion_input_recovered', reason=failure,
                                              paused_s=pause_s, target_preserved=True)
                                    value = measure()
                                    continue
                            if (pause_on_failure is not None and pause_on_failure(failure)
                                    and not getattr(self, '_input_recovery_paused', False)):
                                settle_pause_started_s = time.monotonic()
                                self._input_recovery_paused = True
                            self._last_direct_motion_failure = failure
                            self.emit(event, reason=failure,
                                      max_publish_gap_s=max_publish_gap_s)
                            break
                    if now_s - published_s >= 0.05:
                        if published_s:
                            max_publish_gap_s = max(max_publish_gap_s, now_s - published_s)
                        publisher.publish(command)
                        published_s = time.monotonic()
                    # Drain inputs over this control interval before running the
                    # geometry again; one callback per measurement can leave TF,
                    # depth and odometry queued behind expensive path previews.
                    input_deadline_s = time.monotonic() + 0.02
                    while not self.stop_requested:
                        input_remaining_s = input_deadline_s - time.monotonic()
                        if input_remaining_s <= 0.0:
                            break
                        spin_node(self, timeout_sec=min(0.002, input_remaining_s))
                    value = measure()
                if self.stop_requested:
                    self._last_direct_motion_failure = 'operator_stop'
                elif (time.monotonic() >= deadline_s and not done(value)
                        and not getattr(self, '_input_recovery_paused', False)):
                    self._last_direct_motion_failure = 'direct_motion_timeout'
                    self.emit(event, reason='direct_motion_timeout',
                              max_publish_gap_s=max_publish_gap_s)
            finally:
                for _repeat in range(5):
                    publisher.publish(Twist())
                    spin_node(self, timeout_sec=0.02)
        except BaseException:
            finish_settle_pause()
            if not previous_pause:
                self._input_recovery_paused = False
            raise
        stopped_s = time.monotonic()
        stationary_since_s = None
        stationary_samples = 0
        fresh_motion_samples = 0
        last_motion_stamp_s = None
        stopped_confirmed = not confirm_stopped
        zero_published_s = 0.0
        stop_contract = getattr(self, 'parking_contract', None) or {
            'stopped_linear_mps': 0.01, 'stopped_angular_radps': 0.01}
        stopped_linear_mps = stop_contract['stopped_linear_mps']
        stopped_angular_radps = stop_contract['stopped_angular_radps']
        # The smoother can still be decelerating at the old 0.5 s boundary.
        # Keep commanding zero during that bounded grace period.  Confirm the
        # stop from consecutive fresh odometry samples as well as elapsed time;
        # otherwise a truly stopped base can fail merely because the first
        # below-threshold sample arrived at the end of the settle window.
        settle_deadline_s = (stopped_s + DIRECT_SETTLE_S
                             + (DIRECT_STOP_DECEL_GRACE_S if confirm_stopped else 0.0))
        try:
            while time.monotonic() < settle_deadline_s:
                now_s = time.monotonic()
                if now_s - zero_published_s >= 0.05:
                    publisher.publish(Twist())
                    zero_published_s = now_s
                motion = getattr(self, 'latest_motion', None)
                fresh_sample = (motion is not None and motion[0] > stopped_s
                                and motion[0] != last_motion_stamp_s)
                if fresh_sample:
                    last_motion_stamp_s = motion[0]
                    fresh_motion_samples += 1
                if (motion is not None and motion[0] > stopped_s
                        and abs(motion[1]) <= stopped_linear_mps
                        and abs(motion[2]) <= stopped_angular_radps):
                    if not confirm_stopped:
                        stopped_confirmed = True
                        break
                    if stationary_since_s is None:
                        stationary_since_s = motion[0]
                    if fresh_sample:
                        stationary_samples += 1
                    if (stationary_samples >= DIRECT_STOP_CONFIRM_SAMPLES
                            or motion[0] - stationary_since_s >= DIRECT_STOP_CONFIRM_S):
                        stopped_confirmed = True
                        break
                elif fresh_sample:
                    stationary_since_s = None
                    stationary_samples = 0
                spin_node(self, timeout_sec=0.02)
        finally:
            finish_settle_pause()
        if not stopped_confirmed:
            self._last_direct_motion_failure = 'odometry did not confirm stop'
            motion = getattr(self, 'latest_motion', None)
            self.emit(event, reason='odometry did not confirm stop',
                      fresh_odom_samples=fresh_motion_samples,
                      last_odom_age_s=(None if motion is None else
                                       max(0.0, time.monotonic() - motion[0])),
                      last_odom_linear_mps=(None if motion is None else motion[1]),
                      last_odom_angular_radps=(None if motion is None else motion[2]),
                      stopped_linear_mps=stopped_linear_mps,
                      stopped_angular_radps=stopped_angular_radps,
                      max_publish_gap_s=max_publish_gap_s)
            return None
        return measure()

    def _rotate_in_place(self, delta_rad, *, pause_on_failure=None):
        """
        Turn without translation, reducing the rate for small corrections.

        Same path as the controller (smoother, Collision Monitor); the stop is
        sent early by the smoother's deceleration and the stop latency. Returns the
        signed turn measured in odom once the base is at rest, or None without odom.
        """
        base_frame = self.parking_contract['robot_base_frame']

        def odom_yaw():
            return _quaternion_yaw(self.parking_tf.lookup_transform(
                'odom', base_frame, rclpy.time.Time()).transform.rotation)

        try:
            start = odom_yaw()
        except TransformException:
            return None
        try:
            collision_watch = rotation_collision_watch(self)
        except (RuntimeError, ValueError, TransformException) as error:
            self._last_direct_motion_failure = str(error)
            self.emit('trim_rotation_stopped', reason=str(error))
            return None
        cruise = abs(delta_rad) > STAGING_CRUISE_THRESHOLD_RAD
        command = Twist()
        # Unwrapped, so a turn of any size is counted.
        state = {'last': start, 'turned': 0.0}

        def measure():
            try:
                yaw = odom_yaw()
            except TransformException:
                return state['turned']
            state['turned'] += math.atan2(math.sin(yaw - state['last']),
                                          math.cos(yaw - state['last']))
            state['last'] = yaw
            if cruise:
                remaining = max(0.0, abs(delta_rad) - abs(state['turned']))
                rate = min(STAGING_ANGULAR_RADPS, max(
                    TRIM_ANGULAR_RADPS, remaining / FACE_APPROACH_TIME_S))
                command.angular.z = math.copysign(rate, delta_rad)
            return state['turned']

        # All face and dock trims share this damping; callers cannot bypass it.
        speed_radps = min(TRIM_ANGULAR_RADPS, max(
            PRECISION_HEADING_MAX_RADPS,
            TRIM_ANGULAR_RADPS * abs(delta_rad) / (2.0 * TRIM_STOP_RAD)))
        if cruise:
            speed_radps = STAGING_ANGULAR_RADPS
        command.angular.z = math.copysign(speed_radps, delta_rad)
        # At most half the turn: a trim just over the minimum must still turn.
        early_rad = min(TRIM_STOP_RAD * min(speed_radps, TRIM_ANGULAR_RADPS)
                        / TRIM_ANGULAR_RADPS,
                        abs(delta_rad) / 2.0)
        deadline_s = time.monotonic() + abs(delta_rad) / speed_radps + TRIM_EXTRA_S
        turned_rad = self._direct_motion(
            command, lambda turned: abs(turned) >= abs(delta_rad) - early_rad, measure,
            deadline_s, 'trim_rotation_stopped', watch=lambda: collision_watch(command),
            **({'pause_on_failure': pause_on_failure}
               if pause_on_failure is not None else {}))
        return None if getattr(self, '_last_direct_motion_failure', None) else turned_rad

    def _drive_zero_turn(self, distance_m, watch=None, precision=False,
                         hold_heading=False, target_update=None, command_watch=None,
                         approach_time_s=None, continuous_table=False):
        """Run one straight move and restore the caller's deadline-pause state."""
        previous_pause = getattr(self, '_input_recovery_paused', False)
        try:
            return self._drive_zero_turn_impl(
                distance_m, watch=watch, precision=precision,
                hold_heading=hold_heading, target_update=target_update,
                command_watch=command_watch, approach_time_s=approach_time_s,
                continuous_table=continuous_table)
        finally:
            self._input_recovery_paused = previous_pause

    def _drive_zero_turn_impl(self, distance_m, watch=None, precision=False,
                              hold_heading=False, target_update=None, command_watch=None,
                              approach_time_s=None, continuous_table=False):
        """
        Command (+-STRAIGHT_ESCAPE_SPEED_MPS, 0) until odom shows distance_m.

        The stop is sent early by the smoother's deceleration and the stop latency.
        Precision moves lower their speed with the requested distance and update
        that early-stop point from fresh odometry speed and the observed loop delay.
        Heading hold feeds odometry yaw back into a bounded angular command. A live
        face may supply heading_rad as well as remaining_m; its filtered normal
        replaces the starting heading and slows translation until alignment settles.
        Both use the same velocity-smoother, Collision Monitor and guard path.
        It runs only between goals. Returns the signed travel along the start heading
        once the base is at rest, or None without odom.
        """
        previous_pause = getattr(self, '_input_recovery_paused', False)
        if target_update is not None and not (precision and hold_heading and distance_m != 0.0):
            raise ValueError('live approach requires nonzero precision heading hold')
        if continuous_table and (target_update is None or distance_m <= 0.):
            raise ValueError('continuous table motion requires forward live face feedback')
        if approach_time_s is None:
            approach_time_s = (FACE_APPROACH_TIME_S if target_update is not None
                               else PRECISION_STRAIGHT_TARGET_S)
        if (isinstance(approach_time_s, bool) or not math.isfinite(approach_time_s)
                or approach_time_s <= 0.0):
            raise ValueError('invalid approach time')
        direction = math.copysign(1.0, distance_m)
        base_frame = self.parking_contract['robot_base_frame']

        def odom_pose():
            transform = self.parking_tf.lookup_transform(
                'odom', base_frame, rclpy.time.Time()).transform
            return (transform.translation.x, transform.translation.y,
                    _quaternion_yaw(transform.rotation))

        try:
            start = odom_pose()
        except TransformException:
            return None
        state = {'travelled': 0.0, 'tf_failure': None,
                 'target_failure': None, 'remaining_m': abs(distance_m),
                 'target_reached': False, 'face_heading': None, 'heading_updated_s': None,
                 'validate_control': None, 'allow_near_alignment': False,
                 'alignment_in_progress': False, 'observed_heading': None,
                 'surface_gap_m': None, 'target_confirmed': False,
                 'lateral_tracking': False, 'face_normal_heading': None,
                 'lateral_error_m': None, 'lateral_tolerance_m': None,
                 'target_gap_m': 0.01}
        command = Twist()

        def measure():
            try:
                x_m, y_m, yaw = odom_pose()
            except TransformException:
                if hold_heading:
                    state['tf_failure'] = 'odom TF unavailable during heading hold'
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                return state['travelled']
            state['travelled'] = ((x_m - start[0]) * math.cos(start[2])
                                  + (y_m - start[1]) * math.sin(start[2]))
            if (target_update is not None and state['target_failure'] is None
                    and not state['target_reached']):
                try:
                    update = target_update((x_m, y_m, yaw))
                    remaining = update['remaining_m']
                    if (isinstance(remaining, bool)
                            or not isinstance(remaining, (int, float))
                            or not math.isfinite(remaining)):
                        raise RuntimeError('invalid live approach distance')
                    state['remaining_m'] = remaining
                    confirmed = update.get('target_confirmed', False)
                    if not isinstance(confirmed, bool):
                        raise RuntimeError('invalid live target confirmation')
                    state['target_confirmed'] = confirmed
                    state['surface_gap_m'] = update.get('surface_gap_m')
                    lateral_tracking = update.get('lateral_tracking', False)
                    if not isinstance(lateral_tracking, bool):
                        raise RuntimeError('invalid lateral tracking mode')
                    face_heading = update.get('face_heading_rad')
                    if lateral_tracking and (
                            not continuous_table or isinstance(face_heading, bool)
                            or not isinstance(face_heading, (int, float))
                            or not math.isfinite(face_heading)):
                        raise RuntimeError('lateral tracking requires a finite live face heading')
                    state['lateral_tracking'] = lateral_tracking
                    state['face_normal_heading'] = face_heading
                    lateral_error = update.get('lateral_error_m')
                    lateral_tolerance = update.get('lateral_tolerance_m')
                    target_gap = update.get('target_gap_m', 0.01)
                    if lateral_error is not None:
                        if (not lateral_tracking or not continuous_table
                                or any(isinstance(v, bool)
                                       or not isinstance(v, (int, float))
                                       or not math.isfinite(v)
                                       for v in (lateral_error, lateral_tolerance, target_gap))
                                or lateral_tolerance <= 0.0 or target_gap <= 0.0):
                            raise RuntimeError('invalid lateral tracking error')
                    state['lateral_error_m'] = lateral_error
                    state['lateral_tolerance_m'] = lateral_tolerance
                    state['target_gap_m'] = target_gap
                    allow_near = update.get('allow_near_alignment', False)
                    if not isinstance(allow_near, bool):
                        raise RuntimeError('invalid near alignment mode')
                    state['allow_near_alignment'] = allow_near
                    next_speed = max(PRECISION_STRAIGHT_MIN_SPEED_MPS,
                                     remaining / approach_time_s)
                    if 'heading_rad' in update:
                        heading = update['heading_rad']
                        if (isinstance(heading, bool) or not isinstance(heading, (int, float))
                                or not math.isfinite(heading)):
                            raise RuntimeError('invalid live approach heading')
                        now_s = time.monotonic()
                        elapsed_s = (0.0 if state['heading_updated_s'] is None
                                     else max(0.0, now_s - state['heading_updated_s']))
                        state['face_heading'] = filtered_face_heading(
                            state['face_heading'], heading, elapsed_s)
                        state['observed_heading'] = heading
                        state['heading_updated_s'] = now_s
                        state['validate_control'] = update.get('validate_control')
                        if (state['validate_control'] is not None
                                and not callable(state['validate_control'])):
                            raise RuntimeError('invalid table control validator')
                    else:
                        command.linear.x = direction * min(abs(command.linear.x), next_speed)
                except (KeyError, TypeError, ValueError, RuntimeError) as error:
                    state['target_failure'] = str(error) or type(error).__name__
                    if state['target_failure'] in recoverable_target_failures:
                        if state.get('recovery_paused_s') is None:
                            state['recovery_paused_s'] = time.monotonic()
                        self._input_recovery_paused = True
                    state['resume_speed_mps'] = command.linear.x
                    command.linear.x = command.angular.z = 0.0
            if hold_heading and state['target_failure'] is None:
                if state['target_confirmed']:
                    command.linear.x = command.angular.z = 0.0
                    state['alignment_in_progress'] = False
                    return state['travelled']
                heading = (start[2] if state['face_heading'] is None
                           else state['face_heading'])
                error_rad = math.atan2(
                    math.sin(heading - yaw), math.cos(heading - yaw))
                raw_error_rad = (error_rad if state['observed_heading'] is None else math.atan2(
                    math.sin(state['observed_heading'] - yaw),
                    math.cos(state['observed_heading'] - yaw)))
                geometry_error_rad = (
                    math.atan2(math.sin(state['face_normal_heading'] - yaw),
                               math.cos(state['face_normal_heading'] - yaw))
                    if state['lateral_tracking'] else raw_error_rad)
                state['alignment_in_progress'] = (
                    state['allow_near_alignment']
                    and (abs(geometry_error_rad) if state['lateral_tracking'] else
                         max(abs(error_rad), abs(raw_error_rad))) > FACE_HEADING_TOLERANCE_RAD)
                command.angular.z = max(
                    -PRECISION_HEADING_MAX_RADPS,
                    min(PRECISION_HEADING_MAX_RADPS,
                        PRECISION_HEADING_KP * error_rad))
                if state['face_heading'] is not None:
                    try:
                        if state['validate_control'] is not None:
                            state['validate_control']((x_m, y_m, yaw), heading)
                        if state['lateral_error_m'] is not None:
                            linear, angular = lateral_face_approach_command(
                                state['remaining_m'], geometry_error_rad,
                                state['lateral_error_m'], state['surface_gap_m'],
                                state['lateral_tolerance_m'],
                                target_gap_m=state['target_gap_m'],
                                allow_near_alignment=state['allow_near_alignment'])
                        else:
                            linear, angular = face_approach_command(
                                state['remaining_m'], error_rad, approach_time_s=approach_time_s,
                                allow_near_alignment=state['allow_near_alignment'],
                                **({'continuous_table': True, 'surface_gap_m': state['surface_gap_m']}
                                   if continuous_table else {}),
                                **({'lateral_tracking': True, 'face_error_rad': geometry_error_rad}
                                   if state['lateral_tracking'] else {}))
                        command.linear.x = direction * linear
                        command.angular.z = angular
                        if (getattr(getattr(self, 'station_reference', None),
                                    'observation_method', None) == 'table_front_tape'):
                            # User-requested slow tape alignment, then a slower
                            # final approach to the already accepted odom goal.
                            cap = (.025 if getattr(self, '_station_terminal_goal', None)
                                   is not None else .04)
                            if abs(command.linear.x) > cap:
                                ratio = cap / abs(command.linear.x)
                                command.linear.x *= ratio
                                command.angular.z *= ratio
                        if (state['lateral_error_m'] is None and state['allow_near_alignment']
                                and state['remaining_m'] <= FACE_ALIGNMENT_GAP_M - .01):
                            if state['alignment_in_progress']:
                                command.linear.x = 0.
                            if ((not state['lateral_tracking'] or state['alignment_in_progress'])
                                    and command.angular.z * geometry_error_rad <= 0.):
                                command.angular.z = 0.
                    except (ValueError, RuntimeError) as error:
                        state['target_failure'] = str(error)
                        command.linear.x = command.angular.z = 0.0
            return state['travelled']

        speed_mps = TABLE_APPROACH_SPEED_MPS if continuous_table else STRAIGHT_ESCAPE_SPEED_MPS
        if precision:
            speed_mps = min(speed_mps, max(
                PRECISION_STRAIGHT_MIN_SPEED_MPS,
                abs(distance_m) / approach_time_s))
        command.linear.x = math.copysign(speed_mps, distance_m)
        timing = {'checked_s': None, 'loop_s': 0.02}

        def stopping_distance_m():
            now_s = time.monotonic()
            if timing['checked_s'] is not None:
                timing['loop_s'] = min(0.1, max(0.0, now_s - timing['checked_s']))
            timing['checked_s'] = now_s
            live_speed_mps = abs(command.linear.x) or speed_mps
            motion = getattr(self, 'latest_motion', None)
            odom_age_s = 0.0
            if motion is not None and all(math.isfinite(value) for value in motion):
                odom_age_s = min(0.1, max(0.0, now_s - motion[0]))
                if odom_age_s < 0.1 and abs(motion[1]) >= 0.002:
                    live_speed_mps = abs(motion[1])
            reaction_s = DIRECT_STOP_LATENCY_S + timing['loop_s'] + odom_age_s
            return (live_speed_mps ** 2 / (2.0 * STRAIGHT_DECEL_MPS2)
                    + live_speed_mps * reaction_s)

        def reached_stop_point(travelled):
            if state['tf_failure'] is not None or state['target_failure'] is not None:
                return False
            if target_update is not None:
                if state['target_confirmed']:
                    state['target_reached'] = True
                    return True
                if state['alignment_in_progress']:
                    return False
                reached = state['remaining_m'] <= stopping_distance_m()
                state['target_reached'] = reached
                return reached
            if not precision:
                early_m = min(
                    STRAIGHT_ESCAPE_SPEED_MPS ** 2 / (2.0 * STRAIGHT_DECEL_MPS2)
                    + STRAIGHT_ESCAPE_SPEED_MPS * DIRECT_STOP_LATENCY_S,
                    abs(distance_m) / 2.0)
                return abs(travelled) >= abs(distance_m) - early_m
            early_m = min(stopping_distance_m(), abs(distance_m) / 2.0)
            return abs(travelled) >= abs(distance_m) - early_m

        def motion_watch():
            failure = state['tf_failure'] or state['target_failure']
            if failure is None and command_watch is not None:
                failure = command_watch(command)
                if failure:
                    state['target_failure'] = failure
                    state['resume_speed_mps'] = command.linear.x
                    command.linear.x = command.angular.z = 0.0
            if failure in recoverable_target_failures:
                if state.get('recovery_paused_s') is None:
                    state['recovery_paused_s'] = time.monotonic()
                self._input_recovery_paused = True
            return failure if failure is not None else (watch() if watch else None)

        if target_update is not None:
            # Remaining-distance taper and fresh depth corrections share one move.
            deadline_s = (time.monotonic()
                          + LIVE_APPROACH_MAX_M / PRECISION_STRAIGHT_MIN_SPEED_MPS
                          + STRAIGHT_ESCAPE_EXTRA_S)
        else:
            deadline_s = (time.monotonic() + abs(distance_m) / speed_mps
                          + STRAIGHT_ESCAPE_EXTRA_S)
        recoverable_depth_failures = {
            'table approach stamped odom TF unavailable',
            'table approach observer silent',
            'table approach depth stale or future-dated',
            'table approach observer latency',
            'table approach tape reference pending',
            'table approach stable face unavailable',
        }
        recoverable_rear_failures = {
            'rear box scan unavailable', 'rear box scan stale or future-dated',
            'rear box stamped odom TF unavailable', 'rear box face unavailable',
            'rear box target changed', 'rear docking swept footprint occupied',
        }
        recoverable_target_failures = recoverable_depth_failures | recoverable_rear_failures
        while True:
            travelled_m = self._direct_motion(
                command, reached_stop_point, measure, deadline_s, 'straight_drive_stopped',
                watch=motion_watch if hold_heading or target_update is not None else watch,
                confirm_stopped=precision)
            if (travelled_m is None and not self.stop_requested
                    and target_update is not None
                    and state['target_failure'] in recoverable_target_failures
                    and getattr(self, '_last_direct_motion_failure', None)
                    == 'odometry did not confirm stop'):
                self.emit('precision_input_pause_waiting_for_stop',
                          reason=state['target_failure'], target_preserved=True)
                try:
                    self.capture_stationary_pose(intermediate=True)
                except (RuntimeError, ValueError, TransformException):
                    break
                travelled_m = state['travelled']
                self._last_direct_motion_failure = None
            if (target_update is None or travelled_m is None or self.stop_requested
                    or state['tf_failure'] is not None
                    or state['target_failure'] not in recoverable_target_failures):
                break
            reason = state['target_failure']
            resume_speed = state.get('resume_speed_mps', speed_mps)
            paused_s = state.pop('recovery_paused_s', None) or time.monotonic()
            rear_recovery = reason in recoverable_rear_failures
            event_prefix = 'rear_box' if rear_recovery else 'table_approach'
            self.emit(f'{event_prefix}_waiting_for_input', reason=reason,
                      travelled_m=state['travelled'], target_preserved=True)
            # A temporary depth gap pauses this same odom-frozen move. No goal,
            # nonzero velocity or changed stop point is sent while inputs are bad.
            self._input_recovery_paused = True
            try:
                while not self.stop_requested:
                    self._escape_publisher().publish(Twist())
                    spin_node(self, timeout_sec=0.05)
                    if not self._operator_link_ready():
                        state['target_failure'] = 'operator_link_lost'
                        break
                    guard_failure = self._guard_failure(False)
                    if guard_failure:
                        if self._input_gap_recoverable(guard_failure):
                            continue
                        state['target_failure'] = guard_failure
                        break
                    state['target_failure'] = None
                    command.linear.x = resume_speed
                    measure()
                    if state['tf_failure'] is not None:
                        break
                    if (rear_recovery and state['target_failure'] is None
                            and reached_stop_point(state['travelled'])):
                        # Arrival needs no new motion preview. Valid feedback and the
                        # normal stopped-pose confirmation still precede success.
                        command.linear.x = command.angular.z = 0.0
                        self.emit('rear_box_input_recovered',
                                  paused_s=time.monotonic() - paused_s,
                                  travelled_m=state['travelled'], target_preserved=True,
                                  arrival_reached=True)
                        break
                    if state['target_failure'] is None and command_watch is not None:
                        state['target_failure'] = command_watch(command)
                        if state['target_failure']:
                            command.linear.x = command.angular.z = 0.0
                    if state['target_failure'] is None:
                        self.emit(f'{event_prefix}_input_recovered',
                                  paused_s=time.monotonic() - paused_s,
                                  travelled_m=state['travelled'], target_preserved=True)
                        break
                    if state['target_failure'] not in recoverable_target_failures:
                        break
            finally:
                # measure() may see further gaps while already waiting. They
                # belong to this pause, not to the next recovery interval.
                state.pop('recovery_paused_s', None)
                self._input_recovery_paused = previous_pause
            if self.stop_requested or state['tf_failure'] or state['target_failure']:
                command.linear.x = command.angular.z = 0.0
                break
            paused_duration_s = time.monotonic() - paused_s
            deadline_s += paused_duration_s
            if getattr(self, 'run_deadline_s', None) is not None:
                self.run_deadline_s += paused_duration_s
            if state['target_reached']:
                travelled_m = state['travelled']
                break
            timing['checked_s'] = None
        # Recovery can discover a different terminal failure without entering
        # _direct_motion again. Propagate that cause instead of the old pause.
        if state.get('recovery_paused_s') is not None:
            self._input_recovery_paused = previous_pause
        if not self.stop_requested:
            failure = state['tf_failure'] or state['target_failure']
            if failure:
                self._last_direct_motion_failure = failure
        if precision:
            self.emit('precision_straight_finished',
                      requested_distance_m=distance_m,
                      travelled_m=travelled_m, speed_mps=speed_mps,
                      loop_s=timing['loop_s'],
                      odom_tf_failed=state['tf_failure'] is not None,
                      target_failure=state['target_failure'])
        failed = (state['tf_failure'] or state['target_failure']
                  or (target_update is not None
                      and (self.stop_requested or not state['target_reached'])))
        return None if failed else travelled_m

    def _departure_ready(self):
        """Check, before every goal, the battery reserve and the operator link."""
        return self._departure_battery_ready() and self._operator_link_ready()

    def _operator_link_ready(self):
        """
        Start no further goal once the operator PC has stopped refreshing its heartbeat.

        Without a physical emergency stop the PC is the only remote stop, so a lost
        link ends the cycle where the current goal ends (VDA 5050: the released part is
        finished, then the vehicle stops). The robot is not sent home: going home is
        unsupervised driving as well. A returning link does not resume it either.
        """
        heartbeat = getattr(self, 'operator_heartbeat', None)
        if heartbeat is None:
            return True
        try:
            age_s = time.time() - Path(heartbeat).stat().st_mtime
        except OSError:
            age_s = None
        limit_s = getattr(self, 'operator_link_timeout_s', OPERATOR_LINK_TIMEOUT_S)
        if age_s is not None and age_s <= limit_s:
            return True
        self.last_readiness_failure = 'operator_link_lost'
        self.emit('departure_blocked', reason='operator_link_lost',
                  heartbeat_age_s=age_s, limit_s=limit_s)
        return False

    def battery_return_due(self):
        """Below the departure reserve between stops: skip the rest and dock."""
        voltage_v = self.battery_voltage
        reserve_v = self.service_contract['minimum_start_battery_v']
        if voltage_v is not None and math.isfinite(voltage_v) and voltage_v >= reserve_v:
            return False
        self.emit('battery_return', reason='battery below the departure reserve',
                  voltage_v=voltage_v if voltage_v is not None and math.isfinite(voltage_v)
                  else None, minimum_start_battery_v=reserve_v)
        return True

    def _departure_battery_ready(self):
        """
        Require starting reserve once per executor, running cutoff thereafter.

        A dock return needs only the running cutoff: it is the way to the charger
        (2026-10-02 18:48: 10.52 V refused the return home against the 10.8 V
        start reserve and left the base standing in the room).
        """
        voltage_v = self.battery_voltage
        running = (getattr(self, '_mission_started', False)
                   or getattr(self, 'home_return', False))
        threshold_v = (self.minimum_battery_v if running
                       else self.service_contract['minimum_start_battery_v'])
        if (voltage_v is None or not math.isfinite(voltage_v)
                or voltage_v < threshold_v):
            if voltage_v is None:
                self.last_readiness_failure = 'battery voltage missing'
            elif not math.isfinite(voltage_v):
                self.last_readiness_failure = (
                    f'battery voltage non-finite: value={voltage_v}')
            else:
                self.last_readiness_failure = (
                    f'battery departure reserve: voltage={voltage_v:.3f}V '
                    f'limit={threshold_v:.3f}V')
            self.emit('departure_blocked', reason='battery_departure_reserve',
                      voltage_v=voltage_v if voltage_v is not None
                      and math.isfinite(voltage_v) else None,
                      minimum_start_battery_v=threshold_v,
                      minimum_running_battery_v=self.minimum_battery_v)
            return False
        return True

    def wait_until_ready(self, timeout=15.0):
        """Check a departure reserve after fresh battery and sensor samples arrive."""
        return (super().wait_until_ready(timeout=timeout)
                and self._departure_ready())

    @contextmanager
    def _localization_bound(self, intermediate):
        """Select the AMCL covariance bound for one stage, then restore it."""
        previous = getattr(self, '_intermediate_localization', False)
        self._intermediate_localization = bool(intermediate)
        try:
            yield
        finally:
            self._intermediate_localization = previous

    @staticmethod
    def _input_gap_recoverable(reason):
        return (CorridorRoute._input_gap_recoverable(reason)
                or (isinstance(reason, str) and reason.startswith((
                    'stationary teaching unavailable: need fresh TF, odometry and localization',
                    NAV2_TF_ERROR_REASON))))

    def _wait_for_input_recovery(self, reason, timeout_s=8.0):
        """After a controller TF error, also wait until AMCL publishes map->odom again."""
        started_s = time.monotonic()
        if not super()._wait_for_input_recovery(reason, timeout_s):
            return False
        tf_buffer = getattr(self, 'parking_tf', None)
        if (tf_buffer is None or not isinstance(reason, str)
                or not reason.startswith(NAV2_TF_ERROR_REASON)):
            return True
        # AMCL stamps map->odom at its scan time plus transform_tolerance, so a
        # stamp ahead of the ROS time at this point is a publish newer than the
        # stall. Resending into the same stall would only repeat the 102.
        required_stamp_s = self.get_clock().now().nanoseconds * 1e-9
        while not self.stop_requested and time.monotonic() - started_s < timeout_s:
            try:
                stamp = tf_buffer.lookup_transform(
                    'odom', 'map', rclpy.time.Time()).header.stamp
                if stamp.sec + stamp.nanosec * 1e-9 >= required_stamp_s:
                    return True
            except TransformException:
                pass
            spin_node(self, timeout_sec=0.05)
        if not self.stop_requested:
            self.emit('nav2_transform_recovery_timeout', reason=reason, timeout_s=timeout_s)
        return False

    def execute(self, *, final_parking=True, alignment=False, staging=False, face=False):
        """Bound transit and parking actions and retain their terminal result."""
        if not isinstance(final_parking, bool):
            raise ValueError('final_parking must be boolean')
        if not isinstance(alignment, bool) or (alignment and final_parking):
            raise ValueError('alignment requires non-final parking mode')
        if face and not alignment:
            raise ValueError('face selects the box face alignment checker')
        if staging and (alignment or final_parking):
            raise ValueError('staging is a plain intermediate leg')
        # The 9/29 transit stopped at x covariance 0.010213 against 0.01 with no
        # resume path. A transit leg is intermediate, an alignment leg keeps its
        # caller's stage and a final leg is always strict, input recovery included.
        intermediate = not final_parking and (
            not alignment or getattr(self, '_intermediate_localization', False))
        self._dock_departure = None
        with self._localization_bound(intermediate):
            return self._run_with_input_recovery(
                lambda: self._execute_service_once(
                    final_parking=final_parking, alignment=alignment, staging=staging,
                    **({'face': True} if face else {})))

    def _execute_service_once(self, *, final_parking, alignment, staging=False, face=False):
        """Retry only a canceled input gap, never an unresolved action."""
        if not self.navigate.wait_for_server(timeout_sec=2.0):
            return False
        distance_m = self.config.get('dock_departure_distance_m')
        if (distance_m is not None and not (final_parking or alignment or staging)
                and self._resume_waypoint_index == 0):
            try:
                departed = self._depart_dock_straight(distance_m)
            except RuntimeError as error:
                reason = self._guard_failure(False) or str(error)
                if self.stop_requested:
                    return False
                if not self._input_gap_recoverable(reason):
                    raise
                self._retry_guard_reason = reason
                self.emit('dock_departure_waiting_for_input', reason=reason)
                return False
            if not departed:
                return False
            self._resume_waypoint_index = 1
        through = getattr(self, 'navigate_through', None)
        if (through is not None and not (final_parking or alignment or staging)
                and len(self.waypoints) - self._resume_waypoint_index >= 2):
            return self._execute_through_once()
        self.active_action_type = NavigateToPose
        for index, waypoint in enumerate(self.waypoints):
            if index < self._resume_waypoint_index:
                continue
            self._resume_waypoint_index = index
            if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
                reason = self._guard_failure(False)
                if not self.stop_requested and self._input_gap_recoverable(reason):
                    self._retry_guard_reason = reason
                return False
            if not self._departure_ready():
                return False
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(index, waypoint)
            final = final_parking and index == len(self.waypoints) - 1
            goal.behavior_tree = self.parking_behavior_tree if final else self.behavior_tree
            if alignment and index == len(self.waypoints) - 1:
                goal.behavior_tree = (
                    getattr(self, 'face_alignment_behavior_tree', self.alignment_behavior_tree)
                    if face else self.alignment_behavior_tree)
            if staging and index == len(self.waypoints) - 1:
                goal.behavior_tree = getattr(
                    self, 'staging_behavior_tree', self.alignment_behavior_tree)
            self.navigation_uuid = NavigateToPose.Impl.SendGoalService.Request().goal_id
            self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
            self.pending_goal = self.navigate.send_goal_async(goal, goal_uuid=self.navigation_uuid)
            try:
                handle = self._wait(self.pending_goal, 5.0)
                self.pending_goal = None
                if not handle.accepted:
                    self.emit('failed', reason='navigation_rejected')
                    return False
                self.active_handle = handle
                self._route_event('accepted', index, handle)
                self.navigation_result = handle.get_result_async()
                while not self.navigation_result.done():
                    spin_node(self, timeout_sec=0.05)
                    if (self.stop_requested
                            or not self._navigation_ready(require_fresh_amcl=False)):
                        reason = self._guard_failure(False) or 'operator_or_timeout'
                        self.emit('interrupted', reason=reason)
                        if not self.stop_requested and self._input_gap_recoverable(reason):
                            self._retry_guard_reason = reason
                        return False
                wrapped = self.navigation_result.result()
                self._route_event('result', index, handle,
                                  terminal_status_code=int(wrapped.status),
                                  nav2_error_code=int(wrapped.result.error_code))
                self.navigation_result = None
                if (self.stop_requested or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                        or wrapped.result.error_code):
                    self._tag_blocked(int(wrapped.result.error_code), final)
                    return False
                if final and not self._verify_parking_stop(index, waypoint, handle):
                    return False
            finally:
                if not self.finish_navigation():
                    raise RuntimeError('navigation cancellation unconfirmed')
        return True

    def _depart_dock_straight(self, distance_m):
        """Finish the dock exit along the stopped heading before transit may turn."""
        if self.stop_requested:
            return False
        if (isinstance(distance_m, bool) or not isinstance(distance_m, (float, int))
                or not math.isfinite(distance_m) or distance_m <= 0.0
                or len(self.waypoints) < 2 or self.waypoints[0]['id'] != 'home_exit'):
            raise ValueError('dock departure requires a positive distance and first home_exit')
        return self._advance_dock_straight(distance_m)

    def _clear_rear_dock_straight(self, distance_m):
        """Create alignment room without departure waypoints or landmark teaching."""
        saved = getattr(self, '_dock_departure', None)
        self._dock_departure = None
        try:
            return self._advance_dock_straight(distance_m, motion='rear_dock_clearance')
        finally:
            self._dock_departure = saved

    def _advance_dock_straight(self, distance_m, *, motion='dock_departure'):
        """Share the frozen straight path and collision checks for forward dock moves."""
        if self.stop_requested:
            return False
        if (isinstance(distance_m, bool) or not isinstance(distance_m, (float, int))
                or not math.isfinite(distance_m) or distance_m <= 0.0 or not self.waypoints):
            raise ValueError('forward dock motion requires a positive distance and waypoint')
        if self._dock_departure is None:
            actual, _ = self.capture_stationary_pose(intermediate=True)
            x_m, y_m, yaw_rad = actual
            count = max(2, math.ceil(distance_m / 0.025))
            path = RosPath()
            path.poses = [self._pose(0, {
                'x': x_m + distance_m * i / count * math.cos(yaw_rad),
                'y': y_m + distance_m * i / count * math.sin(yaw_rad),
                'yaw': yaw_rad}) for i in range(count + 1)]
            path.header = path.poses[0].header
            # The live Collision Monitor checks the straight exit while driving.
            # A single global-costmap snapshot must not terminate dock departure;
            # retain the static map/keepout check for the whole swept footprint.
            points = [(p.pose.position.x, p.pose.position.y, yaw_rad) for p in path.poses]
            if not static_corridor_clear(
                    Path(self.registry['map']['yaml_path']),
                    Path(self.registry['keepout']['yaml_path']), points,
                    self._reverse_footprint(), allow_rear_keepout_escape=True):
                self.emit(f'{motion}_path_checked', valid=False,
                          reason='static_obstacle_unknown_keepout_or_map_boundary')
                return False
            frozen, _ = self._frozen_in_odom(path)
            self._dock_departure = {'path': frozen, 'start': actual, 'motion_complete': False}
            self.emit(f'{motion}_started', distance_m=distance_m,
                      start_pose_map=list(actual), tracking_frame='odom')
        departure = self._dock_departure
        frozen, start = departure['path'], departure['start']
        saved_config, saved_waypoints = self.config, self.waypoints
        self.waypoints = [saved_waypoints[0]]
        self.config = {**saved_config, 'waypoints': self.waypoints}
        try:
            if not departure['motion_complete']:
                home = self.registry.get('home') or {}
                if (home.get('rear_parking_reference') == 'lidar_box'
                        and not departure.get('rear_release_complete', False)):
                    found = self._scan_in_base()
                    if found is None:
                        # Path preparation can outlast the scan freshness window.
                        # Stay stopped while callbacks refresh the snapshot; never
                        # extend the age accepted by _scan_in_base.
                        self.emit(f'{motion}_release_waiting_scan')
                        deadline = time.monotonic() + ESCAPE_SCAN_MAX_AGE_S
                        task_deadline_s = getattr(self, 'run_deadline_s', None)
                        if task_deadline_s is not None:
                            deadline = min(task_deadline_s, deadline)
                        while (found is None and not self.stop_requested
                               and time.monotonic() < deadline):
                            spin_node(self, timeout_sec=0.02)
                            found = self._scan_in_base()
                    if self.stop_requested:
                        return False
                    if found is None:
                        received = (getattr(self, 'samples', None) or {}).get('scan')
                        self.emit(f'{motion}_release_failed', reason='no fresh scan',
                                  scan_callback_age_s=(time.monotonic() - received
                                                       if received is not None else None),
                                  scan_present=getattr(self, 'last_scan', None) is not None)
                        return False
                    if self._footprint_intrusion(found=found) in ('front', 'both'):
                        self.emit(f'{motion}_release_failed',
                                  reason='front footprint occupied')
                        return False
                    # A close rear face can hold StopZone during the speed ramp.
                    # The existing escape watches fresh scans ahead while paused,
                    # restores the monitor, then follows the original frozen goal.
                    if self._rear_swing_need(found=found) > 0.0:
                        if not self._escape_forward(found, pause_monitor=True):
                            self.emit(f'{motion}_release_failed',
                                      reason='guarded_forward_escape_failed')
                            return False
                    departure['rear_release_complete'] = True
                reached = self._execute_reverse_once(
                    frozen, goal_checker_id='dock_position_checker', final=False,
                    controller_id='Parking', motion=motion)
                if reached:
                    departure['motion_complete'] = True
                    self.emit(f'{motion}_motion_complete', distance_m=distance_m)
        finally:
            self.config, self.waypoints = saved_config, saved_waypoints
        if not departure['motion_complete'] or self.stop_requested:
            return False
        after, _ = self.capture_stationary_pose(intermediate=True)
        if self.stop_requested:
            return False
        departure['end'] = after
        self.emit(f'{motion}_finished', distance_m=distance_m,
                  start_pose_map=list(start), stopped_pose_map=list(after),
                  tracking_frame='odom')
        return True

    def _execute_through_once(self):
        """
        Drive the remaining transit waypoints as one NavigateThroughPoses goal.

        Separate NavigateToPose legs slowed to a stop and turned to every
        waypoint's yaw before the next leg began (2026-10-01). The waypoints
        now only shape one path; the last one keeps its yaw. A retry after an
        input gap resumes from the first waypoint not yet passed.

        On the MPPI trees the last leg (pre-point to observation pose) is a
        goal of its own on RPP (final_leg_behavior_tree): MPPI turns its path
        critics off within 0.5 m straight-line distance of the final goal, so it
        skipped the pre-point 0.42 m from the observation pose and reversed in
        (2026-10-06). MPPI now ends at the pre-point, facing along the leg.
        """
        first, last = self._resume_waypoint_index, len(self.waypoints) - 1
        final_leg = getattr(self, 'final_leg_behavior_tree', None)
        if final_leg is not None and last > first:
            if not self._send_through(first, last - 1, self.through_behavior_tree):
                return False
            return self._send_through(last, last, final_leg)
        return self._send_through(first, last, self.through_behavior_tree)

    def _send_through(self, first, last, behavior_tree):
        """Send waypoints first..last as one NavigateThroughPoses goal on behavior_tree."""
        if not self.navigate_through.wait_for_server(timeout_sec=2.0):
            return False
        if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
            reason = self._guard_failure(False)
            if not self.stop_requested and self._input_gap_recoverable(reason):
                self._retry_guard_reason = reason
            return False
        if not self._departure_ready():
            return False
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose(index, self.waypoints[index])
                      for index in range(first, last + 1)]
        goal.behavior_tree = behavior_tree

        def passed(message):
            left = int(message.feedback.number_of_poses_remaining)
            self._resume_waypoint_index = max(
                self._resume_waypoint_index, min(last, last + 1 - left))

        self.active_action_type = NavigateThroughPoses
        self.navigation_uuid = NavigateThroughPoses.Impl.SendGoalService.Request().goal_id
        self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
        self.pending_goal = self.navigate_through.send_goal_async(
            goal, feedback_callback=passed, goal_uuid=self.navigation_uuid)
        try:
            handle = self._wait(self.pending_goal, 5.0)
            self.pending_goal = None
            if not handle.accepted:
                self.emit('failed', reason='navigation_rejected')
                return False
            self.active_handle = handle
            self._route_event('accepted', last, handle, through=[
                waypoint['id'] for waypoint in self.waypoints[first:last + 1]])
            self.navigation_result = handle.get_result_async()
            while not self.navigation_result.done():
                spin_node(self, timeout_sec=0.05)
                if (self.stop_requested
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    reason = self._guard_failure(False) or 'operator_or_timeout'
                    self.emit('interrupted', reason=reason)
                    if not self.stop_requested and self._input_gap_recoverable(reason):
                        self._retry_guard_reason = reason
                    return False
            wrapped = self.navigation_result.result()
            self._route_event('result', last, handle,
                              terminal_status_code=int(wrapped.status),
                              nav2_error_code=int(wrapped.result.error_code))
            self.navigation_result = None
            if (self.stop_requested or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code):
                self._tag_blocked(int(wrapped.result.error_code), False)
                return False
            self._resume_waypoint_index = last
            return True
        finally:
            if not self.finish_navigation():
                raise RuntimeError('navigation cancellation unconfirmed')
            self.active_action_type = NavigateToPose

    def _search_parameters_ready(self):
        """Reject absent/unbounded spin profiles before sending a search action."""
        names = ['behavior_plugins', 'max_rotational_vel', 'min_rotational_vel',
                 'local_frame', 'robot_base_frame', 'enable_stamped_cmd_vel']
        response = self._read_parameters('behavior_server', names)
        if response is None:
            return False
        values = [parameter_value_to_python(v) for v in response.values]
        if len(values) != len(names):
            return False
        plugins, maximum, minimum, frame, base, stamped = values
        return (isinstance(plugins, list) and 'spin' in plugins
                and all(not isinstance(v, bool) and isinstance(v, (int, float))
                        and math.isfinite(v) for v in (minimum, maximum))
                and 0.0 < minimum <= maximum <= self.parking_contract['rotate_angular_radps']
                and frame == 'odom' and base == 'base_footprint' and stamped is False)

    def search_rotation(self, delta_yaw_rad):
        """Resume only the remaining angle after a terminal input-gap stop."""
        self._search_target_yaw = None
        self.last_search_failure_reason = None

        def attempt():
            remaining_rad = delta_yaw_rad
            if self._search_target_yaw is not None:
                self.capture_stationary_pose()
                actual = self._odom_pose()
                remaining_rad = math.atan2(
                    math.sin(self._search_target_yaw - actual[2]),
                    math.cos(self._search_target_yaw - actual[2]))
                if abs(remaining_rad) <= math.radians(5):
                    self.last_search_failure_reason = None
                    return True
                if abs(remaining_rad) > math.pi / 6 + 1e-9:
                    self.last_search_failure_reason = 'rotation_resume_target_invalid'
                    return False
            return self._search_rotation_once(remaining_rad, measure_in_odom=True)

        return self._run_with_input_recovery(attempt)

    def _search_rotation_once(self, delta_yaw_rad, *, limit_rad=math.pi / 6,
                              event='search_rotation', measure_in_odom=False):
        """Run one bounded Spin through the existing smoother/monitor chain."""
        self.last_search_error_code = None
        self.last_search_failure_reason = None
        if (isinstance(delta_yaw_rad, bool)
                or not isinstance(delta_yaw_rad, (int, float))
                or not math.isfinite(delta_yaw_rad)
                or not 0.0 < abs(delta_yaw_rad) <= limit_rad + 1e-9):
            raise ValueError(f'{event} must be finite and within its angle bound')
        if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
            reason = ('operator_stop' if self.stop_requested
                      else self._guard_failure(False))
            if not self.stop_requested and self._input_gap_recoverable(reason):
                self._retry_guard_reason = reason
            self.last_search_failure_reason = reason or 'operator_stop'
            self.emit(f'{event}_failed', reason=self.last_search_failure_reason)
            return False
        if (not self._departure_ready()
                or not self.spin_search.wait_for_server(timeout_sec=2.0)
                or not self._search_parameters_ready()):
            self.last_search_failure_reason = 'spin_profile_or_navigation_unavailable'
            self.emit(f'{event}_failed', reason=self.last_search_failure_reason)
            return False
        before, _ = self.capture_stationary_pose()
        if measure_in_odom:
            before = self._odom_pose()
        if event == 'search_rotation' and getattr(self, '_search_target_yaw', None) is None:
            self._search_target_yaw = before[2] + delta_yaw_rad
        self.active_action_type = Spin
        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw_rad)
        # A search is a bounded observation maneuver, not final parking; a
        # staging turn of up to 180 deg gets time for its angle.
        extra_rad = max(0.0, abs(delta_yaw_rad) - math.pi / 6)
        goal.time_allowance.sec = 20 + math.ceil(extra_rad / math.radians(30.0)) * 8
        deadline_s = time.monotonic() + goal.time_allowance.sec
        self.navigation_uuid = Spin.Impl.SendGoalService.Request().goal_id
        self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
        self.pending_goal = self.spin_search.send_goal_async(
            goal, goal_uuid=self.navigation_uuid)
        try:
            handle = self._wait(self.pending_goal, 5.0)
            self.pending_goal = None
            if not handle.accepted:
                self.last_search_failure_reason = 'nav2_spin_rejected'
                self.emit('search_rotation_failed', reason=self.last_search_failure_reason)
                return False
            self.active_handle = handle
            self.navigation_result = handle.get_result_async()
            self._mission_started = True
            self.emit(f'{event}_accepted', requested_yaw_rad=delta_yaw_rad,
                      goal_uuid=bytes(self.navigation_uuid.uuid).hex())
            while not self.navigation_result.done():
                spin_node(self, timeout_sec=0.05)
                if (self.stop_requested or time.monotonic() >= deadline_s
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    guard_failure = self._guard_failure(False)
                    reason = ('operator_stop' if self.stop_requested
                              else guard_failure or 'operator_or_spin_timeout')
                    if (not self.stop_requested and not guard_failure
                            and time.monotonic() >= deadline_s):
                        self.last_search_error_code = Spin.Result.TIMEOUT
                    self.last_search_failure_reason = reason
                    self.emit(f'{event}_failed', reason=reason)
                    if (not self.stop_requested and time.monotonic() < deadline_s
                            and self._input_gap_recoverable(reason)):
                        self._retry_guard_reason = reason
                    return False
            wrapped = self.navigation_result.result()
            self.last_search_error_code = int(wrapped.result.error_code)
            self.emit(f'{event}_result', terminal_status_code=int(wrapped.status),
                      nav2_error_code=int(wrapped.result.error_code))
            if self.stop_requested:
                self.last_search_failure_reason = 'operator_stop'
                return False
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code):
                self.last_search_failure_reason = 'nav2_spin_failed'
                return False
            after, _ = self.capture_stationary_pose()
            if measure_in_odom:
                # The Spin ran in odom; AMCL near the dock was off by ~15 deg.
                after = self._odom_pose()
            turned_rad = math.atan2(math.sin(after[2] - before[2]),
                                    math.cos(after[2] - before[2]))
            displacement_m = math.dist(before[:2], after[:2])
            # A search step only has to look around: Spin at 0.7 rad/s overshot
            # 30 deg by 8 deg once the precision SlowdownZone was off (2026-09-30).
            # Half a step still rejects a stalled or runaway turn.
            error_rad = math.atan2(math.sin(turned_rad - delta_yaw_rad),
                                   math.cos(turned_rad - delta_yaw_rad))
            confirmed = (abs(error_rad) <= math.radians(15)
                         and displacement_m <= 0.05)
            self.emit(f'{event}_observed', confirmed=confirmed,
                      requested_yaw_rad=delta_yaw_rad, observed_yaw_rad=turned_rad,
                      displacement_m=displacement_m,
                      observation_frame=('odom' if measure_in_odom else 'map'))
            if not confirmed:
                self.last_search_failure_reason = 'rotation_observation_not_confirmed'
            return confirmed
        finally:
            if not self.finish_navigation():
                raise RuntimeError('search rotation cancellation unconfirmed')

    def _refresh_amcl_at_rest(self):
        """Request a fresh scan update at rest without replacing the robot pose."""
        motion = getattr(self, 'latest_motion', None)
        if (self.stop_requested or motion is None
                or time.monotonic() - motion[0] > 0.5
                or abs(motion[1]) >= 0.01 or abs(motion[2]) >= 0.01
                or self._guard_failure(False) or not self._operator_link_ready()):
            return False
        client = None
        try:
            requested_s = time.monotonic()
            client = self.create_client(Empty, '/request_nomotion_update')
            if not client.wait_for_service(timeout_sec=0.2):
                return False
            if self._wait(client.call_async(Empty.Request()), 1.0) is None:
                return False
            while time.monotonic() - requested_s < 1.5 and not self.stop_requested:
                if (getattr(self, 'amcl_seen', None) or 0.0) > requested_s:
                    self.emit('stationary_localization_refreshed', pose_reset=False)
                    return True
                spin_node(self, timeout_sec=0.05)
        except (AttributeError, RuntimeError):
            return False
        finally:
            if client is not None:
                self.destroy_client(client)
        return False

    def capture_stationary_pose(self, timeout_s=10.0, *, intermediate=False):
        """Capture fresh TF while odometry and consecutive poses remain still."""
        contract = ({**self.parking_contract, 'hold_s': DIRECT_STOP_CONFIRM_S}
                    if intermediate else self.parking_contract)
        revision = self.parking_motion_revision
        window = getattr(self, '_stationary_capture_window', None)
        self._stationary_capture_window = None
        if (window is not None and window['contract'] == contract
                and window['revision'] == revision):
            # Continue the same measured hold, rather than adding another hold
            # after an observation. ParkingHold still rejects gaps and motion;
            # the last stamp forces a new odometry sample before returning.
            gate, target, last_stamp = (
                window['gate'], window['target'], window['last_stamp'])
        else:
            gate, target, last_stamp = ParkingHold(contract), None, None
        deadline_s = time.monotonic() + timeout_s
        last_guard_failure = None
        refresh_attempted = False
        while time.monotonic() < deadline_s and not self.stop_requested:
            spin_node(self, timeout_sec=0.05)
            if self.parking_odom is None or self.amcl_covariance is None:
                continue
            guard_failure = self._guard_failure(require_fresh_amcl=True)
            if guard_failure:
                last_guard_failure = guard_failure
                target = None
                if (not refresh_attempted and 'AMCL pose stale' in guard_failure):
                    refresh_attempted = True
                    self._refresh_amcl_at_rest()
                continue
            received_s, stamp, linear_mps, angular_radps = self.parking_odom
            stamp_key = (stamp.sec, stamp.nanosec)
            if stamp_key == last_stamp:
                continue
            if revision != self.parking_motion_revision or (last_stamp and stamp_key < last_stamp):
                target = None
            last_stamp = stamp_key
            revision = self.parking_motion_revision
            try:
                transform = self.parking_tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                position = transform.transform.translation
                actual = (position.x, position.y, _quaternion_yaw(transform.transform.rotation))
                ros_now_s = self.get_clock().now().nanoseconds * 1e-9
                ages_s = [time.monotonic() - received_s]
                for source_stamp in (stamp, transform.header.stamp):
                    ages_s.append(ros_now_s - source_stamp.sec - source_stamp.nanosec * 1e-9)
                if min(ages_s) < 0.0:
                    raise ValueError('observation ahead of ROS time')
                if target is None:
                    target = actual
                    gate = ParkingHold(contract)
                command = self.parking_command
                # Teaching measures a stationary robot; a silent command topic
                # is recorded as unobserved rather than as measured zero.
                result = gate.observe(
                    time.monotonic(), target, actual, linear_mps, angular_radps,
                    command[1] if command else 0.0,
                    command[2] if command else 0.0, max(ages_s))
                if result['reason'] in ('position_out_of_tolerance', 'yaw_out_of_tolerance'):
                    target = None
                if result['confirmed']:
                    self._stationary_capture_window = {
                        'contract': dict(contract), 'revision': revision,
                        'gate': gate, 'target': target, 'last_stamp': last_stamp,
                    }
                    return actual, {
                        **result, 'source_ages_s': ages_s,
                        'amcl_xy_covariance_m2': list(self.amcl_covariance),
                        'amcl_yaw_covariance_rad2': self.amcl_yaw_covariance_rad2,
                        'command_observed': command is not None,
                    }
            except (TransformException, ValueError):
                target = None
        detail = f'; last guard: {last_guard_failure}' if last_guard_failure else ''
        raise RuntimeError(
            'stationary teaching unavailable: need fresh TF, odometry and localization'
            + detail)

    def visit(self, table_id, execute=False, timeout_s=180.0):
        """Resolve, plan, navigate, park and confirm one selected table pose."""
        self.table_id = table_id
        self.selected_pose = None
        self.confirmation = None
        poses = candidates(self.registry, table_id)
        # A fresh process may wait up to 30 s for the maps; that discovery must not
        # consume this leg's budget, and an earlier leg's deadline must not end it.
        self.run_deadline_s = None
        self.verify_live_maps()
        self.run_deadline_s = time.monotonic() + timeout_s
        if not self.wait_until_ready(timeout=10.0):
            raise RuntimeError(self.readiness_failure_reason())
        if not self._parking_parameters_ready():
            raise RuntimeError('load the generated Parking controller parameters into Nav2')
        pose, attempts = select_destination(poses, self.plan_pose)
        self.emit('planning', attempts=attempts)
        if pose is None:
            self.emit('failed', reason='no_service_pose_planned')
            return False
        self.selected_pose = pose
        self.emit('selected', target_pose=[pose[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  waypoints=route_config(self.registry, pose)['waypoints'],
                  xy_tolerance_m=self.parking_contract['xy_tolerance_m'],
                  yaw_tolerance_rad=self.parking_contract['yaw_tolerance_rad'])
        if not execute:
            self.emit('planned_only')
            return True
        # A second asset check catches changes made while planning.
        self.verify_live_maps()
        success = self.execute()
        self.emit('arrived' if success else 'failed', confirmation=self.confirmation)
        return success

    def wait_for_box(self, dwell_s, timeout_s):
        """Hold at the destination while the front box remains stably observed."""
        gate = BoxDwell(dwell_s)
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= dwell_s):
            raise ValueError('box timeout must be finite and exceed dwell')
        self.emit('box_wait_started', dwell_s=float(dwell_s), timeout_s=float(timeout_s))
        deadline_s = time.monotonic() + float(timeout_s)
        while not self.stop_requested and time.monotonic() < deadline_s:
            spin_node(self, timeout_sec=0.05)
            result = gate.observe(
                time.monotonic(), self.box_status_received_s, self.box_status)
            if result['confirmed']:
                self.emit('box_wait_complete', **result, observation=self.box_status)
                return True
        self.emit('box_wait_failed', reason=(
            self._guard_failure(False) or 'box_detection_timeout'))
        return False

    def go_home(self, execute=False, timeout_s=180.0):
        """Plan and optionally execute the taught home pose with yaw verification."""
        pose = home_pose(self.registry)
        self.table_id = 'home_dock'
        self.pose_id = pose['id']
        self.selected_pose = pose
        self.confirmation = None
        # Map discovery before the budget, as in visit().
        self.run_deadline_s = None
        self.verify_live_maps()
        self.run_deadline_s = time.monotonic() + timeout_s
        if not self.wait_until_ready(timeout=10.0):
            raise RuntimeError(self.readiness_failure_reason())
        reverse = pose.get('parking_direction', 'forward') == 'reverse'
        if not (self._parking_parameters_ready(reverse=True) if reverse
                else self._parking_parameters_ready()):
            raise RuntimeError('load the generated Parking controller parameters into Nav2')
        if reverse and not self._reverse_smoother_ready():
            raise RuntimeError('reverse velocity is blocked or unbounded in smoother')
        if reverse:
            # A mismatched live dock goal checker fails before any motion,
            # including the parked-pose exit below.
            self._home_goal_checker()
        # Leave the current parked pose before the first home motion of either branch.
        if execute and not self._leave_parked_pose():
            self.emit('failed', reason='parked_pose_exit_failed')
            return False
        if reverse:
            return self._go_home_reverse(pose, execute)
        planned = self._plan_with_input_recovery(pose)
        self.emit('home_planning', attempt=planned)
        if not planned['ok']:
            self.emit('failed', reason='home_not_planned')
            return False
        self.emit('home_selected',
                  target_pose=[pose[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  waypoints=route_config(self.registry, pose)['waypoints'],
                  xy_tolerance_m=self.parking_contract['xy_tolerance_m'],
                  yaw_tolerance_rad=self.parking_contract['yaw_tolerance_rad'])
        if not execute:
            self.emit('home_planned_only')
            return True
        self.verify_live_maps()
        success = self.execute()
        self.emit('home_arrived' if success else 'failed', confirmation=self.confirmation)
        return success

    def _leave_parked_pose(self):
        """Leave the parked pose before a home motion; a table pose needs no escape."""
        return True

    def _make_reverse_path(self, start, target, path_contract=None, curve=False):
        contract = path_contract or self.parking_contract
        if curve:
            points = reverse_curve_waypoints(start, target)
        else:
            points = reverse_waypoints(
                start, target, xy_tolerance_m=contract['xy_tolerance_m'],
                yaw_tolerance_rad=contract['yaw_tolerance_rad'])
        path = RosPath()
        path.poses = [self._pose(i, {'x': x, 'y': y, 'yaw': yaw})
                      for i, (x, y, yaw) in enumerate(points)]
        path.header = path.poses[0].header
        return path

    def _frozen_in_odom(self, path):
        """
        Express a validated map path in odom as map and odom relate right now.

        AMCL heading near the dock and the boxes jumped by up to about 15 deg on
        2026-09-30 while short reverses swung left and right; a path held in
        odom stays straight whatever AMCL does meanwhile. Returns the odom path
        and the map-to-odom pose function used for its confirmation target.
        """
        try:
            transform = self.parking_tf.lookup_transform('odom', 'map', rclpy.time.Time())
        except TransformException as error:
            raise RuntimeError(f'map/odom transform unavailable: {error}') from error
        # Freeze the frame relation itself. Two separate base-chain lookups
        # need overlapping startup histories and can sample different instants.
        odom_x = transform.transform.translation.x
        odom_y = transform.transform.translation.y
        turn = _quaternion_yaw(transform.transform.rotation)

        def to_odom(x, y, yaw):
            return (odom_x + math.cos(turn) * x - math.sin(turn) * y,
                    odom_y + math.sin(turn) * x + math.cos(turn) * y,
                    math.atan2(math.sin(yaw + turn), math.cos(yaw + turn)))

        frozen = RosPath()
        frozen.header.frame_id = 'odom'
        frozen.header.stamp = path.header.stamp
        for item in path.poses:
            x, y, yaw = to_odom(item.pose.position.x, item.pose.position.y,
                                _quaternion_yaw(item.pose.orientation))
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = item.header.stamp
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            frozen.poses.append(pose)
        return frozen, to_odom

    def _odom_pose(self):
        """Return the current base pose in odom (the frame a frozen reverse ran in)."""
        base = self.parking_contract['robot_base_frame']
        try:
            transform = self.parking_tf.lookup_transform('odom', base, rclpy.time.Time())
        except TransformException as error:
            raise RuntimeError(f'odom transform unavailable: {error}') from error
        return (transform.transform.translation.x, transform.transform.translation.y,
                _quaternion_yaw(transform.transform.rotation))

    def _reverse_smoother_ready(self):
        response = self._read_parameters(
            'velocity_smoother', ['min_velocity', 'max_velocity'])
        if response is None:
            return False
        values = [parameter_value_to_python(v) for v in response.values]
        if (len(values) != 2 or any(not isinstance(v, list) or len(v) != 3 for v in values)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for v in values for x in v)):
            return False
        limit = self.parking_contract['desired_linear_mps']
        # Reverse must match the contract; forward transit is no longer capped
        # to the parking speed (the parking controllers bound themselves).
        return (math.isclose(values[0][0], -limit, abs_tol=1e-9)
                and 0.0 < values[1][0] <= max(limit, SERVICE_TRANSIT_MAX_MPS))

    def _reverse_path_valid(self, path, validate_from_m=0.0, phase=None):
        """
        Validate the whole path, or its tail after validate_from_m of travel.

        A straight reverse never re-enters the band inside the current chassis
        front, and the first validated pose covers the space it newly uses.
        """
        points = [(item.pose.position.x, item.pose.position.y,
                   _quaternion_yaw(item.pose.orientation)) for item in path.poses]
        offset = 0
        excluded = {}
        checked = path
        if validate_from_m > 0.0:
            cumulative = [0.0]
            for previous, current in zip(points, points[1:]):
                cumulative.append(cumulative[-1] + math.dist(previous[:2], current[:2]))
            # Generated spacing accumulates rounding; a short path keeps its last pose.
            offset = next((index for index, travel in enumerate(cumulative)
                           if travel >= validate_from_m - 1e-9), len(points))
            offset = min(offset, len(points) - 1)
            points = points[offset:]
            checked = RosPath()
            checked.header = path.header
            checked.poses = path.poses[offset:]
            excluded = {'excluded_indices': [0, offset - 1],
                        'excluded_front_band_m': cumulative[offset]}
        phase_fields = {} if phase is None else {'phase': phase}
        if not static_corridor_clear(
                Path(self.registry['map']['yaml_path']),
                Path(self.registry['keepout']['yaml_path']), points,
                self._footprint_outline()):
            self.emit('reverse_path_checked', valid=False,
                      reason='static_obstacle_unknown_keepout_or_map_boundary',
                      **phase_fields, **excluded)
            return False
        if not self.validate_reverse_path.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('reverse path collision validation unavailable')
        request = IsPathValid.Request()
        request.path = checked

        def wait_abort_reason():
            if self.stop_requested:
                return 'operator_stop'
            if not self._operator_link_ready():
                return 'operator_link_lost'
            return self._guard_failure(False)

        for attempt in range(REVERSE_PATH_CHECK_ATTEMPTS):
            try:
                result = self._wait(self.validate_reverse_path.call_async(request), 3.0)
            except RuntimeError:
                reason = wait_abort_reason()
                if reason:
                    self.emit('path_blocked_wait_aborted', reason=reason,
                              attempt=attempt + 1, **phase_fields, **excluded)
                    return False
                raise
            # Report indices of the original path, not of the validated tail.
            invalid_indices = [index + offset for index in result.invalid_pose_indices]
            rejection = {} if result.is_valid and not invalid_indices else {
                'reason': 'nav2_live_costmap_rejected'}
            self.emit('reverse_path_checked', valid=result.is_valid,
                      invalid_pose_indices=invalid_indices,
                      attempt=attempt + 1, **rejection, **phase_fields, **excluded)
            if result.is_valid and not invalid_indices:
                return True
            if attempt + 1 == REVERSE_PATH_CHECK_ATTEMPTS:
                self.emit('path_blocked_give_up',
                          reason='nav2_live_costmap_rejected',
                          invalid_pose_indices=invalid_indices,
                          attempts=REVERSE_PATH_CHECK_ATTEMPTS,
                          **phase_fields, **excluded)
                return False
            self.emit('reverse_path_waiting_for_update', attempt=attempt + 1,
                      **phase_fields, **excluded)
            until_s = time.monotonic() + REVERSE_PATH_UPDATE_S
            while time.monotonic() < until_s:
                reason = wait_abort_reason()
                if reason:
                    self.emit('path_blocked_wait_aborted', reason=reason,
                              attempt=attempt + 1, **phase_fields, **excluded)
                    return False
                spin_node(self, timeout_sec=0.05)
        return False

    def _footprint_outline(self):
        """
        Return the runtime footprint polygon itself when it carries no padding.

        The escape checks follow the stepped body (wheels wider than the frame
        only near the axle) like the Collision Monitor; with padding the
        conservative bounding rectangle is used instead.
        """
        polygon, padding = self._runtime_footprint()
        if padding == 0.0:
            return [(float(x), float(y)) for x, y in polygon]
        return self._reverse_footprint()

    def _runtime_footprint(self):
        """Return the matching global/local footprint polygon and padding."""
        footprints = []
        for name in ('global_costmap/global_costmap', 'local_costmap/local_costmap'):
            response = self._read_parameters(
                name, ['footprint', 'footprint_padding', 'robot_base_frame'])
            if response is None:
                raise RuntimeError(f'{name} footprint unavailable')
            values = [parameter_value_to_python(v) for v in response.values]
            if len(values) != 3 or values[2] != 'base_footprint':
                raise RuntimeError('reverse footprint frame mismatch')
            polygon = ast.literal_eval(values[0]) if isinstance(values[0], str) else None
            padding = values[1]
            if (not isinstance(polygon, (tuple, list)) or len(polygon) < 3
                    or isinstance(padding, bool) or not isinstance(padding, (int, float))
                    or not math.isfinite(padding) or padding < 0.0):
                raise RuntimeError('reverse footprint geometry unavailable')
            if any(not isinstance(point, (tuple, list)) or len(point) != 2
                   or any(isinstance(v, bool) or not isinstance(v, (int, float))
                          or not math.isfinite(v) for v in point) for point in polygon):
                raise RuntimeError('reverse footprint geometry invalid')
            footprints.append((polygon, padding))
        if footprints[0] != footprints[1]:
            raise RuntimeError('global and local footprints disagree')
        return footprints[0]

    def _reverse_footprint(self):
        """Use matching runtime footprints, including their configured padding."""
        polygon, padding = self._runtime_footprint()
        # A bounding rectangle is conservative for the configured base polygon.
        xmin = min(p[0] for p in polygon) - padding
        xmax = max(p[0] for p in polygon) + padding
        ymin = min(p[1] for p in polygon) - padding
        ymax = max(p[1] for p in polygon) + padding
        return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]

    def _home_goal_checker(self):
        """Name the loaded goal checker whose tolerance is the dock contract."""
        home = getattr(self, 'home_contract', None) or self.parking_contract
        tolerance = (home['xy_tolerance_m'], home['yaw_tolerance_rad'])
        if tolerance == (self.parking_contract['xy_tolerance_m'],
                         self.parking_contract['yaw_tolerance_rad']):
            return 'parking_goal_checker'
        alignment = (MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'],
                     math.radians(MAXIMUM_CONTRACT_VALUES['yaw_tolerance_deg']))
        if not all(math.isclose(value, wanted, rel_tol=0.0, abs_tol=1e-9)
                   for value, wanted in zip(tolerance, alignment)):
            raise RuntimeError('home contract matches no loaded goal checker')
        # Read the live checkers instead of trusting the generated parameters; the
        # dock entry turns with entry_heading_checker (_dock_straight).
        response = self._read_parameters('controller_server', [
            'alignment_goal_checker.xy_goal_tolerance',
            'alignment_goal_checker.yaw_goal_tolerance',
            f'{ENTRY_HEADING_CHECKER}.yaw_goal_tolerance'])
        values = ([parameter_value_to_python(value) for value in response.values]
                  if response is not None else [])
        if (len(values) != 3 or any(
                isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isclose(value, wanted, rel_tol=0.0, abs_tol=1e-9)
                for value, wanted in zip(values, (*alignment, ENTRY_HEADING_TOLERANCE_RAD)))):
            raise RuntimeError('live alignment or entry heading goal checker does not match '
                               'the home contract')
        return 'alignment_goal_checker'

    def _reach_staging(self, stage, *, pause_on_rotation_failure=None):
        """
        Reach the staging position, then turn in place toward the dock heading.

        On 2026-09-30 the staging leg replanned every second (113 times in
        127 s) and the final heading turn followed AMCL, which near the dock was
        off by about 15 deg, so the robot swung the long way and back. The
        position leg now replans only on an invalid, expired or new path; the
        heading uses damped odom turns whose direction is fixed at rest. A position
        offset is left to the curved pre-dock reverse (_dock_straight). Returns the
        last stationary pose, or None on failure.
        """
        if not self.execute(final_parking=False, staging=True):
            return None
        for turn in range(2):   # the large turn, then a correction only if far off
            actual, _ = self.capture_stationary_pose(intermediate=True)
            delta_rad = math.atan2(math.sin(stage['yaw'] - actual[2]),
                                   math.cos(stage['yaw'] - actual[2]))
            self.emit('staging_heading_measured', delta_yaw_rad=delta_rad)
            # Spin overshot by about 3 deg each time, so a 3.9 deg correction
            # swung back 7.1 deg (2026-10-01); the curved reverse absorbs this.
            limit_rad = (self.home_contract['yaw_tolerance_rad'] if turn == 0
                         else STAGING_CORRECTION_MIN_RAD)
            if abs(delta_rad) <= limit_rad:
                return actual
            turned = self._rotate_in_place(
                delta_rad, pause_on_failure=pause_on_rotation_failure)
            self.emit('staging_turn_observed', requested_yaw_rad=delta_rad,
                      observed_yaw_rad=turned, observation_frame='odom', controller='damped')
            if turned is None:
                return None
        actual, _ = self.capture_stationary_pose(intermediate=True)
        return actual

    def _go_home_reverse(self, pose, execute, *, _map_reapproach=False):
        """
        Align outside the dock, then follow a bounded reverse path via Nav2.

        A precision session stages with the alignment checker and confirms the
        staging and the dock with the dock's own contract. A standard session
        uses its single contract for every step, exactly as before.
        """
        full_config = route_config(self.registry, pose)
        self._dock_map_recovery = None
        self.config, self.waypoints = full_config, full_config['waypoints']
        target = tuple(pose[key] for key in ('x_m', 'y_m', 'yaw_rad'))
        home = getattr(self, 'home_contract', None) or self.parking_contract
        precision_home = (
            (home['xy_tolerance_m'], home['yaw_tolerance_rad'])
            != (self.parking_contract['xy_tolerance_m'],
                self.parking_contract['yaw_tolerance_rad']))
        # A standard session passes no contract keywords at all.
        verify_kwargs = {'contract': home} if precision_home else {}
        path_kwargs = {'path_contract': home} if precision_home else {}
        # go_home() already checked this before the parked-pose exit; a direct
        # call still fails here, before staging.
        if precision_home:
            self._home_goal_checker()
        actual = None
        if (execute and not _map_reapproach
                and getattr(self, 'parking_command', None) is not None
                and getattr(self, 'parking_odom', None) is not None):
            try:
                actual = self._parking_observation()['actual_pose']
            except (TransformException, ValueError) as error:
                self.emit('home_stationary_shortcut_unavailable', reason=str(error))
        if actual is not None:
            distance, angle = pose_errors(target, actual)
            if (distance <= home['xy_tolerance_m']
                    and angle <= home['yaw_tolerance_rad']):
                success = self._verify_parking_stop(1, self.waypoints[-1], None, **verify_kwargs)
                self.emit('home_arrived' if success else 'failed',
                          already_at_home=True, confirmation=self.confirmation)
                return success
        stage = full_config['waypoints'][0]
        stage_pose = {**pose, 'id': stage['id'], 'x_m': stage['x'], 'y_m': stage['y'],
                      'parking_direction': 'forward'}
        nominal_path = self._make_reverse_path(
            (stage['x'], stage['y'], stage['yaw']), target, **path_kwargs)
        if not self._reverse_path_valid(nominal_path, phase='nominal_precheck'):
            return False
        try:
            # Precision staging is an intermediate leg; the docking below is not.
            with self._localization_bound(precision_home):
                if precision_home:
                    # Staging is intermediate: the alignment checker judges its plan.
                    planned = self._plan_with_input_recovery(
                        stage_pose, single=True,
                        end_tolerance_m=MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'])
                else:
                    planned = self._plan_with_input_recovery(stage_pose, single=True)
                self.emit('reverse_staging_planned', attempt=planned)
                if not planned['ok']:
                    return False
                if not execute:
                    self.emit('home_planned_only', parking_direction='reverse',
                              final_path_validation='RECHECK_ACTUAL_PATH_AT_STAGING')
                    return True
                staged = None
                if precision_home:
                    staged = self._reach_staging(stage)
                    if staged is None:
                        return False
                elif not self.execute():
                    return False
            pre_dock = None
            reference = stage
            if precision_home:
                # A curved reverse to the pre-dock pose takes out a staging
                # offset (_dock_straight); the alignment leg (turn, move, turn: a
                # full extra turn for 10 cm on 2026-10-01) runs only when no
                # curve fits.
                reference = {**stage, 'x': staged[0], 'y': staged[1], 'yaw': staged[2]}
                if _lateral_offset(staged, target) > STRAIGHT_DOCK_LATERAL_M:
                    candidate = _ahead(target, PRE_DOCK_DISTANCE_M)
                    try:
                        self._make_reverse_path(staged, candidate, curve=True)
                        pre_dock = candidate
                    except ValueError as error:
                        self.emit('dock_curve_unavailable', reason=str(error),
                                  actual_pose=list(staged))
                        reference = stage
                        with self._localization_bound(True):
                            if not self.execute(final_parking=False, alignment=True):
                                return False
            # Confirmed at rest with the strict localization bound like the dock.
            if precision_home and not self._verify_parking_stop(
                    0, reference, None, contract=home):
                return False
        finally:
            self.config, self.waypoints = full_config, full_config['waypoints']
            self.pose_id = pose['id']
        # The actual stationary staging pose, not an assumed waypoint, seeds
        # the path. Reject misalignment before any reverse action is sent.
        self.verify_live_maps()
        actual, _evidence = self.capture_stationary_pose()
        # The staging confirmation describes the intermediate pose.  Do not
        # attach it to a later dock-path failure as if the dock were confirmed.
        self.confirmation = None
        if precision_home:
            success = self._dock_straight(actual, target, pre_dock, home)
            recovery = self._dock_map_recovery
            if (not success and recovery is not None and not _map_reapproach
                    and not self.stop_requested and self._operator_link_ready()
                    and self._navigation_ready(require_fresh_amcl=False)):
                # Leave along the established odom axis before any new heading.
                # Replan from outside the dock so map drift and lateral error are
                # both included in the next staging/curve/freeze operation.
                self.emit('dock_map_reapproach_requested',
                          reason=self.confirmation.get('reason'),
                          confirmation=self.confirmation)
                if self._pull_out_to(*recovery):
                    return self._go_home_reverse(pose, execute, _map_reapproach=True)
        else:
            try:
                path = self._make_reverse_path(actual, target, **path_kwargs)
            except ValueError as error:
                self.emit('reverse_staging_out_of_tolerance', reason=str(error),
                          actual_pose=list(actual))
                return False
            if not self._reverse_path_valid(path):
                return False
            success = self._execute_reverse_path(path)
        self.emit('home_arrived' if success else 'failed',
                  parking_direction='reverse', confirmation=self.confirmation)
        return success

    def _execute_reverse_path(self, path, path_contract=None, validate_from_m=0.0,
                              goal_checker_id='parking_goal_checker', verify_contract=None,
                              final=True, send_path=None, verify_waypoint=None,
                              controller_id='ParkingReverse', curve=False, retry_goal=None):
        """
        Rebuild the remaining reverse path after one confirmed input-gap stop.

        A retry keeps the first path's contract. Its excluded front band
        shrinks by the distance already reversed. A non-final reverse (a box
        escape) already at its goal is left to the caller's fresh stationary
        capture. retry_goal replaces the route's last waypoint as the rebuilt
        path's end (the pre-dock pose).
        """
        first_attempt = True
        # Forward only non-default keywords so existing call shapes are unchanged.
        make_kwargs = {} if path_contract is None else {'path_contract': path_contract}
        if curve:
            # A curved dock reverse is rebuilt as a curve from where it stopped.
            make_kwargs['curve'] = True
        valid_kwargs = {'validate_from_m': validate_from_m} if validate_from_m else {}
        once_kwargs = {}
        if goal_checker_id != 'parking_goal_checker':
            once_kwargs['goal_checker_id'] = goal_checker_id
        if verify_contract is not None:
            once_kwargs['verify_contract'] = verify_contract
        if not final:
            once_kwargs['final'] = False
        if verify_waypoint is not None:
            once_kwargs['verify_waypoint'] = verify_waypoint
        if controller_id != 'ParkingReverse':
            once_kwargs['controller_id'] = controller_id

        def attempt():
            nonlocal first_attempt
            # The first send may be the odom-frozen copy; a retry after an
            # input gap rebuilds on the map from the stationary pose.
            remaining = path if send_path is None else send_path
            if not first_attempt:
                actual, _ = self.capture_stationary_pose()
                goal = retry_goal or self.waypoints[-1]
                try:
                    remaining = self._make_reverse_path(
                        actual, (goal['x'], goal['y'], goal['yaw']), **make_kwargs)
                except ValueError as error:
                    if not final and str(error) == 'already at goal':
                        return True
                    self.emit('reverse_path_rebuild_failed', reason=str(error),
                              actual_pose=list(actual))
                    return False
                retry_kwargs = valid_kwargs
                if validate_from_m:
                    # The band lay inside the chassis at the original start.
                    # Reversing moved the chassis back, so only the part not
                    # yet reversed is still inside; validate everything else.
                    start = path.poses[0].pose
                    yaw = _quaternion_yaw(start.orientation)
                    traveled_m = max(0.0, -(
                        (actual[0] - start.position.x) * math.cos(yaw)
                        + (actual[1] - start.position.y) * math.sin(yaw)))
                    band_m = max(0.0, validate_from_m - traveled_m)
                    retry_kwargs = {'validate_from_m': band_m} if band_m else {}
                if not self._reverse_path_valid(remaining, **retry_kwargs):
                    return False
            first_attempt = False
            return self._execute_reverse_once(remaining, **once_kwargs)

        return self._run_with_input_recovery(attempt)

    def _dock_straight(self, actual, target, pre_dock, home):
        """
        Reverse into the dock along its axis with the heading set beforehand.

        A curve ending at the dock left its heading lag inside the dock and the
        robot turned in place there (2026-10-01, see PRE_DOCK_DISTANCE_M). A
        staging offset is now taken out by a curved reverse to pre_dock, and the
        heading is set to 1.5 deg where the straight part starts. A dock heading
        up to DOCK_END_TURN_MAX_RAD off is trimmed in place there to
        DOCK_HEADING_TOLERANCE_RAD; a larger one pulls straight out to that
        start, sets the heading and enters once more. Staging was confirmed on the map; every leg
        runs in odom as frozen here, so AMCL jumps cannot bend it. Final arrival
        must also agree with the original registered map pose.
        """
        controller = getattr(self, 'dock_leg_controller', 'ParkingReverse')
        # The straight part starts on the dock axis, heading set first.
        entry = pre_dock or (actual[0], actual[1], target[2])
        try:
            legs = ([self._make_reverse_path(actual, pre_dock, curve=True)]
                    if pre_dock is not None else [])
            legs.append(self._make_reverse_path(entry, target, path_contract=home))
        except ValueError as error:
            self.emit('reverse_staging_out_of_tolerance', reason=str(error),
                      actual_pose=list(actual))
            return False
        if (pre_dock is not None
                and not self._reverse_path_valid(legs[0], phase='dock_curve')):
            return False
        if not self._reverse_path_valid(legs[-1], phase='dock_final_straight'):
            return False
        frozen, to_odom = self._frozen_in_odom(legs[0])
        dock = dict(zip(('x', 'y', 'yaw'), to_odom(*target)))
        self.emit('dock_leg_frozen_in_odom',
                  odom_target_pose=[dock['x'], dock['y'], dock['yaw']],
                  controller_id=controller, pre_dock_pose=pre_dock and list(pre_dock),
                  staging_lateral_offset_m=_lateral_offset(actual, target))
        if pre_dock is not None and not self._execute_reverse_path(
                legs[0], path_contract=home, goal_checker_id='dock_position_checker',
                final=False, send_path=frozen, controller_id=controller, curve=True,
                retry_goal=dict(zip(('x', 'y', 'yaw'), pre_dock))):
            return False
        entry_odom = to_odom(*entry)
        for attempt in range(DOCK_ENTRY_ATTEMPTS):
            if attempt and not self._pull_out_to(entry_odom, dock['yaw']):
                return False
            if not self._turn_to_dock_heading(
                    dock, ENTRY_HEADING_TOLERANCE_RAD, ENTRY_HEADING_CHECKER,
                    event='dock_entry_heading_measured'):
                return False
            start = self._odom_pose()
            try:
                points = reverse_waypoints(
                    start, (dock['x'], dock['y'], dock['yaw']),
                    xy_tolerance_m=home['xy_tolerance_m'],
                    yaw_tolerance_rad=home['yaw_tolerance_rad'])
            except ValueError as error:
                self.emit('reverse_staging_out_of_tolerance', reason=str(error),
                          actual_pose=list(start), frame='odom')
                return False
            if controller == 'ParkingReverse':
                reached = self._drive_dock_straight(dock, home)
            else:
                reached = self._execute_reverse_path(
                    legs[-1], path_contract=home, goal_checker_id='dock_position_checker',
                    final=False, send_path=self._odom_path(points),
                    controller_id=controller)
            if not reached:
                return False
            _x, _y, yaw = self._odom_pose()
            error = math.atan2(math.sin(dock['yaw'] - yaw), math.cos(dock['yaw'] - yaw))
            self.emit('dock_heading_measured', delta_yaw_rad=error, attempt=attempt + 1)
            if abs(error) <= DOCK_END_TURN_MAX_RAD:
                if not self._trim_dock_heading(dock):
                    return False
                if not self._verify_parking_stop(
                        len(self.waypoints) - 1, dock, None,
                        contract={**home, 'reference_frame': 'odom'}):
                    return False
                odom_confirmation = dict(self.confirmation or {})
                map_dock = dict(zip(('x', 'y', 'yaw'), target))
                confirmed = self._verify_parking_stop(
                    len(self.waypoints) - 1, map_dock, None, contract=home)
                if (not confirmed and (self.confirmation or {}).get('reason') in {
                        'position_out_of_tolerance', 'yaw_out_of_tolerance'}
                        and self._refresh_amcl_at_rest()):
                    # Reconsider a quiet filter with a new scan before leaving a
                    # stopped dock; the registered goal and tolerances stay fixed.
                    confirmed = self._verify_parking_stop(
                        len(self.waypoints) - 1, map_dock, None, contract=home)
                self.emit('dock_pose_frame_confirmation',
                          odom_confirmation=odom_confirmation,
                          map_confirmation=self.confirmation, confirmed=confirmed)
                if (not confirmed and (self.confirmation or {}).get('reason') in {
                        'position_out_of_tolerance', 'yaw_out_of_tolerance'}):
                    self._dock_map_recovery = (entry_odom, dock['yaw'])
                return confirmed
        self.emit('dock_heading_out_of_tolerance', delta_yaw_rad=error,
                  attempts=DOCK_ENTRY_ATTEMPTS)
        return False

    def _trim_dock_heading(self, dock):
        """
        Trim the heading in the dock to DOCK_HEADING_TOLERANCE_RAD by slow direct turns.

        Each turn is measured again in odom, and one that stopped short or went
        past is trimmed again, at most DOCK_TRIM_ATTEMPTS times; the dock contract
        still judges the stop. False only without odom.
        """
        for trims in range(DOCK_TRIM_ATTEMPTS + 1):
            if (self.stop_requested or not self._departure_ready()
                    or not self._navigation_ready(require_fresh_amcl=False)):
                return False
            _x, _y, yaw = self._odom_pose()
            error = math.atan2(math.sin(dock['yaw'] - yaw), math.cos(dock['yaw'] - yaw))
            if (abs(error) <= DOCK_HEADING_TOLERANCE_RAD or trims == DOCK_TRIM_ATTEMPTS
                    or self.stop_requested):
                self.emit('dock_heading_trimmed', delta_yaw_rad=error, trims=trims)
                return True
            if self._rotate_in_place(error) is None:
                return False

    def _odom_path(self, points):
        """Return (x, y, yaw) points as a path in odom."""
        path = RosPath()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _pull_out_to(self, entry, heading):
        """Drive forward along the dock axis back to the straight entry pose."""
        x, y, _yaw = self._odom_pose()
        count = max(2, math.ceil(math.dist((x, y), entry[:2]) / 0.025))
        points = [(x + (entry[0] - x) * i / count, y + (entry[1] - y) * i / count, heading)
                  for i in range(count + 1)]
        return self._execute_reverse_once(
            self._odom_path(points), goal_checker_id='dock_position_checker', final=False,
            controller_id='Parking', motion='pull_out')

    def _turn_to_dock_heading(self, dock, limit_rad, goal_checker_id,
                              event='dock_heading_measured'):
        """Set the entry heading with damped trims and remeasure after stopping."""
        for attempt in range(DOCK_TRIM_ATTEMPTS + 1):
            _x, _y, yaw = self._odom_pose()
            error = math.atan2(math.sin(dock['yaw'] - yaw), math.cos(dock['yaw'] - yaw))
            self.emit(event, delta_yaw_rad=error, attempt=attempt,
                      goal_checker_id=goal_checker_id)
            if (self.stop_requested or not self._departure_ready()
                    or not self._navigation_ready(require_fresh_amcl=False)):
                return False
            if abs(error) <= limit_rad:
                return True
            if attempt == DOCK_TRIM_ATTEMPTS or self._rotate_in_place(error) is None:
                return False
        return False

    def _drive_dock_straight(self, dock, home):
        """Close the frozen odom target with heading hold, without RPP steering."""
        tolerance_m = min(home['xy_tolerance_m'], DOCK_POSITION_TOLERANCE_M)

        def attempt():
            if (self.stop_requested or not self._departure_ready()
                    or not self._navigation_ready(require_fresh_amcl=False)):
                reason = self._guard_failure(False)
                if not self.stop_requested and self._input_gap_recoverable(reason):
                    self._retry_guard_reason = reason
                return False
            x_m, y_m, yaw_rad = self._odom_pose()
            dx_m, dy_m = dock['x'] - x_m, dock['y'] - y_m
            if math.hypot(dx_m, dy_m) <= tolerance_m:
                return True
            distance_m = dx_m * math.cos(yaw_rad) + dy_m * math.sin(yaw_rad)
            lateral_m = -dx_m * math.sin(yaw_rad) + dy_m * math.cos(yaw_rad)
            if distance_m >= 0.0 or abs(lateral_m) > tolerance_m:
                self.emit('reverse_staging_out_of_tolerance', frame='odom',
                          remaining_forward_m=distance_m, lateral_error_m=lateral_m)
                return False
            travelled_m = self._drive_zero_turn(
                distance_m, precision=True, hold_heading=True,
                watch=lambda: None if self._operator_link_ready() else 'operator_link_lost')
            reason = (getattr(self, '_last_direct_motion_failure', None)
                      or self._guard_failure(False))
            if reason:
                if self._input_gap_recoverable(reason):
                    self._retry_guard_reason = reason
                return False
            if travelled_m is None or self.stop_requested or not self._departure_ready():
                return False
            end_x, end_y, _yaw = self._odom_pose()
            return math.hypot(dock['x'] - end_x, dock['y'] - end_y) <= tolerance_m

        return self._run_with_input_recovery(attempt)

    def _execute_reverse_once(self, path, goal_checker_id='parking_goal_checker',
                              verify_contract=None, final=True, verify_waypoint=None,
                              controller_id='ParkingReverse', motion='reverse',
                              position_handoff_m=None):
        """Keep reverse motion in controller → smoother → monitor → base."""
        if not self.waypoints:
            raise ValueError('reverse execution requires a target waypoint')
        handoff_target = None
        if position_handoff_m is not None:
            if (final or isinstance(position_handoff_m, bool)
                    or not math.isfinite(position_handoff_m)
                    or not 0.0 < position_handoff_m <= .05
                    or path.header.frame_id != 'odom' or not path.poses):
                raise ValueError('invalid intermediate reverse position handoff')
            endpoint = path.poses[-1].pose.position
            handoff_target = (endpoint.x, endpoint.y)
            if not all(math.isfinite(value) for value in handoff_target):
                raise ValueError('nonfinite intermediate reverse handoff target')
        target_index = len(self.waypoints) - 1
        if self.stop_requested or not self._navigation_ready(require_fresh_amcl=False):
            reason = self._guard_failure(False)
            if not self.stop_requested and self._input_gap_recoverable(reason):
                self._retry_guard_reason = reason
            return False
        if (not self._departure_ready()
                or not self.follow_reverse.wait_for_server(timeout_sec=2.0)):
            return False
        self.active_action_type = FollowPath
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id
        goal.goal_checker_id = goal_checker_id
        goal.progress_checker_id = 'progress_checker'
        self.navigation_uuid = FollowPath.Impl.SendGoalService.Request().goal_id
        self.navigation_uuid.uuid = list(uuid.uuid4().bytes)
        self.pending_goal = self.follow_reverse.send_goal_async(
            goal, goal_uuid=self.navigation_uuid)
        try:
            handle = self._wait(self.pending_goal, 5.0)
            self.pending_goal = None
            if not handle.accepted:
                return False
            self.active_handle = handle
            self._route_event('accepted', target_index, handle, motion=motion)
            self.navigation_result = handle.get_result_async()
            while not self.navigation_result.done():
                spin_node(self, timeout_sec=0.05)
                if (self.stop_requested
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    reason = self._guard_failure(False) or 'operator_or_timeout'
                    self.emit('reverse_interrupted', reason=reason)
                    if not self.stop_requested and self._input_gap_recoverable(reason):
                        self._retry_guard_reason = reason
                    return False
                if handoff_target is not None and not self._operator_link_ready():
                    self.emit('reverse_interrupted', reason='operator_link_lost')
                    return False
                if (handoff_target is not None and not self.stop_requested
                        and math.dist(self._odom_pose()[:2], handoff_target) <= position_handoff_m):
                    self.emit('reverse_position_handoff', motion=motion,
                              tolerance_m=position_handoff_m, target_odom=list(handoff_target),
                              final_parking=False)
                    return True
            wrapped = self.navigation_result.result()
            self._route_event('result', target_index, handle, motion=motion,
                              terminal_status_code=int(wrapped.status),
                              nav2_error_code=int(wrapped.result.error_code))
            self.navigation_result = None
            # A progress abort at an intermediate XY endpoint may only mean
            # the heading checker could not finish. The caller must reobserve
            # and align the face; this never confirms final docking.
            if (handoff_target is not None and not self.stop_requested
                    and wrapped.status == GoalStatus.STATUS_ABORTED
                    and wrapped.result.error_code == 105
                    and self._navigation_ready(require_fresh_amcl=False)
                    and self._operator_link_ready()
                    and math.dist(self._odom_pose()[:2], handoff_target) <= position_handoff_m):
                self.emit('reverse_position_handoff', motion=motion,
                          tolerance_m=position_handoff_m, target_odom=list(handoff_target),
                          nav2_error_code=105, final_parking=False)
                return True
            if (self.stop_requested or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code):
                return False
            if not final:
                return True
            if verify_contract is not None:
                return self._verify_parking_stop(
                    target_index, verify_waypoint or self.waypoints[-1], handle,
                    contract=verify_contract)
            return self._verify_parking_stop(target_index, self.waypoints[-1], handle)
        finally:
            if not self.finish_navigation():
                raise RuntimeError('reverse action cancellation unconfirmed')

    def wait_parked(self, dwell_s=5.0):
        """Count dwell only while the selected pose remains stopped and aligned."""
        if self.selected_pose is None:
            raise ValueError('parking dwell requires a selected destination')
        index = len(self.waypoints) - 1
        pose = self.selected_pose
        waypoint = {'x': pose['x_m'], 'y': pose['y_m'], 'yaw': pose['yaw_rad']}
        # A navigation deadline must not expire during the subsequent dwell.
        self.run_deadline_s = None
        self.emit('parked_dwell_started', dwell_s=dwell_s)
        confirm_s = min(dwell_s, PARKED_CONFIRM_MAX_S)
        success = self._verify_parking_stop(index, waypoint, None, hold_s=confirm_s)
        if success and dwell_s > confirm_s:
            success = self._hold_still(dwell_s - confirm_s)
        self.emit('parked_dwell_complete' if success else 'parked_dwell_failed',
                  dwell_s=dwell_s, confirmation=self.confirmation)
        return success

    def _hold_still(self, seconds):
        """Keep a confirmed stop for seconds more; fail if odom moved or went unread."""
        try:
            x0, y0, yaw0 = self._odom_pose()
        except RuntimeError as error:
            self.emit('parked_hold_unobserved', reason=str(error))
            return False
        end_s = time.monotonic() + seconds
        read_s = time.monotonic()
        while time.monotonic() < end_s:
            if self.stop_requested:
                return False
            spin_node(self, timeout_sec=0.1)
            try:
                x, y, yaw = self._odom_pose()
            except RuntimeError as error:
                if time.monotonic() - read_s > self.parking_contract['observation_timeout_s']:
                    self.emit('parked_hold_unobserved', reason=str(error))
                    return False
                continue
            read_s = time.monotonic()
            moved_m = math.dist((x0, y0), (x, y))
            turned = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
            if moved_m > PARKED_HOLD_MAX_MOVE_M or turned > PARKED_HOLD_MAX_TURN_RAD:
                self.emit('parked_hold_moved', moved_m=moved_m, turned_rad=turned)
                return False
        return True

    def serve(self, table_id, execute=False):
        """Dock at home, hold five seconds, serve one table, then dock home."""
        # Resolve every required destination before dispatching the first goal.
        home_pose(self.registry)
        candidates(self.registry, table_id)
        self.emit('serving_started', selected_table=table_id, dwell_s=5.0,
                  pour_stage=False, execute=execute)
        steps = [
            ('initial_home', lambda: self.go_home(execute=execute)),
            ('home_dwell', lambda: self.wait_parked(5.0) if execute else True),
            ('table', lambda: self.visit(table_id, execute=execute)),
            ('table_dwell', lambda: self.wait_parked(5.0) if execute else True),
            ('return_home', lambda: self.go_home(execute=execute)),
        ]
        for phase, action in steps:
            if self.stop_requested:
                self.emit('serving_failed', phase=phase, reason='operator_or_timeout')
                return False
            self.emit('serving_phase_started', phase=phase)
            if not action():
                self.emit('serving_failed', phase=phase)
                return False
        self.run_deadline_s = None
        self.emit('serving_complete' if execute else 'serving_plan_checks_complete',
                  selected_table=table_id, physical_accuracy='NOT_MEASURED')
        return True

    def roundtrip(self, table_ids, execute=False, dwell_s=20.0,
                  box_timeout_s=45.0):
        """Visit selected box stations in order, then return to taught home."""
        destinations = [table_ids] if isinstance(table_ids, str) else list(table_ids)
        if (not destinations or any(
                not isinstance(item, str) or not item for item in destinations)):
            raise ValueError('roundtrip requires at least one destination ID')
        if len(destinations) != len(set(destinations)):
            raise ValueError('roundtrip destination IDs must be unique')
        home = home_pose(self.registry)
        self.emit('roundtrip_started', destination_ids=destinations,
                  home_pose=[home[key] for key in ('x_m', 'y_m', 'yaw_rad')],
                  dwell_s=float(dwell_s))
        for index, table_id in enumerate(destinations):
            self.emit('station_started', station_id=table_id, station_index=index)
            if not self.visit(table_id, execute=execute):
                self.emit('roundtrip_failed', phase='destination',
                          station_id=table_id, station_index=index)
                return False
            if execute and not self.wait_for_box(dwell_s, box_timeout_s):
                self.emit('roundtrip_failed', phase='box_wait',
                          station_id=table_id, station_index=index)
                return False
            self.emit('station_complete', station_id=table_id, station_index=index)
        if not self.go_home(execute=execute):
            self.emit('roundtrip_failed', phase='home')
            return False
        self.emit('roundtrip_complete' if execute else 'roundtrip_planned')
        return True


def parse_args(argv):
    """Keep registry maintenance, read-only planning and execution explicit."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    initialize = commands.add_parser('init')
    initialize.add_argument('--map', type=Path, required=True)
    initialize.add_argument('--keepout', type=Path, required=True)
    initialize.add_argument('--registry', type=Path, required=True)
    listing = commands.add_parser('list')
    listing.add_argument('--registry', type=Path, required=True)
    for command in ('teach', 'go', 'teach-home', 'home', 'roundtrip', 'serve'):
        subparser = commands.add_parser(command)
        subparser.add_argument('--registry', type=Path, required=True)
        subparser.add_argument('--log', type=Path, required=True)
        if command == 'roundtrip':
            subparser.add_argument(
                '--table-id', dest='table_ids', action='append', required=True,
                help='Ordered destination ID; repeat this option for each station')
        elif command not in ('teach-home', 'home'):
            subparser.add_argument('--table-id', required=True)
        if command == 'teach':
            subparser.add_argument('--pose-id', required=True)
            subparser.add_argument('--priority', type=int, choices=(1, 2), default=1)
            subparser.add_argument('--approach-offset-m', type=float, default=0.5)
            subparser.add_argument('--replace', action='store_true')
            subparser.add_argument('--target-front-gap-m', type=float,
                                   help='Requested chassis-front gap; does not shift taught pose')
            subparser.add_argument('--measured-front-gap-m', type=float,
                                   help='Gap measured at this stationary teaching pose')
            subparser.add_argument('--gap-measurement-note',
                                   help='Measurement method and physical reference')
        elif command == 'teach-home':
            subparser.add_argument('--pose-id', default='home_dock')
            subparser.add_argument('--approach-offset-m', type=float, default=0.7)
            subparser.add_argument('--replace', action='store_true')
            subparser.add_argument('--parking-direction', choices=('forward', 'reverse'),
                                   default='forward')
        else:
            subparser.add_argument('--execute', action='store_true')
            if command == 'home':
                # Operational rule, not a check: at a 5 cm box stop the face is
                # inside LiDAR range_min (0.28 m) and the depth minimum
                # (0.35 m), so no sensor sees it before this command turns.
                subparser.description = (
                    'Return to the taught home pose. Do not run this within about '
                    '0.6 m in front of a box: at a box stop the face is inside the '
                    'LiDAR range_min (0.28 m) and the depth minimum (0.35 m), so no '
                    'sensor protects the first turn. After a box approach, use '
                    'box_service --return-home, which reverses straight away from '
                    'the face before turning.')
                subparser.add_argument(
                    '--parking-contract', type=Path,
                    help='Session parking contract; the dock keeps parking_contract.yaml')
            if command == 'roundtrip':
                subparser.add_argument('--dwell-s', type=float, default=20.0)
                subparser.add_argument('--box-timeout-s', type=float, default=45.0)
    parsed = parser.parse_args(remove_ros_args(args=argv)[1:])
    if parsed.command == 'teach':
        try:
            validate_gap_measurement(parsed.measured_front_gap_m, parsed.gap_measurement_note)
            if parsed.target_front_gap_m is not None and (
                    not math.isfinite(parsed.target_front_gap_m)
                    or parsed.target_front_gap_m <= 0):
                raise ValueError('target_front_gap_m must be finite and positive')
        except ValueError as error:
            parser.error(str(error))
    if parsed.command == 'roundtrip':
        if (not math.isfinite(parsed.dwell_s) or parsed.dwell_s <= 0.0
                or not math.isfinite(parsed.box_timeout_s)
                or parsed.box_timeout_s <= parsed.dwell_s):
            parser.error('roundtrip box timeout must exceed a positive dwell')
        if len(parsed.table_ids) != len(set(parsed.table_ids)):
            parser.error('roundtrip destination IDs must be unique')
    return parsed


def main(args=None):
    """Operate on the local Nav2 graph only when a ROS command was requested."""
    argv = args if args is not None else sys.argv
    parsed = parse_args(argv)
    node = None
    ros_started = False
    handlers = {}
    try:
        if parsed.command == 'init':
            save_registry(parsed.registry, new_registry(parsed.map, parsed.keepout))
            print(f'created: {parsed.registry}')
            return
        registry = load_registry(parsed.registry)
        if parsed.command == 'list':
            print(json.dumps(registry, ensure_ascii=False, indent=2, allow_nan=False))
            return
        contract_path = (Path(get_package_share_directory('jdamr_cube_navigation'))
                         / 'config/parking_contract.yaml')
        # Only `home` may run under a precision session's contract; the dock
        # itself is always accepted with the user's parking contract.
        contract = load_parking_contract(
            getattr(parsed, 'parking_contract', None) or contract_path)
        home_contract = load_parking_contract(contract_path)
        with parsed.log.open('x', encoding='utf-8') as stream:
            rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
            ros_started = True
            node = ServiceRoute(registry, contract, stream, home_contract=home_contract)
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.signal(signum, lambda *_: node.request_stop())
            try:
                if parsed.command in ('teach', 'teach-home'):
                    table_id = parsed.table_id if parsed.command == 'teach' else 'home_dock'
                    node.table_id, node.pose_id = table_id, parsed.pose_id
                    node.verify_live_maps()
                    actual_pose, observation = node.capture_stationary_pose()
                    if parsed.command == 'teach':
                        pose = taught_pose(parsed.pose_id, actual_pose, observation,
                                           parsed.priority, parsed.approach_offset_m,
                                           parsed.measured_front_gap_m,
                                           parsed.gap_measurement_note,
                                           parsed.target_front_gap_m)
                        updated = add_pose(
                            registry, parsed.table_id, pose, parsed.replace)
                    else:
                        pose = taught_pose(
                            parsed.pose_id, actual_pose, observation,
                            approach_offset_m=parsed.approach_offset_m)
                        pose['parking_direction'] = parsed.parking_direction
                        updated = set_home_pose(registry, pose, parsed.replace)
                    # Detect an intervening edit before replacing the registry.
                    if load_registry(parsed.registry) != registry:
                        raise RuntimeError('registry changed during teaching; capture not saved')
                    save_registry(parsed.registry, updated, replace=True)
                    node.selected_pose = pose
                    node.emit('taught', pose=pose)
                    ok = True
                elif parsed.command == 'go':
                    ok = node.visit(parsed.table_id, parsed.execute)
                elif parsed.command == 'serve':
                    ok = node.serve(parsed.table_id, parsed.execute)
                elif parsed.command == 'home':
                    ok = node.go_home(parsed.execute)
                else:
                    ok = node.roundtrip(
                        parsed.table_ids, parsed.execute,
                        parsed.dwell_s, parsed.box_timeout_s)
            except Exception as error:
                node.emit('failed', reason=f'{type(error).__name__}: {error}')
                raise
            finally:
                if not node.finish_navigation():
                    raise RuntimeError(
                        'navigation cancellation unconfirmed; inspect the robot stop state')
            if not ok:
                raise RuntimeError('service task did not complete; see JSONL log')
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        if node is not None:
            node.destroy_node()
        if ros_started:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
