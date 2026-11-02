"""Preflight and execute a fail-closed saved-map corridor route."""

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from jdamr_cube_navigation.dedicated_tf import DedicatedTfListener
from jdamr_cube_navigation.mobile_manipulator_protection import (
    evaluate_travel_pose, load_mobile_manipulator_protection,
)
from jdamr_cube_navigation.parking import load_parking_contract, ParkingHold
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
ODOM_QOS = QoSProfile(depth=1)
NAVIGATION_BEHAVIOR_TREES = {
    'corridor': 'navigate_to_pose_corridor_fail_fast.xml',
    'obstacle_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
    'obstacle_base_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
    'new_base_revisit_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
}
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
}
# A transit goal is judged by the position/general goal checkers' xy
# tolerance; test_t13 pins this value to new_base_nav2_params.yaml.
TRANSIT_PLAN_END_TOLERANCE_M = 0.15
# A blocked path is held, then the current leg is retried; a serving robot waits for
# a person to step aside (Pudu: 30 s before replanning; Nav2 default tree: 5 s). The
# start values follow evaluation/20261002_ANOMALY_HANDLING_RESEARCH.md P9.
PATH_BLOCKED_WAIT_S = 20.0
PATH_BLOCKED_RETRIES = 3
# Nav2 Jazzy codes of a leg that could not move on: FollowPath 104 patience
# exceeded, 105 no progress, 106 no valid control; ComputePathToPose and
# ComputePathThroughPoses 205/305 start occupied, 206/306 goal occupied,
# 208/308 no valid path.
PATH_BLOCKED_NAV2_CODES = frozenset({104, 105, 106, 205, 206, 208, 305, 306, 308})
# No path at all: the hold re-plans every PATH_BLOCKED_PROBE_S and goes on as soon
# as one exists. A controller stop (104-106) goes on once the band ahead of the
# chassis is empty on PATH_BLOCKED_CLEAR_PROBES checks in a row (2026-10-02 10:37:
# the blocker had left but the full 20 s hold still ran).
PATH_BLOCKED_PLANNER_CODES = frozenset({205, 206, 208, 305, 306, 308})
PATH_BLOCKED_PROBE_S = 2.0
PATH_BLOCKED_CLEAR_PROBES = 2
# Odom trail kept for backing out the way the base came in: a point every 2 cm, 4 m.
ODOM_TRAIL_STEP_M = 0.02
ODOM_TRAIL_POINTS = 200
# A step from the trail end longer than this is not motion (0.12 m/s): odom reset.
ODOM_TRAIL_RESET_M = 0.25
# Measured reverse speed above which the trail unwinds instead of growing.
ODOM_TRAIL_REVERSE_MPS = 0.01
# A yaw rate above the stopped limit with no translation is motion once it lasts
# this long; one gyro sample is not (2026-10-29 dock init: a single 0.021 rad/s
# sample stopped the capture twice, 3 of 105 stationary samples exceeded 0.02).
# Translation, non-finite values and nonzero commands still count at once, and
# so does a rate above this multiple of the stopped limit (bounds what is missed
# to about 0.2 deg within the confirm window).
PARKING_TURN_CONFIRM_S = 0.08
PARKING_TURN_IMMEDIATE_FACTOR = 2.0


def _expanded_path(value, parent=None):
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not path.is_absolute() and parent is not None:
        path = parent / path
    return path.resolve()


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def revisit_plan_length_ok(actual_m, declared_m):
    """Reject a detour that no longer resembles the recorded corridor route."""
    return (math.isfinite(actual_m) and math.isfinite(declared_m)
            and declared_m > 0 and actual_m <= declared_m * 1.3)


def revisit_goal_witness(start_xy, goal_xy, final_xy, odom_delta_m,
                         goal_tolerance_m, min_motion_m):
    """Require independent motion and a localized final pose for a goal."""
    if start_xy is None or final_xy is None:
        return False
    values = (*start_xy, *goal_xy, *final_xy, odom_delta_m,
              goal_tolerance_m, min_motion_m)
    if (not all(math.isfinite(value) for value in values) or
            not 0.0 < goal_tolerance_m <= 0.35 or
            min_motion_m <= 0.0 or odom_delta_m < 0.0):
        return False
    expected_m = math.dist(start_xy, goal_xy)
    remaining_m = math.dist(final_xy, goal_xy)
    return (remaining_m <= goal_tolerance_m and
            (expected_m <= goal_tolerance_m or
             odom_delta_m >= min(min_motion_m, expected_m * 0.1)))


def current_map_xy(map_to_odom, odom_to_base):
    """Compose the last map correction with the latest odom base pose."""
    yaw = _quaternion_yaw(map_to_odom.transform.rotation)
    map_offset = map_to_odom.transform.translation
    odom_position = odom_to_base.transform.translation
    return (
        map_offset.x + math.cos(yaw) * odom_position.x -
        math.sin(yaw) * odom_position.y,
        map_offset.y + math.sin(yaw) * odom_position.x +
        math.cos(yaw) * odom_position.y,
    )


def revisit_map_correction_ok(age_s, odom_since_correction_m):
    """Use recent localization, or a quiet robot after the last correction."""
    if not math.isfinite(age_s):
        return False
    if -1.0 <= age_s <= 5.0:
        return True
    return (5.0 < age_s <= 25.0 and
            math.isfinite(odom_since_correction_m) and
            0.0 <= odom_since_correction_m <= 0.1)


def odom_distance_since_stamp(history, total_m, stamp_ns):
    """Bound travel after a correction using header time, not receipt time."""
    candidates = (sample for sample in history if sample[0] <= stamp_ns)
    latest = max(candidates, key=lambda sample: sample[0], default=None)
    return total_m - latest[1] if latest is not None else math.inf


def finite_localization_xy(position):
    """Reject invalid AMCL coordinates without using delayed pose as a goal."""
    return (position is not None and len(position) == 2 and
            all(math.isfinite(value) for value in position))


def _positive_finite_config(config, name, default):
    value = float(config.get(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def load_route(route_yaml):
    """Load a route and verify the immutable map/mask artifacts it names."""
    route_yaml = route_yaml.resolve()
    with route_yaml.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('route schema_version must be 1')
    waypoints = config.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError('route requires at least two waypoints')
    identifiers = set()
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            raise ValueError('each waypoint must be a mapping')
        waypoint_id = str(waypoint.get('id', '')).strip()
        if not waypoint_id or waypoint_id in identifiers:
            raise ValueError('waypoint ids must be non-empty and unique')
        identifiers.add(waypoint_id)
        for field in ('x', 'y'):
            value = float(waypoint[field])
            if not math.isfinite(value):
                raise ValueError(f'{waypoint_id} {field} must be finite')
            waypoint[field] = value
        if 'yaw' in waypoint:
            waypoint['yaw'] = float(waypoint['yaw'])
            if not math.isfinite(waypoint['yaw']):
                raise ValueError(f'{waypoint_id} yaw must be finite')
    start_pose = config.get('start_pose')
    if start_pose is not None:
        if not isinstance(start_pose, dict):
            raise ValueError('start_pose must be a mapping')
        for field in ('x', 'y'):
            value = float(start_pose[field])
            if not math.isfinite(value):
                raise ValueError(f'start_pose {field} must be finite')
            start_pose[field] = value

    map_yaml = _expanded_path(config['map_yaml'], route_yaml.parent)
    mask_yaml = _expanded_path(
        config['keepout_mask_yaml'], route_yaml.parent)
    for label, path in (('map', map_yaml), ('keepout mask', mask_yaml)):
        if not path.is_file():
            raise FileNotFoundError(f'{label} YAML not found: {path}')
    with mask_yaml.open(encoding='utf-8') as stream:
        mask_metadata = yaml.safe_load(stream)
    mask_image = _expanded_path(mask_metadata['image'], mask_yaml.parent)
    actual_hash = _sha256(mask_image)
    expected_hash = str(config.get('expected_mask_sha256', ''))
    if not expected_hash or actual_hash != expected_hash:
        raise ValueError(
            f'keepout mask hash mismatch: expected={expected_hash} '
            f'actual={actual_hash}')
    config['route_yaml'] = route_yaml
    config['map_yaml'] = map_yaml
    config['keepout_mask_yaml'] = mask_yaml
    config['keepout_mask_image'] = mask_image
    return config


def _goal_uuid(handle):
    """Render the action goal identifier without changing its byte order."""
    return bytes(handle.goal_id.uuid).hex()


def validate_parking_route(config, contract):
    """Require an explicit final orientation in the declared parking frame."""
    if config.get('frame_id', 'map') != contract['reference_frame']:
        raise ValueError('parking route frame must match parking contract')
    final = config['waypoints'][-1]
    if 'yaw' not in final or not math.isfinite(float(final['yaw'])):
        raise ValueError('final parking waypoint requires a finite yaw [rad]')


def _quaternion_yaw(rotation):
    values = (rotation.x, rotation.y, rotation.z, rotation.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('parking TF quaternion must be finite')
    if abs(sum(value * value for value in values) - 1.0) > 0.001:
        raise ValueError('parking TF quaternion must be normalized')
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def spin_node(node, timeout_sec):
    """
    Run the callbacks of node that are ready within timeout_sec.

    A route node spins on its own events executor (rclpy.experimental). rclpy's
    wait-set executor rebuilt a set of about 80 entities for each of the ~120
    messages a second a route receives (odom 49 Hz, tf 39 Hz, cmd_vel 20 Hz, scan
    10 Hz): box_service took 78 % of a Pi 4 core during the 2026-10-06 14:11 cycle.
    Replaying that run's topics on the PC, the same callbacks took 3.5 % of a core
    on the events executor instead of 17.3 % (with or without re-adding the node
    on each spin). Nodes without one (tests, JDAMR_EXECUTOR=waitset) spin as before.
    """
    executor = getattr(node, 'events_executor', None)
    if executor is None:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    else:
        executor.spin_once(timeout_sec=timeout_sec)


def _events_executor(node):
    """Return node's own events executor, or None for the wait-set executor."""
    if os.environ.get('JDAMR_EXECUTOR', 'events') == 'waitset':
        return None
    try:
        from rclpy.experimental import EventsExecutor
    except ImportError:
        return None
    executor = EventsExecutor(context=node.context)
    executor.add_node(node)
    return executor


class CorridorRoute(Node):
    """Validate every leg, then send one explicit-BT goal at a time."""

    def __init__(self, config, start_index=0, navigation_profile='corridor',
                 parking_contract=None):
        super().__init__('jdamr_corridor_route')
        self.events_executor = _events_executor(self)
        self.config = config
        self.start_index = start_index
        self.waypoints = config['waypoints'][start_index:]
        if not self.waypoints:
            raise ValueError('start_index is past the final waypoint')
        self.minimum_battery_v = float(config.get('minimum_battery_v', 10.5))
        self.freshness_s = float(config.get('sensor_freshness_s', 2.5))
        # Battery freshness and sensor freshness are separate contracts: a
        # delayed battery sample is not equivalent to driving blind.  The
        # measured rate and threshold history live in the evaluation report.
        self.battery_freshness_s = float(
            config.get('battery_freshness_s', 30.0))
        self.max_amcl_covariance = (
            float(config.get('max_amcl_x_covariance', 0.5)),
            float(config.get('max_amcl_y_covariance', 0.5)),
        )
        self.amcl_freshness_s = float(
            config.get('amcl_freshness_s', 15.0))
        self.revisit_motion_amcl_freshness_s = _positive_finite_config(
            config, 'revisit_motion_amcl_freshness_s', 25.0)
        self.revisit_motion_min_distance_m = _positive_finite_config(
            config, 'revisit_motion_min_distance_m', 0.25)
        self.revisit_motion_min_rotation_rad = _positive_finite_config(
            config, 'revisit_motion_min_rotation_rad', 0.5)
        self.revisit_goal_amcl_tolerance_m = _positive_finite_config(
            config, 'revisit_goal_amcl_tolerance_m', 0.35)
        if self.revisit_goal_amcl_tolerance_m > 0.35:
            raise ValueError('revisit_goal_amcl_tolerance_m exceeds bound')
        self.max_resume_start_distance_m = float(
            config.get('max_resume_start_distance_m', 6.0))
        if start_index > 0:
            self.start_reference = self.waypoints[0]
            self.start_distance_limit_m = self.max_resume_start_distance_m
            self.start_check_kind = 'resume'
        else:
            self.start_reference = config.get('start_pose', self.waypoints[0])
            self.start_distance_limit_m = float(
                config.get(
                    'max_route_start_distance_m',
                    self.max_resume_start_distance_m))
            self.start_check_kind = 'route'
        self.start_check_pending = True
        self.stop_requested = False
        self.samples = {'battery': None, 'odom': None, 'scan': None}
        self.battery_voltage = None
        self.amcl_seen = None
        self.amcl_covariance = None
        self.amcl_position = None
        self.odom_last_pose = None
        # Where the base just drove, for backing out the same way (ServiceRoute._retrace).
        self.odom_trail = deque(maxlen=ODOM_TRAIL_POINTS)
        self.odom_total_distance_m = 0.0
        self.odom_history = deque(maxlen=3000)
        self.amcl_motion_distance_m = 0.0
        self.amcl_motion_rotation_rad = 0.0
        self.navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.create_subscription(
            BatteryState, '/battery_state', self._battery_callback, 10)
        # The base driver latches /emergency_stop until its reset service; while
        # engaged every guard fails, so goals cancel and no departure starts.
        self.emergency_stop_engaged = False
        self.create_subscription(
            Bool, '/emergency_stop_state', self._emergency_stop_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_callback, ODOM_QOS)
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback,
            AMCL_QOS)
        package_share = get_package_share_directory('jdamr_cube_navigation')
        self.mobile_manipulator_protection = None
        self.travel_pose_status = None
        if navigation_profile == 'obstacle_candidate':
            protection_path = (
                Path(package_share) / 'config'
                / 'mobile_manipulator_protection.yaml')
            self.mobile_manipulator_protection = (
                load_mobile_manipulator_protection(protection_path))
            self.samples['joint_states'] = None
            self.create_subscription(
                JointState,
                self.mobile_manipulator_protection['travel_pose'][
                    'source_topic'],
                self._joint_state_callback, qos_profile_sensor_data)
        self.behavior_tree = os.path.join(
            package_share, 'behavior_trees',
            NAVIGATION_BEHAVIOR_TREES[navigation_profile])
        self.navigation_profile = navigation_profile
        if navigation_profile == 'new_base_revisit_candidate':
            self.revisit_tf = Buffer()
            self.revisit_tf_listener = TransformListener(self.revisit_tf, self)
        self.parking_contract = parking_contract
        self._parameter_readers = {}
        self.parking_behavior_tree = os.path.join(
            package_share, 'behavior_trees', 'navigate_to_pose_parking.xml')
        self.parking_odom = None
        self.parking_command = None
        self.parking_motion_revision = 0
        self.parking_observation_diagnostics = None
        if parking_contract is not None:
            validate_parking_route(config, parking_contract)
            self._start_parking_tf()
            self.create_subscription(
                Twist, '/cmd_vel', self._parking_command_callback,
                qos_profile_sensor_data)
        self.get_logger().info(
            'navigation behavior tree selected: '
            f'profile={navigation_profile} source=navigation_profile '
            f'path={self.behavior_tree}')
        self._last_feedback = 0.0

    def destroy_node(self):
        """Release the node's events executor before the node."""
        executor = getattr(self, 'events_executor', None)
        if executor is not None:
            self.events_executor = None
            executor.remove_node(self)
            executor.shutdown()
        self._stop_parking_tf()
        return super().destroy_node()

    def _start_parking_tf(self):
        """Receive TF on a dedicated node and daemon thread.

        Stamped lookups (depth image, LiDAR scan) failed while /tf sat queued
        behind a control tick of this route node.
        """
        self.parking_tf = Buffer()
        self.parking_tf_listener = DedicatedTfListener(
            self.parking_tf, self, 'jdamr_corridor_route_tf')

    def _stop_parking_tf(self):
        listener = getattr(self, 'parking_tf_listener', None)
        if listener is None:
            return
        self.parking_tf_listener = None
        listener.shutdown()

    def _route_event(self, event, route_index, handle, **fields):
        """Emit one stable JSON line for goal lifecycle correlation."""
        record = {
            'event': event,
            'goal_uuid': _goal_uuid(handle),
            'waypoint_index': self.start_index + route_index + 1,
            'waypoint_total': len(self.config['waypoints']),
            'waypoint_id': self.waypoints[route_index]['id'],
            'navigation_profile': self.navigation_profile,
            'terminal_status': None,
            'terminal_status_code': None,
        }
        record.update(fields)
        self.get_logger().info(
            'route_event ' + json.dumps(
                record, sort_keys=True, separators=(',', ':')))

    def request_stop(self):
        """Ask the current action to cancel without invalidating ROS context."""
        self.stop_requested = True

    def request_operator_stop(self):
        """Stop on the operator's own request; no operator call follows it."""
        self.operator_stopped = True
        self.request_stop()

    def _emergency_stop_callback(self, message):
        self.emergency_stop_engaged = bool(message.data)

    def engage_emergency_stop(self, reason):
        """Latch the base driver stop; only its reset service releases it."""
        publisher = getattr(self, 'emergency_stop_pub', None)
        if publisher is not None:
            publisher.publish(Bool(data=True))
        self.get_logger().error(f'emergency stop requested: {reason}')

    def _battery_callback(self, message):
        self.battery_voltage = float(message.voltage)
        self.samples['battery'] = time.monotonic()

    @staticmethod
    def _extend_odom_trail(trail, pose, linear_mps):
        """
        Keep the trail as the way the base drove forward up to where it stands.

        Driving forward appends a pose every ODOM_TRAIL_STEP_M. Reversing appends
        nothing and drops the poses now ahead of the base, so backing out unwinds
        the trail and a later retrace continues further back instead of driving
        forward over a stretch it reversed. A pose far from the trail end (an odom
        reset, or the end of a long reverse such as a dock entry) starts a new trail.
        """
        x_m, y_m, yaw = pose
        if trail and math.hypot(x_m - trail[-1][0], y_m - trail[-1][1]) > ODOM_TRAIL_RESET_M:
            trail.clear()
        if linear_mps < -ODOM_TRAIL_REVERSE_MPS:
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            while trail and ((trail[-1][0] - x_m) * cos_yaw
                             + (trail[-1][1] - y_m) * sin_yaw) > 0.0:
                trail.pop()
        elif not trail or math.hypot(
                x_m - trail[-1][0], y_m - trail[-1][1]) >= ODOM_TRAIL_STEP_M:
            trail.append(pose)

    def _odom_callback(self, _message):
        self.samples['odom'] = time.monotonic()
        self.latest_motion = (
            self.samples['odom'],
            math.hypot(_message.twist.twist.linear.x, _message.twist.twist.linear.y),
            float(_message.twist.twist.angular.z))
        position = _message.pose.pose.position
        yaw = _quaternion_yaw(_message.pose.pose.orientation)
        odom_pose = (position.x, position.y, yaw)
        if self.odom_last_pose is not None:
            previous_x, previous_y, previous_yaw = self.odom_last_pose
            step_m = math.hypot(
                position.x - previous_x, position.y - previous_y)
            self.amcl_motion_distance_m += step_m
            self.odom_total_distance_m += step_m
            self.amcl_motion_rotation_rad += abs(math.atan2(
                math.sin(yaw - previous_yaw),
                math.cos(yaw - previous_yaw)))
        self.odom_last_pose = odom_pose
        trail = getattr(self, 'odom_trail', None)
        if trail is not None:
            self._extend_odom_trail(trail, odom_pose, float(_message.twist.twist.linear.x))
        history = getattr(self, 'odom_history', None)
        if history is not None:
            stamp_ns = (_message.header.stamp.sec * 1_000_000_000 +
                        _message.header.stamp.nanosec)
            if stamp_ns > 0:
                history.append((stamp_ns, self.odom_total_distance_m))
        if getattr(self, 'parking_contract', None) is not None:
            self.parking_odom = (
                self.samples['odom'], _message.header.stamp,
                math.hypot(_message.twist.twist.linear.x,
                           _message.twist.twist.linear.y),
                float(_message.twist.twist.angular.z))
            linear_mps, angular_radps = self.parking_odom[2:]
            if (not math.isfinite(linear_mps) or
                    not math.isfinite(angular_radps) or
                    linear_mps > self.parking_contract['stopped_linear_mps']):
                self.parking_motion_revision += 1
            elif abs(angular_radps) > self.parking_contract['stopped_angular_radps']:
                turning_since_s = getattr(self, '_parking_turn_since_s', None)
                if turning_since_s is None:
                    self._parking_turn_since_s = turning_since_s = self.samples['odom']
                if (self.samples['odom'] - turning_since_s >= PARKING_TURN_CONFIRM_S
                        or abs(angular_radps) > PARKING_TURN_IMMEDIATE_FACTOR
                        * self.parking_contract['stopped_angular_radps']):
                    self.parking_motion_revision += 1
            else:
                self._parking_turn_since_s = None

    def _parking_command_callback(self, message):
        self.parking_command = (
            time.monotonic(), math.hypot(message.linear.x, message.linear.y),
            float(message.angular.z))
        if (not math.isfinite(self.parking_command[1]) or
                not math.isfinite(self.parking_command[2]) or
                self.parking_command[1] != 0.0 or
                self.parking_command[2] != 0.0):
            # Preserve excursions between odometry samples instead of letting
            # a later zero overwrite evidence that the hold was interrupted.
            self.parking_motion_revision += 1

    def _scan_callback(self, _message):
        self.samples['scan'] = time.monotonic()

    def _joint_state_callback(self, message):
        self.samples['joint_states'] = time.monotonic()
        try:
            self.travel_pose_status = evaluate_travel_pose(
                list(message.name), list(message.position),
                self.mobile_manipulator_protection)
        except (KeyError, TypeError, ValueError) as error:
            self.travel_pose_status = {
                'status': 'FAIL', 'error': f'{type(error).__name__}: {error}'}

    def _amcl_callback(self, message):
        self.amcl_seen = time.monotonic()
        self.amcl_motion_distance_m = 0.0
        self.amcl_motion_rotation_rad = 0.0
        covariance = message.pose.covariance
        self.amcl_covariance = (float(covariance[0]), float(covariance[7]))
        self.amcl_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _guard_failure(self, require_fresh_amcl=True, covariance_limits=None):
        """Describe the exact fail-closed input instead of a generic stop."""
        if getattr(self, 'emergency_stop_engaged', False):
            return 'emergency stop engaged'
        now = time.monotonic()
        for name, timestamp in self.samples.items():
            if timestamp is None:
                return f'{name} missing'
            age = now - timestamp
            # Scan and odometry going quiet means the robot is driving blind.
            # Battery only needs to be recent enough to trust the voltage.
            limit = (self.battery_freshness_s if name == 'battery'
                     else self.freshness_s)
            if age > limit:
                return (
                    f'{name} stale: age={age:.3f}s '
                    f'limit={limit:.3f}s')
        if self.battery_voltage is None:
            return 'battery voltage missing'
        if not math.isfinite(self.battery_voltage):
            return (
                'battery voltage non-finite: '
                f'value={self.battery_voltage}')
        if self.battery_voltage < self.minimum_battery_v:
            return (
                f'battery low: voltage={self.battery_voltage:.3f}V '
                f'limit={self.minimum_battery_v:.3f}V')
        if getattr(self, 'mobile_manipulator_protection', None) is not None:
            status = self.travel_pose_status
            if status is None:
                return 'travel pose status missing'
            if status.get('status') != 'PASS':
                return 'travel pose invalid: ' + json.dumps(
                    status, sort_keys=True, separators=(',', ':'))
        if self.amcl_seen is None or self.amcl_covariance is None:
            return 'AMCL pose missing'
        amcl_age_s = now - self.amcl_seen
        motion_amcl_required = (
            getattr(self, 'navigation_profile', None) ==
            'new_base_revisit_candidate' and
            (getattr(self, 'amcl_motion_distance_m', 0.0) >=
             self.revisit_motion_min_distance_m or
             getattr(self, 'amcl_motion_rotation_rad', 0.0) >=
             self.revisit_motion_min_rotation_rad))
        amcl_limit_s = (self.revisit_motion_amcl_freshness_s
                        if motion_amcl_required
                        else self.amcl_freshness_s)
        if (require_fresh_amcl or motion_amcl_required) and \
                amcl_age_s > amcl_limit_s:
            return (
                f'AMCL pose stale: age={amcl_age_s:.3f}s '
                f'limit={amcl_limit_s:.3f}s')
        for axis, covariance, limit in zip(
                ('x', 'y'), self.amcl_covariance,
                covariance_limits or self.max_amcl_covariance):
            if not math.isfinite(covariance):
                return (
                    f'AMCL {axis} covariance non-finite: '
                    f'value={covariance}')
            if covariance > limit:
                return (
                    f'AMCL {axis} covariance high: value={covariance:.3f} '
                    f'limit={limit:.3f}')
        if self.start_check_pending:
            if self.amcl_position is None:
                return f'AMCL position missing for {self.start_check_kind} start'
            for axis, position in zip(('x', 'y'), self.amcl_position):
                if not math.isfinite(position):
                    return (
                        f'AMCL {self.start_check_kind} start {axis} '
                        f'non-finite: value={position}')
            distance_m = math.hypot(
                self.amcl_position[0] - self.start_reference['x'],
                self.amcl_position[1] - self.start_reference['y'],
            )
            if distance_m > self.start_distance_limit_m:
                return (
                    f'AMCL {self.start_check_kind} start too far: '
                    f'distance={distance_m:.3f}m '
                    f'limit={self.start_distance_limit_m:.3f}m')
        return None

    def _navigation_ready(self, require_fresh_amcl=True):
        return self._guard_failure(require_fresh_amcl) is None

    @staticmethod
    def _input_gap_recoverable(reason):
        """Do not retry operator stops, invalid geometry, low power or bad poses."""
        return isinstance(reason, str) and reason.startswith((
            'scan stale:', 'odom stale:', 'AMCL pose stale:', 'AMCL stale after motion'))

    def _wait_for_input_recovery(self, reason, timeout_s=8.0):
        """Wait after the previous action is terminal without sending commands."""
        if not self._input_gap_recoverable(reason):
            return False
        started_s = time.monotonic()
        self.get_logger().warning(f'paused for input recovery: {reason}')
        while not self.stop_requested and time.monotonic() - started_s < timeout_s:
            spin_node(self, timeout_sec=0.05)
            if self.stop_requested:
                return False
            failure = self._guard_failure(False)
            if failure and not self._input_gap_recoverable(failure):
                return False
            motion = getattr(self, 'latest_motion', None)
            if motion is None or motion[0] <= started_s:
                continue
            _, linear_mps, angular_radps = motion
            if (failure is None and all(math.isfinite(v) for v in motion)
                    and abs(linear_mps) <= 0.01 and abs(angular_radps) <= 0.01):
                self.get_logger().info('inputs recovered; resume current waypoint')
                return True
        return False

    def _run_with_input_recovery(self, attempt):
        """
        Keep completed waypoints across retries of the current one.

        At most one retry after a fresh-input recovery, and up to
        PATH_BLOCKED_RETRIES after confirmed clearance on a blocked path.
        """
        self._resume_waypoint_index = 0
        return self._retry_loop(attempt)

    def _retry_loop(self, attempt):
        input_retried = False
        blocked_waits = 0
        while True:
            self._retry_guard_reason = None
            self._retry_blocked_reason = None
            self._retry_blocked_code = None
            if attempt():
                return True
            if self.stop_requested:
                return False
            blocked = self._retry_blocked_reason
            if blocked is not None:
                if blocked_waits >= PATH_BLOCKED_RETRIES:
                    self._report('path_blocked_give_up', reason=blocked, waits=blocked_waits)
                    return False
                blocked_waits += 1
                code = self._retry_blocked_code
                # Obstacle recovery holds position; it never backs away or
                # advances to manufacture clearance before waiting.
                if not self._wait_for_path_clear(blocked, blocked_waits, code):
                    return False
                continue
            if input_retried or not self._wait_for_input_recovery(self._retry_guard_reason):
                return False
            input_retried = True

    def _wait_for_path_clear(self, reason, wait, code=None):
        """
        Hold still until clearance, cancellation, or an unrecoverable guard.

        Every PATH_BLOCKED_PROBE_S the hold checks whether it may end: without
        any path (PATH_BLOCKED_PLANNER_CODES) once a fresh plan exists, after a
        controller stop once the band ahead is empty PATH_BLOCKED_CLEAR_PROBES
        times in a row.
        """
        self._report('path_blocked_wait', reason=reason, wait=wait,
                     wait_s=None)
        started_s = time.monotonic()
        probed_s = started_s
        clear_probes = 0
        while not self.stop_requested:
            spin_node(self, timeout_sec=0.05)
            if self.stop_requested:
                return False
            failure = self._guard_failure(False)
            if failure and not self._input_gap_recoverable(failure):
                self._report('path_blocked_wait_aborted', reason=failure)
                return False
            if time.monotonic() - probed_s < PATH_BLOCKED_PROBE_S:
                continue
            probed_s = time.monotonic()
            if code in PATH_BLOCKED_PLANNER_CODES:
                cleared = self._path_open()
            else:
                clear_probes = clear_probes + 1 if self._front_clear() else 0
                cleared = clear_probes >= PATH_BLOCKED_CLEAR_PROBES
            if cleared:
                self._report('path_blocked_cleared', wait=wait,
                             held_s=round(probed_s - started_s, 1))
                return True
        return False

    def _path_open(self):
        """Whether a fresh plan to the remaining goal exists (see ServiceRoute)."""
        return False

    def _escape_blocked(self, wait=1):
        """Move clear of a blocking object before the hold (see ServiceRoute)."""
        return False

    def _front_clear(self):
        """Whether the band ahead of the chassis is empty (see ServiceRoute)."""
        return False

    def _report(self, event, **fields):
        """Record an event in the run log when the node keeps one, else in the ROS log."""
        emit = getattr(self, 'emit', None)
        if emit is not None:
            emit(event, **fields)
        else:
            self.get_logger().warning(f'{event} {json.dumps(fields, sort_keys=True)}')

    def _revisit_protection_ready(self):
        """Check only the three publishers that make a revisit safe."""
        expected = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info':
                'keepout_costmap_filter_info_server',
        }
        for topic, node_name in expected.items():
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) != 1 or publishers[0].node_name != node_name:
                return f'{topic} publisher must be {node_name} only'
        return None

    def wait_until_ready(self, timeout=15.0):
        """Require current sensors, battery, and a bounded AMCL estimate."""
        self.last_readiness_failure = None
        deadline = time.monotonic() + timeout
        protection_failure = None
        missing_amcl_refresh_attempted = False
        missing_amcl_refresh_required = False
        while time.monotonic() < deadline and not self.stop_requested:
            spin_node(self, timeout_sec=0.1)
            protection_failure = (
                self._revisit_protection_ready()
                if self.navigation_profile == 'new_base_revisit_candidate'
                else None)
            failure = self._guard_failure()
            if failure == 'AMCL pose missing':
                missing_amcl_refresh_required = True
            if (missing_amcl_refresh_required
                    and not missing_amcl_refresh_attempted
                    and failure in (None, 'AMCL pose missing')):
                attempted, confirmed = self._refresh_missing_amcl_at_rest(
                    deadline)
                missing_amcl_refresh_attempted = attempted
                if confirmed:
                    missing_amcl_refresh_required = False
            if (not missing_amcl_refresh_required
                    and self._navigation_ready()
                    and protection_failure is None):
                self.start_check_pending = False
                if self.travel_pose_status is not None:
                    maximum_error_rad = self.travel_pose_status[
                        'maximum_error_rad']
                    self.get_logger().info(
                        'travel pose gate passed: '
                        f'max_error={maximum_error_rad:.6f}rad')
                return True
        self.last_readiness_failure = (
            'operator stop' if self.stop_requested else
            self._guard_failure() or protection_failure or 'navigation readiness timeout')
        self.get_logger().error(
            'navigation readiness timeout: ' + self.last_readiness_failure)
        return False

    def _refresh_missing_amcl_at_rest(self, readiness_deadline_s):
        """Return whether a refresh was attempted and confirmed while at rest."""
        def stationary_odometry_is_fresh():
            motion = getattr(self, 'latest_motion', None)
            now_s = time.monotonic()
            return (motion is not None and now_s - motion[0] <= 0.5
                    and all(math.isfinite(value) for value in motion)
                    and abs(motion[1]) < 0.01 and abs(motion[2]) < 0.01)

        if self.stop_requested:
            return True, False
        if not stationary_odometry_is_fresh():
            return False, False
        operator_link_ready = getattr(self, '_operator_link_ready', None)
        if operator_link_ready is not None and not operator_link_ready():
            return True, False
        client = None
        future = None
        try:
            client = self.create_client(Empty, '/request_nomotion_update')
            remaining_s = readiness_deadline_s - time.monotonic()
            if remaining_s <= 0.0 or not client.wait_for_service(
                    timeout_sec=min(0.2, remaining_s)):
                return True, False
            requested_s = time.monotonic()
            future = client.call_async(Empty.Request())
            while (not future.done() and not self.stop_requested
                   and time.monotonic() < readiness_deadline_s):
                if not stationary_odometry_is_fresh():
                    return False, False
                spin_node(self, timeout_sec=0.05)
            if (not future.done() or future.exception() is not None
                    or future.result() is None):
                return True, False
            while (not self.stop_requested
                   and time.monotonic() < readiness_deadline_s):
                if not stationary_odometry_is_fresh():
                    return False, False
                if (getattr(self, 'amcl_seen', None) or 0.0) > requested_s:
                    self.get_logger().info(
                        'AMCL pose recovered by stationary no-motion update')
                    return True, True
                spin_node(self, timeout_sec=0.05)
        except (AttributeError, RuntimeError):
            return True, False
        finally:
            if (client is not None and future is not None
                    and not future.done()):
                client.remove_pending_request(future)
            if client is not None:
                self.destroy_client(client)
        return True, False

    def readiness_failure_reason(self):
        """Retain the failed readiness attempt's cause for the caller's result log."""
        return (getattr(self, 'last_readiness_failure', None)
                or 'navigation data unavailable')

    def _pose(self, index, waypoint):
        if 'yaw' in waypoint:
            yaw = waypoint['yaw']
        elif index + 1 < len(self.waypoints):
            following = self.waypoints[index + 1]
            yaw = math.atan2(
                following['y'] - waypoint['y'],
                following['x'] - waypoint['x'])
        else:
            yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = str(self.config.get('frame_id', 'map'))
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def preflight(self):
        """Require Nav2 to plan the full remaining route before movement."""
        self.last_preflight_failure = None
        if (getattr(self, 'parking_contract', None) is not None and
                not self._parking_parameters_ready()):
            self.last_preflight_failure = self.last_parking_parameters_failure
            return False
        if not self.compute.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('compute_path_through_poses unavailable')
            return False
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [
            self._pose(index, waypoint)
            for index, waypoint in enumerate(self.waypoints)
        ]
        goal.planner_id = 'GridBased'
        goal.use_start = False
        future = self.compute.send_goal_async(goal)
        while not future.done() and not self.stop_requested:
            spin_node(self, timeout_sec=0.1)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error('route preflight goal rejected')
            return False
        result_future = handle.get_result_async()
        while not result_future.done() and not self.stop_requested:
            spin_node(self, timeout_sec=0.1)
        if not result_future.done():
            self._cancel(handle, 'operator interrupt during preflight')
            return False
        result = result_future.result().result
        if result.error_code != 0 or not result.path.poses:
            self.get_logger().error(
                f'route preflight failed: {result.error_code} '
                f'{result.error_msg}')
            return False
        length = sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y)
            for previous, current in zip(
                result.path.poses, result.path.poses[1:]))
        if (self.navigation_profile == 'new_base_revisit_candidate'
                and not revisit_plan_length_ok(
                    length, float(self.config['planned_length_m']))):
            self.get_logger().error(
                'new-base revisit planned detour exceeds corridor bound: '
                f'length={length:.3f}m '
                f'reference={self.config["planned_length_m"]}m')
            return False
        self.get_logger().info(
            f'route preflight passed: poses={len(result.path.poses)} '
            f'length={length:.3f}m')
        end = result.path.poses[-1].pose.position
        self.preflight_path_end_xy = (end.x, end.y)
        return True

    def _read_parameters(self, remote_node, names):
        """
        Reuse one read-only endpoint; never cache the returned parameter values.

        A new client's first response can be lost before the server matches its
        reply reader ("client will not receive response", 20260929_RUNTIME_ROOT_CAUSE).
        Every executor attempt starts with a new node, and this first map_server
        read ended the 2026-10-06 10:26 and 17:09 cycles before any motion.
        Unanswered reads are retried at one-second intervals within the same
        five-second budget, so a reply reader matched after two seconds can
        still receive a fresh request.
        """
        client = self._parameter_readers.get(remote_node)
        if client is None:
            client = self.create_client(GetParameters, f'{remote_node}/get_parameters')
            self._parameter_readers[remote_node] = client
        if getattr(self, 'stop_requested', False):
            raise RuntimeError(
                f'{remote_node} parameter read interrupted before service wait')
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        requested_names = list(names)
        total_budget_s = 5.0
        resend_interval_s = 1.0
        started_s = time.monotonic()
        deadline_s = started_s + total_budget_s
        next_send_s = started_s
        attempt = 0
        pending = []
        try:
            while True:
                if getattr(self, 'stop_requested', False):
                    raise RuntimeError(
                        f'{remote_node} parameter read interrupted after attempts={attempt}')
                for attempt_index, future in enumerate(pending, start=1):
                    if not future.done():
                        continue
                    error = future.exception()
                    if error is not None:
                        raise RuntimeError(
                            f'{remote_node} parameter read failed on '
                            f'attempt={attempt_index}: {error!r}') from error
                    return future.result()
                now_s = time.monotonic()
                if now_s >= deadline_s:
                    raise RuntimeError(
                        f'{remote_node} parameter read timed out: '
                        f'total_budget={total_budget_s:.1f} s attempts={attempt} '
                        f'resend_interval={resend_interval_s:.1f} s')
                if now_s >= next_send_s:
                    attempt += 1
                    request = GetParameters.Request()
                    request.names = requested_names
                    pending.append(client.call_async(request))
                    next_send_s = now_s + resend_interval_s
                    if attempt > 1:
                        self.get_logger().warning(
                            f'parameter read retry: node={remote_node} attempt={attempt}; '
                            f'total_budget={total_budget_s:.1f} s '
                            f'resend_interval={resend_interval_s:.1f} s')
                    continue
                spin_node(
                    self, timeout_sec=min(0.05, deadline_s - now_s, next_send_s - now_s))
        finally:
            for future in pending:
                if not future.done():
                    client.remove_pending_request(future)

    def _parking_parameters_ready(self, reverse=False):
        """Reject missing or mismatched opt-in plugins before any route goal."""
        contract = self.parking_contract
        expected = {
            'controller_plugins': ['Parking'],
            'goal_checker_plugins': ['parking_goal_checker'],
            'parking_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
            'parking_goal_checker.stateful': False,
            'parking_goal_checker.xy_goal_tolerance': contract['xy_tolerance_m'],
            'parking_goal_checker.yaw_goal_tolerance': (
                contract['yaw_tolerance_rad']),
            'Parking.plugin': ('nav2_regulated_pure_pursuit_controller::'
                               'RegulatedPurePursuitController'),
            'Parking.desired_linear_vel': contract['desired_linear_mps'],
            'Parking.min_approach_linear_velocity': (
                contract['min_approach_linear_mps']),
            'Parking.regulated_linear_scaling_min_speed': (
                contract['min_approach_linear_mps']),
            'Parking.rotate_to_heading_angular_vel': (
                contract['rotate_angular_radps']),
            'Parking.stateful': False,
            'Parking.use_rotate_to_heading': True,
            'Parking.allow_reversing': False,
        }
        if reverse:
            expected['controller_plugins'] = ['Parking', 'ParkingReverse']
            expected.update({
                key.replace('Parking.', 'ParkingReverse.'): value
                for key, value in list(expected.items()) if key.startswith('Parking.')
            })
            expected['ParkingReverse.use_rotate_to_heading'] = False
            expected['ParkingReverse.allow_reversing'] = True
        self.last_parking_parameters_failure = None
        try:
            response = self._read_parameters('controller_server', list(expected))
        except RuntimeError as error:
            return self._parking_parameters_failed(str(error))
        if response is None:
            return self._parking_parameters_failed(
                'controller_server parameter service unavailable')
        values = response.values
        if len(values) != len(expected):
            return self._parking_parameters_failed(
                'controller_server parameter response incomplete: '
                f'expected={len(expected)} received={len(values)} '
                f'requested={",".join(expected)}')
        for (name, wanted), value in zip(expected.items(), values):
            # GetParameters returns ParameterValue rather than Parameter.
            actual = parameter_value_to_python(value)
            if isinstance(wanted, list):
                matches = (isinstance(actual, list) and
                           all(item in actual for item in wanted))
            else:
                matches = (type(actual) is type(wanted) and (
                    math.isclose(actual, wanted, rel_tol=0.0, abs_tol=1e-9)
                    if isinstance(wanted, float) else actual == wanted))
            if not matches:
                return self._parking_parameters_failed(
                    f'controller_server parking parameter mismatch: {name}')
        return True

    def _parking_parameters_failed(self, reason):
        """Keep the failed controller read in both the journal and the result event."""
        self.last_parking_parameters_failure = reason
        self.get_logger().error(reason)
        return False

    def _parking_observation(self, reference_frame=None):
        """Read current map (or odom) pose and independent odometry without commanding."""
        contract = self.parking_contract
        if self.parking_odom is None or self.parking_command is None:
            raise ValueError('parking odometry or final command missing')
        transform = self.parking_tf.lookup_transform(
            reference_frame or contract['reference_frame'], contract['robot_base_frame'],
            rclpy.time.Time())
        ros_now_s = self.get_clock().now().nanoseconds * 1e-9
        now_s = time.monotonic()
        odom_seen_s, odom_stamp, linear_mps, angular_radps = self.parking_odom
        _command_seen_s, cmd_linear_mps, cmd_angular_radps = (
            self.parking_command)
        # Odometry and TF must remain fresh. A final zero command need not be
        # periodically republished; any later observed nonzero command or
        # physical motion increments parking_motion_revision and resets hold.
        ages_s = [now_s - odom_seen_s]
        source_ages_s = {'odom_receive_age_s': ages_s[0]}
        for stamp in (odom_stamp, transform.header.stamp):
            age_s = ros_now_s - (stamp.sec + stamp.nanosec * 1e-9)
            if age_s < 0.0:
                raise ValueError('parking observation is ahead of ROS time')
            ages_s.append(age_s)
        source_ages_s.update({
            'odom_header_age_s': ages_s[1],
            'tf_header_age_s': ages_s[2],
        })
        self.parking_observation_diagnostics = source_ages_s
        translation = transform.transform.translation
        return {
            'actual_pose': (translation.x, translation.y,
                            _quaternion_yaw(transform.transform.rotation)),
            'linear_mps': linear_mps, 'angular_radps': angular_radps,
            'cmd_linear_mps': cmd_linear_mps,
            'cmd_angular_radps': cmd_angular_radps,
            'sample_age_s': max(ages_s),
        }

    def _verify_parking_stop(self, index, waypoint, handle, hold_s=None,
                             contract=None):
        """Confirm a bounded stationary pose window after Nav2 succeeds."""
        contract = dict(contract or self.parking_contract)
        if hold_s is not None:
            if (isinstance(hold_s, bool) or not isinstance(hold_s, (int, float))
                    or not math.isfinite(hold_s) or hold_s <= 0.0):
                raise ValueError('parking dwell must be finite and positive')
            contract['hold_s'] = float(hold_s)
            contract['observation_timeout_s'] += float(hold_s)
        gate = ParkingHold(contract)
        deadline_s = time.monotonic() + contract['observation_timeout_s']
        last_stamp = None
        motion_revision = self.parking_motion_revision
        input_recovery_reason = None
        result = {'confirmed': False, 'reason': 'NO_FRESH_OBSERVATION',
                  'physical_accuracy': 'NOT_MEASURED'}
        while time.monotonic() < deadline_s and not self.stop_requested:
            spin_node(self, timeout_sec=0.05)
            # The route node also observes scan, battery and AMCL (TF has its own thread). Drain
            # their ready callbacks so high-rate odometry cannot leave the
            # post-goal witness evaluating old queue entries.
            for _ in range(8):
                spin_node(self, timeout_sec=0.0)
            if self.parking_motion_revision != motion_revision:
                gate = ParkingHold(contract)
                motion_revision = self.parking_motion_revision
            if not self._navigation_ready(require_fresh_amcl=False):
                reason = self._guard_failure(False)
                # One recoverable input gap restarts the whole stationary
                # window without a command; nothing observed before it counts.
                if (input_recovery_reason is None and not self.stop_requested
                        and self._input_gap_recoverable(reason)
                        and self._wait_for_input_recovery(reason)):
                    input_recovery_reason = reason
                    gate = ParkingHold(contract)
                    deadline_s = time.monotonic() + contract['observation_timeout_s']
                    last_stamp = None
                    motion_revision = self.parking_motion_revision
                    continue
                result['reason'] = reason
                break
            if self.parking_odom is None:
                continue
            stamp = self.parking_odom[1]
            stamp_key = (stamp.sec, stamp.nanosec)
            if stamp_key == last_stamp:
                continue
            if last_stamp is not None and stamp_key < last_stamp:
                gate = ParkingHold(contract)
                result['reason'] = 'ODOMETRY_TIME_REGRESSION'
                last_stamp = stamp_key
                continue
            last_stamp = stamp_key
            try:
                frame = contract.get('reference_frame')
                observation = (
                    self._parking_observation()
                    if frame == self.parking_contract.get('reference_frame')
                    else self._parking_observation(frame))
                result = gate.observe(
                    now_s=time.monotonic(),
                    target_pose=(waypoint['x'], waypoint['y'], waypoint['yaw']),
                    **observation)
            except (TransformException, ValueError) as error:
                gate = ParkingHold(contract)
                result = {'confirmed': False, 'reason': str(error),
                          'physical_accuracy': 'NOT_MEASURED'}
            if result['confirmed']:
                self._route_event(
                    'parking_estimate_confirmed', index, handle,
                    observation_ages_s=self.parking_observation_diagnostics,
                    input_recovery_reason=input_recovery_reason, **result)
                return True
        self._route_event(
            'parking_not_confirmed', index, handle,
            observation_ages_s=self.parking_observation_diagnostics,
            input_recovery_reason=input_recovery_reason, **result)
        return False

    def _feedback(self, route_index, message):
        now = time.monotonic()
        if now - self._last_feedback < 5.0:
            return
        self._last_feedback = now
        feedback = message.feedback
        self.get_logger().info(
            f'waypoint={route_index + 1}/{len(self.waypoints)} '
            f'remaining={feedback.distance_remaining:.2f}m '
            f'recoveries={feedback.number_of_recoveries} '
            f'battery={self.battery_voltage:.2f}V')

    def _cancel(self, handle, reason, route_index=None):
        self.get_logger().warning(f'cancel requested: {reason}')
        if route_index is not None:
            self._route_event(
                'cancel_requested', route_index, handle, reason=reason)
        future = handle.cancel_goal_async()
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            spin_node(self, timeout_sec=0.1)
        return future.done()

    def execute(self):
        """Resume the current waypoint once after a confirmed input-gap stop."""
        return self._run_with_input_recovery(self._execute_route_once)

    def _execute_route_once(self):
        """Run the remaining waypoints without restarting completed segments."""
        if not self.navigate.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose unavailable')
            return False
        # wait_until_ready() already proved that localization produced a
        # current pose.  AMCL's pose topic may remain quiet while stationary.
        # New-base movement without an AMCL update eventually cancels the goal.
        for index, waypoint in enumerate(self.waypoints):
            if index < self._resume_waypoint_index:
                continue
            self._resume_waypoint_index = index
            if (
                    self.stop_requested or
                    not self._navigation_ready(require_fresh_amcl=False)):
                self.get_logger().error(
                    'navigation guard blocked next goal: '
                    f'{self._guard_failure(False) or "operator stop"}')
                reason = self._guard_failure(False)
                if not self.stop_requested and self._input_gap_recoverable(reason):
                    self._retry_guard_reason = reason
                return False
            goal = NavigateToPose.Goal()
            if getattr(self, 'navigation_profile', None) == (
                    'new_base_revisit_candidate'):
                goal_start_amcl = self.amcl_position
                goal_start_odom_m = self.odom_total_distance_m
            goal.pose = self._pose(index, waypoint)
            is_parking = (
                getattr(self, 'parking_contract', None) is not None and
                index == len(self.waypoints) - 1)
            goal.behavior_tree = self.behavior_tree
            if is_parking:
                goal.behavior_tree = self.parking_behavior_tree
            self.get_logger().info(
                f'send {index + 1}/{len(self.waypoints)} '
                f'{waypoint["id"]}=({waypoint["x"]:.2f},'
                f'{waypoint["y"]:.2f})')
            future = self.navigate.send_goal_async(
                goal,
                feedback_callback=lambda message, route_index=index:
                self._feedback(route_index, message),
            )
            while not future.done() and not self.stop_requested:
                spin_node(self, timeout_sec=0.1)
            handle = future.result() if future.done() else None
            if handle is None or not handle.accepted:
                self.get_logger().error('navigate goal rejected')
                return False
            self._route_event('accepted', index, handle)
            result_future = handle.get_result_async()
            while not result_future.done():
                spin_node(self, timeout_sec=0.1)
                if self.stop_requested:
                    self._cancel(handle, 'operator interrupt', index)
                    return False
                if not self._navigation_ready(require_fresh_amcl=False):
                    reason = self._guard_failure(False) or 'navigation guard failure'
                    canceled = self._cancel(handle, reason, index)
                    if canceled and self._input_gap_recoverable(reason):
                        deadline_s = time.monotonic() + 5.0
                        while not result_future.done() and time.monotonic() < deadline_s:
                            spin_node(self, timeout_sec=0.05)
                        if (result_future.done() and result_future.exception() is None
                                and result_future.result().status in (
                                    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                                    GoalStatus.STATUS_SUCCEEDED)):
                            self._retry_guard_reason = reason
                    return False
            wrapped = result_future.result()
            self._route_event(
                'result', index, handle,
                terminal_status=GOAL_STATUS_NAMES.get(
                    wrapped.status, 'STATUS_UNKNOWN'),
                terminal_status_code=int(wrapped.status),
                nav2_error_code=int(wrapped.result.error_code))
            if (
                    wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                    wrapped.result.error_code != 0):
                self.get_logger().error(
                    f'navigation failed: status={wrapped.status} '
                    f'error={wrapped.result.error_code} '
                    f'{wrapped.result.error_msg}')
                return False
            if getattr(self, 'navigation_profile', None) == (
                    'new_base_revisit_candidate'):
                motion_m = self.odom_total_distance_m - goal_start_odom_m
                ros_now_ns = self.get_clock().now().nanoseconds
                try:
                    map_to_odom = self.revisit_tf.lookup_transform(
                        'map', 'odom', rclpy.time.Time())
                    odom_to_base = self.revisit_tf.lookup_transform(
                        'odom', 'base_footprint', rclpy.time.Time())
                    final_xy = current_map_xy(map_to_odom, odom_to_base)
                    odom_stamp_ns = (
                        odom_to_base.header.stamp.sec * 1_000_000_000 +
                        odom_to_base.header.stamp.nanosec)
                    map_stamp_ns = (
                        map_to_odom.header.stamp.sec * 1_000_000_000 +
                        map_to_odom.header.stamp.nanosec)
                    odom_tf_age_s = (ros_now_ns - odom_stamp_ns) / 1e9
                    map_tf_age_s = (ros_now_ns - map_stamp_ns) / 1e9
                    correction_stamp_ns = map_stamp_ns - 1_000_000_000
                    odom_since_correction_m = odom_distance_since_stamp(
                        self.odom_history, self.odom_total_distance_m,
                        correction_stamp_ns)
                except (TransformException, ValueError) as error:
                    self.get_logger().error(
                        f'Nav2 success without current map pose: {error}')
                    return False
                if (not 0.0 <= odom_tf_age_s <= 0.5 or
                        not revisit_map_correction_ok(
                            map_tf_age_s, odom_since_correction_m) or
                        not finite_localization_xy(self.amcl_position) or
                        not revisit_goal_witness(
                            goal_start_amcl,
                            (waypoint['x'], waypoint['y']),
                            final_xy, motion_m,
                            self.revisit_goal_amcl_tolerance_m,
                            self.revisit_motion_min_distance_m)):
                    self.get_logger().error(
                        f'Nav2 success lacks physical goal witness: '
                        f'waypoint={waypoint["id"]} odom={motion_m:.3f}m '
                        f'odom_tf_age={odom_tf_age_s:.3f}s '
                        f'map_tf_age={map_tf_age_s:.3f}s '
                        f'odom_since_correction='
                        f'{odom_since_correction_m:.3f}m '
                        f'current_map_xy={final_xy}')
                    return False
            if is_parking and not self._verify_parking_stop(
                    index, waypoint, handle):
                return False
        self.get_logger().info('corridor roundtrip succeeded')
        return True


def parse_args(argv):
    """Parse non-ROS route arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', required=True, type=Path)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--park-final', action='store_true',
                        help='align and verify the final explicit-yaw waypoint')
    parser.add_argument('--parking-contract', type=Path)
    parser.add_argument(
        '--navigation-profile',
        choices=tuple(NAVIGATION_BEHAVIOR_TREES),
        default='corridor')
    parser.add_argument(
        '--execute', action='store_true',
        help='move after preflight; omitted means planning-only')
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(args=None):
    """Verify route artifacts, preflight Nav2, and optionally drive."""
    argv = args if args is not None else sys.argv
    parsed = parse_args(argv)
    if parsed.start_index < 0:
        raise SystemExit('--start-index must be non-negative')
    try:
        config = load_route(parsed.route)
        parking_contract = None
        if parsed.parking_contract and not parsed.park_final:
            raise ValueError('--parking-contract requires --park-final')
        if parsed.park_final:
            contract_path = parsed.parking_contract or (
                Path(get_package_share_directory('jdamr_cube_navigation')) /
                'config' / 'parking_contract.yaml')
            parking_contract = load_parking_contract(contract_path)
            validate_parking_route(config, parking_contract)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f'route configuration error: {error}', file=sys.stderr)
        raise SystemExit(2) from error

    rclpy.init(
        args=argv,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = CorridorRoute(
        config, parsed.start_index, parsed.navigation_profile, parking_contract)

    def request_stop(_signal_number, _frame):
        node.request_stop()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        ready = node.wait_until_ready()
        preflighted = ready and node.preflight()
        succeeded = preflighted and (
            not parsed.execute or node.execute())
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if succeeded else 1)


if __name__ == '__main__':
    main()
