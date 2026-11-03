"""Exercise service routing with ROS messages and in-memory action peers."""

import importlib.util
import inspect
import io
import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
import builtin_interfaces.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
from jdamr_cube_navigation import corridor_route, restaurant_service
from jdamr_cube_navigation.corridor_route import CorridorRoute
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.restaurant_service import (
    BLOCKED_PLAN_CODES, BoxDwell, load_service_contract, main, parse_args,
    select_destination, ServiceRoute,
)
from jdamr_cube_navigation.service_destinations import route_config, taught_pose
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as RosPath
import pytest
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('base_delay_s', [.042, .121])
def test_frozen_map_to_odom_does_not_require_overlapping_base_history(base_delay_s):
    node = object.__new__(ServiceRoute)
    node.parking_contract = {'robot_base_frame': 'base_link'}
    node.parking_tf = Buffer()
    frame = TransformStamped()
    frame.header.frame_id, frame.child_frame_id = 'map', 'odom'
    frame.header.stamp = Time(seconds=100).to_msg()
    frame.transform.translation.x, frame.transform.translation.y = 1.0, 2.0
    frame.transform.rotation.z = math.sin(math.pi / 4)
    frame.transform.rotation.w = math.cos(math.pi / 4)
    node.parking_tf.set_transform(frame, 'synthetic_reproduction')
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'odom', 'base_link'
    base.header.stamp = Time(seconds=100 + base_delay_s).to_msg()
    base.transform.rotation.w = 1.0
    node.parking_tf.set_transform(base, 'synthetic_reproduction')
    with pytest.raises(TransformException):
        node.parking_tf.lookup_transform('map', 'base_link', Time())
    path = RosPath()
    path.header.frame_id, path.header.stamp = 'map', frame.header.stamp
    item = PoseStamped()
    item.header = path.header
    item.pose.position.x, item.pose.position.y = 2.0, 4.0
    item.pose.orientation.z = math.sin((math.pi - .1) / 2)
    item.pose.orientation.w = math.cos((math.pi - .1) / 2)
    path.poses = [item]
    frozen, to_odom = node._frozen_in_odom(path)
    assert frozen.header.frame_id == 'odom'
    assert frozen.header.stamp == path.header.stamp
    assert to_odom(2., 4., math.pi - .1) == pytest.approx((2., -1., math.pi / 2 - .1))
    assert frozen.poses[0].pose.position.x == pytest.approx(2.)
    assert frozen.poses[0].pose.position.y == pytest.approx(-1.)
    assert -math.pi <= to_odom(0., 0., -math.pi)[2] <= math.pi
    empty, same_frame = node._frozen_in_odom(RosPath())
    assert empty.poses == []
    assert same_frame(2., 4., 0.) == pytest.approx((2., -1., -math.pi / 2))


def test_frozen_path_still_rejects_missing_map_to_odom():
    node = object.__new__(ServiceRoute)
    node.parking_contract = {'robot_base_frame': 'base_link'}
    node.parking_tf = Buffer()
    with pytest.raises(RuntimeError, match='map/odom transform unavailable'):
        node._frozen_in_odom(RosPath())


@pytest.mark.parametrize('requested_deg', [-159., 159.])
def test_large_staging_turn_cruises_then_brakes_without_reversing(requested_deg, monkeypatch):
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    yaw, elapsed, rates = [0.0], [0.0], []
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: SimpleNamespace(
        transform=SimpleNamespace(rotation=SimpleNamespace(
            x=0., y=0., z=math.sin(yaw[0] / 2), w=math.cos(yaw[0] / 2)))))

    def direct(command, done, measure, deadline, event, watch=None):
        while not done(measure()):
            rates.append(command.angular.z)
            yaw[0] += command.angular.z * .02
            elapsed[0] += .02
            assert elapsed[0] < 60.
        yaw[0] += command.angular.z * .19
        return measure()

    node._direct_motion = direct
    requested = math.radians(requested_deg)
    turned = node._rotate_in_place(requested)
    assert max(abs(rate) for rate in rates) == pytest.approx(.3)
    assert abs(rates[-1]) == pytest.approx(.08)
    assert all(rate * requested > 0. for rate in rates)
    assert abs(turned - requested) < math.radians(.3)
    assert elapsed[0] < abs(requested) / .3 + 2.


@pytest.mark.parametrize('override,expected', [(None, 'LOCALHOST'), ('SUBNET', 'SUBNET')])
def test_core_launch_resolves_discovery_default_before_environment(override, expected):
    """Execute startup declarations in order, without starting ROS processes."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    spec = importlib.util.spec_from_file_location(
        'core_launch_defaults', PACKAGE / 'launch/onboard_nav2_core.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['discovery_range'] = override
    for action in module.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
            action.execute(context)
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == expected


def done(value):
    """Return a real completed rclpy future."""
    future = Future()
    future.set_result(value)
    return future


def publisher(name, namespace='/'):
    """Describe one DDS publisher without creating a ROS graph."""
    return SimpleNamespace(node_name=name, node_namespace=namespace)


@pytest.mark.parametrize('command_publisher', [None, 'web_teleop', 'collision_monitor'])
def test_prepare_identity_allows_absent_command_but_never_other_publishers(
        command_publisher):
    node = route()
    expected = {
        '/cmd_vel': ([publisher(command_publisher)] if command_publisher else []),
        '/keepout_filter_mask': [publisher('keepout_filter_mask_server')],
        '/keepout_costmap_filter_info': [publisher('keepout_costmap_filter_info_server')],
    }
    node.get_publishers_info_by_topic = lambda topic: expected[topic]
    failure = node._startup_protection_ready(
        discovery_timeout_s=0.0, require_command_path=False)
    assert (failure is None) == (command_publisher != 'web_teleop')
    strict = node._startup_protection_ready(discovery_timeout_s=0.0)
    assert (strict is None) == (command_publisher == 'collision_monitor')


def route():
    """Create the service adapter without touching DDS or robot hardware."""
    node = object.__new__(ServiceRoute)
    node.registry = {'frame_id': 'map'}
    node.parking_contract = load_parking_contract(PACKAGE / 'config/parking_contract.yaml')
    node.get_logger = lambda: Mock()
    node.table_id, node.pose_id = 'table_01', 'main'
    node.result_stream = io.StringIO()
    node.stop_requested = False
    node.active_handle = None
    node.pending_goal = None
    node.navigation_result = None
    node.navigation_uuid = None
    node.amcl_yaw_covariance_rad2 = 0.01
    node.amcl_covariance = (0.001, 0.001)
    node.service_contract = load_service_contract(
        Path(__file__).parent / 'fixtures/restaurant_service_contract_gated.yaml')
    node.battery_voltage = 12.0
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.live_grids = {}
    node.expected_grids = {}
    node.map_mismatch = None
    node._parameter_readers = {}
    node.confirmation = None
    node.start_index = 0
    node.navigation_profile = 'obstacle_base_candidate'
    node.through_behavior_tree, node.staging_behavior_tree = 'transit.xml', 'staging.xml'
    node.behavior_tree, node.parking_behavior_tree = 'transit.xml', 'parking.xml'
    node.config = {'waypoints': [
        {'id': 'approach', 'x': 0.5, 'y': 0.0, 'yaw': 0.2},
        {'id': 'main', 'x': 1.0, 'y': 0.0, 'yaw': 0.2}]}
    node.waypoints = node.config['waypoints']
    node._pose = lambda *_: PoseStamped()
    node._navigation_ready = lambda **_: True
    node._guard_failure = lambda *_: None
    return node


def dock_departure_route():
    node = route()
    node.config = {'frame_id': 'map', 'dock_departure_distance_m': 0.7,
                   'waypoints': [{'id': 'home_exit', 'x': 0.7, 'y': 0.0},
                                 {'id': 'pre', 'x': 0.9, 'y': -1.2},
                                 {'id': 'observation', 'x': 0.9, 'y': -1.8, 'yaw': 0.0}]}
    node.waypoints = node.config['waypoints']
    node._resume_waypoint_index = 0
    node.navigate = Mock()
    node.navigate_through = Mock()
    node.capture_stationary_pose = Mock(return_value=((0.0, 0.0, 0.4), {}))
    node._odom_pose = Mock(return_value=(0.7, 0.0, 0.4))
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time())
    node._pose = lambda index, waypoint: CorridorRoute._pose(node, index, waypoint)
    node._reverse_path_valid = Mock(return_value=True)
    node.registry.update({'map': {'yaml_path': 'map.yaml'},
                          'keepout': {'yaml_path': 'keepout.yaml'}})
    node._reverse_footprint = Mock(return_value=[(-0.28, -0.275), (0.07, -0.275),
                                               (0.07, 0.275), (-0.28, 0.275)])
    node._frozen_in_odom = Mock(side_effect=lambda path: (path, None))
    node._execute_reverse_once = Mock(return_value=True)
    node._execute_through_once = Mock(return_value=True)
    return node


@pytest.fixture
def dock_departure_static_corridor(monkeypatch):
    checker = Mock(return_value=True)
    monkeypatch.setattr(restaurant_service, 'static_corridor_clear', checker)
    monkeypatch.setattr(restaurant_service, 'spin_node', lambda *_a, **_k: None)
    return checker


def test_dock_departure_finishes_straight_before_transit_can_turn(dock_departure_static_corridor):
    node = dock_departure_route()
    calls = []
    node._execute_reverse_once.side_effect = lambda *a, **k: calls.append('straight') or True
    node._execute_through_once.side_effect = lambda: calls.append('transit') or True
    assert node.execute(final_parking=False)
    assert calls == ['straight', 'transit']
    assert node._resume_waypoint_index == 1
    path = node._frozen_in_odom.call_args.args[0]
    start, end = path.poses[0].pose, path.poses[-1].pose
    assert end.position.x - start.position.x == pytest.approx(0.7 * math.cos(0.4))
    assert end.position.y - start.position.y == pytest.approx(0.7 * math.sin(0.4))
    assert all(p.pose.orientation.z == pytest.approx(math.sin(0.2)) for p in path.poses)
    kwargs = node._execute_reverse_once.call_args.kwargs
    assert kwargs['controller_id'] == 'Parking'
    assert kwargs['final'] is False
    node._frozen_in_odom.assert_called_once_with(path)


def test_return_clearance_uses_guarded_motion_without_a_departure_waypoint(
        dock_departure_static_corridor):
    node = dock_departure_route()
    node.emit = Mock()
    node.waypoints = [{'id': 'home_dock_approach', 'x': 0.1, 'y': 0.0}]
    node.config = {**node.config, 'waypoints': node.waypoints}
    saved_config, saved_waypoints = node.config, node.waypoints
    assert node._clear_rear_dock_straight(.2)
    assert node.config is saved_config and node.waypoints is saved_waypoints
    dock_departure_static_corridor.assert_called_once()
    assert node._execute_reverse_once.call_args.kwargs['motion'] == 'rear_dock_clearance'
    names = [call.args[0] for call in node.emit.call_args_list]
    assert 'rear_dock_clearance_finished' in names
    assert 'dock_departure_started' not in names
    with pytest.raises(ValueError, match='first home_exit'):
        node._depart_dock_straight(.2)


@pytest.mark.parametrize('refresh', [True, False])
def test_return_clearance_refreshes_scan_without_accepting_stale_input(
        dock_departure_static_corridor, monkeypatch, refresh):
    node = dock_departure_route()
    node.emit = Mock()
    node.registry['home'] = {'rear_parking_reference': 'lidar_box'}
    node.run_deadline_s = 10.0
    node.waypoints = [{'id': 'home_dock_approach', 'x': 0.1, 'y': 0.0}]
    node.config = {**node.config, 'waypoints': node.waypoints}
    clock = [0.0]
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service, 'spin_node',
                        lambda *_a, **_k: clock.__setitem__(0, clock[0] + .1))
    snapshot = object()
    node._scan_in_base = Mock(side_effect=lambda: snapshot if refresh and clock[0] else None)
    node._footprint_intrusion = Mock(return_value=None)
    node._rear_swing_need = Mock(return_value=0.0)
    assert node._clear_rear_dock_straight(.2) is refresh
    assert clock[0] <= restaurant_service.ESCAPE_SCAN_MAX_AGE_S
    if refresh:
        node._footprint_intrusion.assert_called_once_with(found=snapshot)
        node._execute_reverse_once.assert_called_once()
    else:
        node._footprint_intrusion.assert_not_called()
        node._execute_reverse_once.assert_not_called()


@pytest.mark.parametrize('blocked,stopped,failed', [
    (True, False, False), (False, True, False), (False, False, True)])
def test_return_clearance_preserves_obstacle_stop_and_action_failure(
        dock_departure_static_corridor, blocked, stopped, failed):
    node = dock_departure_route()
    node.waypoints = [{'id': 'home_dock_approach', 'x': 0.1, 'y': 0.0}]
    node.config = {**node.config, 'waypoints': node.waypoints}
    original = node._dock_departure = {'saved_departure': True}
    node.stop_requested = stopped
    node._execute_reverse_once.return_value = not failed
    dock_departure_static_corridor.return_value = not blocked
    assert not node._clear_rear_dock_straight(.2)
    assert node._dock_departure is original
    if blocked or stopped:
        node._execute_reverse_once.assert_not_called()


@pytest.mark.parametrize('blocked,stopped,failed', [(True, False, False),
                                                  (False, True, False),
                                                  (False, False, True)])
def test_dock_departure_never_dispatches_transit_before_clearance(
        blocked, stopped, failed, dock_departure_static_corridor):
    node = dock_departure_route()
    node.stop_requested = stopped
    dock_departure_static_corridor.return_value = not blocked
    node._execute_reverse_once.return_value = not failed
    assert not node.execute(final_parking=False)
    node._execute_through_once.assert_not_called()
    if blocked or stopped:
        node._execute_reverse_once.assert_not_called()


def test_dock_departure_uses_live_monitor_instead_of_global_snapshot_rejection(
        dock_departure_static_corridor):
    node = dock_departure_route()
    node._reverse_path_valid.return_value = False
    assert node.execute(final_parking=False)
    node._reverse_path_valid.assert_not_called()
    node._execute_reverse_once.assert_called_once()


@pytest.mark.parametrize('case', ['release', 'no_rear_object', 'no_scan',
                                  'front_blocked', 'escape_failed'])
def test_rear_box_departure_reuses_guarded_escape_before_original_path(
        case, dock_departure_static_corridor):
    node = dock_departure_route()
    node.registry['home'] = {'rear_parking_reference': 'lidar_box'}
    found = object()
    node._scan_in_base = Mock(return_value=None if case == 'no_scan' else found)
    node._footprint_intrusion = Mock(return_value='front' if case == 'front_blocked' else None)
    node._rear_swing_need = Mock(return_value=0. if case == 'no_rear_object' else .18)
    node._escape_forward = Mock(return_value=case != 'escape_failed')
    assert node.execute(final_parking=False) is (case in ('release', 'no_rear_object'))
    if case == 'release':
        node._escape_forward.assert_called_once_with(found, pause_monitor=True)
        node._execute_reverse_once.assert_called_once()
        path = node._execute_reverse_once.call_args.args[0]
        start, end = path.poses[0].pose.position, path.poses[-1].pose.position
        assert math.hypot(end.x-start.x, end.y-start.y) == pytest.approx(.7)
    elif case == 'no_rear_object':
        node._escape_forward.assert_not_called()
        node._execute_reverse_once.assert_called_once()
    else:
        node._execute_reverse_once.assert_not_called()
        node._execute_through_once.assert_not_called()


def test_dock_entry_heading_uses_damped_stopped_trim():
    node = dock_departure_route()
    node._odom_pose = Mock(side_effect=[(0.25, 0.0, -0.1786), (0.25, 0.0, 0.005)])
    node._rotate_in_place = Mock(return_value=0.1836)
    assert node._turn_to_dock_heading({'yaw': 0.0}, math.radians(1.5),
                                      'entry_heading_checker')
    node._rotate_in_place.assert_called_once_with(pytest.approx(0.1786))
    node._execute_reverse_once.assert_not_called()


@pytest.mark.parametrize('failed', [False, True])
def test_dock_straight_uses_heading_hold_without_rpp_steering(failed):
    node = dock_departure_route()
    actual = [0.7, 0.0, 0.0]
    node._odom_pose = lambda: tuple(actual)
    node._make_reverse_path = lambda start, end, **_: node._odom_path([start, end])
    node._frozen_in_odom = lambda path: (path, lambda *pose: pose)
    node._execute_reverse_path = Mock(return_value=True)
    node._turn_to_dock_heading = Mock(return_value=True)
    node._trim_dock_heading = Mock(return_value=True)
    node._verify_parking_stop = Mock(return_value=True)
    node.confirmation = {'confirmed': True}

    def drive(distance_m, **kwargs):
        assert distance_m == pytest.approx(-0.7)
        assert kwargs['precision'] and kwargs['hold_heading']
        actual[0] = 0.0
        return None if failed else distance_m

    node._drive_zero_turn = Mock(side_effect=drive)
    assert node._dock_straight(tuple(actual), (0.0, 0.0, 0.0), None,
                               node.parking_contract) is (not failed)
    node._drive_zero_turn.assert_called_once()
    node._execute_reverse_path.assert_not_called()


def test_dock_departure_input_retry_keeps_frozen_goal_without_adding_distance(
        dock_departure_static_corridor):
    node = dock_departure_route()
    node._execute_reverse_once.side_effect = [False, True]
    node._wait_for_input_recovery = Mock(return_value=True)

    def first_failure(*args, **kwargs):
        if node._execute_reverse_once.call_count == 1:
            node._retry_guard_reason = 'scan stale: age=1s'
            return False
        return True

    node._execute_reverse_once.side_effect = first_failure
    assert node.execute(final_parking=False)
    assert node._execute_reverse_once.call_count == 2
    node._frozen_in_odom.assert_called_once()
    first, retry = node._execute_reverse_once.call_args_list
    assert retry.args[0] is first.args[0]
    node._execute_through_once.assert_called_once()


def test_transit_without_departure_metadata_keeps_existing_route():
    node = dock_departure_route()
    del node.config['dock_departure_distance_m']
    assert node.execute(final_parking=False)
    assert node._resume_waypoint_index == 0
    node._execute_reverse_once.assert_not_called()
    node._execute_through_once.assert_called_once()


def test_transit_input_recovery_does_not_repeat_completed_dock_departure(
        dock_departure_static_corridor):
    node = dock_departure_route()
    node._wait_for_input_recovery = Mock(return_value=True)

    def transit():
        if node._execute_through_once.call_count == 1:
            node._retry_guard_reason = 'scan stale: age=1s'
            return False
        return True

    node._execute_through_once.side_effect = transit
    assert node.execute(final_parking=False)
    assert node._execute_through_once.call_count == 2
    node._execute_reverse_once.assert_called_once()


@pytest.mark.parametrize('reason', [
    'odom stale: age=1s',
    'stationary teaching unavailable: need fresh TF, odometry and localization',
])
def test_departure_stop_input_gap_retries_confirmation_without_resending_motion(
        reason, dock_departure_static_corridor):
    node = dock_departure_route()
    node.capture_stationary_pose.side_effect = [
        ((0.0, 0.0, 0.4), {}), RuntimeError(reason), ((0.65, 0.27, 0.4), {})]
    node._wait_for_input_recovery = Mock(return_value=True)
    assert node.execute(final_parking=False)
    node._execute_reverse_once.assert_called_once()
    node._frozen_in_odom.assert_called_once()
    node._execute_through_once.assert_called_once()
    assert node.capture_stationary_pose.call_count == 3
    assert node._resume_waypoint_index == 1


@pytest.mark.parametrize('raise_on_stop', [False, True])
def test_operator_stop_during_departure_confirmation_never_starts_transit(
        raise_on_stop, dock_departure_static_corridor):
    node = dock_departure_route()

    def capture(**_kwargs):
        if node.capture_stationary_pose.call_count > 1:
            node.stop_requested = True
            if raise_on_stop:
                raise RuntimeError('stationary teaching unavailable: need fresh TF, odometry and localization')
        return (0.0, 0.0, 0.4), {}

    node.capture_stationary_pose.side_effect = capture
    assert not node.execute(final_parking=False)
    node._execute_reverse_once.assert_called_once()
    node._execute_through_once.assert_not_called()


def test_startup_protection_retries_only_empty_discovery(monkeypatch):
    node = route()
    calls = {'cmd_vel': 0}

    def publishers(topic):
        name = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info': 'keepout_costmap_filter_info_server',
        }[topic]
        if topic == '/cmd_vel':
            calls['cmd_vel'] += 1
            if calls['cmd_vel'] == 1:
                return []
        return [publisher(name)]

    node.get_publishers_info_by_topic = publishers
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node._startup_protection_ready() is None
    assert calls['cmd_vel'] == 2
    spin.assert_called_once_with(node, timeout_sec=0.05)


@pytest.mark.parametrize('actual', [
    [publisher('unsafe_commander')],
    [publisher('collision_monitor', '/other_robot')],
    [publisher('collision_monitor'), publisher('unsafe_commander')],
    [publisher('collision_monitor'), publisher('_NODE_NAME_UNKNOWN_')],
])
def test_startup_protection_immediately_rejects_known_bad_publishers(
        monkeypatch, actual):
    node = route()
    node.get_publishers_info_by_topic = lambda _topic: actual
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    failure = node._startup_protection_ready()
    assert 'actual=' in failure
    assert actual[0].node_name in failure
    spin.assert_not_called()


@pytest.mark.parametrize('unresolved', [
    publisher('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_'),
    publisher('collision_monitor', '_NODE_NAMESPACE_UNKNOWN_'),
])
def test_startup_protection_waits_for_unresolved_identity(monkeypatch, unresolved):
    node = route()
    calls = {'count': 0}

    def publishers(topic):
        expected = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info': 'keepout_costmap_filter_info_server',
        }[topic]
        if topic == '/cmd_vel':
            calls['count'] += 1
            if calls['count'] == 1:
                return [unresolved]
        return [publisher(expected)]

    node.get_publishers_info_by_topic = publishers
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node._startup_protection_ready() is None
    assert calls['count'] == 2
    spin.assert_called_once_with(node, timeout_sec=0.05)


def test_startup_protection_does_not_accept_persistent_unknown_identity(monkeypatch):
    node = route()
    node.get_publishers_info_by_topic = lambda _topic: [
        publisher('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_')]
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    failure = node._startup_protection_ready(discovery_timeout_s=0.0)
    assert '_NODE_NAME_UNKNOWN_' in failure
    spin.assert_not_called()


def test_startup_protection_preserves_unknown_identity_until_deadline(monkeypatch):
    node = route()
    unknown = publisher('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_')
    node.get_publishers_info_by_topic = lambda _topic: [unknown]
    clock = iter([0.0, 0.0, 30.0])
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.time.monotonic', lambda: next(clock))
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert '_NODE_NAME_UNKNOWN_' in node._startup_protection_ready()
    spin.assert_called_once_with(node, timeout_sec=0.05)


def test_startup_protection_does_not_accept_identity_after_stop(monkeypatch):
    node = route()
    node.stop_requested = True
    node.get_publishers_info_by_topic = lambda topic: [publisher({
        '/cmd_vel': 'collision_monitor',
        '/keepout_filter_mask': 'keepout_filter_mask_server',
        '/keepout_costmap_filter_info': 'keepout_costmap_filter_info_server',
    }[topic])]
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node._startup_protection_ready() is not None
    spin.assert_not_called()


def test_startup_protection_does_not_accept_stop_during_last_graph_read(monkeypatch):
    node = route()

    def publishers(topic):
        expected = {
            '/cmd_vel': 'collision_monitor',
            '/keepout_filter_mask': 'keepout_filter_mask_server',
            '/keepout_costmap_filter_info': 'keepout_costmap_filter_info_server',
        }[topic]
        if topic == '/keepout_costmap_filter_info':
            node.stop_requested = True
        return [publisher(expected)]

    node.get_publishers_info_by_topic = publishers
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node._startup_protection_ready() is not None
    spin.assert_not_called()


def test_startup_protection_reports_empty_publishers_after_bound(monkeypatch):
    node = route()
    node.get_publishers_info_by_topic = lambda _topic: []
    spin = Mock()
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    failure = node._startup_protection_ready(discovery_timeout_s=0.0)
    assert 'actual=[]' in failure
    spin.assert_not_called()


def handle(status=GoalStatus.STATUS_SUCCEEDED, error_code=0, future=None):
    """Build one accepted Nav2 response with an identifiable goal."""
    result = NavigateToPose.Result()
    result.error_code = error_code
    wrapped = SimpleNamespace(status=status, result=result)
    value = SimpleNamespace(
        accepted=True, goal_id=SimpleNamespace(uuid=bytes(range(16))),
        get_result_async=lambda: future if future is not None else done(wrapped),
        cancel_goal_async=Mock(return_value=done(SimpleNamespace(goals_canceling=[]))))
    return value


@pytest.mark.parametrize('failure,expected_calls', [
    (ComputePathThroughPoses.Result.GOAL_OCCUPIED, 2),
    (ComputePathThroughPoses.Result.NO_VALID_PATH, 2),
    (ComputePathThroughPoses.Result.TF_ERROR, 1),
    (ComputePathThroughPoses.Result.START_OCCUPIED, 1),
    (ComputePathThroughPoses.Result.TIMEOUT, 1),
    (None, 1),
])
def test_alternate_only_for_destination_path_obstruction(failure, expected_calls):
    """Sensor, TF or start failures must not dispatch alternate targets."""
    poses = [{'id': 'main'}, {'id': 'alternate'}, {'id': 'not_allowed'}]
    planner = Mock(side_effect=[{'ok': False, 'error_code': failure}, {'ok': True}])
    chosen, attempts = select_destination(poses, planner)
    assert len(attempts) == planner.call_count == expected_calls
    assert chosen == (poses[1] if expected_calls == 2 else None)


def test_two_blocked_poses_end_without_retry():
    """Bound retries to the registered primary and one alternate."""
    planner = Mock(return_value={'ok': False, 'error_code': next(iter(BLOCKED_PLAN_CODES))})
    chosen, attempts = select_destination([{'id': 'a'}, {'id': 'b'}], planner)
    assert chosen is None and len(attempts) == 2


def test_plan_only_is_default_and_teaching_requires_named_pose():
    """Parsing a goal does not silently authorize movement."""
    args = parse_args(['service', 'go', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_02', '--log', '/tmp/r.jsonl'])
    assert not args.execute
    assert args.table_id == 'table_02'


def test_teach_parses_gap_without_enabling_motion():
    """Manual gap provenance is a teaching option, not a driving command."""
    args = parse_args(['service', 'teach', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_02', '--pose-id', 'main', '--log', '/tmp/t.jsonl',
                       '--target-front-gap-m', '0.05', '--measured-front-gap-m', '0.06',
                       '--gap-measurement-note', 'ruler'])
    assert args.target_front_gap_m == 0.05
    assert args.measured_front_gap_m == 0.06
    assert not hasattr(args, 'execute')


def test_roundtrip_defaults_to_plan_only_and_twenty_second_box_dwell():
    """Planning is non-moving by default and retains the agreed station wait."""
    args = parse_args([
        'service', 'roundtrip', '--registry', '/tmp/r.yaml',
        '--table-id', 'table_02', '--log', '/tmp/r.jsonl'])
    assert not args.execute
    assert args.table_ids == ['table_02']
    assert args.dwell_s == 20.0
    assert args.box_timeout_s == 45.0


def test_teach_home_uses_named_dock_without_table_id():
    """Home teaching is explicit and cannot be confused with a table pose."""
    args = parse_args([
        'service', 'teach-home', '--registry', '/tmp/r.yaml',
        '--log', '/tmp/home.jsonl'])
    assert args.pose_id == 'home_dock'
    assert args.approach_offset_m == 0.7
    assert not hasattr(args, 'table_id')
    assert not hasattr(args, 'execute')


@pytest.mark.parametrize(('dwell', 'timeout'), [
    ('0', '45'), ('nan', '45'), ('20', '20'), ('20', '10'),
])
def test_roundtrip_rejects_invalid_box_wait(dwell, timeout):
    """A box wait must be positive and finish before its timeout."""
    with pytest.raises(SystemExit):
        parse_args([
            'service', 'roundtrip', '--registry', '/tmp/r.yaml',
            '--table-id', 'table_02', '--log', '/tmp/r.jsonl',
            '--dwell-s', dwell, '--box-timeout-s', timeout])


def test_roundtrip_rejects_duplicate_station_ids():
    """An accidental duplicate cannot trigger the same station twice."""
    with pytest.raises(SystemExit):
        parse_args([
            'service', 'roundtrip', '--registry', '/tmp/r.yaml',
            '--table-id', 'table_02', '--table-id', 'table_02',
            '--log', '/tmp/r.jsonl'])


def stable_box():
    return {
        'detected': True, 'stable': True, 'surface_kind': 'front',
        'front_distance_m': 0.5, 'lateral_error_m': 0.01,
        'edge_angle_deg': 1.0,
    }


def test_box_dwell_requires_continuous_fresh_front_detection():
    """A stale or lost frame resets the full dwell instead of counting gaps."""
    gate = BoxDwell(2.0, maximum_age_s=0.5)
    assert gate.observe(1.0, 1.0, stable_box())['hold_s'] == 0.0
    assert not gate.observe(2.0, 2.0, stable_box())['confirmed']
    assert gate.observe(3.0, 3.0, stable_box())['confirmed']
    assert not gate.observe(4.0, 3.0, stable_box())['confirmed']
    assert gate.observe(5.0, 5.0, stable_box())['hold_s'] == 0.0


def test_box_dwell_rejects_top_plane_and_nonfinite_geometry():
    """The mission waits for a finite front face rather than any depth plane."""
    gate = BoxDwell(1.0)
    top = {**stable_box(), 'surface_kind': 'top'}
    invalid = {**stable_box(), 'edge_angle_deg': math.nan}
    assert gate.observe(1.0, 1.0, top)['reason'] == 'box_not_stable'
    assert gate.observe(2.0, 2.0, invalid)['reason'] == 'box_observation_invalid'


def test_roundtrip_orders_destination_box_wait_and_home():
    """Home is attempted only after arrival and the complete box dwell."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=True)
    node.go_home = Mock(return_value=True)
    node.emit = Mock()
    assert node.roundtrip('table_01', execute=True, dwell_s=20, box_timeout_s=45)
    node.visit.assert_called_once_with('table_01', execute=True)
    node.wait_for_box.assert_called_once_with(20, 45)
    node.go_home.assert_called_once_with(execute=True)
    assert node.emit.call_args.args[0] == 'roundtrip_complete'


def test_roundtrip_visits_multiple_stations_before_one_home_return():
    """Each selected station completes its box dwell before the next goal."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=True)
    node.go_home = Mock(return_value=True)
    node.emit = Mock()

    assert node.roundtrip(
        ['kitchen_station', 'table_01'], execute=True,
        dwell_s=20, box_timeout_s=45)

    assert [call.args[0] for call in node.visit.call_args_list] == [
        'kitchen_station', 'table_01']
    assert node.wait_for_box.call_count == 2
    node.go_home.assert_called_once_with(execute=True)
    completed = [
        call.kwargs['station_id'] for call in node.emit.call_args_list
        if call.args[0] == 'station_complete']
    assert completed == ['kitchen_station', 'table_01']


def test_roundtrip_does_not_leave_destination_after_box_failure():
    """Missing box evidence prevents an unsupported successful return claim."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.visit = Mock(return_value=True)
    node.wait_for_box = Mock(return_value=False)
    node.go_home = Mock()
    node.emit = Mock()
    assert not node.roundtrip('table_01', execute=True)
    node.go_home.assert_not_called()
    assert node.emit.call_args.kwargs['phase'] == 'box_wait'


def test_go_home_plans_taught_pose_and_preserves_yaw_target():
    """A plan-only home check uses the taught pose and never starts motion."""
    node = route()
    home = taught_pose('home_dock', (-0.5, 0.4, -1.2), {},
                       approach_offset_m=0.7)
    node.registry['home'] = home
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node.plan_pose = Mock(return_value={'ok': True, 'error_code': 0})
    node.execute = Mock()
    node.emit = Mock()

    assert node.go_home(execute=False)

    node.plan_pose.assert_called_once_with(home)
    node.execute.assert_not_called()
    selected = next(
        call for call in node.emit.call_args_list
        if call.args[0] == 'home_selected')
    assert selected.kwargs['target_pose'] == [-0.5, 0.4, -1.2]
    assert node.emit.call_args.args[0] == 'home_planned_only'


def serving_route():
    """Prepare an explicitly taught home and one table without ROS hardware."""
    node = route()
    node.registry['home'] = taught_pose('home_dock', (0, 0, 0), {})
    node.registry['tables'] = [{
        'table_id': 'table_01', 'enabled': True,
        'service_poses': [taught_pose('table_01_main', (1, 0, 1.57), {})],
    }]
    return node


@pytest.mark.parametrize('failed_step', [None, 0, 1, 2, 3, 4])
def test_serving_sequence_stops_at_each_failed_phase(failed_step):
    """No later goal is dispatched after failed parking or incomplete dwell."""
    node = serving_route()
    calls = []

    def step(name):
        calls.append(name)
        return len(calls) - 1 != failed_step

    node.go_home = lambda execute: step('home')
    node.visit = lambda table_id, execute: step(table_id)
    node.wait_parked = lambda seconds: step(('wait', seconds))
    node.emit = Mock()
    assert node.serve('table_01', execute=True) is (failed_step is None)
    expected = ['home', ('wait', 5.0), 'table_01', ('wait', 5.0), 'home']
    assert calls == (expected if failed_step is None else expected[:failed_step + 1])
    assert node.emit.call_args.args[0] == (
        'serving_complete' if failed_step is None else 'serving_failed')


def test_serving_rejects_unknown_table_before_initial_home_motion():
    """A wrong destination cannot move the robot toward home first."""
    node = serving_route()
    node.go_home = Mock()
    with pytest.raises(ValueError, match='unknown table'):
        node.serve('absent', execute=True)
    node.go_home.assert_not_called()


def test_serving_plan_skips_dwell_and_never_enables_execution():
    """Preview checks endpoints and never claims a physical completed task."""
    node = serving_route()
    node.go_home = Mock(return_value=True)
    node.visit = Mock(return_value=True)
    node.wait_parked = Mock()
    node.emit = Mock()
    assert node.serve('table_01')
    assert all(call.kwargs == {'execute': False}
               for call in node.go_home.call_args_list)
    node.visit.assert_called_once_with('table_01', execute=False)
    node.wait_parked.assert_not_called()
    assert node.emit.call_args.args[0] == 'serving_plan_checks_complete'


def test_serving_operator_stop_prevents_next_goal():
    """An operator stop during dwell never launches the table transit."""
    node = serving_route()
    node.go_home = Mock(return_value=True)
    node.visit = Mock()

    def hold(_seconds):
        node.stop_requested = True
        return True

    node.wait_parked = hold
    assert not node.serve('table_01', execute=True)
    node.visit.assert_not_called()


def test_serve_cli_selects_one_table_and_defaults_to_no_motion():
    """The new workflow does not require a kitchen or pouring argument."""
    args = parse_args(['service', 'serve', '--registry', '/tmp/r.yaml',
                       '--table-id', 'table_01', '--log', '/tmp/s.jsonl'])
    assert args.command == 'serve' and not args.execute
    assert args.table_id == 'table_01'


def test_wait_parked_uses_selected_pose_and_clears_previous_deadline():
    """The dwell checks the taught yaw and does not inherit transit timeout."""
    node = serving_route()
    node.selected_pose = node.registry['tables'][0]['service_poses'][0]
    node.run_deadline_s = 1.0
    node._verify_parking_stop = Mock(return_value=True)
    assert node.wait_parked(5.0)
    assert node.run_deadline_s is None
    node._verify_parking_stop.assert_called_once_with(
        1, {'x': 1.0, 'y': 0.0, 'yaw': 1.57}, None, hold_s=5.0)


def reverse_route():
    node = route()
    node.registry['home'] = {
        **taught_pose('home_dock', (0.0, 0.0, 0.0), {}, approach_offset_m=.7),
        'parking_direction': 'reverse',
    }
    node.selected_pose = node.registry['home']
    node.parking_command = (0.0, 0.0, 0.0)
    node.parking_odom = (0.0, SimpleNamespace(sec=0, nanosec=0), 0.0, 0.0)
    node._parking_observation = lambda: {'actual_pose': (1.2, 0.0, 0.0)}
    node.verify_live_maps = Mock()
    node.capture_stationary_pose = Mock(return_value=((.7, 0.0, 0.0), {}))
    node._make_reverse_path = Mock(return_value=RosPath())
    node._reverse_path_valid = Mock(return_value=True)
    return node


def test_home_cli_does_not_need_table_and_never_moves_by_default():
    args = parse_args(['service', 'home', '--registry', '/tmp/r.yaml',
                       '--log', '/tmp/h.jsonl'])
    assert args.command == 'home' and not args.execute
    args = parse_args(['service', 'teach-home', '--registry', '/tmp/r.yaml',
                       '--log', '/tmp/t.jsonl', '--parking-direction', 'reverse'])
    assert args.parking_direction == 'reverse'


@pytest.mark.parametrize('failure', [None, 'path', 'plan', 'stage', 'reverse'])
def test_reverse_home_stages_before_following_and_stops_on_failure(failure):
    node = reverse_route()
    events = []
    node._reverse_path_valid = lambda path, **_kwargs: failure != 'path'

    def plan(pose, single):
        events.append(('plan', pose['x_m'], single))
        return {'ok': failure != 'plan'}

    node.plan_pose = plan
    node.execute = lambda: events.append(('stage',)) or failure != 'stage'
    node._execute_reverse_path = lambda path: events.append(('reverse',)) or failure != 'reverse'
    assert node._go_home_reverse(node.registry['home'], True) is (failure is None)
    expected = [('plan', .7, True), ('stage',), ('reverse',)]
    cut = {'path': 0, 'plan': 1, 'stage': 2, 'reverse': 3, None: 3}[failure]
    assert events == expected[:cut]
    assert node.waypoints[-1]['x'] == 0.0


def test_already_home_confirms_without_departing_and_redocking():
    node = reverse_route()
    node._parking_observation = lambda: {'actual_pose': (.01, .01, .01)}
    node._verify_parking_stop = Mock(return_value=True)
    node.execute = Mock()
    assert node._go_home_reverse(node.registry['home'], True)
    node.execute.assert_not_called()
    node._reverse_path_valid.assert_not_called()
    node._verify_parking_stop.assert_called_once()


def test_reverse_preview_has_no_motion_and_no_stationary_capture():
    node = reverse_route()
    node.plan_pose = Mock(return_value={'ok': True})
    node.execute = Mock()
    node._execute_reverse_path = Mock()
    assert node._go_home_reverse(node.registry['home'], False)
    node.execute.assert_not_called()
    node._execute_reverse_path.assert_not_called()
    node.capture_stationary_pose.assert_not_called()


def test_reverse_cold_start_without_velocity_sample_can_reach_staging():
    node = reverse_route()
    node.parking_command = None
    node._parking_observation = Mock(side_effect=ValueError('no command sample'))
    node.plan_pose = Mock(return_value={'ok': True})
    node.execute = Mock(return_value=True)
    node._execute_reverse_path = Mock(return_value=True)
    assert node._go_home_reverse(node.registry['home'], True)
    node.execute.assert_called_once()
    node._parking_observation.assert_not_called()


@pytest.mark.parametrize('status,confirmed,success', [
    (GoalStatus.STATUS_SUCCEEDED, True, True),
    (GoalStatus.STATUS_SUCCEEDED, False, False),
    (GoalStatus.STATUS_ABORTED, True, False),
    (GoalStatus.STATUS_CANCELED, True, False),
])
def test_reverse_follow_path_preserves_verifier_and_terminal_outcome(status, confirmed, success):
    node = reverse_route()
    node.follow_reverse = Mock()
    node.follow_reverse.send_goal_async.return_value = done(handle(status))
    node._verify_parking_stop = Mock(return_value=confirmed)
    path = RosPath()
    assert node._execute_reverse_path(path) is success
    goal = node.follow_reverse.send_goal_async.call_args.args[0]
    assert goal.path == path and goal.controller_id == 'ParkingReverse'
    assert goal.goal_checker_id == 'parking_goal_checker'
    assert node.active_action_type is FollowPath
    assert node.active_handle is None
    assert node._verify_parking_stop.called is (status == GoalStatus.STATUS_SUCCEEDED)


@pytest.mark.parametrize('case', ['handoff', 'outside', 'stop', 'link', 'nav', 'cancel_failed', 'arrived_abort'])
def test_intermediate_reverse_position_handoff_keeps_abort_and_cancel_checks(monkeypatch, case):
    node = reverse_route()
    running = SimpleNamespace(done=Mock(side_effect=([True] if case == 'arrived_abort' else [False, True])), result=Mock(
        return_value=SimpleNamespace(status=GoalStatus.STATUS_ABORTED,
                                     result=SimpleNamespace(error_code=105))))
    accepted = handle(future=running)
    node.follow_reverse = Mock()
    node.follow_reverse.send_goal_async.return_value = done(accepted)
    node._departure_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node._odom_pose = Mock(return_value=((.9, 0., 1.2) if case == 'outside'
                                       else (.96, .01, 1.2)))
    node.finish_navigation = Mock(return_value=case != 'cancel_failed')
    node.emit = Mock()
    node._route_event = Mock()
    node._verify_parking_stop = Mock(side_effect=AssertionError('intermediate is not final'))
    def spin(*_args, **_kwargs):
        if case == 'stop':
            node.stop_requested = True
        if case == 'link':
            node._operator_link_ready.return_value = False
        if case == 'nav':
            node._navigation_ready.return_value = False
    monkeypatch.setattr(restaurant_service, 'spin_node', spin)
    path = RosPath()
    path.header.frame_id = 'odom'
    endpoint = PoseStamped()
    endpoint.pose.position.x = 1.
    path.poses = [endpoint]
    kwargs = dict(goal_checker_id='dock_position_checker', final=False,
                  position_handoff_m=.05)
    if case == 'cancel_failed':
        with pytest.raises(RuntimeError, match='cancellation unconfirmed'):
            node._execute_reverse_once(path, **kwargs)
    else:
        assert node._execute_reverse_once(path, **kwargs) is (case in ('handoff', 'arrived_abort'))
    node.finish_navigation.assert_called_once()
    node._verify_parking_stop.assert_not_called()
    if case == 'handoff':
        assert node.emit.call_args.args == ('reverse_position_handoff',)


def test_reverse_lost_acceptance_uses_follow_path_cancel_and_query(monkeypatch):
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once',
                        lambda *args, **kwargs: None)
    node = reverse_route()
    node.active_action_type = FollowPath
    node.navigation_uuid = FollowPath.Impl.SendGoalService.Request().goal_id
    node.cancel_reverse = Mock()
    node.cancel_reverse.call_async.return_value = done(None)
    node.query_reverse = Mock()
    node.query_reverse.call_async.return_value = done(
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    node.cancel_navigation = Mock()
    node.query_navigation = Mock()
    assert node._cancel_navigation_uuid()
    node.cancel_reverse.call_async.assert_called_once()
    node.query_reverse.call_async.assert_called_once()
    node.cancel_navigation.call_async.assert_not_called()


def test_reverse_single_waypoint_tracks_accepted_goal_before_logging_failure():
    node = reverse_route()
    node.waypoints = node.waypoints[-1:]
    node.follow_reverse = Mock()
    accepted = handle()
    node.follow_reverse.send_goal_async.return_value = done(accepted)
    node._verify_parking_stop = Mock(return_value=True)
    assert node._execute_reverse_path(RosPath())
    assert node._verify_parking_stop.call_args.args[0] == 0


def test_reverse_logging_exception_still_owns_goal_for_cancellation():
    node = reverse_route()
    node.follow_reverse = Mock()
    accepted = handle()
    node.follow_reverse.send_goal_async.return_value = done(accepted)
    node._route_event = Mock(side_effect=RuntimeError('log failed'))
    owned = []
    node.finish_navigation = lambda: owned.append(node.active_handle) or True
    with pytest.raises(RuntimeError, match='log failed'):
        node._execute_reverse_path(RosPath())
    assert owned == [accepted]


@pytest.mark.parametrize('angle', [0.0, math.nan, math.inf, True, math.pi])
def test_search_rotation_rejects_unbounded_request_before_motion(angle):
    node = route()
    node.spin_search = Mock()
    with pytest.raises(ValueError):
        node.search_rotation(angle)
    node.spin_search.send_goal_async.assert_not_called()


@pytest.mark.parametrize('status,error,observed_yaw,expected', [
    (GoalStatus.STATUS_SUCCEEDED, 0, math.pi / 6, True),
    (GoalStatus.STATUS_SUCCEEDED, 0, -math.pi / 6, False),
    (GoalStatus.STATUS_SUCCEEDED, 0, 0.0, False),
    (GoalStatus.STATUS_ABORTED, 0, math.pi / 6, False),
    (GoalStatus.STATUS_SUCCEEDED, 1, math.pi / 6, False),
])
def test_search_uses_spin_and_observed_turn_not_xy_parking(
        status, error, observed_yaw, expected):
    node = route()
    node.spin_search = Mock()
    node.spin_search.send_goal_async.return_value = done(handle(status, error))
    node.capture_stationary_pose = Mock(side_effect=[
        ((1.0, 2.0, 0.0), {}), ((1.0, 2.0, observed_yaw), {})])
    node._odom_pose = Mock(side_effect=[
        (1.0, 2.0, 0.0), (1.0, 2.0, observed_yaw)])
    node._search_parameters_ready = Mock(return_value=True)
    node.navigate = Mock()
    node._verify_parking_stop = Mock()
    assert node.search_rotation(math.pi / 6) is expected
    goal = node.spin_search.send_goal_async.call_args.args[0]
    assert goal.target_yaw == pytest.approx(math.pi / 6)
    assert 0 < goal.time_allowance.sec <= 20
    node.navigate.send_goal_async.assert_not_called()
    node._verify_parking_stop.assert_not_called()
    assert node.active_handle is None
    assert node.active_action_type is Spin


def test_search_distinguishes_nav2_failure_from_observation_rejection():
    node = route()
    node.spin_search = Mock()
    node._search_parameters_ready = Mock(return_value=True)

    node.spin_search.send_goal_async.return_value = done(
        handle(GoalStatus.STATUS_ABORTED, 0))
    node.capture_stationary_pose = Mock(return_value=((1.0, 2.0, 0.0), {}))
    node._odom_pose = Mock(return_value=(1.0, 2.0, 0.0))
    assert not node.search_rotation(math.pi / 6)
    assert node.last_search_failure_reason == 'nav2_spin_failed'

    node.spin_search.send_goal_async.return_value = done(
        handle(GoalStatus.STATUS_SUCCEEDED, 0))
    node.capture_stationary_pose = Mock(side_effect=[
        ((1.0, 2.0, 0.0), {}),
        ((1.135, 2.0, math.pi / 6), {}),
    ])
    node._odom_pose = Mock(side_effect=[
        (1.0, 2.0, 0.0), (1.135, 2.0, math.pi / 6)])
    assert not node.search_rotation(math.pi / 6)
    assert node.last_search_failure_reason == 'rotation_observation_not_confirmed'


def test_search_observes_motion_in_odom_when_map_to_odom_jumps():
    """A localization correction must not look like Spin translation."""
    node = route()
    node.spin_search = Mock()
    node.spin_search.send_goal_async.return_value = done(
        handle(GoalStatus.STATUS_SUCCEEDED, 0))
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0), {}),
        ((0.135171, 0.0, 0.403295), {}),
    ])
    node._odom_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0)),
        ((0.000401, 0.0, 0.516950)),
    ])
    node._search_parameters_ready = Mock(return_value=True)

    assert node.search_rotation(0.455108)
    assert node._odom_pose.call_count == 2
    observed = [json.loads(line) for line in node.result_stream.getvalue().splitlines()
                if json.loads(line)['event'] == 'search_rotation_observed'][-1]
    assert observed['observation_frame'] == 'odom'
    assert observed['displacement_m'] == pytest.approx(0.000401)


def test_successful_search_clears_stale_failure_reason():
    node = route()
    node.last_search_failure_reason = 'nav2_spin_failed'
    node.spin_search = Mock()
    node.spin_search.send_goal_async.return_value = done(
        handle(GoalStatus.STATUS_SUCCEEDED, 0))
    node.capture_stationary_pose = Mock(side_effect=[
        ((1.0, 2.0, 0.0), {}), ((1.0, 2.0, math.pi / 6), {})])
    node._odom_pose = Mock(side_effect=[
        (1.0, 2.0, 0.0), (1.0, 2.0, math.pi / 6)])
    node._search_parameters_ready = Mock(return_value=True)

    assert node.search_rotation(math.pi / 6)
    assert node.last_search_failure_reason is None


def test_search_terminal_operator_stop_is_not_reported_as_nav2_failure():
    node = route()
    node.spin_search = Mock()
    terminal = Mock()
    terminal.done.return_value = True
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=Spin.Result())

    def stopped_result():
        node.stop_requested = True
        return wrapped

    terminal.result.side_effect = stopped_result
    node.spin_search.send_goal_async.return_value = done(handle(future=terminal))
    node.capture_stationary_pose = Mock(return_value=((1.0, 2.0, 0.0), {}))
    node._odom_pose = Mock(return_value=(1.0, 2.0, 0.0))
    node._search_parameters_ready = Mock(return_value=True)
    node.finish_navigation = Mock(return_value=True)

    assert not node.search_rotation(math.pi / 6)
    assert node.last_search_failure_reason == 'operator_stop'


def test_spin_lost_acceptance_cancels_only_its_action_uuid(monkeypatch):
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once',
                        lambda *args, **kwargs: None)
    node = route()
    node.active_action_type = Spin
    node.navigation_uuid = Spin.Impl.SendGoalService.Request().goal_id
    node.cancel_spin = Mock()
    node.cancel_spin.call_async.return_value = done(None)
    node.query_spin = Mock()
    node.query_spin.call_async.return_value = done(
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    node.cancel_navigation = Mock()
    assert node._cancel_navigation_uuid()
    node.cancel_spin.call_async.assert_called_once()
    node.cancel_navigation.call_async.assert_not_called()


@pytest.mark.parametrize('maximum,minimum,plugins,stamped,ready', [
    (.2, .1, ['wait', 'spin'], False, True),
    (.7, .1, ['wait', 'spin'], False, False),
    (.2, .4, ['wait', 'spin'], False, False),
    (.2, .1, ['wait'], False, False),
    (.2, .1, ['wait', 'spin'], True, False),
])
def test_search_checks_installed_spin_profile(
        monkeypatch, maximum, minimum, plugins, stamped, ready):
    node = route()
    client = Mock()
    values = [Parameter('p', value=v).get_parameter_value() for v in (
        plugins, maximum, minimum, 'odom', 'base_footprint', stamped)]
    client.call_async.return_value = done(SimpleNamespace(values=values))
    node.create_client = Mock(return_value=client)
    assert node._search_parameters_ready() is ready


@pytest.mark.parametrize('guard_failure,operator_stop,repositionable,expected_reason', [
    (None, False, True, 'operator_or_spin_timeout'),
    ('scan stale: age=1s', False, False, 'scan stale: age=1s'),
    (None, True, False, 'operator_stop'),
    ('scan stale: age=1s', True, False, 'operator_stop'),
])
def test_search_timeout_cancels_accepted_spin(
        monkeypatch, guard_failure, operator_stop, repositionable, expected_reason):
    node = route()
    node.spin_search = Mock()
    pending = Future()
    accepted = handle(future=pending)
    node.spin_search.send_goal_async.return_value = done(accepted)
    node.capture_stationary_pose = Mock(return_value=((0., 0., 0.), {}))
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._search_parameters_ready = lambda: True
    clock = {'now': 0.0}
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.time.monotonic',
                        lambda: clock['now'])

    def spin_once(*args, **kwargs):
        clock['now'] += 21.0
        node.stop_requested = operator_stop
        node._guard_failure = lambda *_: guard_failure

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin_once)

    def cancel(handle, reason):
        assert handle is accepted
        pending.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    node._cancel = cancel
    assert not node.search_rotation(math.pi / 6)
    assert node.active_handle is None
    assert node.capture_stationary_pose.call_count == 1
    assert node.last_search_error_code == (Spin.Result.TIMEOUT if repositionable else None)
    assert node.last_search_failure_reason == expected_reason


def test_repeated_spin_profile_reads_do_not_accumulate_ros_clients(monkeypatch):
    """Count real rclpy clients, with only remote responses substituted."""
    import rclpy
    from rclpy.client import Client
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setattr(AsyncParameterClient, 'wait_for_services',
                        lambda *args, **kwargs: True)
    monkeypatch.setattr(Client, 'wait_for_service', lambda *args, **kwargs: True)
    context = Context()
    rclpy.init(context=context, domain_id=91)
    node = Node('parameter_reader_regression', context=context,
                start_parameter_services=False, enable_rosout=False)
    node.parking_contract = {'rotate_angular_radps': .2}
    node._parameter_readers = {}
    node._read_parameters = lambda remote, names: ServiceRoute._read_parameters(
        node, remote, names)
    response = SimpleNamespace(values=[
        Parameter('p', value=value).get_parameter_value() for value in
        (['wait', 'spin'], .2, .1, 'odom', 'base_footprint', False)])
    monkeypatch.setattr(Client, 'call_async', lambda *args: done(response))
    node._wait = lambda future, timeout: future.result()
    try:
        for _ in range(12):
            assert ServiceRoute._search_parameters_ready(node)
        assert len(list(node.clients)) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_parameter_reader_reuses_endpoint_but_not_returned_values():
    node = route()
    client = Mock()
    first = SimpleNamespace(values=['old'])
    second = SimpleNamespace(values=['new'])
    client.call_async.side_effect = [done(first), done(second)]
    node.create_client = Mock(return_value=client)
    assert node._read_parameters('map_server', ['yaml_filename']) is first
    assert node._read_parameters('map_server', ['yaml_filename']) is second
    node.create_client.assert_called_once()
    assert client.call_async.call_count == 2
    assert client.call_async.call_args.args[0].names == ['yaml_filename']
    client.remove_pending_request.assert_not_called()


def test_parameter_reader_absent_service_never_sends_request():
    node = route()
    client = Mock()
    client.wait_for_service.return_value = False
    node.create_client = Mock(return_value=client)
    assert node._read_parameters('map_server', ['yaml_filename']) is None
    client.call_async.assert_not_called()


def test_parameter_reader_discards_pending_request_on_interruption(monkeypatch):
    node = route()
    node.stop_requested = False
    client = Mock()
    pending = Future()
    client.call_async.return_value = pending
    node.create_client = Mock(return_value=client)
    clock = {'now': 0.0}
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock['now'])

    def interrupt(_node, timeout_sec):
        clock['now'] += timeout_sec
        node.stop_requested = True

    monkeypatch.setattr(corridor_route, 'spin_node', interrupt)
    with pytest.raises(RuntimeError, match='map_server parameter read interrupted'):
        node._read_parameters('map_server', ['yaml_filename'])
    client.remove_pending_request.assert_called_once_with(pending)
    client.call_async.assert_called_once()


def test_parameter_reader_stop_before_request_never_sends():
    node = route()
    node.stop_requested = True
    client = Mock()
    node.create_client = Mock(return_value=client)
    with pytest.raises(RuntimeError, match='interrupted before service wait'):
        node._read_parameters('map_server', ['yaml_filename'])
    client.wait_for_service.assert_not_called()
    client.call_async.assert_not_called()


def test_parameter_reader_resends_two_lost_reads_then_uses_fresh_third(monkeypatch):
    """A new node's first map_server read went unanswered (2026-10-06 10:26, 17:09)."""
    node = route()
    node.stop_requested = False
    node.get_logger = Mock()
    client = Mock()
    lost_first, lost_second, answered = Future(), Future(), Future()
    response = SimpleNamespace(values=['map.yaml'])
    answered.set_result(response)
    client.call_async.side_effect = [lost_first, lost_second, answered]
    node.create_client = Mock(return_value=client)
    clock = {'now': 0.0}
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock['now'])

    def spin(_node, timeout_sec):
        clock['now'] += timeout_sec

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    assert node._read_parameters('map_server', ['yaml_filename']) is response
    assert client.remove_pending_request.call_args_list == [
        ((lost_first,),), ((lost_second,),)]
    assert client.call_async.call_count == 3
    assert len({id(call.args[0]) for call in client.call_async.call_args_list}) == 3
    assert clock['now'] == pytest.approx(2.0)
    assert 'node=map_server attempt=3' in node.get_logger().warning.call_args.args[0]


def test_parameter_reader_accepts_first_response_after_second_request(monkeypatch):
    node = route()
    node.get_logger = Mock()
    client = Mock()
    first, second = Future(), Future()
    response = SimpleNamespace(values=['map.yaml'])
    client.call_async.side_effect = [first, second]
    node.create_client = Mock(return_value=client)
    clock = {'now': 0.0}
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock['now'])

    def spin(_node, timeout_sec):
        clock['now'] += timeout_sec
        if clock['now'] >= 1.2 and not first.done():
            first.set_result(response)

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    assert node._read_parameters('map_server', ['yaml_filename']) is response
    assert client.call_async.call_count == 2
    client.remove_pending_request.assert_called_once_with(second)
    assert clock['now'] == pytest.approx(1.2)


def test_parameter_reader_timeout_stays_inside_five_second_budget(monkeypatch):
    node = route()
    node.stop_requested = False
    node.get_logger = Mock()
    client = Mock()
    client.call_async.side_effect = lambda _request: Future()
    node.create_client = Mock(return_value=client)
    clock = {'now': 0.0}
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock['now'])

    def spin(_node, timeout_sec):
        clock['now'] += timeout_sec

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    with pytest.raises(RuntimeError, match=(
            r'keepout_filter_mask_server parameter read timed out: '
            r'total_budget=5\.0 s attempts=5 resend_interval=1\.0 s')):
        node._read_parameters('keepout_filter_mask_server', ['yaml_filename'])
    assert clock['now'] == 5.0
    assert client.call_async.call_count == 5
    assert client.remove_pending_request.call_count == 5


def test_parameter_reader_never_sends_after_deadline(monkeypatch):
    node = route()
    node.get_logger = Mock()
    client = Mock()
    pending = Future()
    client.call_async.return_value = pending
    node.create_client = Mock(return_value=client)
    clock = {'now': 0.0}
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock['now'])

    def overrun(_node, timeout_sec):
        assert timeout_sec <= 0.05
        clock['now'] = 5.1

    monkeypatch.setattr(corridor_route, 'spin_node', overrun)
    with pytest.raises(RuntimeError, match='total_budget=5.0 s attempts=1'):
        node._read_parameters('map_server', ['yaml_filename'])
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(pending)


def test_parameter_reader_does_not_retry_completed_service_exception():
    node = route()
    client = Mock()
    failed = Future()
    failed.set_exception(RuntimeError('peer failed'))
    client.call_async.return_value = failed
    node.create_client = Mock(return_value=client)
    with pytest.raises(RuntimeError, match='failed on attempt=1'):
        node._read_parameters('map_server', ['yaml_filename'])
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_not_called()


def test_parameter_reader_uses_real_read_only_service_and_fresh_responses(monkeypatch):
    """A get-only peer works without five unrelated parameter services."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    context = Context()
    rclpy.init(context=context, domain_id=91)
    executor = SingleThreadedExecutor(context=context)
    server = Node('read_only_peer', context=context, start_parameter_services=False,
                  enable_rosout=False)
    node = Node('fresh_parameter_reader', context=context, start_parameter_services=False,
                enable_rosout=False)
    node._parameter_readers = {}
    requests = []

    def respond(request, response):
        requests.append(list(request.names))
        response.values = [Parameter('revision', value=len(requests)).get_parameter_value()]
        return response

    server.create_service(GetParameters, 'read_only_peer/get_parameters', respond)
    executor.add_node(server)
    executor.add_node(node)

    def spin(_node, timeout_sec):
        executor.spin_once(timeout_sec=timeout_sec)

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    try:
        first = ServiceRoute._read_parameters(node, 'read_only_peer', ['revision'])
        second = ServiceRoute._read_parameters(node, 'read_only_peer', ['revision'])
        assert first.values[0].integer_value == 1
        assert second.values[0].integer_value == 2
        assert requests == [['revision'], ['revision']]
        assert len(list(node.clients)) == 1
    finally:
        executor.shutdown()
        node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('minimum,maximum,ready', [
    (-.08, .08, True), (0.0, .08, False), (-.09, .08, False),
    (-.08, .12, True), (-.08, .13, False),
])
def test_reverse_checks_actual_smoother_velocity_bounds(monkeypatch, minimum, maximum, ready):
    node = reverse_route()
    client = Mock()
    values = [Parameter('min_velocity', value=[minimum, 0.0, -.3]).get_parameter_value(),
              Parameter('max_velocity', value=[maximum, 0.0, .3]).get_parameter_value()]
    client.call_async.return_value = done(SimpleNamespace(values=values))
    node.create_client = Mock(return_value=client)
    assert node._reverse_smoother_ready() is ready


def test_reverse_path_static_failure_never_calls_live_validator(monkeypatch):
    node = route()
    node.registry.update(map={'yaml_path': '/tmp/map.yaml'},
                         keepout={'yaml_path': '/tmp/mask.yaml'})
    outline = [(-.3, -.2), (-.1, -.2), (-.1, -.3), (.1, -.3),
               (.1, .3), (-.1, .3), (-.1, .2), (-.3, .2)]
    node._footprint_outline = Mock(return_value=outline)
    node._reverse_footprint = Mock(side_effect=AssertionError('bounding rectangle used'))
    node.validate_reverse_path = Mock()
    path = RosPath()
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    path.poses = [pose]
    static_check = Mock(return_value=False)
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.static_corridor_clear',
                        static_check)
    assert not node._reverse_path_valid(path)
    assert static_check.call_args.args[-1] == outline
    node._reverse_footprint.assert_not_called()
    node.validate_reverse_path.call_async.assert_not_called()


@pytest.mark.parametrize('fresh', [True, False])
def test_dwell_real_verifier_and_event_writer_without_action_handle(monkeypatch, fresh):
    """Both dwell outcomes must log without inventing a completed goal handle."""
    node = serving_route()
    node.selected_pose = node.registry['tables'][0]['service_poses'][0]
    node.parking_motion_revision = 0
    node.parking_odom = None
    node.parking_observation_diagnostics = None
    clock = {'tick': 0}
    monkeypatch.setattr(
        'jdamr_cube_navigation.corridor_route.time.monotonic',
        lambda: clock['tick'] * 0.25)

    def spin_once(_node, timeout_sec):
        if timeout_sec:
            clock['tick'] += 1
            node.parking_odom = (
                clock['tick'] * 0.25,
                SimpleNamespace(sec=clock['tick'], nanosec=0), 0.0, 0.0)

    monkeypatch.setattr(
        'jdamr_cube_navigation.corridor_route.rclpy.spin_once', spin_once)
    node._parking_observation = lambda: {
        'actual_pose': (1.0, 0.0, 1.57),
        'linear_mps': 0.0, 'angular_radps': 0.0,
        'cmd_linear_mps': 0.0, 'cmd_angular_radps': 0.0,
        'sample_age_s': 0.01 if fresh else 1.0,
    }
    assert node.wait_parked(5.0) is fresh
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    assert [record['event'] for record in records] == [
        'parked_dwell_started', 'parked_dwell_observation',
        'parked_dwell_complete' if fresh else 'parked_dwell_failed',
    ]
    assert records[1]['goal_uuid'] is None
    assert records[1]['confirmed'] is fresh
    assert records[2]['confirmation']['confirmed'] is fresh
    assert clock['tick'] * 0.25 >= 5.0


@pytest.mark.parametrize('options', [
    ['--measured-front-gap-m', '0.05'], ['--gap-measurement-note', 'ruler'],
    ['--target-front-gap-m', 'nan'], ['--target-front-gap-m', '-1'],
])
def test_invalid_gap_options_fail_before_ros_startup(options):
    """Invalid measurements must not wait for the robot to connect."""
    with pytest.raises(SystemExit) as failure:
        parse_args(['service', 'teach', '--registry', '/tmp/r.yaml',
                    '--table-id', 'table_02', '--pose-id', 'main',
                    '--log', '/tmp/t.jsonl', *options])
    assert failure.value.code == 2


def test_navigation_terminal_failure_is_propagated():
    """Do not let an unresolved action fall through to the next waypoint."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(handle(GoalStatus.STATUS_ABORTED))
    node.finish_navigation = Mock(return_value=False)
    with pytest.raises(RuntimeError, match='cancellation unconfirmed'):
        node.execute()
    assert node.navigate.send_goal_async.call_count == 1
    assert '"event": "arrived"' not in node.result_stream.getvalue()


def test_verified_maps_reuse_asset_audit_but_keep_live_identity_and_command_check():
    node = route()
    node.registry = {'map': {'yaml_path': 'map'}, 'keepout': {'yaml_path': 'mask'}}
    node._verified_map_identity = json.dumps(node.registry, sort_keys=True)
    node.live_grids = node.expected_grids = {'map': 'a', 'keepout': 'b'}
    node._startup_protection_ready = Mock(return_value=None)
    node._set_collision_monitor = Mock(return_value=True)
    node._read_parameters = Mock()
    node.verify_live_maps()
    node._read_parameters.assert_not_called()
    node._startup_protection_ready.assert_called_once_with(require_command_path=True)
    node._set_collision_monitor.assert_called_once_with(True)
    node.live_grids = {'map': 'changed', 'keepout': 'b'}
    with pytest.raises(RuntimeError, match='changed'):
        node.verify_live_maps()


def test_map_audit_cache_is_created_by_first_call_not_seeded_by_test(monkeypatch):
    node = route()
    node.registry = {'map': {'yaml_path': 'map'}, 'keepout': {'yaml_path': 'mask'}}
    node.live_grids = {'map': 'map', 'keepout': 'mask'}
    module = 'jdamr_cube_navigation.restaurant_service.'
    validate = Mock()
    monkeypatch.setattr(module + 'validate_registry', validate)
    monkeypatch.setattr(module + 'map_grid_signature', lambda path: path)
    monkeypatch.setattr(module + 'verify_identity', Mock())
    node._read_parameters = Mock(side_effect=lambda name, _: SimpleNamespace(values=[
        Parameter('yaml_filename', value='map' if name == 'map_server' else 'mask')
        .get_parameter_value()]))
    node._startup_protection_ready = Mock(return_value=None)
    node._set_collision_monitor = Mock(return_value=True)
    node.verify_live_maps(require_command_path=False)
    node._set_collision_monitor.assert_not_called()
    node.verify_live_maps()
    node._set_collision_monitor.assert_called_once_with(True)
    assert node._read_parameters.call_count == 2
    validate.assert_called_once()
    assert [call.kwargs for call in node._startup_protection_ready.call_args_list] == [
        {'require_command_path': False}, {'require_command_path': True}]


def test_input_gap_between_waypoints_recovers_without_restarting_completed_leg():
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = lambda *_, **__: done(handle())
    node._navigation_ready = Mock(side_effect=[True, False, True])
    node._guard_failure = Mock(return_value='scan stale: age=1s')
    node._wait_for_input_recovery = Mock(side_effect=node._input_gap_recoverable)
    assert node.execute(final_parking=False)
    assert node.navigate.send_goal_async.call_count == 2
    node._wait_for_input_recovery.assert_called_once_with('scan stale: age=1s')


@pytest.mark.parametrize('entry', ['service', 'corridor', 'spin', 'reverse'])
@pytest.mark.parametrize('reason,stopped,retry', [
    ('scan stale: age=1s', False, True),
    ('scan stale: age=1s', True, False),
    ('battery low: voltage=10.4V', False, False),
    (None, False, False),
])
def test_action_boundary_marks_only_recoverable_input_gaps(entry, reason, stopped, retry):
    node = route()
    node.navigate = Mock()
    node.spin_search = Mock()
    node.follow_reverse = Mock()
    node._resume_waypoint_index = 0
    node._retry_guard_reason = None
    node.stop_requested = stopped
    node._navigation_ready = lambda **_: False
    node._guard_failure = lambda *_: reason
    if entry == 'service':
        result = node._execute_service_once(final_parking=False, alignment=False)
    elif entry == 'corridor':
        result = CorridorRoute._execute_route_once(node)
    elif entry == 'spin':
        result = node._search_rotation_once(math.pi / 6)
    else:
        result = node._execute_reverse_once(RosPath())
    assert not result
    assert node._retry_guard_reason == (reason if retry else None)
    if entry == 'spin':
        assert node.last_search_failure_reason == (
            'operator_stop' if stopped else reason or 'operator_stop')
    for client in (node.navigate, node.spin_search, node.follow_reverse):
        client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('execute', [False, True])
def test_alternate_selection_retains_waypoints_and_gap_evidence(execute):
    """Plan and report the selected table's two poses without reusing the primary."""
    node = route()
    primary = taught_pose('main', (1, 2, 0), {})
    alternate = taught_pose('other', (3, 4, math.pi / 2), {}, priority=2,
                            target_front_gap_m=0.05, measured_front_gap_m=0.06,
                            gap_measurement_note='ruler')
    node.registry['tables'] = [{'table_id': 'table_01', 'enabled': True,
                                'service_poses': [primary, alternate]}]
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    failed = ComputePathThroughPoses.Result()
    failed.error_code = ComputePathThroughPoses.Result.GOAL_OCCUPIED
    successful = ComputePathThroughPoses.Result()
    successful.path.poses = [PoseStamped()]
    successful.path.poses[-1].pose.position.x = 3.0
    successful.path.poses[-1].pose.position.y = 4.0
    node.compute = Mock()
    node.compute.send_goal_async.side_effect = [
        done(SimpleNamespace(accepted=True, get_result_async=lambda: done(
            SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=failed)))),
        done(SimpleNamespace(accepted=True, get_result_async=lambda: done(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=successful)))),
    ]
    expected = route_config(node.registry, alternate)['waypoints']

    def dispatch():
        assert node.waypoints == expected
        assert node.pose_id == 'other'
        return True

    node.execute = Mock(side_effect=dispatch)
    assert node.visit('table_01', execute=execute)
    assert node.execute.call_count == int(execute)
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    selected = next(record for record in records if record['event'] == 'selected')
    assert selected['waypoints'] == expected
    assert selected['front_gap']['teaching_measured_m'] == 0.06
    assert records[-1]['event'] == ('arrived' if execute else 'planned_only')
    assert records[-1]['front_gap']['arrival_verification'] == 'NOT_MEASURED'


@pytest.mark.parametrize('confirmed', [False, True])
def test_final_action_uses_parking_and_requires_confirmation(confirmed):
    """Two Nav2 successes alone cannot confirm service arrival."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = lambda *_, **__: done(handle())
    node._verify_parking_stop = Mock(return_value=confirmed)
    assert node.execute() is confirmed
    goals = [call.args[0] for call in node.navigate.send_goal_async.call_args_list]
    assert [goal.behavior_tree for goal in goals] == ['transit.xml', 'parking.xml']
    assert node._verify_parking_stop.call_count == 1


def test_observation_transit_replans_without_forcing_final_parking_yaw():
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = lambda *_, **__: done(handle())
    node._verify_parking_stop = Mock(return_value=False)
    assert node.execute(final_parking=False)
    goals = [call.args[0] for call in node.navigate.send_goal_async.call_args_list]
    assert [goal.behavior_tree for goal in goals] == ['transit.xml', 'transit.xml']
    node._verify_parking_stop.assert_not_called()


def test_intermediate_alignment_uses_separate_checker_without_final_confirmation():
    node = route()
    node.alignment_behavior_tree = 'alignment.xml'
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = lambda *_, **__: done(handle())
    node._verify_parking_stop = Mock(return_value=False)
    assert node.execute(final_parking=False, alignment=True)
    goals = [call.args[0] for call in node.navigate.send_goal_async.call_args_list]
    assert [goal.behavior_tree for goal in goals] == ['transit.xml', 'alignment.xml']
    node._verify_parking_stop.assert_not_called()


def test_sensor_gap_cancels_terminal_then_retries_only_current_waypoint(monkeypatch):
    node = route()
    result = Future()
    interrupted = handle(future=result)
    node.navigate = Mock()
    node.navigate.send_goal_async.side_effect = [done(handle()), done(interrupted), done(handle())]
    node._navigation_ready = Mock(side_effect=[True, True, False, True])
    node._guard_failure = Mock(return_value='scan stale: age=1s')
    node._wait_for_input_recovery = Mock(return_value=True)

    def spin(_node, timeout_sec):
        if interrupted.cancel_goal_async.called and not result.done():
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node.execute(final_parking=False)
    interrupted.cancel_goal_async.assert_called_once()
    node._wait_for_input_recovery.assert_called_once_with('scan stale: age=1s')
    assert node.navigate.send_goal_async.call_count == 3
    records = [json.loads(line) for line in node.result_stream.getvalue().splitlines()]
    accepted = [item['waypoint_id'] for item in records if item['event'] == 'accepted']
    assert accepted == ['approach', 'main', 'main']


def test_search_input_recovery_uses_remaining_angle_not_another_full_turn():
    node = route()
    node._wait_for_input_recovery = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0, 0, math.radians(10)), {}))
    node._odom_pose = Mock(return_value=(0, 0, math.radians(10)))
    angles = []

    def attempt(angle, *, measure_in_odom):
        assert measure_in_odom is True
        angles.append(angle)
        node._search_target_yaw = math.radians(30)
        node._retry_guard_reason = 'scan stale: age=1s'
        return len(angles) == 2

    node._search_rotation_once = attempt
    assert node.search_rotation(math.radians(30))
    assert angles == pytest.approx([math.radians(30), math.radians(20)])


def test_search_input_recovery_clears_failure_when_remaining_turn_is_complete():
    node = route()
    node._wait_for_input_recovery = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0, 0, 0), {}))
    node._odom_pose = Mock(return_value=(0, 0, math.radians(28)))

    def interrupted(angle, *, measure_in_odom):
        assert angle == pytest.approx(math.radians(30))
        assert measure_in_odom is True
        node._search_target_yaw = math.radians(30)
        node._retry_guard_reason = 'scan stale: age=1s'
        node.last_search_failure_reason = 'scan stale: age=1s'
        return False

    node._search_rotation_once = Mock(side_effect=interrupted)

    assert node.search_rotation(math.radians(30))
    node._search_rotation_once.assert_called_once()
    assert node.last_search_failure_reason is None


def test_reverse_recovery_rebuilds_and_revalidates_remaining_path():
    node = route()
    node._wait_for_input_recovery = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0.7, 0, 0.2), {}))
    node._make_reverse_path = Mock(return_value='remaining')
    node._reverse_path_valid = Mock(return_value=True)
    paths = []

    def attempt(path):
        paths.append(path)
        node._retry_guard_reason = 'odom stale: age=1s'
        return len(paths) == 2

    node._execute_reverse_once = attempt
    assert node._execute_reverse_path('original')
    assert paths == ['original', 'remaining']
    node._make_reverse_path.assert_called_once_with((0.7, 0, 0.2), (1.0, 0.0, 0.2))
    node._reverse_path_valid.assert_called_once_with('remaining')


@pytest.mark.parametrize('value', [None, 0, 1, 'false'])
def test_final_parking_mode_rejects_non_boolean_before_dispatch(value):
    node = route()
    node.navigate = Mock()
    with pytest.raises(ValueError, match='final_parking must be boolean'):
        node.execute(final_parking=value)
    node.navigate.send_goal_async.assert_not_called()


def test_action_failure_does_not_send_parking_goal():
    """Transit failure ends execution without crossing to the final waypoint."""
    node = route()
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(handle(GoalStatus.STATUS_ABORTED))
    node._verify_parking_stop = Mock()
    assert not node.execute()
    assert node.navigate.send_goal_async.call_count == 1
    node._verify_parking_stop.assert_not_called()


def test_stop_during_action_cancels_and_waits_for_terminal(monkeypatch):
    """A cancel acknowledgment is not the terminal action result."""
    node = route()
    pending = Future()
    accepted = handle(future=pending)
    node.navigate = Mock()
    node.navigate.send_goal_async.return_value = done(accepted)

    def spin(_node, timeout_sec):
        node.stop_requested = True
        if accepted.cancel_goal_async.called:
            pending.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert not node.execute()
    accepted.cancel_goal_async.assert_called_once()
    assert node.active_handle is None
    assert 'navigation_terminal' in node.result_stream.getvalue()


def test_late_goal_acceptance_is_canceled(monkeypatch):
    """Shutdown drains a pending send and cancels the accepted goal."""
    node = route()
    node.stop_requested = True
    node.pending_goal = Future()
    result = Future()
    accepted = handle(future=result)

    def spin(_node, timeout_sec):
        if node.pending_goal is not None and not node.pending_goal.done():
            node.pending_goal.set_result(accepted)
        elif accepted.cancel_goal_async.called:
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    assert node.finish_navigation()
    accepted.cancel_goal_async.assert_called_once()
    assert node.active_handle is None


def test_uuid_cancellation_confirms_our_goal_without_acceptance(monkeypatch):
    """Use an exact goal UUID, never a cancel-all request, on lost acceptance."""
    node = route()
    node.navigation_uuid = NavigateToPose.Impl.SendGoalService.Request().goal_id
    node.navigation_uuid.uuid = list(range(16))
    node.cancel_navigation = Mock()
    node.query_navigation = Mock()
    node.cancel_navigation.call_async.return_value = done(SimpleNamespace())
    response = NavigateToPose.Impl.GetResultService.Response()
    response.status = GoalStatus.STATUS_CANCELED
    node.query_navigation.call_async.return_value = done(response)
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.rclpy.spin_once', lambda *_, **__: None)
    assert node._cancel_navigation_uuid()
    request = node.cancel_navigation.call_async.call_args.args[0]
    assert bytes(request.goal_info.goal_id.uuid) == bytes(range(16))
    assert request.goal_info.stamp.sec == 0
    assert 'navigation_terminal' in node.result_stream.getvalue()


def test_amcl_stale_after_motion_is_blocked(monkeypatch):
    """Service mode retains the existing moving-base localization timeout."""
    node = route()
    del node._guard_failure
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.CorridorRoute._guard_failure',
        lambda *_, **__: None)
    node.amcl_covariance = (0.01, 0.01)
    node.amcl_seen = 0.0
    node.amcl_motion_distance_m = 0.3
    node.amcl_motion_rotation_rad = 0.0
    node.revisit_motion_min_distance_m = 0.25
    node.revisit_motion_min_rotation_rad = 0.5
    node.revisit_motion_amcl_freshness_s = 25.0
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.time.monotonic', lambda: 26.0)
    assert node._guard_failure(False) == 'AMCL stale after motion'
    node.amcl_seen = 20.0
    assert node._guard_failure(False) is None
    node.amcl_yaw_covariance_rad2 = math.nan
    assert node._guard_failure(False) == 'AMCL yaw covariance invalid'
    node.amcl_yaw_covariance_rad2 = 1.0
    assert node._guard_failure(False).startswith('AMCL yaw covariance high')


def test_planner_cancellation_without_error_is_not_success():
    """Even a nonempty path needs a successful terminal action status."""
    node = route()
    result = ComputePathThroughPoses.Result()
    result.path.poses = [PoseStamped()]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=result)
    node.compute = Mock()
    node.compute.send_goal_async.return_value = done(SimpleNamespace(
        accepted=True, get_result_async=lambda: done(wrapped)))
    planned = node.plan_pose({'id': 'main', 'x_m': 1.0, 'y_m': 2.0,
                             'yaw_rad': 0.4, 'approach_offset_m': 0.5})
    assert not planned['ok']
    goal = node.compute.send_goal_async.call_args.args[0]
    assert len(goal.goals) == 2 and goal.planner_id == 'GridBased'


def test_wrong_live_map_prevents_any_goal(monkeypatch):
    """Check live server assets before planning or movement."""
    node = route()
    node.registry = {'map': {'yaml_path': 'expected'}, 'keepout': {'yaml_path': 'mask'}}
    node.live_grids = {'map': 'expected', 'keepout': 'mask'}
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.map_grid_signature', lambda x: x)
    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.validate_registry', lambda _: None)
    client = Mock()
    client.call_async.return_value = done(SimpleNamespace(values=[
        Parameter('yaml_filename', value='/wrong/map.yaml').get_parameter_value()]))
    node.create_client = Mock(return_value=client)
    verify = Mock(side_effect=ValueError('map hash mismatch'))
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.verify_identity', verify)
    with pytest.raises(ValueError, match='hash mismatch'):
        node.verify_live_maps()
    assert verify.call_args.args[1] == '/wrong/map.yaml'


def test_delayed_live_map_discovery_preserves_identity_checks(monkeypatch):
    node = route()
    node.registry = {'map': {'yaml_path': 'expected'}, 'keepout': {'yaml_path': 'mask'}}
    module = 'jdamr_cube_navigation.restaurant_service.'
    monkeypatch.setattr(module + 'map_grid_signature', lambda x: x)
    monkeypatch.setattr(module + 'validate_registry', lambda _: None)
    clock = {'now': 0.0}
    monkeypatch.setattr(module + 'time.monotonic', lambda: clock['now'])

    def spin(*_, **__):
        clock['now'] += 1.0
        if clock['now'] >= 4.0:
            node.live_grids = {'map': 'wrong', 'keepout': 'mask'}

    monkeypatch.setattr(module + 'rclpy.spin_once', spin)
    with pytest.raises(RuntimeError, match='does not match registered data'):
        node.verify_live_maps()
    assert clock['now'] == 4.0


@pytest.mark.parametrize('stage', ['visit', 'go_home'])
def test_live_map_wait_does_not_consume_leg_budget(monkeypatch, stage):
    """A fresh process waits up to 30 s for maps; the leg budget starts afterwards."""
    node = serving_route()
    node.registry['home'] = taught_pose('home_dock', (-0.5, 0.4, -1.2), {},
                                        approach_offset_m=0.7)
    clock = {'now': 100.0}
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.time.monotonic',
                        lambda: clock['now'])
    node.run_deadline_s = 50.0  # an earlier leg's expired deadline
    seen = []

    def discover():
        seen.append(node.run_deadline_s)
        clock['now'] += 30.0

    node.verify_live_maps = Mock(side_effect=discover)
    node.wait_until_ready = Mock(return_value=False)
    with pytest.raises(RuntimeError, match='navigation data unavailable'):
        if stage == 'visit':
            node.visit('table_01')
        else:
            node.go_home()
    assert seen == [None]
    assert node.run_deadline_s == 130.0 + 180.0


def test_missing_live_map_reports_missing_streams(monkeypatch):
    node = route()
    node.registry = {'map': {'yaml_path': 'expected'}, 'keepout': {'yaml_path': 'mask'}}
    node.live_grids = {'map': 'expected'}
    module = 'jdamr_cube_navigation.restaurant_service.'
    monkeypatch.setattr(module + 'map_grid_signature', lambda x: x)
    monkeypatch.setattr(module + 'validate_registry', lambda _: None)
    clock = {'now': 0.0}
    monkeypatch.setattr(module + 'time.monotonic', lambda: clock['now'])
    monkeypatch.setattr(module + 'rclpy.spin_once',
                        lambda *_, **__: clock.update(now=clock['now'] + 1.0))
    with pytest.raises(RuntimeError, match='live map data unavailable: keepout'):
        node.verify_live_maps()
    # 30 s: an idle Pi delivered /map to a fresh process in 1-6 s; under driving
    # load a 10 s wait failed (2026-09-30 table_02 legs).
    assert clock['now'] == 30.0


def test_live_map_reload_is_detected_even_with_unchanged_parameters():
    """A changed OccupancyGrid invalidates the session independently of YAML names."""
    node = route()
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 100, -1, 0]
    node._map_callback('map', grid)
    node.expected_grids['map'] = node.live_grids['map']
    grid.header.stamp.sec = 20
    node._map_callback('map', grid)
    assert not node.stop_requested
    grid.data = [100, 100, -1, 0]
    node._map_callback('map', grid)
    assert node.stop_requested
    assert 'live map changed' in node.map_mismatch


def test_localization_limits_use_squared_si_units():
    """The candidate confidence thresholds have explicit independent units."""
    contract = load_service_contract(
        Path(__file__).parent / 'fixtures/restaurant_service_contract_gated.yaml')
    assert contract['max_x_covariance_m2'] == pytest.approx(0.1 ** 2)
    assert contract['max_y_covariance_m2'] == pytest.approx(0.1 ** 2)
    assert contract['max_yaw_covariance_rad2'] == pytest.approx(math.radians(10.0) ** 2)
    # Intermediate legs stop between the recorded converged and unconverged states.
    assert contract['intermediate_max_x_covariance_m2'] == pytest.approx(0.2 ** 2)
    assert contract['intermediate_max_y_covariance_m2'] == pytest.approx(0.2 ** 2)
    assert contract['intermediate_max_yaw_covariance_rad2'] == pytest.approx(
        math.radians(15.0) ** 2)


@pytest.mark.parametrize('key,value', [
    ('intermediate_max_x_covariance_m2', None), ('intermediate_max_y_covariance_m2', 0.005),
    ('intermediate_max_yaw_covariance_rad2', math.inf),
    ('intermediate_max_x_covariance_m2', math.nan), ('intermediate_max_y_covariance_m2', True),
    ('intermediate_max_x_covariance_m2', '0.25')],
    ids=['missing', 'below_strict', 'inf', 'nan', 'bool', 'string'])
def test_intermediate_limits_never_below_strict_limits(tmp_path, key, value):
    """Reject an intermediate bound that is missing, non-finite or tighter than strict."""
    document = yaml.safe_load(
        (PACKAGE / 'config/restaurant_service_contract.yaml').read_text(encoding='utf-8'))
    if value is None:
        document.pop(key, None)
    else:
        document[key] = value
    path = tmp_path / 'contract.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(ValueError, match=key):
        load_service_contract(path)


@pytest.mark.parametrize('sequence,confirmed', [('steady', True), ('moving', False)])
def test_stationary_teaching_uses_fresh_unique_samples(monkeypatch, sequence, confirmed):
    """Exercise real hold logic instead of replacing its verdict."""
    node = route()
    clock = {'now': 10.0, 'step': 0}
    node.parking_motion_revision = 0
    node.parking_command = None
    node.parking_odom = None
    node.amcl_covariance = (0.01, 0.01)
    node._guard_failure = lambda **_: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=round(clock['now'] * 1e9)))
    transform = TransformStamped()
    transform.transform.translation.x = 2.0
    transform.transform.translation.y = 1.0
    transform.transform.rotation.w = 1.0
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = transform

    def spin(_node, timeout_sec):
        clock['step'] += 1
        clock['now'] = 10.0 + clock['step'] * 0.1
        stamp = transform.header.stamp
        stamp.sec = int(clock['now'])
        stamp.nanosec = round((clock['now'] - stamp.sec) * 1e9)
        node.parking_odom = (clock['now'], stamp, 0.0 if sequence == 'steady' else 0.2, 0.0)

    monkeypatch.setattr(
        'jdamr_cube_navigation.restaurant_service.time.monotonic', lambda: clock['now'])
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once', spin)
    if confirmed:
        pose, evidence = node.capture_stationary_pose(timeout_s=2.0)
        assert pose == (2.0, 1.0, 0.0)
        assert evidence['hold_s'] >= 1.0
        assert evidence['command_observed'] is False
        assert evidence['physical_accuracy'] == 'NOT_MEASURED'
    else:
        with pytest.raises(RuntimeError, match='stationary teaching unavailable'):
            node.capture_stationary_pose(timeout_s=2.0)


def test_arrival_evidence_retains_final_errors():
    """Persist the actual parking verdict in a parseable JSONL event."""
    node = route()
    node._route_event('parking_estimate_confirmed', 1, handle(),
                      confirmed=True, position_error_m=0.034,
                      yaw_error_rad=math.radians(2.0), hold_s=1.1)
    event = json.loads(node.result_stream.getvalue())
    assert event['table_id'] == 'table_01'
    assert event['position_error_m'] == 0.034
    assert event['physical_accuracy'] == 'NOT_MEASURED'
    assert node.confirmation['confirmed']


@pytest.mark.parametrize('discovery_range', ['SUBNET', 'LOCALHOST'])
@pytest.mark.parametrize('reverse', [False, True])
def test_service_launch_adds_parking_without_changing_costmaps(
        monkeypatch, discovery_range, reverse):
    """Construct launch parameters without starting nodes or moving a robot."""
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'service_launch', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(module, 'load_registry', lambda _: {
        'home': {'parking_direction': 'reverse' if reverse else 'forward'},
        'map': {'yaml_path': '/maps/new_base_room.yaml'},
        'keepout': {'yaml_path': '/maps/new_base_mask.yaml'}})
    context = LaunchContext()
    source = PACKAGE / 'config/new_base_nav2_params.yaml'
    context.launch_configurations.update({
        'registry': '/registry.yaml', 'params_file': str(source),
        'navigation_profile': 'new_base_candidate', 'use_sim_time': 'false',
        'discovery_range': discovery_range})
    original = yaml.safe_load(source.read_text())
    actions = module._configure(context)
    arguments = dict(actions[1].launch_arguments)
    generated = Path(arguments['params_file'])
    try:
        output = yaml.safe_load(generated.read_text())
        controller = output['controller_server']['ros__parameters']
        assert controller['Parking']['desired_linear_vel'] == 0.08
        assert controller['Parking']['use_collision_detection'] is False
        assert controller['parking_goal_checker']['xy_goal_tolerance'] == 0.05
        if reverse:
            assert controller['ParkingReverse']['allow_reversing'] is True
            assert controller['ParkingReverse']['use_rotate_to_heading'] is False
            assert controller['ParkingReverse']['use_collision_detection'] is False
            assert output['velocity_smoother']['ros__parameters']['min_velocity'][0] == -.08
            controller['controller_plugins'].remove('ParkingReverse')
            del controller['ParkingReverse']
            output['velocity_smoother']['ros__parameters']['min_velocity'][0] = 0.0
            # Transit is capped at the service speed, not the parking contract.
            smoother = output['velocity_smoother']['ros__parameters']
            assert smoother['max_velocity'][0] == 0.12
            smoother['max_velocity'][0] = original['velocity_smoother'][
                'ros__parameters']['max_velocity'][0]
            for key in ('desired_linear_vel', 'min_approach_linear_velocity',
                        'regulated_linear_scaling_min_speed'):
                assert controller['FollowPath'][key] <= 0.12
                controller['FollowPath'][key] = original['controller_server'][
                    'ros__parameters']['FollowPath'][key]
            controller['controller_plugins'].remove('GracefulReverse')
            del controller['GracefulReverse']
        else:
            assert 'ParkingReverse' not in controller
            assert output['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0.0
        controller['controller_plugins'].remove('Parking')
        controller['controller_plugins'].remove('GracefulParking')
        controller['goal_checker_plugins'].remove('parking_goal_checker')
        controller['goal_checker_plugins'].remove('alignment_goal_checker')
        controller['goal_checker_plugins'].remove('staging_position_checker')
        controller['goal_checker_plugins'].remove('dock_position_checker')
        controller['goal_checker_plugins'].remove('face_alignment_checker')
        controller['goal_checker_plugins'].remove('entry_heading_checker')
        del controller['Parking'], controller['parking_goal_checker']
        del controller['alignment_goal_checker'], controller['GracefulParking']
        del controller['staging_position_checker'], controller['dock_position_checker']
        del controller['face_alignment_checker'], controller['entry_heading_checker']
        # The service session adds the through-poses navigator with our own tree.
        navigator = output['bt_navigator']['ros__parameters']
        assert navigator['navigators'][-1] == 'navigate_through_poses'
        assert navigator.pop('default_nav_through_poses_bt_xml').endswith(
            'navigate_through_poses_transit.xml')
        assert navigator.pop('navigate_through_poses')['plugin'] == (
            'nav2_bt_navigator::NavigateThroughPosesNavigator')
        navigator['navigators'].pop()
        assert output == original
        assert arguments['map'] == '/maps/new_base_room.yaml'
        assert arguments['asset_registry'].perform(context) == '/registry.yaml'
        assert arguments['discovery_range'].perform(context) == discovery_range
        assert arguments['use_composition'].perform(context) == 'true'
        assert arguments['coordinated_startup'].perform(context) == 'true'
    finally:
        generated.unlink()


@pytest.mark.parametrize('override,expected', [(None, 'LOCALHOST'), ('SUBNET', 'SUBNET')])
def test_service_launch_defaults_to_physical_sensor_discovery(monkeypatch, override, expected):
    """Match onboard sensors by default and retain explicit network overrides."""
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'service_launch_defaults', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    description = module.generate_launch_description()
    declaration = next(action for action in description.entities
                       if isinstance(action, DeclareLaunchArgument)
                       and action.name == 'discovery_range')
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['discovery_range'] = override
    declaration.execute(context)
    assert context.launch_configurations['discovery_range'] == expected


def test_precision_launch_matches_box_contract_speed_and_search_capability(monkeypatch):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'precision_service_launch', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(
        PACKAGE if name == 'jdamr_cube_navigation' else PACKAGE.parent / name))
    monkeypatch.setattr(module, 'load_registry', lambda _: {
        'home': {'parking_direction': 'reverse'},
        'map': {'yaml_path': '/map.yaml'}, 'keepout': {'yaml_path': '/mask.yaml'}})
    context = LaunchContext()
    context.launch_configurations.update({
        'registry': '/registry.yaml',
        'params_file': str(PACKAGE / 'config/new_base_nav2_params.yaml'),
        'parking_contract': str(PACKAGE / 'config/box_parking_contract.yaml'),
        'precision_parking': 'true'})
    actions = module._configure(context)
    arguments = dict(actions[1].launch_arguments)
    generated = Path(arguments['params_file'])
    try:
        document = yaml.safe_load(generated.read_text())
        smoother = document['velocity_smoother']['ros__parameters']
        assert smoother['min_velocity'][0] == -.08
        # Transit and parking retain separate forward/reverse speed bounds.
        assert smoother['max_velocity'][0] == .12
        behavior = document['behavior_server']['ros__parameters']
        assert behavior['max_rotational_vel'] == .2
        assert behavior['min_rotational_vel'] == .1
        assert behavior['enable_stamped_cmd_vel'] is False
        assert arguments['enable_box_search'].perform(context) == 'true'
        assert document['collision_monitor']['ros__parameters']['source_timeout'] == 1.0
    finally:
        generated.unlink()


@pytest.mark.parametrize('search', [False, True])
def test_core_spin_opt_in_uses_guarded_command_route(monkeypatch, search):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'core_spin_launch', PACKAGE / 'launch/onboard_nav2_core.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(module, '_validate_new_base_params', lambda *a, **k: None)
    captured = []
    original = module.ComposableNode

    def capture(**kwargs):
        captured.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'ComposableNode', capture)
    context = LaunchContext()
    context.launch_configurations.update({
        'navigation_profile': 'new_base_candidate',
        'map': '/map.yaml', 'keepout_mask': '/mask.yaml',
        'params_file': str(PACKAGE / 'config/new_base_nav2_params.yaml'),
        'use_sim_time': 'false', 'autostart': 'true', 'use_composition': 'true',
        'precision_parking': 'true', 'enable_box_search': str(search).lower()})
    module._launch_navigation(context)
    behavior = next(value for value in captured if value['name'] == 'behavior_server')
    parameters = next(value for value in behavior['parameters']
                      if isinstance(value, dict) and 'behavior_plugins' in value)
    assert parameters['behavior_plugins'] == (['wait', 'spin'] if search else ['wait'])
    if search:
        assert parameters['spin']['plugin'] == 'nav2_behaviors::Spin'
    assert ('cmd_vel', 'cmd_vel_nav') in behavior['remappings']


def test_service_launch_defaults_to_composed_ordered_startup(monkeypatch):
    """Keep the verified composed stack while preserving ordered activation."""
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'service_launch_startup_defaults',
        PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda _: str(PACKAGE))
    description = module.generate_launch_description()
    declarations = {
        action.name: action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name in {'use_composition', 'coordinated_startup'}
    }
    context = LaunchContext()
    declarations['use_composition'].execute(context)
    declarations['coordinated_startup'].execute(context)
    assert context.launch_configurations['use_composition'] == 'true'
    assert context.launch_configurations['coordinated_startup'] == 'true'


def test_service_launch_keeps_box_observer_explicit():
    """Ordinary table navigation stays light; round trips opt into RGB-D."""
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location(
        'service_launch_box_observer',
        PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declaration = next(
        action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'use_box_observer')
    assert declaration.default_value[0].text == 'false'


PLANNED_POSE = {'id': 'main', 'x_m': 1.0, 'y_m': 2.0, 'yaw_rad': 0.4,
                'approach_offset_m': 0.5}


def _planned_route(end_xy, contract='config/parking_contract.yaml'):
    """Run the real plan_pose against a planner path ending at end_xy."""
    node = route()
    del node._pose
    node.parking_contract = load_parking_contract(PACKAGE / contract)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10 ** 12))
    result = ComputePathThroughPoses.Result()
    start, finish = PoseStamped(), PoseStamped()
    finish.pose.position.x, finish.pose.position.y = end_xy
    result.path.poses = [start, finish]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
    node.compute = Mock()
    node.compute.wait_for_server.return_value = True
    node.compute.send_goal_async.return_value = done(SimpleNamespace(
        accepted=True, get_result_async=lambda: done(wrapped)))
    return node


def test_t16a_plan_end_shortfall_is_not_success():
    """Reject an error-free plan that ends 0.15 m short of the final goal."""
    planned = _planned_route((0.85, 2.0)).plan_pose(PLANNED_POSE)
    assert planned['ok'] is False, f'HEADFAIL[T16a]: truncated plan accepted: {planned}'
    assert planned['reason'] == 'goal_not_reachable_within_tolerance'
    assert planned['end_error_m'] == pytest.approx(0.15)
    assert planned['end_tolerance_m'] == pytest.approx(0.05)


def test_t16b_nonfinite_plan_end_is_not_success():
    """Reject a non-finite plan end and keep the result strict JSON."""
    planned = _planned_route((math.nan, 2.0)).plan_pose(PLANNED_POSE)
    assert planned['ok'] is False, f'HEADFAIL[T16b]: non-finite plan end accepted: {planned}'
    assert planned['end_error_m'] is None
    json.dumps(planned, allow_nan=False)


def test_t16c_exact_plan_end_remains_success():
    """Keep the bench-proven exact plan end successful."""
    planned = _planned_route((1.0, 2.0)).plan_pose(PLANNED_POSE)
    assert planned['ok'] is True
    assert planned['reason'] == 'planned'


def test_t16d_alignment_plan_end_tolerance():
    """Judge intermediate alignment plans with the 0.05 m alignment checker."""
    assert 'end_tolerance_m' in inspect.signature(ServiceRoute.plan_pose).parameters, (
        'NEW[T16d]: plan_pose has no end_tolerance_m')
    contract = 'config/box_parking_contract.yaml'
    rejected = _planned_route((0.94, 2.0), contract).plan_pose(
        PLANNED_POSE, single=True, end_tolerance_m=0.05)
    accepted = _planned_route((0.96, 2.0), contract).plan_pose(
        PLANNED_POSE, single=True, end_tolerance_m=0.05)
    assert rejected['ok'] is False and rejected['end_error_m'] == pytest.approx(0.06)
    assert accepted['ok'] is True and accepted['end_error_m'] == pytest.approx(0.04)


def test_t18_end_point_failure_tries_alternate():
    """Try the registered alternate when the first goal cell is unreachable."""
    poses = [{'id': 'main'}, {'id': 'alternate'}, {'id': 'not_allowed'}]
    planner = Mock(side_effect=[
        {'ok': False, 'error_code': 0, 'reason': 'goal_not_reachable_within_tolerance',
         'end_error_m': 0.15, 'end_tolerance_m': 0.05},
        {'ok': True, 'error_code': 0, 'reason': 'planned'}])
    chosen, attempts = select_destination(poses, planner)
    assert planner.call_count == 2, 'HEADFAIL[T18]: alternate not attempted'
    assert chosen == poses[1] and len(attempts) == 2


def test_t14_parking_contract_option_is_home_only():
    """Accept --parking-contract for home only, never for table or serve commands."""
    base = ['service', 'home', '--registry', '/tmp/r.yaml', '--log', '/tmp/h.jsonl']
    try:
        args = parse_args([*base, '--parking-contract', '/tmp/box.yaml'])
    except SystemExit as error:
        raise AssertionError('NEW[T14]: home rejects --parking-contract') from error
    assert args.parking_contract == Path('/tmp/box.yaml')
    assert parse_args(base).parking_contract is None
    for command in (['go', '--table-id', 'table_01'], ['serve', '--table-id', 'table_01'],
                    ['roundtrip', '--table-id', 'table_01'], ['teach-home']):
        with pytest.raises(SystemExit):
            parse_args(['service', command[0], '--registry', '/tmp/r.yaml',
                        '--log', '/tmp/x.jsonl', *command[1:],
                        '--parking-contract', '/tmp/box.yaml'])


def test_t14b_home_main_loads_session_and_home_contracts(monkeypatch, tmp_path):
    """Use the session contract for home motion and keep the dock contract apart."""
    argv = ['service', 'home', '--registry', str(tmp_path / 'registry.yaml'),
            '--log', str(tmp_path / 'home.jsonl'), '--execute',
            '--parking-contract', str(PACKAGE / 'config/box_parking_contract.yaml')]
    try:
        parse_args(argv)
    except SystemExit as error:
        raise AssertionError('NEW[T14b]: home rejects --parking-contract') from error
    module = 'jdamr_cube_navigation.restaurant_service.'
    monkeypatch.setattr(module + 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(module + 'get_package_share_directory', lambda _name: str(PACKAGE))
    monkeypatch.setattr(module + 'rclpy.init', lambda **_kwargs: None)
    monkeypatch.setattr(module + 'rclpy.shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        def __init__(self, _registry, contract, _stream, home_contract=None):
            self.contract, self.home_contract = contract, home_contract
            self.go_home = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            created.append(self)

    monkeypatch.setattr(module + 'ServiceRoute', Route)
    main(argv)
    (node,) = created
    assert node.contract['xy_tolerance_m'] == 0.01
    assert node.home_contract['xy_tolerance_m'] == 0.05
    node.go_home.assert_called_once()


def test_t15_precision_params_match_box_contract_only(monkeypatch):
    """Match live precision parking parameters to the box contract, not home."""
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        'precision_contract_launch', PACKAGE / 'launch/restaurant_service.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(
        PACKAGE if name == 'jdamr_cube_navigation' else PACKAGE.parent / name))
    monkeypatch.setattr(module, 'load_registry', lambda _: {
        'home': {'parking_direction': 'reverse'},
        'map': {'yaml_path': '/map.yaml'}, 'keepout': {'yaml_path': '/mask.yaml'}})
    context = LaunchContext()
    context.launch_configurations.update({
        'registry': '/registry.yaml',
        'params_file': str(PACKAGE / 'config/new_base_nav2_params.yaml'),
        'parking_contract': str(PACKAGE / 'config/box_parking_contract.yaml'),
        'precision_parking': 'true'})
    generated = Path(dict(module._configure(context)[1].launch_arguments)['params_file'])
    try:
        controller = yaml.safe_load(generated.read_text())['controller_server'][
            'ros__parameters']
    finally:
        generated.unlink()
    flat = {}

    def flatten(prefix, value):
        if isinstance(value, dict):
            for key, item in value.items():
                flatten(f'{prefix}{key}.', item)
        else:
            flat[prefix[:-1]] = value

    flatten('', controller)
    node = route()
    node._read_parameters = Mock(side_effect=lambda remote, names: SimpleNamespace(values=[
        Parameter('value', value=flat.get(name)).get_parameter_value() for name in names]))
    node.parking_contract = load_parking_contract(
        PACKAGE / 'config/box_parking_contract.yaml')
    assert node._parking_parameters_ready(reverse=True)
    home = load_parking_contract(PACKAGE / 'config/parking_contract.yaml')
    node.parking_contract = home
    assert not node._parking_parameters_ready(reverse=True)
    assert flat['alignment_goal_checker.xy_goal_tolerance'] == home['xy_tolerance_m']
    assert flat['alignment_goal_checker.yaw_goal_tolerance'] == pytest.approx(
        home['yaw_tolerance_rad'])


def test_m2_home_help_forbids_use_in_front_of_a_box(capsys):
    """State that `home` must not start in front of a box; box_service escapes first."""
    with pytest.raises(SystemExit):
        parse_args(['service', 'home', '--help'])
    text = ' '.join(capsys.readouterr().out.split())
    assert '0.6 m' in text and 'box_service --return-home' in text, (
        f'REVIEW[M2]: home --help lacks the box-front rule: {text}')


@pytest.mark.parametrize('observed_deg,expected', [
    (38.2, True),    # 2026-09-30 water station: Spin at 0.7 rad/s overshot 30 deg by 8.2 deg
    (18.0, True),
    (44.0, True),
    (50.0, False),
    (10.0, False),
    (0.0, False),
])
def test_t38_search_rotation_tolerates_spin_overshoot_but_not_a_missing_turn(
        observed_deg, expected):
    """A 30 deg look-around step accepts +/-15 deg; a stalled or runaway turn fails."""
    node = route()
    node.spin_search = Mock()
    node.spin_search.send_goal_async.return_value = done(handle(GoalStatus.STATUS_SUCCEEDED, 0))
    node.capture_stationary_pose = Mock(side_effect=[
        ((1.0, 2.0, 0.0), {}), ((1.0, 2.0, math.radians(observed_deg)), {})])
    node._odom_pose = Mock(side_effect=[
        (1.0, 2.0, 0.0), (1.0, 2.0, math.radians(observed_deg))])
    node._search_parameters_ready = Mock(return_value=True)
    assert node.search_rotation(math.pi / 6) is expected


def test_t41_deployed_contract_disables_covariance_gates():
    """Operator decision 2026-09-30: no AMCL covariance stop in the deployed contract."""
    contract = load_service_contract(PACKAGE / 'config/restaurant_service_contract.yaml')
    for key in ('max_x_covariance_m2', 'max_y_covariance_m2', 'max_yaw_covariance_rad2',
                'intermediate_max_x_covariance_m2', 'intermediate_max_y_covariance_m2',
                'intermediate_max_yaw_covariance_rad2'):
        assert contract[key] >= 1.0e6


def test_engaged_emergency_stop_fails_every_guard_without_retry():
    """A latched base stop cancels goals and blocks departure; it is never retried."""
    node = route()
    # route() stubs both checks; use the real ones.
    del node._guard_failure, node._navigation_ready
    node.emergency_stop_engaged = True
    reason = node._guard_failure(False)
    assert reason == 'emergency stop engaged'
    assert not node._input_gap_recoverable(reason)
    assert not node._navigation_ready(require_fresh_amcl=False)


def test_unconfirmed_cancel_latches_the_base_emergency_stop(monkeypatch):
    """A goal that does not confirm its cancel within 5 s is stopped below Nav2."""
    clock = iter(range(0, 1000, 3))
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.time.monotonic',
                        lambda: next(clock))
    monkeypatch.setattr('jdamr_cube_navigation.restaurant_service.rclpy.spin_once',
                        lambda *args, **kwargs: None)
    node = route()
    node.emergency_stop_pub = Mock()
    running = Future()
    node.active_handle = SimpleNamespace(get_result_async=lambda: running)
    node.navigation_result = running
    node._cancel = Mock()
    assert node.finish_navigation() is False
    published = node.emergency_stop_pub.publish.call_args_list
    assert len(published) == 1 and published[0].args[0].data is True
    events = [json.loads(line)['event'] for line in node.result_stream.getvalue().splitlines()]
    assert events[-2:] == ['cancel_unconfirmed', 'emergency_stop_requested']


def _fast_clock(monkeypatch, step_s=5):
    clock = iter(range(0, 100000, step_s))
    monkeypatch.setattr('jdamr_cube_navigation.corridor_route.time.monotonic',
                        lambda: next(clock))
    monkeypatch.setattr('jdamr_cube_navigation.corridor_route.rclpy.spin_once',
                        lambda *args, **kwargs: None)


def _events(node):
    return [json.loads(line) for line in node.result_stream.getvalue().splitlines()]


def _blocked_attempts(node, blocked_tries, code=105):
    tries = []

    def attempt():
        tries.append(node._resume_waypoint_index)
        if len(tries) <= blocked_tries:
            node._retry_blocked_reason = f'path blocked: nav2 error {code}'
            node._retry_blocked_code = code
            return False
        return True
    return tries, attempt


def test_blocked_leg_is_held_then_retried(monkeypatch):
    """A person in the aisle: hold PATH_BLOCKED_WAIT_S, then retry the same leg."""
    _fast_clock(monkeypatch)
    node = route()
    node._front_clear = Mock(return_value=True)
    tries, attempt = _blocked_attempts(node, blocked_tries=2)
    assert node._run_with_input_recovery(attempt) is True
    assert len(tries) == 3
    waits = [e for e in _events(node) if e['event'] == 'path_blocked_wait']
    assert [e['wait'] for e in waits] == [1, 2]
    assert waits[0]['wait_s'] is None


def test_blocked_leg_gives_up_after_the_retry_budget(monkeypatch):
    _fast_clock(monkeypatch)
    node = route()
    node._front_clear = Mock(return_value=True)
    tries, attempt = _blocked_attempts(node, blocked_tries=99)
    assert node._run_with_input_recovery(attempt) is False
    assert len(tries) == corridor_route.PATH_BLOCKED_RETRIES + 1
    assert _events(node)[-1]['event'] == 'path_blocked_give_up'


def test_blocked_wait_ends_on_an_unrecoverable_guard(monkeypatch):
    """An emergency stop during the hold ends the leg instead of retrying it."""
    _fast_clock(monkeypatch)
    node = route()
    node._guard_failure = lambda *_: 'emergency stop engaged'
    tries, attempt = _blocked_attempts(node, blocked_tries=99)
    assert node._run_with_input_recovery(attempt) is False
    assert len(tries) == 1
    assert _events(node)[-1] == {**_events(node)[-1], 'event': 'path_blocked_wait_aborted',
                                 'reason': 'emergency stop engaged'}


@pytest.mark.parametrize('code, final, tagged', [
    (105, False, True), (104, False, True), (208, False, True), (306, False, True),
    (105, True, False), (101, False, False), (0, False, False)])
def test_only_blocking_nav2_codes_of_intermediate_legs_are_retried(code, final, tagged):
    node = route()
    node._retry_blocked_reason = None
    node._tag_blocked(code, final)
    assert (node._retry_blocked_reason is not None) is tagged


@pytest.mark.parametrize('event,reason', [
    ('interrupted', 'emergency stop engaged'),
    ('path_blocked_give_up', 'path blocked: nav2 error 105'),
    ('battery_return', 'battery below the departure reserve'),
    ('failed', 'scan stale: age=3.0s limit=2.5s'),
])
def test_failure_evidence_remains_without_a_separate_operator_call(event, reason):
    node = route()
    node.emit(event, reason=reason)
    events = _events(node)
    assert events[-1]['event'] == event
    assert events[-1]['reason'] == reason
    assert not any(item['event'] == 'operator_call' for item in events)


def test_operator_link_blocks_the_next_goal_once_the_heartbeat_is_old(tmp_path):
    node = route()
    assert node._operator_link_ready()          # no heartbeat configured: not followed
    heartbeat = tmp_path / 'run.heartbeat'
    node.operator_heartbeat = heartbeat
    assert not node._operator_link_ready()      # never written
    heartbeat.touch()
    assert node._operator_link_ready()
    old = time.time() - restaurant_service.OPERATOR_LINK_TIMEOUT_S - 5.0
    os.utime(heartbeat, (old, old))
    assert not node._operator_link_ready()
    blocked = _events(node)[-1]
    assert (blocked['event'], blocked['reason']) == ('departure_blocked', 'operator_link_lost')


@pytest.mark.parametrize('voltage_v, due', [(10.7, True), (10.8, False), (11.6, False),
                                            (None, True), (float('nan'), True)])
def test_battery_return_between_stops(voltage_v, due):
    node = route()
    node.battery_voltage = voltage_v
    assert node.battery_return_due() is due
    assert any(e['event'] == 'battery_return' for e in _events(node)) is due


def test_no_path_hold_ends_as_soon_as_a_plan_exists(monkeypatch):
    """A person who steps aside after 10 s: the 308 hold re-plans and goes on."""
    _fast_clock(monkeypatch, step_s=1)
    node = route()
    node._path_open = Mock(side_effect=[False, True])
    node._escape_blocked = Mock()
    tries, attempt = _blocked_attempts(node, blocked_tries=1, code=308)
    assert node._run_with_input_recovery(attempt) is True
    assert len(tries) == 2
    node._escape_blocked.assert_not_called()      # no path: nothing to back away from
    cleared = [e for e in _events(node) if e['event'] == 'path_blocked_cleared']
    assert len(cleared) == 1 and cleared[0]['held_s'] < corridor_route.PATH_BLOCKED_WAIT_S


def test_controller_stop_waits_without_backing_away(monkeypatch):
    _fast_clock(monkeypatch)
    node = route()
    node._path_open = Mock(return_value=True)
    node._front_clear = Mock(side_effect=[False, True, True, False, True, True])
    node._escape_blocked = Mock(return_value=True)
    tries, attempt = _blocked_attempts(node, blocked_tries=2, code=105)
    assert node._run_with_input_recovery(attempt) is True
    node._escape_blocked.assert_not_called()
    node._path_open.assert_not_called()           # a 105 hold runs its full time


def _scan(ranges):
    return SimpleNamespace(ranges=ranges, angle_min=0.0, angle_increment=math.pi / 2,
                           range_min=0.28, range_max=12.0)


STEPPED = [(0.085, 0.245), (0.053, 0.245), (0.053, 0.29), (-0.053, 0.29), (-0.053, 0.245),
           (-0.295, 0.245), (-0.295, -0.245), (-0.053, -0.245), (-0.053, -0.29),
           (0.053, -0.29), (0.053, -0.245), (0.085, -0.245)]


@pytest.mark.parametrize('ranges, ahead', [
    # Laser 1 cm behind the axle, turned 180 deg; beam 2 points ahead of the base.
    ([math.inf, math.inf, 0.31, math.inf], True),      # 0.30 m ahead: in the sweep
    ([math.inf, math.inf, 0.50, math.inf], False),     # beyond 0.085 + 0.30 m
    ([math.inf, 0.29, math.inf, math.inf], False),     # beside the base (desk leg)
    ([math.inf, math.inf, 0.2, math.inf], False),      # inside the LiDAR blind range
])
def test_only_an_object_ahead_counts_as_blocked_ahead(ranges, ahead):
    from jdamr_cube_navigation.reverse_parking import straight_sweep_hit
    assert straight_sweep_hit(_scan(ranges), (-0.01, 0.0, math.pi), STEPPED, 0.30) is ahead


def test_controller_stop_hold_ends_once_the_band_ahead_stays_empty(monkeypatch):
    """The blocker steps aside: two empty checks in a row end the 105 hold early."""
    _fast_clock(monkeypatch, step_s=1)
    node = route()
    node._escape_blocked = Mock(return_value=False)
    node._front_clear = Mock(side_effect=[False, True, False, True, True])
    tries, attempt = _blocked_attempts(node, blocked_tries=1, code=105)
    assert node._run_with_input_recovery(attempt) is True
    assert len(tries) == 2 and node._front_clear.call_count == 5
    cleared = [e for e in _events(node) if e['event'] == 'path_blocked_cleared']
    assert len(cleared) == 1 and cleared[0]['held_s'] < corridor_route.PATH_BLOCKED_WAIT_S


@pytest.mark.parametrize('ranges, needed', [
    # Laser 1 cm behind the axle, turned 180 deg: beam 0 points back, 1 right, 2 ahead.
    ([math.inf, 0.29, math.inf, math.inf], math.sqrt(0.45 ** 2 - 0.29 ** 2) - 0.01),
    ([0.30, math.inf, math.inf, math.inf], 0.45 - 0.31),   # 0.31 m behind the axle
    ([math.inf, math.inf, 0.30, math.inf], 0.0),           # ahead of the axle
    ([math.inf, math.inf, math.inf, math.inf], 0.0),
])
def test_rear_swing_clearance_frees_the_turn_circle(ranges, needed):
    from jdamr_cube_navigation.reverse_parking import rear_swing_clearance
    assert rear_swing_clearance(_scan(ranges), (-0.01, 0.0, math.pi), 0.45) == pytest.approx(
        needed)


SNAPSHOT = ('scan', (-0.01, 0.0, math.pi), 'outline', [])


def test_side_rear_obstacle_drives_forward_out_of_the_turn_circle():
    """11:09: a leg beside the rear corner holds every turn; the base drives on first."""
    node = route()
    node._scan_in_base = Mock(return_value=SNAPSHOT)
    node._footprint_intrusion = Mock(return_value=None)
    node._blocked_ahead = Mock(side_effect=[False, False])
    node._rear_swing_need = Mock(return_value=0.147)
    node._drive_straight = Mock(return_value=True)
    assert node._escape_blocked() is True
    distance, event = node._drive_straight.call_args.args
    assert distance == pytest.approx(0.177) and event == 'path_blocked_escape_forward'


def test_escape_backs_off_when_blocked_ahead_and_holds_when_nothing_is_near():
    node = route()
    node._scan_in_base = Mock(return_value=SNAPSHOT)
    node._footprint_intrusion = Mock(return_value=None)
    node._blocked_ahead = Mock(return_value=True)
    node._blocked_behind = Mock(return_value=False)
    node._drive_straight = Mock(return_value=True)
    assert node._escape_blocked() is True
    assert node._drive_straight.call_args.args == (
        -restaurant_service.PATH_BLOCKED_BACKOFF_M, 'path_blocked_back_off')
    assert node._blocked_behind.call_args.kwargs['band_m'] == pytest.approx(
        restaurant_service.PATH_BLOCKED_BACKOFF_M + 0.05)
    node = route()
    node._scan_in_base = Mock(return_value=SNAPSHOT)
    node._footprint_intrusion = Mock(return_value=None)
    node._blocked_ahead = Mock(return_value=False)
    node._rear_swing_need = Mock(return_value=0.0)
    node._drive_straight = Mock()
    assert node._escape_blocked() is False
    node._drive_straight.assert_not_called()
    node = route()
    node._scan_in_base = Mock(return_value=None)          # no fresh scan: no move
    node._drive_straight = Mock()
    node._retrace = Mock()
    assert node._escape_blocked() is False
    node._drive_straight.assert_not_called()
    node._retrace.assert_not_called()


def test_forward_escape_needs_the_band_ahead_clear_over_its_length():
    node = route()
    node._scan_in_base = Mock(return_value=SNAPSHOT)
    node._footprint_intrusion = Mock(return_value=None)
    node._blocked_ahead = Mock(side_effect=[False, True])
    node._rear_swing_need = Mock(return_value=0.30)
    node._drive_straight = Mock()
    assert node._escape_blocked() is False
    node._drive_straight.assert_not_called()
    assert node._blocked_ahead.call_args.kwargs['band_m'] == pytest.approx(0.30 + 0.03 + 0.05)


def test_rpp_transit_switches_only_the_transit_and_staging_trees(monkeypatch):
    monkeypatch.setattr(restaurant_service, 'get_package_share_directory',
                        lambda _: str(PACKAGE))
    route = object.__new__(restaurant_service.ServiceRoute)
    route.through_behavior_tree, route.staging_behavior_tree = 'rpp_transit', 'rpp_staging'
    route.alignment_behavior_tree = 'alignment'
    route.final_approach_controller, route.dock_leg_controller = 'Parking', 'ParkingReverse'
    route.use_rpp_transit()
    for tree in (route.through_behavior_tree, route.staging_behavior_tree):
        assert Path(tree).is_file() and 'controller_id="FollowPath"' in Path(tree).read_text()
    assert route.alignment_behavior_tree == 'alignment'
    assert (route.final_approach_controller, route.dock_leg_controller) == (
        'Parking', 'ParkingReverse')


@pytest.mark.parametrize('points, distance, hit', [
    # 13:48 (run 134626): a return 5 mm beside the rear corner, outside the frame.
    ([(-0.37, -0.25)] * 3, -0.15, False),
    ([(-0.37, -0.20)], -0.15, True),                     # behind the frame
    ([(-0.39, 0.0)], -0.10, True),                       # 0.095 m behind the rear edge
    ([(-0.40, 0.0)], -0.10, False),                      # 0.105 m behind
    ([(-0.27, -0.243)], 0.22, False),                    # 11:09 leg in the rear padding
    ([(0.07, 0.0)], 0.10, True),                         # inside the front padding
    ([(0.07, 0.0)], -0.10, False),
    ([(0.10, 0.27)], 0.10, True),                        # ahead of the left wheel
    ([(0.10, 0.27)], -0.10, False),
])
def test_straight_sweep_follows_the_stepped_footprint(points, distance, hit):
    from jdamr_cube_navigation.reverse_parking import straight_sweep_hit
    assert straight_sweep_hit(_points_scan(points), (0.0, 0.0, 0.0), STEPPED, distance) is hit


def _points_scan(points):
    """Build a 0.5 deg scan in the base frame (laser at the origin) with returns at points."""
    ranges = [math.inf] * 720
    for x_m, y_m in points:
        ranges[int(round((math.atan2(y_m, x_m) + math.pi) / math.radians(0.5))) % 720] = (
            math.hypot(x_m, y_m))
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=math.radians(0.5),
                           range_min=0.05, range_max=12.0)


OUTLINE = [(-0.295, -0.29), (0.085, -0.29), (0.085, 0.29), (-0.295, 0.29)]
# 11:09 (bag table_02_20261002_110821): returns 0-7 mm inside the right rear corner.
LEG_1109 = [(-0.273, -0.241), (-0.27, -0.243), (-0.266, -0.244), (-0.262, -0.244),
            (-0.258, -0.245), (-0.25, -0.26)]


@pytest.mark.parametrize('points, half', [
    (LEG_1109, 'rear'),
    ([(0.06, -0.27), (0.065, -0.272), (0.07, -0.274)], 'front'),
    (LEG_1109 + [(0.06, -0.27), (0.065, -0.272), (0.07, -0.274)], 'both'),
    (LEG_1109[:2], None),                                  # below the monitor's 3 points
    ([(-0.27, -0.40), (-0.26, -0.41), (-0.25, -0.42)], None),   # beside, outside
])
def test_outline_intrusion_names_the_half_the_monitor_holds(points, half):
    from jdamr_cube_navigation.reverse_parking import outline_intrusion
    assert outline_intrusion(_points_scan(points), (0.0, 0.0, 0.0), OUTLINE, 3) == half


def test_zero_turn_drive_commands_no_rotation_and_stops_at_the_distance(monkeypatch):
    node = route()
    published = []
    node.escape_velocity = SimpleNamespace(publish=lambda m: published.append(
        (m.linear.x, m.angular.z)))
    poses = iter([(1.0 + 0.02 * k * math.cos(0.5), 2.0 + 0.02 * k * math.sin(0.5))
                  for k in range(100)])

    def lookup(*_args, **_kwargs):
        x_m, y_m = next(poses)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x_m, y=y_m),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.25), w=math.cos(0.25))))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    travelled = node._drive_zero_turn(0.10)
    assert 0.10 - 1e-9 <= travelled <= 0.12 + 1e-9    # stops on the first odom step past it
    assert all(w == 0.0 for _v, w in published)
    # Motion commands, then only zero commands: the stop burst and the settle
    # window that keeps commanding zero while the smoother decelerates.
    first_stop = published.index((0.0, 0.0))
    assert {v for v, _w in published[:first_stop]} == {
        restaurant_service.STRAIGHT_ESCAPE_SPEED_MPS}
    assert published[first_stop:] == [(0.0, 0.0)] * (len(published) - first_stop)
    assert len(published) - first_stop >= 5


@pytest.mark.parametrize('distance_m, expected_speed, approach_time_s', [
    (0.022, 0.01, None), (0.08, 0.02, None), (0.30, 0.05, None),
    (0.09990421312116261, 0.05, 0.5),
])
@pytest.mark.parametrize('setup_delay_s', [0.0, 11.0])
def test_precision_zero_turn_uses_live_motion_and_returns_settled_distance(
        monkeypatch, distance_m, expected_speed, approach_time_s, setup_delay_s):
    node = route()
    clock = [0.0]
    position = [0.0]
    velocity = [0.0]
    requested = [0.0]
    published = []
    first_stop_position = []
    node.latest_motion = (-1.0, 0.0, 0.0)

    def publish(message):
        requested[0] = message.linear.x
        published.append((message.linear.x, message.angular.z))
        if message.linear.x == 0.0 and not first_stop_position:
            first_stop_position.append(position[0])

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        change = restaurant_service.STRAIGHT_DECEL_MPS2 * 0.02
        velocity[0] += max(-change, min(change, requested[0] - velocity[0]))
        position[0] += velocity[0] * 0.02
        node.latest_motion = (clock[0], abs(velocity[0]), 0.0)

    def lookup(*_args, **_kwargs):
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    def create_publisher(*_args, **_kwargs):
        clock[0] += setup_delay_s
        return SimpleNamespace(publish=publish)

    node.create_publisher = create_publisher
    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)

    approach_options = {} if approach_time_s is None else {'approach_time_s': approach_time_s}
    travelled = node._drive_zero_turn(distance_m, precision=True, **approach_options)

    assert {round(v, 6) for v, _w in published if v} == {round(expected_speed, 6)}
    assert published[-5:] == [(0.0, 0.0)] * 5
    assert velocity[0] == pytest.approx(0.0)
    assert first_stop_position[0] < distance_m
    assert position[0] >= first_stop_position[0]
    assert travelled == pytest.approx(position[0])
    assert travelled <= distance_m + 0.003
    assert node._last_direct_motion_failure is None
    finished = [event for event in _events(node)
                if event['event'] == 'precision_straight_finished']
    assert finished[-1]['travelled_m'] == pytest.approx(travelled)
    assert finished[-1]['speed_mps'] == pytest.approx(expected_speed)


def test_precision_zero_turn_fails_without_fresh_stopped_odometry(monkeypatch):
    node = route()
    clock = [0.0]
    node.latest_motion = (-1.0, 0.0, 0.0)
    node.escape_velocity = SimpleNamespace(publish=lambda _message: None)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_args, **_kwargs:
                                      SimpleNamespace(transform=SimpleNamespace(
                                          translation=SimpleNamespace(
                                              x=clock[0] * 0.01, y=0.0),
                                          rotation=SimpleNamespace(
                                              x=0.0, y=0.0, z=0.0, w=1.0))))

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        node.latest_motion = (clock[0], 0.02, 0.0)

    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)

    assert node._drive_zero_turn(0.022, precision=True) is None
    stopped = [e for e in _events(node) if e['event'] == 'straight_drive_stopped']
    assert stopped[-1]['reason'] == 'odometry did not confirm stop'
    assert stopped[-1]['fresh_odom_samples'] > 0
    assert stopped[-1]['last_odom_linear_mps'] == pytest.approx(.02)


@pytest.mark.parametrize('live_heading', [False, True])
@pytest.mark.parametrize('failure', [None, 'update', 'guard', 'stop', 'link', 'corridor'])
def test_live_precision_approach_updates_distance_without_intermediate_stop(
        monkeypatch, failure, live_heading):
    node = route()
    clock, position, requested = [0.0], [0.0], [0.0]
    commands = []

    def publish(message):
        requested[0] = message.linear.x
        commands.append((message.linear.x, message.angular.z))

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        position[0] += requested[0] * 0.02
        node.latest_motion = (clock[0], requested[0], 0.0)
        if failure == 'stop' and position[0] > 0.01:
            node.stop_requested = True

    def update(actual):
        if failure == 'corridor' and not live_heading:
            raise RuntimeError('table approach static corridor blocked')
        if failure == 'update' and actual[0] > 0.01:
            raise RuntimeError('stale depth')
        # A later frame during settle must not move an already reached goal.
        result = {'remaining_m': 0.10 if commands and commands[-1][0] == 0.0
                  else 0.04 - actual[0]}
        if live_heading:
            result['heading_rad'] = 0.001
            if failure == 'corridor':
                result['validate_control'] = Mock(side_effect=RuntimeError(
                    'table approach static corridor blocked'))
        return result

    node.latest_motion = (-1.0, 0.0, 0.0)
    if failure == 'guard':
        node._guard_failure = lambda *_: 'obstacle' if position[0] > 0.01 else None
    if failure == 'link':
        node._operator_link_ready = lambda: position[0] <= 0.01
    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    travelled = node._drive_zero_turn(
        0.022, precision=True, hold_heading=True, target_update=update)
    if failure:
        assert travelled is None
        assert position[0] < 0.02
    else:
        assert travelled == pytest.approx(0.04, abs=0.003)
        assert position[0] > 0.035
    first_zero = next(i for i, command in enumerate(commands) if command[0] == 0.0)
    assert all(command == (0.0, 0.0) for command in commands[first_zero:])


@pytest.mark.parametrize('new_pose', [False, True])
def test_stationary_amcl_refresh_requires_a_new_scan_pose_without_reset(monkeypatch, new_pose):
    node = route()
    clock = [10.0]
    node.latest_motion = (clock[0], 0.0, 0.0)
    node.amcl_seen = 0.0
    node._operator_link_ready = Mock(return_value=True)
    client = Mock()
    client.call_async.return_value = done(SimpleNamespace())
    node.create_client = Mock(return_value=client)
    node.destroy_client = Mock()
    node._escape_publisher = Mock()

    def spin(*_args, **_kwargs):
        clock[0] += 0.05
        if new_pose:
            node.amcl_seen = clock[0]

    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    assert node._refresh_amcl_at_rest() is new_pose
    assert node.create_client.call_args.args[1] == '/request_nomotion_update'
    node.destroy_client.assert_called_once_with(client)
    node._escape_publisher.assert_not_called()


def test_live_control_validates_filtered_heading_and_uses_shared_speed_law(monkeypatch):
    node = route()
    clock = [10.0]
    pose = (0.0, 0.0, 0.0)
    validated, commands = [], []
    observation_heading = [0.1]
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=pose[0], y=pose[1]),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))

    def validate(actual, heading):
        validated.append((actual, heading))

    def update(_actual):
        return {'remaining_m': 0.3, 'heading_rad': observation_heading[0],
                'validate_control': validate}

    def motion(command, _reached, measure, *_a, **_k):
        measure()
        commands.append((command.linear.x, command.angular.z))
        observation_heading[0] = -0.1
        clock[0] += 0.35
        measure()
        commands.append((command.linear.x, command.angular.z))
        return 0.0

    node._direct_motion = motion
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    # The tiny initial request must not leave a hidden .01 cap in a revised .3m approach.
    node._drive_zero_turn(0.01, precision=True, hold_heading=True, target_update=update)
    expected = 0.1 + (1.0 - math.exp(-1.0)) * -0.2
    assert [actual for actual, _heading in validated] == [pose, pose]
    assert [value for _actual, value in validated] == pytest.approx([0.1, expected])
    assert commands[0] == pytest.approx(restaurant_service.face_approach_command(0.3, 0.1))
    assert commands[1] == pytest.approx(restaurant_service.face_approach_command(0.3, expected))


@pytest.mark.parametrize('heading,gap_m,offset_deg,allow_near', [
    (0.0, 0.15, 12.0, False), (0.0, 0.45, -12.0, False),
    (math.pi - 0.03, 0.15, 8.0, False), (-math.pi + 0.03, 0.45, -8.0, False),
    (0.0, .033676181083231255, .80565973, True), (0.0, .014, -.80565973, True),
])
@pytest.mark.parametrize('direction,continuous_table', [(-1., False), (1., False), (1., True)])
def test_live_face_approach_aligns_while_advancing_and_stops_at_both_corners(
        monkeypatch, heading, gap_m, offset_deg, allow_near, direction, continuous_table):
    node = route()
    clock, commands, velocity = [0.0], [], [0.0, 0.0]
    pose = [0.0, 0.0, heading + math.radians(offset_deg)]
    front, half_width = (0.065 if direction > 0 else 0.275), 0.225
    normal = (-direction * math.cos(heading), -direction * math.sin(heading))
    initial_lead = (
        front * math.cos(math.radians(offset_deg))
        + half_width * abs(math.sin(math.radians(offset_deg))))
    face_distance = initial_lead + gap_m
    face = (direction * face_distance * math.cos(heading),
            direction * face_distance * math.sin(heading))

    def publish(message):
        velocity[:] = [message.linear.x, message.angular.z]
        commands.append(tuple(velocity))

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        pose[0] += velocity[0] * math.cos(pose[2]) * 0.02
        pose[1] += velocity[0] * math.sin(pose[2]) * 0.02
        pose[2] += velocity[1] * 0.02
        node.latest_motion = (clock[0], *velocity)

    def update(actual):
        residual = math.atan2(math.sin(heading - actual[2]),
                              math.cos(heading - actual[2]))
        lead = front * math.cos(residual) + half_width * abs(math.sin(residual))
        nearest = sum((actual[i] - face[i]) * normal[i] for i in (0, 1)) - lead
        return {'remaining_m': (nearest - 0.01) / math.cos(residual),
                'heading_rad': heading, 'allow_near_alignment': allow_near,
                'surface_gap_m': nearest}

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=pose[0], y=pose[1]),
            rotation=SimpleNamespace(x=0.0, y=0.0,
                                     z=math.sin(pose[2] / 2.0), w=math.cos(pose[2] / 2.0)))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)

    assert node._drive_zero_turn(
        direction * (gap_m - 0.01), precision=True,
        hold_heading=True, target_update=update, continuous_table=continuous_table) is not None
    error = math.atan2(math.sin(pose[2] - heading), math.cos(pose[2] - heading))
    front_center = (pose[0] + direction * front * math.cos(pose[2]),
                    pose[1] + direction * front * math.sin(pose[2]))
    gaps = [sum((front_center[i] + side * half_width * (-math.sin(pose[2]),
                                                        math.cos(pose[2]))[i]
                 - face[i]) * normal[i] for i in (0, 1)) for side in (-1, 1)]
    assert all(0.007 <= gap <= 0.013 for gap in gaps)
    assert abs(error) <= math.radians(0.3)
    moving = [command for command in commands if command != (0.0, 0.0)]
    assert moving and any(direction * linear > 0.0 for linear, _angular in moving)
    assert all(direction * linear >= 0.0 if allow_near or continuous_table
               else direction * linear > 0.0
               for linear, _angular in moving)
    assert max(abs(linear) for linear, _angular in moving) <= (0.1 if continuous_table else 0.05)
    angular = [w for _v, w in moving if w]
    assert all(w * offset_deg < 0.0 for w in angular)
    first_zero = commands.index((0.0, 0.0))
    assert all(command == (0.0, 0.0) for command in commands[first_zero:])


def test_lateral_curve_passes_face_geometry_separately_without_a_false_alignment_stop(
        monkeypatch):
    node = route()
    clock, x, velocity, commands = [0.], [0.], [0.], []
    command_law = Mock(wraps=restaurant_service.face_approach_command)

    def publish(message):
        velocity[0] = message.linear.x
        commands.append((message.linear.x, message.angular.z))

    def spin(*_args, **_kwargs):
        clock[0] += .02
        x[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.)

    def update(actual):
        return {'remaining_m': .15 - actual[0], 'surface_gap_m': .16 - actual[0],
                'heading_rad': .05, 'face_heading_rad': 0.,
                'lateral_tracking': True, 'allow_near_alignment': True}

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    monkeypatch.setattr(restaurant_service, 'face_approach_command', command_law)

    assert node._drive_zero_turn(
        .15, precision=True, hold_heading=True,
        target_update=update, continuous_table=True) is not None
    assert command_law.call_count > 0
    assert all(call.kwargs['lateral_tracking'] is True
               and call.kwargs['face_error_rad'] == 0.
               for call in command_law.call_args_list)
    first_stop = commands.index((0., 0.))
    assert all(v > 0. for v, _w in commands[:first_stop])
    assert any(w > 0. for _v, w in commands[:first_stop])


def test_live_lateral_approach_replays_water_offset_without_precision_band_deadlock(monkeypatch):
    node = route()
    clock, pose = [0.], [0., -.055285, -.007085]
    velocity, commands, gaps = [0., 0.], [], []
    face_x = .3475

    def publish(message):
        velocity[:] = [message.linear.x, message.angular.z]
        commands.append(tuple(velocity))

    def spin(*_args, **_kwargs):
        clock[0] += .02
        pose[0] += velocity[0] * math.cos(pose[2]) * .02
        pose[1] += velocity[0] * math.sin(pose[2]) * .02
        pose[2] += velocity[1] * .02
        node.latest_motion = (clock[0], *velocity)

    def update(actual):
        error = -actual[2]
        center_gap = face_x - actual[0] - .065 * math.cos(error)
        gap = center_gap - .225 * abs(math.sin(error))
        gaps.append(gap)
        return {
            'remaining_m': (center_gap - .01) / math.cos(error),
            'heading_rad': 0., 'face_heading_rad': 0., 'surface_gap_m': gap,
            'lateral_tracking': True, 'lateral_error_m': -actual[1],
            'lateral_tolerance_m': .005, 'target_gap_m': .01,
            'allow_near_alignment': True,
        }

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(
        lookup_transform=lambda *_a, **_k: SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=pose[0], y=pose[1]),
            rotation=SimpleNamespace(x=0., y=0., z=math.sin(pose[2] / 2),
                                     w=math.cos(pose[2] / 2)))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)

    assert node._drive_zero_turn(
        .2824, precision=True, hold_heading=True, target_update=update,
        continuous_table=True) is not None
    assert node._last_direct_motion_failure is None
    assert abs(pose[1]) <= .005
    assert abs(pose[2]) <= math.radians(.3)
    assert .007 <= gaps[-1] <= .014
    assert min(gaps) > 0.
    assert any(v > 0. and abs(w) > 0. for v, w in commands)


def test_near_face_flip_holds_motion_until_heading_filter_agrees(monkeypatch):
    node = route()
    clock = [0.]
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: transform)
    update = Mock(side_effect=[
        {'remaining_m': .005, 'heading_rad': .01, 'allow_near_alignment': True},
        {'remaining_m': .005, 'heading_rad': -.01, 'allow_near_alignment': True},
    ])
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])

    def motion(command, done, measure, *_args, **_kwargs):
        measure()
        assert command.linear.x == 0.
        assert command.angular.z > 0.
        assert not done(0.)
        clock[0] += .02
        measure()
        assert command.linear.x == command.angular.z == 0.
        assert not done(0.)
        return 0.

    node._direct_motion = motion
    assert node._drive_zero_turn(.005, precision=True, hold_heading=True,
                                 target_update=update) is None


@pytest.mark.parametrize('continuous_table', [False, True])
def test_live_face_approach_tapers_before_slow_base_coast(monkeypatch, continuous_table):
    node = route()
    clock, position = [0.0], [0.0]
    requested, velocity = [0.0], [0.0]
    target_m = 0.4
    # Stress a plant that coasts more slowly than the configured command smoother.
    # This is a synthetic regression condition, not measured chassis calibration.
    simulated_deceleration_mps2 = 0.08

    def publish(message):
        requested[0] = message.linear.x

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        acceleration_mps2 = (0.35 if requested[0] >= velocity[0]
                             else simulated_deceleration_mps2)
        change_mps = acceleration_mps2 * 0.02
        velocity[0] += max(-change_mps, min(change_mps, requested[0] - velocity[0]))
        position[0] += velocity[0] * 0.02
        node.latest_motion = (clock[0], velocity[0], 0.0)

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
                                      SimpleNamespace(transform=SimpleNamespace(
                                          translation=SimpleNamespace(
                                              x=position[0], y=0.0),
                                          rotation=SimpleNamespace(
                                              x=0.0, y=0.0, z=0.0, w=1.0))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)

    def update(actual):
        return {'remaining_m': target_m - actual[0], 'heading_rad': 0.0,
                'surface_gap_m': target_m - actual[0] + .01}

    assert node._drive_zero_turn(
        target_m, precision=True, hold_heading=True, target_update=update,
        continuous_table=continuous_table) is not None
    assert target_m - 0.003 <= position[0] <= target_m + 0.003


@pytest.mark.parametrize('drift_rad, correction_sign', [
    (0.01, -1.0),
    (-0.01, 1.0),
])
def test_precision_heading_hold_corrects_both_drift_directions(
        monkeypatch, drift_rad, correction_sign):
    node = route()
    clock = [0.0]
    published = []
    poses = iter([
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.001, 0.0, drift_rad),
        (0.002, 0.0, drift_rad),
        (0.003, 0.0, drift_rad),
        (0.02, 0.0, drift_rad),
    ] + [(0.02, 0.0, 0.0)] * 100)

    def lookup(*_args, **_kwargs):
        x_m, y_m, yaw = next(poses)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x_m, y=y_m),
            rotation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))))

    node.escape_velocity = SimpleNamespace(publish=lambda message: published.append(
        (message.linear.x, message.angular.z)))
    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once',
                        lambda *a, **k: clock.__setitem__(0, clock[0] + 0.02))

    node._drive_zero_turn(0.022, hold_heading=True)

    corrections = [angular for linear, angular in published if linear and angular]
    assert corrections
    assert math.copysign(1.0, corrections[0]) == correction_sign
    assert all(abs(value) <= restaurant_service.PRECISION_HEADING_MAX_RADPS
               for value in corrections)


def test_precision_heading_hold_wraps_error_across_pi(monkeypatch):
    node = route()
    clock = [0.0]
    published = []
    start_yaw = math.pi - 0.01
    drifted_yaw = -math.pi + 0.01
    poses = iter([
        (0.0, 0.0, start_yaw),
        (0.0, 0.0, start_yaw),
        (0.001, 0.0, drifted_yaw),
        (0.002, 0.0, drifted_yaw),
        (0.003, 0.0, drifted_yaw),
        (0.02, 0.0, drifted_yaw),
    ] + [(0.02, 0.0, start_yaw)] * 100)

    def lookup(*_args, **_kwargs):
        x_m, y_m, yaw = next(poses)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x_m, y=y_m),
            rotation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))))

    node.escape_velocity = SimpleNamespace(publish=lambda message: published.append(
        (message.linear.x, message.angular.z)))
    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once',
                        lambda *a, **k: clock.__setitem__(0, clock[0] + 0.02))

    node._drive_zero_turn(0.022, hold_heading=True)

    corrections = [angular for linear, angular in published if linear and angular]
    assert corrections[0] == pytest.approx(-0.02)


def test_heading_hold_drops_angular_command_when_tf_disappears(monkeypatch):
    node = route()
    clock = [1.0]
    published = []
    calls = [0]

    def lookup(*_args, **_kwargs):
        calls[0] += 1
        if calls[0] >= 4:
            raise restaurant_service.TransformException('TF unavailable')
        yaw = 0.0 if calls[0] == 1 else 0.02
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.001 * calls[0], y=0.0),
            rotation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))))

    node.escape_velocity = SimpleNamespace(publish=lambda message: published.append(
        (message.linear.x, message.angular.z)))
    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    # Raw odometry can remain fresh while only the odom->base TF lookup fails.
    node._guard_failure = Mock(return_value=None)
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once',
                        lambda *a, **k: clock.__setitem__(0, clock[0] + 0.02))

    travelled = node._drive_zero_turn(0.022, hold_heading=True)

    assert travelled is None
    correction_index = next(
        index for index, (_linear, angular) in enumerate(published) if angular < 0.0)
    assert all(command == (0.0, 0.0) for command in published[correction_index + 1:])
    assert published[-5:] == [(0.0, 0.0)] * 5
    stopped = [e for e in _events(node) if e['event'] == 'straight_drive_stopped']
    assert stopped[-1]['reason'] == 'odom TF unavailable during heading hold'


def test_back_off_grows_with_each_hold_at_the_same_block():
    for wait, distance in ((1, 0.10), (2, 0.20), (3, 0.30)):
        node = route()
        node._scan_in_base = Mock(return_value=SNAPSHOT)
        node._blocked_ahead = Mock(return_value=True)
        node._footprint_intrusion = Mock(return_value=None)
        node._blocked_behind = Mock(return_value=False)
        node._drive_straight = Mock(return_value=True)
        assert node._escape_blocked(wait) is True
        assert node._drive_straight.call_args.args[0] == pytest.approx(-distance)
        assert node._blocked_behind.call_args.kwargs['band_m'] == pytest.approx(distance + 0.05)


def test_controller_block_retries_the_leg_on_the_same_rpp_trees(monkeypatch):
    """2026-10-06: no MPPI retry; the escape, then the same trees with a new plan."""
    _fast_clock(monkeypatch, step_s=1)
    node = route()
    node.through_behavior_tree, node.staging_behavior_tree = 'transit_rpp.xml', 'staging_rpp.xml'
    node._escape_blocked = Mock(return_value=True)
    node._front_clear = Mock(return_value=True)
    trees = []
    tries, attempt = _blocked_attempts(node, blocked_tries=1, code=105)

    def recording_attempt():
        trees.append(node.staging_behavior_tree)
        return attempt()

    assert node._run_with_input_recovery(recording_attempt) is True
    assert trees == ['staging_rpp.xml', 'staging_rpp.xml']
    node._escape_blocked.assert_not_called()
    assert not any(e['event'] == 'recovery_controller' for e in _events(node))


def test_planner_block_keeps_the_controller(monkeypatch):
    """Without a path (208) nothing is wrong with the controller: no MPPI retry."""
    _fast_clock(monkeypatch, step_s=1)
    node = route()
    node._path_open = Mock(return_value=True)
    node._escape_blocked = Mock()
    trees = []
    tries, attempt = _blocked_attempts(node, blocked_tries=1, code=208)

    def recording_attempt():
        trees.append(node.staging_behavior_tree)
        return attempt()

    assert node._run_with_input_recovery(recording_attempt) is True
    assert trees == ['staging.xml', 'staging.xml']
    node._escape_blocked.assert_not_called()


def test_a_touching_object_backs_out_along_the_trail_further_each_hold():
    """18:41: something touching the body; back out the way the base came in."""
    for wait, distance in ((1, 0.30), (2, 0.45), (3, 0.60), (4, 0.60)):
        for ahead, intrusion, behind in ((False, 'rear', False), (True, 'front', False),
                                         (True, None, True), (True, None, None)):
            node = route()
            node._scan_in_base = Mock(return_value=SNAPSHOT)
            node._blocked_ahead = Mock(return_value=ahead)
            node._footprint_intrusion = Mock(return_value=intrusion)
            node._blocked_behind = Mock(return_value=behind)
            node._drive_straight = Mock()
            node._retrace = Mock(return_value=True)
            assert node._escape_blocked(wait) is True
            node._drive_straight.assert_not_called()
            assert node._retrace.call_args.args[0] == pytest.approx(distance)
            # Only something touching the body justifies pausing the monitor.
            assert node._retrace.call_args.kwargs['pause_monitor'] is (intrusion is not None)
            assert node._retrace.call_args.kwargs['found'] is SNAPSHOT


def _empty_scan():
    return SimpleNamespace(ranges=[math.inf] * 4, angle_min=0.0, angle_increment=math.pi / 2,
                           range_min=0.28, range_max=12.0)


def _trail_route(points=(), memory=()):
    """Drove +x 0.78 m along y=2 in odom and stands at the trail end."""
    node = route()
    node.odom_trail = [(1.0 + 0.02 * k, 2.0, 0.0) for k in range(40)]
    node._odom_pose = Mock(return_value=(1.78, 2.0, 0.0))
    node._scan_in_base = Mock(return_value=(
        _points_scan(points), (0.0, 0.0, 0.0), STEPPED, list(memory)))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: builtin_interfaces.msg.Time()))
    node.engage_emergency_stop = Mock()
    return node


def test_retrace_reverses_the_last_part_of_the_trail_with_the_monitor_paused():
    node = _trail_route()
    switched = []
    node._set_collision_monitor = Mock(side_effect=lambda on: switched.append(on) or True)
    node._execute_reverse_once = Mock(return_value=True)
    assert node._retrace(0.30) is True
    path = node._execute_reverse_once.call_args.args[0]
    assert path.header.frame_id == 'odom'               # no map jump moves the trail
    xs = [p.pose.position.x for p in path.poses]
    assert xs[0] == pytest.approx(1.78) and xs[-1] == pytest.approx(1.48)
    assert xs == sorted(xs, reverse=True)                  # back the way it came
    assert node._execute_reverse_once.call_args.kwargs['controller_id'] == 'ParkingReverse'
    assert switched == [False, True]
    node.engage_emergency_stop.assert_not_called()
    assert node._monitor_confirmed_on is True


def test_retrace_without_a_pause_keeps_the_monitor_on():
    node = _trail_route()
    node._set_collision_monitor = Mock(return_value=True)
    node._execute_reverse_once = Mock(return_value=True)
    assert node._retrace(0.30, pause_monitor=False) is True
    node._set_collision_monitor.assert_not_called()


def test_retrace_latches_the_emergency_stop_when_the_monitor_stays_off():
    node = _trail_route()
    node._set_collision_monitor = Mock(side_effect=[True, False, False, False])
    node._execute_reverse_once = Mock(side_effect=RuntimeError('aborted'))
    with pytest.raises(RuntimeError):
        node._retrace(0.30)
    node.engage_emergency_stop.assert_called_once()
    assert node.stop_requested is True                     # no later goal runs unprotected


def test_retrace_that_cannot_restore_the_monitor_stops_the_run():
    node = _trail_route()
    node._set_collision_monitor = Mock(side_effect=[True, False, False, False])
    node._execute_reverse_once = Mock(return_value=True)
    assert node._retrace(0.30) is False
    node.engage_emergency_stop.assert_called_once()
    assert node.stop_requested is True
    assert node._monitor_confirmed_on is False


def test_retrace_whose_pause_failed_switches_the_monitor_back_on():
    """A timed-out pause may still land late; switch it back on before giving up."""
    node = _trail_route()
    node._set_collision_monitor = Mock(side_effect=[False, True])
    node._execute_reverse_once = Mock()
    assert node._retrace(0.30) is None                     # refused before moving
    assert [call.args for call in node._set_collision_monitor.call_args_list] == [
        (False,), (True,)]
    node._execute_reverse_once.assert_not_called()
    node.engage_emergency_stop.assert_not_called()


def test_retrace_without_a_trail_does_not_move():
    node = _trail_route()
    node.odom_trail = [(0.0, 0.0, 0.0)]
    node._set_collision_monitor = Mock()
    node._execute_reverse_once = Mock()
    assert node._retrace(0.30) is None
    node._set_collision_monitor.assert_not_called()
    node._execute_reverse_once.assert_not_called()


@pytest.mark.parametrize('points, memory', [
    ([(-0.45, 0.0)], ()),          # behind the rear edge on the way back
    ((), [(-0.40, 0.10)]),         # remembered inside the blind range
    ([(-0.25, 0.0)] * 3, ()),      # touches the rear: the way back pushes it
])
def test_retrace_refuses_a_way_back_that_is_not_clear(points, memory):
    node = _trail_route(points, memory)
    node._set_collision_monitor = Mock()
    node._execute_reverse_once = Mock()
    assert node._retrace(0.30) is None
    node._set_collision_monitor.assert_not_called()
    node._execute_reverse_once.assert_not_called()
    refused = [e for e in _events(node) if e['event'] == 'path_blocked_retrace']
    assert refused[-1]['reason'] == 'trail not clear'


def test_retrace_slides_off_what_touches_the_body_but_never_into_it():
    """18:41: the touching object is what the base backs away from or slides along."""
    # In front of the body and beside the front frame, ahead of the wheels.
    node = _trail_route([(0.07, 0.0)] * 3 + [(0.075, -0.243)] * 3)
    node._set_collision_monitor = Mock(return_value=True)
    node._execute_reverse_once = Mock(return_value=True)
    assert node._retrace(0.30) is True


def test_retrace_after_a_back_off_continues_backwards_not_over_the_back_off():
    """Review 2026-10-06: a trail with the back-off in it drove 0.30 m forward to the touch."""
    from collections import deque
    node = _trail_route()
    trail = deque(maxlen=corridor_route.ODOM_TRAIL_POINTS)
    for k in range(81):                                    # forward 0.80 m in 1 cm steps
        corridor_route.CorridorRoute._extend_odom_trail(trail, (0.01 * k, 0.0, 0.0), 0.12)
    for k in range(1, 11):                                 # straight back-off 0.10 m
        corridor_route.CorridorRoute._extend_odom_trail(
            trail, (0.80 - 0.01 * k, 0.0, 0.0), -0.05)
    assert trail[-1][0] <= 0.70 + 1e-9                     # unwound past the back-off
    node.odom_trail = trail
    node._odom_pose = Mock(return_value=(0.70, 0.0, 0.0))
    node._set_collision_monitor = Mock(return_value=True)
    node._execute_reverse_once = Mock(return_value=True)
    assert node._retrace(0.30) is True
    xs = [p.pose.position.x for p in node._execute_reverse_once.call_args.args[0].poses]
    assert xs[0] == pytest.approx(0.70) and max(xs) == pytest.approx(0.70)
    assert xs == sorted(xs, reverse=True)
    assert 0.37 <= xs[-1] <= 0.40 + 1e-9                   # 0.30 m back, to a trail step


@pytest.mark.parametrize('current, trail, end', [
    # A raw trail with a stretch driven in reverse: walked back it leads forward.
    ((0.04, 0.0, 0.0), [(0.0, 0.0, 0.0), (0.02, 0.0, 0.0), (0.04, 0.0, 0.0),
                        (0.06, 0.0, 0.0), (0.04, 0.0, 0.0)], (0.04, 0.0)),
    # An odom jump.
    ((0.06, 0.0, 0.0), [(5.0, 3.0, 0.0), (0.02, 0.0, 0.0), (0.04, 0.0, 0.0)], (0.02, 0.0)),
    # A turn on the spot between two trail poses.
    ((0.06, 0.0, 0.0), [(-0.02, 0.0, math.pi / 2), (0.0, 0.0, math.pi / 2),
                        (0.02, 0.0, 0.0), (0.04, 0.0, 0.0)], (0.02, 0.0)),
    # Sideways off the heading (the base turned since): not the way in.
    ((0.0, 0.0, math.pi / 2), [(0.0, -0.04, 0.0), (0.0, -0.02, 0.0)], (0.0, 0.0)),
])
def test_trail_walk_stops_where_the_way_back_is_not_the_way_in(current, trail, end):
    from jdamr_cube_navigation.reverse_parking import trail_retrace_points
    points, _length = trail_retrace_points(
        current, trail, 1.0, restaurant_service.RETRACE_MAX_GAP_M,
        restaurant_service.RETRACE_MAX_TURN_RAD)
    assert points[-1][:2] == pytest.approx(end)


def test_odom_trail_unwinds_while_reversing_and_restarts_after_a_jump():
    from collections import deque
    trail = deque(maxlen=corridor_route.ODOM_TRAIL_POINTS)
    extend = corridor_route.CorridorRoute._extend_odom_trail
    for k in range(6):
        extend(trail, (0.02 * k, 0.0, 0.0), 0.1)
    extend(trail, (0.05, 0.0, 0.0), -0.05)                 # reversed 5 cm
    assert [round(x, 2) for x, _y, _yaw in trail] == [0.0, 0.02, 0.04]
    extend(trail, (0.05, 0.0, math.pi), 0.0)                # turned on the spot: kept
    assert len(trail) == 3
    extend(trail, (3.0, 0.0, 0.0), 0.1)                     # odom reset
    assert list(trail) == [(3.0, 0.0, 0.0)]


def test_straight_drive_short_or_wrong_way_is_not_done():
    for travelled, done in ((0.215, True), (0.15, False), (-0.22, False), (None, False)):
        node = route()
        node._drive_zero_turn = Mock(return_value=travelled)
        assert node._drive_straight(0.22, 'path_blocked_escape_forward') is done


def test_odom_trail_keeps_a_point_every_2_cm():
    from collections import deque
    from nav_msgs.msg import Odometry
    node = route()
    node.samples = {}
    node.odom_last_pose = None
    node.amcl_motion_distance_m = node.odom_total_distance_m = 0.0
    node.amcl_motion_rotation_rad = 0.0
    node.odom_trail = deque(maxlen=corridor_route.ODOM_TRAIL_POINTS)
    for k in range(11):
        message = Odometry()
        message.pose.pose.position.x = 0.005 * k
        message.pose.pose.orientation.w = 1.0
        node._odom_callback(message)
    assert [round(x, 3) for x, _y, _yaw in node.odom_trail] == [0.0, 0.02, 0.04]


def test_dock_return_needs_only_the_running_battery_cutoff():
    node = route()
    node.battery_voltage = (node.minimum_battery_v
                            + node.service_contract['minimum_start_battery_v']) / 2
    assert node._departure_battery_ready() is False      # a new departure
    node.home_return = True
    assert node._departure_battery_ready() is True       # the way to the charger
    node.battery_voltage = node.minimum_battery_v - 0.01
    assert node._departure_battery_ready() is False


def test_in_place_trim_commands_no_translation_and_stops_early_by_the_overshoot(monkeypatch):
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    published = []
    node.escape_velocity = SimpleNamespace(publish=lambda m: published.append(
        (m.linear.x, m.angular.z)))
    yaws = iter([0.001 * k for k in range(400)])

    def lookup(*_args, **_kwargs):
        yaw = next(yaws)
        return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    turned = node._rotate_in_place(math.radians(2.0))
    early = restaurant_service.TRIM_STOP_RAD
    assert math.radians(2.0) - early <= turned <= math.radians(2.0) - early + 0.003
    assert all(v == 0.0 for v, _w in published)
    first_stop = published.index((0.0, 0.0))
    assert {w for _v, w in published[:first_stop]} == {restaurant_service.TRIM_ANGULAR_RADPS}
    assert published[first_stop:] == [(0.0, 0.0)] * (len(published) - first_stop)
    assert len(published) - first_stop >= 5


@pytest.mark.parametrize('points, poses, hit', [
    ([(-0.40, 0.0)], [(0.0, 0.0, 0.0), (-0.20, 0.0, 0.0)], True),     # behind on the way back
    ([(-0.40, 0.0)], [(0.0, 0.0, 0.0), (-0.05, 0.0, 0.0)], False),    # short of it
    ([(-0.27, -0.243)], [(0.0, 0.0, 0.0), (-0.20, 0.0, 0.0)], False),  # already inside
    ([(-0.30, 0.40)], [(0.0, 0.0, 0.0), (-0.20, 0.0, 0.0)], False),    # beside the way
    # A reverse arc to the left swings the rear right (base turns +0.4 rad).
    ([(-0.30, -0.27)], [(0.0, 0.0, 0.0), (-0.10, 0.0, 0.2), (-0.18, -0.02, 0.4)], True),
])
def test_path_sweep_follows_the_body_along_the_way_back(points, poses, hit):
    from jdamr_cube_navigation.reverse_parking import path_sweep_hit
    assert path_sweep_hit(points, STEPPED, poses) is hit


def test_returns_the_base_drove_into_the_blind_range_are_remembered():
    """An object 0.40 m ahead of the laser is 0.20 m away after 0.20 m forward: G4 blind."""
    from jdamr_cube_navigation.reverse_parking import remembered_blind_points
    laser = (-0.01, 0.0, math.pi)
    # Beam 2 looks ahead (laser turned 180 deg); beam 1 to the right, far.
    scan = SimpleNamespace(ranges=[math.inf, 1.0, 0.40, math.inf], angle_min=0.0,
                           angle_increment=math.pi / 2, range_min=0.28, range_max=12.0)
    found = remembered_blind_points([((0.0, 0.0, 0.0), scan)], (0.20, 0.0, 0.0), laser, 0.28)
    assert len(found) == 1 and found[0] == pytest.approx((0.19, 0.0))
    # Still visible from where the keyframe was taken: the live scan decides.
    assert remembered_blind_points([((0.0, 0.0, 0.0), scan)], (0.0, 0.0, 0.0), laser, 0.28) == []


def test_filtered_blind_memory_uses_one_current_pose_snapshot(monkeypatch):
    node = route()
    laser = (-0.01, 0.0, math.pi)
    scan = SimpleNamespace(
        ranges=[math.inf, 1.0, .40, math.inf], angle_min=0.0,
        angle_increment=math.pi / 2, range_min=.28, range_max=12.0)
    node.scan_memory = [(9.0, (0.0, 0.0, 0.0), scan)]
    node._odom_pose = Mock(return_value=(.20, 0.0, 0.0))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: 10.0)

    found = node._remembered_blind_points(
        laser, scan,
        source_point_filter=lambda _pose, _scan, _point: True)

    assert len(found) == 1 and found[0] == pytest.approx((.19, 0.0))
    node._odom_pose.assert_called_once_with()
    source_pose, source_scan, point, current = node._blind_point_sources[0]
    assert source_pose == (0., 0., 0.)
    assert source_scan is scan
    assert point == pytest.approx((.39, 0.))
    assert current == (.20, 0., 0.)
    assert node._blind_point_diagnostics[0]['reason'] == (
        'current_scan_timestamp_invalid')


def test_escape_snapshot_needs_a_fresh_scan_and_carries_the_blind_memory(monkeypatch):
    node = route()
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *a, **k: SimpleNamespace(
        transform=SimpleNamespace(translation=SimpleNamespace(x=-0.01, y=0.0),
                                  rotation=SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0))))
    node._footprint_outline = Mock(return_value=STEPPED)
    node.last_scan = SimpleNamespace(header=SimpleNamespace(frame_id='laser_link'),
                                     range_min=0.28)
    node.samples = {'scan': time.monotonic() - 2.0}
    assert node._scan_in_base() is None                     # stale scan: no decision
    node.samples = {'scan': time.monotonic()}
    node._remembered_blind_points = Mock(return_value=[(0.15, 0.0)])
    scan, laser, outline, memory = node._scan_in_base()
    assert laser == pytest.approx((-0.01, 0.0, math.pi)) and memory == [(0.15, 0.0)]
    node._footprint_outline = Mock(side_effect=ValueError('malformed footprint'))
    assert node._scan_in_base() is None


def test_scan_memory_keeps_one_scan_per_5_cm_or_10_deg():
    from collections import deque
    node = route()
    node.scan_memory = deque(maxlen=restaurant_service.SCAN_MEMORY_KEYFRAMES)
    for pose in ((0.0, 0.0, 0.0), (0.03, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.0, 0.1),
                 (0.05, 0.0, 0.2)):
        node.odom_last_pose = pose
        node._remember_scan('scan')
    assert [frame[1] for frame in node.scan_memory] == [
        (0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.0, 0.2)]


def test_scan_memory_stores_pose_at_scan_stamp_and_fails_closed_without_it():
    from collections import deque
    node = route()
    node.scan_memory = deque(maxlen=restaurant_service.SCAN_MEMORY_KEYFRAMES)
    node.odom_last_pose = (9., 9., 1.)
    node.parking_contract = {'robot_base_frame': 'base_link'}
    scan = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=12, nanosec=250_000_000)))
    yaw = .3
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1., y=2.),
        rotation=SimpleNamespace(x=0., y=0., z=math.sin(yaw / 2.),
                                 w=math.cos(yaw / 2.))))
    node.parking_tf = SimpleNamespace(lookup_transform=Mock(return_value=transform))

    node._remember_scan(scan)

    assert node.scan_memory[0][1] == pytest.approx((1., 2., yaw))
    node.scan_memory.clear()
    node.parking_tf.lookup_transform.side_effect = [
        TransformException('missing'), transform]
    node._remember_scan(scan)
    assert not node.scan_memory
    node._remember_scan(scan)
    assert node.scan_memory[0][1] == pytest.approx((1., 2., yaw))


def test_something_in_the_front_blind_range_backs_off_instead_of_driving_forward():
    """Review 2026-10-06: the forward escape could not see 0.08-0.20 m ahead of the front."""
    node = route()
    rear_leg = [(-0.20, -0.32)]                              # holds the turn circle
    node._scan_in_base = Mock(return_value=(
        _points_scan(rear_leg), (0.0, 0.0, 0.0), STEPPED, [(0.15, 0.0)]))
    node._drive_straight = Mock(return_value=True)
    assert node._escape_blocked() is True
    assert node._drive_straight.call_args.args == (
        -restaurant_service.PATH_BLOCKED_BACKOFF_M, 'path_blocked_back_off')
    node._scan_in_base = Mock(return_value=(
        _points_scan(rear_leg), (0.0, 0.0, 0.0), STEPPED, []))
    node._drive_straight = Mock(return_value=True)
    assert node._escape_blocked() is True
    assert node._drive_straight.call_args.args[1] == 'path_blocked_escape_forward'


@pytest.mark.parametrize('setup_s, task_deadline_s, stopped, guard_failure, should_move', [
    (11.0, None, False, None, True),
    (11.0, 5.0, False, None, False),
    (11.0, None, True, None, False),
    (11.0, None, False, 'scan stale', False),
])
def test_direct_motion_setup_keeps_move_budget_without_extending_task(
        monkeypatch, setup_s, task_deadline_s, stopped, guard_failure, should_move):
    node = route()
    clock = [0.0]
    travelled = [0.0]
    published = []
    node.run_deadline_s = task_deadline_s

    def publish(message):
        published.append(message.linear.x)
        if message.linear.x:
            travelled[0] += 0.02

    def prepare():
        clock[0] += setup_s
        node.stop_requested = stopped
        return SimpleNamespace(publish=publish)

    def spin(*_args, **_kwargs):
        clock[0] += 0.02
        node.latest_motion = (clock[0], 0.0, 0.0)

    node._escape_publisher = prepare
    node._guard_failure = lambda *_args: guard_failure
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    command = restaurant_service.Twist()
    command.linear.x = 0.05

    result = node._direct_motion(
        command, lambda distance: distance >= 0.10, lambda: travelled[0],
        7.0, 'straight_drive_stopped')

    assert any(published) is should_move
    assert (result >= 0.10) is should_move
    assert published[-5:] == [0.0] * 5
    assert node.run_deadline_s == task_deadline_s
    expected_failure = ('operator_stop' if stopped else guard_failure
                        or ('direct_motion_timeout' if not should_move else None))
    assert node._last_direct_motion_failure == expected_failure


def test_direct_motion_stops_on_a_failed_input(monkeypatch):
    node = route()
    published = []
    node.escape_velocity = SimpleNamespace(publish=lambda m: published.append(
        (m.linear.x, m.angular.z)))
    poses = iter([(0.001 * k, 0.0) for k in range(1000)])

    def lookup(*_args, **_kwargs):
        x_m, y_m = next(poses)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x_m, y=y_m),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    node._guard_failure = lambda *_: 'scan stale: age=1.000s limit=0.500s'
    travelled = node._drive_zero_turn(0.10)
    assert travelled < 0.01
    assert published[-5:] == [(0.0, 0.0)] * 5
    stopped = [e for e in _events(node) if e['event'] == 'straight_drive_stopped']
    assert stopped and stopped[0]['reason'].startswith('scan stale')


def test_in_place_trim_counts_a_turn_across_pi(monkeypatch):
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    node.escape_velocity = SimpleNamespace(publish=lambda m: None)
    yaws = iter([math.pi - 0.05 + 0.002 * k for k in range(400)])

    def lookup(*_args, **_kwargs):
        yaw = next(yaws)
        return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    turned = node._rotate_in_place(0.20)
    assert 0.18 <= turned <= 0.21


def test_collision_monitor_is_confirmed_on_once_per_executor():
    node = route()
    node._set_collision_monitor = Mock(return_value=False)
    with pytest.raises(RuntimeError, match='Collision Monitor'):
        node._confirm_collision_monitor_on()
    node._set_collision_monitor = Mock(return_value=True)
    node._confirm_collision_monitor_on()
    node._confirm_collision_monitor_on()
    node._set_collision_monitor.assert_called_once_with(True)


def test_collision_monitor_toggle_drops_a_request_that_timed_out(monkeypatch):
    node = route()
    future = SimpleNamespace(done=lambda: False)
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    node.monitor_toggle = client
    clock = iter(range(0, 100))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    assert node._set_collision_monitor(True) is False
    client.remove_pending_request.assert_called_once_with(future)


def test_a_touching_rear_drives_off_it_forward_when_the_way_back_pushes_it():
    """11:09 and review 2026-10-06: the rear corner on a desk leg; reversing pushes it."""
    node = route()
    node._scan_in_base = Mock(return_value=SNAPSHOT)
    node._blocked_ahead = Mock(return_value=False)
    node._footprint_intrusion = Mock(return_value='rear')
    node._retrace = Mock(return_value=None)                 # refused before moving
    node._rear_swing_need = Mock(return_value=0.15)
    node._drive_straight = Mock(return_value=True)
    assert node._escape_blocked() is True
    assert node._drive_straight.call_args.args == (
        pytest.approx(0.18), 'path_blocked_escape_forward')
    assert node._drive_straight.call_args.kwargs['pause_monitor'] is True
    for intrusion, ahead, retraced in (('rear', True, None), ('front', False, None),
                                       ('rear', False, False)):
        node = route()
        node._scan_in_base = Mock(return_value=SNAPSHOT)
        node._blocked_ahead = Mock(return_value=ahead)
        node._footprint_intrusion = Mock(return_value=intrusion)
        node._retrace = Mock(return_value=retraced)
        node._drive_straight = Mock()
        assert node._escape_blocked() is False
        node._drive_straight.assert_not_called()


def test_remembered_returns_never_pause_the_monitor():
    """The monitor sees the live scan only; a stale remembered return cannot hold it."""
    node = route()
    found = (_points_scan([]), (0.0, 0.0, 0.0), STEPPED, [(-0.25, 0.0)] * 3)
    assert node._footprint_intrusion(found=found) is None
    assert node._blocked_behind(0.10, found=found) is True  # but it still blocks motion


def test_paused_straight_drive_switches_the_monitor_back_on():
    node = route()
    switched = []
    node._set_collision_monitor = Mock(side_effect=lambda on: switched.append(on) or True)
    node._drive_zero_turn = Mock(return_value=0.18)
    node.engage_emergency_stop = Mock()
    found = (_points_scan([]), (0.0, 0.0, 0.0), STEPPED, [])
    assert node._drive_straight(0.18, 'path_blocked_escape_forward', pause_monitor=True,
                                found=found)
    assert switched == [False, True]
    assert callable(node._drive_zero_turn.call_args.kwargs['watch'])
    node._set_collision_monitor = Mock(side_effect=[True, False, False, False])
    assert not node._drive_straight(0.18, 'path_blocked_escape_forward', pause_monitor=True,
                                    found=found)
    node.engage_emergency_stop.assert_called_once()
    assert node.stop_requested is True
    node = route()
    node._set_collision_monitor = Mock()
    assert not node._drive_straight(0.18, 'path_blocked_escape_forward', pause_monitor=True)
    node._set_collision_monitor.assert_not_called()          # nothing to watch the way with


def test_paused_straight_drive_stops_when_someone_steps_in_front(monkeypatch):
    """Review 2026-10-06: with the monitor paused, every fresh scan watches the way."""
    node = route()
    node.escape_velocity = SimpleNamespace(publish=lambda m: None)
    poses = iter([(0.001 * k, 0.0) for k in range(1000)])

    def lookup(*_args, **_kwargs):
        x_m, y_m = next(poses)
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=x_m, y=y_m),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    found = (_points_scan([]), (0.0, 0.0, 0.0), STEPPED, [])
    watch = node._band_watch(found, 0.18)
    node.samples = {'scan': time.monotonic()}
    node.last_scan = _points_scan([])
    assert watch() is None
    node.last_scan = _points_scan([(0.15, 0.0)])            # a leg 6.5 cm ahead of the front
    travelled = node._drive_zero_turn(0.18, watch=watch)
    assert travelled < 0.01
    stopped = [e for e in _events(node) if e['event'] == 'straight_drive_stopped']
    assert stopped[-1]['reason'] == 'way not clear while the monitor is paused'
    node.samples = {'scan': time.monotonic() - 1.0}
    assert watch() == 'no fresh scan while the monitor is paused'


def test_a_trim_just_over_the_minimum_still_turns(monkeypatch):
    """Review 2026-10-06: an early stop larger than a 0.32 deg trim skipped it."""
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    published = []
    node.escape_velocity = SimpleNamespace(publish=lambda m: published.append(m.angular.z))
    yaws = iter([0.0002 * k for k in range(400)])

    def lookup(*_args, **_kwargs):
        yaw = next(yaws)
        return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))

    node.parking_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    turned = node._rotate_in_place(math.radians(0.32))
    assert any(0.0 < rate < restaurant_service.TRIM_ANGULAR_RADPS for rate in published)
    assert turned >= math.radians(0.16)


def _dwell_node(monkeypatch, poses, verified=True):
    node = route()
    node.selected_pose = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    node.waypoints = [{'id': 'main', 'x': 1.0, 'y': 0.0, 'yaw': 0.0}]
    node._verify_parking_stop = Mock(return_value=verified)
    poses = iter(poses)
    node._odom_pose = lambda: next(poses)
    clock = iter(range(0, 10000))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: float(next(clock)))
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', lambda *a, **k: None)
    return node


def test_short_dwell_is_one_stationary_window(monkeypatch):
    node = _dwell_node(monkeypatch, [])
    assert node.wait_parked(2.0) is True
    assert node._verify_parking_stop.call_args.kwargs['hold_s'] == 2.0


def test_long_dwell_confirms_five_seconds_then_only_has_to_stay_put(monkeypatch):
    """A 30 s measurement dwell: 5 s confirmed window, then odom must not move."""
    still = [(0.0, 0.0, 0.0)] + [(0.001, 0.0, 0.001)] * 100
    node = _dwell_node(monkeypatch, still)
    assert node.wait_parked(30.0) is True
    assert node._verify_parking_stop.call_args.kwargs['hold_s'] == (
        restaurant_service.PARKED_CONFIRM_MAX_S)
    moved = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.03, 0.0, 0.0)] + [(0.03, 0.0, 0.0)] * 100
    node = _dwell_node(monkeypatch, moved)
    assert node.wait_parked(30.0) is False
    assert [e for e in _events(node) if e['event'] == 'parked_hold_moved']


def test_long_dwell_fails_when_odom_goes_unread(monkeypatch):
    """No odom for longer than the observation timeout is not a held stop."""
    node = _dwell_node(monkeypatch, [])
    node.parking_contract = {'observation_timeout_s': 5.0}
    reads = iter([(0.0, 0.0, 0.0)])

    def odom():
        try:
            return next(reads)
        except StopIteration:
            raise RuntimeError('odom transform unavailable') from None
    node._odom_pose = odom
    assert node.wait_parked(30.0) is False
    assert [e for e in _events(node) if e['event'] == 'parked_hold_unobserved']
    assert [e for e in _events(node) if e['event'] == 'parked_dwell_failed']


def test_default_transit_follows_navfn_paths_with_rpp(monkeypatch):
    """2026-10-06 selection: transit and staging run on the RPP trees by default."""
    monkeypatch.setattr(restaurant_service, 'get_package_share_directory',
                        lambda _: str(PACKAGE))
    assert restaurant_service.DEFAULT_TRANSIT == 'navfn-rpp'
    node = route()
    node.use_transit(restaurant_service.DEFAULT_TRANSIT)
    assert node.through_behavior_tree.endswith('navigate_through_poses_transit_rpp.xml')
    assert node.staging_behavior_tree.endswith('navigate_to_pose_staging_rpp.xml')
    assert node.final_leg_behavior_tree is None
    node.use_transit('navfn-mppi')
    assert node.final_leg_behavior_tree.endswith('navigate_through_poses_transit_rpp.xml')


@pytest.mark.parametrize('variant, planners, controller', [
    ('lattice-mppi', ['Lattice', 'GridBased'], 'MPPI'),
    ('navfn-mppi', ['GridBased'], 'MPPI'),
    ('navfn-rpp', ['GridBased'], 'FollowPath'),
])
def test_transit_variants_pick_their_trees(monkeypatch, variant, planners, controller):
    from xml.etree import ElementTree as ET
    monkeypatch.setattr(restaurant_service, 'get_package_share_directory',
                        lambda _: str(PACKAGE))
    node = route()
    node.use_transit(variant)
    for tree in (node.through_behavior_tree, node.staging_behavior_tree):
        root = ET.parse(tree).getroot()
        used = [e.get('planner_id') for e in root.iter() if e.get('planner_id')]
        assert sorted(set(used)) == sorted(planners)
        assert [e.get('controller_id') for e in root.iter('FollowPath')] == [controller]


@pytest.mark.parametrize('requested_deg', [0.32, 0.59, 0.87, 1.2, -0.32, -0.59, -0.87, -1.2])
def test_default_tiny_trim_stays_within_tolerance_without_reversing(requested_deg, monkeypatch):
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: SimpleNamespace(
        transform=SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    commands = []
    def direct(command, done, measure, deadline, event, watch=None):
        commands.append((command.linear.x, command.angular.z))
        turned = 0.0
        while not done(turned):
            turned += command.angular.z * 0.02
        return turned + command.angular.z * 0.19
    node._direct_motion = direct
    requested = math.radians(requested_deg)
    turned = node._rotate_in_place(requested)
    assert len(commands) == 1
    linear, angular = commands[0]
    assert linear == 0.0
    assert 0 < abs(angular) < restaurant_service.TRIM_ANGULAR_RADPS
    assert angular * requested > 0.0
    assert abs(turned - requested) < math.radians(0.3)


@pytest.mark.parametrize('requested_deg', [-1.2, 1.2])
def test_dock_trim_uses_same_damped_turn_without_an_opt_in(requested_deg, monkeypatch):
    node = route()
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        lambda _node: lambda _command: None)
    yaw = [0.0]
    rates = []
    node._odom_pose = lambda: (0.0, 0.0, yaw[0])
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: SimpleNamespace(
        transform=SimpleNamespace(rotation=SimpleNamespace(
            x=0., y=0., z=math.sin(yaw[0] / 2), w=math.cos(yaw[0] / 2)))))
    def direct(command, done, measure, deadline, event, watch=None):
        rates.append(command.angular.z)
        turned = 0.0
        while not done(turned):
            turned += command.angular.z * 0.02
        turned += command.angular.z * 0.19
        yaw[0] += turned
        return turned
    node._direct_motion = direct
    target = math.radians(requested_deg)
    assert node._trim_dock_heading({'yaw': target})
    assert len(rates) == 1
    assert 0 < abs(rates[0]) < restaurant_service.TRIM_ANGULAR_RADPS
    assert rates[0] * target > 0.0
    assert abs(yaw[0] - target) <= restaurant_service.DOCK_HEADING_TOLERANCE_RAD


@pytest.mark.parametrize('interruption', ['none', 'operator', 'link'])
def test_live_depth_gap_waits_stopped_and_resumes_same_target(monkeypatch, interruption):
    node = route()
    clock, position, requested = [0.0], [0.0], [0.0]
    commands = []
    def publish(message):
        requested[0] = message.linear.x
        commands.append((clock[0], message.linear.x, message.angular.z))
    def spin(*_a, **_k):
        clock[0] += 0.02
        position[0] += requested[0] * 0.02
        node.latest_motion = (clock[0], requested[0], 0.0)
        if ((interruption == 'operator' and clock[0] >= 1.6)
                or (interruption == 'link' and clock[0] >= 3.0)):
            node.stop_requested = True
    def update(actual):
        if actual[0] > 0.01 and clock[0] < 1.9:
            raise RuntimeError('table approach depth stale or future-dated')
        return {'remaining_m': 0.04 - actual[0]}
    node._guard_failure = lambda *_a: ('scan stale: recovering' if 1.7 <= clock[0] < 2.1 else None)
    node._operator_link_ready = lambda: not (interruption == 'link' and 1.5 <= clock[0] < 2.0)
    node.latest_motion = (-1.0, 0.0, 0.0)
    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0., z=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    travelled = node._drive_zero_turn(.022, precision=True, hold_heading=True,
                                     target_update=update)
    first_zero = next(i for i, (_t, v, _w) in enumerate(commands) if v == 0.)
    assert all(v == 0. and w == 0. for t, v, w in commands[first_zero:] if t < 2.1)
    events = _events(node)
    assert any(e['event'] == 'table_approach_waiting_for_input' for e in events)
    if interruption != 'none':
        assert travelled is None
        assert not any(e['event'] == 'table_approach_input_recovered' for e in events)
        assert position[0] < .02
    else:
        assert travelled == pytest.approx(.04, abs=.003)
        assert any(e['event'] == 'table_approach_input_recovered' for e in events)
        assert any(t >= 2.1 and v > 0. for t, v, _w in commands)


@pytest.mark.parametrize('failure', [
    'rear box face unavailable', 'rear box target changed',
    'rear docking swept footprint occupied', 'table approach tape reference pending'])
def test_rear_input_gap_pauses_task_deadline_and_resumes_same_motion(monkeypatch, failure):
    node = route()
    clock, position, velocity = [10.0], [0.0], [0.0]
    commands = []
    node.run_deadline_s = 14.0
    node._operator_link_ready = lambda: True
    node.latest_motion = (clock[0], 0.0, 0.0)
    def publish(message):
        velocity[0] = message.linear.x
        commands.append((clock[0], message.linear.x))
    def spin(*_a, **_k):
        clock[0] += .02
        position[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.0)
        node._deadline_tick()
    def update(actual):
        if actual[0] > .01 and clock[0] < 15.5:
            raise RuntimeError(failure)
        return {'remaining_m': .04 - actual[0]}
    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    result = node._drive_zero_turn(.04, precision=True, hold_heading=True, target_update=update)
    prefix = 'table_approach' if failure.startswith('table ') else 'rear_box'
    waiting = next(e for e in _events(node) if e['event'] == f'{prefix}_waiting_for_input')
    assert waiting['target_preserved']
    assert all(v == 0. for t, v in commands if waiting['monotonic_s'] <= t < 15.5)
    assert result == pytest.approx(.04, abs=.003)
    assert clock[0] > 14.0
    assert node.run_deadline_s > 18.0
    assert any(e['event'] == f'{prefix}_input_recovered' for e in _events(node))


@pytest.mark.parametrize('previous_pause', [False, True])
def test_rear_input_gap_pauses_deadline_before_stop_confirmation(
        monkeypatch, previous_pause):
    node = route()
    clock, position, velocity = [10.0], [0.0], [0.0]
    commands = []
    node.run_deadline_s = 10.16
    node._input_recovery_paused = previous_pause
    node._operator_link_ready = lambda: True
    node.latest_motion = (clock[0], 0.0, 0.0)

    def publish(message):
        velocity[0] = message.linear.x
        commands.append((clock[0], message.linear.x))

    def spin(*_a, **_k):
        clock[0] += .02
        position[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.0)
        node._deadline_tick()

    def update(actual):
        if actual[0] > .006:
            if clock[0] < 10.5:
                raise RuntimeError('rear box face unavailable')
            return {'remaining_m': 0.0, 'target_confirmed': True}
        return {'remaining_m': .04 - actual[0]}

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    result = node._drive_zero_turn(
        .2, precision=True, hold_heading=True, target_update=update)
    assert result == pytest.approx(position[0])
    assert not node.stop_requested
    assert node.run_deadline_s > 10.5
    assert any(e['event'] == 'rear_box_waiting_for_input' for e in _events(node))
    waiting_s = next(e['monotonic_s'] for e in _events(node)
                     if e['event'] == 'rear_box_waiting_for_input')
    assert all(v == 0. for t, v in commands if waiting_s <= t < 10.5)
    assert node._input_recovery_paused is previous_pause


@pytest.mark.parametrize('previous_pause', [False, True])
def test_first_measure_failure_exception_restores_deadline_timer(
        monkeypatch, previous_pause):
    node = route()
    clock, position = [10.0], [0.0]
    node.run_deadline_s = 12.0
    node._input_recovery_paused = previous_pause
    node._operator_link_ready = lambda: True
    node.latest_motion = (clock[0], 0.0, 0.0)
    node.escape_velocity = SimpleNamespace(publish=lambda _message: None)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))

    def spin(*_a, **_k):
        clock[0] += .02
        raise RuntimeError('callback failed during zero flush')

    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service, 'spin_node', spin)
    with pytest.raises(RuntimeError, match='callback failed during zero flush'):
        node._drive_zero_turn(
            .2, precision=True, hold_heading=True,
            target_update=Mock(side_effect=RuntimeError('rear box face unavailable')))
    assert node._input_recovery_paused is previous_pause


@pytest.mark.parametrize('terminal_failure', ['operator_link_lost', 'battery low'])
def test_rear_input_pause_aborts_on_communication_or_battery_guard(
        monkeypatch, terminal_failure):
    node = route()
    clock, position, velocity = [10.0], [0.0], [0.0]
    commands = []
    node.run_deadline_s = 14.0
    node.latest_motion = (clock[0], 0.0, 0.0)
    node._operator_link_ready = lambda: not (
        terminal_failure == 'operator_link_lost' and clock[0] >= 11.0)
    node._guard_failure = lambda *_a: (
        'battery low: voltage=10.400V limit=10.500V'
        if terminal_failure == 'battery low' and clock[0] >= 11.0 else None)
    def publish(message):
        velocity[0] = message.linear.x
        commands.append((clock[0], message.linear.x))
    def spin(*_a, **_k):
        clock[0] += .02
        position[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.0)
    def update(actual):
        if actual[0] > .01:
            raise RuntimeError('rear box face unavailable')
        return {'remaining_m': .04 - actual[0]}
    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    assert node._drive_zero_turn(
        .04, precision=True, hold_heading=True, target_update=update) is None
    assert node._last_direct_motion_failure.startswith(terminal_failure)
    waiting_s = next(e['monotonic_s'] for e in _events(node)
                     if e['event'] == 'rear_box_waiting_for_input')
    assert all(v == 0. for t, v in commands if t >= waiting_s)
    assert clock[0] < 11.2


@pytest.mark.parametrize('intermediate', [True, False])
@pytest.mark.parametrize('next_capture', ['continuous', 'motion', 'gap', 'contract'])
def test_intermediate_stationary_capture_uses_short_hold_without_relaxing_inputs(
        monkeypatch, intermediate, next_capture):
    node = route()
    clock = [10.0]
    node._guard_failure = lambda *_a, **_k: None
    node.parking_motion_revision = 0
    node.parking_command = (10., 0., 0.)
    node.parking_odom = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(clock[0]*1e9)))
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=int(clock[0]), nanosec=int(clock[0] % 1 * 1e9))),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
                                     rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    def spin(*_a, **_k):
        clock[0] += .05
        stamp = SimpleNamespace(sec=int(clock[0]), nanosec=int(clock[0] % 1 * 1e9))
        node.parking_odom = (clock[0], stamp, 0., 0.)
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    actual, evidence = node.capture_stationary_pose(intermediate=intermediate)
    assert actual == (0., 0., 0.)
    assert evidence['confirmed']
    assert evidence['command_observed']
    assert 0.08 <= clock[0]-10.0 < (.3 if intermediate else 1.3)
    if not intermediate:
        assert clock[0]-10.0 >= node.parking_contract['hold_s']
    if next_capture == 'motion':
        node.parking_motion_revision += 1
    elif next_capture == 'gap':
        clock[0] += max(node.parking_contract['sample_max_age_s'],
                        node.parking_contract['observation_timeout_s']) + .1
    elif next_capture == 'contract':
        node.parking_contract = {**node.parking_contract,
                                 'xy_tolerance_m': node.parking_contract['xy_tolerance_m'] / 2}
    started_s = clock[0]
    _, evidence = node.capture_stationary_pose(intermediate=intermediate)
    assert evidence['confirmed']
    elapsed_s = clock[0] - started_s
    if next_capture == 'continuous':
        assert 0.0 < elapsed_s < .1  # A fresh sample, without another full hold.
    else:
        required_s = (restaurant_service.DIRECT_STOP_CONFIRM_S if intermediate
                      else node.parking_contract['hold_s'])
        assert elapsed_s >= required_s


@pytest.mark.parametrize('failure', [None, RuntimeError('controller_server parameter read timed out')])
def test_parking_read_failure_retains_controller_cause(failure):
    node = route()
    node.parking_contract = load_parking_contract(PACKAGE / 'config/box_parking_contract.yaml')
    node._read_parameters = Mock(return_value=None, side_effect=failure)
    assert not node._parking_parameters_ready()
    assert node.last_parking_parameters_failure == (
        str(failure) if failure else 'controller_server parameter service unavailable')
    assert node._read_parameters.call_args.args[0] == 'controller_server'


@pytest.mark.parametrize('reached,invalid', [(True, False), (False, False), (True, True)])
def test_rear_recovery_checks_valid_arrival_before_another_motion_preview(
        monkeypatch, reached, invalid):
    node = route()
    clock, position, velocity = [10.], [0.], [0.]
    commands, watch_calls = [], []
    node.run_deadline_s = 12.
    node._operator_link_ready = lambda: True
    node._guard_failure = lambda *_a: None
    node.latest_motion = (clock[0], 0., 0.)
    def publish(message):
        velocity[0] = message.linear.x
        commands.append(message.linear.x)
    def spin(*_a, **_k):
        clock[0] += .02
        position[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.)
        if clock[0] >= 11.0 and not (reached and not invalid):
            node.stop_requested = True
    def update(actual):
        if len(watch_calls) >= 1:
            if invalid:
                raise RuntimeError('rear box face unavailable')
            if reached:
                return {'remaining_m': 0., 'heading_rad': math.radians(.7),
                        'allow_near_alignment': True, 'target_confirmed': True}
        return {'remaining_m': .2 + actual[0]}
    def watch(command):
        watch_calls.append(command.linear.x)
        return 'rear docking swept footprint occupied'
    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0., z=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    result = node._drive_zero_turn(-.2, precision=True, hold_heading=True,
                                  target_update=update, command_watch=watch)
    assert all(value == 0. for value in commands)
    if reached and not invalid:
        assert result == pytest.approx(0.)
        assert len(watch_calls) == 1
    else:
        assert result is None


def test_odom_rotation_installs_live_sweep_before_any_nonzero_command(monkeypatch):
    node = route()
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: SimpleNamespace(
        transform=SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    guard = Mock(return_value='rotation swept footprint occupied')
    factory = Mock(return_value=guard)
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch', factory, raising=False)
    def direct(command, done, measure, deadline, event, watch=None):
        assert callable(watch)
        assert watch() == 'rotation swept footprint occupied'
        node._last_direct_motion_failure = 'rotation swept footprint occupied'
        return 0.
    node._direct_motion = direct
    assert node._rotate_in_place(math.radians(57.)) is None
    factory.assert_called_once_with(node)
    assert guard.call_args.args[0].linear.x == 0.
    assert guard.call_args.args[0].angular.z != 0.


def test_rotation_without_valid_scan_never_starts_motion(monkeypatch):
    node = route()
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k: SimpleNamespace(
        transform=SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service, 'rotation_collision_watch',
                        Mock(side_effect=RuntimeError('rotation scan unavailable')))
    node._direct_motion = Mock()
    assert node._rotate_in_place(.3) is None
    node._direct_motion.assert_not_called()
    assert node._last_direct_motion_failure == 'rotation scan unavailable'


def test_obstacle_wait_does_not_restart_when_old_wait_duration_expires(monkeypatch):
    _fast_clock(monkeypatch, step_s=5)
    node = route()
    node._front_clear = Mock(side_effect=[False] * 12 + [True, True])
    node._escape_blocked = Mock()
    tries, attempt = _blocked_attempts(node, blocked_tries=1, code=105)
    assert node._run_with_input_recovery(attempt) is True
    assert len(tries) == 2
    node._escape_blocked.assert_not_called()
    cleared = [e for e in _events(node) if e['event'] == 'path_blocked_cleared']
    assert cleared[0]['held_s'] > corridor_route.PATH_BLOCKED_WAIT_S


def test_repeated_input_gaps_do_not_recount_previous_wait(monkeypatch):
    node = route()
    clock, position, velocity = [10.0], [0.0], [0.0]
    node.run_deadline_s = 100.0
    node._operator_link_ready = lambda: True
    node.latest_motion = (clock[0], 0.0, 0.0)

    def publish(message):
        velocity[0] = message.linear.x

    def spin(*_a, **_k):
        clock[0] += .02
        position[0] += velocity[0] * .02
        node.latest_motion = (clock[0], velocity[0], 0.0)

    def update(actual):
        if ((actual[0] > .01 and clock[0] < 15.0)
                or (actual[0] > .025 and clock[0] < 17.0)):
            raise RuntimeError('rear box face unavailable')
        return {'remaining_m': .06 - actual[0]}

    node.escape_velocity = SimpleNamespace(publish=publish)
    node.parking_tf = SimpleNamespace(lookup_transform=lambda *_a, **_k:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=position[0], y=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    monkeypatch.setattr(restaurant_service.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_service.rclpy, 'spin_once', spin)
    result = node._drive_zero_turn(.06, precision=True, hold_heading=True,
                                   target_update=update)
    recovered = [e for e in _events(node) if e['event'] == 'rear_box_input_recovered']
    assert result == pytest.approx(.06, abs=.003)
    assert len(recovered) == 2
    assert recovered[1]['paused_s'] <= 2.0
    assert node.run_deadline_s - 100.0 <= clock[0] - 10.0
    assert not node._input_recovery_paused
