"""Verify bounded table transit and face-based parking without hardware."""

import contextlib
from collections import deque
from copy import deepcopy
import inspect
import json
import math
from pathlib import Path
import time
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from jdamr_cube_navigation import box_service
from jdamr_cube_navigation.box_service import BoxServiceRoute, parse_args
from jdamr_cube_navigation.corridor_route import CorridorRoute
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import Path as RosPath
import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.task import Future
from rclpy.time import Time
import yaml


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'
BOX_CONTRACT = ROOT / 'jdamr_cube_navigation/config/box_parking_contract.yaml'
CAMERA_MOUNT = ROOT / 'jdamr_cube_navigation/config/camera_mount_derived_20261012.yaml'
TABLE_EDGE_ALIGNMENT_FIXTURE = (
    ROOT / 'jdamr_cube_navigation/test/fixtures/table_edge_alignment_20261013.json')


@pytest.fixture
def clear_precision_corridor(monkeypatch):
    """Isolate trim geometry; corridor acceptance/rejection has separate cases."""
    monkeypatch.setattr(box_service, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=True)))
    def configure(node):
        node.registry = {'map': {'yaml_path': 'map'}, 'keepout': {'yaml_path': 'mask'}}
        node._footprint_outline = Mock(return_value=[
            (-.28, -.275), (.07, -.275), (.07, .275), (-.28, .275)])
    return configure


@pytest.fixture
def mission(monkeypatch):
    """Provide synthetic action peers and a map-bound observation route."""
    route = {'frame_id': 'map', 'map_yaml': 'map', 'keepout_mask_yaml': 'mask',
             'start_pose': {'x': 0.0, 'y': 0.0}, 'max_route_start_distance_m': 0.3,
             'waypoints': [{'id': 'exit'}, {'id': 'observation'}]}
    monkeypatch.setattr(box_service, 'load_route', lambda _: route)
    monkeypatch.setattr(box_service, 'verify_identity', Mock())
    node = object.__new__(BoxServiceRoute)
    node.registry = {'map': {}, 'keepout': {}}
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    for method in ('verify_live_maps', 'emit'):
        setattr(node, method, Mock())
    for method in ('wait_until_ready', '_parking_parameters_ready', 'preflight', 'execute'):
        setattr(node, method, Mock(return_value=True))
    node._precision_collision_ready = Mock(return_value=True)
    # The straight odom approach is exercised on its own; here it succeeds and
    # the gap is judged on the map as before.
    node._straight_final_approach = Mock(return_value=(True, None))
    node.capture_stationary_pose = Mock(side_effect=[((0, 0, 0), {}), ((0.885, 0, 0), {})])
    node.plan_pose = Mock(return_value={'ok': True})
    node.search_rotation = Mock(return_value=True)
    node.confirmation = {'confirmed': True, 'physical_accuracy': 'NOT_MEASURED'}
    node.parking_contract = {'yaw_tolerance_rad': math.radians(1)}
    target = {'x_m': 0.885, 'y_m': 0.0, 'yaw_rad': 0.0,
              'face_center_map_xy_m': (1.0, 0.0), 'outward_normal_map_xy': (-1.0, 0.0)}
    node.observe_target = Mock(return_value=target)
    node._set_observer_perception = Mock(return_value=True)
    return node


def invoke(node, execute=True, *, search=False):
    """Run the table attempt with explicit execution semantics."""
    return node.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=execute,
        candidate_trial=execute, search=search)


def test_visit_enables_depth_perception_only_after_transit(mission):
    assert invoke(mission)

    assert mission._set_observer_perception.call_args_list == [
        ((False,), {}), ((True,), {})]


def test_visit_keeps_transit_when_unneeded_depth_perception_cannot_disable(mission):
    mission._set_observer_perception.side_effect = [False, True]

    assert invoke(mission)

    assert mission.execute.called


@pytest.mark.parametrize('entrypoint', ['visit', 'observation'])
def test_readiness_cause_reaches_box_caller_without_motion(mission, entrypoint):
    mission.wait_until_ready.return_value = False
    mission.last_readiness_failure = 'battery low: voltage=10.192V limit=10.500V'
    with pytest.raises(RuntimeError, match=mission.last_readiness_failure):
        if entrypoint == 'visit':
            invoke(mission)
        else:
            mission.at_observation('route')
    mission.execute.assert_not_called()
    mission.capture_stationary_pose.assert_not_called()


def _rear_home_node(monkeypatch, *, planned=True, parked=True):
    node = object.__new__(BoxServiceRoute)
    home = {'id': 'home_dock', 'x_m': .12, 'y_m': .165, 'yaw_rad': .134,
            'parking_direction': 'reverse', 'approach_offset_m': .6,
            'rear_parking_reference': 'lidar_box', 'target_rear_gap_m': .01}
    node.registry = {'frame_id': 'map', 'home': deepcopy(home)}
    node._localization_bound = lambda _intermediate: contextlib.nullcontext()
    node._plan_with_input_recovery = Mock(return_value={'ok': planned})
    node._reach_staging = Mock(return_value=(.72, .24, .134))
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.run_deadline_s = time.monotonic() + 180
    node.home_rear_geometry = {'chassis': 'measured'}
    node.home_rear_route_path = Path('/existing/route.yaml')
    node.confirmation = {'confirmed': True, 'reference': 'old_staging'}

    def park(*_args, **kwargs):
        assert node.confirmation is None
        node.confirmation = {'confirmed': parked, 'reference': 'live_rear_lidar_face',
                             'nearest_gap_m': kwargs['target_gap_m'],
                             'external_accuracy_verified': False}
        return parked

    node.park_rear_box = Mock(side_effect=park)
    node._reverse_path_valid = Mock(side_effect=AssertionError(
        'a box can occupy the old coordinate endpoint'))
    node.emit = Mock()
    return node, home


@pytest.mark.parametrize('parked', [True, False])
def test_configured_rear_home_stages_then_parks_against_live_box(monkeypatch, parked):
    node, home = _rear_home_node(monkeypatch, parked=parked)
    original = deepcopy(node.registry['home'])
    assert node._go_home_reverse(home, True) is parked
    target = node._plan_with_input_recovery.call_args.args[0]
    assert target['x_m'] == pytest.approx(.12 + .6 * math.cos(.134))
    assert target['y_m'] == pytest.approx(.165 + .6 * math.sin(.134))
    node._reach_staging.assert_called_once()
    node.park_rear_box.assert_called_once()
    assert node.park_rear_box.call_args.kwargs['target_gap_m'] == .01
    assert 0 < node.park_rear_box.call_args.kwargs['timeout_s'] <= 180
    assert node.registry['home'] == original
    assert node.pose_id == 'home_dock'
    node._reverse_path_valid.assert_not_called()
    arrived = [c for c in node.emit.call_args_list if c.args[0] == 'home_arrived']
    assert bool(arrived) is parked
    if parked:
        assert arrived[-1].kwargs['confirmation']['reference'] == 'live_rear_lidar_face'
        assert arrived[-1].kwargs['confirmation']['external_accuracy_verified'] is False


@pytest.mark.parametrize('end_error_m,expected', [
    (math.sqrt(.05 ** 2 * 2), True), (.149, True), (.151, False),
])
def test_rear_home_transit_endpoint_is_not_judged_as_final_parking(
        monkeypatch, end_error_m, expected):
    node, home = _rear_home_node(monkeypatch)
    node.parking_contract = {'xy_tolerance_m': .01}
    node._navigation_ready = Mock(return_value=True)
    node._pose = lambda *_: PoseStamped()
    node._wait = lambda value, *_: value
    node.plan_pose = ServiceRoute.plan_pose.__get__(node)
    node._plan_with_input_recovery = ServiceRoute._plan_with_input_recovery.__get__(node)
    stage_x = home['x_m'] + home['approach_offset_m'] * math.cos(home['yaw_rad'])
    stage_y = home['y_m'] + home['approach_offset_m'] * math.sin(home['yaw_rad'])
    result = ComputePathThroughPoses.Result()
    finish = PoseStamped()
    finish.pose.position.x, finish.pose.position.y = stage_x - end_error_m, stage_y
    result.path.poses = [PoseStamped(), finish]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
    node.compute = Mock()
    node.compute.send_goal_async.return_value = SimpleNamespace(
        accepted=True, get_result_async=lambda: wrapped)
    assert node._go_home_reverse(home, True) is expected
    assert node.park_rear_box.call_count == int(expected)
    if expected:
        assert node.park_rear_box.call_args.kwargs['target_gap_m'] == .01
    assert node.registry['home'] == home


@pytest.mark.parametrize('case', ['plan_only', 'blocked_stage', 'failed_stage',
                                  'operator_stop', 'link_lost', 'sensor_lost', 'timeout'])
def test_rear_home_does_not_dispatch_docking_before_valid_staging(monkeypatch, case):
    node, home = _rear_home_node(monkeypatch, planned=case != 'blocked_stage')
    if case == 'failed_stage':
        node._reach_staging.return_value = None
    elif case == 'operator_stop':
        node.stop_requested = True
    elif case == 'link_lost':
        node._operator_link_ready.return_value = False
    elif case == 'sensor_lost':
        node._guard_failure.return_value = 'scan_stale'
    elif case == 'timeout':
        node.run_deadline_s = time.monotonic() - 1
    assert node._go_home_reverse(home, case != 'plan_only') is (case == 'plan_only')
    node.park_rear_box.assert_not_called()
    if case in ('plan_only', 'blocked_stage'):
        node._reach_staging.assert_not_called()
    assert not any(c.args[0] == 'home_arrived' for c in node.emit.call_args_list)


def test_legacy_coordinate_home_does_not_select_a_nearby_lidar_box(monkeypatch):
    node, home = _rear_home_node(monkeypatch)
    del home['rear_parking_reference']
    legacy = Mock(return_value=True)
    monkeypatch.setattr(ServiceRoute, '_go_home_reverse', legacy)
    assert node._go_home_reverse(home, True)
    legacy.assert_called_once_with(home, True, _map_reapproach=False)
    node.park_rear_box.assert_not_called()


def test_plan_only_does_not_observe_or_move(mission):
    assert invoke(mission, execute=False)
    mission.execute.assert_not_called()
    mission.observe_target.assert_not_called()
    mission._precision_collision_ready.assert_not_called()


@pytest.mark.parametrize('ready', [True, False])
def test_normal_visit_queries_parking_parameters_once_through_preflight(mission, ready):
    mission._parking_parameters_ready.return_value = ready
    mission.preflight.side_effect = mission._parking_parameters_ready
    assert invoke(mission) is ready
    mission._parking_parameters_ready.assert_called_once()
    if not ready:
        mission.execute.assert_not_called()
        mission.observe_target.assert_not_called()


def test_observation_resume_rejects_missing_parking_parameters_before_motion(mission):
    mission._parking_parameters_ready.return_value = False
    with pytest.raises(RuntimeError, match='parking controller parameters unavailable'):
        mission.visit_observed_box(
            'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                          'wheel_outer_width': {'value': 0.540}},
            'table_01', (1, 0), 0.6, execute=True,
            candidate_trial=True, resume_at_observation=True)
    mission._parking_parameters_ready.assert_called_once()
    mission.preflight.assert_not_called()
    mission.execute.assert_not_called()
    mission.observe_target.assert_not_called()


def test_observation_resume_keeps_running_cutoff_without_requiring_new_departure_reserve(
        mission, monkeypatch):
    route = {'frame_id': 'map', 'map_yaml': 'map', 'keepout_mask_yaml': 'mask',
             'start_pose': {'x': 0., 'y': 0.}, 'max_route_start_distance_m': .3,
             'waypoints': [{'id': 'exit'}, {'id': 'observation', 'x': 0., 'y': 0.}]}
    monkeypatch.setattr(box_service, 'load_route', lambda _: route)
    mission._mission_started = False
    mission.minimum_battery_v = 10.5
    mission.service_contract = {'minimum_start_battery_v': 10.8}
    mission.battery_voltage = 10.788
    assert not ServiceRoute._departure_battery_ready(mission)
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540}},
        'table_01', (1, 0), .6, execute=True, candidate_trial=True,
        resume_at_observation=True)
    assert ServiceRoute._departure_battery_ready(mission)
    mission.battery_voltage = 10.49
    assert not ServiceRoute._departure_battery_ready(mission)


def test_transit_alignment_final_sequence(mission):
    assert invoke(mission)
    # The final approach is the straight odom leg, not a planned execute.
    assert mission.execute.call_count == 2
    assert [call.kwargs for call in mission.execute.call_args_list] == [
        {'final_parking': False}, {'final_parking': False, 'alignment': True, 'face': True}]
    mission._straight_final_approach.assert_called_once()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    # The final plan only proves the way is free: one 5 cm costmap cell + 1 cm.
    assert [call.kwargs.get('end_tolerance_m') for call in mission.plan_pose.call_args_list] == [
        0.05, 0.06]
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(0.05)
    assert mission.emit.call_args.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.emit.call_args.kwargs['final_depth_measurement'] is False


def _table_edge_final_target(face_x):
    """Return a map-frame table edge used by final measurement tests."""
    return {
        'x_m': face_x - 0.115,
        'y_m': 0.0,
        'yaw_rad': 0.0,
        'face_center_map_xy_m': (face_x, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
    }


def _run_table_edge_final_measurement(mission, fresh_target):
    """Run table-edge parking with a distinct post-motion observation."""
    mission.surface_mode = 'table_edge'
    before = _table_edge_final_target(0.97)
    mission.observe_target.side_effect = [before, before, fresh_target]

    def stale_odom_transform(*_args):
        raise AssertionError('post-motion table edge reused frozen odom transform')

    mission._straight_final_approach.return_value = (True, stale_odom_transform)
    return invoke(mission)


def test_table_edge_final_depth_rejects_odom_success_at_eight_centimetres(mission):
    assert not _run_table_edge_final_measurement(
        mission, _table_edge_final_target(1.03))

    result = mission.emit.call_args
    assert result.args == ('box_gap_not_confirmed',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.08)
    assert result.kwargs['final_depth_measurement'] is True
    assert result.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.observe_target.call_count == 3


def test_table_edge_final_depth_confirms_fresh_five_centimetres_in_map(mission):
    assert _run_table_edge_final_measurement(
        mission, _table_edge_final_target(1.0))

    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.05)
    assert result.kwargs['final_depth_measurement'] is True
    assert mission.selected_pose == {
        'id': 'final_approach', 'x_m': 0.885, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert mission.last_box_face == {
        'face_center_map_xy_m': [1.0, 0.0],
        'outward_normal_map_xy': [-1.0, 0.0],
    }
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [
        0.45, 0.05, 0.05]


def test_table_edge_final_depth_observation_failure_has_no_success(mission):
    mission.surface_mode = 'table_edge'
    before = _table_edge_final_target(0.97)
    mission.observe_target.side_effect = [
        before,
        before,
        box_service.BoxObservationUnavailable(
            'no_stable_table_edge', retryable=True),
    ]

    assert not invoke(mission)

    result = mission.emit.call_args
    assert result.args == ('failed',)
    assert result.kwargs == {
        'phase': 'final_depth_measurement',
        'reason': 'no_stable_table_edge',
        'final_depth_measurement': False,
    }
    assert not any(call.args == ('box_approach_finished',)
                   for call in mission.emit.call_args_list)
    assert mission.observe_target.call_count == 3


def test_one_centimetre_table_uses_frozen_face_when_fresh_depth_is_occluded(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.observer_stable_frames = 8
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target,
        target,
        box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            fresh_frames=8, required_fresh_frames=8,
            fresh_rejection_reasons={'no_table_edge_candidate'}),
    ]
    mission._straight_final_approach.return_value = (
        True, lambda x, y, yaw: (x, y, yaw))
    mission.capture_stationary_pose.side_effect = [
        ((0.0, 0.0, 0.0), {}), ((0.925, 0.0, 0.0), {})]
    mission._odom_pose = Mock(return_value=(0.925, 0.0, 0.0))

    assert invoke(mission)

    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['confirmation_basis'] == 'frozen_table_edge_odom_stop'
    assert result.kwargs['internal_estimate_only'] is True
    assert result.kwargs['final_depth_measurement'] is False
    assert result.kwargs['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.last_box_face == {
        'reference_frame': 'odom',
        'face_center_xy_m': [1.0, 0.0],
        'outward_normal_xy': [-1.0, 0.0],
    }


def test_occluded_five_millimetre_correction_reuses_original_odom_face(mission, clear_precision_corridor):
    clear_precision_corridor(mission)
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    target = _table_edge_final_target(1.0)

    def occluded():
        return box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            fresh_frames=8, required_fresh_frames=8,
            fresh_rejection_reasons={'no_table_edge_candidate'})
    mission.observe_target.side_effect = [target, target, occluded(), occluded()]
    turn = math.radians(30.0)

    def original_to_odom(x, y, yaw):
        return (2.0 + math.cos(turn) * x - math.sin(turn) * y,
                -1.0 + math.sin(turn) * x + math.cos(turn) * y,
                yaw + turn)

    mission._straight_final_approach.return_value = (True, original_to_odom)
    mission.capture_stationary_pose.side_effect = [
        ((0.0, 0.0, 0.0), {}),
        ((20.0, -10.0, math.radians(25.0)), {}),
        ((20.02, -10.01, math.radians(17.0)), {}),
        ((19.98, -9.99, math.radians(12.0)), {}),
    ]
    face_x, face_y, _ = original_to_odom(1.0, 0.0, 0.0)
    outward = (-math.cos(turn), -math.sin(turn))

    def odom_pose(gap_m):
        centre_distance_m = 0.065 + gap_m
        return (face_x + outward[0] * centre_distance_m,
                face_y + outward[1] * centre_distance_m, turn)

    mission._odom_pose = Mock(side_effect=[
        odom_pose(0.015), odom_pose(0.015), odom_pose(0.010)])
    mission._drive_zero_turn = Mock(return_value=0.005)
    mission._frozen_in_odom = Mock(
        side_effect=AssertionError('correction re-froze the stale map face'))

    assert invoke(mission)

    mission._straight_final_approach.assert_called_once()
    mission._frozen_in_odom.assert_not_called()
    mission._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.005), precision=True, hold_heading=True)
    result = mission.emit.call_args
    assert result.args == ('box_approach_finished',)
    assert result.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(0.01)
    assert result.kwargs['confirmation_basis'] == 'frozen_table_edge_odom_stop'


@pytest.mark.parametrize(
    'kwargs', [
        {'latency_limited': True, 'fresh_frames': 8,
         'fresh_rejection_reasons': {'no_table_edge_candidate'}},
        {'fresh_frames': 7, 'fresh_rejection_reasons': {'no_table_edge_candidate'}},
        {'latency_limited': True, 'fresh_frames': 8,
         'fresh_rejection_reasons': {'no_table_edge_candidate'},
         'latency_reasons': {'observer_silent'}},
        {'fresh_frames': 8, 'fresh_rejection_reasons': {'stale_or_future_depth'}},
    ])
def test_one_centimetre_table_never_uses_odom_for_unhealthy_depth(mission, kwargs):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.observer_stable_frames = 8
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target,
        target,
        box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            required_fresh_frames=8, **kwargs),
    ]
    mission._straight_final_approach.return_value = (
        True, lambda x, y, yaw: (x, y, yaw))

    assert not invoke(mission)
    assert mission.emit.call_args.args == ('failed',)
    assert mission.emit.call_args.kwargs['phase'] == 'final_depth_measurement'


def test_transit_failure_never_observes_or_dispatches_alignment(mission):
    mission.execute.return_value = False
    assert not invoke(mission)
    mission.observe_target.assert_not_called()


def test_resume_observation_skips_transit_but_keeps_both_parking_phases(mission):
    mission.config = {}
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 1.0), {}), ((0.885, 0, 0), {})]
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0, y=0)
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True,
        candidate_trial=True, resume_at_observation=True)
    assert mission.execute.call_count == 1
    mission._straight_final_approach.assert_called_once()
    mission.preflight.assert_not_called()
    mission._parking_parameters_ready.assert_called_once()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]
    mission.verify_live_maps.assert_called()
    mission._precision_collision_ready.assert_called_once()


def test_resume_final_approach_skips_transit_and_face_alignment(mission):
    mission.surface_mode = 'table_edge'
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0, y=0)
    before = _table_edge_final_target(0.97)
    fresh = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [before, fresh]

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True,
        resume_at_observation=True, resume_at_final_approach=True)

    mission.preflight.assert_not_called()
    mission.execute.assert_not_called()
    mission._straight_final_approach.assert_called_once_with(before, 0.065)
    assert mission.observe_target.call_count == 2
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.05, 0.05]
    mission.plan_pose.assert_called_once_with(
        {**before, 'id': 'final_approach', 'priority': 1, 'approach_offset_m': 0.5},
        single=True, end_tolerance_m=box_service.FINAL_PLAN_END_TOLERANCE_M)


def test_resume_final_approach_defaults_off(mission):
    assert inspect.signature(BoxServiceRoute.visit_observed_box).parameters[
        'resume_at_final_approach'].default is False
    assert invoke(mission)
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.05]


def test_resume_observation_rejects_robot_outside_observation_region(mission):
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=1, y=0)
    with pytest.raises(RuntimeError, match='not at the reached observation'):
        mission.visit_observed_box(
            'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                          'wheel_outer_width': {'value': 0.540}},
            'table_01', (1, 0), 0.6, execute=True,
            candidate_trial=True, resume_at_observation=True)
    mission.execute.assert_not_called()
    mission.observe_target.assert_not_called()


def test_missing_face_has_no_parking_dispatch(mission):
    mission.observe_target.side_effect = RuntimeError('no stable face')
    with pytest.raises(RuntimeError, match='no stable face'):
        invoke(mission)
    assert mission.execute.call_count == 1
    mission.plan_pose.assert_not_called()


def test_opt_in_search_rotates_once_then_resumes_parking(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True),
        target,
        target,
    ]
    assert invoke(mission, search=True)
    mission.search_rotation.assert_called_once_with(math.radians(30.0))
    assert mission.execute.call_count == 2
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [
        0.45, 0.45, 0.05]


def test_search_budget_stops_after_one_full_turn(mission):
    mission.observe_target.side_effect = lambda *_args, **_kwargs: (
        _raise_observation('no_stable_box', retryable=True))
    with pytest.raises(RuntimeError, match='search exhausted after 12 rotations'):
        invoke(mission, search=True)
    assert mission.search_rotation.call_count == 12
    assert sum(call.args[0] for call in mission.search_rotation.call_args_list) == (
        pytest.approx(math.tau))
    exhausted = [call for call in mission.emit.call_args_list
                 if call.args == ('box_search_exhausted',)]
    assert len(exhausted) == 1
    assert exhausted[0].kwargs['search_steps_used'] == 12
    assert exhausted[0].kwargs['search_cumulative_yaw_rad'] == pytest.approx(
        math.tau)


def _raise_observation(reason, *, retryable):
    raise box_service.BoxObservationUnavailable(reason, retryable=retryable)


def test_search_does_not_rotate_for_sensor_or_geometry_failure(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable(
            'LiDAR scan is stale or future-dated', retryable=False))
    mission._wait_for_observation_input = Mock(return_value=(False, 'operator_link_lost'))
    with pytest.raises(RuntimeError, match='operator_link_lost'):
        invoke(mission, search=True)
    mission.search_rotation.assert_not_called()
    failure = [call for call in mission.emit.call_args_list
               if call.args == ('box_observation_failed',)][0]
    assert failure.kwargs['retryable'] is False


def test_temporary_observation_input_waits_without_search_then_reobserves(mission):
    target = mission.observe_target.return_value
    error = box_service.BoxObservationUnavailable(
        'no_stable_box', retryable=False, latency_limited=True,
        latency_reasons={'observer_silent'})
    mission.observe_target.side_effect = [error, target, target]
    mission._wait_for_observation_input = Mock(return_value=(True, None))

    assert invoke(mission, search=True)

    assert mission._wait_for_observation_input.call_args.args[0] is error
    mission.search_rotation.assert_not_called()
    assert any(call.args == ('box_observation_input_recovered',)
               for call in mission.emit.call_args_list)


@pytest.mark.parametrize('reason', [
    'stale_or_future_depth',
    'camera_info_missing',
    'LiDAR scan is stale or future-dated',
    'fresh LiDAR scan is required for box witness',
])
def test_depth_and_scan_input_gaps_are_temporary(mission, reason):
    error = box_service.BoxObservationUnavailable(reason, retryable=False)

    assert mission._temporary_observation_input_gap(error) is True


def test_temporary_tf_input_waits_without_search_then_reobserves(mission):
    target = mission.observe_target.return_value
    error = RuntimeError(
        'stationary teaching unavailable: need fresh TF, odometry and localization')
    mission.observe_target.side_effect = [error, target, target]
    mission._input_gap_recoverable = Mock(return_value=True)
    mission._wait_for_observation_input = Mock(return_value=(True, None))

    assert invoke(mission, search=True)

    assert mission._wait_for_observation_input.call_args.args[0] is error
    mission.search_rotation.assert_not_called()


@pytest.mark.parametrize('reason', [
    'LiDAR and depth face normals disagree',
    'LiDAR scan frame must be measured laser_link',
    'observed_box_outside_selected_table_region',
])
def test_geometry_and_calibration_observation_failures_never_wait(mission, reason):
    error = box_service.BoxObservationUnavailable(reason, retryable=True)
    mission.observe_target.side_effect = error
    mission._wait_for_observation_input = Mock(
        side_effect=AssertionError('non-input failure waited'))

    with pytest.raises(box_service.BoxObservationUnavailable):
        invoke(mission)

    mission._wait_for_observation_input.assert_not_called()


def _observation_wait_node():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._input_recovery_paused = False
    node.run_deadline_s = 20.0
    node.box_status_received_s = None
    node.samples = {}
    node.depth_camera_info_received_s = None
    node.depth_camera_info = None
    node.box_status = None
    node.last_scan = None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    node._operator_link_ready = Mock(return_value=True)
    node._departure_battery_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.readiness_failure_reason = Mock(return_value='battery_guard')
    node._input_gap_recoverable = Mock(return_value=False)
    node._escape_publisher = Mock(return_value=Mock())
    return node


def test_observation_input_wait_holds_zero_and_preserves_global_deadline(monkeypatch):
    node = _observation_wait_node()
    clock = [10.0]

    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])

    def deliver(_node, timeout_sec):
        clock[0] += timeout_sec
        node.depth_camera_info_received_s = clock[0]
        node.depth_camera_info = SimpleNamespace(
            header=SimpleNamespace(stamp=_stamp(10.0)))

    monkeypatch.setattr(box_service, 'spin_node', deliver)
    error = box_service.BoxObservationUnavailable(
        'no_stable_box', retryable=False, latency_limited=True,
        latency_reasons={'camera_info_missing'})

    baseline = node._observation_input_snapshot()
    assert node._wait_for_observation_input(error, baseline) == (True, None)
    assert node.run_deadline_s == pytest.approx(20.05)
    assert node._input_recovery_paused is False
    command = node._escape_publisher.return_value.publish.call_args.args[0]
    assert command.linear.x == 0.0 and command.angular.z == 0.0


def test_camera_gap_ignores_unrelated_scan_updates_until_fresh_camera_info(monkeypatch):
    node = _observation_wait_node()
    clock = [10.0]
    spins = [0]
    baseline = node._observation_input_snapshot()

    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])

    def deliver(_node, timeout_sec):
        spins[0] += 1
        clock[0] += timeout_sec
        node.samples['scan'] = clock[0]
        if spins[0] == 4:
            node.depth_camera_info_received_s = clock[0]
            node.depth_camera_info = SimpleNamespace(
                header=SimpleNamespace(stamp=_stamp(10.0)))

    monkeypatch.setattr(box_service, 'spin_node', deliver)
    error = box_service.BoxObservationUnavailable(
        'no_stable_box', retryable=False, latency_limited=True,
        latency_reasons={'camera_info_missing'})

    assert node._wait_for_observation_input(error, baseline) == (True, None)
    assert spins[0] == 4
    assert node.run_deadline_s == pytest.approx(20.2)


def test_observation_wait_sees_input_delivered_after_attempt_before_wait(monkeypatch):
    node = _observation_wait_node()
    clock = [10.0]
    baseline = node._observation_input_snapshot()
    node.depth_camera_info_received_s = 10.01
    node.depth_camera_info = SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(10.0)))
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])
    spin = Mock(side_effect=AssertionError('fresh one-shot input was missed'))
    monkeypatch.setattr(box_service, 'spin_node', spin)
    error = box_service.BoxObservationUnavailable(
        'no_stable_box', retryable=False, latency_limited=True,
        latency_reasons={'camera_info_missing'})

    assert node._wait_for_observation_input(error, baseline) == (True, None)
    spin.assert_not_called()
    node._escape_publisher.return_value.publish.assert_called_once()


def test_mixed_guard_and_observer_gap_recovers_after_fresh_observer(monkeypatch):
    node = _observation_wait_node()
    clock = [10.0]
    baseline = node._observation_input_snapshot()
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])

    def deliver(_node, timeout_sec):
        clock[0] += timeout_sec
        node.box_status_received_s = clock[0]
        node.box_status = {'stamp_s': 10.0, 'reason': 'no_box_surface_candidate'}

    monkeypatch.setattr(box_service, 'spin_node', deliver)
    error = box_service.BoxObservationUnavailable(
        'no_stable_box', retryable=False, latency_limited=True,
        latency_reasons={'input_gap', 'observer_silent'})

    assert node._wait_for_observation_input(error, baseline) == (True, None)
    assert node.run_deadline_s == pytest.approx(20.05)


@pytest.mark.parametrize('failure_kind, expected', [
    ('stop', 'operator_stop'),
    ('link', 'operator_link_lost'),
    ('battery', 'battery_guard'),
    ('guard', 'AMCL covariance high'),
])
def test_observation_input_wait_keeps_safety_stops_terminal(failure_kind, expected):
    node = _observation_wait_node()
    if failure_kind == 'stop':
        node.stop_requested = True
    elif failure_kind == 'link':
        node._operator_link_ready.return_value = False
    elif failure_kind == 'battery':
        node._departure_battery_ready.return_value = False
    else:
        node._guard_failure.return_value = expected
    error = box_service.BoxObservationUnavailable(
        'LiDAR scan is stale or future-dated', retryable=False)

    baseline = node._observation_input_snapshot()
    assert node._wait_for_observation_input(error, baseline) == (False, expected)
    assert node.run_deadline_s == 20.0
    assert node._input_recovery_paused is False


def test_final_approach_observation_loss_never_rotates_near_box(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        target,
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True),
    ]
    with pytest.raises(RuntimeError, match='no_stable_box'):
        invoke(mission, search=True)
    mission.search_rotation.assert_not_called()
    assert mission.execute.call_count == 2


def test_failed_search_rotation_stops_without_repeating(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True))
    mission.search_rotation.return_value = False
    mission.last_search_failure_reason = 'nav2_spin_failed'
    with pytest.raises(RuntimeError, match='search rotation failed'):
        invoke(mission, search=True)
    mission.search_rotation.assert_called_once_with(math.radians(30.0))
    assert mission.observe_target.call_count == 1
    failure = [call for call in mission.emit.call_args_list
               if call.args == ('failed',)][-1]
    assert failure.kwargs['reason'] == 'nav2_spin_failed'


def test_failed_search_rotation_reports_observation_reason(mission):
    mission.observe_target.side_effect = (
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True))
    mission.search_rotation.return_value = False
    mission.last_search_failure_reason = 'rotation_observation_not_confirmed'

    with pytest.raises(RuntimeError, match='search rotation failed'):
        invoke(mission, search=True)

    failure = [call for call in mission.emit.call_args_list
               if call.args == ('failed',)][-1]
    assert failure.kwargs['reason'] == 'rotation_observation_not_confirmed'


def test_collision_blocked_search_can_reposition_once_and_continue(mission):
    target = mission.observe_target.return_value
    mission.observe_target.side_effect = [
        box_service.BoxObservationUnavailable('no_stable_box', retryable=True), target, target]
    mission.search_rotation.return_value = False
    mission._reposition_search = Mock(return_value=True)
    assert invoke(mission, search=True)
    mission._reposition_search.assert_called_once()


@pytest.mark.parametrize('error_name', ['COLLISION_AHEAD', 'TIMEOUT'])
def test_search_reposition_requires_recoverable_result_and_valid_path(mission, error_name):
    from nav2_msgs.action import Spin
    mission.last_search_error_code = Spin.Result.TF_ERROR
    assert not mission._reposition_search()
    mission.plan_pose.assert_not_called()
    mission.last_search_error_code = getattr(Spin.Result, error_name)
    mission.plan_pose.side_effect = [{'ok': False}, {'ok': True}]
    assert mission._reposition_search()
    assert mission.plan_pose.call_count == 2
    mission.execute.assert_called_once_with(final_parking=False, alignment=True, face=True)


def test_failed_alignment_never_dispatches_final_approach(mission):
    mission.execute.side_effect = [True, False]
    assert not invoke(mission)
    assert mission.observe_target.call_count == 1


def test_goal_success_with_wrong_gap_is_not_parking_success(mission):
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), ((0.84, 0, 0), {})]
    assert not invoke(mission)
    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)


def test_center_gap_cannot_hide_rotated_front_corner(mission):
    yaw = math.radians(3)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.95 - 0.065 * math.cos(yaw), 0, yaw), {})]
    assert not invoke(mission)
    result = mission.emit.call_args.kwargs
    assert result['estimated_front_gap_m'] == pytest.approx(0.05)
    assert min(result['estimated_front_corner_gaps_m']) < 0.04


def test_corner_gaps_inside_yaw_tolerance_are_logged_not_gated(mission):
    """2026-09-30 table_02: 2.74 deg, centre 5.7 cm, corners 4.4/7.0 cm was a good park."""
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}
    yaw = math.radians(2.74)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.943 - 0.065 * math.cos(yaw), 0, yaw), {})]
    assert invoke(mission)
    result = mission.emit.call_args.kwargs
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert result['estimated_front_gap_m'] == pytest.approx(0.057)
    assert max(result['estimated_front_corner_gaps_m']) > 0.06
    assert result['gap_confirmation_basis'] == 'nearest_front_corner'
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(3))


def _table_edge_pose_for_corner_gaps(
        left_gap_m, right_gap_m, front_extent_m=0.065, frame_width_m=0.45):
    """Pose facing a face at x=1 with the requested two front-corner gaps."""
    # box_service orders corners as robot-right, then robot-left.
    yaw = -math.asin((right_gap_m - left_gap_m) / frame_width_m)
    centre_gap_m = (left_gap_m + right_gap_m) / 2.0
    return (1.0 - centre_gap_m - front_extent_m * math.cos(yaw), 0.0, yaw)


def test_table_edge_rejects_synthetic_five_and_six_point_five_centimetre_tilt(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.05, 0.065)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.065, 0.05])
    assert result['estimated_face_yaw_error_rad'] == pytest.approx(math.radians(1.910213))
    assert result['gap_confirmation_basis'] == 'both_front_corners'
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(1))


def test_table_edge_rejects_far_corner_over_six_centimetres_below_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.055, 0.061)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_face_yaw_error_rad'] < math.radians(1)
    assert result['estimated_nearest_front_gap_m'] == pytest.approx(0.055)
    assert max(result['estimated_front_corner_gaps_m']) == pytest.approx(0.061)


def test_table_edge_rejects_yaw_over_one_degree_with_both_gaps_in_range(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.045, 0.055)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.055, 0.045])
    assert result['estimated_face_yaw_error_rad'] > math.radians(1)
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(1))


def test_table_edge_preserves_contract_yaw_tighter_than_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(0.5)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.047, 0.053)).kwargs

    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.053, 0.047])
    assert result['estimated_face_yaw_error_rad'] > math.radians(0.5)
    assert result['allowed_face_yaw_error_rad'] == pytest.approx(math.radians(0.5))


def test_table_edge_accepts_both_corners_within_gap_and_one_degree(mission):
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}

    result = _final_judged(
        mission, _table_edge_pose_for_corner_gaps(0.052, 0.058)).kwargs

    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert result['estimated_front_corner_gaps_m'] == pytest.approx([0.058, 0.052])
    assert result['estimated_face_yaw_error_rad'] < math.radians(1)


def _replay_table_edge_arrival(mission, camera_mount, sample):
    """Judge one saved observation at its stationary base-frame origin."""
    observed = sample['observation']
    target = box_service.compute_table_edge_docking_target(
        observed,
        camera_mount,
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        requested_gap_m=0.05,
        front_extent_m=0.065,
        now_s=observed['stamp_s'],
    )
    mission.surface_mode = 'table_edge'
    mission.parking_contract = {'yaw_tolerance_rad': math.radians(3)}
    mission.observe_target.side_effect = None
    mission.observe_target.return_value = target
    return _final_judged(mission, (0.0, 0.0, 0.0))


def test_saved_table_edges_with_provisional_yaw_do_not_confirm_arrival(mission):
    with CAMERA_MOUNT.open(encoding='utf-8') as stream:
        camera_mount = yaml.safe_load(stream)
    with TABLE_EDGE_ALIGNMENT_FIXTURE.open(encoding='utf-8') as stream:
        fixture = json.load(stream)

    for sample in fixture['samples']:
        event = _replay_table_edge_arrival(mission, camera_mount, sample)
        result = event.kwargs
        expected_gaps = [
            sample['measured_right_gap_m'], sample['measured_left_gap_m']]
        physical_heading_error_rad = math.asin(
            (sample['measured_right_gap_m'] - sample['measured_left_gap_m'])
            / fixture['frame_width_m'])

        assert event.args == ('box_gap_not_confirmed',)
        assert all(
            abs(projected - measured) < 0.005
            for projected, measured in zip(
                result['estimated_front_corner_gaps_m'], expected_gaps))
        assert abs(
            result['estimated_face_yaw_error_rad'] - physical_heading_error_rad
        ) < math.radians(0.32)


def test_latest_saved_table_edge_with_zero_yaw_still_passes_new_gate(mission):
    with CAMERA_MOUNT.open(encoding='utf-8') as stream:
        camera_mount = yaml.safe_load(stream)
    with TABLE_EDGE_ALIGNMENT_FIXTURE.open(encoding='utf-8') as stream:
        sample = json.load(stream)['samples'][-1]
    baseline_mount = deepcopy(camera_mount)
    baseline_mount['camera_mount']['transform']['yaw_rad'] = 0.0

    event = _replay_table_edge_arrival(mission, baseline_mount, sample)

    assert event.args == ('box_approach_finished',)
    assert event.kwargs['estimated_face_yaw_error_rad'] < math.radians(1)


def test_wrong_start_never_dispatches_transit(mission):
    mission.capture_stationary_pose.side_effect = [((1, 1, 0), {})]
    with pytest.raises(RuntimeError, match='starting place'):
        invoke(mission)
    mission.execute.assert_not_called()


def test_parser_does_not_enable_execution_by_default():
    args = parse_args([
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303'])
    assert args.execute is False
    assert args.candidate_trial is False


def test_pointcloud_parser_accepts_only_table_edge_approach_tasks():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
        '--pointcloud-perception',
    ]
    assert parse_args([*base, '--surface-mode', 'table_edge']).pointcloud_perception
    for extra in (['--surface-mode', 'front'],
                  ['--surface-mode', 'table_edge', '--home-only'],
                  ['--surface-mode', 'table_edge', '--rear-box-only']):
        with pytest.raises(SystemExit) as error:
            parse_args([*base, *extra])
        assert error.value.code == 2


def test_parser_requires_explicit_candidate_trial_for_execution():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
        '--execute',
    ]
    with pytest.raises(SystemExit):
        parse_args(base)
    assert parse_args([*base, '--candidate-trial']).candidate_trial is True


def test_final_gap_and_dwell_options_are_bounded():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    args = parse_args(base)
    assert args.final_gap_m is None and args.dwell_s == box_service.STOP_DWELL_S
    assert args.surface_mode == 'front'
    assert parse_args([*base, '--surface-mode', 'table_edge']).surface_mode == 'table_edge'
    args = parse_args([*base, '--final-gap-m', '0.02', '--dwell-s', '30'])
    assert (args.final_gap_m, args.dwell_s) == (0.02, 30.0)
    assert parse_args([*base, '--final-gap-m', '0.01']).final_gap_m == 0.01
    assert parse_args([*base, '--dwell-s', '0']).dwell_s == 0.0
    for extra in (['--final-gap-m', '0'], ['--final-gap-m', '0.005'], ['--final-gap-m', '0.2'],
                  ['--final-gap-m', 'nan'], ['--dwell-s', '-1'], ['--dwell-s', '500']):
        with pytest.raises(SystemExit):
            parse_args([*base, *extra])


def test_final_gap_comes_from_the_option_then_the_contract():
    fake = SimpleNamespace(final_gap_override_m=None,
                           parking_contract={'target_front_gap_m': 0.015})
    assert BoxServiceRoute._final_gap_m(fake) == pytest.approx(0.015)
    fake.final_gap_override_m = 0.02
    assert BoxServiceRoute._final_gap_m(fake) == pytest.approx(0.02)
    older = SimpleNamespace(parking_contract={})
    assert BoxServiceRoute._final_gap_m(older) == pytest.approx(0.05)
    table = SimpleNamespace(final_gap_override_m=None, surface_mode='table_edge',
                            parking_contract={'target_front_gap_m': 0.01})
    assert BoxServiceRoute._final_gap_m(table) == pytest.approx(0.01)
    table.final_gap_override_m = 0.02
    assert BoxServiceRoute._final_gap_m(table) == pytest.approx(0.02)


@pytest.mark.parametrize('flag', ['--search', '--resume-at-observation'])
def test_search_and_resume_require_explicit_execution(flag):
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    with pytest.raises(SystemExit):
        parse_args([*base, flag])
    args = parse_args([*base, '--execute', '--candidate-trial', flag])
    assert getattr(args, flag.removeprefix('--').replace('-', '_')) is True


def test_final_approach_resume_requires_complete_table_edge_resume_flags():
    base = [
        '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
        '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
        '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
    ]
    execute = [*base, '--execute', '--candidate-trial']
    valid = parse_args([
        *execute, '--resume-at-observation', '--resume-at-final-approach',
        '--surface-mode', 'table_edge'])
    assert valid.resume_at_final_approach is True
    for invalid in (
            [*base, '--resume-at-final-approach'],
            [*execute, '--resume-at-final-approach'],
            [*execute, '--resume-at-observation', '--resume-at-final-approach']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def test_direct_execution_requires_candidate_before_runtime_or_motion(mission):
    with pytest.raises(RuntimeError, match='explicit nominal-camera candidate'):
        mission.visit_observed_box(
            'route', {}, {'front_to_wheel_axis': {'value': 0.065}},
            'table_01', (1, 0), 0.6, execute=True,
            candidate_trial=False)
    mission._precision_collision_ready.assert_not_called()
    mission.execute.assert_not_called()


def test_bad_runtime_precision_profile_blocks_before_execute(mission):
    mission._precision_collision_ready.return_value = False
    with pytest.raises(RuntimeError, match='precision collision-monitor'):
        invoke(mission)
    mission.execute.assert_not_called()


def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int((seconds - whole) * 1e9))


def _observer_node(monkeypatch, *, scan=True, observation_stamp=10.1):
    node = object.__new__(BoxServiceRoute)
    node.capture_stationary_pose = Mock(return_value=((0.0, 0.0, 0.0), {}))
    node.parking_motion_revision = 0
    node.candidate_trial = True
    node.surface_mode = 'front'
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.emit = Mock()
    node._configure_observer = Mock(return_value=True)
    node._observer_stable_frame_quorum = Mock(return_value=8)
    node.box_geometry = yaml.safe_load(GEOMETRY.read_text())
    node.box_status = {
        'frame_id': 'camera_color_optical_frame',
        'stamp_s': observation_stamp,
        'detected': True,
        'stable': True,
        'surface_kind': 'front',
        'front_distance_m': 0.5,
        'lateral_error_m': 0.0,
        'plane_normal': [0.0, 0.0, -1.0],
        'confidence': 0.9,
        'control_ready': False,
    }
    node.last_scan = (SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(10.1), frame_id='laser_link'),
        ranges=[0.5] * 10, angle_min=-0.1, angle_increment=0.02,
        range_min=0.05, range_max=12.0) if scan else None)
    clock_values = iter((10.0, 10.2))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))
    monkeypatch.setattr(box_service.rclpy, 'spin_once', lambda *_a, **_k: None)
    monkeypatch.setattr(box_service, 'compute_box_docking_target', Mock(
        return_value={
            'x_m': 0.385, 'y_m': 0.0, 'yaw_rad': 0.0,
            'estimate_gap_m': 0.05,
            'face_center_map_xy_m': (0.5, 0.0),
            'outward_normal_map_xy': (-1.0, 0.0),
            'provenance': 'NOMINAL_CAMERA_MOUNT_ESTIMATE',
        }))
    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', Mock(
        return_value={
            'fused_face_center_map_xy_m': (0.51, 0.01),
            'outward_normal_map_xy': (-1.0, 0.0),
            'residual_rms_m': 0.005,
            'distance_difference_m': -0.01,
            'normal_yaw_difference_rad': 0.01,
            'support_count': 8,
            'tangent_spread_m': 0.12,
            'provenance': 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY',
        }))
    return node


def test_observe_target_preserves_global_deadline_during_initial_input_recovery(
        monkeypatch):
    node = _observer_node(monkeypatch)
    clock = [20.0]
    node.run_deadline_s = 21.0
    node._input_recovery_paused = False
    node._navigation_ready.side_effect = [False] + [True] * 10
    node._guard_failure.return_value = 'scan stale: age=1.000s limit=0.500s'
    node._input_gap_recoverable = Mock(return_value=True)

    def recover(_reason):
        assert node._input_recovery_paused is True
        clock[0] += 2.0
        node.box_status['stamp_s'] = 12.1
        node.last_scan.header.stamp = _stamp(12.1)
        return True

    node._wait_for_input_recovery = Mock(side_effect=recover)
    ros_clock = iter((10.0, 12.2))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(ros_clock) * 1e9)))
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])

    target = node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)

    assert target['candidate_trial'] is True
    assert node.run_deadline_s == pytest.approx(23.0)
    assert node._input_recovery_paused is False


def test_lidar_witness_rejection_records_replayable_scan_once(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.ranges = [0.5, math.inf, math.nan, 0.6]

    def reject(*_args, diagnostics, **_kwargs):
        diagnostics.update({
            'valid_scan_points': 2,
            'support_count': 1,
            'plane_band_only_count': 1,
            'tangent_band_only_count': 2,
            'closest_normal_distance_m': 0.07,
        })
        raise ValueError('LiDAR face support is below five points')

    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', reject)
    spin_count = 0

    def spin(*_args, **_kwargs):
        nonlocal spin_count
        spin_count += 1
        if spin_count == 2:
            node.stop_requested = True

    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    clock_values = iter((10.0, 10.2, 10.2))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=int(next(clock_values) * 1e9)))
    with pytest.raises(RuntimeError, match='below five'):
        node.observe_target({'height_m': 0.215}, 0.065, 0.05, (0.5, 0.0), 0.6)
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert len(rejected) == 1
    evidence = rejected[0].kwargs
    assert evidence['scan_frame'] == 'laser_link'
    assert evidence['scan_ranges'] == [0.5, None, None, 0.6]
    assert evidence['scan_ranges_truncated'] is False
    assert evidence['diagnostics']['support_count'] == 1
    assert evidence['observation']['stamp_s'] == 10.1
    assert evidence['target']['face_center_map_xy_m'] == [0.5, 0.0]
    assert evidence['robot_pose'] == {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert evidence['camera_mount'] == {'height_m': 0.215}
    assert evidence['box_geometry'] == BoxServiceRoute._json_evidence(
        node.box_geometry)
    assert evidence['reason'] == 'LiDAR face support is below five points'


def test_wrong_lidar_frame_is_rejected_before_witness(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.header.frame_id = 'base_link'
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='measured laser_link'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert rejected
    assert rejected[-1].kwargs['scan_frame'] == 'base_link'


def test_nonfinite_scan_scalar_keeps_original_rejection_reason(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.angle_min = math.nan
    box_service.witness_box_face_with_lidar.side_effect = ValueError(
        'angle_min must be finite')
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable,
                       match='angle_min must be finite') as caught:
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert caught.value.retryable is False
    rejected = [call for call in node.emit.call_args_list
                if call.args == ('box_lidar_witness_rejected',)]
    assert len(rejected) == 1
    evidence = rejected[0].kwargs
    assert evidence['scan_angle_min'] is None
    assert evidence['reason'] == 'angle_min must be finite'
    json.dumps(evidence, allow_nan=False)


def test_scan_evidence_is_bounded_to_nine_thousand_ranges():
    values, truncated = BoxServiceRoute._json_scan_ranges(
        (float(index) for index in range(9001)))
    assert len(values) == 9000
    assert values[0] == 0.0
    assert values[-1] == 8999.0
    assert truncated is True


def test_fused_lidar_face_recomputes_final_goal_and_preserves_provenance(
        monkeypatch):
    node = _observer_node(monkeypatch)
    result = node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert result['x_m'] == pytest.approx(0.395)
    assert result['y_m'] == pytest.approx(0.01)
    assert result['yaw_rad'] == pytest.approx(0.0)
    assert result['face_center_map_xy_m'] == (0.51, 0.01)
    assert result['depth_target_provenance'] == 'NOMINAL_CAMERA_MOUNT_ESTIMATE'
    assert result['lidar_witness']['support_count'] == 8
    assert result['candidate_trial'] is True
    assert result['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert [call.args for call in node.emit.call_args_list].count(
        ('box_target_observed',)) == 1


def test_table_edge_target_succeeds_without_lidar_face(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node.box_status['surface_kind'] = 'table_edge'
    expected = {
        'x_m': 0.385, 'y_m': 0.0, 'yaw_rad': 0.0,
        'estimate_gap_m': 0.05,
        'face_center_map_xy_m': (0.5, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
        'surface_kind': 'table_edge', 'table_height_m': 0.75,
        'edge_basis_map_xy': (0.0, 1.0), 'edge_rms_m': 0.003,
        'provenance': 'TABLE_EDGE_CAMERA_MOUNT_ESTIMATE',
    }
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target', Mock(return_value=expected))
    result = node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert result is expected
    assert result['front_extent_m'] == pytest.approx(0.065)
    assert result['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    box_service.compute_table_edge_docking_target.assert_called_once()
    box_service.witness_box_face_with_lidar.assert_not_called()
    observed = [call for call in node.emit.call_args_list
                if call.args == ('box_target_observed',)]
    assert observed[-1].kwargs['target_basis'] == 'tabletop_front_edge'
    assert observed[-1].kwargs['lidar_face_fusion'] is False


@pytest.mark.parametrize('known_heading', [False, True])
def test_table_edge_observer_reports_fresh_close_occlusion_frames(monkeypatch, known_heading):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node.observer_stable_frames = 8
    if known_heading:
        node._known_table_heading_rad = 0.
    node.box_status.update(
        detected=False, stable=False, surface_kind=None,
        reason='no_table_edge_candidate')
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target',
        Mock(side_effect=ValueError(
            'stable detected table_edge surface is required')))
    frames = iter(range(8))

    def spin(*_args, **_kwargs):
        try:
            frame = next(frames)
        except StopIteration:
            node.stop_requested = True
            return
        node.box_status['stamp_s'] = 10.1 + frame * 0.01

    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    clock_values = iter([10.0] + [10.2] * 8)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))

    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.01, (0.5, 0.0), 0.6)

    error = caught.value
    assert error.reason == 'stable detected table_edge surface is required'
    assert error.fresh_frames == 8
    assert error.required_fresh_frames == 8
    assert error.fresh_rejection_reasons == {'no_table_edge_candidate'}
    assert not error.latency_reasons
    assert node.stop_requested is not known_heading


@pytest.mark.parametrize(
    'yaml_frames,runtime_frames,fresh_frames,accepted',
    [(8, 4, 4, True), (4, 8, 4, False)])
def test_close_occlusion_uses_runtime_observer_quorum(
        monkeypatch, yaml_frames, runtime_frames, fresh_frames, accepted):
    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.return_value = 'future'
    node = object.__new__(BoxServiceRoute)
    node.observer_stable_frames = yaml_frames
    node.observer_parameters = client
    node._wait = Mock(return_value=SimpleNamespace(values=[runtime_frames]))
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)

    quorum = node._observer_stable_frame_quorum()
    error = box_service.BoxObservationUnavailable(
        'stable detected table_edge surface is required', retryable=True,
        fresh_frames=fresh_frames, required_fresh_frames=quorum,
        fresh_rejection_reasons={'no_table_edge_candidate'})

    assert node._fresh_table_edge_occlusion(error) is accepted
    client.get_parameters.assert_called_once_with(['stable_frames'])


def test_close_occlusion_is_blocked_when_observer_parameter_service_is_down():
    client = Mock()
    client.wait_for_services.return_value = False
    node = object.__new__(BoxServiceRoute)
    node.observer_parameters = client
    error = box_service.BoxObservationUnavailable(
        'stable detected table_edge surface is required', retryable=True,
        fresh_frames=8, required_fresh_frames=node._observer_stable_frame_quorum(),
        fresh_rejection_reasons={'no_table_edge_candidate'})

    assert not node._fresh_table_edge_occlusion(error)
    client.get_parameters.assert_not_called()


def test_table_edge_observer_parameter_failure_blocks_before_consuming_status(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node._configure_observer.return_value = False
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock())
    with pytest.raises(RuntimeError, match='observer surface and camera parameters'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_table_edge_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_front_mode_switch_failure_also_blocks_a_cli_attempt(monkeypatch):
    node = _observer_node(monkeypatch)
    node.surface_mode = 'front'
    node.require_observer_configuration = True
    node._configure_observer.return_value = False
    with pytest.raises(RuntimeError, match='observer surface and camera parameters'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_box_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_stale_table_edge_never_reaches_target_geometry(monkeypatch):
    node = _observer_node(monkeypatch, scan=False, observation_stamp=9.9)
    node.surface_mode = 'table_edge'
    node.box_status['surface_kind'] = 'table_edge'
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock())
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='no_stable_box'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.compute_table_edge_docking_target.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_observation_consumer_requires_explicit_candidate(monkeypatch):
    node = _observer_node(monkeypatch)
    node.candidate_trial = False
    with pytest.raises(RuntimeError, match='explicit candidate trial'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    node.capture_stationary_pose.assert_not_called()
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_missing_scan_never_returns_final_target(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='fresh LiDAR scan'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


def test_stale_scan_never_returns_final_target(monkeypatch):
    node = _observer_node(monkeypatch)
    node.last_scan.header.stamp = _stamp(9.0)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='LiDAR scan is stale'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


def test_observer_false_control_flag_requires_separate_candidate_gate(monkeypatch):
    node = _observer_node(monkeypatch)
    node.box_status['control_ready'] = True
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='perception-only'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_motion_during_sensor_witness_discards_fused_target(monkeypatch):
    node = _observer_node(monkeypatch)

    def witness(*_args, **_kwargs):
        node.parking_motion_revision += 1
        return {
            'fused_face_center_map_xy_m': (0.51, 0.01),
            'outward_normal_map_xy': (-1.0, 0.0),
            'support_count': 8,
            'provenance': 'LIDAR_DEPTH_FACE_AGREEMENT_NOT_EXTERNAL_ACCURACY',
        }

    monkeypatch.setattr(box_service, 'witness_box_face_with_lidar', witness)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='moved during stationary box observation'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)


@pytest.mark.parametrize('repeated_motion', [False, True])
def test_stationary_window_recaptures_once_and_discards_pre_motion_depth(
        monkeypatch, repeated_motion):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node._ensure_depth_info = Mock()
    node._set_depth_window = Mock(return_value=True)
    node._observation_pose_diagnostic = Mock()
    node._station_anchor_target = lambda target: target
    node._observer_stable_frame_quorum = Mock(return_value=1)
    node.capture_stationary_pose.side_effect = [
        ((0., 0., 0.), {}), ((.01, 0., 0.), {})]
    clock = [10.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(clock[0] * 1e9)))
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])
    spins = [0]

    def advance(*_args, **_kwargs):
        spins[0] += 1
        clock[0] += .1
        if spins[0] == 1 or (spins[0] == 2 and repeated_motion):
            node.parking_motion_revision += 1
        if spins[0] >= 3:
            node.box_status = {**node.box_status, 'stamp_s': clock[0] - .01}

    monkeypatch.setattr(box_service, 'spin_node', advance)
    target = {'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
              'face_center_map_xy_m': [.5, 0.], 'outward_normal_map_xy': [-1., 0.]}
    compute = Mock(return_value=target)
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', compute)

    if repeated_motion:
        with pytest.raises(RuntimeError, match='robot moved during stationary'):
            node.observe_target({}, .065, .01, (.5, 0.), .6, timeout_s=1.)
        compute.assert_not_called()
    else:
        result = node.observe_target({}, .065, .01, (.5, 0.), .6, timeout_s=1.)
        assert spins[0] == 3
        assert result['observation_robot_pose_map'] == [.01, 0., 0.]
        assert compute.call_args.args[0]['stamp_s'] > 10.2
    assert node.capture_stationary_pose.call_count == 2
    assert node.capture_stationary_pose.call_args.kwargs['timeout_s'] <= 1.


def test_stale_depth_observation_never_reaches_fusion(monkeypatch):
    node = _observer_node(monkeypatch, observation_stamp=9.9)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(RuntimeError, match='no_stable_box'):
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    box_service.witness_box_face_with_lidar.assert_not_called()


def test_fresh_no_box_status_is_retryable_by_heading_search(monkeypatch):
    node = _observer_node(monkeypatch)
    node.box_status.update(
        detected=False, stable=False, surface_kind=None,
        reason='no_box_surface_candidate')
    box_service.compute_box_docking_target.side_effect = ValueError(
        'stable detected front surface is required')
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.05, (0.5, 0.0), 0.6)
    assert caught.value.retryable is True
    box_service.witness_box_face_with_lidar.assert_not_called()


@pytest.mark.parametrize('reason', [
    'angle_min must be finite', 'angle_increment must be nonzero',
    'laser range bounds are invalid', 'ranges must not be empty',
    'scan has no valid ranges', 'geometry is missing laser mounting data',
])
def test_invalid_witness_never_requests_search_rotation(monkeypatch, reason):
    node = _observer_node(monkeypatch)
    node.search_rotation = Mock()
    box_service.witness_box_face_with_lidar.side_effect = ValueError(reason)
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node._observe_with_search(
            {}, 0.065, 0.45, (0.5, 0.0), 0.6,
            phase='face_alignment', search_enabled=True,
            search_budget=box_service.BoxSearchBudget())
    assert caught.value.retryable is False
    node.search_rotation.assert_not_called()


@pytest.mark.parametrize('stamp_s', [10.01, 11.0, math.nan])
def test_stale_or_future_no_box_does_not_enable_search(monkeypatch, stamp_s):
    node = _observer_node(monkeypatch, observation_stamp=stamp_s)
    node.box_status.update(detected=False, stable=False, surface_kind=None)
    clock_values = iter((10.0, 10.7))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(next(clock_values) * 1e9)))
    monkeypatch.setattr(
        box_service.rclpy, 'spin_once',
        lambda *_a, **_k: setattr(node, 'stop_requested', True))
    with pytest.raises(box_service.BoxObservationUnavailable) as caught:
        node.observe_target({}, 0.065, 0.45, (0.5, 0.0), 0.6)
    assert caught.value.retryable is False
    box_service.compute_box_docking_target.assert_not_called()


def test_scan_callback_preserves_parent_freshness_and_full_message():
    node = object.__new__(BoxServiceRoute)
    node.samples = {'scan': None}
    node.last_scan = None
    message = SimpleNamespace()
    node._scan_callback(message)
    assert node.samples['scan'] is not None
    assert node.last_scan is message


@pytest.mark.parametrize('polygon_name', [
    'StopZone.translation_forward.points',
    'StopZone.translation_backward.points',
])
def test_live_precision_profile_matches_generated_candidate(monkeypatch, polygon_name):
    baseline = yaml.safe_load(PARAMS.read_text())
    geometry = yaml.safe_load(GEOMETRY.read_text())
    candidate = apply_docking_stop_profile(baseline, geometry)
    monitor = candidate['collision_monitor']['ros__parameters']

    def nested(name):
        value = monitor
        for component in name.split('.'):
            value = value[component]
        return deepcopy(value)

    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.side_effect = lambda names: list(names)
    node = object.__new__(BoxServiceRoute)
    node.precision_parameters = client
    node._wait = lambda names, _timeout: SimpleNamespace(
        values=[nested(name) for name in names])
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)
    monkeypatch.setattr(
        box_service, 'get_package_share_directory',
        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    assert node._precision_collision_ready(geometry)

    original_wait = node._wait
    node._wait = lambda names, timeout: SimpleNamespace(values=[
        ('[[0.135, 0.29], [0.135, -0.29], [-0.295, -0.29], '
         '[-0.295, 0.29]]') if name == polygon_name
        else nested(name) for name in names])
    assert not node._precision_collision_ready(geometry)
    node._wait = original_wait


def _done(value):
    future = Future()
    future.set_result(value)
    return future


def _transit_planner(node, end_xy):
    """Answer route preflight with one successful plan that ends at end_xy."""
    result = ComputePathThroughPoses.Result()
    start, finish = PoseStamped(), PoseStamped()
    finish.pose.position.x, finish.pose.position.y = end_xy
    result.path.poses = [start, finish]
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
    node.compute = Mock()
    node.compute.wait_for_server.return_value = True
    node.compute.send_goal_async.return_value = _done(SimpleNamespace(
        accepted=True, get_result_async=lambda: _done(wrapped)))
    node.navigation_profile = 'obstacle_base_candidate'
    node.get_logger = lambda: Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10 ** 12))
    route = box_service.load_route('route')
    route['waypoints'] = [
        {'id': 'home_exit', 'x': -0.255, 'y': 0.2, 'yaw': 1.61},
        {'id': 'table_01_observation', 'x': 1.45, 'y': 0.25, 'yaw': 0.0}]
    del node.preflight
    return route


def test_t17_box_preflight_rejects_transit_end_shortfall(mission):
    """Reject a transit plan that ends 0.26 m short of the observation pose."""
    _transit_planner(mission, (1.19, 0.25))
    result = invoke(mission, execute=False)
    failed = [call.kwargs for call in mission.emit.call_args_list
              if call.args == ('failed',) and call.kwargs.get('phase') == 'preflight']
    assert result is False and failed, (
        f'HEADFAIL[T17]: 0.26 m transit end shortfall passed preflight ({result})')
    assert failed[0]['end_error_m'] == pytest.approx(0.26)
    mission.execute.assert_not_called()


def test_t17_corridor_preflight_keeps_same_end_shortfall(mission):
    """Keep the base corridor preflight verdict for the same short plan."""
    route = _transit_planner(mission, (1.19, 0.25))
    mission.config, mission.waypoints = route, route['waypoints']
    assert CorridorRoute.preflight(mission) is True


def _latency_limited(reason='stale_or_future_depth'):
    error = box_service.BoxObservationUnavailable(reason, retryable=False)
    error.latency_limited = True
    return error


def _search(node):
    return node._observe_with_search(
        {}, 0.065, 0.45, (1, 0), 0.6, phase='face_alignment', search_enabled=True,
        search_budget=box_service.BoxSearchBudget())


@pytest.mark.parametrize('case', ['ok', 'twice', 'stop'])
def test_t20_latency_reobserve(mission, case):
    """Re-observe one latency-limited window in place; never after a stop."""
    target = mission.observe_target.return_value
    mission._temporary_observation_input_gap = Mock(return_value=False)
    if case == 'ok':
        mission.observe_target.side_effect = [_latency_limited(), target]
        try:
            result = _search(mission)
        except box_service.BoxObservationUnavailable as error:
            raise AssertionError(
                f'HEADFAIL[T20]: latency-limited window was not re-observed: '
                f'{error.reason}') from error
        assert result is target
        assert mission.observe_target.call_count == 2
        assert [call.args for call in mission.emit.call_args_list].count(
            ('box_observation_latency_reobserve',)) == 1
    elif case == 'twice':
        mission.observe_target.side_effect = [_latency_limited(), _latency_limited()]
        with pytest.raises(box_service.BoxObservationUnavailable) as caught:
            _search(mission)
        assert mission.observe_target.call_count == 2, (
            f'HEADFAIL[T20]: observations={mission.observe_target.call_count}')
        assert caught.value.retryable is False
    else:
        mission.stop_requested = True
        mission.observe_target.side_effect = [
            box_service.BoxObservationUnavailable('no_stable_box', retryable=True), target]
        try:
            _search(mission)
            raised = None
        except box_service.BoxObservationUnavailable as error:
            raised = error
        assert (mission.search_rotation.call_count == 0
                and mission.observe_target.call_count == 1), (
            f'HEADFAIL[T20]: stop request rotated={mission.search_rotation.call_count} '
            f'observed={mission.observe_target.call_count}')
        assert raised is not None
        failed = [call.kwargs for call in mission.emit.call_args_list
                  if call.args == ('box_observation_failed',)]
        assert failed and failed[-1]['stop_requested'] is True
    mission.search_rotation.assert_not_called()


RETURN_ARGS = [
    '--registry', '/r', '--approach-route', '/a', '--camera-mount', '/c',
    '--geometry', '/g', '--log', '/l', '--parking-contract', '/p',
    '--table-id', 'table_01', '--region-xy', '1.896', '0.303',
]


def _parse_new(tag, argv):
    try:
        return parse_args(argv)
    except SystemExit as error:
        raise AssertionError(f'NEW[{tag}]: parser rejected {argv[-4:]}') from error


def test_t28_return_cli_requires_explicit_timeouts():
    """Require --execute and an explicit return budget for the home return."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial']
    args = _parse_new('T28', [*execute, '--return-home', '--return-timeout-s', '500'])
    assert args.return_home is True and args.return_timeout_s == 500.0
    assert args.task_timeout_s == 240.0
    assert _parse_new('T28', [*execute, '--task-timeout-s', '300']).task_timeout_s == 300.0
    for invalid in ([*execute, '--return-home'],
                    [*RETURN_ARGS, '--return-home', '--return-timeout-s', '500'],
                    [*execute, '--return-home', '--return-timeout-s', 'nan'],
                    [*execute, '--task-timeout-s', '0']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


@pytest.mark.parametrize('cleanup_failure', [False, True])
def test_resident_result_is_published_atomically_after_attempt(monkeypatch, tmp_path,
                                                              cleanup_failure):
    spool = tmp_path / 'spool'
    spool.mkdir()
    (spool / 'run.request').write_text(json.dumps({'argv': []}))
    handlers = {}

    def signal_handler(signum, handler):
        previous = handlers.get(signum)
        handlers[signum] = handler
        return previous

    monkeypatch.setattr(box_service.signal, 'signal', signal_handler)
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **k: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(box_service, 'parse_args', lambda argv: object())

    def attempt(args, active):
        handlers[box_service.signal.SIGTERM]()
        if cleanup_failure:
            raise RuntimeError('camera stop unconfirmed')
        return 0

    monkeypatch.setattr(box_service, 'run_attempt', attempt)
    replace = Path.replace
    publications = []

    def publish(source, destination):
        assert not destination.exists()
        publications.append(json.loads(source.read_text()))
        return replace(source, destination)

    monkeypatch.setattr(Path, 'replace', publish)
    assert box_service.serve(spool) == 0
    expected = {'code': 1 if cleanup_failure else 0}
    assert publications == [expected]
    assert json.loads((spool / 'run.result').read_text()) == expected
    assert sorted(p.name for p in spool.iterdir()) == ['run.lock', 'run.result']


@pytest.mark.parametrize('camera_stop_ok', [True, False])
@pytest.mark.parametrize('incomplete,stopped,linked', [
    (False, False, True), (True, False, True), (True, True, True), (True, False, False)])
def test_failed_visit_returns_only_after_confirmed_incomplete_stop(
        monkeypatch, tmp_path, camera_stop_ok, incomplete, stopped, linked):
    """Dwell and return only after a confirmed approach; exit 0 needs every step."""
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial',
            '--return-home', '--return-timeout-s', '500']
    _parse_new('T28', [*argv, '--log', str(tmp_path / 'probe.jsonl')])
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        visit_ok = False

        def __init__(self, *args, **kwargs):
            self.kwargs = kwargs
            self.table_id = None
            self.visit_observed_box = Mock(return_value=Route.visit_ok)
            self.dwell_and_return_home = Mock(return_value=True)
            self.go_home = Mock(return_value=True)
            self.incomplete_gap_stop = incomplete and not Route.visit_ok
            self.stop_requested = stopped and not Route.visit_ok
            self._operator_link_ready = Mock(return_value=linked)
            self._navigation_ready = Mock(return_value=True)
            self._camera = Mock(return_value=camera_stop_ok)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    for visit_ok, expected in ((False, 1), (True, 0)):
        Route.visit_ok = visit_ok
        log = tmp_path / f'visit_{visit_ok}.jsonl'
        if camera_stop_ok:
            assert box_service.main([*argv, '--log', str(log)]) == expected
        else:
            with pytest.raises(RuntimeError, match='camera stop unconfirmed'):
                box_service.main([*argv, '--log', str(log)])
    failed, succeeded = created
    failed.dwell_and_return_home.assert_not_called()
    failed.go_home.assert_not_called()
    # Every attempt ends with the box camera off, the failed one included.
    for node in created:
        assert node.camera_on_demand is True
        node._camera.assert_called_with(False)
        node.destroy_node.assert_called_once_with()
        if not camera_stop_ok:
            node.emit.assert_called_with('failed', phase='cleanup', reason='camera_stop_failed')
    # Failed and successful attempts both finish without a notification callback.
    failed.call_operator.assert_not_called()
    succeeded.call_operator.assert_not_called()
    succeeded.dwell_and_return_home.assert_called_once_with(0.0, 500.0)
    assert succeeded.visit_observed_box.call_args.kwargs['task_timeout_s'] == 240.0
    assert succeeded.kwargs['home_contract']['xy_tolerance_m'] == 0.05


def test_t28_final_execute_releases_task_deadline(mission):
    """Release the task deadline after the final execute, before final capture."""
    assert 'task_timeout_s' in inspect.signature(
        BoxServiceRoute.visit_observed_box).parameters, 'NEW[T28]: no task_timeout_s'
    seen = []
    mission.execute.side_effect = lambda **_kwargs: seen.append(
        ('execute', mission.run_deadline_s)) or True
    poses = iter([((0, 0, 0), {}), ((0.885, 0, 0), {})])

    def capture(*_args, **_kwargs):
        seen.append(('capture', getattr(mission, 'run_deadline_s', None)))
        return next(poses)

    mission.capture_stationary_pose = Mock(side_effect=capture)
    started_s = time.monotonic()
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True,
        task_timeout_s=300.0)
    assert [kind for kind, _deadline in seen] == [
        'capture', 'execute', 'execute', 'capture']
    deadlines = [deadline for kind, deadline in seen if kind == 'execute']
    assert deadlines[0] - started_s == pytest.approx(300.0, abs=5.0)
    assert seen[-1] == ('capture', None)
    assert mission.selected_pose['id'] == 'final_approach'


def test_l9_home_contract_load_failure_uses_failure_path(monkeypatch, tmp_path, capsys):
    """Report an invalid dock contract through main's JSON failure path."""
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    (share / 'config/parking_contract.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--log', str(tmp_path / 'run.jsonl')]
    monkeypatch.setattr(box_service, 'get_package_share_directory', lambda _name: str(share))
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []
    monkeypatch.setattr(box_service, 'BoxServiceRoute',
                        lambda *args, **kwargs: created.append(kwargs))
    try:
        code = box_service.main(argv)
    except ValueError as error:
        raise AssertionError(f'REVIEW[L9]: dock contract error escaped main: {error}') from error
    assert code == 1
    lines = capsys.readouterr().out.strip().splitlines()
    assert json.loads(lines[-1])['failed'].startswith('parking contract keys invalid')
    assert created == []


VIA_ARGS = ['--via-id', 'water_station', '--via-route', '/w',
            '--via-region-xy', '-0.14', '-1.53']


def test_t36_via_cli_is_all_or_nothing_and_ends_at_the_dock():
    """A first stop needs its route and region, execution and the dock return."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial',
               '--return-home', '--return-timeout-s', '500']
    args = _parse_new('T36', [*execute, *VIA_ARGS, '--via-region-radius-m', '0.35'])
    assert (args.via_id, str(args.via_route), args.via_region_xy,
            args.via_region_radius_m) == ('water_station', '/w', [-0.14, -1.53], 0.35)
    assert _parse_new('T36', execute).via_id is None
    for invalid in ([*execute, *VIA_ARGS[:2]],
                    [*execute, *VIA_ARGS[:4]],
                    [*RETURN_ARGS, '--execute', '--candidate-trial', *VIA_ARGS],
                    [*execute, *VIA_ARGS, '--via-region-radius-m', '0.7']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def _via_main(monkeypatch, tmp_path, via_ok=True, leave_ok=True, battery_low=False,
              profiles=False):
    for name in ('registry', 'mount', 'geometry', 'route', 'water'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.5', '-1.49', '--execute', '--candidate-trial', '--search',
            '--return-home', '--return-timeout-s', '500',
            '--via-id', 'water_station', '--via-route', str(tmp_path / 'water.yaml'),
            '--via-region-xy', '-0.14', '-1.53']
    if profiles:
        profile_path = tmp_path / 'profiles.json'
        profile_path.write_text('{}\n')
        argv.extend(['--station-profiles', str(profile_path)])
    _parse_new('T36', [*argv, '--log', str(tmp_path / 'probe.jsonl')])
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    calls = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.table_id = None
            self.visit_observed_box = Mock(side_effect=lambda route, *a, **k: calls.append(
                ('visit', Path(route).name, a[2], list(a[3]), k['search'])) or (
                    via_ok if Path(route).name == 'water.yaml' else True))
            self.dwell_and_leave = Mock(side_effect=lambda dwell: calls.append(
                ('dwell_and_leave', dwell)) or leave_ok)
            self.dwell_and_return_home = Mock(side_effect=lambda dwell, timeout: calls.append(
                ('dwell_and_return_home', dwell, timeout)) or True)
            self.configure_station_reference = Mock(side_effect=lambda _path, station: calls.append(
                ('configure_station_reference', station)))
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(side_effect=lambda: calls.append(('call_operator',))
                                      or 'URGENT')
            self.battery_return_due = Mock(return_value=battery_low)
            self.go_home = Mock(side_effect=lambda **k: calls.append(('go_home',)) or True)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    code = box_service.main([*argv, '--log', str(tmp_path / f'run_{via_ok}_{leave_ok}.jsonl')])
    return code, calls


def test_t36_water_station_then_table_then_dock(monkeypatch, tmp_path):
    """The call runs the water stop, its dwell and escape, the table, then the dock."""
    code, calls = _via_main(monkeypatch, tmp_path)
    assert code == 0
    assert calls == [('visit', 'water.yaml', 'water_station', [-0.14, -1.53], True),
                     ('dwell_and_leave', 0.0),
                     ('visit', 'route.yaml', 'table_01', [1.5, -1.49], True),
                     ('dwell_and_return_home', 0.0, 500.0)]


def test_via_selects_each_station_reference_before_its_own_visit(monkeypatch, tmp_path):
    code, calls = _via_main(monkeypatch, tmp_path, profiles=True)

    assert code == 0
    assert calls == [
        ('configure_station_reference', 'water_station'),
        ('visit', 'water.yaml', 'water_station', [-0.14, -1.53], True),
        ('dwell_and_leave', 0.0),
        ('configure_station_reference', 'table_01'),
        ('visit', 'route.yaml', 'table_01', [1.5, -1.49], True),
        ('dwell_and_return_home', 0.0, 500.0),
    ]


def test_low_battery_after_the_water_stop_docks_without_an_operator_call(monkeypatch, tmp_path):
    """Below the departure reserve between stops: skip the table and return to the dock."""
    code, calls = _via_main(monkeypatch, tmp_path, battery_low=True)
    assert code == 1
    assert calls == [('visit', 'water.yaml', 'water_station', [-0.14, -1.53], True),
                     ('dwell_and_leave', 0.0), ('go_home',)]


@pytest.mark.parametrize('via_ok,leave_ok,expected', [
    (False, True, [('visit', 'water.yaml')]),
    (True, False, [('visit', 'water.yaml'), ('dwell_and_leave',)]),
])
def test_t36_failed_water_stop_never_goes_to_the_table(
        monkeypatch, tmp_path, via_ok, leave_ok, expected):
    """A failed water approach or escape ends the run before any table motion."""
    code, calls = _via_main(monkeypatch, tmp_path, via_ok=via_ok, leave_ok=leave_ok)
    assert code == 1
    assert [call[:len(step)] for call, step in zip(calls, expected)] == expected
    # Nothing moves or invokes a notification after the failed step.
    assert calls[len(expected):] == []


@pytest.mark.parametrize('parked,left,expected', [
    (False, True, False), (True, False, False), (True, True, True)])
def test_t36_dwell_and_leave_holds_then_escapes(parked, left, expected):
    """Hold the verified stop, then reverse straight away; never escape unverified."""
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    route.wait_parked = Mock(return_value=parked)
    route._leave_parked_pose = Mock(return_value=left)
    route.emit = Mock()
    assert hasattr(BoxServiceRoute, 'dwell_and_leave'), 'NEW[T36]: no dwell_and_leave'
    assert route.dwell_and_leave(5.0) is expected
    route.wait_parked.assert_called_once_with(5.0)
    assert route._leave_parked_pose.call_count == (1 if parked else 0)
    events = [call.args[0] for call in route.emit.call_args_list]
    assert ('via_stop_finished' in events) is expected


def test_t37_live_precision_profile_requires_slowdown_disabled(monkeypatch):
    """A live collision monitor that still slows near the box is not the profile."""
    baseline = yaml.safe_load(PARAMS.read_text())
    geometry = yaml.safe_load(GEOMETRY.read_text())
    monitor = apply_docking_stop_profile(baseline, geometry)[
        'collision_monitor']['ros__parameters']

    def nested(name):
        value = monitor
        for component in name.split('.'):
            value = value[component]
        return deepcopy(value)

    client = Mock()
    client.wait_for_services.return_value = True
    client.get_parameters.side_effect = lambda names: list(names)
    node = object.__new__(BoxServiceRoute)
    node.precision_parameters = client
    monkeypatch.setattr(box_service, 'parameter_value_to_python', lambda value: value)
    monkeypatch.setattr(
        box_service, 'get_package_share_directory',
        lambda _name: str(ROOT / 'jdamr_cube_navigation'))
    node._wait = lambda names, _timeout: SimpleNamespace(values=[
        True if name == 'SlowdownZone.enabled' else nested(name) for name in names])
    assert node._precision_collision_ready(geometry) is False


@pytest.mark.parametrize('toward_deg,expected_deg', [
    (-24.5, -24.5),   # 2026-09-30 water station: region 24 deg clockwise, sweep turned CCW
    (-60.0, -30.0),   # far off: one bounded step toward the region
    (16.0, 16.0),
    (4.0, 30.0),      # centred region: the failure is not a heading problem
    (None, 30.0),     # no diagnostic: the fixed counter-clockwise sweep
])
def test_t42_search_turns_toward_the_region_first(toward_deg, expected_deg):
    """A known region bearing sets the step direction and size; the budget stays absolute."""
    budget = box_service.BoxSearchBudget()
    toward = None if toward_deg is None else math.radians(toward_deg)
    step = budget.next_rotation(toward_rad=toward)
    assert step == pytest.approx(math.radians(expected_deg))
    assert budget.cumulative_yaw_rad == pytest.approx(abs(math.radians(expected_deg)))


def test_t42_observation_search_uses_the_diagnostic_bearing(mission):
    """The first search rotation follows the recorded region bearing."""
    failure = box_service.BoxObservationUnavailable('no_box_surface_candidate', retryable=True)
    target = mission.observe_target.return_value

    def observe(*_args, **_kwargs):
        mission.last_region_bearing_error_rad = math.radians(-24.5)
        if mission.search_rotation.call_count == 0:
            raise failure
        return target

    mission.observe_target = Mock(side_effect=observe)
    assert invoke(mission, search=True)
    first = mission.search_rotation.call_args_list[0].args[0]
    assert first == pytest.approx(math.radians(-24.5))


def test_t43_resume_options_parse():
    """Resume at the via observation, or from a parked stop recorded in a log."""
    execute = [*RETURN_ARGS, '--execute', '--candidate-trial',
               '--return-home', '--return-timeout-s', '500']
    assert _parse_new('T43', [*execute, *VIA_ARGS, '--resume-at-observation']).via_id
    args = _parse_new('T43', [*execute, *VIA_ARGS, '--resume-parked-from-log', '/log'])
    assert str(args.resume_parked_from_log) == '/log'
    for invalid in ([*RETURN_ARGS, '--execute', '--candidate-trial',
                     '--resume-parked-from-log', '/log'],
                    [*execute, '--resume-parked-from-log', '/log', '--resume-at-observation']):
        with pytest.raises(SystemExit):
            parse_args(invalid)


def _resume_main(monkeypatch, tmp_path, extra, at_table_observation=False):
    for name in ('registry', 'mount', 'geometry', 'route', 'water'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    log = tmp_path / 'previous.jsonl'
    log.write_text(json.dumps({'event': 'box_target_observed', 'target': {
        'face_center_map_xy_m': [0.02, -1.2], 'outward_normal_map_xy': [1.0, 0.0]}}) + '\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_02',
            '--region-xy', '0.7', '-2.6', '--execute', '--candidate-trial', '--search',
            '--return-home', '--return-timeout-s', '500',
            *[str(log) if item == 'LOG' else item for item in extra]]
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    calls = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.visit_observed_box = Mock(side_effect=lambda route, *a, **k: calls.append(
                ('visit', Path(route).name, a[2], k.get('resume_at_observation'))) or True)
            self.resume_parked = Mock(side_effect=lambda face, dwell: calls.append(
                ('resume_parked', face['face_center_map_xy_m'], dwell)) or True)
            self.dwell_and_leave = Mock(side_effect=lambda dwell: calls.append(
                ('dwell_and_leave', dwell)) or True)
            self.dwell_and_return_home = Mock(side_effect=lambda dwell, timeout: calls.append(
                ('dwell_and_return_home', dwell)) or True)
            self.go_home = Mock(side_effect=lambda **k: calls.append(('go_home',)) or True)
            self.at_observation = Mock(side_effect=lambda route: calls.append(
                ('at_observation', Path(route).name)) or at_table_observation)
            self.verify_live_maps = Mock()
            self.wait_until_ready = Mock(return_value=True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    code = box_service.main([*argv, '--log', str(tmp_path / 'run.jsonl')])
    return code, calls


def test_t43_resume_at_via_observation_applies_to_the_water_stop_only(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-at-observation'])
    assert code == 0
    assert calls == [('at_observation', 'route.yaml'),
                     ('visit', 'water.yaml', 'water_station', True), ('dwell_and_leave', 0.0),
                     ('visit', 'route.yaml', 'table_02', False), ('dwell_and_return_home', 0.0)]


def test_resume_at_the_table_observation_skips_the_done_water_stop(monkeypatch, tmp_path):
    """2026-10-01 15:49: a resume at table_01 must not demand the water station."""
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-at-observation'],
                               at_table_observation=True)
    assert code == 0
    assert calls == [('at_observation', 'route.yaml'),
                     ('visit', 'route.yaml', 'table_02', True), ('dwell_and_return_home', 0.0)]


@pytest.mark.parametrize('pose, expected', [((0.80, -1.86, 0.2), True),
                                            ((0.40, -1.82, 0.0), False)])
def test_at_observation_compares_the_still_pose_with_the_route_end(
        monkeypatch, pose, expected):
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    monkeypatch.setattr(box_service, 'load_route', lambda _path: {
        'max_route_start_distance_m': 0.3,
        'waypoints': [{'x': 0.438, 'y': -1.817}, {'x': 0.858, 'y': -1.817, 'yaw': 0.0}]})
    route.verify_live_maps = Mock()
    route.wait_until_ready = Mock(return_value=True)
    route._localization_bound = lambda _relaxed: contextlib.nullcontext()
    route.capture_stationary_pose = Mock(return_value=(pose, {}))
    assert route.at_observation('table_01_route.yaml') is expected


def test_t43_parked_water_stop_resumes_with_logged_face_then_table(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path,
                               [*VIA_ARGS[:2], '--via-route', str(tmp_path / 'water.yaml'),
                                *VIA_ARGS[4:], '--resume-parked-from-log', 'LOG'])
    assert code == 0
    assert calls == [('resume_parked', [0.02, -1.2], 0.0),
                     ('visit', 'route.yaml', 'table_02', False), ('dwell_and_return_home', 0.0)]


def test_t43_parked_table_resumes_with_logged_face_then_docks(monkeypatch, tmp_path):
    code, calls = _resume_main(monkeypatch, tmp_path, ['--resume-parked-from-log', 'LOG'])
    assert code == 0
    assert calls == [('resume_parked', [0.02, -1.2], 0.0), ('go_home',)]


def test_t43_last_logged_face_requires_an_observed_face(tmp_path):
    path = tmp_path / 'log.jsonl'
    path.write_text('{"event": "accepted"}\nnot json\n')
    with pytest.raises(ValueError, match='no observed box face'):
        box_service.last_logged_face(path)
    path.write_text('\n'.join(json.dumps({'event': 'box_target_observed', 'target': {
        'face_center_map_xy_m': [x, 0.0], 'outward_normal_map_xy': [1.0, 0.0]}})
        for x in (1.0, 2.0)))
    assert box_service.last_logged_face(path)['face_center_map_xy_m'] == [2.0, 0.0]


@pytest.mark.parametrize('moved,left,expected', [
    (0.0, True, True), (0.05, True, False), (0.0, False, False)])
def test_t43_resume_parked_holds_still_then_escapes(monkeypatch, moved, left, expected):
    """The resumed dwell needs a stationary robot; the escape uses the logged face."""
    monkeypatch.setattr(box_service.time, 'sleep', lambda _s: None)
    route = BoxServiceRoute.__new__(BoxServiceRoute)
    route.capture_stationary_pose = Mock(side_effect=[((0.1, -1.2, math.pi), {}),
                                                      ((0.1 + moved, -1.2, math.pi), {})])
    route._leave_parked_pose = Mock(return_value=left)
    route._hold_still = Mock(return_value=True)
    route.emit = Mock()
    face = {'face_center_map_xy_m': [0.0, -1.2], 'outward_normal_map_xy': [1.0, 0.0]}
    assert route.resume_parked(face, 5.0) is expected
    assert route._leave_parked_pose.call_count == (0 if moved else 1)
    route._hold_still.assert_called_once_with(5.0)
    if moved == 0.0:
        assert route.last_box_face == face


def test_resident_executor_runs_requests_in_one_context(tmp_path, monkeypatch):
    """One rclpy context for every attempt; results land beside the requests."""
    import signal
    calls = []
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kw: calls.append('init'))
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    monkeypatch.setattr(box_service, 'parse_args', lambda argv: SimpleNamespace(argv=argv))

    def attempt(args, active):
        calls.append(tuple(args.argv))
        assert active == {'node': None, 'stop_requested': False}
        if args.argv == ['b']:
            signal.raise_signal(signal.SIGTERM)   # leave after this attempt
        return 0 if args.argv == ['a'] else 1

    monkeypatch.setattr(box_service, 'run_attempt', attempt)
    (tmp_path / '0.request').write_text(json.dumps({'argv': ['cancelled']}))
    (tmp_path / '0.cancelled').touch()
    (tmp_path / '1.request').write_text(json.dumps({'argv': ['a']}))
    (tmp_path / '2.request').write_text(json.dumps({'argv': ['b']}))
    assert box_service.serve(tmp_path) == 0
    assert calls == ['init', ('a',), ('b',), 'shutdown']
    assert json.loads((tmp_path / '1.result').read_text()) == {'code': 0}
    assert json.loads((tmp_path / '2.result').read_text()) == {'code': 1}
    assert not list(tmp_path.glob('*.running'))
    assert not (tmp_path / '0.request').exists()


def test_stop_during_node_construction_prevents_motion(tmp_path, monkeypatch):
    mount = tmp_path / 'mount.yaml'
    mount.write_text('{}')
    args = SimpleNamespace(registry=mount, camera_mount=mount, geometry=mount,
                           parking_contract=mount, log=tmp_path / 'events.jsonl')
    monkeypatch.setattr(box_service, 'load_registry', lambda _: {})
    monkeypatch.setattr(box_service, 'load_parking_contract', lambda _: {})
    active = {'node': None, 'stop_requested': False}
    node = Mock()

    def construct(*_args, **_kwargs):
        active['stop_requested'] = True  # SIGINT while no node existed yet.
        return node

    monkeypatch.setattr(box_service, 'BoxServiceRoute', construct)
    assert box_service.run_attempt(args, active) == 1
    node.request_operator_stop.assert_called_once_with()
    node.visit_observed_box.assert_not_called()
    node.go_home.assert_not_called()
    node.finish_navigation.assert_called_once_with()
    node.destroy_node.assert_called_once_with()


class _ParameterClientStub:
    """Answers set_parameters with the given success, recording the request."""

    def __init__(self, successful=True, available=True):
        self.successful, self.available, self.requests = successful, available, []

    def wait_for_services(self, timeout_sec=None):
        return self.available

    def set_parameters_atomically(self, parameters):
        self.requests.append([(p.name, p.value) for p in parameters])
        return SetParametersAtomically.Response(result=SetParametersResult(
            successful=self.successful, reason='' if self.successful else 'rejected'))


def test_observer_perception_gate_changes_only_once_and_clears_cached_status():
    node = object.__new__(BoxServiceRoute)
    node.emit = Mock()
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub()
    node._observer_perception_enabled = None
    node.box_status = {'detected': True, 'stamp_s': 10.}
    node.box_status_received_s = 10.
    node.box_status_history = deque([(10., node.box_status)], maxlen=16)

    assert node._set_observer_perception(False)
    assert node._set_observer_perception(False)

    assert len(node.observer_parameters.requests) == 1
    assert dict(node.observer_parameters.requests[0]) == {'perception_enabled': False}
    assert node.box_status is None
    assert node.box_status_received_s is None
    assert list(node.box_status_history) == []


def test_observer_perception_enable_discards_gate_status_until_fresh_depth():
    node = object.__new__(BoxServiceRoute)
    node.emit = Mock()
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub()
    node._observer_perception_enabled = False
    node.box_status = None
    node.box_status_received_s = None
    node.box_status_history = deque(maxlen=16)

    assert node._set_observer_perception(True)
    node._box_callback(SimpleNamespace(data=json.dumps({
        'reason': 'perception_enabled_waiting_for_depth', 'stamp_s': 11.})))
    assert node.box_status is None
    node._box_callback(SimpleNamespace(data=json.dumps({
        'reason': 'stable_detection', 'detected': True, 'stamp_s': 11.1})))
    assert node.box_status['stamp_s'] == 11.1


def test_failed_observer_disable_still_invalidates_local_cached_status():
    node = object.__new__(BoxServiceRoute)
    node.emit = Mock()
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub(successful=False)
    node._observer_perception_enabled = None
    node.box_status = {'detected': True, 'stamp_s': 10.}
    node.box_status_received_s = 10.
    node.box_status_history = deque([(10., node.box_status)], maxlen=16)

    assert not node._set_observer_perception(False)

    assert node.box_status is None
    assert node.box_status_received_s is None
    assert list(node.box_status_history) == []


def test_pointcloud_mode_passes_the_selected_region_and_complete_mount_revision():
    node = object.__new__(BoxServiceRoute)
    node.emit = lambda *args, **kwargs: None
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub()
    node.pointcloud_perception, node.surface_mode = True, 'table_edge'
    node._pointcloud_target_region_map = [1., 2., .3]
    mount = yaml.safe_load(CAMERA_MOUNT.read_text())
    assert node._configure_observer(mount, .3)
    requested = dict(node.observer_parameters.requests[0])
    assert requested['table_request_enabled'] is True
    assert json.loads(requested['table_target_region_map']) == [1., 2., .3]
    first = requested['table_calibration_id']
    mount['camera_mount']['transform']['yaw_rad'] += .01
    assert node._configure_observer(mount, .3)
    assert dict(node.observer_parameters.requests[1])['table_calibration_id'] != first
    node.pointcloud_perception = False
    assert node._configure_observer(mount, .3)
    disabled = dict(node.observer_parameters.requests[2])
    assert disabled['table_request_enabled'] is False
    assert disabled['table_target_region_map'] == disabled['table_calibration_id'] == ''


@pytest.mark.parametrize('distance_m, radius_m, expected_m', [
    (0.746, 0.4, 1.146),     # table_01 15:37: the 1.33 m wall is left out
    (1.9, 0.4, 2.0),         # never wider than the configured observer range
    (None, 0.4, 2.0),        # no region bearing: the full range
])
def test_observation_atomically_configures_mode_mount_and_depth(
        distance_m, radius_m, expected_m):
    node = object.__new__(BoxServiceRoute)
    events = []
    node.emit = lambda event, **fields: events.append((event, fields))
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = _ParameterClientStub()
    node.last_region_distance_m = distance_m
    node.surface_mode = 'table_edge'
    mount = yaml.safe_load(CAMERA_MOUNT.read_text())
    assert node._configure_observer(mount, radius_m) is True
    requested = dict(node.observer_parameters.requests[0])
    transform = mount['camera_mount']['transform']
    assert requested.pop('surface_mode') == 'table_edge'
    assert requested.pop('table_request_enabled') is False
    assert requested.pop('table_target_region_map') == requested.pop('table_calibration_id') == ''
    assert requested == pytest.approx({
        'maximum_depth_m': expected_m,
        'camera_height_m': transform['z_m'],
        'camera_roll_rad': transform['roll_rad'],
        'camera_pitch_rad': transform['pitch_rad'],
    })
    assert events[-1][0] == 'box_observer_configuration'
    assert events[-1][1]['applied'] is True


@pytest.mark.parametrize('client', [_ParameterClientStub(successful=False),
                                    _ParameterClientStub(available=False)])
def test_depth_window_failure_is_recorded_and_does_not_stop_the_observation(client):
    node = object.__new__(BoxServiceRoute)
    events = []
    node.emit = lambda event, **fields: events.append((event, fields))
    node._wait = lambda future, timeout_s: future
    node.observer_parameters = client
    node.last_region_distance_m = 0.7
    node.surface_mode = 'front'
    assert node._configure_observer(
        yaml.safe_load(CAMERA_MOUNT.read_text()), 0.4) is False
    assert events[-1][1]['applied'] is False and events[-1][1]['reason']


def test_rpp_transit_flag_selects_rpp_before_the_cycle(monkeypatch, tmp_path):
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial']
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        def __init__(self, *args, **kwargs):
            calls = []
            self.use_rpp_transit = Mock(side_effect=lambda: calls.append('rpp'))
            self.visit_observed_box = Mock(side_effect=lambda *a, **k: calls.append('visit'))
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            self.calls = calls
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    box_service.main([*argv, '--log', str(tmp_path / 'rpp.jsonl')])
    box_service.main([*argv, '--rpp-transit', '--log', str(tmp_path / 'mppi.jsonl')])
    default, rpp = created
    assert default.calls == ['visit']
    assert rpp.calls == ['rpp', 'visit']


def test_final_gap_and_dwell_reach_the_attempt(monkeypatch, tmp_path):
    for name in ('registry', 'mount', 'geometry', 'route'):
        (tmp_path / f'{name}.yaml').write_text('{}\n')
    argv = ['--registry', str(tmp_path / 'registry.yaml'),
            '--approach-route', str(tmp_path / 'route.yaml'),
            '--camera-mount', str(tmp_path / 'mount.yaml'),
            '--geometry', str(tmp_path / 'geometry.yaml'),
            '--parking-contract', str(BOX_CONTRACT), '--table-id', 'table_01',
            '--region-xy', '1.896', '0.303', '--execute', '--candidate-trial', '--return-home',
            '--return-timeout-s', '600']
    monkeypatch.setattr(box_service, 'load_registry', lambda _path: {'home': {}})
    monkeypatch.setattr(box_service.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(box_service.rclpy, 'shutdown', lambda **_kwargs: None)
    created = []

    class Route:
        def __init__(self, *args, **kwargs):
            self.visit_observed_box = Mock(return_value=True)
            self.dwell_and_return_home = Mock(return_value=True)
            self.dwell_and_leave = Mock(return_value=True)
            self._camera = Mock(return_value=True)
            self.finish_navigation = Mock(return_value=True)
            self.destroy_node, self.emit, self.request_stop = Mock(), Mock(), Mock()
            self.call_operator = Mock(return_value='CRITICAL')
            self.battery_return_due = Mock(return_value=False)
            created.append(self)

    monkeypatch.setattr(box_service, 'BoxServiceRoute', Route)
    box_service.main([*argv, '--log', str(tmp_path / 'default.jsonl')])
    box_service.main([*argv, '--final-gap-m', '0.02', '--dwell-s', '30',
                      '--via-id', 'water_station', '--via-route', str(tmp_path / 'route.yaml'),
                      '--via-region-xy', '0.06', '-1.27', '--log', str(tmp_path / 'trial.jsonl')])
    box_service.main([*argv, '--resume-at-observation', '--resume-at-final-approach',
                      '--surface-mode', 'table_edge',
                      '--log', str(tmp_path / 'resume_final.jsonl')])
    default, trial, resume_final = created
    assert default.final_gap_override_m is None
    assert default.dwell_and_return_home.call_args.args[0] == box_service.STOP_DWELL_S
    assert trial.final_gap_override_m == 0.02
    assert trial.dwell_and_return_home.call_args.args[0] == 30.0
    # The water stop holds as long as the table.
    assert trial.dwell_and_leave.call_args.args[0] == 30.0
    assert resume_final.visit_observed_box.call_args.kwargs[
        'resume_at_observation'] is True
    assert resume_final.visit_observed_box.call_args.kwargs[
        'resume_at_final_approach'] is True


def _final_judged(mission, final_pose, frame_width_m=0.45):
    """Run one visit whose straight approach ended at final_pose; return the last event."""
    mission.capture_stationary_pose = Mock(side_effect=[((0, 0, 0), {}), (final_pose, {})])
    mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540},
                      'frame_width': {'value': frame_width_m}},
        'table_01', (1, 0), 0.6, execute=True, candidate_trial=True)
    return mission.emit.call_args


@pytest.mark.parametrize('valid,lateral_error_m,expected', [
    (False, None, False),
    (True, .028, False),
    (True, .004, True),
])
def test_taught_station_does_not_leave_with_only_the_front_gap_confirmed(
        mission, valid, lateral_error_m, expected):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission._station_lateral_status = Mock(return_value={
        'lateral_reference_valid': valid,
        'internal_lateral_error_m': lateral_error_m,
        'internal_lateral_within_tolerance': (
            lateral_error_m is not None and abs(lateral_error_m) <= .005),
    })
    mission.capture_stationary_pose.side_effect = [
        ((0., 0., 0.), {}), ((.925, 0., 0.), {})]

    result = invoke(mission)

    assert result is expected
    finished = [call for call in mission.emit.call_args_list
                if call.args == ('box_approach_finished',)]
    assert bool(finished) is expected
    assert bool(getattr(mission, '_confirmed_box_stop', None)) is expected
    if not expected:
        assert mission.emit.call_args.kwargs['reason'] == 'station_lateral_arrival_unconfirmed'


def test_box_contract_judges_the_nearest_front_corner_at_its_target(mission):
    """Face at x 1.0 facing -x; the contract target is met at the nearest frame corner."""
    from jdamr_cube_navigation.parking import load_parking_contract
    mission.parking_contract = load_parking_contract(BOX_CONTRACT)
    gap = mission.parking_contract['target_front_gap_m']
    finished = _final_judged(mission, (1.0 - 0.065 - gap, 0.0, 0.0))
    assert finished.args == ('box_approach_finished',)
    assert finished.kwargs['desired_front_gap_m'] == pytest.approx(gap)
    assert finished.kwargs['estimated_nearest_front_gap_m'] == pytest.approx(gap)
    # The old 5.5 cm stop is now far from the target.
    assert _final_judged(mission, (1.0 - 0.065 - 0.055, 0.0, 0.0)).args == (
        'box_gap_not_confirmed',)
    # 3 deg inside the contract, centre 0.6 cm: a corner is 0.58 cm inside the face.
    tilt = math.radians(3.0)
    centre_x = 1.0 - 0.006 - 0.065 * math.cos(tilt)
    tilted = _final_judged(mission, (centre_x, 0.0, tilt))
    assert tilted.args == ('box_gap_not_confirmed',)
    assert min(tilted.kwargs['estimated_front_corner_gaps_m']) == pytest.approx(
        0.006 - 0.225 * math.sin(tilt))
    # 2.69 deg with the nearest corner at the target is a park.
    tilt = math.radians(2.69)
    corner_x = 1.0 - gap - box_service.front_lead_m(0.065, 0.225, tilt)
    assert _final_judged(mission, (corner_x, 0.0, tilt)).args == ('box_approach_finished',)


def test_final_gap_option_overrides_the_contract_in_the_judgement(mission):
    from jdamr_cube_navigation.parking import load_parking_contract
    mission.parking_contract = load_parking_contract(BOX_CONTRACT)
    mission.final_gap_override_m = 0.05
    finished = _final_judged(mission, (1.0 - 0.065 - 0.05, 0.0, 0.0))
    assert finished.args == ('box_approach_finished',)
    assert finished.kwargs['desired_front_gap_m'] == pytest.approx(0.05)


def test_table_edge_alignment_pose_still_closes_from_45cm_to_default_5cm():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = None
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.capture_stationary_pose = Mock(return_value=((0.5, 0.0, 0.0), {}))
    node._pose = Mock(side_effect=lambda _index, pose: SimpleNamespace(
        header=SimpleNamespace(), pose=pose))
    node._frozen_in_odom = Mock(return_value=(
        SimpleNamespace(), lambda x, y, yaw: (x, y, yaw)))
    node._execute_reverse_path = Mock(return_value=True)
    node.emit = Mock()
    node.config = {'frame_id': 'map', 'waypoints': []}
    node.waypoints = []
    target = {
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-1.0, 0.0),
    }
    reached, _to_odom = node._parking_final_approach(target, 0.065)
    assert reached is True
    straight = next(call for call in node.emit.call_args_list
                    if call.args == ('final_approach_straight',))
    # The robot starts with its front 43.5 cm from the edge and advances toward 5 cm.
    assert straight.kwargs['travel_m'] == pytest.approx(0.39)
    node._execute_reverse_path.assert_called_once()


def test_final_approach_constants_keep_their_measured_basis():
    """0.5 cm: 38 Parking stops ended 0.51 cm short of 5 cm (09-30 to 10-02)."""
    assert box_service.FINAL_STOP_SHORT_M == pytest.approx(0.005)
    assert box_service.FINAL_GAP_TOLERANCE_M == pytest.approx(0.01)
    assert box_service.front_lead_m(0.065, 0.225, 0.0) == pytest.approx(0.065)
    assert box_service.front_lead_m(0.065, 0.225, math.radians(-2.7)) == pytest.approx(
        0.065 * math.cos(math.radians(2.7)) + 0.225 * math.sin(math.radians(2.7)))


def _camera_node():
    node = object.__new__(BoxServiceRoute)
    node.camera_on_demand = True
    node.stop_requested = False
    node.events = []
    node.emit = lambda event, **fields: node.events.append((event, fields))
    return node


def test_box_camera_starts_and_stops_through_systemd_only_on_demand(monkeypatch):
    """2026-10-06: camera and observer held the Pi at 80 deg C; they run for box stops."""
    calls = []
    monkeypatch.setattr(box_service.subprocess, 'run', lambda command, **_kw: calls.append(
        command) or SimpleNamespace(returncode=0, stderr=''))
    node = _camera_node()
    assert node._camera(True) and node._camera(False)
    assert calls == [['sudo', '-n', 'systemctl', '--no-block', 'start', *box_service.CAMERA_UNITS],
                     ['sudo', '-n', 'systemctl', '--no-block', 'stop', *box_service.CAMERA_UNITS]]
    assert [fields['action'] for event, fields in node.events if event == 'box_camera'] == [
        'start', 'stop']
    node.camera_on_demand = False                    # --camera-always-on
    calls.clear()
    assert node._camera(False) is True and calls == []


def test_box_camera_failure_is_reported(monkeypatch):
    node = _camera_node()
    monkeypatch.setattr(box_service.subprocess, 'run', lambda *_a, **_k: SimpleNamespace(
        returncode=1, stderr='Unit not found'))
    assert node._camera(True) is False
    assert node.events[-1][1]['reason'] == 'Unit not found'


def test_camera_is_not_restarted_between_water_and_table(monkeypatch):
    calls = []
    monkeypatch.setattr(box_service.subprocess, 'run', lambda command, **_kw: calls.append(
        command) or SimpleNamespace(returncode=0, stderr=''))
    node = _camera_node()
    assert node._camera(True)
    requested_s = node._camera_requested_s
    assert node._camera(True)
    assert node._camera_requested_s == requested_s
    assert len(calls) == 1
    assert '--no-block' in calls[0]
    assert node._camera(False)
    assert node._camera(True)
    assert len(calls) == 3


def test_box_camera_ready_needs_an_observer_status_after_the_start(monkeypatch):
    node = _camera_node()
    node._camera_requested_s = 100.0
    clock = iter([100.0 + 0.1 * k for k in range(1000)])
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: next(clock))
    statuses = iter([None, 99.0, 99.0, 111.3])         # an old status does not count

    def spin(*_a, **_k):
        node.box_status_received_s = next(statuses, 111.3)
    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    node.box_status_received_s = None
    assert node._camera_ready(timeout_s=30.0) is True
    node = _camera_node()
    node._camera_requested_s = 100.0
    node.box_status_received_s = 50.0
    monkeypatch.setattr(box_service.rclpy, 'spin_once', lambda *_a, **_k: None)
    assert node._camera_ready(timeout_s=1.0) is False
    assert node.events[-1] == ('box_camera_ready', {'ready': False, 'timeout_s': 1.0})


@pytest.mark.parametrize('status_ready, expected_wait_s', [(True, 0.0), (False, 2.0)])
def test_camera_readiness_separates_background_transit_from_blocking_wait(
        monkeypatch, status_ready, expected_wait_s):
    node = _camera_node()
    node._camera_requested_s = 100.0
    node.box_status_received_s = 110.0 if status_ready else None
    clock = [126.5]
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])

    def spin(*_a, **_k):
        clock[0] += 2.0
        node.box_status_received_s = clock[0]

    monkeypatch.setattr(box_service.rclpy, 'spin_once', spin)
    assert node._camera_ready(timeout_s=30.0)
    event, fields = node.events[-1]
    assert event == 'box_camera_ready'
    assert fields['waited_s'] == 26.5 + expected_wait_s
    assert fields['blocking_wait_s'] == expected_wait_s


def test_transit_option_selects_the_planner_and_controller_trees():
    base = ['--registry', 'r', '--approach-route', 'a', '--camera-mount', 'm', '--geometry',
            'g', '--log', 'l', '--parking-contract', 'p', '--table-id', 'table_02',
            '--region-xy', '0', '0']
    args = parse_args([*base, '--transit', 'navfn-mppi'])
    assert args.transit == 'navfn-mppi' and args.camera_always_on is False
    with pytest.raises(SystemExit):
        parse_args([*base, '--transit', 'dwb'])


@pytest.mark.parametrize('final_gap_m', [0.010, 0.012])
def test_one_centimetre_table_parking_closes_remaining_gap_in_same_visit(mission, final_gap_m):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {}),
        ((1.0 - 0.065 - final_gap_m, 0, 0), {})]
    deadlines = []

    def approach(*_args):
        deadlines.append(mission.run_deadline_s)
        return True, None

    mission._straight_final_approach.side_effect = approach
    assert invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.observe_target.call_count == 4
    assert all(deadline is not None for deadline in deadlines)
    assert mission.run_deadline_s is None
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(final_gap_m)


def test_one_centimetre_table_parking_does_not_accept_two_centimetres(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.915, 0, 0), {}), ((0.925, 0, 0), {})]
    assert invoke(mission)
    assert mission._straight_final_approach.call_count == 2


def test_continuous_table_stop_rechecks_depth_before_accepting_odom_gap(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((.902, 0, 0), {}), ((.925, 0, 0), {})]
    mission._odom_pose = Mock(return_value=(.925, 0, 0))
    target = deepcopy(mission.observe_target.return_value)

    def approach(*_args):
        mission._continuous_table_stop = {
            'target': target, 'basis': 'aligned_face_frozen_for_precision_band',
            'observation_stamp_s': 100.0}
        return True, lambda x, y, yaw: (x, y, yaw)

    mission._straight_final_approach.side_effect = approach
    assert invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.observe_target.call_count == 4
    assert mission.emit.call_args.kwargs['confirmation_basis'] == 'fresh_depth_table_edge'
    assert mission.emit.call_args.kwargs['final_depth_measurement'] is True


def test_one_centimetre_table_correction_stops_when_gap_does_not_improve(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.observe_target.call_count == 4
    assert any(call.kwargs.get('reason') == 'final_gap_no_progress'
               for call in mission.emit.call_args_list)


@pytest.mark.parametrize('map_shift', [(.02, 0.0, 0.0), (0.0, .03, .4), (-.1, .1, -2.0)])
def test_final_depth_gap_survives_stationary_map_pose_updates(mission, map_shift):
    evidence = json.loads((ROOT / 'jdamr_cube_navigation/test/fixtures/'
                           'table_final_gap_map_shift_20261020.json').read_text())
    observed = evidence['target']['observation_robot_pose_map']
    parked = tuple(observed[i] + map_shift[i] for i in range(3))
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = evidence['target_gap_m']
    mission.parking_motion_revision = 7
    before = deepcopy(mission.observe_target.return_value)
    measured = deepcopy(evidence['target'])
    mission.observe_target.side_effect = [before, before, measured]
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), (parked, {})]

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': evidence['front_extent_m']},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': evidence['frame_width_m']}},
        'water_station', (0.06, -1.27), .6, execute=True, candidate_trial=True)

    mission._straight_final_approach.assert_called_once()
    final = mission.emit.call_args.kwargs
    assert final['estimated_front_corner_gaps_m'] == pytest.approx(
        evidence['coherent_corner_gaps_m'])
    assert final['confirmation_basis'] == 'fresh_depth_table_edge'
    assert final['physical_accuracy'] == 'NOT_EXTERNALLY_MEASURED'
    assert mission.selected_pose == dict(
        zip(('id', 'x_m', 'y_m', 'yaw_rad'), ('final_approach', *parked)))
    # The escape face must share the current map pose used by the stop hold.
    face = mission.last_box_face['face_center_map_xy_m']
    normal = mission.last_box_face['outward_normal_map_xy']
    assert sum((parked[i] - face[i]) * normal[i] for i in (0, 1)) == pytest.approx(
        sum((observed[i] - measured['face_center_map_xy_m'][i])
            * measured['outward_normal_map_xy'][i] for i in (0, 1)))
    assert measured == evidence['target']


def test_final_depth_is_reobserved_if_motion_occurs_during_stop_capture(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission.parking_motion_revision = 7
    before = deepcopy(mission.observe_target.return_value)
    measured = {**before, 'observation_robot_pose_map': [.925, 0.0, 0.0]}
    fresh = {**before, 'observation_robot_pose_map': [.945, 0.0, 0.0],
             'face_center_map_xy_m': [1.02, 0.0], 'x_m': .945}
    mission.observe_target.side_effect = [before, before, measured, fresh]
    captures = iter([(0.0, 0.0, 0.0), (.945, 0.0, 0.0), (.945, 0.0, 0.0)])

    def capture(**_kwargs):
        pose = next(captures)
        if pose[0] > 0.0:
            mission.parking_motion_revision = 8
        return pose, {}

    mission.capture_stationary_pose.side_effect = capture
    assert invoke(mission)
    mission._straight_final_approach.assert_called_once()
    assert mission.observe_target.call_count == 4
    assert any(call.args == ('box_target_discarded',)
               and call.kwargs.get('reason') == 'motion_after_table_observation'
               for call in mission.emit.call_args_list)
    assert mission.emit.call_args.kwargs['estimated_front_gap_m'] == pytest.approx(.01)


def test_one_centimetre_table_correction_never_reuses_face_after_observation_loss(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    target = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [
        target, target, target, RuntimeError('depth unavailable')]
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 2
    assert mission.emit.call_args.kwargs['final_depth_measurement'] is False


def test_one_centimetre_table_correction_never_advances_after_overshoot(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.933, 0, 0), {})]
    mission._retreat_table_gap = Mock(return_value=False)
    assert not invoke(mission)
    mission._straight_final_approach.assert_called_once()
    mission._retreat_table_gap.assert_called_once()
    assert mission.emit.call_args.args == ('box_gap_not_confirmed',)


@pytest.mark.parametrize('case', ['success', 'stopped', 'no_progress'])
def test_aligned_overshoot_retreats_in_the_same_visit_without_returning_home(mission, case):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    near = ((.932, 0, 0), {})
    final = near if case == 'no_progress' else ((.925, 0, 0), {})
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), near, final]
    deadlines = []

    def retreat(distance_m, start_map, correction):
        deadlines.append(mission.run_deadline_s)
        assert distance_m == pytest.approx(.007)
        assert start_map == near[0] and correction == 1
        return case != 'stopped'

    mission._retreat_table_gap = Mock(side_effect=retreat)
    mission.go_home = Mock(side_effect=AssertionError('a small gap error is not a home task'))
    assert invoke(mission) is (case == 'success')
    mission._straight_final_approach.assert_called_once()
    mission._retreat_table_gap.assert_called_once()
    assert len(deadlines) == 1 and deadlines[0] is not None
    if case == 'success':
        assert mission.emit.call_args.args == ('box_approach_finished',)
    elif case == 'no_progress':
        assert mission.emit.call_args.kwargs['reason'] == 'final_gap_no_progress'


@pytest.mark.parametrize('case', [
    'clear', 'blocked', 'user_stop', 'link_lost', 'drive_failed',
    'during_user_stop', 'during_link_loss', 'during_sensor_loss', 'during_timeout',
])
def test_small_gap_retreat_checks_full_static_outline_and_keeps_motion_guards(monkeypatch, case):
    node = object.__new__(BoxServiceRoute)
    node.registry = {'map': {'yaml_path': '/map.yaml'}, 'keepout': {'yaml_path': '/mask.yaml'}}
    node._final_gap_m = lambda: .01
    node.stop_requested = case == 'user_stop'
    link = [case != 'link_lost']
    node._operator_link_ready = lambda: link[0]
    node._navigation_ready = lambda **_: True
    outline = [(-.275, -.225), (.065, -.225), (.065, .225), (-.275, .225)]
    node._footprint_outline = lambda: outline
    check = Mock(return_value=case != 'blocked')
    factory = Mock(return_value=check)
    monkeypatch.setattr(box_service, 'CachedStaticCorridorValidator', factory)

    def drive(*_args, **_kwargs):
        if case == 'during_user_stop':
            node.stop_requested = True
        elif case == 'during_link_loss':
            link[0] = False
        elif case in ('during_sensor_loss', 'during_timeout'):
            node._last_direct_motion_failure = case
        return None if case == 'drive_failed' else -.002 if case.startswith('during_') else -.007

    node._drive_zero_turn = Mock(side_effect=drive)
    node.emit = Mock()
    start = (.3, .4, math.pi / 2)
    assert node._retreat_table_gap(.007, start, 1) is (case == 'clear')
    if case in ('user_stop', 'link_lost'):
        factory.assert_not_called()
    else:
        assert factory.call_args.args[2] == outline
        assert check.call_args.args[0][0] == start
        assert check.call_args.args[0][1] == pytest.approx((.3, .393, math.pi / 2))
    if case in ('clear', 'drive_failed') or case.startswith('during_'):
        node._drive_zero_turn.assert_called_once_with(-.007, precision=True, hold_heading=True)
    else:
        node._drive_zero_turn.assert_not_called()


def test_near_skewed_gap_reobserves_instead_of_ending_the_visit(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    yaw = math.radians(.8)
    skewed = (1.0 - .006 - .065 * math.cos(yaw), 0.0, yaw)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), (skewed, {}), ((.925, 0, 0), {})]
    mission._leave_parked_pose = Mock(return_value=True)
    mission._retreat_table_gap = Mock()
    assert invoke(mission)
    mission._leave_parked_pose.assert_called_once()
    mission._retreat_table_gap.assert_not_called()
    assert mission._straight_final_approach.call_count == 2
    assert mission.emit.call_args.args == ('box_approach_finished',)


def test_small_retreat_preserves_the_occluded_table_plane(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    near, final = (.932, 0, 0), (.925, 0, 0)
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), (near, {}), (final, {})]
    mission._odom_pose = Mock(side_effect=[near, final])
    target = deepcopy(mission.observe_target.return_value)

    def occluded():
        return box_service.BoxObservationUnavailable(
            'stable detected table_edge surface is required', retryable=True,
            fresh_frames=8, required_fresh_frames=8,
            fresh_rejection_reasons={'no_table_edge_candidate'})

    mission.observe_target.side_effect = [target, target, occluded(), occluded()]

    def approach(*_args):
        mission._continuous_table_stop = {
            'target': target, 'basis': 'aligned_face_frozen_for_precision_band',
            'observation_stamp_s': 100.0}
        return True, lambda x, y, yaw: (x, y, yaw)

    mission._straight_final_approach.side_effect = approach
    mission._retreat_table_gap = Mock(return_value=True)
    assert invoke(mission)
    mission._retreat_table_gap.assert_called_once()
    assert mission._straight_final_approach.call_count == 1
    assert mission.emit.call_args.args == ('box_approach_finished',)
    assert mission.emit.call_args.kwargs['confirmation_basis'] == 'frozen_table_edge_odom_stop'


@pytest.mark.parametrize('case', ['success', 'escape_failed', 'face_lost', 'still_skewed'])
def test_skewed_near_corner_reapproaches_once_without_ending_the_visit(mission, case):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    yaw = math.radians(0.75)
    skewed = (1.0 - 0.012 - 0.065 * math.cos(yaw), 0.0, yaw)
    final = skewed if case == 'still_skewed' else (0.925, 0.0, 0.0)
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), (skewed, {}), (final, {})]
    order, deadlines = [], []
    target = deepcopy(mission.observe_target.return_value)

    def observe(*_args, **_kwargs):
        order.append('observe')
        if case == 'face_lost' and 'escape' in order:
            raise RuntimeError('depth unavailable')
        return deepcopy(target)

    def approach(*_args):
        order.append('approach')
        deadlines.append(mission.run_deadline_s)
        return True, None

    def escape():
        order.append('escape')
        assert mission.last_box_face['face_center_map_xy_m'] == [1.0, 0.0]
        return case != 'escape_failed'

    mission.observe_target.side_effect = observe
    mission._straight_final_approach.side_effect = approach
    mission._leave_parked_pose = Mock(side_effect=escape)
    assert invoke(mission) is (case == 'success')
    assert order.count('escape') == 1
    assert order.count('approach') == (1 if case in {'escape_failed', 'face_lost'} else 2)
    assert all(value == deadlines[0] and value is not None for value in deadlines)
    if case == 'success':
        assert order[order.index('escape') + 1:][:2] == ['observe', 'approach']
        assert mission.emit.call_args.args == ('box_approach_finished',)


def test_logged_close_skew_corrects_nearby_once_without_a_full_escape(mission):
    """Reproduce the close skew from table_01_20261020_164220_05cd09c4.

    The water stop recorded 14.800/30.461 mm at 1.994 degrees.
    """
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    skewed = _table_edge_pose_for_corner_gaps(
        .014800294039719174, .030461438817617405)
    mission.capture_stationary_pose.side_effect = [
        ((0., 0., 0.), {}), (skewed, {}), (skewed, {})]
    order = []

    def approach(*_args):
        order.append('approach')
        return True, None

    def escape(*_args, **_kwargs):
        order.append('escape')
        return True

    mission._straight_final_approach.side_effect = approach
    mission._leave_parked_pose = Mock(side_effect=escape)

    assert not mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': .450}},
        'table_01', (1., 0.), .6, execute=True, candidate_trial=True)
    assert order == ['approach', 'approach']
    assert mission._straight_final_approach.call_count == 2
    mission._leave_parked_pose.assert_not_called()
    measured = [call.kwargs['estimated_front_corner_gaps_m']
                for call in mission.emit.call_args_list
                if call.args == ('box_gap_not_confirmed',)]
    assert len(measured) == 2
    assert measured[0] == pytest.approx(
        [.030461438817617405, .014800294039719174]), measured
    assert measured[1] == pytest.approx(
        [.030461438817617405, .014800294039719174]), measured
    assert mission.emit.call_args.args == ('failed',)
    assert mission.emit.call_args.kwargs['reason'] == 'final_table_alignment_unconfirmed'
    assert mission.emit.call_args.kwargs['near_alignment_attempted'] is True


def test_improving_near_alignment_continues_to_one_cm_in_same_visit(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    first = _table_edge_pose_for_corner_gaps(.0156, .0491)
    improved = _table_edge_pose_for_corner_gaps(.0275, .0371)
    mission.capture_stationary_pose.side_effect = [
        ((0., 0., 0.), {}), (first, {}), (improved, {}), ((.925, 0., 0.), {})]
    mission._leave_parked_pose = Mock(side_effect=AssertionError('unexpected retreat'))

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': .450}},
        'table_01', (1., 0.), .6, execute=True, candidate_trial=True)
    assert mission._straight_final_approach.call_count == 3
    mission._leave_parked_pose.assert_not_called()
    assert mission.emit.call_args.args == ('box_approach_finished',)


@pytest.mark.parametrize('case', ['success', 'still_skewed', 'approach_failed'])
def test_logged_table_stop_uses_near_alignment_instead_of_reverse_reapproach(mission, case):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    skewed = _table_edge_pose_for_corner_gaps(.02296500085977187, .030941691420788328)
    final = skewed if case == 'still_skewed' else (.925, 0., 0.)
    mission.capture_stationary_pose.side_effect = [((0., 0., 0.), {}), (skewed, {}), (final, {})]
    mission._straight_final_approach.side_effect = [(True, None),
                                                  (case != 'approach_failed', None)]
    mission._leave_parked_pose = Mock(side_effect=AssertionError('unnecessary escape'))
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': .450}},
        'table_01', (1., 0.), .6, execute=True, candidate_trial=True) is (case == 'success')
    mission._leave_parked_pose.assert_not_called()
    assert mission._straight_final_approach.call_count == 2
    assert sum(call.args[0] == 'table_near_alignment_correction'
               for call in mission.emit.call_args_list) == 1
    assert not any(call.args[0] == 'table_realignment_reapproach'
                   for call in mission.emit.call_args_list)
    if case == 'approach_failed':
        assert mission.emit.call_args.args == ('failed',)
        assert mission.emit.call_args.kwargs['reason'] == 'nav2_or_stop_confirmation'


@pytest.mark.parametrize('interruption', ['stop', 'operator_link', 'navigation'])
def test_near_alignment_never_dispatches_after_an_interruption(mission, interruption):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    skewed = _table_edge_pose_for_corner_gaps(.02296500085977187, .030941691420788328)
    mission.capture_stationary_pose.side_effect = [((0., 0., 0.), {}), (skewed, {})]
    mission._leave_parked_pose = Mock(side_effect=AssertionError('unexpected escape'))
    def emitted(event, **_kwargs):
        if event != 'box_gap_not_confirmed':
            return
        if interruption == 'stop':
            mission.stop_requested = True
        elif interruption == 'operator_link':
            mission._operator_link_ready.return_value = False
        else:
            mission._navigation_ready.return_value = False
    mission.emit.side_effect = emitted
    assert not mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': .450}},
        'table_01', (1., 0.), .6, execute=True, candidate_trial=True)
    assert mission._straight_final_approach.call_count == 1
    mission._leave_parked_pose.assert_not_called()


def test_one_centimetre_table_correction_has_a_bounded_iteration_count(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), *[((x, 0, 0), {}) for x in [0.855, 0.865, 0.875, 0.885]]]
    assert not invoke(mission)
    assert mission._straight_final_approach.call_count == 4
    assert mission.emit.call_args.kwargs['reason'] == 'final_gap_correction_limit'


def test_fine_trim_corrects_small_overshoot_without_restarting_the_visit(clear_precision_corridor):
    node = object.__new__(BoxServiceRoute)
    clear_precision_corridor(node)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.7, 0.0, 0.0), {}), ((0.7, 0.0, 0.0249), {})])
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node._rotate_in_place = Mock(side_effect=[0.031, -0.0061])
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.3)
    node.emit = Mock()
    reached, _ = node._zero_turn_final_approach({
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-math.cos(0.025), -math.sin(0.025)),
    }, 0.065)
    assert reached
    assert node._rotate_in_place.call_count == 2
    assert node._rotate_in_place.call_args.args[0] == pytest.approx(-0.006)
    assert node._drive_zero_turn.call_args.kwargs == {'precision': True, 'hold_heading': True}


def test_one_centimetre_table_correction_requires_current_operator_link(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission._operator_link_ready.return_value = False
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((0.905, 0, 0), {})]
    assert not invoke(mission)
    mission._straight_final_approach.assert_called_once()
    assert mission.emit.call_args.kwargs['reason'] == 'navigation_not_ready'


@pytest.mark.parametrize('remaining_deg, reached_expected', [(0.5, True), (1.1, False)])
def test_fine_trim_residual_is_judged_by_final_heading_contract(remaining_deg, reached_expected, clear_precision_corridor):
    node = object.__new__(BoxServiceRoute)
    clear_precision_corridor(node)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.7, 0.0, 0.0), {}), ((0.7, 0.0, math.radians(2.0 - remaining_deg)), {})])
    node._rotate_in_place = Mock(side_effect=[math.radians(2.0 - remaining_deg), 0.0, 0.0])
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.3)
    node.emit = Mock()
    heading_rad = math.radians(2.0)
    reached, _ = node._zero_turn_final_approach({
        'face_center_map_xy_m': (1.0, 0.0),
        'outward_normal_map_xy': (-math.cos(heading_rad), -math.sin(heading_rad)),
    }, 0.065)
    assert reached is reached_expected
    assert node._rotate_in_place.call_count == 3
    assert node._drive_zero_turn.call_count == int(reached_expected)


@pytest.mark.parametrize('start_x', [0.50, 0.90])
def test_table_fine_approach_uses_one_continuous_move_without_five_cm_stop(start_x):
    node = object.__new__(BoxServiceRoute)
    node._table_observation_context = ({}, (1.0, 0.0), 0.6)
    node._final_gap_m = Mock(return_value=0.01)
    node.observe_target = Mock()
    node._zero_turn_final_approach = Mock(return_value=(True, 'odom'))
    target = _table_edge_final_target(1.0)
    assert node._table_fine_approach(target, 0.065) == (True, 'odom')
    node._zero_turn_final_approach.assert_called_once()
    call = node._zero_turn_final_approach.call_args
    assert call.kwargs['continuous_table'] is True
    assert call.kwargs['allow_trim'] is False
    assert 'gap_m' not in call.kwargs
    node.observe_target.assert_not_called()


def test_default_parking_routes_one_centimetre_table_to_precision_approach():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.zero_turn_final = False
    node._table_fine_approach = Mock(return_value=(True, 'odom'))
    node._parking_final_approach = Mock()
    target = _table_edge_final_target(1.0)

    assert node._straight_final_approach(target, 0.065) == (True, 'odom')
    node._table_fine_approach.assert_called_once_with(target, 0.065)
    node._parking_final_approach.assert_not_called()


@pytest.mark.parametrize('escape', [False, True])
@pytest.mark.parametrize('reason', [
    'table alignment requires more clearance',
    'table lateral alignment requires more clearance',
])
def test_near_skewed_table_backs_out_once_before_continuous_reapproach(escape, reason):
    node = object.__new__(BoxServiceRoute)
    node._table_observation_context = ({}, (1.0, 0.0), 0.6)
    node._final_gap_m = Mock(return_value=0.01)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node._last_direct_motion_failure = reason
    node._zero_turn_final_approach = Mock(side_effect=[(False, None), (True, 'odom')])
    node._leave_parked_pose = Mock(return_value=escape)
    node.observe_target = Mock(return_value=_table_edge_final_target(1.0))
    node.emit = Mock()
    node.run_deadline_s = 123.0
    result = node._table_fine_approach(_table_edge_final_target(1.0), 0.065)
    assert result == ((True, 'odom') if escape else (False, None))
    assert node._zero_turn_final_approach.call_count == (2 if escape else 1)
    assert node.observe_target.call_count == int(escape)
    assert node.run_deadline_s == 123.0


def test_millimetre_corner_miss_closes_straight_without_full_escape(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    gaps = (.010557480365786562, .013277306556505254)
    yaw = math.asin((gaps[1] - gaps[0]) / .54)
    x = 1.0 - sum(gaps) / 2 - .065 * math.cos(yaw)
    mission.capture_stationary_pose.side_effect = [
        ((0, 0, 0), {}), ((x, 0, yaw), {}), ((.925, 0, 0), {})]
    mission._leave_parked_pose = Mock(side_effect=AssertionError('unnecessary full escape'))
    assert invoke(mission)
    mission._leave_parked_pose.assert_not_called()
    assert mission._straight_final_approach.call_count == 2


def test_near_feasible_corner_spread_uses_centred_straight_gap():
    node = object.__new__(BoxServiceRoute)
    node.box_front_half_width_m = .225
    node.parking_contract = {'yaw_tolerance_rad': math.radians(1)}
    node._final_gap_m = Mock(return_value=.01)
    node._zero_turn_final_approach = Mock(return_value=(True, 'odom'))
    node.emit = Mock()
    gaps = (.010557480365786562, .013277306556505254)
    yaw = math.asin((gaps[1] - gaps[0]) / .45)
    target = _table_edge_final_target(1.0)
    target['observation_robot_pose_map'] = [
        1.0 - sum(gaps) / 2 - .065 * math.cos(yaw), 0., yaw]
    assert node._table_fine_approach(target, .065) == (True, 'odom')
    options = node._zero_turn_final_approach.call_args.kwargs
    assert options['continuous_table'] is False
    assert options['allow_trim'] is False
    assert options['gap_m'] == pytest.approx(.01 - (gaps[1] - gaps[0]) / 2)


def test_near_straight_correction_does_not_bypass_unaligned_station_reference():
    node = object.__new__(BoxServiceRoute)
    node.box_front_half_width_m = .225
    node.parking_contract = {'yaw_tolerance_rad': math.radians(1)}
    node._final_gap_m = Mock(return_value=.01)
    node._station_lateral_status = Mock(return_value={
        'internal_lateral_within_tolerance': False})
    node._zero_turn_final_approach = Mock(return_value=(True, 'odom'))
    target = _table_edge_final_target(1.0)
    target['observation_robot_pose_map'] = [.915, 0., 0.]

    assert node._table_fine_approach(target, .065) == (True, 'odom')
    assert node._zero_turn_final_approach.call_args.kwargs['continuous_table'] is True


def test_alignment_reobserve_retreat_keeps_current_camera_view():
    node = object.__new__(BoxServiceRoute)
    mount = {'camera_mount': {'table_observation': {'base_offset_m': .37}}}
    node._table_observation_context = (mount, (1., 0.), .4)
    node._final_gap_m = Mock(return_value=.01)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node._last_direct_motion_failure = 'table alignment requires more clearance'
    node._zero_turn_final_approach = Mock(side_effect=[(False, None), (True, 'odom')])
    node._leave_parked_pose = Mock(return_value=True)
    node.observe_target = Mock(return_value=_table_edge_final_target(1.0))
    node.emit = Mock()
    assert node._table_fine_approach(_table_edge_final_target(1.0), .065)[0]
    node._leave_parked_pose.assert_called_once_with(clearance_m=.37)


def test_fine_table_starts_with_heading_error_without_a_stationary_trim():
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(1.0)}
    node.box_front_half_width_m = 0.225
    node.capture_stationary_pose = Mock(return_value=((0.5, 0.0, 0.12), {}))
    node._odom_pose = Mock(return_value=(0.5, 0.0, 0.12))
    node._frozen_in_odom = Mock(return_value=(RosPath(), lambda x, y, yaw: (x, y, yaw)))
    node._rotate_in_place = Mock(side_effect=AssertionError('stationary trim requested'))
    node._table_approach_update = Mock(return_value=lambda _pose: {
        'remaining_m': 0.4, 'heading_rad': 0.0})
    node._continuous_table_stop = None
    node.emit = Mock()
    target = _table_edge_final_target(1.0)

    def drive(*_args, **kwargs):
        assert kwargs['target_update']((0.5, 0.0, 0.12))['heading_rad'] == 0.0
        assert kwargs['continuous_table'] is True
        node._continuous_table_stop = {'target': dict(target)}
        return 0.4

    node._drive_zero_turn = drive
    reached, transform = node._table_fine_approach(target, 0.065)
    assert reached and callable(transform)
    node._rotate_in_place.assert_not_called()


def test_final_table_leg_freezes_face_before_computing_distance(clear_precision_corridor):
    node = object.__new__(BoxServiceRoute)
    clear_precision_corridor(node)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node._operator_link_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((10.0, 0.0, 0.0), {}))
    node._odom_pose = Mock(return_value=(0.924, 0.0, 0.0))
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(RosPath(), lambda x, y, yaw: (x - 10.0, y, yaw)))
    node._drive_zero_turn = Mock(return_value=0.001)
    node.emit = Mock()

    reached, to_odom = node._zero_turn_final_approach(
        _table_edge_final_target(11.0), 0.065, allow_trim=False)

    assert reached
    assert callable(to_odom)
    node._frozen_in_odom.assert_called_once()
    node._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.001), precision=True, hold_heading=True)


def test_occluded_table_escape_keeps_frozen_odom_face_after_amcl_shift():
    node = object.__new__(BoxServiceRoute)
    node.last_box_face = {
        'reference_frame': 'odom',
        'face_center_xy_m': [1.0, 0.0],
        'outward_normal_xy': [-1.0, 0.0],
    }
    node.stop_requested = False
    node.config = {'frame_id': 'map', 'waypoints': []}
    node.waypoints = node.config['waypoints']
    node._parking_parameters_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.980, 0.0, 0.0), {}),
        ((0.555, 0.0, 0.0), {}),
    ])
    node._odom_pose = Mock(side_effect=[
        (0.920, 0.0, 0.0),
        (0.495, 0.0, 0.0),
    ])
    node._make_reverse_path = Mock(return_value=RosPath())
    node._reverse_path_valid = Mock(return_value=True)

    def stale_map_face_transform(*_args):
        raise AssertionError('escape transformed the frozen odom face through current map')

    node._frozen_in_odom = Mock(
        return_value=(RosPath(), stale_map_face_transform))
    node._execute_reverse_path = Mock(return_value=True)
    node.emit = Mock()

    assert not node._leave_parked_pose()

    start, target = node._make_reverse_path.call_args.args
    assert start == pytest.approx((0.980, 0.0, 0.0))
    assert target == pytest.approx((0.495, 0.0, 0.0))
    assert start[0] - target[0] == pytest.approx(0.485)
    result = node.emit.call_args
    assert result.args == ('failed',)
    assert result.kwargs['reason'] == 'box_escape_not_confirmed'
    assert result.kwargs['face_distance_m'] == pytest.approx(0.505)
    assert result.kwargs['face_distance_m'] < 0.515
    assert node.last_box_face['reference_frame'] == 'odom'




def test_zero_turn_remeasures_face_after_rotation_before_computing_final_distance(clear_precision_corridor):
    node = object.__new__(BoxServiceRoute)
    clear_precision_corridor(node)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    yaw = math.radians(2.0)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0), {}), ((0.0, 0.0, yaw), {}), ((0.0, 0.0, yaw), {})])
    node._rotate_in_place = Mock(return_value=yaw)
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, 'odom'))
    node._drive_zero_turn = Mock(return_value=0.125)
    node.emit = Mock()
    outward = (-math.cos(yaw), -math.sin(yaw))
    target = {'face_center_map_xy_m': (0.30 * math.cos(yaw), 0.30 * math.sin(yaw)),
              'outward_normal_map_xy': outward}
    fresh = {'face_center_map_xy_m': (0.20 * math.cos(yaw), 0.20 * math.sin(yaw)),
             'outward_normal_map_xy': outward}
    reobserve = Mock(return_value=fresh)
    assert node._zero_turn_final_approach(target, 0.065, reobserve=reobserve) == (True, 'odom')
    reobserve.assert_called_once()
    assert target == fresh
    node._drive_zero_turn.assert_called_once_with(
        pytest.approx(0.125), precision=True, hold_heading=True)


@pytest.mark.parametrize('direction', [-1.0, 1.0])
@pytest.mark.parametrize('continuous', [False, True])
def test_trim_crossing_reobserves_before_any_counter_rotation(direction, continuous, clear_precision_corridor):
    node = object.__new__(BoxServiceRoute)
    clear_precision_corridor(node)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    yaw = [0.0]
    events = []
    node.capture_stationary_pose = lambda **_kwargs: ((0.0, 0.0, yaw[0]), {})
    node._pose = Mock(return_value=PoseStamped())
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, angle: (x, y, angle)))
    node._table_approach_update = Mock(return_value=lambda _pose: {'remaining_m': 0.1})
    node.emit = Mock()
    def target_for(heading):
        return {'face_center_map_xy_m': (0.3 * math.cos(heading), 0.3 * math.sin(heading)),
                'outward_normal_map_xy': (-math.cos(heading), -math.sin(heading))}
    target = target_for(direction * math.radians(0.8))
    def turn(requested, **_kwargs):
        events.append('turn')
        turned = requested * 1.4
        yaw[0] += turned
        return turned
    def observe():
        events.append('observe')
        return target_for(yaw[0] + direction * math.radians(0.2))
    def drive(*_args, **_kwargs):
        events.append('drive')
        node._continuous_table_stop = {'target': dict(target)}
        return 0.3
    node._rotate_in_place = turn
    node._drive_zero_turn = drive
    reached, _ = node._zero_turn_final_approach(
        target, 0.065, reobserve=observe, continuous_table=continuous)
    assert reached
    assert events == ['turn', 'observe', 'drive']


def test_zero_turn_never_advances_using_pre_turn_face_when_new_observation_fails():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = 0.01
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.0)}
    node.box_front_half_width_m = 0.225
    yaw = math.radians(2.0)
    node.capture_stationary_pose = Mock(side_effect=[
        ((0.0, 0.0, 0.0), {}), ((0.0, 0.0, yaw), {})])
    node._rotate_in_place = Mock(return_value=yaw)
    node._drive_zero_turn = Mock()
    node.emit = Mock()
    target = {'face_center_map_xy_m': (0.3, 0.0),
              'outward_normal_map_xy': (-math.cos(yaw), -math.sin(yaw))}
    assert node._zero_turn_final_approach(
        target, 0.065, reobserve=Mock(side_effect=RuntimeError('depth lost'))) == (False, None)
    node._drive_zero_turn.assert_not_called()
    assert node.emit.call_args.kwargs['reason'] == 'table_post_trim_observation_lost'




def test_near_table_alignment_never_navigates_to_a_behind_goal(mission):
    mission.surface_mode = 'table_edge'
    mission.capture_stationary_pose.side_effect = [
        ((0.75, 0.0, 0.0), {}), ((0.885, 0.0, 0.0), {})]
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0.75, y=0.0)
    behind = {**_table_edge_final_target(1.0), 'x_m': 0.485,
              'observation_robot_pose_map': [0.75, 0.0, 0.0]}
    final = _table_edge_final_target(1.0)
    mission.observe_target.side_effect = [behind, final, final]

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True,
        candidate_trial=True, resume_at_observation=True)

    mission.execute.assert_not_called()
    mission.plan_pose.assert_called_once()
    mission._straight_final_approach.assert_called_once_with(final, 0.065)
    assert any(call.args == ('table_alignment_transit_skipped',)
               for call in mission.emit.call_args_list)


def test_recovered_depth_history_does_not_block_fresh_table_occlusion():
    node = object.__new__(BoxServiceRoute)
    error = box_service.BoxObservationUnavailable(
        'stable detected table_edge surface is required', retryable=True,
        fresh_frames=24, required_fresh_frames=8,
        fresh_rejection_reasons={'no_table_edge_candidate'},
        latency_reasons={'box observation is stale or future-dated'})
    assert node._fresh_table_edge_occlusion(error)
    error.latency_limited = True
    assert not node._fresh_table_edge_occlusion(error)




def _moving_table_route(monkeypatch):
    node = object.__new__(BoxServiceRoute)
    node._station_reference_lock = Lock()
    monkeypatch.setattr(box_service, 'spin_node', Mock())
    clock = [10.0]
    mount = {'camera_mount': {
        'status': 'derived', 'parent_frame': 'base_link', 'child_frame': 'camera_link',
        'transform': {'x_m': -0.275, 'y_m': 0.0, 'z_m': 1.0,
                      'roll_rad': 0.0, 'pitch_rad': 0.0, 'yaw_rad': 0.0}}}
    node._table_observation_context = (mount, (0.5, 0.0), 0.6)
    node.parking_contract = {'robot_base_frame': 'base_link'}
    node.box_front_half_width_m = 0.225
    node.observer_stable_frames = 2
    node._final_gap_m = Mock(return_value=0.01)
    node._odom_pose = Mock(return_value=(0.0, 0.0, 0.0))
    node.emit = Mock()
    node._table_approach_validator = Mock(return_value=Mock())
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(clock[0] * 1e9)))
    monkeypatch.setattr(box_service.time, 'monotonic', lambda: clock[0])
    node.box_status = {'stamp_s': 9.99}
    node.box_status_received_s = clock[0]
    lookup = Mock(return_value=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.1, y=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    # The stamped odom TF for each image is already in the buffer.
    node.parking_tf = SimpleNamespace(lookup_transform=lookup,
                                      can_transform=Mock(return_value=True))
    update = node._table_approach_update(
        _table_edge_final_target(0.5), 0.065, lambda x, y, yaw: (x, y, yaw))
    clock[0] = 10.2
    node.box_status = {
        'stamp_s': 10.1, 'frame_id': 'camera_color_optical_frame',
        'control_ready': False, 'detected': True, 'stable': True,
        'surface_kind': 'table_edge', 'confidence': 0.9, 'reason': 'stable_detection',
        'table_height_m': 0.75, 'edge_rms_m': 0.001,
        'plane_normal': [0.0, -1.0, 0.0],
        'edge_endpoints_optical_m': [[-0.4, 0.25, 0.705], [0.4, 0.25, 0.705]]}
    node.box_status_received_s = clock[0]
    return node, clock, update, lookup


def test_moving_table_distance_uses_stamped_pose_and_revises_far_face(monkeypatch):
    node, _clock, update, lookup = _moving_table_route(monkeypatch)
    result = update((0.2, 0.0, 0.0))
    # Frame edge x=.43 plus stamped odom x=.1 is world x=.53, not current x=.63.
    assert result['remaining_m'] == pytest.approx(0.255)
    assert result['heading_rad'] == pytest.approx(0.0)
    assert lookup.call_args.args[2].nanoseconds == 10_100_000_000
    assert node._continuous_table_stop['target']['face_center_map_xy_m'] == pytest.approx((0.53, 0.0))
    update((0.21, 0.0, 0.0))
    assert lookup.call_count == 1


def test_live_lateral_feedback_and_preview_share_one_goal_per_control_tick(monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference = Mock()
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    first = {'valid': True, 'goal_odom': [.5, .02, 0.], 'stamp_s': 10.1}
    second = {'valid': True, 'goal_odom': [.5, -.02, 0.], 'stamp_s': 10.2}
    node.station_reference_goal = first
    node._station_goal_reference = Mock(return_value=(first, first))

    control = update((.2, 0., 0.))
    calls = node._station_goal_reference.call_count
    node._station_goal_reference.return_value = (second, second)
    control['validate_control']((.2, 0., 0.), 0.)
    validator = node._table_approach_validator.return_value

    assert control['lateral_error_m'] == pytest.approx(.02)
    assert node._station_goal_reference.call_count == calls
    assert validator.call_args.args[0] == (.2, 0., 0.)
    assert validator.call_args.kwargs['lateral_goal_odom'] == tuple(first['goal_odom'])
    changed = update((.2, 0., 0.))
    assert changed['lateral_error_m'] == pytest.approx(-.02)
    assert changed['remaining_m'] == pytest.approx(control['remaining_m'])


def test_missing_rgbd_goal_cannot_latch_the_final_precision_band(monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_taught_map_pose = (.5, .003, 0.)
    node.station_reference_goal = None
    node._station_goal_reference = Mock(return_value=(None, {
        'valid': False, 'reason': 'station_reference_not_observed'}))

    control = update((.42, 0., 0.))

    # Before the first match the base steers to the registered pose (ede1022),
    # but that fallback is never lateral arrival evidence.
    assert control['lateral_tracking'] is True
    assert control['lateral_error_m'] == pytest.approx(.003)
    assert node._continuous_table_stop['basis'] != 'aligned_face_frozen_for_precision_band'
    assert node._continuous_table_stop['lateral_reference_valid'] is False
    assert getattr(node, '_station_terminal_goal', None) is None


def test_map_station_goal_shares_the_faces_frozen_odom_transform(monkeypatch):
    node, _clock, _update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference_mode = 'map_pose_only'
    node.station_reference_taught_map_pose = (.5, 0., 0.)
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_goal = None
    node.station_reference_goal_frozen = False
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'reason': 'registered_map_pose_using_amcl',
        'goal_map': [.5, 0., 0.], 'goal_odom': [2.5, -.985, 0.], 'stamp_s': 10.2})
    node._odom_pose = Mock(return_value=(2.1, -1., 0.))
    # Keep the accepted face, so this test isolates localization movement rather
    # than a change to the observed table or robot odometry.
    node.box_status = {'stamp_s': 10.1, 'detected': False, 'stable': False,
                       'reason': 'no_table_edge_candidate'}
    update = node._table_approach_update(
        _table_edge_final_target(.5), .065,
        lambda x, y, yaw: (x + 2., y - 1., yaw))

    first = update((2.1, -1., 0.))
    node._fresh_station_reference.return_value['goal_odom'] = [2.5, -.95, 0.]
    second = update((2.1, -1., 0.))
    second['validate_control']((2.1, -1., 0.), 0.)

    assert first['lateral_error_m'] == pytest.approx(0.)
    assert second['lateral_error_m'] == pytest.approx(0.)
    validator = node._table_approach_validator.return_value
    assert validator.call_args.kwargs['lateral_goal_odom'] == pytest.approx((2.5, -1., 0.))
    assert node.station_reference_goal_frozen is True
    assert node._station_lateral_status()['internal_lateral_error_m'] == pytest.approx(0.)


def test_moving_table_centres_both_corner_gaps_instead_of_stopping_at_the_nearer_one(
        monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    yaw = .005

    control = update((.2, 0., yaw))

    center_gap = .53 - .2 - .065 * math.cos(yaw)
    assert control['remaining_m'] == pytest.approx((center_gap - .01) / math.cos(yaw))
    assert control['surface_gap_m'] == pytest.approx(
        center_gap - .225 * math.sin(yaw))


def test_moving_table_consumes_queued_fresh_depth_before_stopping(monkeypatch):
    node, _clock, update, lookup = _moving_table_route(monkeypatch)
    fresh = deepcopy(node.box_status)
    node.box_status = {**fresh, 'stamp_s': 9.6}

    def deliver_pending(_node, timeout_sec):
        assert timeout_sec == 0.0
        node.box_status = fresh

    monkeypatch.setattr(box_service, 'spin_node', deliver_pending)
    assert update((0.2, 0.0, 0.0))['remaining_m'] == pytest.approx(0.255)
    assert lookup.call_args.args[2].nanoseconds == 10_100_000_000


def test_accepted_table_face_bridges_processing_with_fresh_depth_frame(monkeypatch):
    node, clock, update, lookup = _moving_table_route(monkeypatch)
    update((0.2, 0.0, 0.0))
    clock[0] = 10.65
    node.depth_camera_info = SimpleNamespace(header=SimpleNamespace(
        frame_id='camera_color_optical_frame',
        stamp=SimpleNamespace(sec=10, nanosec=640_000_000)))
    node.depth_camera_info_received_s = 10.64
    assert update((0.25, 0.0, 0.0))['remaining_m'] == pytest.approx(0.205)
    assert lookup.call_count == 1
    clock[0] = 11.2
    with pytest.raises(RuntimeError, match='depth stale'):
        update((0.26, 0.0, 0.0))


@pytest.mark.parametrize('frame, stamp', [
    ('camera_color_optical_frame', 12), ('wrong_optical_frame', 10)])
def test_table_processing_bridge_rejects_wrong_or_future_depth_frame(monkeypatch, frame, stamp):
    node, clock, update, _lookup = _moving_table_route(monkeypatch)
    update((0.2, 0.0, 0.0))
    clock[0] = 10.65
    node.depth_camera_info = SimpleNamespace(header=SimpleNamespace(
        frame_id=frame, stamp=SimpleNamespace(sec=stamp, nanosec=640_000_000)))
    node.depth_camera_info_received_s = 10.64
    with pytest.raises(RuntimeError, match='depth stale'):
        update((0.25, 0.0, 0.0))


@pytest.mark.parametrize('yaw', [-0.15, 0.15])
def test_moving_table_supplies_observed_heading_in_stamped_odom(monkeypatch, yaw):
    node, _clock, update, lookup = _moving_table_route(monkeypatch)
    lookup.return_value.transform.rotation = SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
    assert update((0.2, 0.0, 0.0))['heading_rad'] == pytest.approx(yaw)


@pytest.mark.parametrize('clear', [False, True])
def test_live_table_corridor_uses_frozen_map_conversion_and_runtime_footprint(monkeypatch, clear):
    node = object.__new__(BoxServiceRoute)
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node.box_front_half_width_m = 0.225
    node._final_gap_m = Mock(return_value=0.01)
    node._footprint_outline = Mock(return_value=[(-0.28, -0.275), (0.07, 0.275)])
    node.emit = Mock()
    monkeypatch.setattr(box_service, 'face_approach_waypoints', Mock(return_value=[
        (0.0, 0.0, 0.1), (0.1, 0.01, 0.0)]))
    check = Mock(return_value=clear)
    factory = Mock(return_value=check)
    monkeypatch.setattr(box_service, 'CachedStaticCorridorValidator', factory)
    validate = node._table_approach_validator(
        0.065, lambda pose: (pose[0] + 10.0, pose[1] - 1.0, pose[2] + 0.3))
    if clear:
        validate((0.0, 0.0, 0.1), (0.5, 0.0), (-1.0, 0.0), 0.08)
    else:
        with pytest.raises(RuntimeError, match='static corridor blocked'):
            validate((0.0, 0.0, 0.1), (0.5, 0.0), (-1.0, 0.0), 0.08)
    assert box_service.face_approach_waypoints.call_args.kwargs['command_heading_rad'] == 0.08
    assert box_service.face_approach_waypoints.call_args.kwargs['continuous_table'] is True
    assert check.call_args.args[0] == [(10.0, -1.0, 0.4), (10.1, -0.99, 0.3)]
    assert factory.call_args.args[2] == node._footprint_outline.return_value


def test_near_fine_table_reuses_face_and_skips_separate_alignment_goal(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = 0.01
    mission.capture_stationary_pose.side_effect = [
        ((0.5, 0.0, 0.0), {}), ((0.925, 0.0, 0.0), {})]
    original = box_service.load_route('route')
    original['waypoints'][-1].update(x=0.5, y=0.0)
    mission.observe_target.return_value = _table_edge_final_target(1.0)

    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': 0.065},
                      'wheel_outer_width': {'value': 0.540}},
        'table_01', (1, 0), 0.6, execute=True,
        candidate_trial=True, resume_at_observation=True)

    mission.execute.assert_not_called()
    mission.plan_pose.assert_not_called()
    mission._straight_final_approach.assert_called_once()
    assert [call.args[2] for call in mission.observe_target.call_args_list] == [0.45, 0.01]
    assert any(call.args == ('table_alignment_transit_skipped',)
               and call.kwargs['reason'] == 'within_continuous_approach_range'
               for call in mission.emit.call_args_list)


@pytest.mark.parametrize('degrees,expected', [(2.377, None), (-0.798, None),
                                             (20.0, 20.0), (-45.0, -30.0), (None, None)])
def test_known_table_search_never_scans_away_from_center(degrees, expected):
    budget = box_service.BoxSearchBudget(target_directed=True)
    result = budget.next_rotation(None if degrees is None else math.radians(degrees))
    if expected is None:
        assert result is None
    else:
        assert result == pytest.approx(math.radians(expected))
        assert budget.next_rotation(math.radians(degrees)) is None


def test_known_table_view_recovery_never_dispatches_a_reverse(mission):
    mission.surface_mode = 'table_edge'
    mission._advance_to_table_view = Mock(return_value=True)
    mission._execute_reverse_path = Mock()
    assert mission._reposition_search()
    mission._advance_to_table_view.assert_called_once_with(require_motion=True)
    mission._execute_reverse_path.assert_not_called()
    mission.execute.assert_not_called()
    mission.plan_pose.assert_not_called()


def _table_view_node(monkeypatch, pose, *, clear=True):
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node._known_table_heading_rad = 0.
    node._table_observation_context = (
        {'camera_mount': {'table_observation': {'base_offset_m': .37}}},
        (1.438, -1.817), .4)
    node.registry = {'map': {'yaml_path': 'map'}, 'keepout': {'yaml_path': 'mask'}}
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=(pose, {}))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, -.275),
                                               (.07, .275), (-.28, .275)])
    node._drive_zero_turn = Mock(side_effect=lambda distance, **_k: distance)
    node.emit = Mock()
    check = Mock(return_value=clear)
    monkeypatch.setattr(box_service, 'CachedStaticCorridorValidator', Mock(return_value=check))
    return node, check


def test_recorded_table_views_advance_to_map_view_instead_of_reversing(monkeypatch):
    path = ROOT / 'jdamr_cube_navigation/test/fixtures/table_view_failure_20261019.json'
    fixture = json.loads(path.read_text())
    for pose in fixture['poses_map']:
        node, check = _table_view_node(monkeypatch, pose)
        assert node._advance_to_table_view()
        length = node._drive_zero_turn.call_args.args[0]
        assert 0 < length <= box_service.LIVE_APPROACH_MAX_M
        start, end = check.call_args.args[0]
        assert start == pose
        assert end[0] == pytest.approx(fixture['region_xy_m'][0] - .37)
        assert end[2] == pose[2]
        assert node._drive_zero_turn.call_args.kwargs == {
            'precision': True, 'hold_heading': True,
            'approach_time_s': 0.5}


@pytest.mark.parametrize('case', ['already_there', 'too_near', 'facing_away',
                                  'blocked', 'stop', 'link_lost', 'input_lost',
                                  'motion_failed', 'short_motion', 'off_axis'])
def test_table_view_approach_does_not_invent_motion_or_success(monkeypatch, case):
    pose = (1., -1.817, 0.)
    if case == 'already_there':
        pose = (1.068, -1.817, 0.)
    elif case == 'too_near':
        pose = (1.3, -1.817, 0.)
    elif case == 'facing_away':
        pose = (1., -1.817, math.pi)
    elif case == 'off_axis':
        pose = (1., 0., 0.)
    node, check = _table_view_node(monkeypatch, pose, clear=case != 'blocked')
    node.stop_requested = case == 'stop'
    node._operator_link_ready.return_value = case != 'link_lost'
    node._navigation_ready.return_value = case != 'input_lost'
    if case in {'motion_failed', 'short_motion'}:
        node._drive_zero_turn.side_effect = None
        node._drive_zero_turn.return_value = None if case == 'motion_failed' else .001
    assert not node._advance_to_table_view(require_motion=True)
    if case not in {'motion_failed', 'short_motion'}:
        node._drive_zero_turn.assert_not_called()


def test_mapped_table_view_is_reached_before_the_first_observation(monkeypatch):
    node, _check = _table_view_node(monkeypatch, (1., -1.817, 0.))
    order = []
    node._advance_to_table_view = Mock(side_effect=lambda **_k: order.append('forward') or True)
    node.observe_target = Mock(side_effect=lambda *_a: order.append('observe') or {'fresh': True})
    assert node._observe_with_search(
        {}, .065, .45, (1.438, -1.817), .4, phase='face_alignment',
        search_enabled=True, search_budget=box_service.BoxSearchBudget(target_directed=True))
    assert order == ['forward', 'observe']


@pytest.mark.parametrize('case', ['visible', 'stale', 'other_table'])
def test_visible_selected_table_does_not_add_a_map_only_approach(monkeypatch, case):
    node, check = _table_view_node(monkeypatch, (1., -1.817, 0.))
    node.box_status = {'detected': True, 'stable': True, 'surface_kind': 'table_edge'}
    node.box_geometry = {'front_to_wheel_axis': {'value': .065}}
    node._final_gap_m = Mock(return_value=.01)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10 ** 10))
    compute = Mock(return_value={
        'face_center_map_xy_m': (1.438, -1.817) if case != 'other_table' else (3., 2.)})
    if case == 'stale':
        compute.side_effect = ValueError('box observation is stale or future-dated')
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', compute)
    assert node._advance_to_table_view()
    assert node._drive_zero_turn.called is (case != 'visible')
    assert check.called is (case != 'visible')


@pytest.mark.parametrize('case', ['ready', 'single', 'missing', 'stale', 'spread',
                                  'yaw_spread', 'stop'])
def test_tape_entry_uses_only_coherent_fresh_pair_before_approach(case):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .01}
    node.parking_contract = {'yaw_tolerance_rad': math.radians(3.)}
    node.stop_requested = case == 'stop'
    node.run_deadline_s = 0.
    node._operator_link_ready = Mock(return_value=True)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**10))
    goal = {'valid': True, 'goal_odom': [1., .30, 0.], 'stamp_s': 9.9,
            'goal_filter_sample_count': 3, 'goal_filter_xy_spread_m': .002,
            'goal_filter_yaw_spread_rad': .005}
    if case == 'single':
        goal['goal_filter_sample_count'] = 1
    elif case == 'stale':
        goal['stamp_s'] = 8.
    elif case == 'spread':
        goal['goal_filter_xy_spread_m'] = .08
    elif case == 'yaw_spread':
        goal['goal_filter_yaw_spread_rad'] = .10
    node._station_goal_reference = Mock(return_value=(goal, {'valid': case != 'missing'}))
    anchored = {'tape_anchor': True}
    node._station_anchor_target = Mock(return_value=anchored)
    node.emit = Mock()
    target = {'map_anchor': True}
    if case == 'stop':
        with pytest.raises(RuntimeError, match='operator_stop'):
            node._prepare_table_tape_entry(target)
    else:
        assert node._prepare_table_tape_entry(target) is (anchored if case == 'ready' else target)
        node._station_goal_reference.assert_called_with(continuous_rebase=True)
    assert node._station_anchor_target.called is (case == 'ready')


@pytest.mark.parametrize('offset_m, restage', [(.005, False), (.30, True), (-.30, True)])
def test_wide_tape_entry_selects_restage_before_close_approach(offset_m, restage):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .01}
    node.station_reference_goal = {'goal_filter_sample_count': 3}
    node._station_reference_enabled = Mock(return_value=True)
    node._final_gap_m = Mock(return_value=.01)
    node.box_front_half_width_m = .225
    target = {'observation_robot_pose_map': [.435, -offset_m, 0.],
              'face_center_map_xy_m': [1., 0.], 'outward_normal_map_xy': [-1., 0.],
              'x_m': .485, 'y_m': 0., 'yaw_rad': 0., 'rgbd_reference_valid': True}
    reason, lateral = node._table_lateral_restage_reason(target, .065)
    assert (reason is not None) is restage
    assert lateral == pytest.approx(offset_m)


@pytest.mark.parametrize('tape, raises', [(True, False), (False, False), (True, True)])
def test_tape_observation_checker_is_scoped_and_restored(tape, raises):
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.station_reference = SimpleNamespace(
        observation_method='table_front_tape' if tape else 'surface_only')
    original = '/test/behavior_trees/navigate_through_poses_transit_rpp.xml'
    node.through_behavior_tree = original
    node.waypoints = []
    node.emit = Mock()
    selected = []
    def execute(**_kwargs):
        selected.append(Path(node.through_behavior_tree).name)
        if raises:
            raise RuntimeError('navigation interrupted')
        return True
    node.execute = execute
    if raises:
        with pytest.raises(RuntimeError, match='navigation interrupted'):
            node._execute_table_observation_transit()
    else:
        assert node._execute_table_observation_transit()
    assert selected == [('navigate_through_poses_table_observation_rpp.xml' if tape
                         else 'navigate_through_poses_transit_rpp.xml')]
    assert node.through_behavior_tree == original


def test_tape_view_corrects_heading_before_advancing(monkeypatch):
    node, _ = _table_view_node(monkeypatch, (1., -1.817, .2))
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.capture_stationary_pose.side_effect = [((1., -1.817, .2), {}),
                                                ((1., -1.817, 0.), {})]
    calls = []
    node.search_rotation = Mock(side_effect=lambda yaw: calls.append(('turn', yaw)) or True)
    node._drive_zero_turn.side_effect = lambda distance, **_k: calls.append(('drive', distance)) or distance
    assert node._advance_to_table_view()
    assert calls[0][0] == 'turn'
    assert calls[0][1] == pytest.approx(-.2)
    assert calls[1][0] == 'drive'


def test_moving_table_healthy_occlusion_keeps_same_odom_face(monkeypatch):
    node, clock, update, lookup = _moving_table_route(monkeypatch)
    valid = deepcopy(node.box_status)
    update((0.2, 0.0, 0.0))
    for stamp in (10.3, 10.4):
        clock[0] = stamp
        node.box_status = {'stamp_s': stamp, 'reason': 'no_table_edge_candidate',
                           'detected': False, 'stable': False}
        node.box_status_received_s = stamp
        result = update((0.35, 0.0, 0.0))
    assert result['remaining_m'] == pytest.approx(0.105)
    assert result['basis'] == 'healthy_edge_occlusion_frozen_odom'
    assert node._continuous_table_stop['target']['face_center_map_xy_m'] == pytest.approx((0.53, 0.0))
    clock[0] = 10.5
    node.box_status = {**valid, 'stamp_s': clock[0],
                       'edge_endpoints_optical_m': [[-0.4, 0.25, 0.805], [0.4, 0.25, 0.805]]}
    node.box_status_received_s = clock[0]
    assert update((0.35, 0.0, 0.0))['remaining_m'] == pytest.approx(0.205)
    assert lookup.call_count == 2


@pytest.mark.parametrize('failure', ['stale', 'latency', 'tf', 'range'])
def test_moving_table_rejects_unhealthy_inputs_and_unbounded_extension(monkeypatch, failure):
    node, clock, update, lookup = _moving_table_route(monkeypatch)
    if failure == 'stale':
        clock[0] = 11.0
    elif failure == 'latency':
        node.box_status['reason'] = 'camera_info_missing'
    elif failure == 'tf':
        lookup.side_effect = RuntimeError('TF not buffered')
    else:
        node.box_status['edge_endpoints_optical_m'] = [[-0.4, 0.25, 1.0], [0.4, 0.25, 1.0]]
    with pytest.raises(RuntimeError):
        update((0.2, 0.0, 0.0))


def test_moving_table_stability_reset_updates_a_quality_same_surface(monkeypatch):
    node, clock, update, lookup = _moving_table_route(monkeypatch)
    valid = deepcopy(node.box_status)
    update((0.2, 0.0, 0.0))
    for stamp in (12.0, 12.1):
        clock[0] = stamp
        node.box_status = {**valid, 'stamp_s': stamp, 'stable': False,
                           'reason': 'collecting_stability',
                           'edge_endpoints_optical_m': [[-0.4, 0.25, 0.715],
                                                        [0.4, 0.25, 0.715]]}
        node.box_status_received_s = stamp
        result = update((0.35, 0.0, 0.0))
    assert result['remaining_m'] == pytest.approx(0.115)
    assert result['basis'] == 'moving_candidate_at_stamped_odom'
    assert node._continuous_table_stop['target']['face_center_map_xy_m'] == pytest.approx((0.54, 0.0))
    clock[0] = 12.2
    node.box_status = {**valid, 'stamp_s': 12.2,
                       'edge_endpoints_optical_m': [[-0.4, 0.25, 0.805],
                                                    [0.4, 0.25, 0.805]]}
    node.box_status_received_s = clock[0]
    assert update((0.35, 0.0, 0.0))['remaining_m'] == pytest.approx(0.205)
    assert lookup.call_count == 4


def test_moving_table_candidate_still_rejects_a_different_surface(monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    update((.2, 0., 0.))
    node.box_status = {**node.box_status, 'stamp_s': 10.15, 'stable': False,
                       'reason': 'collecting_stability',
                       'edge_endpoints_optical_m': [[-.4, .25, .805], [.4, .25, .805]]}
    with pytest.raises(RuntimeError, match='candidate changed surface'):
        update((.2, 0., 0.))


def test_post_trim_reobservation_keeps_configuration_and_requires_fresh_face(monkeypatch):
    node = _observer_node(monkeypatch, scan=False)
    node.surface_mode = 'table_edge'
    node.box_status['surface_kind'] = 'table_edge'
    node._configure_observer.return_value = False
    target = {'face_center_map_xy_m': (0.5, 0.0)}
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock(return_value=target))
    assert node.observe_target({}, 0.065, 0.01, (0.5, 0.0), 0.6,
                               reuse_configuration=True) == target
    node._configure_observer.assert_not_called()
    node.capture_stationary_pose.assert_called_once()
    assert box_service.compute_table_edge_docking_target.call_args.kwargs['now_s'] == pytest.approx(10.2)


@pytest.mark.parametrize('return_home', [False, True])
def test_zero_dwell_hands_a_confirmed_visit_to_the_next_motion(mission, return_home):
    mission.parking_motion_revision = 7
    assert invoke(mission)
    assert mission._confirmed_box_stop['motion_revision'] == 7
    mission.wait_parked = Mock(side_effect=AssertionError('duplicate parking confirmation'))
    mission._leave_parked_pose = Mock(return_value=True)
    mission._camera = Mock(return_value=True)
    mission.go_home = Mock(return_value=True)
    if return_home:
        assert mission.dwell_and_return_home(0.0, 120.0)
        mission.go_home.assert_called_once_with(execute=True, timeout_s=120.0)
    else:
        assert mission.dwell_and_leave(0.0)
        mission._leave_parked_pose.assert_called_once()
    mission.wait_parked.assert_not_called()
    events = [c for c in mission.emit.call_args_list if c.args == ('parked_dwell_complete',)]
    assert events[-1].kwargs['dwell_s'] == 0.0
    assert events[-1].kwargs['reused_final_stop'] is True


@pytest.mark.parametrize('case', ['unconfirmed', 'motion', 'different_pose',
                                  'operator_stop', 'link_lost', 'stale_input'])
def test_zero_dwell_does_not_turn_a_failed_or_interrupted_visit_into_motion(mission, case):
    mission.parking_motion_revision = 7
    assert invoke(mission)
    mission._leave_parked_pose = Mock(return_value=True)
    if case == 'unconfirmed':
        mission._confirmed_box_stop = None
    elif case == 'motion':
        mission.parking_motion_revision += 1
    elif case == 'different_pose':
        mission.selected_pose = {**mission.selected_pose, 'x_m': 2.0}
    elif case == 'operator_stop':
        mission.stop_requested = True
    elif case == 'link_lost':
        mission._operator_link_ready.return_value = False
    else:
        mission._navigation_ready.return_value = False
    assert not mission.dwell_and_leave(0.0)
    mission._leave_parked_pose.assert_not_called()


def test_zero_dwell_resume_has_one_short_capture_and_no_sleep():
    node = object.__new__(BoxServiceRoute)
    node.capture_stationary_pose = Mock(return_value=((.1, -1.2, math.pi), {}))
    node._hold_still = Mock(side_effect=AssertionError('unrequested dwell'))
    node._leave_parked_pose = Mock(return_value=True)
    node.emit = Mock()
    face = {'face_center_map_xy_m': [0., -1.2], 'outward_normal_map_xy': [1., 0.]}
    assert node.resume_parked(face, 0.0)
    node.capture_stationary_pose.assert_called_once_with(intermediate=True)
    node._hold_still.assert_not_called()
    node._leave_parked_pose.assert_called_once()


def test_failed_preflight_reports_controller_failure_without_motion(mission):
    mission.preflight.return_value = False
    mission.last_preflight_failure = 'controller_server parameter service unavailable'
    assert not invoke(mission)
    failure = next(call for call in mission.emit.call_args_list
                   if call.args == ('failed',) and call.kwargs.get('phase') == 'preflight')
    assert failure.kwargs['reason'] == mission.last_preflight_failure
    mission.execute.assert_not_called()


# --- 2026-10-29 driving review: fore/aft hunting, re-approach holds, lateral re-stage ---

def test_fine_correction_never_advances_again_after_an_overshoot_retreat(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    near = ((.932, 0, 0), {})   # front 3 mm from the face: overshoot
    long = ((.915, 0, 0), {})   # 20 mm after the retreat
    mission.capture_stationary_pose.side_effect = [((0, 0, 0), {}), near, long]
    mission._retreat_table_gap = Mock(return_value=True)
    assert not invoke(mission)
    mission._straight_final_approach.assert_called_once()
    mission._retreat_table_gap.assert_called_once()
    assert mission.emit.call_args.args == ('failed',)
    assert mission.emit.call_args.kwargs['reason'] == 'final_gap_reversal_blocked'


def _corner_gap_visit(mission, left_gap_m, right_gap_m):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    pose = _table_edge_pose_for_corner_gaps(left_gap_m, right_gap_m)
    mission.capture_stationary_pose.side_effect = [((0., 0., 0.), {}), (pose, {})]
    mission._retreat_table_gap = Mock(side_effect=AssertionError('sub-millimetre retreat'))
    return mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540},
                      'frame_width': {'value': .450}},
        'table_01', (1., 0.), .6, execute=True, candidate_trial=True)


@pytest.mark.parametrize('left_gap_m, right_gap_m', [
    (.0078, .0136),   # centring advance of 0.7 mm
    (.0069, .0126),   # centring retreat of 0.25 mm
])
def test_fine_correction_below_the_base_resolution_is_not_commanded(
        mission, left_gap_m, right_gap_m):
    assert not _corner_gap_visit(mission, left_gap_m, right_gap_m)
    mission._straight_final_approach.assert_called_once()
    mission._retreat_table_gap.assert_not_called()
    assert mission.emit.call_args.kwargs['reason'] == 'final_gap_correction_below_resolution'
    assert abs(mission.emit.call_args.kwargs['requested_correction_m']) < (
        box_service.TABLE_FINE_MIN_PROGRESS_M)


@pytest.mark.parametrize('reason, released', [
    ('registered_map_pose_frozen_with_table_face', True),
    ('operator_measured_relative_lateral_correction', False),
])
def test_backing_out_releases_only_the_previous_approach_goal_holds(reason, released):
    node = object.__new__(BoxServiceRoute)
    node._table_observation_context = (
        {'camera_mount': {'table_observation': {'base_offset_m': .5}}}, (1., 0.), .6)
    node._leave_parked_pose = Mock(return_value=True)
    node._station_terminal_goal = {'goal_odom': [1., 0., 0.]}
    node.station_reference_goal = {'valid': True, 'reason': reason, 'goal_odom': [1., .02, 0.]}
    node.station_reference_goal_frozen = True
    assert node._leave_for_table_reobservation()
    node._leave_parked_pose.assert_called_once_with(clearance_m=.5)
    assert node._station_terminal_goal is None
    assert (node.station_reference_goal is None) is released
    assert node.station_reference_goal_frozen is (not released)


def _fine_approach_node(restage=(None, 0.)):
    node = object.__new__(BoxServiceRoute)
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = .01
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node._table_observation_context = ({}, (1., 0.), .6)
    node.emit = Mock()
    node._table_clearance_reapproaches = 0
    node._table_lateral_restages = 0
    target = _table_edge_final_target(1.0)
    order = []
    node.observe_target = Mock(side_effect=lambda *a, **k: order.append('observe') or dict(target))
    node._leave_for_table_reobservation = Mock(
        side_effect=lambda: order.append('reverse') or True)
    node._table_lateral_restage_reason = Mock(return_value=restage)
    node._restage_table_face_alignment = Mock(
        side_effect=lambda *a: order.append('restage') or True)

    def failed_approach(*_args, **_kwargs):
        order.append('approach')
        node._last_direct_motion_failure = 'table lateral alignment requires more clearance'
        return False, None

    node._zero_turn_final_approach = Mock(side_effect=failed_approach)
    return node, target, order


def test_table_clearance_reapproach_runs_once_per_visit():
    node, target, order = _fine_approach_node()
    assert node._table_fine_approach(dict(target), .065) == (False, None)
    assert node._table_fine_approach(dict(target), .065) == (False, None)
    assert order == ['approach', 'reverse', 'observe', 'approach', 'approach']
    assert any(call.args[0] == 'table_alignment_clearance_reapproach_exhausted'
               for call in node.emit.call_args_list)


def test_infeasible_lateral_reapproach_restages_sideways_before_approaching_again():
    reason = 'table approach preview heading out of range'
    node, target, order = _fine_approach_node(restage=(reason, .2))
    results = iter([(False, None), (True, 'to_odom')])

    def approach(*_args, **_kwargs):
        order.append('approach')
        result = next(results)
        node._last_direct_motion_failure = None if result[0] else reason
        return result

    node._zero_turn_final_approach.side_effect = approach
    assert node._table_fine_approach(dict(target), .065) == (True, 'to_odom')
    assert order == ['approach', 'reverse', 'observe', 'restage', 'approach']
    assert node._restage_table_face_alignment.call_args.args[1:] == (.065, reason, .2)


def _tape_restage_node(outcomes):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._table_rebase_requires_restage = False
    node._table_observation_context = ({}, (1., 0.), .6)
    node._final_gap_m = Mock(return_value=.01)
    node.emit = Mock()
    node._leave_parked_pose = Mock(return_value=True)
    node.plan_pose = Mock(return_value={'ok': True})
    node.execute = Mock(return_value=True)
    node._table_restage_tape_goal = {'goal_odom': [.925, .1, 0.]}
    node._table_restage_tape_target = Mock(side_effect=lambda target: target)
    target = _table_edge_final_target(1.)
    node.observe_target = Mock(side_effect=lambda *a: dict(target))
    node._table_lateral_restage_reason = Mock(side_effect=outcomes)
    return node, target


@pytest.mark.parametrize('initial_latch', [False, True])
def test_rebased_goal_latch_is_cleared_only_after_fresh_restage_preview(initial_latch):
    node, target = _tape_restage_node([(None, .005)])
    node._table_rebase_requires_restage = initial_latch
    assert node._restage_table_face_alignment(target, .065, 'clearance', .1)
    node._table_lateral_restage_reason.assert_called_once_with(
        target, .065, honor_rebase_latch=False)
    assert node._table_rebase_requires_restage is False
    node.execute.assert_called_once_with(final_parking=False, alignment=True, face=False)
    assert node._leave_parked_pose.call_count == 1


def test_carried_goal_residual_is_corrected_before_near_approach():
    node, target = _tape_restage_node([
        ('table lateral alignment requires more clearance', -.09859664981649491),
        (None, .005)])
    assert node._restage_table_face_alignment(target, .065, 'clearance', .15)
    assert node.execute.call_count == 2
    assert node._leave_parked_pose.call_count == 2
    assert node._table_lateral_restages == 1
    correction = next(call for call in node.emit.call_args_list
                      if call.args[0] == 'table_lateral_restage_correction')
    assert correction.kwargs['near_surface_approach_started'] is False
    assert correction.kwargs['lateral_offset_m'] == pytest.approx(-.0985966498)
    assert node._table_rebase_requires_restage is False


def test_rebased_goal_latch_remains_when_fresh_restage_preview_fails():
    reason = 'table lateral alignment requires more clearance'
    node, target = _tape_restage_node([(reason, .1), (reason, .1)])
    node._table_rebase_requires_restage = True
    assert not node._restage_table_face_alignment(target, .065, 'clearance', .1)
    assert node._table_rebase_requires_restage is True
    assert node.execute.call_count == 2
    assert node.emit.call_args.kwargs['reason'] == 'table_lateral_restage_not_confirmed'


def test_restage_stop_does_not_start_a_correction():
    node, target = _tape_restage_node([
        ('table lateral alignment requires more clearance', .1)])
    def stopped(**kwargs):
        node.stop_requested = True
        return True
    node.execute.side_effect = stopped
    assert not node._restage_table_face_alignment(target, .065, 'clearance', .1)
    assert node.execute.call_count == 1
    node._table_lateral_restage_reason.assert_not_called()


def test_lateral_restage_is_bounded_per_visit():
    node, target, order = _fine_approach_node(restage=('surface_only_lateral_offset', .2))
    node._table_lateral_restages = box_service.TABLE_LATERAL_RESTAGE_LIMIT
    assert node._table_fine_approach(dict(target), .065) == (False, None)
    assert order == ['approach', 'reverse', 'observe']
    assert node.emit.call_args.kwargs['reason'] == 'table_lateral_restage_exhausted'


def test_lateral_restage_uses_the_face_alignment_leg_from_turning_room():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._table_observation_context = ({}, (1., 0.), .6)
    node._final_gap_m = Mock(return_value=.01)
    node.observe_target = Mock(return_value=_table_edge_final_target(1.))
    node.emit = Mock()
    order = []
    node._leave_parked_pose = Mock(side_effect=lambda **k: order.append(('escape', k)) or True)
    node.plan_pose = Mock(
        side_effect=lambda pose, **k: order.append(('plan', pose, k)) or {'ok': True})
    node.execute = Mock(side_effect=lambda **k: order.append(('execute', k)) or True)
    target = {**_table_edge_final_target(1.0), 'face_center_map_xy_m': (1.0, .2)}
    assert node._restage_table_face_alignment(target, .065, 'reason', .2)
    assert order[0] == ('escape', {'clearance_m': box_service.ESCAPE_CLEARANCE_M,
                                  'max_face_angle_rad': box_service.FINAL_TRIM_MAX_RAD})
    planned = order[1][1]
    assert planned['id'] == 'face_alignment'
    assert planned['x_m'] == pytest.approx(1.0 - .065 - box_service.FACE_ALIGNMENT_FRONT_GAP_M)
    assert planned['y_m'] == pytest.approx(.2)
    assert order[1][2] == {'single': True, 'end_tolerance_m': .05}
    assert order[2] == ('execute', {'final_parking': False, 'alignment': True, 'face': True})
    assert node.last_box_face == {'face_center_map_xy_m': [1.0, .2],
                                  'outward_normal_map_xy': [-1.0, 0.0]}
    assert node._table_lateral_restages == 1


def test_initial_tape_restage_preserves_the_reported_plan_failure(mission):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission.station_reference = SimpleNamespace(observation_method='table_front_tape')
    mission._start_station_reference_perception = Mock()
    box_service.load_route('route')['waypoints'][-1].update(x=.5, y=.1)
    observed = {'x_m': .485, 'y_m': 0., 'yaw_rad': 0.,
                'face_center_map_xy_m': (1., 0.), 'outward_normal_map_xy': (-1., 0.),
                'observation_robot_pose_map': [.5, .1, 0.]}
    mission.observe_target = Mock(return_value=observed)
    mission._prepare_table_tape_entry = Mock(side_effect=lambda target: target)
    mission.capture_stationary_pose.side_effect = [((.5, .1, 0.), {})]
    mission._table_lateral_restage_reason = Mock(return_value=('clearance', .1))
    mission._table_restage_unavailable = Mock(return_value=None)

    def cannot_plan(*_args):
        mission.emit('failed', reason='table_lateral_restage_unplanned')
        return False

    mission._restage_table_face_alignment = Mock(side_effect=cannot_plan)
    assert not mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540}},
        'water_station', (1., 0.), .6, execute=True, candidate_trial=True,
        resume_at_observation=True)
    assert mission.emit.call_args.kwargs['reason'] == 'table_lateral_restage_unplanned'
    mission.execute.assert_not_called()


@pytest.mark.parametrize('offset_m, restaged', [(.20, True), (.02, False)])
def test_surface_only_table_restages_a_large_lateral_offset_before_the_straight_approach(
        mission, offset_m, restaged):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission.station_reference = None
    mission.station_reference_mode = 'surface_only'
    box_service.load_route('route')['waypoints'][-1].update(x=.5, y=offset_m)
    # The face-alignment target is already anchored to the registered y=0.
    observed = {'x_m': 1.0 - .065 - .45, 'y_m': 0.0, 'yaw_rad': 0.0,
                'face_center_map_xy_m': (1.0, 0.0), 'outward_normal_map_xy': (-1.0, 0.0),
                'observation_robot_pose_map': [.5, offset_m, 0.0]}
    later = {**observed, 'observation_robot_pose_map': [.925, 0.0, 0.0]}
    views = iter([observed])
    mission.observe_target = Mock(side_effect=lambda *a, **k: dict(next(views, later)))
    mission.capture_stationary_pose.side_effect = [((.5, offset_m, 0), {}), ((.925, 0, 0), {})]
    mission._leave_parked_pose = Mock(return_value=True)
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540}},
        'table_01', (1, 0), .6, execute=True, candidate_trial=True,
        resume_at_observation=True)
    events = [call.args[0] for call in mission.emit.call_args_list]
    assert ('table_lateral_restage' in events) is restaged
    assert ('table_alignment_transit_skipped' in events) is (not restaged)
    if restaged:
        mission._leave_parked_pose.assert_called_once_with(
            clearance_m=box_service.ESCAPE_CLEARANCE_M,
            max_face_angle_rad=box_service.FINAL_TRIM_MAX_RAD)
        mission.execute.assert_called_once_with(
            final_parking=False, alignment=True, face=True)
        assert mission.plan_pose.call_args_list[0].args[0]['y_m'] == 0.0
    else:
        mission._leave_parked_pose.assert_not_called()
        mission.execute.assert_not_called()


def _surface_only_visit(mission, observed_pose):
    mission.surface_mode = 'table_edge'
    mission.final_gap_override_m = .01
    mission.station_reference = None
    mission.station_reference_mode = 'surface_only'
    box_service.load_route('route')['waypoints'][-1].update(x=observed_pose[0], y=observed_pose[1])
    observed = {'x_m': 1.0 - .065 - .45, 'y_m': 0.0, 'yaw_rad': 0.0,
                'face_center_map_xy_m': (1.0, 0.0), 'outward_normal_map_xy': (-1.0, 0.0),
                'observation_robot_pose_map': list(observed_pose)}
    later = {**observed, 'observation_robot_pose_map': [.925, 0.0, 0.0]}
    views = iter([observed])
    mission.observe_target = Mock(side_effect=lambda *a, **k: dict(next(views, later)))
    mission.capture_stationary_pose.side_effect = [(tuple(observed_pose), {}), ((.925, 0, 0), {})]
    mission._leave_parked_pose = Mock(return_value=True)
    assert mission.visit_observed_box(
        'route', {}, {'front_to_wheel_axis': {'value': .065},
                      'wheel_outer_width': {'value': .540}},
        'table_01', (1, 0), .6, execute=True, candidate_trial=True,
        resume_at_observation=True)
    return [call.args[0] for call in mission.emit.call_args_list]


def test_far_observation_uses_the_normal_alignment_leg_without_a_counted_restage(mission):
    # 0.8 m from the face: the face-alignment leg runs anyway and already moves
    # to the anchored lateral target, so no escape and no restage budget is used.
    events = _surface_only_visit(mission, (.2, .2, 0.))
    assert 'table_lateral_restage' not in events
    mission._leave_parked_pose.assert_not_called()
    mission.execute.assert_called_once_with(final_parking=False, alignment=True, face=True)
    assert mission._table_lateral_restages == 0


def test_oblique_table_restage_does_not_skip_required_lateral_alignment(mission):
    # The approach accepts up to 30 degrees; its clearance escape must accept
    # that same heading rather than falling back to an infeasible approach.
    events = _surface_only_visit(mission, (.5, .2, math.radians(20.)))
    assert 'table_lateral_restage_unavailable' not in events
    assert 'table_lateral_restage' in events
    mission._leave_parked_pose.assert_called_once_with(
        clearance_m=box_service.ESCAPE_CLEARANCE_M,
        max_face_angle_rad=box_service.FINAL_TRIM_MAX_RAD)
    mission.execute.assert_called_once_with(final_parking=False, alignment=True, face=True)


@pytest.mark.parametrize('lateral_m, feasible', [(.02, True), (.20, False)])
def test_lateral_restage_reason_uses_the_live_controller_preview(lateral_m, feasible):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.box_front_half_width_m = .225
    node._final_gap_m = Mock(return_value=.01)
    target = {'x_m': .925, 'y_m': lateral_m, 'yaw_rad': 0.,
              'face_center_map_xy_m': (1., 0.), 'outward_normal_map_xy': (-1., 0.),
              'observation_robot_pose_map': [.5, 0., 0.]}
    reason, offset_m = node._table_lateral_restage_reason(target, .065)
    assert offset_m == pytest.approx(lateral_m)
    assert (reason is None) is feasible


@pytest.mark.parametrize(('lateral_m', 'goal_matches', 'expected_reason'), [
    (.039, True, None),
    (.20, True, 'table approach preview heading out of range'),
    (.039, False, 'carried_tape_goal_changed'),
])
def test_carried_tape_restage_requires_same_goal_and_feasible_continuous_preview(
        lateral_m, goal_matches, expected_reason):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.box_front_half_width_m = .225
    node._final_gap_m = Mock(return_value=.01)
    target = {'x_m': .925, 'y_m': lateral_m, 'yaw_rad': 0.,
              'face_center_map_xy_m': (1., 0.), 'outward_normal_map_xy': (-1., 0.),
              'observation_robot_pose_map': [.5, 0., 0.]}
    held = {'goal_odom': [target['x_m'], target['y_m'], target['yaw_rad']]}
    node._table_restage_tape_goal = held
    node.station_reference_goal = dict(held)
    node.station_reference_control_goal = dict(held)
    if not goal_matches:
        node.station_reference_control_goal['goal_odom'] = [.925, lateral_m + .01, 0.]
    node._table_restage_tape_target = Mock(return_value=target)

    reason, offset_m = node._table_lateral_restage_reason(target, .065)

    assert offset_m == pytest.approx(lateral_m)
    assert reason == expected_reason


def test_lateral_restage_reason_honors_a_rebased_goal_latch():
    node = object.__new__(BoxServiceRoute)
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.box_front_half_width_m = .225
    node._final_gap_m = Mock(return_value=.01)
    node._table_restage_tape_goal = None
    node._table_rebase_requires_restage = True
    target = {'x_m': .925, 'y_m': .002, 'yaw_rad': 0.,
              'face_center_map_xy_m': (1., 0.), 'outward_normal_map_xy': (-1., 0.),
              'observation_robot_pose_map': [.5, 0., 0.]}

    reason, offset_m = node._table_lateral_restage_reason(target, .065)

    assert reason == 'rebased_station_goal_requires_restage'
    assert offset_m == pytest.approx(.002)


def test_surface_only_face_target_uses_the_registered_lateral_position():
    node = object.__new__(BoxServiceRoute)
    node.station_reference = None
    node.station_reference_mode = 'surface_only'
    node.station_reference_taught_map_pose = (.485, 0., 0.)
    target = {'x_m': .485, 'y_m': .2, 'yaw_rad': 0.,
              'face_center_map_xy_m': (1., .2), 'outward_normal_map_xy': (-1., 0.)}
    anchored = node._station_anchor_target(target)
    assert anchored['x_m'] == pytest.approx(.485)
    assert anchored['y_m'] == pytest.approx(0.)
    assert anchored['face_center_map_xy_m'] == pytest.approx([1., 0.])
    # The shift is signed along the face tangent (-n_y, n_x) = (0, -1).
    assert anchored['lateral_shift_m'] == pytest.approx(.2)
    assert anchored['lateral_reference_mode'] == 'surface_only_registered_map_pose'
    assert anchored['lateral_fine_correction'] is False
    assert anchored['rgbd_reference_valid'] is False
    # The odom-frame control path keeps surface_only without any lateral goal.
    assert node._station_anchor_target(target, reference_frame='odom') is target


def test_rgbd_station_before_its_first_match_targets_the_registered_pose_in_map_only():
    node = object.__new__(BoxServiceRoute)
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    node.station_reference_taught_map_pose = (.485, .03, 0.)
    node._station_goal_reference = Mock(return_value=(None, {
        'valid': False, 'reason': 'station_reference_not_observed'}))
    target = {'x_m': .485, 'y_m': .2, 'yaw_rad': 0.,
              'face_center_map_xy_m': (1., .2), 'outward_normal_map_xy': (-1., 0.)}
    anchored = node._station_anchor_target(target)
    assert anchored['y_m'] == pytest.approx(.03)
    assert anchored['lateral_goal_pending'] is True
    assert anchored['rgbd_reference_valid'] is False
    assert anchored['lateral_goal_basis'] == 'registered_map_pose_until_rgbd_match'
    pending = node._station_anchor_target(target, reference_frame='odom')
    assert pending['y_m'] == .2
    assert pending['lateral_goal_pending'] is True


def test_tape_frozen_approach_keeps_endpoint_when_new_image_moves_it():
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    node.station_reference_goal_frozen = True
    anchor = {'valid': True, 'goal_odom': [1., .02, 0.], 'stamp_s': 1.}
    node.station_reference_goal = anchor
    newer = {'valid': True, 'goal_odom': [1., -.02, 0.], 'stamp_s': 2.}
    node._fresh_station_reference = Mock(return_value=newer)
    goal, reference = node._station_goal_reference(freeze=True)
    assert goal is anchor
    assert reference is newer
    assert goal['goal_filter_sample_count'] == 2
    node._station_goal_reference(freeze=True)
    assert goal['goal_filter_sample_count'] == 2
    node._fresh_station_reference.return_value = dict(newer, stamp_s=3.)
    goal, _ = node._station_goal_reference(freeze=True)
    assert goal['goal_filter_sample_count'] == 3
    node._fresh_station_reference.return_value = dict(newer, stamp_s=4.)
    goal, _ = node._station_goal_reference(freeze=True)
    assert goal['goal_filter_sample_count'] == 3
    assert goal['goal_odom'] == [1., .02, 0.]
    assert getattr(node, '_station_terminal_goal', None) is None


def test_tape_retreat_clears_old_endpoint_and_samples():
    from collections import deque
    from threading import Lock
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_goal = {'goal_odom': [1., .02, 0.]}
    node.station_reference_control_goal = dict(node.station_reference_goal)
    node.station_reference_goal_frozen = True
    node._station_reference_lock = Lock()
    node._station_reference_goal_samples = deque([dict(node.station_reference_goal)])
    node._table_observation_context = ({'camera_mount': {}}, (1., 0.), .6)
    node._leave_parked_pose = Mock(return_value=True)
    assert node._leave_for_table_reobservation()
    assert node.station_reference_goal is None
    assert node.station_reference_control_goal is None
    assert not node.station_reference_goal_frozen
    assert not node._station_reference_goal_samples


@pytest.mark.parametrize('reason,count,frozen,expected', [
    ('tape_pair_missing', 3, True, True),
    ('tape_pair_missing', 1, True, False),
    ('tape_pair_missing', 3, False, False),
    ('station_reference_stale', 3, True, False),
    ('station_reference_not_observed', 3, True, False),
    ('camera_disconnected', 3, True, False),
])
def test_terminal_tape_hold_requires_accepted_goal_and_current_input(reason, count, frozen, expected):
    node = object.__new__(BoxServiceRoute)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_goal = {'valid': True, 'goal_filter_sample_count': count}
    node.station_reference_goal_frozen = frozen
    assert node._table_terminal_reference_ready({'valid': False, 'reason': reason}) is expected


def test_accepted_tape_goal_latches_in_near_band_when_pair_leaves_view(monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    goal = {'valid': True, 'goal_odom': [.455,.001,0.], 'stamp_s': 10.1,
            'goal_filter_sample_count': 3}
    node.station_reference_goal = goal
    node.station_reference_goal_frozen = True
    missing = {'valid': False, 'reason': 'tape_pair_missing', 'stamp_s': 10.2}
    node._station_goal_reference = Mock(return_value=(goal,missing))
    update((.44,0.,0.))
    assert node._station_terminal_goal['goal_odom'] == goal['goal_odom']
    assert node._continuous_table_stop['basis'] == 'aligned_face_frozen_for_precision_band'


@pytest.mark.parametrize('angle_deg', [15.77, -20.0, 30.0])
def test_oblique_restage_reverse_reaches_projected_clearance(angle_deg):
    node = object.__new__(BoxServiceRoute)
    angle = math.radians(angle_deg)
    node.last_box_face = {'face_center_map_xy_m': [0.0, 0.0],
                          'outward_normal_map_xy': [-1.0, 0.0]}
    node._parking_parameters_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((-0.45, 0.0, angle), {}))
    node.emit = Mock()
    node.config = {'waypoints': []}
    node.waypoints = []
    node._make_reverse_path = Mock(return_value=RosPath())
    node._reverse_path_valid = Mock(return_value=False)
    # The normal escape retains its 15-degree guard.
    assert not node._leave_parked_pose()
    node._make_reverse_path.assert_not_called()
    # A restage may plan this reverse, but cannot bypass collision validation.
    assert not node._leave_parked_pose(max_face_angle_rad=box_service.FINAL_TRIM_MAX_RAD)
    start, target = node._make_reverse_path.call_args.args
    assert target[0] == pytest.approx(-box_service.ESCAPE_CLEARANCE_M)
    assert target[2] == angle
    assert target[1] == pytest.approx(-(.565-.45)*math.tan(angle))
    node._reverse_path_valid.assert_called_once()
    assert node.emit.call_args.kwargs['reason'] == 'box_escape_blocked'


@pytest.mark.parametrize('blocked', [False, True])
def test_coherent_rebase_checks_forward_path_before_reversing(monkeypatch, blocked):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.parking_contract['yaw_tolerance_rad'] = .03
    node._station_reference_lock = Lock()
    goal = {'valid': True, 'goal_odom': [.455, .027, 0.], 'stamp_s': 10.1,
            'goal_filter_sample_count': 3}
    node.station_reference_goal = goal
    node._station_goal_reference = Mock(return_value=(goal, goal))
    node._table_rebase_requires_restage = True
    validator = node._table_approach_validator.return_value
    if blocked:
        validator.side_effect = RuntimeError('table lateral alignment requires more clearance')
        with pytest.raises(RuntimeError, match='requires more clearance'):
            update((.2, 0., 0.))
        assert node._table_rebase_requires_restage
    else:
        result = update((.2, 0., 0.))
        assert result['lateral_error_m'] == pytest.approx(.027)
        assert not node._table_rebase_requires_restage
    assert validator.call_args.kwargs['lateral_goal_odom'] == tuple(goal['goal_odom'])
    assert validator.call_args.args[0] == (.2, 0., 0.)


def test_tape_acquisition_does_not_steer_to_opposite_map_lateral_goal(monkeypatch):
    node, _clock, update, _lookup = _moving_table_route(monkeypatch)
    node.parking_contract['yaw_tolerance_rad'] = .03
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    # Recorded 40 cm failure: map fallback asks for the opposite-side correction.
    node.station_reference_taught_map_pose = (.5, -.028, 0.)
    node.station_reference_goal = None
    node._station_goal_reference = Mock(return_value=(None, {
        'valid': False, 'reason': 'station_reference_not_observed'}))
    control = update((.2, 0., 0.))
    assert control['lateral_error_m'] == pytest.approx(0.)
    assert node._continuous_table_stop['lateral_reference_valid'] is False
    assert getattr(node, '_station_terminal_goal', None) is None
    node._station_goal_reference.return_value = (
        {'valid': True, 'goal_odom': [.5,.014,0.], 'stamp_s': 10.2},
        {'valid': True, 'stamp_s': 10.2})
    control = update((.21,0.,0.))
    assert control['lateral_error_m'] == pytest.approx(.014)
    node._station_goal_reference.assert_called_with(freeze=False, continuous_rebase=True)
    assert node.station_reference_goal_frozen is False


def test_far_tape_goal_is_ignored_and_reset_once_at_thirty_cm(monkeypatch):
    from collections import deque
    node, clock, update, _lookup = _moving_table_route(monkeypatch)
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .01}
    node.parking_contract['yaw_tolerance_rad'] = .03
    node._station_reference_lock = Lock()
    node._station_reference_pending = None
    far = {'valid': True, 'goal_odom': [.5, -.028, 0.], 'stamp_s': 10.1,
           'goal_filter_sample_count': 3}
    node.station_reference_goal = far
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = dict(far)
    node._station_reference_goal_samples = deque([far])
    node._station_goal_reference = Mock(return_value=(far,far))
    far_control = update((.1,0.,0.))  # beyond the 30 cm alignment band
    assert far_control['lateral_error_m'] == pytest.approx(0.)
    node._station_goal_reference.assert_not_called()
    node._station_goal_reference.return_value = (None, {'valid': False})
    close_control = update((.2,0.,0.))
    assert close_control['lateral_error_m'] == pytest.approx(0.)
    assert node.station_reference_goal is None
    assert node._station_terminal_goal is None
    assert not node._station_reference_goal_samples
    assert node._station_reference_min_stamp_s == pytest.approx(10.2)
    accepted = {'valid': True, 'goal_odom': [.5,.004,0.], 'stamp_s': 10.21}
    node.station_reference_goal = accepted
    node._station_goal_reference.return_value = (accepted,accepted)
    update((.21,0.,0.))
    assert node.station_reference_goal is accepted  # no per-tick clearing


def _close_tape_node(samples):
    node = object.__new__(BoxServiceRoute)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    node._station_reference_lock = Lock()
    node._station_reference_goal_samples = deque(samples, maxlen=5)
    node._station_reference_pending = None
    node.station_reference_result = None
    node.station_reference_goal = {'goal_odom': [1., .2, 0.]}
    node._station_terminal_goal = dict(node.station_reference_goal)
    return node


def _close_tape_sample(stamp, x=.395, lateral=.004):
    return {'valid': True, 'stamp_s': stamp, 'goal_odom': [.5, lateral, 0.],
            'quality': {'tape_center_base_xy_m': [x, 0.]}}


def test_close_entry_keeps_distinct_recent_near_samples_and_pending_frame():
    samples = [_close_tape_sample(9.7), _close_tape_sample(9.8), _close_tape_sample(9.9)]
    node = _close_tape_node(samples)
    node.station_reference_result = samples[-1]  # duplicate frame is not a fourth sample
    node._station_reference_pending = ('image', 'info', 9.95)
    cutoff, count = node._start_table_tape_close_alignment(.065)
    assert cutoff == 9.7 and count == 3
    assert list(node._station_reference_goal_samples) == samples
    assert node._station_reference_pending[2] == 9.95
    assert node.station_reference_result is samples[-1]
    assert node.station_reference_goal is None
    assert node._station_terminal_goal is None


def test_close_entry_drops_far_expired_and_unknown_range_samples():
    node = _close_tape_node([
        _close_tape_sample(9.1), _close_tape_sample(9.8, x=.6),
        {'valid': True, 'stamp_s': 9.9, 'goal_odom': [.5, 0., 0.]}])
    node._station_reference_pending = ('image', 'info', 9.9)
    cutoff, count = node._start_table_tape_close_alignment(.065)
    assert cutoff == 10. and count == 0
    assert not node._station_reference_goal_samples
    assert node._station_reference_pending is None


def test_close_entry_keeps_unresolved_near_goal_contradiction():
    node = _close_tape_node([_close_tape_sample(9.8), _close_tape_sample(9.9)])
    node._station_reference_goal_challenged = True
    node._station_reference_goal_correction_pending = True
    cutoff, count = node._start_table_tape_close_alignment(.065)
    assert count == 2 and cutoff == 9.8
    assert node._station_reference_goal_challenged is True
    assert node._station_reference_goal_correction_pending is True
    assert node.station_reference_goal is None  # two frames do not authorize a goal


def test_identical_depth_geometry_reuses_preview_but_changed_geometry_rechecks(monkeypatch):
    node, clock, update, _lookup = _moving_table_route(monkeypatch)
    validate = node._table_approach_validator.return_value
    control = update((.2, 0., 0.))
    control['validate_control']((.2, 0., 0.), 0.)
    assert validate.call_count == 1
    clock[0] = 10.21
    node.box_status['stamp_s'] = 10.11
    control = update((.2, 0., 0.))
    control['validate_control']((.2, 0., 0.), 0.)
    assert validate.call_count == 1
    # Even a small plane change is a different geometry, not timestamp-only reuse.
    node.box_status['stamp_s'] = 10.12
    for point in node.box_status['edge_endpoints_optical_m']:
        point[2] += .001
    control = update((.2, 0., 0.))
    control['validate_control']((.2, 0., 0.), 0.)
    assert validate.call_count == 2
    clock[0] = 11.
    with pytest.raises(RuntimeError, match='stale|silent'):
        update((.2, 0., 0.))
