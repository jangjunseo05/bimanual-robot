"""Rear docking keeps StopZone while replacing only predictive approach."""

import base64
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import zlib

from geometry_msgs.msg import Twist
from jdamr_cube_navigation import rear_collision_profile as module
import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.task import Future


OUTLINE = ((-.28, -.23), (-.28, .23), (.12, .23), (.12, -.23))


def _value(value):
    return Parameter('p', value=value).get_parameter_value()


class ParameterClient:
    def __init__(self, values, restore_fails=False, restore_times_out=False):
        self.values = dict(values)
        self.restore_fails = restore_fails
        self.restore_times_out = restore_times_out
        self.sets = []

    def wait_for_services(self, timeout_sec=None):
        return True

    @staticmethod
    def _future(response):
        future = Future()
        future.set_result(response)
        return future

    def get_parameters(self, names):
        response = GetParameters.Response()
        response.values = [_value(self.values[name]) for name in names]
        return self._future(response)

    def set_parameters_atomically(self, parameters):
        requested = {parameter.name: parameter.value for parameter in parameters}
        self.sets.append(requested)
        enabled = requested['FootprintApproach.enabled']
        if self.restore_times_out and enabled is True:
            return Future()
        successful = not (self.restore_fails and enabled is True)
        if successful:
            self.values.update(requested)
        response = SetParametersAtomically.Response()
        response.result = SetParametersResult(
            successful=successful, reason='' if successful else 'restore rejected')
        return self._future(response)


def scan(points=(), stamp_s=10.0):
    ranges = [math.inf] * 721
    for x_m, y_m in points:
        angle = math.atan2(y_m, x_m)
        index = int(round((angle + math.pi) / (2.0 * math.pi / 720)))
        ranges[index] = math.hypot(x_m, y_m)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='laser_link', stamp=SimpleNamespace(
                sec=int(stamp_s), nanosec=int((stamp_s % 1.0) * 1e9))),
        ranges=ranges, angle_min=-math.pi, angle_increment=2.0 * math.pi / 720,
        range_min=.05, range_max=12.0)


def command(linear=0.0, angular=0.0):
    result = Twist()
    result.linear.x, result.angular.z = linear, angular
    return result


def target_face(*, stamp_s=10.0, current_gap_m=.0175,
                tangent_bounds=(-.18, .18), offset_m=-.2925, residual_rms_m=.0005):
    return ({
        'normal_base_xy': (1.0, 0.0),
        'tangent_base_xy': (0.0, 1.0),
        'tangent_bounds_m': tangent_bounds,
        'line_offset_m': offset_m,
        'residual_rms_m': residual_rms_m,
        'stamp_s': stamp_s,
    }, .01, current_gap_m)


def node(monkeypatch, *, stop_zone=True, approach=True, restore_fails=False,
         restore_times_out=False, points=(), memory=()):
    clock = {'mono': 10.0, 'ros': 10.0}
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['mono'])
    message = scan(points)
    monitor = ParameterClient({
        'StopZone.enabled': stop_zone,
        'FootprintApproach.enabled': approach,
    }, restore_fails=restore_fails, restore_times_out=restore_times_out)
    smoother = ParameterClient({'max_decel': [-.5, 0.0, -1.5]})
    result = SimpleNamespace(
        last_scan=message, samples={'scan': 10.0}, latest_motion=None,
        precision_parameters=monitor,
        rear_collision_smoother_parameters=smoother,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(clock['ros'] * 1e9))),
        _remembered_blind_points=Mock(return_value=list(memory)),
        engage_emergency_stop=Mock(), request_stop=Mock(), emit=Mock(),
    )
    result._scan_in_base = Mock(return_value=(message, (0., 0., 0.), OUTLINE, list(memory)))
    return result, monitor, clock


@pytest.mark.parametrize('exit_mode', ['normal', 'exception', 'stop'])
def test_profile_restores_approach_for_every_exit(monkeypatch, exit_mode):
    route, monitor, _clock = node(monkeypatch)
    route.stop_requested = False
    with pytest.raises(RuntimeError) if exit_mode == 'exception' else _does_not_raise():
        with module.rear_collision_profile(route) as watch:
            assert monitor.values['StopZone.enabled'] is True
            assert monitor.values['FootprintApproach.enabled'] is False
            assert watch(command()) is None
            if exit_mode == 'exception':
                raise RuntimeError('body failed')
            if exit_mode == 'stop':
                route.stop_requested = True
    assert monitor.values == {
        'StopZone.enabled': True, 'FootprintApproach.enabled': True}
    assert monitor.sets == [
        {'FootprintApproach.enabled': False},
        {'FootprintApproach.enabled': True},
    ]


class _does_not_raise:
    def __enter__(self):
        return None

    def __exit__(self, *_args):
        return False


def test_restore_failure_is_propagated(monkeypatch):
    route, monitor, _clock = node(monkeypatch, restore_fails=True)
    with pytest.raises(RuntimeError, match='restore rejected'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored after rear docking')
    route.request_stop.assert_called_once_with()


def test_restore_timeout_is_fail_closed(monkeypatch):
    route, monitor, clock = node(monkeypatch, restore_times_out=True)

    def advance(_node, timeout_sec):
        clock['mono'] += timeout_sec

    monkeypatch.setattr(module, 'spin_node', advance)
    with pytest.raises(RuntimeError, match='timed out'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored after rear docking')
    route.request_stop.assert_called_once_with()


def test_stale_disabled_profile_is_repaired_then_scoped_run_continues(monkeypatch):
    route, monitor, _clock = node(monkeypatch, approach=False)
    route.stop_requested = False
    with module.rear_collision_profile(route) as watch:
        assert monitor.values['FootprintApproach.enabled'] is False
        assert watch(command()) is None
    assert monitor.values == {
        'StopZone.enabled': True, 'FootprintApproach.enabled': True}
    assert monitor.sets == [
        {'FootprintApproach.enabled': True},
        {'FootprintApproach.enabled': False},
        {'FootprintApproach.enabled': True},
    ]
    route.emit.assert_called_once_with(
        'rear_collision_profile_recovered',
        footprint_approach_enabled=True, stop_zone_enabled=True)
    route.engage_emergency_stop.assert_not_called()
    route.request_stop.assert_not_called()


def test_stale_disabled_profile_repair_failure_is_fail_closed(monkeypatch):
    route, monitor, _clock = node(
        monkeypatch, approach=False, restore_fails=True)
    with pytest.raises(RuntimeError, match='restore rejected'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored before rear docking')
    route.request_stop.assert_called_once_with()


def test_operator_stop_after_baseline_recovery_prevents_profile_disable(monkeypatch):
    route, monitor, _clock = node(monkeypatch, approach=False)
    route.stop_requested = True
    with pytest.raises(RuntimeError, match='operator_stop'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is True
    assert monitor.sets == [{'FootprintApproach.enabled': True}]


def test_profile_refuses_disabled_stop_zone(monkeypatch):
    route, monitor, _clock = node(monkeypatch, stop_zone=False)
    with pytest.raises(RuntimeError, match='requires StopZone'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.sets == []


@pytest.mark.parametrize('condition,reason', [
    ('missing', 'unavailable'),
    ('receive_stale', 'stale or future-dated'),
    ('header_stale', 'stale or future-dated'),
    ('header_future', 'stale or future-dated'),
])
def test_watch_rejects_unknown_or_stale_scan(monkeypatch, condition, reason):
    route, _monitor, clock = node(monkeypatch)
    with module.rear_collision_profile(route) as watch:
        if condition == 'missing':
            route.last_scan = None
        elif condition == 'receive_stale':
            clock['mono'] = 10.6
        elif condition == 'header_stale':
            clock['ros'] = 10.6
        else:
            clock['ros'] = 9.9
        assert watch(command(-.05)) == f'rear box scan {reason}'


def test_reverse_path_entry_is_detected(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_verified_goal_face_is_excluded_from_low_speed_reverse_prediction(
        monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.2925, 0.0)])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face())
        assert watch(command(-.05)) is None


def test_recorded_dock_face_blocks_prediction_until_same_scan_target_is_known(
        monkeypatch):
    evidence = json.loads((Path(__file__).parent / 'fixtures' /
                           'rear_laser_yaw_20261020.json').read_text())['guard_replay']
    route, _monitor, clock = node(monkeypatch)
    stamp = evidence['stamp_s']
    ranges = json.loads(zlib.decompress(base64.b64decode(
        evidence['ranges_zlib_base64'])))
    message = SimpleNamespace(
        **evidence['scan_metadata'],
        header=SimpleNamespace(frame_id=evidence['frame_id'], stamp=SimpleNamespace(
            sec=int(stamp), nanosec=int(round((stamp % 1.) * 1e9)))),
        ranges=[math.inf if value is None else value for value in ranges])
    route.last_scan = message
    clock['ros'] = stamp
    laser = tuple(evidence['laser_pose_m_rad'])
    route._scan_in_base.return_value = message, laser, OUTLINE, []
    with module.rear_collision_profile(route) as watch:
        move = command(evidence['velocity_mps'])
        assert watch(move) == 'rear docking swept footprint occupied'
        observation = dict(evidence['observation'], stamp_s=stamp)
        watch.set_target(observation, evidence['target_gap_m'], observation['gap_m'])
        assert watch(move) is None
        watch.clear_target()
        assert watch(move) == 'rear docking swept footprint occupied'


@pytest.mark.parametrize('case', [
    'outside_residual', 'outside_tangent', 'stale_stamp', 'past_target',
])
def test_goal_exception_preserves_non_target_and_invalid_context_obstacles(
        monkeypatch, case):
    point = (-.2925, 0.0)
    kwargs = {}
    if case == 'outside_residual':
        point = (-.289, 0.0)
    elif case == 'outside_tangent':
        point = (-.2925, .20)
        kwargs['tangent_bounds'] = (-.18, .18)
    elif case == 'stale_stamp':
        kwargs['stamp_s'] = 9.9
    elif case == 'past_target':
        kwargs['current_gap_m'] = .006
    route, _monitor, _clock = node(monkeypatch, points=[point])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(**kwargs))
        assert watch(command(-.05)) == (
            'rear docking swept footprint occupied')


def test_only_source_verified_goal_face_is_removed_from_blind_range_memory(
        monkeypatch):
    remembered = (-.2725, 0.0)
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._remembered_blind_points.side_effect = (
        lambda _laser, _scan, source_point_filter=None:
        [remembered] if source_point_filter is None or
        source_point_filter(None, None, None) else [])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(), lambda _pose, _scan, _point: True)
        assert watch(command(-.05)) is None


def test_source_verified_memory_reuses_provenance_without_recomputing(monkeypatch):
    target_point = (-.2725, 0.0)
    obstacle = (-.2725, .10)
    source_scan = scan()
    sources = (
        ((0., 0., 0.), source_scan, target_point, (0., 0., 0.)),
        ((0., 0., 0.), source_scan, obstacle, (0., 0., 0.)),
    )
    route, _monitor, _clock = node(
        monkeypatch, memory=(target_point, obstacle))
    route._blind_point_sources = list(sources)
    route._remembered_blind_points.side_effect = None
    route._remembered_blind_points.return_value = [target_point, obstacle]
    with module.rear_collision_profile(route) as watch:
        watch.set_target(
            *target_face(),
            lambda _pose, _scan, point: point == target_point)
        assert watch(command(-.05)) == (
            'rear docking swept footprint occupied')
        route._remembered_blind_points.assert_not_called()
        assert watch._state['filtered_memory'] == (obstacle,)
        assert watch._state['filtered_sources'] == (sources[1],)


def test_missing_memory_provenance_falls_back_to_filtered_recompute(monkeypatch):
    remembered = (-.2725, 0.0)
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._remembered_blind_points.side_effect = (
        lambda _laser, _scan, source_point_filter=None:
        [remembered] if source_point_filter is None else [])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(), lambda _pose, _scan, _point: True)
        assert watch(command(-.05)) is None
        route._remembered_blind_points.assert_called_once()


def test_unverified_or_different_source_memory_still_blocks(monkeypatch):
    remembered = (-.2725, 0.0)
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._remembered_blind_points.side_effect = (
        lambda _laser, _scan, source_point_filter=None:
        [remembered] if source_point_filter is None or
        source_point_filter(None, None, None) else [])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(), lambda _pose, _scan, _point: False)
        assert watch(command(-.05)) == (
            'rear docking swept footprint occupied')


def test_turned_memory_projected_inside_physical_chassis_does_not_hold_dock(
        monkeypatch):
    remembered = (-.27168920228390203, -.0948226257381165)
    source_scan = scan()
    source = ((1.4079365054397368, -.38436400281713257, -2.8540292428385063),
              source_scan, (-.5033593359334124, .7924763722111895),
              (1.8588671637060996, -.8711378052246369, 2.335495866508298))
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._blind_point_sources = [source]
    route._remembered_blind_points.side_effect = None
    route._remembered_blind_points.return_value = [remembered]
    with module.rear_collision_profile(route) as watch:
        watch.set_rear_extent(.275)
        watch.set_target(*target_face(), lambda *_args: False)
        assert watch(command(-.05)) is None


def test_turned_memory_outside_physical_chassis_still_blocks(monkeypatch):
    remembered = (-.279, 0.)
    source_scan = scan()
    source = ((0., 0., -1.), source_scan, (-.5, .4), (0., 0., 0.))
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._blind_point_sources = [source]
    route._remembered_blind_points.side_effect = None
    route._remembered_blind_points.return_value = [remembered]
    with module.rear_collision_profile(route) as watch:
        watch.set_rear_extent(.275)
        watch.set_target(*target_face(), lambda *_args: False)
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_filtered_provider_without_diagnostics_keeps_external_memory(monkeypatch):
    remembered = (-.279, 0.)
    source_scan = scan()
    source = ((0., 0., -1.), source_scan, (-.5, .4), (0., 0., 0.))
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))

    def filtered_memory(_laser, _scan, source_point_filter=None):
        assert source_point_filter is not None
        route._blind_point_sources = [source]
        route._blind_point_diagnostics = []
        return [remembered]

    route._remembered_blind_points.side_effect = filtered_memory
    with module.rear_collision_profile(route) as watch:
        watch.set_rear_extent(.275)
        watch.set_target(*target_face(), lambda *_args: False)
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


@pytest.mark.parametrize('remembered,angular', [
    ((-.275, 0.), 0.),
    ((-.2, .23), -.1),
    ((-.275, .23), -.1),
])
def test_physical_chassis_boundary_memory_remains_in_guard(
        monkeypatch, remembered, angular):
    source_scan = scan()
    source = ((0., 0., -1.), source_scan, (-.5, .4), (0., 0., 0.))
    route, _monitor, _clock = node(monkeypatch, memory=(remembered,))
    route._blind_point_sources = [source]
    route._remembered_blind_points.side_effect = None
    route._remembered_blind_points.return_value = [remembered]
    with module.rear_collision_profile(route) as watch:
        watch.set_rear_extent(.275)
        watch.set_target(*target_face(), lambda *_args: False)
        assert watch(command(-.05, angular)) == (
            'rear docking swept footprint occupied')


@pytest.mark.parametrize('point', [(-.289, 0.0), (-.2925, .20)])
def test_non_target_blind_range_memory_still_blocks(monkeypatch, point):
    route, _monitor, _clock = node(monkeypatch, memory=(point,))
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face())
        assert watch(command(-.05)) == (
            'rear docking swept footprint occupied')


@pytest.mark.parametrize('protrusion_m', [.004, .010, .014])
@pytest.mark.parametrize('remembered', [False, True])
def test_rough_accepted_target_never_hides_a_near_side_protrusion(
        monkeypatch, protrusion_m, remembered):
    point = (-.2925 + protrusion_m, 0.)
    route, _monitor, _clock = node(
        monkeypatch, points=() if remembered else [point],
        memory=[point] if remembered else ())
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(residual_rms_m=.004))
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_rough_target_accepts_far_side_fit_noise_without_loosening_near_side():
    target = module._target_context(*target_face(residual_rms_m=.004))
    assert module._is_target_face_point((-.3025, 0.), target)
    assert not module._is_target_face_point((-.2885, 0.), target)


def test_goal_face_is_not_excluded_for_in_place_rotation(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, .15)])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(offset_m=-.29))
        assert watch(command(0.0, .3)) == (
            'rear docking swept footprint occupied')


def test_contact_face_is_excluded_only_for_straight_motion_away(monkeypatch):
    route, _monitor, _clock = node(
        monkeypatch, memory=((-0.28, .15),))
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(
            offset_m=-.28, current_gap_m=0.0))
        assert watch(command(.05)) is None
        assert watch(command(-.05)) == (
            'rear docking swept footprint occupied')
    target = module._target_context(*target_face(
        offset_m=-.28, current_gap_m=0.0))
    assert module._may_ignore_target(command(.05), target, 10.0)
    assert not module._may_ignore_target(command(.05, .3), target, 10.0)
    assert not module._may_ignore_target(command(0.0, .3), target, 10.0)


def test_profile_finally_clears_goal_face_context(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.2925, 0.0)])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face())
        assert watch.target_active is True
    assert watch.target_active is False
    assert watch._state['filtered_memory'] == ()


def test_rotation_corner_collision_is_detected(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, .15)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01, .3)) == 'rear docking swept footprint occupied'


def test_touching_point_is_allowed_only_when_moving_away(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.28, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(.05)) is None
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_one_centimetre_target_is_allowed_at_fresh_low_speed(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.285, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) is None


def test_larger_live_reverse_speed_sets_prediction_horizon(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) is None
        route.latest_motion = (10.0, .05, 0.0)
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'


def test_larger_live_turn_rate_sets_prediction_arc(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, .15)])
    route.latest_motion = (10.0, .01, .3)
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'


def test_zero_command_can_stop_with_a_stale_scan(monkeypatch):
    route, _monitor, clock = node(monkeypatch)
    with module.rear_collision_profile(route) as watch:
        clock['mono'] = clock['ros'] = 20.0
        assert watch(command()) is None


def test_stale_scan_emits_throttled_latency_breakdown(monkeypatch):
    route, _monitor, clock = node(monkeypatch)
    route.scan_input_timing = {'receive_gap_s': .12, 'stamp_age_s': .61}
    with module.rear_collision_profile(route) as watch:
        clock['mono'] = clock['ros'] = 10.7
        assert watch(command(-.05)) == 'rear box scan stale or future-dated'
        assert watch(command(-.05)) == 'rear box scan stale or future-dated'
        route.emit.assert_called_once()
        event, = route.emit.call_args.args
        values = route.emit.call_args.kwargs
        assert event == 'rear_scan_latency_breakdown'
        assert values['receive_age_s'] == pytest.approx(.7)
        assert values['ros_age_s'] == pytest.approx(.7)
        assert values['scan_input_timing'] == {
            'receive_gap_s': .12, 'stamp_age_s': .61}


def test_new_scan_refreshes_blind_range_memory(monkeypatch):
    route, _monitor, _clock = node(monkeypatch)
    route._remembered_blind_points.return_value = [(-.29, 0.0)]
    with module.rear_collision_profile(route) as watch:
        route.last_scan = scan(stamp_s=10.1)
        route.samples['scan'] = 10.0
        route.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10100000000))
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'
        route._remembered_blind_points.assert_called_once()


def test_runtime_deceleration_must_match_direct_motion(monkeypatch):
    route, _monitor, _clock = node(monkeypatch)
    route.rear_collision_smoother_parameters.values['max_decel'] = [-.4, 0., -1.5]
    with pytest.raises(RuntimeError, match='does not match'):
        with module.rear_collision_profile(route):
            pass


@pytest.mark.parametrize('angular', [-.3, .3])
@pytest.mark.parametrize('remembered', [False, True])
def test_rotation_watch_covers_single_live_or_blind_leg_without_cm_override(
        monkeypatch, angular, remembered):
    point = ((.06, math.copysign(.232, angular)) if remembered else
             (-.29, math.copysign(.20, angular)))
    route, monitor, _clock = node(
        monkeypatch, points=() if remembered else (point,),
        memory=(point,) if remembered else ())
    route.last_scan.range_min = .28
    watch = module.rotation_collision_watch(route)
    assert watch(command(angular=angular)) == 'rotation swept footprint occupied'
    assert monitor.sets == []
    assert monitor.values['FootprintApproach.enabled'] is True
    assert monitor.values['StopZone.enabled'] is True


def test_rotation_watch_stops_on_stale_scan_but_passes_zero_and_clear_turn(monkeypatch):
    route, monitor, clock = node(monkeypatch, points=((2., 0.),))
    watch = module.rotation_collision_watch(route)
    assert watch(command(angular=.3)) is None
    clock['mono'] = clock['ros'] = 11.
    assert watch(command(angular=.3)) == 'rotation scan stale or future-dated'
    assert watch(command()) is None
    assert monitor.sets == []


def test_remembered_collision_records_original_scan_and_pose(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, memory=[(-.281, .051)])
    original = scan([(-.5,.05)], stamp_s=9.)
    route._blind_point_sources = [((1.,2.,.1), original, (-.5,.05), (1.2,2.,.1))]
    diagnostic = {
        'reason': 'three_finite_rays_not_beyond_point',
        'projected_distance_m': .325,
        'ray_index': 42,
        'rays_m': [.34, .34, .34],
    }
    route._blind_point_diagnostics = [diagnostic]
    with module.rear_collision_profile(route) as watch:
        assert watch(command(linear=-.05)) == 'rear docking swept footprint occupied'
    events = [call.kwargs for call in route.emit.call_args_list
              if call.args[0] == 'rear_collision_witness']
    origin = events[0]['remembered_source']
    assert origin['pose_odom'] == [1.,2.,.1]
    assert origin['point_base_xy_m'] == [-.5,.05]
    assert origin['scan_stamp_s'] == 9.
    assert len(origin['ranges']) == len(original.ranges)
    assert None in origin['ranges']  # JSON has no Infinity literals
    assert events[0]['remembered_clearance_diagnostic'] == diagnostic


def test_terminal_reverse_uses_measured_rear_without_masking_returns(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.28, .196)])
    with module.rear_collision_profile(route) as watch:
        watch.set_target(*target_face(current_gap_m=.0145))
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'
        watch.set_rear_extent(.275)
        assert watch(command(-.01)) is None
        # Higher travel prediction still reaches the same return and stops.
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'
        watch.clear_target()
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'


@pytest.mark.parametrize('extent', [.26, .29, float('nan'), True])
def test_terminal_rear_extent_cannot_remove_more_than_existing_padding(monkeypatch, extent):
    route, _, _ = node(monkeypatch)
    with module.rear_collision_profile(route) as watch:
        with pytest.raises(ValueError): watch.set_rear_extent(extent)


@pytest.mark.parametrize('gap,stamp,linear,angular', [
    (.04,10.,-.01,0.), (.0145,9.9,-.01,0.), (.0145,10.,0.,.1)])
def test_rear_padding_remains_outside_fresh_terminal_reverse(monkeypatch,gap,stamp,linear,angular):
    route, _, _ = node(monkeypatch, points=[(-.28,.196)])
    seen=[]
    monkeypatch.setattr(module,'path_sweep_hit',lambda points,outline,*a,**k: seen.append(outline) or False)
    with module.rear_collision_profile(route) as watch:
        watch.set_rear_extent(.275)
        watch.set_target(*target_face(stamp_s=stamp,current_gap_m=gap))
        watch(command(linear,angular))
    assert seen[-1]==OUTLINE
