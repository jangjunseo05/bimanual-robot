"""Rear-box control integration without a camera or hardware motion."""

from contextlib import contextmanager, nullcontext
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import rear_box_service as module
from jdamr_cube_navigation.box_service import BoxServiceRoute
import pytest


GEOMETRY = {
    'frame_length': {'unit': 'm', 'value': 0.34},
    'front_to_wheel_axis': {'unit': 'm', 'value': 0.065},
    'frame_width': {'unit': 'm', 'value': 0.45},
}


@pytest.fixture(autouse=True)
def isolate_collision_profile(monkeypatch):
    monkeypatch.setattr(module, 'rear_collision_profile', lambda _node: nullcontext(None))


def route_node():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._approach_rear_corridor = Mock(return_value=True)
    node._center_rear_box = Mock(return_value=True)
    return node


@pytest.mark.parametrize('center_ok', [True, False])
def test_centering_without_departure_reference_precedes_fine_profile(monkeypatch, center_ok):
    node = route_node()
    events = []
    initial = face(.2)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._approach_rear_corridor = Mock(side_effect=lambda *_a, **_k: events.append('coarse') or True)
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._center_rear_box = Mock(side_effect=lambda *_a: events.append('center') or center_ok)
    node._park_rear_face = Mock(side_effect=lambda *_a: events.append('fine') or True)
    node.emit = Mock()

    @contextmanager
    def profile(_node):
        events.append('profile_enter')
        yield None
        events.append('profile_exit')

    monkeypatch.setattr(module, 'rear_collision_profile', profile)
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01) is center_ok
    assert events == (['coarse', 'center', 'profile_enter', 'fine', 'profile_exit']
                      if center_ok else ['coarse', 'center'])
    assert node._approach_rear_corridor.call_args.kwargs == {
        'handoff_m': module.REAR_CENTER_HANDOFF_M}
    if not center_ok:
        node._park_rear_face.assert_not_called()


@pytest.mark.parametrize('clearance_ok', [True, False])
def test_near_off_center_box_gets_curve_clearance_before_fine(monkeypatch, clearance_ok):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    initial = face(.01)
    initial.update(face_odom=(-.285, 0.), midpoint_odom=(-.285, .225))
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._clear_rear_dock_straight = Mock(return_value=clearance_ok)
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    monkeypatch.setattr(module, 'load_route', lambda _p: {
        'dock_departure_distance_m': .7,
        'waypoints': [{'id': 'home_exit'}, {'id': 'observation'}]})
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01) is clearance_ok
    node._clear_rear_dock_straight.assert_called_once_with(.99)
    assert 'dock_departure_distance_m' not in node.config
    if clearance_ok:
        node._center_rear_box.assert_called_once_with(
            initial, pytest.approx(.275), after_clearance=True)
        node._observe_rear_box_recovering.assert_called_once()
    else:
        node._center_rear_box.assert_not_called()
        node._park_rear_face.assert_not_called()


@pytest.mark.parametrize('lateral', [-.23, .23])
def test_visible_face_is_centered_by_reverse_without_unnecessary_forward(lateral):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    initial, centered = face(.85), face(.2)
    for observation in (initial, centered):
        observation.update(face_odom=(-1.125, 0.), midpoint_odom=(-1.125, lateral))
    centered['base_pose_odom'] = (-.65, lateral, 0.)
    node._observe_rear_box_recovering = Mock(side_effect=[initial, centered])
    node._odom_pose = Mock(return_value=(-.65, lateral, 0.))
    def center(*_a, **_k):
        node._rear_center_moved = True
        return True
    node._center_rear_box = Mock(side_effect=center)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unnecessary forward'))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    assert node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90, target_gap_m=.01)
    node._approach_rear_corridor.assert_called_once_with(
        GEOMETRY, pytest.approx(.275), pytest.approx(.225),
        handoff_m=module.REAR_CENTER_HANDOFF_M)
    node._clear_rear_dock_straight.assert_not_called()
    node._center_rear_box.assert_called_once_with(initial, pytest.approx(.275))
    node._park_rear_face.assert_called_once()


def test_feasible_reverse_curve_absorbs_heading_without_separate_rotation():
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    angle = -.1
    initial = face(.75, angle)
    normal = math.cos(angle), math.sin(angle)
    tangent = -normal[1], normal[0]
    distance = .275 + .75
    initial['midpoint_odom'] = (
        -distance * normal[0] + .12 * tangent[0],
        -distance * normal[1] + .12 * tangent[1])
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._align_rear_face_before_clearance = Mock(
        side_effect=AssertionError('separate rotation is unnecessary'))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()

    assert node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90,
        target_gap_m=.01)
    node._align_rear_face_before_clearance.assert_not_called()
    node._center_rear_box.assert_called_once_with(initial, pytest.approx(.275))


@pytest.mark.parametrize('invalid', [math.nan, 3.])
def test_invalid_or_excessive_center_curve_does_not_request_forward(invalid):
    observed = face(.85)
    observed['midpoint_odom'] = (invalid, .23)
    with pytest.raises(ValueError):
        module._rear_center_curve_available(observed, .275)


def test_clearance_carries_accepted_odom_face_and_reconfirms_after_centering(monkeypatch):
    node = route_node()
    initial, centered = face(.4), face(.2)
    for observation in (initial, centered):
        observation.update(face_odom=(-.675, 0.), midpoint_odom=(-.675, .225))
    order = []
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._rear_dock_reference = {'recorded': True}
    node._rear_reference_stage = Mock(return_value={})
    node.registry = {'home': {}}
    node._localization_bound = lambda *_a: nullcontext()
    node._align_rear_reference = Mock(side_effect=lambda *_a, **_k: order.append('align') or True)
    observations = iter([initial, centered])
    def observe(*_a, **_k):
        order.append('observe')
        return next(observations)
    node._observe_rear_box_recovering = Mock(side_effect=observe)
    node._clear_rear_dock_straight = Mock(side_effect=lambda *_a: order.append('clearance') or True)
    def center(observation, _extent, *, after_clearance):
        assert observation is initial and after_clearance
        order.append('center')
        node._rear_center_moved = True
        return True
    node._center_rear_box = Mock(side_effect=center)
    node._odom_pose = Mock(return_value=(0., .225, 0.))
    node._park_rear_face = Mock(side_effect=lambda *_a: order.append('fine') or True)
    node.emit = Mock()
    monkeypatch.setattr(module, 'load_route', lambda _p: {
        'waypoints': [{'id': 'exit'}, {'id': 'observation'}]})
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    assert order == ['observe', 'align', 'clearance', 'center', 'observe', 'fine']
    assert node._align_rear_reference.call_args.kwargs['observed'] is initial
    assert node._park_rear_face.call_args.args[0] is centered


def test_rear_readiness_failure_records_battery_cause_without_motion():
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=False)
    node.last_readiness_failure = 'battery low: voltage=10.192V limit=10.500V'
    node.emit = Mock()
    assert not node.park_rear_box(GEOMETRY, 'route', execute=True, timeout_s=90)
    node.emit.assert_called_once_with(
        'failed', phase='rear_box_readiness', reason=node.last_readiness_failure)
    node._approach_rear_corridor.assert_not_called()


@pytest.mark.parametrize('interruption', [None, 'stop', 'deadline'])
def test_center_curve_residual_never_starts_another_forward_clearance(
        monkeypatch, interruption):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    initial, centered = face(.3, stamp=1.), face(.2, stamp=2.)
    for observation, lateral in ((initial, .23), (centered, .057698)):
        observation.update(face_odom=(-.575, 0.), midpoint_odom=(-.575, 0.),
                           base_pose_odom=(0., lateral, 0.))
    node._observe_rear_box_recovering = Mock(side_effect=[initial, centered])
    node._align_rear_face_before_clearance = Mock(side_effect=lambda found, *_a: found)
    node._clear_rear_dock_straight = Mock(return_value=True)
    node._odom_pose = Mock(return_value=(0., .057698, 0.))
    def center(*_a, **_k):
        node._rear_center_moved = True
        if interruption == 'stop':
            node.stop_requested = True
        elif interruption == 'deadline':
            node.run_deadline_s = time.monotonic() - 1.
        return True
    node._center_rear_box = Mock(side_effect=center)
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    monkeypatch.setattr(module, 'load_route', lambda _p: {
        'waypoints': [{'id': 'home_dock_approach'}]})
    assert not node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90, target_gap_m=.01)
    node._center_rear_box.assert_called_once()
    node._clear_rear_dock_straight.assert_called_once()
    node._park_rear_face.assert_not_called()


@pytest.mark.parametrize('interrupted', [False, True])
def test_face_heading_is_corrected_before_a_large_straight_clearance(interrupted):
    node = route_node()
    initial, aligned = face(.4, math.radians(35.)), face(.4)
    for observation in (initial, aligned):
        observation.update(normal_odom=(1., 0.), face_odom=(-.675, 0.))
    node.stop_requested = interrupted
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rotate_in_place = Mock(return_value=math.radians(35.))
    node._observe_rear_box_recovering = Mock(return_value=aligned)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('translation before alignment'))
    node.emit = Mock()
    if interrupted:
        with pytest.raises(RuntimeError, match='operator_stop'):
            node._align_rear_face_before_clearance(initial, GEOMETRY, .275, .225)
        node._rotate_in_place.assert_not_called()
    else:
        assert node._align_rear_face_before_clearance(
            initial, GEOMETRY, .275, .225) is aligned
        node._rotate_in_place.assert_called_once_with(math.radians(35.))
    node._clear_rear_dock_straight.assert_not_called()



def test_rear_heading_overshoot_reduces_next_command():
    node = route_node()
    initial = face(.2, math.radians(6.4))
    overshot = face(.2, math.radians(-7.1))
    aligned = face(.2, math.radians(1.))
    for observation in (initial, overshot, aligned):
        observation.update(normal_odom=(1., 0.), face_odom=(-.475, 0.))
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rotate_in_place = Mock(return_value=.1)
    node._observe_rear_box_recovering = Mock(side_effect=[overshot, aligned])
    node.emit = Mock()
    assert node._align_rear_face_before_clearance(initial, GEOMETRY, .275, .225) is aligned
    commands = [call.args[0] for call in node._rotate_in_place.call_args_list]
    assert commands == pytest.approx([math.radians(6.4), math.radians(-7.1) * 6.4 / 13.5])

def face(gap=0.01, angle=0.0, stamp=5.0):
    normal = math.cos(angle), math.sin(angle)
    tangent = -normal[1], normal[0]
    center = -0.275-gap, 0.
    return {'gap_m': gap, 'normal_yaw_error_rad': angle, 'stamp_s': stamp,
            'span_m': .35, 'face_center_base_xy_m': center,
            'normal_base_xy': normal, 'tangent_base_xy': tangent,
            'line_offset_m': sum(center[i] * normal[i] for i in (0, 1)),
            'tangent_bounds_m': (-.175, .175), 'residual_rms_m': .001,
            'face_odom': (-0.275-gap, 0.),
            'normal_odom': normal, 'midpoint_odom': center,
            'base_pose_odom': (0., 0., 0.), 'observation_roi_clipped': False}


def _historical_face_scan(x_m):
    count = 7201
    increment = 2*math.pi/(count-1)
    ranges = [math.inf] * count
    for index in range(101):
        y_m = -.175 + .35*index/100
        scan_x, scan_y = -(x_m+.01), -y_m
        bearing = math.atan2(scan_y, scan_x)
        beam = round((bearing+math.pi)/increment)
        ranges[beam] = math.hypot(scan_x, scan_y)
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                           angle_increment=increment, range_min=.28, range_max=16.)


def _source_face_classifier():
    node = route_node()
    initial, final = face(.2), face(.01)
    final['face_odom'] = initial['face_odom']
    node._observe_rear_box_recovering = Mock(return_value=final)
    node._rear_box_update = Mock(return_value=Mock(return_value={'remaining_m': 0.}))
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node.emit = Mock()
    watch = Mock()
    geometry = {**GEOMETRY,
                'laser_translation': {'unit': 'm', 'value': [-.01, 0., .15]},
                'laser_yaw': {'unit': 'rad', 'value': math.pi}}
    assert node._park_rear_face(initial, geometry, 'route', .275, .225, .01, watch)
    return watch.set_target.call_args.args[3]


def test_real_source_face_classifier_preserves_non_target_points():
    classifier = _source_face_classifier()
    target = _historical_face_scan(-.475)
    assert classifier((0., 0., 0.), target, (-.475, 0.))
    assert not classifier((0., 0., 0.), target, (-.471, 0.))
    assert not classifier((0., 0., 0.), target, (-.475, .3))
    other = _historical_face_scan(-.65)
    assert not classifier((0., 0., 0.), other, (-.65, 0.))


def test_source_face_cache_checks_scan_identity_and_acquisition_pose(monkeypatch):
    monkeypatch.setattr(module, 'id', lambda _scan: 7, raising=False)
    classifier = _source_face_classifier()
    target, other = _historical_face_scan(-.475), _historical_face_scan(-.65)
    assert classifier((0., 0., 0.), target, (-.475, 0.))
    assert not classifier((0., 0., 0.), other, (-.65, 0.))
    assert classifier((0., 0., 0.), target, (-.475, 0.))
    assert not classifier((.2, 0., 0.), target, (-.475, 0.))
    assert classifier((0., 0., 0.), target, (-.475, 0.))


@pytest.mark.parametrize('gap,error,remaining,moved', [
    (.004, -.035, -.006, None),
    (.20, 0., .19, -.19),
    (.20, 0., .19, None),
])
def test_fine_rear_approach_never_pulls_forward(gap, error, remaining, moved):
    node = route_node()
    initial, final = face(gap, error, stamp=1.), face(.01, 0., stamp=2.)
    final['face_odom'] = initial['face_odom']
    node._observe_rear_box_recovering = Mock(return_value=final)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unexpected forward'))
    update = Mock(return_value={'remaining_m': remaining})
    node._rear_box_update = Mock(return_value=update)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=moved)
    node.emit = Mock()
    result = node._park_rear_face(initial, GEOMETRY, 'route', .275, .225, .01, None)
    assert result is (remaining <= 0. or moved is not None)
    node._clear_rear_dock_straight.assert_not_called()
    if remaining <= 0.:
        node._drive_zero_turn.assert_not_called()
    else:
        assert node._drive_zero_turn.call_args.args[0] < 0.
        node._drive_zero_turn.assert_called_once()


def test_terminal_corner_miss_is_reported_without_forward_or_reapproach():
    node = route_node()
    initial, missed = face(.2, 0., stamp=1.), face(.01085, math.radians(.548), stamp=2.)
    missed['face_odom'] = initial['face_odom']
    node._observe_rear_box_recovering = Mock(return_value=missed)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unexpected forward'))
    node._rear_box_update = Mock(return_value=Mock(return_value={'remaining_m': .19}))
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=-.19)
    node.emit = Mock()
    assert not node._park_rear_face(initial, GEOMETRY, 'route', .275, .225, .01, Mock())
    node._drive_zero_turn.assert_called_once()
    node._clear_rear_dock_straight.assert_not_called()
    node._observe_rear_box_recovering.assert_called_once()
    assert node.confirmation['confirmed'] is False
    assert node.confirmation['nearest_gap_m'] == pytest.approx(.01085)
    assert node.confirmation['farthest_gap_m'] > .013


def test_terminal_miss_does_not_retry_after_operator_link_loss():
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0., 0., 0.), {}))
    initial = face(.2, 0., stamp=1.)
    missed = face(.01085, math.radians(.548), stamp=2.)
    for item in (initial, missed):
        item['face_odom'], item['normal_odom'] = (-.475, 0.), (1., 0.)
    node._observe_rear_box = Mock(side_effect=[initial, missed])
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._operator_link_ready = Mock(side_effect=[True, True, False])
    node._clear_rear_dock_straight = Mock(return_value=True)
    update = Mock(return_value={'remaining_m': .19})
    node._rear_box_update = Mock(return_value=update)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=-.19)
    node.emit = Mock()

    assert not node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90)
    node._clear_rear_dock_straight.assert_not_called()
    node._rear_box_update.assert_called_once()
    assert node._observe_rear_box.call_count == 2
    assert node.confirmation['reason'] == 'rear_box_gap_out_of_tolerance'


def test_far_corridor_moves_before_any_precision_face_is_required():
    node = route_node()
    del node._approach_rear_corridor
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(side_effect=[
        {'gap_m': 1.45}, {'gap_m': .95}, {'gap_m': .5}])
    node._rear_coarse_step = Mock(return_value=True)
    node._rear_box_observation = Mock(side_effect=AssertionError('fine face not required yet'))
    node.run_deadline_s = time.monotonic() + 10.
    node.emit = Mock()
    assert node._approach_rear_corridor(GEOMETRY, .275, .225)
    assert [call.args[3] for call in node._rear_coarse_step.call_args_list] == (
        pytest.approx([.5, .46]))
    assert all(call.args[2] == pytest.approx(.275)
               for call in node._rear_coarse_step.call_args_list)
    node._rear_box_observation.assert_not_called()


@pytest.mark.parametrize('clear', [True, False])
def test_coarse_reverse_validates_static_path_before_command(monkeypatch, clear):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._drive_zero_turn = Mock(return_value=-.49)
    node.emit = Mock()
    check = Mock(return_value=clear)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock(return_value=check))
    if not clear:
        with pytest.raises(RuntimeError, match='static corridor blocked'):
            node._rear_coarse_step(GEOMETRY, .275, .275, .5)
        node._drive_zero_turn.assert_not_called()
        return
    assert node._rear_coarse_step(GEOMETRY, .275, .275, .5)
    path = check.call_args.args[0]
    assert path[0] == pytest.approx((0., 0., 0.))
    assert path[-1] == pytest.approx((-.5, 0., 0.))
    assert node._drive_zero_turn.call_args.args[0] == -.5


def test_coarse_range_reaches_handoff_without_a_plane(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .49})
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=True)))

    def drive(_distance, **kwargs):
        reason = kwargs['watch']()
        assert reason == 'rear box coarse handoff ready'
        node._last_direct_motion_failure = reason
        return None
    node._drive_zero_turn = Mock(side_effect=drive)
    assert node._rear_coarse_step(GEOMETRY, .275, .275, .5)


def test_range_reobserve_stops_only_after_face_is_stable_in_odom(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(
        None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .75})
    observations = []
    for stamp in range(1, 9):
        observed = face(.7, stamp=float(stamp))
        observed.update(
            normal_odom=(1., 0.), face_odom=(-1.0, 0.))
        observations.append(observed)
    node._rear_box_observation = Mock(side_effect=observations)
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=True)))

    def drive(_distance, **kwargs):
        reasons = [kwargs['watch']() for _ in range(8)]
        assert reasons[:7] == [None] * 7
        reason = reasons[-1]
        assert reason == 'rear box face stable'
        node._last_direct_motion_failure = reason
        return None

    node._drive_zero_turn = Mock(side_effect=drive)
    assert node._rear_coarse_step(
        GEOMETRY, .275, .275, .2, handoff_m=.2,
        stop_when_face_visible=True, face_half_width_m=.225)
    assert node._rear_box_observation.call_count == 8
    node._rear_box_observation.assert_called_with(GEOMETRY, .275, .225)


def test_range_reobserve_does_not_stop_for_one_transient_face(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(side_effect=[(0., 0., 0.)] + [(-.2, 0., 0.)] * 4)
    node._frozen_in_odom = Mock(return_value=(
        None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .75})
    node._rear_box_observation = Mock(side_effect=[
        face(.7, stamp=1.),
        RuntimeError('rear box face unavailable'),
        RuntimeError('rear box face unavailable'),
    ])
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=True)))

    def drive(_distance, **kwargs):
        assert kwargs['watch']() is None
        assert kwargs['watch']() is None
        assert kwargs['watch']() is None
        return -.2

    node._drive_zero_turn = Mock(side_effect=drive)
    assert node._rear_coarse_step(
        GEOMETRY, .275, .275, .2, handoff_m=.2,
        stop_when_face_visible=True, face_half_width_m=.225)
    assert node._rear_box_observation.call_count == 3


def test_temporary_face_loss_waits_in_same_task_then_recovers():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    observed = face(.4)
    node._observe_rear_box = Mock(side_effect=[
        RuntimeError('rear box observation unavailable: rear box unstable'), observed])
    node.emit = Mock()
    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True) is observed
    assert node._observe_rear_box.call_count == 2
    assert node.emit.call_args.args[0] == 'rear_box_waiting_for_observation'


def test_invalid_scan_is_fitted_and_reported_once_per_stamp(monkeypatch):
    node = route_node()
    node._rear_box_scan = Mock(return_value=(None, 5.))
    node._rear_scan_values = Mock(return_value={})
    node.emit = Mock()
    fit = Mock(side_effect=ValueError('rear face tangent spread is too narrow'))
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    for _ in range(3):
        with pytest.raises(RuntimeError, match='rear box face unavailable'):
            node._rear_box_observation(GEOMETRY, .275, .225)
    assert fit.call_count == 3
    assert node.emit.call_count == 1
    node._rear_box_scan.return_value = None, 5.1
    with pytest.raises(RuntimeError):
        node._rear_box_observation(GEOMETRY, .275, .225)
    assert fit.call_count == 6


def test_missing_near_face_reobserves_after_bounded_live_range_advance():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    observed = face(.2)
    node._observe_rear_box = Mock(side_effect=[RuntimeError(
        'rear box observation unavailable: rear box face unavailable'), observed])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .35})
    node._rear_coarse_step = Mock(return_value=True)
    node.emit = Mock()
    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True) is observed
    call = node._rear_coarse_step.call_args
    assert call.args[3] == pytest.approx(.15)
    assert call.kwargs['handoff_m'] == pytest.approx(.2)


def test_missing_face_never_advances_past_live_range_reobserve_gap():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    observed = face(.19)
    node._observe_rear_box = Mock(side_effect=[RuntimeError(
        'rear box observation unavailable: rear box face unavailable'), observed])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .19})
    node._rear_coarse_step = Mock()
    node.emit = Mock()
    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True) is observed
    node._rear_coarse_step.assert_not_called()


def test_logged_handoff_keeps_approaching_until_the_box_face_is_visible():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    observed = face(.2)
    missing = RuntimeError('rear box observation unavailable: rear box face unavailable')
    node._observe_rear_box = Mock(side_effect=[missing, missing, missing, observed])
    node._rear_corridor_observation = Mock(side_effect=[
        {'gap_m': .654145928792223}, {'gap_m': .454145928792223},
        {'gap_m': .254145928792223}])
    node._rear_coarse_step = Mock(return_value=True)
    node.emit = Mock()
    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True) is observed
    assert [call.args[3] for call in node._rear_coarse_step.call_args_list] == pytest.approx(
        [.2, .2, .054145928792223])
    assert all(call.kwargs['handoff_m'] == .2
               for call in node._rear_coarse_step.call_args_list)
    assert all(call.kwargs['timeout_s'] == 1.
               for call in node._observe_rear_box.call_args_list)
    assert not any(call.args[0] == 'rear_box_waiting_for_observation'
                   for call in node.emit.call_args_list)


def test_blocked_range_reapproach_does_not_repeat_or_start_fine():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._observe_rear_box = Mock(side_effect=RuntimeError(
        'rear box observation unavailable: rear box face unavailable'))
    node._rear_corridor_observation = Mock(return_value={'gap_m': .65})
    node._rear_coarse_step = Mock(return_value=False)
    node.emit = Mock()
    with pytest.raises(RuntimeError, match='rear box range reobservation blocked'):
        node._observe_rear_box_recovering(
            GEOMETRY, .275, .225, allow_range_reapproach=True)
    node._rear_coarse_step.assert_called_once()


@pytest.mark.parametrize('reason', [
    'operator_stop', 'operator_link_lost', 'rear box scan unavailable',
    'rear docking swept footprint occupied'])
def test_range_reapproach_preserves_the_reason_for_stopping(reason):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._observe_rear_box = Mock(side_effect=RuntimeError(
        'rear box observation unavailable: rear box face unavailable'))
    node._rear_corridor_observation = Mock(return_value={'gap_m': .65})
    def interrupted(*_args, **_kwargs):
        if reason == 'operator_stop':
            node.stop_requested = True
        elif reason == 'operator_link_lost':
            node._operator_link_ready.return_value = False
        else:
            node._last_direct_motion_failure = reason
        return False
    node._rear_coarse_step = Mock(side_effect=interrupted)
    node.emit = Mock()
    with pytest.raises(RuntimeError, match=reason):
        node._observe_rear_box_recovering(
            GEOMETRY, .275, .225, allow_range_reapproach=True)
    node._rear_coarse_step.assert_called_once()


@pytest.mark.parametrize('range_gap', [.85, 1.0])
def test_explicit_range_recovery_handoff_is_respected(range_gap):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    observed = face(.9)
    node._observe_rear_box = Mock(side_effect=[RuntimeError(
        'rear box observation unavailable: rear box face unavailable'), observed])
    node._rear_corridor_observation = Mock(return_value={'gap_m': range_gap})
    node._rear_coarse_step = Mock(return_value=True)
    node.emit = Mock()
    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True,
        range_handoff_m=.9) is observed
    if range_gap > .9:
        assert node._rear_coarse_step.call_args.args[3] == pytest.approx(.1)
        assert node._rear_coarse_step.call_args.kwargs['handoff_m'] == .9
    else:
        node._rear_coarse_step.assert_not_called()


def test_operator_stop_during_face_wait_never_starts_range_reapproach():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)

    def stopped(*_args, **_kwargs):
        node.stop_requested = True
        raise RuntimeError('rear box observation unavailable: rear box face unavailable')
    node._observe_rear_box = Mock(side_effect=stopped)
    node._rear_corridor_observation = Mock()
    node._rear_coarse_step = Mock()
    with pytest.raises(RuntimeError, match='operator_stop'):
        node._observe_rear_box_recovering(
            GEOMETRY, .275, .225, allow_range_reapproach=True)
    node._rear_corridor_observation.assert_not_called()
    node._rear_coarse_step.assert_not_called()


def test_latched_face_confirmation_never_starts_range_only_motion():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    observed = face(.01)
    node._observe_rear_box = Mock(side_effect=[RuntimeError(
        'rear box observation unavailable: rear box face unavailable'), observed])
    node._rear_corridor_observation = Mock()
    node._rear_coarse_step = Mock()
    node.emit = Mock()
    assert node._observe_rear_box_recovering(GEOMETRY, .275, .225) is observed
    node._rear_corridor_observation.assert_not_called()
    node._rear_coarse_step.assert_not_called()


@pytest.mark.parametrize('target_gap', [True, 0., .009, .101, math.inf])
def test_explicit_rear_target_gap_reuses_parking_contract_bounds(target_gap):
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    with pytest.raises(ValueError, match=r'\[0.01, 0.10\]'):
        node.park_rear_box(
            GEOMETRY, 'route', execute=False, timeout_s=90,
            target_gap_m=target_gap)
    node._final_gap_m.assert_not_called()


def test_fine_rear_docking_uses_live_face_without_static_map_gate(monkeypatch):
    node = route_node()
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._rear_box_observation = Mock(return_value=face(.2))
    node.emit = Mock()
    factory = Mock(side_effect=AssertionError('static maps cannot gate live dock fine control'))
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', factory)
    update = node._rear_box_update(face(.2), GEOMETRY, .275, .225, .01)
    result = update((0., 0., 0.))
    assert result['remaining_m'] == pytest.approx(.19)
    assert result['heading_rad'] == pytest.approx(0.)
    assert result.get('validate_control') is None
    factory.assert_not_called()


@pytest.mark.parametrize('failure', ['scan', 'heading', 'travel'])
def test_live_dock_fine_control_keeps_input_heading_and_travel_guards(failure):
    node = route_node()
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    initial = face(.2, math.radians(40.) if failure == 'heading' else 0.)
    node._rear_box_observation = Mock(return_value=initial)
    expected = {'scan': 'scan unavailable', 'heading': 'heading outside',
                'travel': 'bounded travel'}[failure]
    if failure == 'scan':
        node._rear_box_observation.side_effect = RuntimeError('rear box scan unavailable')
    update = node._rear_box_update(initial, GEOMETRY, .275, .225, .01)
    with pytest.raises(RuntimeError, match=expected):
        update((.5, 0., 0.) if failure == 'travel' else (0., 0., 0.))


@pytest.mark.parametrize('alignment_ok', [True, False])
def test_full_width_landmark_is_matched_after_coarse_and_never_returns_to_stage(alignment_ok):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._rear_dock_reference = {'observed': True}
    node.registry = {'home': {'id': 'home'}}
    node._rear_reference_stage = Mock(return_value={'id': 'home_dock_approach'})
    node._localization_bound = lambda _intermediate: nullcontext()
    sequence = Mock()
    sequence.attach_mock(node._approach_rear_corridor, 'coarse')
    initial = face(.48)
    node._observe_rear_box_recovering = Mock(return_value=initial)
    sequence.attach_mock(node._observe_rear_box_recovering, 'observe')
    node._align_rear_reference = Mock(return_value=alignment_ok)
    sequence.attach_mock(node._align_rear_reference, 'align')
    sequence.attach_mock(node._center_rear_box, 'center')
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()

    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01) is alignment_ok
    assert [call[0] for call in sequence.mock_calls] == (
        ['coarse', 'observe', 'align', 'center'] if alignment_ok else
        ['coarse', 'observe', 'align'])
    node._align_rear_reference.assert_called_once_with(
        {'id': 'home_dock_approach'}, node.registry['home'],
        observed=initial, return_to_stage=False)
    if alignment_ok:
        assert node._park_rear_face.call_args.args[0] is initial
    else:
        node._park_rear_face.assert_not_called()


@pytest.mark.parametrize('age,frame', [(1., 'laser_link'), (0., 'wrong_frame')])
def test_bad_scan_stops_before_fitting_or_any_motion(monkeypatch, age, frame):
    node = route_node()
    node.last_scan = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=5, nanosec=0), frame_id=frame))
    node.samples = {'scan': 5.0}
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5e9))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 5.0+age)
    fit = Mock()
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    with pytest.raises(RuntimeError):
        node._rear_box_observation(GEOMETRY, .275, .225)
    fit.assert_not_called()


@pytest.mark.parametrize('change', ['position', 'normal', 'clock'])
def test_live_scan_cannot_switch_target_or_regress_time(monkeypatch, change):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    next_face = face(.2, stamp=6.)
    if change == 'position':
        next_face['face_odom'] = (-1., 0.)
    elif change == 'normal':
        next_face['normal_odom'] = (0., 1.)
    else:
        next_face['stamp_s'] = 4.
    node._rear_box_observation = Mock(return_value=next_face)
    command_watch = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(
        face(.2), GEOMETRY, .275, .225, .01,
        command_watch=command_watch)
    with pytest.raises(RuntimeError):
        update((0., 0., 0.))
    command_watch.set_target.assert_not_called()


def test_latest_verified_fine_face_refreshes_collision_target(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(
        None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    observation = face(.0175, stamp=6.)
    node._rear_box_observation = Mock(return_value=observation)
    node.emit = Mock()
    command_watch = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())

    update = node._rear_box_update(
        face(.02, stamp=5.), GEOMETRY, .275, .225, .01,
        command_watch=command_watch)
    update((0., 0., 0.))

    command_watch.set_target.assert_called_once_with(
        observation, .01, .0175, None)


@pytest.mark.parametrize('change', ['position', 'normal'])
def test_small_cumulative_changes_cannot_move_the_latched_target(monkeypatch, change):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    node.emit = Mock()
    observations = []
    for i in (1, 2):
        item = face(.2, stamp=5.+i)
        if change == 'position':
            item['face_odom'] = (-.475 - .039*i, 0.)
        else:
            angle = math.radians(7.*i)
            item['normal_odom'] = (math.cos(angle), math.sin(angle))
        observations.append(item)
    node._rear_box_observation = Mock(side_effect=observations)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(face(.2), GEOMETRY, .275, .225, .01)
    update((0., 0., 0.))
    with pytest.raises(RuntimeError, match='target changed'):
        update((0., 0., 0.))


def test_final_confirmation_rejects_a_different_box_at_the_right_gap():
    with pytest.raises(RuntimeError, match='target changed'):
        module._same_target(face(.2), {**face(.01), 'face_odom': (-1., 0.)})


@pytest.mark.parametrize('translation', [(0., 0.), (125., -84.)])
def test_same_target_is_invariant_to_odom_origin_and_small_normal_noise(translation):
    support = translation[0] - .475, translation[1] + .18

    def fitted(angle):
        observed = face(.2, angle)
        normal = math.cos(angle), math.sin(angle)
        offset = sum(normal[i] * support[i] for i in (0, 1))
        observed.update(
            normal_odom=normal,
            face_odom=tuple(offset * value for value in normal),
            midpoint_odom=support,
        )
        return observed

    module._same_target(fitted(0.), fitted(math.radians(2.)))


def test_same_target_rejects_a_locally_displaced_parallel_face():
    reference = face(.2)
    reference.update(face_odom=(10., -8.), midpoint_odom=(10., -8.))
    observed = face(.2)
    observed.update(face_odom=(10.08, -8.), midpoint_odom=(10.08, -8.))
    with pytest.raises(RuntimeError, match='target changed'):
        module._same_target(reference, observed)


def test_one_noisy_close_range_scan_does_not_interrupt_an_aligned_approach(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    node._rear_box_observation = Mock(return_value=face(
        .0274, -.0053988349885327755, stamp=5.1))
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(face(.0274), GEOMETRY, .275, .225, .01)
    result = update((0., 0., 0.))
    assert abs(result['heading_rad']) < math.radians(.1)
    assert 0.017 < result['remaining_m'] < 0.018
    repeated = update((0., 0., 0.))
    assert repeated['heading_rad'] == result['heading_rad']
    assert repeated['remaining_m'] == result['remaining_m']


def test_fine_rear_approach_centers_both_corner_gaps_on_target():
    node = route_node()
    angle = math.radians(.5)
    initial = face(.02, angle, stamp=5.)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._rear_box_observation = Mock(return_value=initial)
    node.emit = Mock()

    result = node._rear_box_update(
        initial, GEOMETRY, .275, .225, .01)((0., 0., 0.))
    expected = (result['center_gap_m'] - .01) / math.cos(
        result['heading_rad'])
    assert result['remaining_m'] == pytest.approx(expected)
    assert result['center_gap_m'] == pytest.approx(
        (result['nearest_gap_m'] + result['farthest_gap_m']) / 2.)


def test_stopped_confirmation_uses_all_eight_scans_not_the_last_angle_outlier():
    observations = [face(.01, 0., stamp=5.+i*.1) for i in range(7)]
    observations.append(face(.01, math.radians(.31), stamp=5.7))
    result = module._stationary_face(observations, .275, .225)
    assert result['averaged_frames'] == 8
    assert math.degrees(result['normal_yaw_error_rad']) == pytest.approx(.03875, abs=1e-6)
    far_gap = result['gap_m'] + .45 * abs(math.sin(result['normal_yaw_error_rad']))
    assert .007 <= result['gap_m'] <= far_gap <= .013
    normal = result['normal_base_xy']
    point = result['face_center_base_xy_m']
    assert sum(normal[i]*point[i] for i in (0, 1)) == pytest.approx(result['line_offset_m'])


def test_stopped_face_keeps_odom_plane_anchor_near_sensor_support():
    support = (125., -84.)
    observations = []
    for index, angle in enumerate(math.radians(value) for value in (-1., -.5, .5, 1.)):
        observed = face(.2, angle, stamp=5. + index * .1)
        normal = math.cos(angle), math.sin(angle)
        offset = sum(normal[i] * support[i] for i in (0, 1))
        observed.update(
            normal_odom=normal,
            face_odom=tuple(offset * value for value in normal),
            midpoint_odom=support,
        )
        observations.append(observed)

    result = module._stationary_face(observations, .275, .225)

    assert result['face_odom'] == pytest.approx(support)
    assert result['midpoint_odom'] == pytest.approx(support)


@pytest.mark.parametrize('raw_gap,angle,complete', [
    (.012, 0., True), (.007, math.radians(.7), True),
    (.014, 0., False), (.011, math.radians(.6), False),
])
def test_fresh_corner_confirmation_can_end_filtered_rear_approach(raw_gap, angle, complete):
    node = route_node()
    initial = face(.205, 0., stamp=1.)
    observed = face(raw_gap, angle, stamp=1.01)
    actual = (-.188, 0., 0.)
    normal = math.cos(angle), math.sin(angle)
    offset = (actual[0] * normal[0] - .275 * math.cos(angle)
              - .225 * abs(math.sin(angle)) - raw_gap)
    observed.update(normal_odom=normal, face_odom=tuple(offset*n for n in normal))
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._rear_box_observation = Mock(return_value=observed)
    node.emit = Mock()
    update = node._rear_box_update(initial, GEOMETRY, .275, .225, .01)
    result = update(actual)
    assert result['target_confirmed'] is complete
    if complete:
        assert result['remaining_m'] == 0.
    else:
        assert result['remaining_m'] > 0.


def test_centering_aligns_skewed_face_before_rejecting_midpoint_residual(monkeypatch):
    node = route_node()
    initial, skewed, aligned = face(.77), face(.139, .379), face(.2)
    for observed in (initial, skewed, aligned):
        observed.update(normal_odom=(1.,0.), face_odom=(-.475,0.))
    skewed['midpoint_odom'] = (-.475, .099)
    aligned['midpoint_odom'] = (-.475, 0.)
    for name in ('verify_live_maps','_confirm_collision_monitor_on'):
        setattr(node,name,Mock())
    for name in ('wait_until_ready','_reverse_smoother_ready','_parking_parameters_ready'):
        setattr(node,name,Mock(return_value=True))
    monkeypatch.setattr(module,'_rear_center_curve_available',lambda *_a: True)
    node._observe_rear_box_recovering = Mock(side_effect=[initial,skewed])
    def center(*_a):
        node._rear_center_moved = True
        return True
    node._center_rear_box = center
    order=[]
    node._align_rear_face_before_clearance = Mock(side_effect=lambda *_a: order.append('align') or aligned)
    node._odom_pose = Mock(return_value=(0.,.003,0.))
    node._park_rear_face = Mock(side_effect=lambda *_a: order.append('fine') or True)
    node.emit = Mock()
    assert node.park_rear_box(GEOMETRY,'route',execute=True,timeout_s=90,target_gap_m=.01)
    assert order == ['align','fine']
    assert node._park_rear_face.call_args.args[0] is aligned


def test_small_lateral_offset_at_57cm_skips_curve():
    node = route_node()
    observed = face(.57)
    observed.update(observation_roi_clipped=False, midpoint_odom=(-.845,0.), normal_odom=(1.,0.))
    node._odom_pose = Mock(return_value=(0.,.003,0.))
    node.emit = Mock()
    node._make_reverse_path = Mock(side_effect=AssertionError('unnecessary curve'))
    assert module.RearBoxParkingMixin._center_rear_box(node,observed,.275)
    assert node._rear_center_skipped and not node._rear_center_moved


@pytest.mark.parametrize('angle', [-.0466, 0., .0466])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_longer_reverse_curve_avoids_forward_clearance_at_five_cm_offset(angle, sign):
    # Nov 2 log: gap=1-.507577504, lateral=-.058358623. The initial
    # orientation was not logged; bracket it with explicit synthetic +/-2.67deg.
    normal = math.cos(angle), math.sin(angle)
    tangent = -normal[1], normal[0]
    gap, lateral, extent = .492422496, sign*.058358623, .275
    midpoint = tuple(-(gap+extent)*normal[i]-lateral*tangent[i] for i in (0,1))
    observed = face(gap, angle)
    observed.update(normal_odom=normal, midpoint_odom=midpoint, face_odom=midpoint,
                    base_pose_odom=(0.,0.,0.), observation_roi_clipped=False)
    with pytest.raises(ValueError, match='tighter than min_radius_m'):
        module.reverse_curve_waypoints((0.,0.,0.), module._rear_center_target(observed,extent))
    selected = module._rear_center_curve_gap(observed,extent)
    assert selected in (.15,.1)
    path = module.reverse_curve_waypoints(
        (0.,0.,0.), module._rear_center_target(observed,extent,selected))
    assert path[-1] == pytest.approx(module._rear_center_target(observed,extent,selected))
    node = route_node()
    for name in ('verify_live_maps','_confirm_collision_monitor_on'):
        setattr(node,name,Mock())
    for name in ('wait_until_ready','_reverse_smoother_ready','_parking_parameters_ready'):
        setattr(node,name,Mock(return_value=True))
    node.emit = Mock()
    node._observe_rear_box_recovering = Mock(return_value=observed)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unnecessary forward'))
    node._align_rear_face_before_clearance = Mock(side_effect=AssertionError('unnecessary rotation'))
    node._park_rear_face = Mock(return_value=True)
    assert node.park_rear_box(GEOMETRY,'route',execute=True,timeout_s=90,target_gap_m=.01)
    node._clear_rear_dock_straight.assert_not_called()
    node._center_rear_box.assert_called_once_with(observed, pytest.approx(extent))
