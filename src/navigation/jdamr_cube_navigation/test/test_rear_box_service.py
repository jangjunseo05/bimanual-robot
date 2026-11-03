"""Rear-box control integration without a camera or hardware motion."""

from contextlib import contextmanager, nullcontext
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import rear_box_service as module
from jdamr_cube_navigation import restaurant_service as restaurant_module
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.restaurant_service import ServiceRoute
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
    node._align_rear_face_before_clearance = Mock(side_effect=lambda observed, *_args: observed)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._set_observer_perception = Mock(return_value=True)
    return node


@pytest.mark.parametrize('fine_ok', [True, False])
def test_straight_entry_without_departure_reference_precedes_fine_profile(monkeypatch, fine_ok):
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
    node._park_rear_face = Mock(side_effect=lambda *_a: events.append('fine') or fine_ok)
    node.emit = Mock()

    @contextmanager
    def profile(_node):
        events.append('profile_enter')
        yield None
        events.append('profile_exit')

    monkeypatch.setattr(module, 'rear_collision_profile', profile)
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01) is fine_ok
    assert events == ['coarse', 'profile_enter', 'fine', 'profile_exit']
    assert node._approach_rear_corridor.call_args.kwargs == {
        'handoff_m': module.REAR_COARSE_HANDOFF_M}
    node._center_rear_box.assert_not_called()


def test_near_off_center_box_fails_without_forward_clearance(monkeypatch):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    initial = face(.01)
    initial.update(face_odom=(-.285, 0.), midpoint_odom=(-.285, .225))
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._clear_rear_dock_straight = Mock(
        side_effect=AssertionError('one-pass centering cannot drive forward'))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    monkeypatch.setattr(module, '_rear_center_curve_available', lambda *_a: False)

    assert not node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    node._clear_rear_dock_straight.assert_not_called()
    node._center_rear_box.assert_not_called()
    node._park_rear_face.assert_not_called()
    assert node.emit.call_args.kwargs['forward_retry_requested'] is False


@pytest.mark.parametrize('lateral', [-.23, .23])
def test_off_center_visible_face_is_rejected_without_curve_or_forward(lateral):
    node = route_node()
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    initial = face(.5)
    initial.update(face_odom=(-.775, 0.), midpoint_odom=(-.775, lateral))
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unnecessary forward'))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    assert not node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90, target_gap_m=.01)
    node._approach_rear_corridor.assert_called_once_with(
        GEOMETRY, pytest.approx(.275), pytest.approx(.225),
        handoff_m=module.REAR_COARSE_HANDOFF_M)
    node._clear_rear_dock_straight.assert_not_called()
    node._center_rear_box.assert_not_called()
    node._park_rear_face.assert_not_called()


def test_heading_is_aligned_before_straight_reverse_entry():
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
    centered = face(.2, math.radians(1.))
    centered.update(face_odom=initial['face_odom'], midpoint_odom=initial['midpoint_odom'])
    node._observe_rear_box_recovering = Mock(return_value=initial)
    centered.update(normal_odom=initial['normal_odom'],
                    face_odom=initial['face_odom'], midpoint_odom=initial['midpoint_odom'])
    node._odom_pose = Mock(return_value=(0., .12, 0.))
    node._align_rear_face_before_clearance = Mock(return_value=centered)
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()

    assert node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90,
        target_gap_m=.01)
    node._align_rear_face_before_clearance.assert_called_once_with(
        initial, GEOMETRY, pytest.approx(.275), pytest.approx(.225))
    node._center_rear_box.assert_not_called()


def test_rear_staging_rotation_repeats_residual_without_reissuing_position(monkeypatch):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node.registry = {'home': {'rear_parking_reference': 'lidar_box'}}
    node.home_rear_geometry = GEOMETRY
    node.home_contract = {'yaw_tolerance_rad': math.radians(3.)}
    node._rear_wait_for_input = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((.7, 0., .1), {}), ((.7, 0., .16), {}),
        ((.7, 0., .2), {}), ((.7, 0., .29), {}),
    ])
    rotations = []

    def rotate(residual, **_kwargs):
        rotations.append(residual)
        if len(rotations) < 3:
            node._last_direct_motion_failure = 'rotation scan stale or future-dated'
            return None
        node._last_direct_motion_failure = None
        return .09

    node._rotate_in_place = Mock(side_effect=rotate)
    node.execute = Mock(return_value=True)
    node.emit = Mock()
    attempts = []

    def reach_staging(_self, stage, *, pause_on_rotation_failure=None):
        assert pause_on_rotation_failure('rotation scan unavailable')
        assert not pause_on_rotation_failure('rotation swept footprint occupied')
        attempts.append(stage)
        node.execute(final_parking=False, staging=True)
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    monkeypatch.setattr(ServiceRoute, '_reach_staging', reach_staging)
    stage = {'id': 'home_dock_approach', 'x': .7, 'y': 0., 'yaw': .3}

    assert node._reach_staging(stage) == pytest.approx((.7, 0., .29))
    assert attempts == [stage]
    node.execute.assert_called_once_with(final_parking=False, staging=True)
    assert node._rear_wait_for_input.call_count == 3
    assert all(item.args == ('rotation scan stale or future-dated',)
               and item.kwargs == {'preserve_motion_deadline': True}
               for item in node._rear_wait_for_input.call_args_list)
    assert rotations == pytest.approx([.2, .14, .1])
    for call in node._rotate_in_place.call_args_list:
        predicate = call.kwargs['pause_on_failure']
        assert predicate('rotation scan unavailable')
        assert predicate('rotation scan stale or future-dated')
        assert not predicate('rotation swept footprint occupied')
    assert node.capture_stationary_pose.call_count == 4
    assert all(not item.args and item.kwargs == {'intermediate': True}
               for item in node.capture_stationary_pose.call_args_list)


def test_rear_staging_rotation_rejects_unbounded_fresh_pose_travel(monkeypatch):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node.home_contract = {'yaw_tolerance_rad': math.radians(3.)}
    node._rear_wait_for_input = Mock(return_value=True)
    node.capture_stationary_pose = Mock(side_effect=[
        ((.7, 0., .1), {}), ((.7, 0., 1.0), {}),
    ])

    def stale_rotation(_residual, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    node._rotate_in_place = Mock(side_effect=stale_rotation)
    node.emit = Mock()

    def reach_staging(_self, _stage, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    monkeypatch.setattr(ServiceRoute, '_reach_staging', reach_staging)
    stage = {'id': 'home_dock_approach', 'x': .7, 'y': 0., 'yaw': .3}

    assert node._reach_staging(stage) is None
    call = node._rotate_in_place.call_args
    assert call.args == (pytest.approx(.2),)
    assert call.kwargs['pause_on_failure']('rotation scan unavailable')
    assert node._last_direct_motion_failure == (
        'rear staging residual rotation exceeded bounded travel')


def test_initial_rear_staging_scan_gap_pauses_deadline_during_rotation_settle(monkeypatch):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 10.01
    node._input_recovery_paused = False
    node.home_contract = {'yaw_tolerance_rad': math.radians(3.)}
    node.parking_contract = {
        'robot_base_frame': 'base_link',
        'stopped_linear_mps': .01,
        'stopped_angular_radps': .01,
    }
    rotation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    node.parking_tf = SimpleNamespace(lookup_transform=Mock(return_value=SimpleNamespace(
        transform=SimpleNamespace(rotation=rotation))))
    node.latest_motion = None
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._escape_publisher = Mock(return_value=Mock())
    node.request_stop = Mock()
    node.emit = Mock()
    node.execute = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((.7, 0., .1), {}))
    node._rear_wait_for_input = Mock(return_value=False)

    def spin(*_args, **_kwargs):
        clock[0] += .02
        node.latest_motion = (clock[0], 0., 0.)
        node._deadline_tick()

    monkeypatch.setattr(restaurant_module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(restaurant_module, 'spin_node', spin)
    monkeypatch.setattr(
        restaurant_module, 'rotation_collision_watch',
        lambda _node: lambda _command: 'rotation scan stale or future-dated')

    stage = {'id': 'home_dock_approach', 'x': .7, 'y': 0., 'yaw': .3}
    assert node._reach_staging(stage) is None
    node.request_stop.assert_not_called()
    assert node.run_deadline_s > 10.01
    assert node._input_recovery_paused is False
    assert node._last_direct_motion_failure == 'rotation scan stale or future-dated'


def test_rear_face_rotation_waits_for_fresh_scan_then_reobserves_same_target():
    node = route_node()
    initial = face(.2, math.radians(6.4), stamp=1.)
    aligned = face(.2, math.radians(1.), stamp=2.)
    aligned.update(
        face_odom=initial['face_odom'], midpoint_odom=initial['midpoint_odom'],
        normal_odom=initial['normal_odom'],
        base_pose_odom=(0., 0., math.radians(5.4)))
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)

    def stale_rotation(_correction, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    node._rotate_in_place = Mock(side_effect=stale_rotation)
    node._rear_wait_for_input = Mock(return_value=True)
    node._observe_rear_box_recovering = Mock(return_value=aligned)
    node.emit = Mock()

    assert module.RearBoxParkingMixin._align_rear_face_before_clearance(
        node, initial, GEOMETRY, .275, .225) is aligned
    predicate = node._rotate_in_place.call_args.kwargs['pause_on_failure']
    assert predicate('rotation scan stale or future-dated')
    assert not predicate('rotation swept footprint occupied')
    node._rear_wait_for_input.assert_called_once_with(
        'rotation scan stale or future-dated', preserve_motion_deadline=True)
    node._observe_rear_box_recovering.assert_called_once_with(
        GEOMETRY, .275, .225, allow_range_reapproach=True)


def test_rear_face_rotation_recovers_repeated_scan_gaps_against_same_target():
    node = route_node()
    initial = face(.2, math.radians(6.4), stamp=1.)
    observations = [
        face(.2, math.radians(5.), stamp=2.),
        face(.2, math.radians(4.), stamp=3.),
        face(.2, math.radians(1.), stamp=4.),
    ]
    for reobserved in observations:
        reobserved.update(
            face_odom=initial['face_odom'], midpoint_odom=initial['midpoint_odom'],
            normal_odom=initial['normal_odom'],
            base_pose_odom=(
                0., 0., initial['normal_yaw_error_rad']
                - reobserved['normal_yaw_error_rad']))
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)

    def stale_rotation(_correction, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    node._rotate_in_place = Mock(side_effect=stale_rotation)
    node._rear_wait_for_input = Mock(return_value=True)
    node._observe_rear_box_recovering = Mock(side_effect=observations)
    node.emit = Mock()

    assert module.RearBoxParkingMixin._align_rear_face_before_clearance(
        node, initial, GEOMETRY, .275, .225) is observations[-1]
    assert node._rear_wait_for_input.call_count == 3
    assert all(item.kwargs == {'preserve_motion_deadline': True}
               for item in node._rear_wait_for_input.call_args_list)
    assert node._observe_rear_box_recovering.call_count == 3
    assert node._rotate_in_place.call_count == 3


@pytest.mark.parametrize('reason, options', [
    ('rotation scan stale or future-dated', {'preserve_motion_deadline': True}),
    ('rear box scan unavailable', {}),
    ('rear box scan stale or future-dated', {}),
])
def test_rotation_scan_wait_preserves_motion_deadline(monkeypatch, reason, options):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 10.01
    node._input_recovery_paused = False
    node.samples = {'scan': 9.0}
    node.last_scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rear_box_scan = Mock(side_effect=[
        RuntimeError('rear box scan stale or future-dated'), (object(), 2.0)])
    publisher = Mock()
    node._escape_publisher = Mock(return_value=publisher)
    node.request_stop = Mock()
    node.emit = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    def spin(*_args, **_kwargs):
        assert node._input_recovery_paused is True
        clock[0] += .05
        node.samples['scan'] = clock[0]
        node._deadline_tick()

    monkeypatch.setattr(module, 'spin_node', spin)

    assert node._rear_wait_for_input(
        reason, **options)
    assert node.run_deadline_s == pytest.approx(10.11)
    assert node._input_recovery_paused is False
    node.request_stop.assert_not_called()
    assert publisher.publish.call_count == 2
    assert all(call.args[0].linear.x == 0.0 and call.args[0].angular.z == 0.0
               for call in publisher.publish.call_args_list)
    recovered = node.emit.call_args
    assert recovered.args == ('rear_box_input_recovered',)
    assert recovered.kwargs['paused_s'] == pytest.approx(.1)
    assert recovered.kwargs['motion_deadline_preserved'] is True


def test_rotation_scan_wait_checks_operator_stop_before_freshness(monkeypatch):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node.samples = {}
    node._input_recovery_paused = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rear_box_scan = Mock()
    node._escape_publisher = Mock(return_value=Mock())
    node.emit = Mock()

    def stop_during_wait(*_args, **_kwargs):
        node.stop_requested = True

    monkeypatch.setattr(module, 'spin_node', stop_during_wait)
    assert not node._rear_wait_for_input(
        'rotation scan stale or future-dated', preserve_motion_deadline=True)
    assert node._last_direct_motion_failure == 'operator_stop'
    assert node._input_recovery_paused is False
    node._rear_box_scan.assert_not_called()


def test_rotation_scan_wait_requires_a_new_received_scan_and_stamp(monkeypatch):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 11.0
    node.samples = {'scan': 9.0}
    node.last_scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rear_box_scan = Mock(side_effect=[
        (object(), 2.0), (object(), 1.0), (object(), 2.0)])
    node._escape_publisher = Mock(return_value=Mock())
    node.emit = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    def spin(*_args, **_kwargs):
        clock[0] += .05
        if clock[0] > 10.05:
            node.samples['scan'] = clock[0]

    monkeypatch.setattr(module, 'spin_node', spin)

    assert node._rear_wait_for_input(
        'rotation scan stale or future-dated', preserve_motion_deadline=True)
    assert node._rear_box_scan.call_count == 3


def test_rotation_scan_wait_stops_on_nonrecoverable_guard_failure(monkeypatch):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 11.0
    node.samples = {}
    node._input_recovery_paused = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value='battery voltage below running minimum')
    node._rear_box_scan = Mock()
    node._escape_publisher = Mock(return_value=Mock())
    node.emit = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(
        module, 'spin_node', lambda *_a, **_k: clock.__setitem__(0, clock[0] + .05))

    assert not node._rear_wait_for_input(
        'rotation scan stale or future-dated', preserve_motion_deadline=True)
    assert node._last_direct_motion_failure == 'battery voltage below running minimum'
    assert node._input_recovery_paused is False
    node._rear_box_scan.assert_not_called()


def test_rotation_scan_wait_preserves_nested_pause_and_none_deadline(monkeypatch):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = None
    node.samples = {'scan': 9.0}
    node.last_scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    node._input_recovery_paused = True
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node._rear_box_scan = Mock(return_value=(object(), 2.0))
    node._escape_publisher = Mock(return_value=Mock())
    node.emit = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    def spin(*_args, **_kwargs):
        clock[0] += .05
        node.samples['scan'] = clock[0]

    monkeypatch.setattr(module, 'spin_node', spin)

    assert node._rear_wait_for_input(
        'rotation scan unavailable', preserve_motion_deadline=True)
    assert node.run_deadline_s is None
    assert node._input_recovery_paused is True


def test_rotation_scan_wait_does_not_resume_after_operator_link_loss(monkeypatch):
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node.samples = {}
    node._input_recovery_paused = False
    node._operator_link_ready = Mock(return_value=False)
    node._guard_failure = Mock()
    node._rear_box_scan = Mock()
    node._escape_publisher = Mock(return_value=Mock())
    node.emit = Mock()
    monkeypatch.setattr(module, 'spin_node', lambda *_a, **_k: None)

    assert not node._rear_wait_for_input(
        'rotation scan unavailable', preserve_motion_deadline=True)
    assert node._last_direct_motion_failure == 'operator_link_lost'
    assert node._input_recovery_paused is False
    node._guard_failure.assert_not_called()
    node._rear_box_scan.assert_not_called()


@pytest.mark.parametrize('abort', ['operator_stop', 'operator_link_lost'])
def test_repeated_rotation_scan_wait_preserves_abort_priority(abort):
    node = route_node()
    initial = face(.2, math.radians(6.4), stamp=1.)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)

    def stale_rotation(_correction, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    def stopped_wait(*_args, **_kwargs):
        if abort == 'operator_stop':
            node.stop_requested = True
        else:
            node._operator_link_ready.return_value = False
        return False

    node._rotate_in_place = Mock(side_effect=stale_rotation)
    node._rear_wait_for_input = Mock(side_effect=stopped_wait)
    node.emit = Mock()
    with pytest.raises(RuntimeError, match=abort):
        module.RearBoxParkingMixin._align_rear_face_before_clearance(
            node, initial, GEOMETRY, .275, .225)


def test_rear_face_rotation_reports_nonrecoverable_failure_found_while_waiting():
    node = route_node()
    initial = face(.2, math.radians(6.4), stamp=1.)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)

    def stale_rotation(_correction, **_kwargs):
        node._last_direct_motion_failure = 'rotation scan stale or future-dated'
        return None

    def failed_wait(*_args, **_kwargs):
        node._last_direct_motion_failure = 'battery voltage below running minimum'
        return False

    node._rotate_in_place = Mock(side_effect=stale_rotation)
    node._rear_wait_for_input = Mock(side_effect=failed_wait)
    node.emit = Mock()

    with pytest.raises(RuntimeError, match='battery voltage below running minimum'):
        module.RearBoxParkingMixin._align_rear_face_before_clearance(
            node, initial, GEOMETRY, .275, .225)


def test_rear_face_rotation_does_not_retry_a_collision_stop():
    node = route_node()
    initial = face(.2, math.radians(6.4), stamp=1.)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)

    def collision_stop(_correction, **_kwargs):
        node._last_direct_motion_failure = 'rotation swept footprint occupied'
        return None

    node._rotate_in_place = Mock(side_effect=collision_stop)
    node._rear_wait_for_input = Mock(return_value=True)
    node._observe_rear_box_recovering = Mock()
    node.emit = Mock()

    with pytest.raises(RuntimeError, match='rotation swept footprint occupied'):
        module.RearBoxParkingMixin._align_rear_face_before_clearance(
            node, initial, GEOMETRY, .275, .225)
    node._rear_wait_for_input.assert_not_called()
    node._observe_rear_box_recovering.assert_not_called()


@pytest.mark.parametrize('invalid', [math.nan, 3.])
def test_invalid_or_excessive_center_curve_does_not_request_forward(invalid):
    observed = face(.85)
    observed['midpoint_odom'] = (invalid, .23)
    with pytest.raises(ValueError):
        module._rear_center_curve_available(observed, .275)


def test_centering_uses_fixed_initial_midpoint_when_visible_endpoints_change(monkeypatch):
    node = route_node()
    initial, centered = face(.77), face(.2)
    initial.update(face_odom=(-1.045, 0.), midpoint_odom=(-1.045, .1))
    centered.update(face_odom=(-1.045, 0.), midpoint_odom=(-1.045, .14))
    order = []
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._align_rear_face_before_clearance = Mock(
        side_effect=lambda *_a: order.append('align') or centered)
    node._clear_rear_dock_straight = Mock(
        side_effect=AssertionError('fixed target must not request clearance'))
    node._odom_pose = Mock(return_value=(0., .1, 0.))
    node._park_rear_face = Mock(side_effect=lambda *_a: order.append('fine') or True)
    node.emit = Mock()
    monkeypatch.setattr(module, '_rear_center_curve_available', lambda *_a: True)
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    assert order == ['align', 'fine']
    node._clear_rear_dock_straight.assert_not_called()
    assert node._park_rear_face.call_args.args[0] is centered
    ready = next(call for call in node.emit.call_args_list
                 if call.args == ('rear_dock_straight_entry_ready',))
    assert ready.kwargs['lateral_error_m'] == pytest.approx(0.)
    assert abs(.1 - centered['midpoint_odom'][1]) > module.REAR_CENTER_TOLERANCE_M


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
    node._observe_rear_box_recovering = Mock(return_value=initial)
    node._observe_rear_box = Mock(return_value=centered)
    node._align_rear_face_before_clearance = Mock(side_effect=lambda found, *_a: found)
    node._clear_rear_dock_straight = Mock(
        side_effect=AssertionError('residual cannot request forward clearance'))
    node._odom_pose = Mock(return_value=(0., .057698, 0.))
    def center(*_a, **_k):
        node._rear_center_moved = True
        if interruption == 'stop':
            node.stop_requested = True
        elif interruption == 'deadline':
            node.run_deadline_s = time.monotonic() - 1.
        return True
    def align(found, *_a):
        center()
        return found
    node._align_rear_face_before_clearance = Mock(side_effect=align)
    node._center_rear_box = Mock()
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    monkeypatch.setattr(module, '_rear_center_curve_available', lambda *_a: True)
    assert node.park_rear_box(
        GEOMETRY, 'return-route', execute=True, timeout_s=90, target_gap_m=.01) is (interruption is None)
    node._center_rear_box.assert_not_called()
    node._clear_rear_dock_straight.assert_not_called()
    if interruption is None:
        node._park_rear_face.assert_called_once()
    else:
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
            module.RearBoxParkingMixin._align_rear_face_before_clearance(
                node, initial, GEOMETRY, .275, .225)
        node._rotate_in_place.assert_not_called()
    else:
        assert module.RearBoxParkingMixin._align_rear_face_before_clearance(
            node, initial, GEOMETRY, .275, .225) is aligned
        call = node._rotate_in_place.call_args
        assert call.args == (math.radians(35.),)
        assert call.kwargs['pause_on_failure']('rotation scan unavailable')
        node._observe_rear_box_recovering.assert_called_once_with(
            GEOMETRY, .275, .225, allow_range_reapproach=True)
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
    assert module.RearBoxParkingMixin._align_rear_face_before_clearance(
        node, initial, GEOMETRY, .275, .225) is aligned
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


def _historical_face_scan(x_m, lower_m=-.175, upper_m=.175):
    count = 7201
    increment = 2*math.pi/(count-1)
    ranges = [math.inf] * count
    for index in range(101):
        y_m = lower_m + (upper_m-lower_m)*index/100
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


def test_source_face_classifier_accepts_only_overlapping_partial_target_segment():
    classifier = _source_face_classifier()
    partial_target = _historical_face_scan(-.475, -.09, .095)
    assert classifier((0., 0., 0.), partial_target, (-.475, -.03))

    # A different coplanar segment must remain collision memory even though its
    # normal and plane match the target.
    separate_segment = _historical_face_scan(-.475, .24, .43)
    assert not classifier((0., 0., 0.), separate_segment, (-.475, .30))


@pytest.mark.parametrize('x_m', [-.465, -.455, -.445])
def test_source_face_classifier_preserves_nearer_parallel_partial_surface(x_m):
    classifier = _source_face_classifier()
    nearer_surface = _historical_face_scan(x_m, -.09, .095)
    assert not classifier((0., 0., 0.), nearer_surface, (x_m, -.03))


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
    assert node._drive_zero_turn.call_count == 2
    node._clear_rear_dock_straight.assert_not_called()
    assert node._observe_rear_box_recovering.call_count == 2
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
        [.454145928792223, .254145928792223, .054145928792223])
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
    node._rear_wait_for_input = Mock(return_value=False)
    node.emit = Mock()
    with pytest.raises(RuntimeError, match=reason):
        node._observe_rear_box_recovering(
            GEOMETRY, .275, .225, allow_range_reapproach=True)
    node._rear_coarse_step.assert_called_once()


def test_stale_scan_during_range_reapproach_waits_then_resumes_same_target():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    observed = face(.2)
    node._observe_rear_box = Mock(side_effect=[RuntimeError(
        'rear box observation unavailable: rear box face unavailable'), observed])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .65})

    def stale_once(*_args, **_kwargs):
        node._last_direct_motion_failure = 'rear box scan stale or future-dated'
        return False

    node._rear_coarse_step = Mock(side_effect=stale_once)
    node._rear_wait_for_input = Mock(return_value=True)
    node.emit = Mock()

    assert node._observe_rear_box_recovering(
        GEOMETRY, .275, .225, allow_range_reapproach=True) is observed
    node._rear_wait_for_input.assert_called_once_with(
        'rear box scan stale or future-dated', None)
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


def test_normal_nav2_departure_captures_one_fresh_reference_without_reteaching(
        monkeypatch):
    node = route_node()
    node.config = {}
    node.waypoints = [{'id': 'home_exit'}]
    node._resume_waypoint_index = 0
    node.home_rear_geometry = GEOMETRY
    node.registry = {'home': {'rear_parking_reference': 'lidar_box'}}
    observed = face(.45)
    observed.update(
        support_count=12, span_m=.35, observation_roi_clipped=False,
        base_pose_odom=(1.1, 2.2, .3))
    events = []
    node._rear_box_observation = Mock(
        side_effect=lambda *_a: events.append('observe') or observed)
    node.capture_stationary_pose = Mock(
        side_effect=lambda **_k: events.append('capture') or ((10., 20., .5), None))
    node._odom_pose = Mock(return_value=(1., 2., .1))
    node.emit = Mock()

    def parent_execute(_self, **kwargs):
        events.append('nav2')
        return kwargs

    monkeypatch.setattr(ServiceRoute, '_execute_service_once', parent_execute)
    arguments = dict(final_parking=False, alignment=False, staging=False, face=False)

    assert node._execute_service_once(**arguments) == arguments
    reference = node._rear_dock_reference
    assert events == ['observe', 'capture', 'nav2']
    assert reference['observation'] is observed
    assert reference['observation_phase'] == 'before_nav2_departure'
    assert reference['start_pose_map'] == [10., 20., .5]
    assert reference['approach_distance_m'] == pytest.approx(.5)

    assert node._execute_service_once(**arguments) == arguments
    assert events == ['observe', 'capture', 'nav2', 'nav2']
    assert node._rear_dock_reference is reference
    node._rear_box_observation.assert_called_once()
    node.capture_stationary_pose.assert_called_once_with(intermediate=True)


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
        ['coarse', 'observe', 'align'] if alignment_ok else
        ['coarse', 'observe', 'align'])
    node._align_rear_reference.assert_called_once_with(
        {'id': 'home_dock_approach'}, node.registry['home'],
        observed=initial, return_to_stage=False)
    if alignment_ok:
        assert node._park_rear_face.call_args.args[0] is initial
        node._center_rear_box.assert_not_called()
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


def test_corner_confirmation_accepts_one_lidar_bin_but_not_more():
    error = 0.008083628030596349
    assert module._rear_gap_confirmed(
        0.009541270916847748, error, .225, .01)
    assert not module._rear_gap_confirmed(.0095, 0.0085, .225, .01)


def test_straight_entry_heading_failure_does_not_start_fine(monkeypatch):
    node = route_node()
    initial, skewed = face(.77), face(.2, math.radians(3.1))
    skewed.update(face_odom=initial['face_odom'], midpoint_odom=initial['midpoint_odom'])
    for name in ('verify_live_maps','_confirm_collision_monitor_on'):
        setattr(node,name,Mock())
    for name in ('wait_until_ready','_reverse_smoother_ready','_parking_parameters_ready'):
        setattr(node,name,Mock(return_value=True))
    node._observe_rear_box_recovering = Mock(return_value=initial)
    skewed.update(normal_odom=initial['normal_odom'])
    node._align_rear_face_before_clearance = Mock(return_value=skewed)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    assert not node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    node._align_rear_face_before_clearance.assert_called_once()
    node._center_rear_box.assert_not_called()
    node._park_rear_face.assert_not_called()
    assert any(call.kwargs.get('reason') ==
               'rear box final entry heading exceeds tolerance'
               for call in node.emit.call_args_list)


def test_small_lateral_offset_at_57cm_skips_curve():
    node = route_node()
    observed = face(.57)
    observed.update(observation_roi_clipped=False, midpoint_odom=(-.845,0.), normal_odom=(1.,0.))
    node._odom_pose = Mock(return_value=(0.,.003,0.))
    node.emit = Mock()
    node._make_reverse_path = Mock(side_effect=AssertionError('unnecessary curve'))
    assert module.RearBoxParkingMixin._center_rear_box(node,observed,.275)
    assert node._rear_center_skipped and not node._rear_center_moved


@pytest.mark.parametrize('lateral,accepted', [
    (-.02, True), (.02, True), (-.020001, True), (.020001, True),
    (-.0321746744, True), (.0321746744, True), (-.23, False), (.23, False),
])
def test_straight_docking_reports_precision_miss_but_requires_face_overlap(
        monkeypatch, lateral, accepted):
    node = route_node()
    observed = face(.57)
    observed['midpoint_odom'] = (-.845, -lateral)
    for name in ('verify_live_maps', '_confirm_collision_monitor_on'):
        setattr(node, name, Mock())
    for name in ('wait_until_ready', '_reverse_smoother_ready',
                 '_parking_parameters_ready'):
        setattr(node, name, Mock(return_value=True))
    node._observe_rear_box_recovering = Mock(return_value=observed)
    node._clear_rear_dock_straight = Mock(
        side_effect=AssertionError('lateral rejection cannot request forward clearance'))
    node._park_rear_face = Mock(return_value=True)
    node.emit = Mock()
    monkeypatch.setattr(module, '_rear_center_curve_available', lambda *_a: False)

    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90,
        target_gap_m=.01) is accepted
    if accepted:
        node._center_rear_box.assert_not_called()
        node._park_rear_face.assert_called_once()
    else:
        node._center_rear_box.assert_not_called()
        node._park_rear_face.assert_not_called()
    node._clear_rear_dock_straight.assert_not_called()
    if abs(lateral) > .02:
        assert any(call.args == ('rear_dock_lateral_precision_unmet',)
                   for call in node.emit.call_args_list)


@pytest.mark.parametrize('lateral', [-.058358623, .058358623])
def test_five_cm_lateral_offset_never_starts_curve_or_forward(lateral):
    observed = face(.492422496)
    observed.update(midpoint_odom=(-.767422496, -lateral),
                    base_pose_odom=(0., 0., 0.), observation_roi_clipped=False)
    node = route_node()
    for name in ('verify_live_maps','_confirm_collision_monitor_on'):
        setattr(node,name,Mock())
    for name in ('wait_until_ready','_reverse_smoother_ready','_parking_parameters_ready'):
        setattr(node,name,Mock(return_value=True))
    node.emit = Mock()
    node._observe_rear_box_recovering = Mock(return_value=observed)
    node._clear_rear_dock_straight = Mock(side_effect=AssertionError('unnecessary forward'))
    node._park_rear_face = Mock(return_value=True)
    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    node._clear_rear_dock_straight.assert_not_called()
    node._center_rear_box.assert_not_called()
    node._park_rear_face.assert_called_once()


def test_far_corner_only_miss_gets_reverse_trim():
    node = route_node()
    initial = face(.08, 0., stamp=1.)
    missed = face(.012273624984817724, .0023, stamp=2.)
    final = face(.01, 0., stamp=3.)
    missed['face_odom'] = final['face_odom'] = initial['face_odom']
    node._observe_rear_box_recovering = Mock(side_effect=[missed, final])
    node._rear_box_update = Mock(side_effect=[
        Mock(return_value={'remaining_m': .07}),
        Mock(return_value={'remaining_m': .002672394})])
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=-.00267)
    node.emit = Mock()
    assert node._park_rear_face(initial, GEOMETRY, 'route', .275, .225, .01, None)
    assert node._drive_zero_turn.call_count == 2
    assert all(call.args[0] < 0. for call in node._drive_zero_turn.call_args_list)
    assert node.confirmation['confirmed'] is True


@pytest.mark.parametrize('mode', ['clear', 'occupied', 'missing'])
def test_dock_static_disagreement_requires_live_swept_clearance(monkeypatch, mode):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    node = route_node()
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    path = Path()
    for x in (0., -.1, -.2):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    node._make_reverse_path = Mock(return_value=path)
    node._odom_path = Mock(return_value=path)
    node._execute_reverse_once = Mock(return_value=True)
    outline = [(-.28, -.23), (.07, -.23), (.07, .23), (-.28, .23)]
    node._footprint_outline = Mock(return_value=outline)
    node.registry = {'map': {'yaml_path': '/tmp/map'}, 'keepout': {'yaml_path': '/tmp/mask'}}
    node.emit = Mock()
    scan = SimpleNamespace(ranges=[.4 if mode == 'occupied' else 2.],
                           angle_min=math.pi, angle_increment=.01,
                           range_min=.28, range_max=12.)
    node._scan_in_base = Mock(return_value=(
        None if mode == 'missing' else (scan, (0., 0., 0.), outline, [])))
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=False)))
    monkeypatch.setattr(module, 'static_corridor_clear', Mock(side_effect=[False, True]))
    if mode == 'clear':
        assert module.RearBoxParkingMixin._center_rear_box(node, face(.65), .275)
        node._execute_reverse_once.assert_called_once()
    else:
        with pytest.raises(RuntimeError, match='live (scan unavailable|corridor occupied)'):
            module.RearBoxParkingMixin._center_rear_box(node, face(.65), .275)
        node._execute_reverse_once.assert_not_called()


def test_stationary_rear_observation_keeps_window_after_scan_gap(monkeypatch):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 10.2
    node._operator_link_ready = Mock(return_value=True)
    node.latest_motion = (10.0, 0.0, 0.0)
    observations = [face(.2, stamp=20.0 + index * .01) for index in range(8)]
    node._rear_box_observation = Mock(side_effect=[
        RuntimeError('rear box scan unavailable'), *observations])
    def spin(*_a, **_k):
        clock[0] += .01
        node.latest_motion = (clock[0], 0.0, 0.0)
    def wait(reason):
        assert reason == 'rear box scan unavailable'
        clock[0] += 2.0
        node.run_deadline_s += 2.0
        return True
    node._rear_wait_for_input = Mock(side_effect=wait)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module, 'spin_node', spin)
    monkeypatch.setattr(module, '_stationary_face', lambda rows, *_a: rows[-1])
    result = node._observe_rear_box(GEOMETRY, .275, .225, timeout_s=.2)
    assert result is observations[-1]
    node._rear_wait_for_input.assert_called_once_with('rear box scan unavailable')
    assert clock[0] > 12.0
    assert not node.stop_requested


@pytest.mark.parametrize('previous_pause', [False, True])
def test_stationary_scan_gap_at_deadline_does_not_become_operator_stop(
        monkeypatch, previous_pause):
    node = route_node()
    clock = [10.0]
    node.run_deadline_s = 10.015
    node._input_recovery_paused = previous_pause
    node._operator_link_ready = Mock(return_value=True)
    node.latest_motion = (10.0, 0.0, 0.0)
    node.request_stop = Mock()
    observed = face(.2, stamp=20.0)
    node._rear_box_observation = Mock(side_effect=[
        RuntimeError('rear box scan unavailable'), observed])
    def spin(*_a, **_k):
        clock[0] += .02
        node.latest_motion = (clock[0], 0.0, 0.0)
        node._deadline_tick()
    def wait(_reason):
        clock[0] += 2.0
        node.run_deadline_s += 2.0
        return True
    node._rear_wait_for_input = Mock(side_effect=wait)
    tracker = Mock()
    tracker.update.return_value = True
    monkeypatch.setattr(module, 'RearBoxFaceTracker', Mock(return_value=tracker))
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module, 'spin_node', spin)
    monkeypatch.setattr(module, '_stationary_face', lambda rows, *_a: rows[-1])
    assert node._observe_rear_box(GEOMETRY, .275, .225) is observed
    node.request_stop.assert_not_called()
    assert node._input_recovery_paused is previous_pause
    assert node.run_deadline_s == pytest.approx(12.035)
