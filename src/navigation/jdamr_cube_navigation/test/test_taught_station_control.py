"""Regression tests for taught-station control integration without robot motion."""

import json
import math
import threading
import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import box_service
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.restaurant_service import ServiceRoute
import pytest


def _pose(x=0., y=0., yaw=0.):
    rotation = SimpleNamespace(x=0., y=0., z=math.sin(yaw / 2.),
                               w=math.cos(yaw / 2.))
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y), rotation=rotation)


def _clock(stamp_s=10.):
    return SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(stamp_s * 1e9)))


def _node():
    node = object.__new__(BoxServiceRoute)
    node.registry = {
        'map': {'yaml_sha256': 'a' * 64, 'image_sha256': 'b' * 64},
        'keepout': {'yaml_sha256': 'c' * 64, 'image_sha256': 'd' * 64},
        'tables': [{
            'table_id': 'water_station',
            'service_poses': [{
                'priority': 1, 'x_m': 1.2, 'y_m': -.4, 'yaw_rad': .1,
            }],
        }],
    }
    node.parking_contract = {'robot_base_frame': 'base_link',
                             'yaw_tolerance_rad': math.radians(3.)}
    node.box_geometry = {'front_to_wheel_axis': {'value': .065}}
    node.station_reference = None
    # Isolate worker state-machine cases from the separately tested depth join.
    node._station_tape_surface = Mock(return_value={})
    node.station_reference_profile = {'lateral_tolerance_m': .01}
    node.station_reference_rgb_info = None
    node.station_reference_result = None
    node.station_reference_result_received_s = None
    node.station_reference_image_subscription = None
    node.station_reference_info_subscription = None
    node._station_reference_pending = None
    node._station_reference_lock = threading.Lock()
    node._station_reference_wake = threading.Event()
    node._station_reference_stop = threading.Event()
    node._station_reference_thread = None
    node._station_reference_perception_active = True
    node._observer_perception_enabled = False
    node.stop_requested = False
    node.parking_motion_revision = 0
    node.run_deadline_s = None
    node.emit = Mock()
    node.create_subscription = Mock(side_effect=['info-sub', 'image-sub'])
    node.get_clock = Mock(return_value=_clock())
    return node


def _rgbd_model(**kwargs):
    # Status and worker arithmetic need the bound capture and run offsets.
    return Mock(goal_left_offset_m=0., run_left_offset_m=0., **kwargs)


def test_live_feedback_corrects_the_first_anchor_and_arrival_uses_the_same_goal():
    node = _node()
    node.station_reference = _rgbd_model()
    node.station_reference_mode = 'rgbd'
    reference = {'valid': True, 'stamp_s': 10., 'goal_odom': [.4, 0., 0.]}
    node._fresh_station_reference = Mock(return_value=reference)
    first, _ = node._station_goal_reference(freeze=True)
    assert first['goal_odom'] == [.4, 0., 0.]

    # Later observations agree on a 28 mm correction. The initial estimate must
    # not remain the steering goal merely because the anchor has been frozen.
    reference.update(stamp_s=10.2, goal_odom=[.4, .028, 0.])
    node._station_goal_reference()
    reference.update(stamp_s=10.4)
    control, _ = node._station_goal_reference()
    node._odom_pose = Mock(return_value=(.4, 0., 0.))
    status = node._station_lateral_status()

    assert node.station_reference_goal['goal_odom'] == [.4, 0., 0.]
    assert control['goal_odom'][1] == pytest.approx(.028)
    assert status['lateral_goal_odom'] == control['goal_odom']
    assert status['internal_lateral_error_m'] == pytest.approx(.028)
    assert status['internal_lateral_within_tolerance'] is False
    assert status['millimeter_accuracy_validated'] is False


def test_first_control_call_uses_observations_already_processed_in_background():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_mode = 'rgbd'
    observations = [
        {'valid': True, 'stamp_s': 10. + index * .2,
         'goal_odom': [.4, y, 0.]}
        for index, y in enumerate((.030, .031, .070))]
    node._station_reference_goal_samples = deque(observations, maxlen=5)
    node._fresh_station_reference = Mock(return_value=observations[-1])

    goal, _ = node._station_goal_reference(freeze=True)

    assert goal['goal_odom'][1] == pytest.approx(.031)
    assert goal['goal_filter_sample_count'] == 3


def test_lost_video_keeps_steering_goal_but_cannot_confirm_precision():
    node = _node()
    node.station_reference = _rgbd_model()
    node.station_reference_mode = 'rgbd'
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'stamp_s': 10., 'goal_odom': [.4, .02, 0.]})
    node._station_goal_reference(freeze=True)
    node._fresh_station_reference.return_value = {'valid': False, 'reason': 'station_reference_stale'}

    goal, _ = node._station_goal_reference()
    status = node._station_lateral_status()

    assert goal['goal_odom'] == [.4, .02, 0.]
    assert status['lateral_goal_valid'] is True
    assert status['lateral_reference_valid'] is False
    assert status['internal_lateral_within_tolerance'] is False


def _profiles(tmp_path):
    path = tmp_path / 'station_profiles.json'
    path.write_text(json.dumps({
        'schema_version': 1,
        'map': {'yaml_sha256': 'a' * 64, 'image_sha256': 'b' * 64},
        'keepout': {'yaml_sha256': 'c' * 64, 'image_sha256': 'd' * 64},
        'stations': {
            'water_station': {
                'target_priority': 'operator_taught_map_pose_and_heading',
                'lateral_tolerance_m': .005,
                'lateral_precision_status': 'NOT_VALIDATED',
                'rgbd_reference': {'immutable': 'reference'},
            },
        },
    }), encoding='utf-8')
    return path


def test_profile_loads_hash_bound_registered_pose_and_one_rgb_pair(
        tmp_path, monkeypatch):
    node = _node()
    model = Mock()
    monkeypatch.setattr(box_service, 'TaughtStationReference', Mock(return_value=model))

    node.configure_station_reference(_profiles(tmp_path), 'water_station')

    assert node.station_reference is model
    assert node.station_reference_taught_map_pose == pytest.approx((1.2, -.4, .1))
    assert node.create_subscription.call_count == 2
    topics = [call.args[1] for call in node.create_subscription.call_args_list]
    assert topics == ['/camera/color/camera_info', '/camera/color/image_raw']
    assert node._station_reference_perception_active is False
    assert node._station_reference_thread is None
    node._start_station_reference_perception()
    assert node._station_reference_thread.is_alive()
    node._stop_station_reference_perception()


def test_profile_rejects_different_map_before_subscribing(tmp_path, monkeypatch):
    node = _node()
    profile_path = _profiles(tmp_path)
    document = json.loads(profile_path.read_text())
    document['map']['image_sha256'] = '0' * 64
    profile_path.write_text(json.dumps(document), encoding='utf-8')
    constructor = Mock()
    monkeypatch.setattr(box_service, 'TaughtStationReference', constructor)

    with pytest.raises(ValueError, match='map image_sha256 mismatch'):
        node.configure_station_reference(profile_path, 'water_station')

    constructor.assert_not_called()
    node.create_subscription.assert_not_called()


def test_switch_from_water_rgbd_to_featureless_table_clears_the_old_reference(
        tmp_path, monkeypatch):
    node = _node()
    node.registry['tables'].append({
        'table_id': 'table_01', 'service_poses': [
            {'priority': 1, 'x_m': 2., 'y_m': .3, 'yaw_rad': .2}],
    })
    profile_path = _profiles(tmp_path)
    document = json.loads(profile_path.read_text())
    document['stations']['water_station']['reference_mode'] = 'rgbd_auxiliary'
    document['stations']['table_01'] = {
        'target_priority': 'operator_taught_map_pose_and_heading',
        'lateral_tolerance_m': .005,
        'lateral_precision_status': 'NOT_VALIDATED',
        'reference_mode': 'map_pose_only', 'rgbd_reference': None,
    }
    profile_path.write_text(json.dumps(document), encoding='utf-8')
    constructor = Mock(return_value=Mock())
    monkeypatch.setattr(box_service, 'TaughtStationReference', constructor)
    node.configure_station_reference(profile_path, 'water_station')
    node._start_station_reference_perception()
    worker = node._station_reference_thread
    node.station_reference_goal = {'valid': True, 'goal_odom': [8., 8., 0.]}
    node.station_reference_goal_frozen = True

    node.configure_station_reference(profile_path, 'table_01')

    assert not worker.is_alive()
    assert node.station_reference is None
    assert node.station_reference_mode == 'map_pose_only'
    assert node.station_reference_taught_map_pose == pytest.approx((2., .3, .2))
    assert node.station_reference_goal is None
    assert node.station_reference_goal_frozen is False
    constructor.assert_called_once_with({'immutable': 'reference'})
    assert node.create_subscription.call_count == 2


def test_map_pose_reference_uses_current_transform_and_does_not_claim_rgbd_precision():
    node = _node()
    node.station_reference_mode = 'map_pose_only'
    node.station_reference_taught_map_pose = (1.2, -.4, .1)
    node.amcl_freshness_s = 15.
    node._navigation_ready = Mock(return_value=True)
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        transform=_pose(2., 3., math.pi / 2))
    node._odom_pose = Mock(return_value=(2.4, 4.2, math.pi / 2 + .1))

    reference = node._fresh_station_reference()
    status = node._station_lateral_status()
    anchored = node._station_anchor_target({
        'x_m': .4, 'y_m': .2, 'face_center_map_xy_m': [.5, .2],
        'outward_normal_map_xy': [-1., 0.],
    })

    assert reference['goal_odom'] == pytest.approx((2.4, 4.2, math.pi / 2 + .1))
    assert status['lateral_goal_basis'] == 'registered_map_pose_in_odom'
    assert status['internal_lateral_within_tolerance'] is True
    assert status['millimeter_accuracy_validated'] is False
    assert anchored['rgbd_reference_valid'] is False
    assert anchored['lateral_reference_mode'] == 'map_pose_only'


def test_map_pose_reference_rejects_stale_localization_and_transform():
    node = _node()
    node.station_reference_mode = 'map_pose_only'
    node.station_reference_taught_map_pose = (1., 2., 0.)
    node.amcl_freshness_s = 1.
    node._navigation_ready = Mock(return_value=False)
    node.parking_tf = Mock()

    assert node._fresh_station_reference()['reason'] == 'map_pose_localization_unavailable'
    node.parking_tf.lookup_transform.assert_not_called()

    node._navigation_ready.return_value = True
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
        transform=_pose())
    assert node._fresh_station_reference()['reason'] == 'map_pose_transform_invalid'

    node.parking_tf.lookup_transform.side_effect = box_service.TransformException('no transform')
    failed = node._fresh_station_reference()
    assert failed['reason'] == 'map_pose_tf_lookup_failed'
    assert failed['detail'] == 'no transform'


def test_invalid_rgbd_reference_can_recover_while_stopped(monkeypatch):
    node = _node()
    node.station_reference_mode = 'rgbd_auxiliary'
    node.station_reference = Mock()
    invalid = {'lateral_reference_valid': False}
    recovered = {'lateral_reference_valid': True,
                 'internal_lateral_within_tolerance': True}
    node._station_lateral_status = Mock(return_value=recovered)
    spin = Mock()
    monkeypatch.setattr(box_service, 'spin_node', spin)

    result, moved = node._retry_station_lateral_reference(invalid, 0)

    assert result == recovered
    assert moved is False
    spin.assert_called_once()


def test_lateral_reference_retry_never_hides_valid_error_or_motion(monkeypatch):
    node = _node()
    node.station_reference_mode = 'rgbd_auxiliary'
    node.station_reference = Mock()
    spin = Mock(side_effect=lambda *_args, **_kwargs: setattr(
        node, 'parking_motion_revision', 1))
    monkeypatch.setattr(box_service, 'spin_node', spin)
    misaligned = {'lateral_reference_valid': True,
                  'internal_lateral_within_tolerance': False}

    assert node._retry_station_lateral_reference(misaligned, 0) == (misaligned, False)
    spin.assert_not_called()
    invalid = {'lateral_reference_valid': False}
    assert node._retry_station_lateral_reference(invalid, 0) == (invalid, True)


def test_map_lateral_reference_can_recover_without_an_rgbd_model(monkeypatch):
    node = _node()
    node.station_reference_mode = 'map_pose_only'
    invalid = {'lateral_reference_valid': False}
    recovered = {'lateral_reference_valid': True,
                 'lateral_goal_basis': 'registered_map_pose_in_odom'}
    node._station_lateral_status = Mock(return_value=recovered)
    monkeypatch.setattr(box_service, 'spin_node', Mock())

    assert node._retry_station_lateral_reference(invalid, 0) == (recovered, False)


def test_worker_uses_image_stamp_tf_and_keeps_only_registered_pose_correction(monkeypatch):
    # A fresh Jazzy process need not expose duration on the rclpy package.
    monkeypatch.delattr(box_service.rclpy, 'duration', raising=False)
    node = _node()
    model = _rgbd_model(contact_reference=None)
    model.observe.return_value = {
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'goal_odom': [1., .25, .2], 'stamp_s': 10., 'goal_left_offset_m': 0.,
    }
    node.station_reference = model
    node._station_reference_perception_active = True
    node.parking_tf = Mock()

    def lookup(target, source, stamp, **kwargs):
        if (target, source) == ('odom', 'base_link'):
            assert stamp.nanoseconds == int(10e9)
            assert 0 < kwargs['timeout'].nanoseconds <= int(.2e9)
            return SimpleNamespace(transform=_pose(.8, .1, .05))
        assert (target, source) == ('map', 'odom')
        # Map only presents the image-time odom goal: latest TF, no wait.
        assert stamp.nanoseconds == 0 and 'timeout' not in kwargs
        return SimpleNamespace(transform=_pose(2., 3., math.pi / 2))

    node.parking_tf.lookup_transform.side_effect = lookup
    node.station_reference_rgb_info = SimpleNamespace(
        header=SimpleNamespace(frame_id='camera_color_optical_frame'))
    image = SimpleNamespace(
        encoding='rgb8', height=2, width=2, step=6,
        data=bytes(range(12)),
        header=SimpleNamespace(
            frame_id='camera_color_optical_frame',
            stamp=SimpleNamespace(sec=10, nanosec=0)))
    worker = threading.Thread(target=node._station_reference_loop)
    worker.start()
    node._station_image_callback(image)
    deadline = time.monotonic() + 1.
    while node.station_reference_result is None and time.monotonic() < deadline:
        time.sleep(.005)
    node._station_reference_stop.set()
    node._station_reference_wake.set()
    worker.join(timeout=1.)

    assert model.observe.call_count == 1
    observed_image, observed_info, observed_pose, stamp_s = model.observe.call_args.args
    assert observed_image.flags['C_CONTIGUOUS']
    assert observed_image[0, 0].tolist() == [2, 1, 0]
    assert observed_info is node.station_reference_rgb_info
    assert observed_pose == pytest.approx((.8, .1, .05))
    assert stamp_s == 10.
    assert node.station_reference_result['goal_map'] == pytest.approx(
        (1.75, 4., math.pi / 2 + .2))


def test_expired_match_never_seeds_control_but_can_seed_fresh_image_search():
    node = _node()
    contact = Mock()
    checkpoint = ({'cup': (12., 13.)}, .1)
    contact.tracking_checkpoint.return_value = checkpoint
    model = _rgbd_model(contact_reference=contact)
    node.station_reference = model
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())

    def observe(*args):
        node.get_clock.return_value = _clock(11.)
        return {'valid': True, 'goal_odom': [1., .25, .2],
                'stamp_s': 10., 'goal_left_offset_m': 0.}

    model.observe.side_effect = observe
    node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), 10.)
    # End after this iteration, without timing-dependent sleeps.
    node.emit.side_effect = lambda *args, **kwargs: node._station_reference_stop.set()
    node._station_reference_wake.set()
    node._station_reference_loop()

    assert node.station_reference_result['reason'] == 'processing_exceeded_image_age'
    contact.restore_tracking.assert_not_called()
    assert not getattr(node, '_station_reference_goal_samples', None)


def test_worker_preserves_tf_failure_detail_for_final_diagnostics():
    node = _node()
    node.station_reference = Mock()
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.side_effect = box_service.TransformException(
        'image timestamp is newer than available TF')
    node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), 10.)
    worker = threading.Thread(target=node._station_reference_loop)
    worker.start()
    node._station_reference_wake.set()
    deadline = time.monotonic() + 1.
    while node.station_reference_result is None and time.monotonic() < deadline:
        time.sleep(.005)
    node._station_reference_stop.set()
    node._station_reference_wake.set()
    worker.join(timeout=1.)

    assert node.station_reference_result['reason'] == 'station_reference_runtime:TransformException'
    assert node.station_reference_result['detail'] == 'image timestamp is newer than available TF'
    assert node._station_lateral_status()['lateral_reference_detail'] == (
        'image timestamp is newer than available TF')


def test_rgb_callback_is_nonblocking_and_replaces_pending_frame():
    node = _node()
    model = Mock()
    node.station_reference = model
    node._station_reference_perception_active = True
    node.station_reference_rgb_info = SimpleNamespace(
        header=SimpleNamespace(frame_id='camera_color_optical_frame'))

    def image(stamp):
        return SimpleNamespace(
            encoding='bgr8', height=1, width=1, step=3,
            data=bytes((stamp, 0, 0)),
            header=SimpleNamespace(
                frame_id='camera_color_optical_frame',
                stamp=SimpleNamespace(sec=stamp, nanosec=0)))

    node._station_image_callback(image(10))
    node._station_image_callback(image(11))

    model.observe.assert_not_called()
    pending_image, _pending_info, pending_stamp = node._station_reference_pending
    assert pending_stamp == 11.
    assert pending_image[0, 0].tolist() == [11, 0, 0]


def test_stationary_target_anchors_tangent_to_registered_pose_by_default():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_taught_map_pose = (1.2, -.4, .1)
    node._fresh_station_reference = Mock(return_value={
        'valid': False, 'reason': 'station_reference_not_observed'})
    target = {
        'x_m': .40, 'y_m': .20,
        'face_center_map_xy_m': [.50, .20],
        'outward_normal_map_xy': [-1., 0.],
        'estimate_gap_m': .01,
    }

    anchored = node._station_anchor_target(target)

    assert anchored['x_m'] == pytest.approx(.40)
    assert anchored['y_m'] == pytest.approx(-.40)
    assert anchored['face_center_map_xy_m'] == pytest.approx((.50, -.40))
    assert anchored['outward_normal_map_xy'] == [-1., 0.]
    assert anchored['estimate_gap_m'] == .01
    assert anchored['rgbd_reference_valid'] is False
    assert anchored['millimeter_accuracy_validated'] is False


def test_fresh_rgbd_goal_replaces_only_taught_tangent():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_taught_map_pose = (1.2, -.4, .1)
    node.station_reference_mode = 'rgbd'
    # Control filters fresh results by stamp in odom and carries their map goal.
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'stamp_s': 10., 'goal_odom': [.1, .08, .02],
        'goal_map': [1.1, .08, .02],
    })
    target = {
        'x_m': .40, 'y_m': .20,
        'face_center_map_xy_m': [.50, .20],
        'outward_normal_map_xy': [-1., 0.],
    }

    anchored = node._station_anchor_target(target)

    assert anchored['x_m'] == pytest.approx(.40)
    assert anchored['y_m'] == pytest.approx(.08)
    assert anchored['face_center_map_xy_m'] == pytest.approx((.50, .08))
    assert anchored['rgbd_reference_valid'] is True


def test_invalid_or_stale_reference_never_claims_lateral_precision():
    node = _node()
    node.station_reference = Mock()
    node._fresh_station_reference = Mock(return_value={
        'valid': False, 'reason': 'station_reference_stale'})

    status = node._station_lateral_status()

    assert status == {
        'lateral_reference_valid': False,
        'lateral_reference_reason': 'station_reference_stale',
        'lateral_tolerance_m': .005,
        'lateral_precision_status': 'NOT_VALIDATED',
        'millimeter_accuracy_validated': False,
        'internal_lateral_error_m': None,
        'internal_lateral_within_tolerance': False,
        'lateral_goal_valid': False,
        'lateral_goal_basis': None,
        'lateral_goal_observation_stamp_s': None,
        'lateral_goal_frozen': False,
        'lateral_goal_odom': None,
        'lateral_actual_odom': None,
        'lateral_anchor_odom': None,
        'rgbd_observed_lateral_error_m': None,
        'rgbd_goal_lateral_difference_m': None,
    }


def test_rejected_rgb_frame_retains_accepted_odom_goal_only_for_steering():
    node = _node()
    node.station_reference = Mock()
    accepted = {
        'valid': True, 'goal_odom': [.4, .03, 0.],
        'goal_map': [1.4, .13, 0.], 'stamp_s': 9.9,
    }
    rejected = {'valid': False, 'reason': 'camera_pose_has_excess_pitch_roll'}
    node._fresh_station_reference = Mock(side_effect=[accepted, rejected, rejected])
    first, current = node._station_goal_reference()
    carried, current = node._station_goal_reference()

    # The steering goal is the filtered accepted frame plus its filter metadata.
    assert first == carried
    assert {key: first[key] for key in accepted} == accepted
    assert first['goal_filter_sample_count'] == 1
    assert current == rejected
    status = node._station_lateral_status()
    assert status['lateral_goal_valid'] is True
    assert status['lateral_reference_valid'] is False
    assert status['internal_lateral_within_tolerance'] is False
    assert status['lateral_goal_observation_stamp_s'] == 9.9


def test_final_lateral_status_uses_current_observation_even_after_goal_freezes():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_goal_frozen = True
    node.station_reference_goal = {
        'valid': True, 'goal_odom': [.4, 0., 0.], 'stamp_s': 9.8,
    }
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'goal_odom': [.4, .028, 0.], 'stamp_s': 10.,
    })
    node._odom_pose = Mock(return_value=(.4, 0., 0.))

    status = node._station_lateral_status()

    assert status['internal_lateral_error_m'] == pytest.approx(.028)
    assert status['internal_lateral_within_tolerance'] is False


def test_auxiliary_rgb_rejection_does_not_replace_or_block_registered_goal():
    node = _node()
    node.station_reference_mode = 'rgbd_auxiliary'
    node.station_reference = Mock()
    node.station_reference_goal = {
        'valid': True, 'goal_odom': [.4, .03, 0.], 'goal_map': [.4, .03, 0.],
        'stamp_s': 9.8}
    node.station_reference_goal_frozen = True
    node._fresh_station_reference = Mock(return_value={
        'valid': False, 'reason': 'camera_pose_has_excess_pitch_roll'})
    node._navigation_ready = Mock(return_value=True)
    node._odom_pose = Mock(return_value=(.4, .03, 0.))

    status = node._station_lateral_status()

    assert status['lateral_reference_valid'] is True
    assert status['lateral_goal_basis'] == 'registered_map_pose_in_odom'
    assert status['internal_lateral_within_tolerance'] is True
    assert status['rgbd_reference_valid'] is False
    assert status['rgbd_reference_reason'] == 'camera_pose_has_excess_pitch_roll'
    assert status['millimeter_accuracy_validated'] is False


def test_moving_table_update_anchors_face_and_steers_on_existing_heading_path(
        monkeypatch):
    node = _node()
    node.station_reference = Mock()
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_taught_map_pose = (.40, .20, 0.)
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'goal_odom': [.40, .10, 0.], 'stamp_s': 10.,
    })
    node._table_observation_context = ({}, (.50, 0.), .6)
    node._table_approach_validator = Mock(return_value=Mock())
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node.box_status = {
        'stamp_s': 9., 'detected': True, 'stable': True,
        'surface_kind': 'table_edge',
        'frame_id': 'camera_color_optical_frame',
        'control_ready': False, 'width_m': .6,
    }
    node.box_status_received_s = time.monotonic()
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(
        transform=_pose())
    node.get_clock = Mock(return_value=_clock())
    node._fresh_depth_frame_after = Mock(return_value=True)
    node._final_gap_m = Mock(return_value=.01)
    node.box_front_half_width_m = .225
    node.observer_stable_frames = 8
    node.emit = Mock()
    measured = {
        'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
        'face_center_map_xy_m': [.50, 0.],
        'outward_normal_map_xy': [-1., 0.],
    }
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target',
        Mock(return_value=measured))
    target = {
        'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
        'face_center_map_xy_m': [.50, 0.],
        'outward_normal_map_xy': [-1., 0.],
        'observed_face_span_m': .6,
    }

    update = node._table_approach_update(
        target, .065, lambda x, y, yaw: (x, y, yaw))
    node.box_status = {**node.box_status, 'stamp_s': 10.}
    control = update((0., 0., 0.))

    assert control['remaining_m'] == pytest.approx(.425)
    assert control['heading_rad'] == pytest.approx(0.)
    assert control['lateral_tracking'] is True
    assert control['lateral_error_m'] == pytest.approx(.10)
    stop = node._continuous_table_stop
    assert stop['target']['face_center_map_xy_m'] == pytest.approx((.50, .10))
    assert stop['target']['outward_normal_map_xy'] == pytest.approx((-1., 0.))
    assert stop['target']['x_m'] == pytest.approx(.425)
    assert stop['target']['y_m'] == pytest.approx(.10)
    assert stop['lateral_reference_valid'] is True

    # Entering the old 5 cm precision band must not discard a remaining
    # lateral error simply because the chassis is currently face-parallel.
    near = update((.39, 0., 0.))
    assert near['heading_rad'] == pytest.approx(0.)
    assert near['lateral_error_m'] == pytest.approx(.10)
    assert node._continuous_table_stop['basis'] != 'aligned_face_frozen_for_precision_band'

    aligned = update((.39, .10, 0.))
    assert aligned['heading_rad'] == pytest.approx(0.)
    assert node.station_reference_goal_frozen is True


def test_destroy_stops_worker_before_ros_node(monkeypatch):
    node = _node()
    worker = threading.Thread(target=node._station_reference_loop)
    node._station_reference_thread = worker
    worker.start()
    destroyed = Mock(return_value='destroyed')
    monkeypatch.setattr(ServiceRoute, 'destroy_node', destroyed)

    assert node.destroy_node() == 'destroyed'

    assert not worker.is_alive()
    destroyed.assert_called_once_with()


def test_go_home_stops_station_rgb_worker_before_dock_return(monkeypatch):
    node = _node()
    node.station_reference = _rgbd_model()
    node._station_reference_perception_active = True
    node._station_reference_pending = ('queued-rgb', 'info', 10.)
    node._set_observer_perception = Mock(return_value=True)
    worker = threading.Thread(target=node._station_reference_loop)
    node._station_reference_thread = worker
    worker.start()
    inherited = Mock(return_value='home')
    monkeypatch.setattr(ServiceRoute, 'go_home', inherited)

    assert node.go_home(execute=True, timeout_s=500.) == 'home'

    assert node._station_reference_perception_active is False
    assert node._station_reference_thread is None
    assert node._station_reference_pending is None
    assert not worker.is_alive()
    inherited.assert_called_once_with(execute=True, timeout_s=500.)
    node._set_observer_perception.assert_called_once_with(False)


def test_go_home_keeps_lidar_route_when_depth_perception_cannot_disable(monkeypatch):
    node = _node()
    node._set_observer_perception = Mock(return_value=False)
    inherited = Mock(return_value=True)
    monkeypatch.setattr(ServiceRoute, 'go_home', inherited)

    assert node.go_home(execute=True, timeout_s=500.) is True

    inherited.assert_called_once_with(execute=True, timeout_s=500.)


def test_go_home_keeps_lidar_route_when_station_worker_stop_times_out(monkeypatch):
    node = _node()
    node._stop_station_reference_perception = Mock(return_value=False)
    node._set_observer_perception = Mock(return_value=True)
    inherited = Mock(return_value=True)
    monkeypatch.setattr(ServiceRoute, 'go_home', inherited)

    assert node.go_home(execute=True, timeout_s=500.) is True

    inherited.assert_called_once_with(execute=True, timeout_s=500.)


def test_rear_box_parking_disables_front_depth_perception(monkeypatch):
    node = _node()
    node._set_observer_perception = Mock(return_value=True)
    inherited = Mock(return_value='parked')
    monkeypatch.setattr(box_service.RearBoxParkingMixin, 'park_rear_box', inherited)

    assert node.park_rear_box('geometry', 'route', execute=True) == 'parked'

    node._set_observer_perception.assert_called_once_with(False)
    inherited.assert_called_once_with('geometry', 'route', execute=True)


def test_station_rgb_callback_does_no_decode_after_perception_stops(monkeypatch):
    node = _node()
    node.station_reference = _rgbd_model()
    node._station_reference_perception_active = False
    decode = Mock(side_effect=AssertionError('RGB decode must stay off at the dock'))
    monkeypatch.setattr(node, '_station_image_array', decode)

    node._station_image_callback(object())

    decode.assert_not_called()
    assert node._station_reference_pending is None


def test_station_worker_drops_inflight_result_when_perception_stops():
    node = _node()
    entered = threading.Event()
    release = threading.Event()
    model = _rgbd_model(contact_reference=None)

    def observe(*_args):
        entered.set()
        assert release.wait(timeout=1.)
        return {'valid': False, 'reason': 'late_result', 'stamp_s': 10.}

    model.observe.side_effect = observe
    node.station_reference = model
    node._station_reference_perception_active = True
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node.get_clock.return_value = _clock(10.1)
    node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), 10.)
    worker = threading.Thread(target=node._station_reference_loop)
    node._station_reference_thread = worker
    worker.start()
    node._station_reference_wake.set()
    assert entered.wait(timeout=1.)
    stopped = threading.Thread(target=node._stop_station_reference_perception)
    stopped.start()
    release.set()
    stopped.join(timeout=1.)

    assert not stopped.is_alive()
    assert not worker.is_alive()
    assert node.station_reference_result is None
    assert not any(call.args and call.args[0] == 'station_reference_input_status'
                   for call in node.emit.call_args_list)


def test_three_consistent_near_tape_views_replace_challenged_goal_without_arrival():
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.parking_contract['yaw_tolerance_rad'] = .03
    old = {'valid': True, 'stamp_s': 9., 'goal_odom': [.4, 0., 0.]}
    node._station_reference_goal_samples = deque([dict(old)]*3, maxlen=5)
    node.station_reference_goal = dict(old)
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = dict(old)
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node._odom_pose = Mock(return_value=(.39, 0., 0.))
    for index, lateral in enumerate((.070, .072, .071)):
        stamp = 10. + index * .4
        node.get_clock.return_value = _clock(stamp+.1)
        node.station_reference.observe.return_value = {
            'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, lateral, .015],
            'goal_left_offset_m': 0.,
            'quality': {'tape_center_base_xy_m': [.2, lateral]},
        }
        node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), stamp)
        node._station_reference_stop.clear()
        # Stop after a single worker iteration, including repeated statuses.
        node._station_reference_reported_status = None
        node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
        node._station_reference_wake.set()
        node._station_reference_loop()
        node._fresh_station_reference = lambda: node.station_reference_result
        if index < 2:
            node._station_goal_reference(freeze=True)
            assert node._station_terminal_goal is None
            assert not node._table_terminal_reference_ready(node.station_reference_result)
            assert node.station_reference_goal['goal_odom'] == old['goal_odom']
    node.emit.side_effect = None
    goal, _ = node._station_goal_reference(freeze=True)
    assert goal['goal_odom'] == pytest.approx([.4, .071, .015])
    assert node._table_rebase_requires_restage
    assert node._table_restage_tape_goal['goal_odom'] == goal['goal_odom']
    assert not node._table_terminal_reference_ready(node.station_reference_result)
    assert node._station_terminal_goal is None
    assert contact.restore_tracking.call_count == 2
    assert node._station_lateral_status()['lateral_reference_valid'] is False


def test_frozen_tape_goal_detects_recorded_drift_against_held_destination():
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.parking_contract['yaw_tolerance_rad'] = .03
    held = {'valid': True, 'stamp_s': 9., 'goal_odom': [.4, 0., 0.]}
    node._station_reference_goal_samples = deque([
        {'valid': True, 'stamp_s': 9. + index * .2,
         'goal_odom': [.4, lateral, 0.]}
        for index, lateral in enumerate((.034, .035, .036))
    ], maxlen=5)
    node.station_reference_goal = dict(held)
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = None
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node._odom_pose = Mock(return_value=(.39, 0., 0.))

    for index, lateral in enumerate((.0394, .0396, .0395)):
        stamp = 10. + index * .3
        node.get_clock.return_value = _clock(stamp + .1)
        node.station_reference.observe.return_value = {
            'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, lateral, 0.],
            'goal_left_offset_m': 0.,
            'quality': {'tape_center_base_xy_m': [.0484566, .0405519]},
        }
        node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), stamp)
        node._station_reference_stop.clear()
        node._station_reference_reported_status = None
        node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
        node._station_reference_wake.set()
        node._station_reference_loop()
        node._fresh_station_reference = lambda: node.station_reference_result
        if index == 0:
            quality = node.station_reference_result['quality']
            assert quality['static_goal_lateral_jump_m'] == pytest.approx(.0044)
            assert quality['held_goal_lateral_jump_m'] == pytest.approx(.0394)
            assert node._station_reference_challenged_goal['goal_odom'] == held['goal_odom']
        if index < 2:
            node._station_goal_reference(freeze=True)
            assert node._station_terminal_goal is None

    node.emit.side_effect = None
    goal, _ = node._station_goal_reference(
        freeze=True, continuous_rebase=True)

    assert goal['goal_odom'] == pytest.approx([.4, .0395, 0.])
    assert node._table_rebase_requires_restage
    assert node._station_terminal_goal is None
    rebased = [call for call in node.emit.call_args_list
               if call.args and call.args[0] == 'table_station_goal_rebased']
    assert rebased[-1].kwargs['continuous_approach_validation'] is False


def test_first_continuous_approach_absorbs_coherent_five_mm_tape_correction():
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.parking_contract['yaw_tolerance_rad'] = .03
    held = {'valid': True, 'stamp_s': 9., 'goal_odom': [.4, 0., 0.]}
    node._station_reference_goal_samples = deque([
        {'valid': True, 'stamp_s': 9. + index * .2,
         'goal_odom': [.4, lateral, 0.]}
        for index, lateral in enumerate((-.0055, -.0060, -.0065))
    ], maxlen=5)
    node.station_reference_goal = dict(held)
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = None
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node._odom_pose = Mock(return_value=(.39, 0., 0.))

    goal = None
    for index, lateral in enumerate((-.0070, -.0072, -.0071)):
        stamp = 10. + index * .3
        node.get_clock.return_value = _clock(stamp + .1)
        node.station_reference.observe.return_value = {
            'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, lateral, 0.],
            'goal_left_offset_m': 0.,
            'quality': {'tape_center_base_xy_m': [.241, -.031]},
        }
        node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), stamp)
        node._station_reference_stop.clear()
        node._station_reference_reported_status = None
        node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
        node._station_reference_wake.set()
        node._station_reference_loop()
        node._fresh_station_reference = lambda: node.station_reference_result
        goal, _ = node._station_goal_reference(
            freeze=True, continuous_rebase=True)
        if index == 0:
            assert node.station_reference_result['valid'] is True
            assert node._station_reference_goal_correction_pending
            assert not getattr(node, '_station_reference_goal_challenged', False)
            assert not node._table_terminal_reference_ready(
                node.station_reference_result)
            status = node._station_lateral_status()
            # The old frozen goal is still exactly centred at the base, but
            # one soft correction frame cannot confirm final parking.
            assert status['lateral_reference_valid'] is False
            assert status['internal_lateral_within_tolerance'] is False
            missing_stamp = stamp + .1
            node.get_clock.return_value = _clock(missing_stamp + .1)
            node.station_reference.observe.return_value = {
                'valid': False, 'stamp_s': missing_stamp,
                'reason': 'tape_pair_missing', 'quality': {},
            }
            node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), missing_stamp)
            node._station_reference_stop.clear()
            node._station_reference_reported_status = None
            node._station_reference_wake.set()
            node._station_reference_loop()
            assert node._station_reference_goal_correction_pending
            assert len(node._station_reference_rebase_samples) == 1
            assert not node._table_terminal_reference_ready(
                node.station_reference_result)

    assert goal['goal_odom'] == pytest.approx([.4, -.0071, 0.])
    assert node.station_reference_goal['goal_odom'] == goal['goal_odom']
    assert not node._table_rebase_requires_restage
    rebased = [call for call in node.emit.call_args_list
               if call.args and call.args[0] == 'table_station_goal_rebased']
    assert rebased[-1].kwargs['continuous_approach_validation'] is True
    assert rebased[-1].kwargs['restage_required'] is False


def test_terminal_tape_goal_is_challenged_by_three_fresh_drifted_views():
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.parking_contract['yaw_tolerance_rad'] = .03
    terminal = {'valid': True, 'stamp_s': 9., 'goal_odom': [.4, 0., 0.]}
    node._station_reference_goal_samples = deque([
        {'valid': True, 'stamp_s': 9. + index * .2,
         'goal_odom': [.4, lateral, 0.]}
        for index, lateral in enumerate((.028, .030, .032))
    ], maxlen=5)
    node.station_reference_goal = dict(terminal)
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = dict(terminal)
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node._odom_pose = Mock(return_value=(.39, 0., 0.))

    for index, lateral in enumerate((.033, .034, .032)):
        stamp = 10. + index * .4
        node.get_clock.return_value = _clock(stamp + .1)
        node.station_reference.observe.return_value = {
            'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, lateral, 0.],
            'goal_left_offset_m': 0.,
            'quality': {'tape_center_base_xy_m': [.0484566, lateral]},
        }
        node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), stamp)
        node._station_reference_stop.clear()
        node._station_reference_reported_status = None
        node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
        node._station_reference_wake.set()
        node._station_reference_loop()
        node._fresh_station_reference = lambda: node.station_reference_result
        if index < 2:
            node._station_goal_reference(freeze=True)
            assert node._station_terminal_goal is None
            assert node._station_reference_goal_challenged
            assert node._station_reference_challenged_goal['goal_odom'] == [0.4, 0., 0.]
            missing_stamp = stamp + .2
            node.get_clock.return_value = _clock(missing_stamp + .1)
            node.station_reference.observe.return_value = {
                'valid': False, 'stamp_s': missing_stamp,
                'reason': 'tape_pair_missing', 'quality': {},
            }
            node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), missing_stamp)
            node._station_reference_stop.clear()
            node._station_reference_reported_status = None
            node._station_reference_wake.set()
            node._station_reference_loop()
            assert len(node._station_reference_rebase_samples) == index + 1

    node.emit.side_effect = None
    goal, _ = node._station_goal_reference(freeze=True)

    assert goal['goal_odom'] == pytest.approx([.4, .033, 0.])
    assert node._table_rebase_requires_restage
    assert node._table_restage_tape_goal['goal_odom'] == goal['goal_odom']
    assert contact.restore_tracking.call_count == 2


def test_tape_rebase_close_view_range_uses_face_alignment_from_base_origin():
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')

    def sample(stamp):
        return dict(valid=True, stamp_s=stamp, goal_odom=[.4, .071, 0.],
                    image_age_at_completion_s=.1,
                    quality={'tape_center_base_xy_m': [.478, .071]})

    assert node._consider_tape_rebase(sample(10.)) is None
    assert node._consider_tape_rebase(sample(10.3)) is None
    assert node._consider_tape_rebase(sample(10.6))['goal_filter_sample_count'] == 3


def test_tape_rebase_accepts_recorded_center_inside_measured_front_extent():
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')

    def sample(stamp, y):
        return dict(valid=True, stamp_s=stamp, goal_odom=[.4, y, 0.],
                    image_age_at_completion_s=.1,
                    quality={'tape_center_base_xy_m': [.0484566338, .0405519012]})

    assert node._consider_tape_rebase(sample(10., .0394)) is None
    assert node._consider_tape_rebase(sample(10.3, .0396)) is None
    rebased = node._consider_tape_rebase(sample(10.6, .0395))

    assert rebased['goal_filter_sample_count'] == 3
    assert rebased['goal_odom'] == pytest.approx([.4, .0395, 0.])
    assert rebased['goal_filter_xy_spread_m'] == pytest.approx(.0001)
    assert rebased['goal_filter_yaw_spread_rad'] == 0.


def test_tape_rebase_remains_coherent_when_consumed_by_entry():
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    rebased = None
    for stamp, y in ((10., .0394), (10.2, .0396), (10.4, .0395)):
        rebased = node._consider_tape_rebase(dict(
            valid=True, stamp_s=stamp, goal_odom=[.4, y, 0.],
            image_age_at_completion_s=.1,
            quality={'tape_center_base_xy_m': [.35, .04]}))
    node._station_reference_rebase_goal = rebased
    node._station_reference_goal_correction_pending = True
    node._fresh_station_reference = Mock(return_value=rebased)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node.get_clock.return_value = _clock(10.5)
    node._operator_link_ready = Mock(return_value=True)
    anchored = {'tape_anchor': True}
    node._station_anchor_target = Mock(return_value=anchored)

    assert node._prepare_table_tape_entry({'map_anchor': True}) is anchored
    assert node.station_reference_goal_frozen
    assert node.station_reference_goal['goal_filter_xy_spread_m'] == pytest.approx(.0001)
    assert node.station_reference_goal['goal_filter_yaw_spread_rad'] == 0.


@pytest.mark.parametrize('dimension', ['lateral', 'yaw'])
def test_tape_rebase_respects_station_coherence_limits(dimension):
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_profile['lateral_tolerance_m'] = .005
    node.parking_contract['yaw_tolerance_rad'] = math.radians(1.)
    for stamp in (10., 10.2):
        assert node._consider_tape_rebase(dict(
            valid=True, stamp_s=stamp, goal_odom=[.4, 0., 0.],
            image_age_at_completion_s=.1,
            quality={'tape_center_base_xy_m': [.35, 0.]})) is None
    third = dict(valid=True, stamp_s=10.4,
                 goal_odom=[.4, .007 if dimension == 'lateral' else 0.,
                            math.radians(1.5) if dimension == 'yaw' else 0.],
                 image_age_at_completion_s=.1,
                 quality={'tape_center_base_xy_m': [.35, 0.]})
    assert node._consider_tape_rebase(third) is None
    assert len(node._station_reference_rebase_samples) == 1


@pytest.mark.parametrize('geometry', [None, {},
                                      {'front_to_wheel_axis': {'value': False}},
                                      {'front_to_wheel_axis': {'value': '0.065'}}])
def test_tape_rebase_rejects_malformed_chassis_geometry(geometry):
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.box_geometry = geometry
    candidate = dict(valid=True, stamp_s=10., goal_odom=[.4, .071, 0.],
                     image_age_at_completion_s=.1,
                     quality={'tape_center_base_xy_m': [.478, .071]})

    assert node._consider_tape_rebase(candidate) is None
    assert node._station_reference_rebase_samples == []


@pytest.mark.parametrize('bad', ['duplicate', 'stale', 'far', 'outlier', 'not_tape'])
def test_tape_rebase_does_not_accept_unconfirmed_frames(bad):
    node = _node()
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    def sample(stamp, y=.071):
        return dict(valid=True, stamp_s=stamp, goal_odom=[.4,y,0.],
                    image_age_at_completion_s=.1,
                    quality={'tape_center_base_xy_m': [.2,y]})
    assert node._consider_tape_rebase(sample(10.)) is None
    assert node._consider_tape_rebase(sample(10.3)) is None
    third = sample(10.6)
    if bad == 'duplicate': third['stamp_s'] = 10.3
    if bad == 'stale': third['image_age_at_completion_s'] = 10.
    if bad == 'far': third['quality']['tape_center_base_xy_m'][0] = .8
    if bad == 'outlier': third['goal_odom'][1] = -.07
    if bad == 'not_tape': node.station_reference.observation_method = 'cup_bottle'
    assert node._consider_tape_rebase(third) is None


@pytest.mark.parametrize('hard,age,discard', [
    (False, .1, True), (True, .1, False), (False, 3., False),
])
def test_missing_pair_discards_only_unconfirmed_soft_goal_candidate(hard, age, discard):
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.station_reference_goal = {
        'valid': True, 'stamp_s': 9.9, 'goal_odom': [.4, 0., 0.],
        'goal_filter_sample_count': 3}
    node.station_reference_goal_frozen = True
    node._station_reference_goal_correction_pending = True
    node._station_reference_goal_challenged = hard
    node._station_reference_rebase_samples = [{'goal_odom': [.4, .0052873, 0.]}]
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node.get_clock.return_value = _clock(10. + age)
    node.station_reference.observe.return_value = {
        'valid': False, 'stamp_s': 10., 'reason': 'tape_pair_missing', 'quality': {}}
    node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), 10.)
    node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
    node._station_reference_wake.set()
    node._station_reference_loop()
    assert node._station_reference_goal_correction_pending is (not discard)
    assert node._station_reference_goal_challenged is hard
    if discard:
        assert node._station_reference_rebase_samples == []
        assert node._table_terminal_reference_ready(node.station_reference_result)
        assert node.station_reference_goal['goal_odom'] == [.4, 0., 0.]


def test_terminal_hold_ignores_queued_soft_rebase_from_recorded_final_approach():
    node = _node()
    node.station_reference_mode = 'rgbd'
    node.station_reference = _rgbd_model(observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    terminal = {'valid': True, 'stamp_s': 10., 'goal_filter_sample_count': 3,
                'goal_odom': [-.8257655896, 3.9196747358, 2.1776272699]}
    changed = dict(terminal, stamp_s=10.5,
                   goal_odom=[-.8336221787, 3.9124874027, 2.1759260628])
    node.station_reference_goal = dict(terminal)
    node._station_terminal_goal = dict(terminal)
    node._station_reference_goal_correction_pending = True
    node._station_reference_rebase_goal = changed
    node._fresh_station_reference = Mock(return_value=changed)
    goal, observed = node._station_goal_reference(freeze=True, continuous_rebase=True)
    assert goal['goal_odom'] == terminal['goal_odom']
    assert observed['goal_odom'] == changed['goal_odom']
    assert node._station_reference_rebase_goal is None
    assert not node._station_reference_goal_correction_pending
    assert node._station_terminal_goal['goal_odom'] == terminal['goal_odom']


def test_terminal_tape_worker_keeps_small_coherent_change_diagnostic():
    node = _node()
    contact = Mock()
    node.station_reference = _rgbd_model(contact_reference=contact,
                                        observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.parking_contract['yaw_tolerance_rad'] = .03
    held = {'valid': True, 'stamp_s': 9., 'goal_odom': [.4, 0., 0.],
            'goal_filter_sample_count': 3}
    node._station_reference_goal_samples = deque([
        dict(held, stamp_s=9.+i*.2) for i in range(3)], maxlen=5)
    node.station_reference_goal = dict(held)
    node.station_reference_goal_frozen = True
    node._station_terminal_goal = dict(held)
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    for index, lateral in enumerate((.0104, .0106, .0105)):
        stamp = 10. + index * .3
        node.get_clock.return_value = _clock(stamp + .1)
        node.station_reference.observe.return_value = {
            'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, lateral, 0.],
            'goal_left_offset_m': 0.,
            'quality': {'tape_center_base_xy_m': [.10, lateral]}}
        node._station_reference_pending = (None, SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame')), stamp)
        node._station_reference_stop.clear()
        node._station_reference_reported_status = None
        node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
        node._station_reference_wake.set()
        node._station_reference_loop()
        node._fresh_station_reference = lambda: node.station_reference_result
        goal, reference = node._station_goal_reference(freeze=True, continuous_rebase=True)
        assert goal['goal_odom'] == held['goal_odom']
        assert reference['goal_odom'][1] == lateral
        assert not getattr(node, '_station_reference_goal_correction_pending', False)
        assert getattr(node, '_station_reference_rebase_goal', None) is None


@pytest.mark.parametrize('case', ['stale', 'wrong_frame', 'not_detected', 'skew'])
def test_tape_depth_join_rejects_unusable_observer_geometry(case):
    node = _node()
    node._table_observation_context = ({'camera_mount': {}}, None, None)
    node.get_clock.return_value = _clock(10.2)
    doc = {'detected': True, 'frame_id': 'camera_color_optical_frame', 'stamp_s': 10.1}
    if case == 'stale': doc['stamp_s'] = 9.6
    if case == 'wrong_frame': doc['frame_id'] = 'unregistered_depth_frame'
    if case == 'not_detected': doc['detected'] = False
    if case == 'skew': doc['stamp_s'] = 9.8
    node.box_status = doc
    node.box_status_history = deque()
    node.parking_tf = Mock()
    with pytest.raises(ValueError, match='live depth table plane unavailable'):
        BoxServiceRoute._station_tape_surface(
            node, 10.15, (0.,0.,0.), 'camera_color_optical_frame')
    node.parking_tf.lookup_transform.assert_not_called()


def test_tape_worker_always_forwards_matched_live_surface():
    node = _node()
    node.station_reference = _rgbd_model(observation_method='table_front_tape')
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_pose())
    node.station_reference.observe.return_value = {
        'valid': False, 'stamp_s': 10., 'reason': 'tape_pair_missing', 'quality': {}}
    surface = {'source': 'live_depth_table_plane'}
    node._station_tape_surface.return_value = surface
    info = SimpleNamespace(header=SimpleNamespace(frame_id='camera_color_optical_frame'))
    node._station_reference_pending = (None, info, 10.)
    node.emit.side_effect = lambda *_a, **_kw: node._station_reference_stop.set()
    node._station_reference_wake.set()
    node._station_reference_loop()
    node.station_reference.observe.assert_called_once_with(
        None, info, (0.,0.,0.), 10., surface=surface)


@pytest.mark.parametrize('stamp,available', [(9.8, True), (9.4, False), (10.1, False)])
def test_tape_initial_goal_keeps_only_still_fresh_measured_samples(stamp, available):
    node = _node()
    node.station_reference = _rgbd_model(observation_method='table_front_tape')
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_mode = 'rgbd'
    node.station_reference_goal = None
    node.station_reference_control_goal = None
    node.station_reference_goal_frozen = False
    node._station_reference_goal_samples = deque([
        {'valid': True, 'stamp_s': stamp, 'goal_odom': [.5, .014, 0.]}])
    latest = {'valid': False, 'reason': 'station_reference_runtime:ValueError',
              'detail': 'tape live depth table plane unavailable'}
    node._fresh_station_reference = Mock(return_value=latest)
    goal, reference = node._station_goal_reference(freeze=True)
    assert (goal is not None) is available
    if available:
        assert goal['goal_odom'] == [.5, .014, 0.]
    assert reference is latest and reference['valid'] is False
    assert not node.station_reference_goal_frozen


def test_profile_accepts_operator_ten_mm_lateral_tolerance(tmp_path, monkeypatch):
    node = _node()
    path = _profiles(tmp_path)
    document = json.loads(path.read_text())
    document['stations']['water_station']['lateral_tolerance_m'] = .01
    path.write_text(json.dumps(document))
    monkeypatch.setattr(box_service, 'TaughtStationReference', Mock(return_value=_rgbd_model()))
    node.configure_station_reference(path, 'water_station')
    assert node.station_reference_profile['lateral_tolerance_m'] == .01
