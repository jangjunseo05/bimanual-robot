"""ROS message boundary tests for the perception-only parking observer."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

from jdamr_cube_navigation.box_top_detection import BoxTopConfig
from jdamr_cube_navigation.depth_box_parking import (
    decode_depth_observation,
    DepthBoxParkingNode,
    LATEST_SENSOR_QOS,
)
import numpy as np
import pytest
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


OPTICAL_FRAME = 'camera_color_optical_frame'


def test_depth_inputs_keep_only_latest_best_effort_sample():
    assert LATEST_SENSOR_QOS.history == HistoryPolicy.KEEP_LAST
    assert LATEST_SENSOR_QOS.depth == 1
    assert LATEST_SENSOR_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert LATEST_SENSOR_QOS.durability == DurabilityPolicy.VOLATILE
    source = Path(__file__).resolve().parents[1].joinpath(
        'jdamr_cube_navigation', 'depth_box_parking.py').read_text()
    assert source.count('self._on_camera_info, LATEST_SENSOR_QOS') == 1
    assert source.count(
        'self._on_depth_with_watchdog_activity, LATEST_SENSOR_QOS') == 1
    assert source.count(
        "String, '/box_parking/perception_status', 1") == 1


def test_predicted_late_depth_yields_once_for_latest_without_starvation():
    node = object.__new__(DepthBoxParkingNode)
    node._max_image_age_s = .5
    node._minimum_period_s = .2
    node._processing_duration_estimate_s = .18
    node._freshness_skip_pending = False
    node._precalculation_skips = 0

    assert node._skip_for_fresher_depth(10.0, 10.25) is True
    assert node._skip_for_fresher_depth(10.0, 10.25) is False
    assert node._precalculation_skips == 1
    assert node._freshness_skip_pending is False


def test_predicted_late_depth_skips_before_decode_and_rate_limit():
    _info, image = _messages(np.ones((2, 2), dtype='<u2'), stamp_s=10.0)
    node = object.__new__(DepthBoxParkingNode)
    node._perception_enabled = True
    node._minimum_period_s, node._last_processed_s = 0.2, -math.inf
    node._processing_duration_estimate_s = .18
    node._freshness_skip_pending = False
    node._precalculation_skips = 0
    node._max_image_age_s = .5
    node._camera_info_history = SimpleNamespace(
        select=lambda *_a, **_k: pytest.fail('predicted-late input was decoded'))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_250_000_000))

    node._on_depth(image)

    assert node._last_processed_s == -math.inf
    assert node._precalculation_skips == 1


def test_measurement_reset_keeps_timing_prediction_and_skip_evidence():
    node = object.__new__(DepthBoxParkingNode)
    node._processing_duration_estimate_s = .3
    node._freshness_skip_pending = True
    node._precalculation_skips = 2
    node._stability = SimpleNamespace(update=lambda _detection: None)
    node._table_tracker = SimpleNamespace(reset=lambda: None)

    node._reset_measurements()

    assert node._processing_duration_estimate_s == .3
    assert node._freshness_skip_pending is False
    assert node._precalculation_skips == 2


def test_long_depth_callback_does_not_trigger_silent_subscription_recovery(
        monkeypatch):
    """Detector runtime is reader activity, without changing image freshness."""
    node = object.__new__(DepthBoxParkingNode)
    node._depth_subscription = SimpleNamespace(get_publisher_count=lambda: 1)
    node._max_image_age_s = .5
    node._last_depth_callback_s = 10.
    node.destroy_subscription = lambda subscription: pytest.fail(
        'active callback must not recreate subscriptions')
    node._on_depth = lambda message: node._recover_silent_depth_subscriptions()
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.time.monotonic',
        lambda: 20.1)

    node._on_depth_with_watchdog_activity(SimpleNamespace())

    assert node._last_depth_callback_s == 20.1
    assert node._depth_callback_active is False


def test_disabled_perception_keeps_watchdog_active_without_running_detection(monkeypatch):
    """The executor can pause point-cloud work without making DDS look silent."""
    node = object.__new__(DepthBoxParkingNode)
    node._perception_enabled = False
    node._last_processed_s = 7.0
    node._last_depth_callback_s = 10.0
    node._depth_callback_active = False
    monotonic = iter((20.0,))
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.time.monotonic',
        lambda: next(monotonic))
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.decode_depth_observation',
        lambda *_a, **_kw: pytest.fail('disabled perception decoded depth'))

    node._on_depth_with_watchdog_activity(SimpleNamespace())

    assert node._last_processed_s == 7.0
    assert node._last_depth_callback_s == 20.0
    assert node._depth_callback_active is False


def test_silent_matched_depth_subscription_is_recreated(monkeypatch):
    """A matched writer with no callbacks rematches without widening freshness."""
    node = object.__new__(DepthBoxParkingNode)
    old_info = SimpleNamespace(name='old_info')
    old_depth = SimpleNamespace(get_publisher_count=lambda: 1, name='old_depth')
    node._camera_info_subscription = old_info
    node._depth_subscription = old_depth
    node._max_image_age_s = .5
    node._last_depth_callback_s = 10.
    node._camera_info_history = SimpleNamespace(name='old_history')
    node._camera_geometry = SimpleNamespace(name='old_geometry')
    destroyed, created, rejected = [], [], []
    node.destroy_subscription = destroyed.append
    node._create_depth_subscriptions = lambda: created.append(True)
    node._reject = rejected.append
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.time.monotonic', lambda: 12.1)

    node._recover_silent_depth_subscriptions()

    assert destroyed == [old_info, old_depth]
    assert created == [True]
    assert rejected == ['depth_subscription_recreated_after_silence']
    assert node._camera_geometry is None
    assert node._last_depth_callback_s == 12.1


def test_silent_depth_subscription_waits_without_a_matched_writer(monkeypatch):
    node = object.__new__(DepthBoxParkingNode)
    node._depth_subscription = SimpleNamespace(get_publisher_count=lambda: 0)
    node._max_image_age_s = .5
    node._last_depth_callback_s = 10.
    node.destroy_subscription = lambda subscription: pytest.fail('unexpected recovery')
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.time.monotonic', lambda: 20.)

    node._recover_silent_depth_subscriptions()


def _messages(depth, *, stamp_s=10.0, step=None, bigendian=False):
    depth = np.asarray(depth)
    height, width = depth.shape
    info = CameraInfo()
    info.header.frame_id = OPTICAL_FRAME
    info.width, info.height = width, height
    info.k = [float('nan')] * 9
    info.d = [0.0] * 5
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [5.0, 0.0, 1.0, 0.0,
              0.0, 5.0, 1.0, 0.0,
              0.0, 0.0, 1.0, 0.0]
    image = Image()
    image.header.frame_id = OPTICAL_FRAME
    image.header.stamp.sec = int(stamp_s)
    image.header.stamp.nanosec = int((stamp_s % 1.0) * 1e9)
    image.width, image.height = width, height
    image.encoding = '16UC1'
    image.is_bigendian = bigendian
    image.step = width * 2 if step is None else step
    image.data = depth.tobytes()
    return info, image


def test_uses_rectified_projection_when_k_contains_nan():
    """Astra's invalid K must not replace its valid projection matrix."""
    depth = np.array([[1000, 2000], [3000, 4000]], dtype='<u2')
    info, image = _messages(depth)
    decoded, intrinsics, stamp_s, age_s = decode_depth_observation(
        info, image, OPTICAL_FRAME, 'rectified_projection', 10.2, 0.5)
    np.testing.assert_array_equal(decoded, depth)
    assert intrinsics.focal_x_px == 5.0
    assert intrinsics.center_x_px == 1.0
    assert stamp_s == pytest.approx(10.0)
    assert age_s == pytest.approx(0.2)


def test_camera_info_callback_preserves_projection_message():
    """The callback retains P instead of prematurely extracting invalid K."""
    info, _ = _messages(np.ones((2, 2), dtype='<u2'))
    node = object.__new__(DepthBoxParkingNode)
    node._on_camera_info(info)
    assert node._camera_info_history.select(
        10_000_000_000, maximum_lag_ns=500_000_000) is info


def _camera_sequence_node(monkeypatch):
    from jdamr_cube_navigation.box_top_detection import DetectionStability
    from jdamr_cube_navigation.camera_info_history import CameraInfoHistory

    detection = SimpleNamespace(
        surface_kind='front', front_distance_m=.8, center_x_m=.04, center_y_m=.23,
        edge_angle_rad=0., width_m=.5, height_m=.25, depth_extent_m=.1,
        plane_normal=(0., 0., -1.), plane_rms_m=.001, inlier_count=100,
        candidate_count=120, confidence=.9)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0., -math.inf
    node._camera_info_history, node._camera_geometry = CameraInfoHistory(), None
    node._optical_frame, node._calibration_model = OPTICAL_FRAME, 'rectified_projection'
    node._max_image_age_s, node._surface_mode, node._config = .5, 'front', BoxTopConfig()
    node._stability = DetectionStability(required_frames=2)
    node.track_resets = []
    node._table_tracker = SimpleNamespace(reset=lambda: node.track_resets.append(True))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=node.now_ns))
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_box_front',
                        lambda *args: detection)

    def frame(stamp, focal=5., *, info_stamp=None, publish_info=True):
        info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000, stamp_s=stamp)
        when = stamp if info_stamp is None else info_stamp
        info.header.stamp.sec = int(when)
        info.header.stamp.nanosec = int(round((when % 1.) * 1e9))
        info.p[0] = focal
        if publish_info:
            node._on_camera_info(info)
        node.now_ns = int((stamp + .1) * 1e9)
        node._on_depth(image)
        return json.loads(messages[-1].data)

    return node, frame


def test_camera_geometry_change_resets_both_histories_without_waiting(monkeypatch):
    node, frame = _camera_sequence_node(monkeypatch)
    assert frame(10.)['stable_frame_count'] == 1
    assert frame(10.125)['stable']
    # NaN factory K is unchanged; only the selected, validated P enters identity.
    assert frame(10.25)['stable']
    assert not node.track_resets
    changed = frame(10.375, focal=6.)
    assert changed['stable'] is False and changed['stable_frame_count'] == 1
    assert len(node.track_resets) == 1
    assert frame(10.5, focal=6.)['stable']


def test_future_camera_info_does_not_overwrite_the_prior_geometry(monkeypatch):
    node, frame = _camera_sequence_node(monkeypatch)
    assert frame(10.)['stable_frame_count'] == 1
    prior = frame(10.125, focal=7., info_stamp=11.)
    assert prior['stable']
    assert prior['camera_info_stamp_s'] == 10.
    assert prior['camera_info_age_s'] == .125
    assert not node.track_resets
    current = frame(11., publish_info=False)
    assert not current['stable'] and current['stable_frame_count'] == 1
    assert len(node.track_resets) == 1


def test_future_only_info_recovers_on_the_next_valid_frame(monkeypatch):
    _, frame = _camera_sequence_node(monkeypatch)
    assert frame(10., info_stamp=11.)['reason'] == 'camera_info_unavailable_for_depth_stamp'
    result = frame(10.125)
    assert result['detected'] and result['stable_frame_count'] == 1


def test_depth_clock_rewind_starts_new_observation_history(monkeypatch):
    node, frame = _camera_sequence_node(monkeypatch)
    frame(10.)
    assert frame(10.125)['stable']
    result = frame(1.)
    assert result['detected'] and result['stable_frame_count'] == 1
    assert not result['stable'] and len(node.track_resets) == 1
    assert frame(1.125)['stable']


def test_stale_stamped_info_is_rejected_but_static_info_remains_timeless(monkeypatch):
    node, frame = _camera_sequence_node(monkeypatch)
    frame(10.)
    result = frame(20., publish_info=False)
    assert result['reason'] == 'camera_info_unavailable_for_depth_stamp'
    assert not result['detected'] and not result['stable']
    assert frame(20.125)['stable_frame_count'] == 1

    _, static_frame = _camera_sequence_node(monkeypatch)
    assert static_frame(10., info_stamp=0.)['camera_info_mode'] == 'static'
    result = static_frame(20., publish_info=False)
    assert result['stable'] and result['camera_info_age_s'] is None


def test_invalid_info_cannot_be_bypassed_by_cached_valid_calibration(monkeypatch):
    node, frame = _camera_sequence_node(monkeypatch)
    frame(10.)
    bad, _ = _messages(np.ones((2, 2), dtype='<u2'))
    bad.header.stamp.sec = -1
    node._on_camera_info(bad)
    result = frame(10.125, publish_info=False)
    assert result['reason'] == 'camera_info_unavailable_for_depth_stamp'
    assert not result['detected']
    assert frame(10.25)['stable_frame_count'] == 1


def test_honors_big_endian_rows_with_padding():
    """The shared decoder preserves row stride and byte order."""
    rows = np.array([[1000, 2000, 9999], [3000, 4000, 9999]], dtype='>u2')
    info, image = _messages(rows[:, :2], step=6, bigendian=True)
    image.data = rows.tobytes()
    decoded, _, _, _ = decode_depth_observation(
        info, image, OPTICAL_FRAME, 'rectified_projection', 10.1, 0.5)
    np.testing.assert_array_equal(decoded, [[1000, 2000], [3000, 4000]])


@pytest.mark.parametrize(
    ('mutate', 'reason'),
    [
        (lambda info, image: setattr(info, 'p', [float('nan')] * 12),
         'invalid_rectified_projection'),
        (lambda info, image: setattr(image, 'width', 3),
         'camera_dimensions_mismatch'),
        (lambda info, image: setattr(image, 'data', b'\x00'),
         'data buffer is smaller than image dimensions'),
        (lambda info, image: setattr(image, 'encoding', 'rgb8'),
         'unsupported_depth_encoding:rgb8'),
    ],
)
def test_rejects_invalid_ros_inputs_without_numpy_reshape_errors(
        mutate, reason):
    """Malformed ROS fields fail with a bounded input reason."""
    info, image = _messages(np.ones((2, 2), dtype='<u2'))
    mutate(info, image)
    with pytest.raises(ValueError, match=reason):
        decode_depth_observation(
            info, image, OPTICAL_FRAME, 'rectified_projection', 10.1, 0.5)


@pytest.mark.parametrize(('stamp_s', 'now_s'), [(9.0, 10.0), (11.0, 10.0)])
def test_rejects_stale_and_future_images(stamp_s, now_s):
    """Only measurements inside the configured age window are accepted."""
    info, image = _messages(
        np.ones((2, 2), dtype='<u2'), stamp_s=stamp_s)
    with pytest.raises(ValueError, match='stale_or_future_depth'):
        decode_depth_observation(
            info, image, OPTICAL_FRAME, 'rectified_projection', now_s, 0.5)


def test_rejection_callback_resets_stability_and_publishes_safe_state():
    """Every rejection clears history and keeps motion control disabled."""
    class StabilityRecorder:
        def __init__(self):
            self.updates = []

        def update(self, detection):
            self.updates.append(detection)
            return False

    node = object.__new__(DepthBoxParkingNode)
    node._stability = StabilityRecorder()

    class PublisherRecorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node._status = PublisherRecorder()
    node._reject('invalid_rectified_projection')
    assert node._stability.updates == [None]
    assert json.loads(node._status.messages[0].data) == {
        'detected': False,
        'stable': False,
        'stable_frame_count': 0,
        'reason': 'invalid_rectified_projection',
        'control_ready': False,
    }


def test_invalid_camera_info_is_rejected_by_depth_callback_without_raising():
    """Malformed calibration reaches a safe callback rejection document."""
    info, image = _messages(np.ones((2, 2), dtype='<u2'))
    info.p = [float('nan')] * 12
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s = 0.0
    node._last_processed_s = -math.inf
    node._on_camera_info(info)
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model = 'rectified_projection'
    node._max_image_age_s = 0.5
    node._surface_mode = 'front'
    node._config = BoxTopConfig()
    node._stability = SimpleNamespace(update=lambda detection: False)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert document['reason'] == 'invalid_rectified_projection'
    assert document['detected'] is False
    assert document['stable'] is False
    assert document['control_ready'] is False


def test_observer_source_never_publishes_velocity_or_control_ready_true():
    """The observer source cannot acquire a velocity output path."""
    source = Path(__file__).resolve().parents[1].joinpath(
            'jdamr_cube_navigation', 'depth_box_parking.py').read_text()
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source
    assert "'control_ready': True" not in source


def test_observer_declares_camera_mount_angles_for_front_detection():
    """Startup parameters carry URDF roll and pitch into detection config."""
    source = Path(__file__).resolve().parents[1].joinpath(
        'jdamr_cube_navigation', 'depth_box_parking.py').read_text()
    assert source.count(
        "('camera_roll_rad', defaults.camera_roll_rad)") == 1
    assert source.count(
        "('camera_pitch_rad', defaults.camera_pitch_rad)") == 1


def test_result_expiring_during_detection_is_rejected_with_latency_details(monkeypatch):
    """A slow detector must not turn an expired frame into stable evidence."""
    info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0.0, -math.inf
    node._on_camera_info(info)
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model, node._max_image_age_s = 'rectified_projection', .5
    node._surface_mode, node._config = 'front', BoxTopConfig()
    stability_updates = []
    node._stability = SimpleNamespace(
        update=lambda detection: stability_updates.append(detection) or False,
        frame_count=7)
    clock_ticks = iter((10_100_000_000, 10_700_000_000))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=next(clock_ticks)))
    detection = SimpleNamespace(surface_kind='front')
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.detect_box_front', lambda *_: detection)
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert stability_updates == [None]
    assert document['reason'] == 'expired_during_processing'
    assert document['detected'] is False
    assert document['stable'] is False
    assert document['stable_frame_count'] == 0
    assert document['input_age_s'] == pytest.approx(.1)
    assert document['age_s'] == pytest.approx(.7)
    assert document['processing_duration_s'] >= 0.
    assert document['processing_duration_s'] == pytest.approx(
        document['input_decode_duration_s']
        + document['calculation_duration_s']
        + document['postprocess_duration_s'])
    assert document['previous_status_serialization_duration_s'] is None
    assert document['previous_status_publish_duration_s'] is None
    assert document['control_ready'] is False


def test_publish_reports_previous_output_cost_without_hiding_current_phases(monkeypatch):
    node = object.__new__(DepthBoxParkingNode)
    node._last_status_serialization_duration_s = .004
    node._last_status_publish_duration_s = .006
    node._precalculation_skips = 2
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    ticks = iter((4., 5., 5.1, 5.2, 5.3))
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.time.monotonic',
        lambda: next(ticks))

    node._publish({'detected': False}, timing_markers=(1., 2., 3.))

    document = json.loads(messages[0].data)
    assert document['input_decode_duration_s'] == 1.
    assert document['calculation_duration_s'] == 1.
    assert document['postprocess_duration_s'] == 1.
    assert document['processing_duration_s'] == 3.
    assert document['precalculation_skipped_frames'] == 2
    assert document['previous_status_serialization_duration_s'] == .004
    assert document['previous_status_publish_duration_s'] == .006
    assert node._last_status_serialization_duration_s == pytest.approx(.1)
    assert node._last_status_publish_duration_s == pytest.approx(.1)
    assert node._processing_duration_estimate_s == 3.
    assert node._precalculation_skips == 0


def test_front_observation_publishes_optical_vertical_position(monkeypatch):
    """The JSON boundary retains the detected optical Y face coordinate."""
    info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000)
    detection = SimpleNamespace(
        surface_kind='front', front_distance_m=0.8,
        center_x_m=0.04, center_y_m=0.23,
        edge_angle_rad=0.0, width_m=0.5, height_m=0.25,
        depth_extent_m=0.1, plane_normal=(0.0, 0.0, -1.0),
        plane_rms_m=0.001, inlier_count=100, candidate_count=120,
        confidence=0.9)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0.0, -math.inf
    node._on_camera_info(info)
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model, node._max_image_age_s = 'rectified_projection', .5
    node._surface_mode, node._config = 'front', BoxTopConfig()
    node._stability = SimpleNamespace(update=lambda item: True, frame_count=8)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    monkeypatch.setattr(
        'jdamr_cube_navigation.depth_box_parking.detect_box_front',
        lambda *_: detection)
    messages = []
    node._status = SimpleNamespace(publish=messages.append)

    node._on_depth(image)

    document = json.loads(messages[0].data)
    assert document['vertical_position_m'] == pytest.approx(0.23)


def test_executor_may_narrow_only_the_maximum_depth_within_the_configured_range():
    """table_01 (2026-10-01): a far wide plane beat the box; the window cuts it."""
    class StabilityRecorder:
        def __init__(self):
            self.updates = []

        def update(self, detection):
            self.updates.append(detection)
            return False

    node = object.__new__(DepthBoxParkingNode)
    node._config = BoxTopConfig()
    node._depth_ceiling_m = node._config.maximum_depth_m
    node._stability = StabilityRecorder()

    def set_(name, value):
        return node._on_parameters([SimpleNamespace(name=name, value=value)])

    assert set_('maximum_depth_m', 1.15).successful
    assert node._config.maximum_depth_m == 1.15
    assert node._stability.updates == [None]
    assert set_('maximum_depth_m', node._depth_ceiling_m).successful
    for value in (node._depth_ceiling_m + 0.1, node._config.minimum_depth_m, True,
                  float('nan')):
        assert not set_('maximum_depth_m', value).successful
    assert not set_('minimum_width_m', 0.1).successful
    assert node._config.maximum_depth_m == node._depth_ceiling_m


def _surface_parameter_node():
    from jdamr_cube_navigation.table_edge_detection import TableEdgeConfig
    node = object.__new__(DepthBoxParkingNode)
    node._config = BoxTopConfig()
    node._table_config = TableEdgeConfig()
    node._surface_mode = 'front'
    node._depth_ceiling_m = node._config.maximum_depth_m
    node.resets = []
    node._stability = SimpleNamespace(update=node.resets.append)
    return node


def _parameters(**values):
    return [SimpleNamespace(name=name, value=value) for name, value in values.items()]


def test_unchanged_observer_parameters_preserve_accumulated_stability():
    node = _surface_parameter_node()
    values = {'surface_mode': 'table_edge', 'camera_height_m': 1.26,
              'camera_pitch_rad': .9, 'camera_roll_rad': -.03, 'maximum_depth_m': 1.2}
    assert node._on_parameters(_parameters(**values)).successful
    node.resets.clear()
    assert node._on_parameters(_parameters(**values)).successful
    assert not node.resets
    assert node._on_parameters(_parameters(camera_pitch_rad=.91)).successful
    assert node.resets == [None]


def test_perception_gate_is_boolean_and_resets_accumulated_stability():
    node = _surface_parameter_node()
    messages = []
    node._status = SimpleNamespace(publish=messages.append)

    assert not node._on_parameters(_parameters(perception_enabled=0)).successful
    assert node._on_parameters(_parameters(perception_enabled=False)).successful
    assert node._perception_enabled is False
    assert node.resets == [None]
    assert json.loads(messages[-1].data)['reason'] == 'perception_disabled'

    node.resets.clear()
    assert node._on_parameters(_parameters(perception_enabled=False)).successful
    assert not node.resets
    assert node._on_parameters(_parameters(perception_enabled=True)).successful
    assert node._perception_enabled is True
    assert node.resets == [None]
    assert json.loads(messages[-1].data)['reason'] == 'perception_enabled_waiting_for_depth'


def test_table_mode_requires_height_and_updates_geometry_as_one_configuration():
    """No old frame survives the atomic surface and calibration switch."""
    node = _surface_parameter_node()
    assert not node._on_parameters(_parameters(surface_mode='table_edge')).successful
    assert node._surface_mode == 'front'
    assert not node.resets
    result = node._on_parameters(_parameters(
        surface_mode='table_edge', camera_height_m=1.1,
        camera_roll_rad=0.02, camera_pitch_rad=0.6))
    assert result.successful
    assert node._surface_mode == 'table_edge'
    assert node._table_config.camera_height_m == 1.1
    assert node._table_config.camera_pitch_rad == node._config.camera_pitch_rad == 0.6
    assert node.resets == [None]


def test_real_observer_keeps_table_bounds_when_switching_from_box_parameters():
    """Table mode uses the sensor range and retains its horizontal-plane bound."""
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(domain_id=91)
    node = None
    try:
        node = DepthBoxParkingNode()
        result = node.set_parameters_atomically([
            Parameter('surface_mode', value='table_edge'),
            Parameter('camera_height_m', value=1.0),
            Parameter('camera_pitch_rad', value=0.5),
        ])
        assert result.successful
        assert node._table_config.minimum_depth_m == 0.35
        assert node._table_config.minimum_normal_y == 0.90
        assert node._config.minimum_depth_m == 0.35
        assert node._config.minimum_normal_y == 0.65
        assert not node.set_parameters_atomically([
            Parameter('maximum_depth_m', value=0.35)]).successful
        assert node._table_config.maximum_depth_m == 2.0
        assert node.set_parameters_atomically([
            Parameter('maximum_depth_m', value=0.39)]).successful
        assert node._table_config.maximum_depth_m == 0.39
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_standalone_observer_loads_current_mount_and_allows_runtime_updates(monkeypatch):
    import rclpy
    from rclpy.parameter import Parameter
    from jdamr_cube_navigation.observer_camera_mount import load_observer_camera_mount

    package = Path(__file__).resolve().parents[1]
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.get_package_share_directory',
                        lambda name: str(package))
    expected = load_observer_camera_mount('camera_mount_current.yaml', package / 'config')
    rclpy.init(args=['--ros-args', '--params-file',
                    str(package / 'config/depth_box_parking.yaml')], domain_id=91)
    node = None
    try:
        node = DepthBoxParkingNode()
        for key, value in expected.items():
            assert node.get_parameter(key).value == value
            assert getattr(node._table_config, key) == value
        assert node._config.camera_pitch_rad == expected['camera_pitch_rad']
        assert node.set_parameters_atomically([
            Parameter('camera_pitch_rad', value=.6)]).successful
        assert node._table_config.camera_pitch_rad == .6
        assert not node.set_parameters_atomically([
            Parameter('camera_mount_file', value='another.yaml')]).successful
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('minimum_depth_m', [0.30, 0.50])
def test_table_mode_inherits_configured_sensor_minimum(minimum_depth_m):
    """Switching surface modes must not impose another sensor range cutoff."""
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(
        args=['--ros-args', '-p', f'minimum_depth_m:={minimum_depth_m}'],
        domain_id=91)
    node = None
    try:
        node = DepthBoxParkingNode()
        assert node._table_config.minimum_depth_m == minimum_depth_m
        assert node.set_parameters_atomically([
            Parameter('surface_mode', value='table_edge'),
            Parameter('camera_height_m', value=1.0)]).successful
        assert node._table_config.minimum_depth_m == minimum_depth_m
        assert not node.set_parameters_atomically([
            Parameter('maximum_depth_m', value=minimum_depth_m)]).successful
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('height', [0.0, -1.0, True, float('nan')])
def test_bad_height_does_not_partially_change_mode_or_camera_angles(height):
    """An invalid batch keeps the previous usable observer state."""
    node = _surface_parameter_node()
    before = node._config
    result = node._on_parameters(_parameters(
        surface_mode='table_edge', camera_roll_rad=0.1, camera_height_m=height))
    assert not result.successful
    assert node._config == before
    assert node._surface_mode == 'front'
    assert node._table_config.camera_height_m == 0.0
    assert not node.resets


def test_camera_reposition_and_return_to_box_clear_previous_stability():
    """Both mount changes and switching back discard prior evidence."""
    node = _surface_parameter_node()
    node._processing_duration_estimate_s = .3
    node._freshness_skip_pending = True
    assert node._on_parameters(_parameters(
        surface_mode='table_edge', camera_height_m=0.95, camera_pitch_rad=0.4)).successful
    assert node._processing_duration_estimate_s is None
    assert node._freshness_skip_pending is False
    assert node._on_parameters(_parameters(
        camera_height_m=1.1, camera_pitch_rad=0.6)).successful
    assert node._table_config.camera_height_m == 1.1
    assert node._table_config.camera_pitch_rad == 0.6
    assert node._on_parameters(_parameters(surface_mode='front')).successful
    assert node._surface_mode == 'front'
    assert node.resets == [None, None, None]


def test_table_edge_json_carries_endpoints_and_calibration_without_velocity(monkeypatch):
    """Endpoint geometry crosses the ROS boundary without granting motion."""
    from jdamr_cube_navigation.table_edge_detection import TableEdgeConfig
    info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000)
    detection = SimpleNamespace(
        surface_kind='table_edge', front_distance_m=0.8,
        center_x_m=0.04, center_y_m=0.23,
        edge_angle_rad=0.0, width_m=0.5, height_m=0.0,
        depth_extent_m=0.3, plane_normal=(0.0, -0.9, -0.435889894),
        plane_rms_m=0.001, inlier_count=200, candidate_count=240,
        confidence=0.9, edge_endpoints_optical_m=((-0.25, 0.23, 0.8), (0.25, 0.23, 0.8)),
        table_height_m=0.75, edge_rms_m=0.002)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0.0, -math.inf
    node._on_camera_info(info)
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model, node._max_image_age_s = 'rectified_projection', .5
    node._surface_mode, node._config = 'table_edge', BoxTopConfig()
    node._table_config = TableEdgeConfig(camera_height_m=1.0, camera_pitch_rad=0.5)
    node._stability = SimpleNamespace(update=lambda item: True, frame_count=8)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    called = []

    def detect(_depth, _intrinsics, config, *, plane_result=None):
        called.append(config)
        return detection

    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_edge', detect)
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert called == [node._table_config]
    assert document['surface_kind'] == 'table_edge'
    assert document['edge_endpoints_optical_m'] == [[-0.25, 0.23, 0.8], [0.25, 0.23, 0.8]]
    assert document['table_height_m'] == 0.75
    assert document['camera_height_m'] == 1.0
    assert document['camera_pitch_rad'] == 0.5
    assert document['stable'] and document['control_ready'] is False


def test_component_tracking_adapter_preserves_measurement_time_and_odom_identity(monkeypatch):
    from jdamr_cube_navigation.table_surface_tracking import TableSurfaceTracker, TrackConfig
    from test_table_edge_detection import INTRINSICS, config
    node = object.__new__(DepthBoxParkingNode)
    node._table_components = True
    node._table_request_enabled = True
    node._table_config = config()
    node._table_tracker = TableSurfaceTracker(TrackConfig(required_frames=2))
    transform = np.eye(4)
    node._table_tf = SimpleNamespace(
        lookup_transform=lambda *a, **kw: SimpleNamespace(transform=None))
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.transform_matrix',
                        lambda value: transform.copy())
    world = np.array([[-.4, 1., .75], [.4, 1., .75]])
    candidates = [SimpleNamespace(edge_endpoints_optical_m=world)]
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene',
                        lambda *a, **kw: SimpleNamespace(
                            candidates=tuple(candidates), component_sizes=(200,),
                            input_point_count=240, height_point_count=200,
                            evaluated_components=1))
    image = np.ones((240, 320), dtype=np.uint16)
    _, first, stable = node._table_observation(image, INTRINSICS, 10., OPTICAL_FRAME)
    assert stable is False
    transform[1, 3] = .02
    candidates[:] = [SimpleNamespace(edge_endpoints_optical_m=world - transform[:3, 3])]
    detection, second, stable = node._table_observation(image, INTRINSICS, 10.1, OPTICAL_FRAME)
    assert stable is True and detection is candidates[0]
    assert first['table_track_id'] == second['table_track_id']
    assert second['table_track_frame_count'] == 2
    candidates.clear()
    detection, predicted, stable = node._table_observation(image, INTRINSICS, 10.2, OPTICAL_FRAME)
    assert detection is None and stable is False
    assert predicted['table_track_state'] == 'predicted'
    assert predicted['table_measurement_stamp_s'] == 10.1


def test_an_ambiguous_scene_is_not_forwarded_as_a_parking_target(monkeypatch):
    from test_table_edge_detection import INTRINSICS, config
    node = object.__new__(DepthBoxParkingNode)
    node._table_components, node._table_tracker = True, None
    node._table_request_enabled = True
    node._table_config = config()
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene',
                        lambda *a, **kw: SimpleNamespace(
                            candidates=(object(), object()), component_sizes=(200, 200),
                            input_point_count=500, height_point_count=400,
                            evaluated_components=2))
    detection, fields, _ = node._table_observation(None, INTRINSICS, 10., OPTICAL_FRAME)
    assert detection is None and fields['table_candidate_count'] == 2


def test_target_region_is_applied_before_fitting_and_final_selection(monkeypatch):
    from jdamr_cube_navigation.table_surface_model import TableScene
    from test_table_edge_detection import INTRINSICS, config

    node = object.__new__(DepthBoxParkingNode)
    node._table_components = node._table_request_enabled = True
    node._table_tracker, node._table_config = None, config()
    node._table_region_map, node._table_region_odom = (1., 2., .3), None
    transform = np.eye(4)
    transform[0, 3] = 2.
    lookups = []

    def lookup(target, source, *args, **kwargs):
        lookups.append(source)
        return SimpleNamespace(transform=transform.copy())

    node._table_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.transform_matrix', lambda x: x)
    outside = SimpleNamespace(edge_endpoints_optical_m=((10., 2., 0.), (11., 2., 0.)))
    selected = SimpleNamespace(edge_endpoints_optical_m=((.9, 2., 0.), (1.1, 2., 0.)))
    priorities = []

    def detect(*args, **kwargs):
        priorities.append(kwargs)
        return TableScene((outside, selected), (400, 200), 800, 600, 2, 1)

    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene', detect)
    detection, fields, _ = node._table_observation(None, INTRINSICS, 10., OPTICAL_FRAME)
    assert detection is selected
    assert priorities[0]['target_region_odom'].center_xy_m == (3., 2.)
    np.testing.assert_array_equal(priorities[0]['odom_from_optical'], transform)
    assert fields['table_region_prioritized_components'] == 1
    node._table_observation(None, INTRINSICS, 10.1, OPTICAL_FRAME)
    assert lookups == [OPTICAL_FRAME, 'map', OPTICAL_FRAME]


@pytest.mark.parametrize('candidate_count', [0, 1])
@pytest.mark.parametrize('missing_frame', [OPTICAL_FRAME, 'map'])
@pytest.mark.parametrize('invalid_transform', [False, True])
def test_tf_failure_keeps_scene_evidence_and_clears_stability(
        monkeypatch, candidate_count, missing_frame, invalid_transform):
    from jdamr_cube_navigation.box_top_detection import DetectionStability
    from jdamr_cube_navigation.table_surface_model import TableScene
    from jdamr_cube_navigation.table_surface_tracking import TableSurfaceTracker, TrackConfig
    from test_table_edge_detection import config
    from tf2_ros import TransformException

    info, image = _messages(np.ones((2, 2), dtype='<u2') * 1000)
    node = object.__new__(DepthBoxParkingNode)
    node._minimum_period_s, node._last_processed_s = 0.0, -math.inf
    node._on_camera_info(info)
    node._optical_frame = OPTICAL_FRAME
    node._calibration_model, node._max_image_age_s = 'rectified_projection', .5
    node._surface_mode, node._config = 'table_edge', BoxTopConfig()
    node._table_components = node._table_request_enabled = True
    node._table_config = config()
    node._table_calibration_id = 'mount-a'
    node._table_region_map, node._table_region_odom = (0., 1., .2), None
    node._table_tracker = TableSurfaceTracker(TrackConfig(required_frames=2))
    node._stability = DetectionStability()
    candidate = SimpleNamespace(edge_endpoints_optical_m=np.array([
        [-.4, 1., .75], [.4, 1., .75]]))
    node._table_tracker.update((candidate,), 9.8, np.eye(4), 'previous')
    node._table_tracker.update((candidate,), 9.9, np.eye(4), 'previous')
    scene = TableScene((candidate,) if candidate_count else (), (200,), 500, 200, 1)
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene',
                        lambda *args, **kwargs: scene)
    def matrix(value):
        if value is None:
            raise ValueError('invalid_transform_quaternion')
        return value

    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.transform_matrix', matrix)

    def lookup(target, source, *args, **kwargs):
        if source == missing_frame:
            if invalid_transform:
                return SimpleNamespace(transform=None)
            raise TransformException(f'missing {source}')
        return SimpleNamespace(transform=np.eye(4))

    node._table_tf = SimpleNamespace(lookup_transform=lookup)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    messages = []
    node._status = SimpleNamespace(publish=messages.append)
    node._on_depth(image)
    document = json.loads(messages[0].data)
    assert document['table_candidate_count'] == candidate_count
    assert document['table_component_count'] == 1
    assert document['table_evaluated_components'] == 1
    assert document['table_input_point_count'] == 500
    assert document['table_height_point_count'] == 200
    assert document['table_track_state'] == 'transform_unavailable'
    assert document['reason'].startswith(
        'table_tracking_transform_invalid:' if invalid_transform
        else 'table_tracking_transform_unavailable:')
    assert document['stamp_s'] == 10.
    assert document['frame_id'] == OPTICAL_FRAME
    assert document['processing_duration_s'] >= 0.
    assert document['detected'] is document['stable'] is document['control_ready'] is False
    assert document['stable_frame_count'] == 0
    assert node._table_tracker._world is None

    node._table_tf.lookup_transform = lambda *args, **kwargs: SimpleNamespace(
        transform=np.eye(4))
    _, fields, stable = node._table_observation(
        np.ones((2, 2), dtype=np.uint16),
        decode_depth_observation(info, image, OPTICAL_FRAME,
                                 'rectified_projection', 10.1, .5)[1],
        10.1, OPTICAL_FRAME)
    assert fields['table_track_frame_count'] == candidate_count
    assert stable is False


def test_selected_region_is_frozen_in_odom_across_map_localization_jumps(monkeypatch):
    from jdamr_cube_navigation.table_surface_tracking import TableSurfaceTracker, TrackConfig
    from test_table_edge_detection import INTRINSICS, config
    node = _surface_parameter_node()
    node._table_components, node._table_config = True, config()
    node._table_request_enabled = True
    node._table_calibration_id = 'mount-a'
    node._table_tracker = TableSurfaceTracker(TrackConfig(required_frames=2))
    node._table_region_map, node._table_region_odom = (0., 1., .2), None
    calls = []
    shifted_map = np.eye(4)

    def lookup(target, source, *args, **kwargs):
        calls.append((target, source))
        return SimpleNamespace(transform=shifted_map.copy() if source == 'map' else np.eye(4))

    node._table_tf = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.transform_matrix',
                        lambda value: value)
    candidate = SimpleNamespace(edge_endpoints_optical_m=np.array([
        [-.4, 1., .75], [.4, 1., .75]]))
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene',
                        lambda *a, **kw: SimpleNamespace(candidates=(candidate,),
                                                   input_point_count=240, height_point_count=200,
                                                   component_sizes=(200,), evaluated_components=1))
    _, first, stable = node._table_observation(None, INTRINSICS, 10., OPTICAL_FRAME)
    assert not stable
    shifted_map[0, 3] = 1.
    detection, second, stable = node._table_observation(None, INTRINSICS, 10.1, OPTICAL_FRAME)
    assert detection is candidate and stable
    assert second['table_track_id'] == first['table_track_id']
    assert calls.count(('odom', 'map')) == 1
    anchored_region = node._table_region_odom
    assert node._on_parameters(_parameters(maximum_depth_m=1.9)).successful
    assert node._table_region_odom is anchored_region
    detection, _, _ = node._table_observation(None, INTRINSICS, 10.2, OPTICAL_FRAME)
    assert detection is candidate
    assert calls.count(('odom', 'map')) == 1


def test_runtime_region_and_full_mount_revision_update_atomically():
    node = _surface_parameter_node()
    node._table_components = True
    values = {'table_request_enabled': True,
              'table_target_region_map': '[1.0,2.0,0.3]', 'table_calibration_id': 'mount-a'}
    assert node._on_parameters(_parameters(**values)).successful
    assert node._table_region_map == (1., 2., .3)
    assert node._table_calibration_id == 'mount-a'
    node.resets.clear()
    assert node._on_parameters(_parameters(**values)).successful
    assert not node.resets
    assert not node._on_parameters(_parameters(
        table_target_region_map='[true,2.0,0.3]', table_calibration_id='mount-b')).successful
    assert node._table_calibration_id == 'mount-a'


def test_explicit_pointcloud_request_does_not_silently_run_the_legacy_observer():
    node = _surface_parameter_node()
    assert not node._on_parameters(_parameters(table_target_region_map='[1,2,0.3]')).successful


@pytest.mark.parametrize('values', [
    {'table_request_enabled': True},
    {'table_request_enabled': True, 'table_target_region_map': '[1,2,0.3]'},
    {'table_request_enabled': True, 'table_calibration_id': 'mount-a'},
    {'table_request_enabled': True, 'table_target_region_map': '[1,2,0.3]',
     'table_calibration_id': ' '},
])
def test_pointcloud_enable_needs_a_complete_selection_and_calibration(values):
    node = _surface_parameter_node()
    node._table_components = True
    assert not node._on_parameters(_parameters(**values)).successful
    assert not getattr(node, '_table_request_enabled', False)


def test_disabling_pointcloud_request_clears_selection_and_uses_legacy_detector(monkeypatch):
    node = _surface_parameter_node()
    node._table_components = True
    assert node._on_parameters(_parameters(
        table_request_enabled=True, table_target_region_map='[1,2,0.3]',
        table_calibration_id='mount-a')).successful
    node._table_region_odom = object()
    node.resets.clear()
    assert node._on_parameters(_parameters(
        table_request_enabled=False, table_target_region_map='',
        table_calibration_id='')).successful
    assert node._table_region_map is None and node._table_region_odom is None
    assert node._table_calibration_id == '' and node.resets
    legacy = object()
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_edge',
                        lambda *a, **kw: legacy)
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_scene',
                        lambda *a: pytest.fail('disabled request used experimental detection'))
    result, fields, stable = node._table_observation(None, None, 10., OPTICAL_FRAME)
    assert result is legacy and fields == {} and stable is None


def test_tabletop_plane_is_published_even_when_front_edge_is_occluded(monkeypatch):
    node = object.__new__(DepthBoxParkingNode)
    node._table_components = False
    node._table_config = object()
    measured = {'source': 'current_depth_tabletop_fit', 'inlier_count': 200}
    def detect(_depth, _intrinsics, _config, *, plane_result):
        plane_result.update(measured)
        return None
    monkeypatch.setattr('jdamr_cube_navigation.depth_box_parking.detect_table_edge', detect)
    detection, fields, stable = node._table_observation(None, None, 10., OPTICAL_FRAME)
    assert detection is None and stable is None
    assert fields == {'tabletop_plane': measured}
