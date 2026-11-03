"""Tests for the fixed-station reference model and lateral helpers."""

import json
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from jdamr_cube_navigation.rgbd_parking_reference import (
    PlanarBaseErrorEstimate,
    PlanarBasisEstimate,
    ReferencePoseEstimate,
)
from jdamr_cube_navigation import station_parking_reference as station


def test_live_goal_filter_rejects_one_jump_without_waiting_for_initial_frames():
    def sample(stamp, y):
        return {'valid': True, 'stamp_s': stamp, 'goal_odom': [.4, y, 0.]}

    first = sample(10., .03)
    assert station.filter_station_goal_observations([first], .5)['goal_odom'][1] == .03
    history = [sample(9., -.8), first, sample(10.2, .031), sample(10.4, .09)]
    filtered = station.filter_station_goal_observations(history + [history[-1]], .5)
    assert filtered['goal_odom'][1] == pytest.approx(.031)
    assert filtered['goal_filter_sample_count'] == 3
    assert filtered['goal_filter_xy_spread_m'] == pytest.approx(.059)
    assert filtered['goal_filter_yaw_spread_rad'] == 0.


def test_live_goal_filter_handles_yaw_wrap_and_omits_incomplete_map_frame():
    samples = [
        {'valid': True, 'stamp_s': 10., 'goal_odom': [.4, 0., math.pi - .01],
         'goal_map': [1., 0., 0.]},
        {'valid': True, 'stamp_s': 10.2, 'goal_odom': [.4, 0., -math.pi + .01]},
    ]
    filtered = station.filter_station_goal_observations(samples, .5)
    assert abs(abs(filtered['goal_odom'][2]) - math.pi) < 1e-9
    assert filtered['goal_filter_yaw_spread_rad'] == pytest.approx(.01)
    assert 'goal_map' not in filtered


def _reference_files(tmp_path):
    rgb = np.zeros((48, 64, 3), dtype=np.uint8)
    rgb[8:40, 4:28] = (30, 120, 210)
    rgb[8:40, 36:60] = (210, 80, 20)
    rgb_path = tmp_path / 'reference_rgb.ppm'
    assert cv2.imwrite(str(rgb_path), rgb)
    depth = np.full((24, 32), 1000, dtype=np.uint16)
    depth_path = tmp_path / 'reference_depth.npz'
    np.savez_compressed(depth_path, depth_raw_uint16=depth)
    rgb_info = {
        'k': [50., 0., 31.5, 0., 50., 23.5, 0., 0., 1.],
        'p': [50., 0., 31.5, 0., 0., 50., 23.5, 0., 0., 0., 1., 0.],
        'd': [0.] * 5, 'width': 64, 'height': 48, 'frame': 'optical',
    }
    depth_info = {
        'k': [25., 0., 15.75, 0., 25., 11.75, 0., 0., 1.],
        'p': [25., 0., 15.75, 0., 0., 25., 11.75, 0., 0., 0., 1., 0.],
        'd': [0.] * 5, 'width': 32, 'height': 24, 'frame': 'optical',
    }
    capture = {
        'captured_at': '2026-10-26T08:30:58.384428+00:00',
        'frame_counts': {'rgb': 23, 'depth': 15},
        'camera_info': {'rgb': rgb_info, 'depth': depth_info},
        'rgb': {'shape': [48, 64, 3], 'frame': 'optical', 'stamp': 10.0},
        'depth': {'shape': [24, 32], 'frame': 'optical', 'stamp': 10.01},
        'rgb_depth_delta_s': .01,
        'goal_binding': {
            'kind': station.CAPTURE_EQUALS_GOAL, 'robot_pose_equals_goal': True},
    }
    capture_path = tmp_path / 'reference_capture.json'
    capture_path.write_text(json.dumps(capture), encoding='utf-8')
    config = {
        'reference_capture_path': str(capture_path),
        'reference_capture_sha256': hashlib.sha256(capture_path.read_bytes()).hexdigest(),
        'reference_rgb_path': str(rgb_path),
        'reference_rgb_sha256': hashlib.sha256(rgb_path.read_bytes()).hexdigest(),
        'reference_depth_path': str(depth_path),
        'reference_depth_sha256': hashlib.sha256(depth_path.read_bytes()).hexdigest(),
        'reference_rgb_rois': {'cup': [4, 8, 28, 40], 'bottle': [36, 8, 60, 40]},
        'tabletop_depth_roi': [0, 0, 32, 24],
        'tabletop_depth_exclusion_rois': [],
        'camera_yaw_rad': 0.0,
        'camera_lever_xy_m': [-0.275, 0.0],
        'lateral_tolerance_m': 0.005,
        'lateral_precision_status': 'NOT_VALIDATED',
    }
    return config, rgb, rgb_info


@pytest.fixture
def reference_model(tmp_path, monkeypatch):
    config, rgb, rgb_info = _reference_files(tmp_path)
    basis = PlanarBasisEstimate(
        True, 'reference_tabletop_plane_candidate', np.eye(3),
        np.array((0., 0., 1.)), np.array((1., 0., 0.)),
        np.array((0., 1., 0.)), -1., 768, 760, .99, .001,
        0., (-.275, 0.))
    monkeypatch.setattr(
        station, 'estimate_reference_planar_basis', lambda *args, **kwargs: basis)

    class FakePreparedReference:
        constructions = 0

        def __init__(self, *args, **kwargs):
            type(self).constructions += 1
            self.observe_calls = 0
            self.result = ReferencePoseEstimate(False, 'not_observed')

        def observe(self, current_rgb, current_info):
            self.observe_calls += 1
            return self.result

    monkeypatch.setattr(station, 'PreparedRGBDReference', FakePreparedReference)
    model = station.TaughtStationReference(config)
    model._test_prepared_class = FakePreparedReference
    return model, rgb, rgb_info


def test_observe_composes_forward_left_and_yaw_into_odom(
        reference_model, monkeypatch):
    model, rgb, info = reference_model
    pose = ReferencePoseEstimate(
        True, 'observed', np.eye(4), np.eye(4), 42, 38, .4,
        (('bottle', 19), ('cup', 19)))
    error = PlanarBaseErrorEstimate(
        True, 'observed', .20, .10, .15, 0.,
        camera_yaw_rad=0., camera_lever_xy_m=(-.275, 0.))
    model.prepared_reference.result = pose
    monkeypatch.setattr(
        station, 'camera_reference_pose_to_planar_base_error',
        lambda *args, **kwargs: error)

    result = model.observe(rgb, info, (1., 2., math.pi / 2), 12.5)

    assert result['valid']
    assert result['goal_odom'] == pytest.approx((.9, 2.2, math.pi / 2 + .15))
    assert (result['forward_m'], result['left_m'], result['yaw_rad']) == pytest.approx(
        (.20, .10, .15))
    assert result['quality']['inlier_count'] == 38
    assert result['assumptions']['registered_lateral_tolerance_m'] == .005
    assert result['assumptions']['lateral_precision_status'] == 'NOT_VALIDATED'
    assert result['assumptions']['target_role'] == 'registered_reference_pose_not_object_centroid'
    assert result['assumptions']['reference_rgb_depth_delta_s'] == pytest.approx(.01)
    assert model.R_base_from_optical == pytest.approx(np.eye(3))


def test_observe_rejects_current_camera_identity_mismatch_before_matching(
        reference_model, monkeypatch):
    model, rgb, info = reference_model
    changed = dict(info)
    changed['k'] = list(info['k'])
    changed['k'][0] += 1.
    changed['p'] = list(info['p'])
    changed['p'][0] += 1.

    result = model.observe(rgb, changed, (0., 0., 0.), 1.)

    assert not result['valid']
    assert result['reason'] == 'current_camera_intrinsics_identity_mismatch'
    assert model.prepared_reference.observe_calls == 0


def test_observe_rejects_different_distortion_length_without_exception(
        reference_model, monkeypatch):
    model, rgb, info = reference_model
    changed = dict(info)
    changed['d'] = [0.] * 8

    result = model.observe(rgb, changed, (0., 0., 0.), 1.)

    assert not result['valid']
    assert result['reason'] == 'current_camera_intrinsics_identity_mismatch'
    assert model.prepared_reference.observe_calls == 0


def test_observe_reuses_prepared_reference_across_frames(reference_model):
    model, rgb, info = reference_model
    prepared = model.prepared_reference

    first = model.observe(rgb, info, (0., 0., 0.), 1.)
    second = model.observe(rgb, info, (0., 0., 0.), 2.)

    assert not first['valid'] and not second['valid']
    assert model.prepared_reference is prepared
    assert model._test_prepared_class.constructions == 1
    assert prepared.observe_calls == 2


def test_reference_loader_rejects_missing_file_and_capture_frame_mismatch(
        tmp_path, monkeypatch):
    config, _, _ = _reference_files(tmp_path)
    config['reference_rgb_path'] = str(tmp_path / 'missing.ppm')
    with pytest.raises(ValueError, match='reference_rgb_not_found'):
        station.TaughtStationReference(config)

    config, _, _ = _reference_files(tmp_path)
    capture_path = Path(config['reference_capture_path'])
    capture = json.loads(capture_path.read_text())
    capture['depth']['frame'] = 'other_optical'
    capture_path.write_text(json.dumps(capture), encoding='utf-8')
    config['reference_capture_sha256'] = hashlib.sha256(
        capture_path.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='reference_capture_frame_mismatch'):
        station.TaughtStationReference(config)


@pytest.mark.parametrize(('path_key', 'hash_key'), [
    ('reference_capture_path', 'reference_capture_sha256'),
    ('reference_rgb_path', 'reference_rgb_sha256'),
    ('reference_depth_path', 'reference_depth_sha256'),
])
def test_reference_loader_rejects_same_path_with_replaced_bytes(
        tmp_path, path_key, hash_key):
    config, _, _ = _reference_files(tmp_path)
    path = Path(config[path_key])
    original_size = path.stat().st_size
    payload = bytearray(path.read_bytes())
    payload[-1] ^= 1
    path.write_bytes(payload)
    assert path.stat().st_size == original_size

    with pytest.raises(ValueError, match=f'{hash_key}_mismatch'):
        station.TaughtStationReference(config)


def test_reference_loader_preserves_invalid_basis_reason(tmp_path, monkeypatch):
    config, _, _ = _reference_files(tmp_path)
    monkeypatch.setattr(
        station, 'estimate_reference_planar_basis',
        lambda *args, **kwargs: PlanarBasisEstimate(False, 'plane_missing'))
    with pytest.raises(ValueError, match='invalid_reference_planar_basis:plane_missing'):
        station.TaughtStationReference(config)


def test_reference_loader_rejects_inconsistent_timestamps(tmp_path):
    config, _, _ = _reference_files(tmp_path)
    capture_path = Path(config['reference_capture_path'])
    capture = json.loads(capture_path.read_text())
    capture['rgb_depth_delta_s'] = .2
    capture_path.write_text(json.dumps(capture), encoding='utf-8')
    config['reference_capture_sha256'] = hashlib.sha256(
        capture_path.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='invalid_reference_capture_timestamps'):
        station.TaughtStationReference(config)


def test_anchor_face_shifts_only_tangent_and_preserves_normal_gap():
    target = {
        'x_m': .40, 'y_m': .20,
        'face_center_map_xy_m': [.50, .20],
        'outward_normal_map_xy': [-1., 0.],
        'target_front_gap_m': .01,
    }

    anchored = station.anchor_face_to_taught_lateral(target, (.35, .05))

    assert anchored['x_m'] == pytest.approx(.40)
    assert anchored['y_m'] == pytest.approx(.05)
    assert anchored['face_center_map_xy_m'] == pytest.approx((.50, .05))
    assert anchored['outward_normal_map_xy'] == [-1., 0.]
    assert anchored['target_front_gap_m'] == .01
    assert anchored['lateral_shift_m'] == pytest.approx(.15)
    assert anchored['lateral_tolerance_m'] == .005
    assert anchored['lateral_precision_status'] == 'NOT_VALIDATED'
    assert target['y_m'] == .20


@pytest.mark.parametrize(('goal_y', 'sign'), [(.20, 1), (-.20, -1)])
def test_lateral_heading_sign_and_bound(goal_y, sign):
    result = station.lateral_approach_heading(
        0., (1., goal_y, 0.), (0., 0., 0.), .20, .005, .30)
    assert math.copysign(1., result) == sign
    assert abs(result) == pytest.approx(.30)


def test_lateral_heading_uses_minimum_lookahead_and_deadband():
    bounded = station.lateral_approach_heading(
        .4, (1., .01, 0.), (0., 0., 0.), 0., .005, .5,
        minimum_lookahead_m=.10)
    left_error = -math.sin(.4) * 1. + math.cos(.4) * .01
    current_heading_error = -.4
    expected = math.atan(
        2. * left_error / .10 - .25 * math.tan(current_heading_error))
    expected = max(-.5, min(.5, expected))
    assert bounded == pytest.approx(.4 + expected)

    parallelizing = station.lateral_approach_heading(
        .4, (1., 0., 0.), (0., 0., 0.), .20, 2., .5)
    expected = .4 + math.atan(-.25 * math.tan(-.4))
    assert parallelizing == pytest.approx(expected)
    assert (parallelizing - .4) * (0. - .4) < 0.
    assert abs(parallelizing - .4) < abs(0. - .4)


def test_lateral_heading_blends_current_yaw_toward_parallel_arrival():
    current_yaw = math.radians(16.)

    result = station.lateral_approach_heading(
        0., (.4, 0., 0.), (0., 0., current_yaw), .4, .005,
        math.radians(30.))

    assert result * current_yaw < 0.
    assert abs(result) < abs(current_yaw)


@pytest.mark.parametrize(('start_y', 'start_yaw'), [
    (-.04, math.radians(8.)),
    (.04, math.radians(-8.)),
    (-.06, 0.),
])
def test_lateral_heading_rollout_reaches_tolerance_and_parallel_heading(
        start_y, start_yaw):
    """A bounded forward rollout converges without a terminal turn-back."""
    goal = (.4, 0., 0.)
    x, y, yaw = 0., start_y, start_yaw
    previous_abs_y = abs(y)
    for _ in range(78):
        remaining = max(0., goal[0] - x)
        command = station.lateral_approach_heading(
            0., goal, (x, y, yaw), remaining, .005,
            math.radians(30.))
        yaw = math.atan2(
            math.sin(yaw + .5 * station._wrapped(command - yaw)),
            math.cos(yaw + .5 * station._wrapped(command - yaw)))
        x += .005 * math.cos(yaw)
        y += .005 * math.sin(yaw)
        if abs(y) > .005:
            assert abs(y) <= previous_abs_y + 1e-12
        previous_abs_y = abs(y)

    assert abs(y) <= .005
    assert abs(yaw) <= math.radians(.2)


@pytest.mark.parametrize(('field', 'value', 'message'), [
    ('remaining_m', -1., 'nonnegative'),
    ('tolerance_m', 0., 'positive'),
    ('max_heading_rad', math.inf, 'finite'),
    ('max_heading_rad', 4., 'out_of_bounds'),
])
def test_lateral_heading_rejects_invalid_bounds(field, value, message):
    values = dict(face_heading=0., goal_odom=(1., 0., 0.),
                  actual_odom=(0., 0., 0.), remaining_m=.2,
                  tolerance_m=.005, max_heading_rad=.3)
    values[field] = value
    with pytest.raises(ValueError, match=message):
        station.lateral_approach_heading(**values)
