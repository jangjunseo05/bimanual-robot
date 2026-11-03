"""Tests for live tabletop projection in the tape reference detector."""

from types import SimpleNamespace

import cv2

from jdamr_cube_navigation.table_tape_reference import TableTapeReference

import numpy as np

import pytest


WIDTH = 200
HEIGHT = 120
LIVE_K = np.array([
    [500., 0., 100.],
    [0., 500., 60.],
    [0., 0., 1.],
])
LIVE_INFO = {
    'k': LIVE_K.reshape(-1),
    'width': WIDTH,
    'height': HEIGHT,
    'frame': 'camera_color_optical_frame',
}


def _basis():
    # Deliberately stale: its upward rays cannot reach the saved table plane.
    return SimpleNamespace(
        R_base_from_optical=np.eye(3),
        plane_offset_m=1.,
        camera_lever_xy_m=(9., 9.),
        basis_source='stale_reference_tabletop',
        camera_yaw_rad=.7,
    )


def _config():
    return {
        'spacing_m': .12,
        'width_m': .022,
        'goal_front_from_base_m': .10,
        'spacing_tolerance_m': .002,
        'width_tolerance_m': .001,
        'maximum_yaw_rad': .1,
        'maximum_lateral_m': .20,
    }


def _image(top=50):
    image = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)
    cv2.rectangle(image, (65, top), (75, 105), (0, 0, 0), -1)
    cv2.rectangle(image, (125, top), (135, 105), (0, 0, 0), -1)
    return image


def _surface(**updates):
    value = {
        'plane_normal_optical': [0., 0., 1.],
        'plane_point_optical_m': [0., 0., 1.],
        'R_base_from_optical': [
            [0., 1., 0.],
            [-1., 0., 0.],
            [0., 0., 1.],
        ],
        'camera_lever_xy_m': [.4, .05],
        'stamp_s': 41.25,
        'rgb_stamp_s': 41.27,
        'source': 'live_depth_table_plane',
    }
    value.update(updates)
    return value


def _detector():
    old_info = {
        'k': np.array([
            [100., 0., 100.],
            [0., 100., 60.],
            [0., 0., 1.],
        ]).reshape(-1),
        'width': WIDTH,
        'height': HEIGHT,
    }
    return TableTapeReference(old_info, _basis(), _config())


@pytest.mark.parametrize('offset_m, valid', [(.30, True), (-.30, True),
                                            (.05, True), (.47, False)])
def test_default_tape_acquisition_covers_displaced_arrival(offset_m, valid):
    config = _config()
    config.pop('maximum_lateral_m')
    detector = TableTapeReference(LIVE_INFO, _basis(), config)
    estimate, quality = detector.observe(
        _image(), surface=_surface(camera_lever_xy_m=[.4, offset_m]),
        rgb_info=LIVE_INFO)
    assert estimate.valid is valid
    if valid:
        assert estimate.left_m == pytest.approx(offset_m)
        assert quality['tape_spacing_m'] == pytest.approx(.12)
    else:
        assert quality['lateral_rejected_candidates']


def test_explicit_tape_acquisition_limit_still_applies():
    detector = _detector()
    estimate, quality = detector.observe(
        _image(), surface=_surface(camera_lever_xy_m=[.4, .30]),
        rgb_info=LIVE_INFO)
    assert not estimate.valid
    assert len(quality['lateral_rejected_candidates']) == 2


@pytest.mark.parametrize('limit', [True, 0., -1., float('nan'), float('inf')])
def test_invalid_tape_acquisition_limit_is_rejected(limit):
    with pytest.raises(ValueError, match='invalid_tape_maximum_lateral_m'):
        TableTapeReference(LIVE_INFO, _basis(), {**_config(), 'maximum_lateral_m': limit})


def test_live_plane_and_rgb_intrinsics_override_stale_saved_basis():
    """Live geometry and current RGB K must replace stale calibration."""
    detector = _detector()

    estimate, quality = detector.observe(
        _image(), surface=_surface(), rgb_info=LIVE_INFO)

    assert estimate.valid, estimate.reason
    assert estimate.forward_m == pytest.approx(.28)
    assert estimate.left_m == pytest.approx(.05)
    assert estimate.yaw_rad == pytest.approx(0.)
    assert estimate.basis_source == 'live_depth_table_plane'
    assert estimate.camera_lever_xy_m == pytest.approx([.4, .05])
    assert quality['tape_spacing_m'] == pytest.approx(.12)
    assert quality['tape_center_base_xy_m'] == pytest.approx([.38, .05])
    assert quality['projection_basis'] == 'live_depth_table_plane'
    assert quality['depth_measurement_stamp_s'] == pytest.approx(41.25)
    assert quality['rgb_stamp_s'] == pytest.approx(41.27)
    assert quality['rgb_depth_delta_s'] == pytest.approx(.02)


def test_live_projection_handles_changed_camera_height_and_tilt():
    """A general optical plane works after camera height and tilt change."""
    angle = np.deg2rad(23.)
    cosine, sine = np.cos(angle), np.sin(angle)
    normal = np.array([0., sine, cosine])
    rotation = np.array([
        [1., 0., 0.],
        [0., cosine, sine],
        [0., -sine, cosine],
    ])
    surface = _surface(
        plane_normal_optical=normal,
        plane_point_optical_m=[0., .18, 1.35],
        R_base_from_optical=rotation,
        camera_lever_xy_m=[-.08, .11],
    )
    projection = TableTapeReference._live_projection(
        surface, np.linalg.inv(LIVE_K))

    point = TableTapeReference._live_point(130., 46., projection)

    ray = np.linalg.inv(LIVE_K) @ np.array([130., 46., 1.])
    scale = normal @ np.array([0., .18, 1.35]) / (normal @ ray)
    expected = (rotation @ (scale * ray))[:2] + np.array([-.08, .11])
    assert point == pytest.approx(expected)


@pytest.mark.parametrize('updates', [
    {'plane_normal_optical': [0., 0., 2.]},
    {'plane_point_optical_m': [0., np.nan, 1.]},
    {'R_base_from_optical': np.diag([1., 1., 2.])},
    {'camera_lever_xy_m': [0.]},
    {'stamp_s': np.inf},
    {'source': 'saved_table_plane'},
])
def test_invalid_live_surface_is_rejected_without_legacy_fallback(updates):
    """Malformed metric geometry cannot fall back to the saved basis."""
    estimate, quality = _detector().observe(
        _image(), surface=_surface(**updates), rgb_info=LIVE_INFO)

    assert not estimate.valid
    assert estimate.reason == 'invalid_tape_surface'
    assert quality['candidates'] == []


@pytest.mark.parametrize('surface,image', [
    (_surface(plane_normal_optical=[0., 1., 0.],
              plane_point_optical_m=[0., 1., 0.]), _image(top=60)),
    (_surface(plane_point_optical_m=[0., 0., -1.]), _image()),
])
def test_parallel_or_behind_live_plane_produces_invalid_observation(
        surface, image):
    """Rays that cannot meet the live plane never produce tape points."""
    estimate, quality = _detector().observe(
        image, surface=surface, rgb_info=LIVE_INFO)

    assert not estimate.valid
    assert estimate.reason == 'tape_pair_missing'
    assert quality['projection_basis'] == 'live_depth_table_plane'
    assert quality['candidates'] == []


def test_rgb_intrinsics_override_requires_current_image_dimensions():
    """Frame-specific intrinsics must describe the observed image size."""
    bad_info = dict(LIVE_INFO, width=WIDTH // 2)

    estimate, quality = _detector().observe(
        _image(), surface=_surface(), rgb_info=bad_info)

    assert not estimate.valid
    assert estimate.reason == 'invalid_tape_rgb_info_dimensions'
    assert quality['candidates'] == []
