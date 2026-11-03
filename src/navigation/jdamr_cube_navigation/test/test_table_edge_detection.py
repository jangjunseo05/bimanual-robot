"""Synthetic projection tests for table-top front-edge detection."""

from dataclasses import replace
import json
import math
from pathlib import Path

from jdamr_cube_navigation.box_top_detection import (
    CameraIntrinsics,
    DetectionStability,
)
from jdamr_cube_navigation.table_edge_detection import (
    _has_depth_discontinuity_below,
    _horizontal_plane_is_valid,
    _optical_to_level_rotation,
    _visible_front_edge,
    detect_table_edge,
    TableEdgeConfig,
    TableEdgeDetection,
)

import numpy as np

import pytest


INTRINSICS = CameraIntrinsics(
    width=320,
    height=240,
    focal_x_px=285.1711,
    focal_y_px=285.1711,
    center_x_px=159.5,
    center_y_px=119.5,
)


def config(camera_height_m=1.0, pitch_deg=27.0, roll_deg=0.0):
    """Return a compact table detector configuration for synthetic images."""
    return TableEdgeConfig(
        camera_height_m=camera_height_m,
        camera_pitch_rad=math.radians(pitch_deg),
        camera_roll_rad=math.radians(roll_deg),
        minimum_depth_m=0.30,
        maximum_depth_m=2.0,
        pixel_step=2,
        plane_distance_m=0.015,
        minimum_inliers=80,
        ransac_iterations=100,
        minimum_edge_length_m=0.25,
        maximum_edge_rms_m=0.01,
    )


def _render_table(
        configured, *, table_height_m=0.75, front_x_m=0.58,
        width_m=0.80, depth_m=0.75, yaw_deg=0.0,
        invalid_fraction=0.0, seed=4, background_height_m=None,
        background_depth_m=2.5):
    """Ray-cast a finite horizontal tabletop over a distant background."""
    rows, columns = np.mgrid[0:INTRINSICS.height, 0:INTRINSICS.width]
    rays_optical = np.stack((
        (columns - INTRINSICS.center_x_px) / INTRINSICS.focal_x_px,
        (rows - INTRINSICS.center_y_px) / INTRINSICS.focal_y_px,
        np.ones_like(rows, dtype=np.float64),
    ), axis=-1)
    rotation = _optical_to_level_rotation(configured)
    rays_level = rays_optical @ rotation.T
    relative_height_m = table_height_m - configured.camera_height_m
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = relative_height_m / rays_level[..., 2]
    points = rays_level * scale[..., None]
    yaw = math.radians(yaw_deg)
    inward = np.array([math.cos(yaw), math.sin(yaw)])
    edge_basis = np.array([-math.sin(yaw), math.cos(yaw)])
    front_center = inward * front_x_m
    offset = points[..., :2] - front_center
    along_depth = offset @ inward
    along_edge = offset @ edge_basis
    inside = np.isfinite(scale) & (scale > 0.0)
    inside &= (along_depth >= 0.0) & (along_depth <= depth_m)
    inside &= np.abs(along_edge) <= width_m / 2.0
    if background_height_m is None:
        depth = np.full(
            rows.shape, round(background_depth_m * 1000.0),
            dtype=np.uint16)
    else:
        background_relative_m = (
            background_height_m - configured.camera_height_m)
        with np.errstate(divide='ignore', invalid='ignore'):
            background_scale = background_relative_m / rays_level[..., 2]
        background_scale = np.where(background_scale > 0.0,
                                    background_scale, 2.5)
        depth = np.rint(np.clip(background_scale, 0.0, 65.0) * 1000.0).astype(
            np.uint16)
    optical_depth_m = scale
    usable = inside & (optical_depth_m > 0.0)
    depth[usable] = np.rint(optical_depth_m[usable] * 1000.0).astype(
        np.uint16)
    if invalid_fraction:
        random = np.random.default_rng(seed)
        invalid = random.random(depth.shape) < invalid_fraction
        depth[invalid & usable] = 0
        # Sparse invalid background samples must not erase the whole edge.
        depth[invalid & ~usable] = 0
    return depth, front_center, edge_basis


def _to_level(detection, configured):
    endpoints = np.asarray(detection.edge_endpoints_optical_m)
    return endpoints @ _optical_to_level_rotation(configured).T


def test_visible_edge_endpoints_remain_on_tilted_fitted_plane(monkeypatch):
    """Edge endpoints preserve x/y plane slope instead of using median z."""
    configured = config(pitch_deg=0.0)
    rotation = _optical_to_level_rotation(configured)
    plane_normal = np.array([-0.08, 0.12, 1.0])
    plane_normal /= np.linalg.norm(plane_normal)
    plane_offset = -0.72 * plane_normal[2]
    level_xy = np.column_stack((
        np.linspace(0.45, 1.05, 32),
        np.linspace(-0.25, 0.25, 32),
    ))
    level_xy = np.repeat(level_xy, 4, axis=0)
    level_xy[:, 0] += np.tile([.03, .02, .01, 0.0], 32)
    level_z = -(plane_normal[0] * level_xy[:, 0]
                + plane_normal[1] * level_xy[:, 1]
                + plane_offset) / plane_normal[2]
    points_level = np.column_stack((level_xy, level_z))
    points_optical = points_level @ rotation
    columns = np.repeat(np.arange(32), 4)
    rows = np.tile([14, 16, 18, 20], 32)
    plane_mask = np.ones(128, dtype=bool)
    depth_mm = np.full((80, 32), 1000, dtype=np.uint16)
    depth_mm[20, :] = np.rint(points_optical[3::4, 2] * 1000).astype(np.uint16)
    monkeypatch.setattr(
        'jdamr_cube_navigation.table_edge_detection.'
        '_has_depth_discontinuity_below',
        lambda *args: True,
    )

    result = _visible_front_edge(
        depth_mm, points_optical, points_level, columns, rows, plane_mask,
        configured, INTRINSICS, rotation, plane_normal, plane_offset)

    assert result is not None
    endpoints_optical, _, _ = result
    endpoints_level = endpoints_optical @ rotation.T
    residuals = endpoints_level @ plane_normal + plane_offset
    assert residuals == pytest.approx([0.0, 0.0], abs=1e-12)


@pytest.mark.parametrize('angle_deg', [-0.7, 89.5, -89.5])
def test_stability_accepts_reversed_table_edge_endpoints(angle_deg):
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    angle_rad = math.radians(angle_deg)
    reversed_angle_rad = math.atan2(
        math.sin(angle_rad + math.pi), math.cos(angle_rad + math.pi))
    observations = (
        replace(detection, edge_angle_rad=angle_rad),
        replace(detection, edge_angle_rad=reversed_angle_rad,
                edge_endpoints_optical_m=tuple(
                    reversed(detection.edge_endpoints_optical_m))),
    )
    tracker = DetectionStability(required_frames=8)

    results = [tracker.update(observations[index % 2]) for index in range(8)]

    assert results == [False] * 7 + [True]
    assert tracker.frame_count == 8


def test_stability_rejects_changed_table_edge_direction():
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    changed = replace(
        detection, edge_angle_rad=detection.edge_angle_rad + math.radians(6.0))
    tracker = DetectionStability(required_frames=8)

    results = [tracker.update((detection, changed)[index % 2])
               for index in range(8)]

    assert results == [False] * 8
    assert tracker.frame_count == 1


def test_stability_rejects_three_dimensional_line_tilt_with_same_xz_heading():
    # Rotation about the optical Z axis changes a line's Y component while its
    # projected X-Z heading and closest point can remain identical.
    def observation(angle):
        tangent = np.array((math.cos(angle), math.sin(angle), 0.0))
        center = np.array((0.0, 0.0, 0.5))
        return TableEdgeDetection(
            center_x_m=0.0, center_y_m=0.0, front_distance_m=0.5,
            edge_angle_rad=0.0, confidence=0.9, inlier_count=100,
            plane_normal=(0.0, -1.0, 0.0), table_height_m=0.75,
            width_m=0.4, depth_extent_m=0.5, plane_rms_m=0.001,
            candidate_count=100, surface_kind='table_edge', edge_rms_m=0.001,
            edge_endpoints_optical_m=tuple(tuple(p) for p in (
                center - 0.2 * tangent, center + 0.2 * tangent)))

    tracker = DetectionStability(required_frames=8)
    frames = (observation(0.0), observation(math.radians(6.0)))
    assert [tracker.update(frames[i % 2]) for i in range(8)] == [False] * 8


@pytest.mark.parametrize('perpendicular_shift_m,stable', [(0.0, True), (0.06, False)])
def test_stability_tracks_line_offset_instead_of_visible_segment_midpoint(
        perpendicular_shift_m, stable):
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    endpoints = np.asarray(detection.edge_endpoints_optical_m)
    tangent = endpoints[1] - endpoints[0]
    tangent /= np.linalg.norm(tangent)
    normal = np.cross(tangent, (0.0, 1.0, 0.0))
    normal /= np.linalg.norm(normal)
    shifted = endpoints + tangent * 0.10 + normal * perpendicular_shift_m
    center = shifted.mean(axis=0)
    changed = replace(detection, center_x_m=center[0], center_y_m=center[1],
                      front_distance_m=center[2],
                      edge_endpoints_optical_m=tuple(tuple(p) for p in shifted))
    tracker = DetectionStability(required_frames=8)
    results = [tracker.update((detection, changed)[i % 2]) for i in range(8)]
    assert results == [False] * 7 + [stable]


def test_recorded_stationary_water_face_stabilizes_despite_segment_cropping():
    fixture = json.loads((Path(__file__).parent / 'fixtures/table_edge_stability_20261016.json')
                         .read_text())
    tracker = DetectionStability(required_frames=8)
    results = []
    for frame in fixture['observations']:
        detection = TableEdgeDetection(
            center_x_m=frame['lateral_error_m'], center_y_m=frame['vertical_position_m'],
            front_distance_m=frame['front_distance_m'], width_m=frame['width_m'],
            depth_extent_m=frame['depth_extent_m'],
            edge_angle_rad=math.radians(frame['edge_angle_deg']),
            plane_normal=tuple(frame['plane_normal']), plane_rms_m=frame['plane_rms_m'],
            inlier_count=frame['inlier_count'], candidate_count=frame['candidate_count'],
            confidence=frame['confidence'], surface_kind='table_edge',
            edge_endpoints_optical_m=tuple(tuple(p) for p in frame['edge_endpoints_optical_m']),
            table_height_m=frame['table_height_m'], edge_rms_m=frame['edge_rms_m'])
        results.append(tracker.update(detection))
    assert results == [False] * 7 + [True] * 9


def test_stability_uses_table_edge_geometry_when_fitted_plane_tilt_varies():
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    rotation = _optical_to_level_rotation(configured)
    normals = [tuple(np.array([math.sin(angle), 0.0, math.cos(angle)]) @ rotation)
               for angle in (math.radians(-5.0), math.radians(5.0))]
    tracker = DetectionStability(required_frames=8)

    results = [tracker.update(replace(detection, plane_normal=normals[index % 2]))
               for index in range(8)]

    assert results == [False] * 7 + [True]


@pytest.mark.parametrize('field', ['center_x_m', 'center_y_m', 'front_distance_m'])
def test_stability_rejects_changed_table_edge_position(field):
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    changed = replace(detection, **{field: getattr(detection, field) + 0.06})
    tracker = DetectionStability(required_frames=8)

    assert [tracker.update((detection, changed)[index % 2])
            for index in range(8)] == [False] * 8


@pytest.mark.parametrize('normal', [(0.0, 0.0, 0.0), (math.nan, 0.0, 1.0)])
def test_stability_rejects_invalid_table_plane_normal(normal):
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    tracker = DetectionStability(required_frames=8)

    assert [tracker.update(replace(detection, plane_normal=normal))
            for _ in range(8)] == [False] * 8


@pytest.mark.parametrize(
    ('camera_height_m', 'pitch_deg', 'roll_deg', 'table_yaw_deg'),
    [
        (0.92, 24.0, 0.0, 0.0),
        (1.00, 27.0, 2.0, 12.0),
        (1.10, 32.0, -3.0, -15.0),
    ],
)
def test_recovers_visible_front_edge_across_mount_geometry(
        camera_height_m, pitch_deg, roll_deg, table_yaw_deg):
    """Camera pose changes preserve the recovered physical front line."""
    configured = config(camera_height_m, pitch_deg, roll_deg)
    depth, front_center, edge_basis = _render_table(
        configured, yaw_deg=table_yaw_deg, background_depth_m=1.8)

    detection = detect_table_edge(depth, INTRINSICS, configured)

    assert detection is not None
    assert detection.surface_kind == 'table_edge'
    assert detection.table_height_m == pytest.approx(0.75, abs=0.015)
    endpoints = _to_level(detection, configured)
    detected_center = np.mean(endpoints[:, :2], axis=0)
    inward = np.array([edge_basis[1], -edge_basis[0]])
    assert abs(float((detected_center - front_center) @ inward)) < 0.025
    detected_basis = endpoints[1, :2] - endpoints[0, :2]
    detected_basis /= np.linalg.norm(detected_basis)
    assert abs(float(detected_basis @ edge_basis)) > 0.995
    assert detection.width_m >= configured.minimum_edge_length_m
    assert detection.edge_rms_m <= configured.maximum_edge_rms_m
    assert detection.center_x_m == pytest.approx(
        np.mean(np.asarray(detection.edge_endpoints_optical_m), axis=0)[0])
    assert detection.center_y_m == pytest.approx(
        np.mean(np.asarray(detection.edge_endpoints_optical_m), axis=0)[1])
    assert detection.front_distance_m == pytest.approx(
        np.mean(np.asarray(detection.edge_endpoints_optical_m), axis=0)[2])


def test_tolerates_sparse_invalid_depth_without_using_a_side_face():
    """Sparse missing pixels do not require a visible thin side face."""
    configured = config()
    depth, front_center, _ = _render_table(
        configured, invalid_fraction=0.12, background_depth_m=1.8)

    detection = detect_table_edge(depth, INTRINSICS, configured)

    assert detection is not None
    endpoints = _to_level(detection, configured)
    assert np.mean(endpoints[:, :2], axis=0) == pytest.approx(
        front_center, abs=0.03)


def test_current_derived_mount_sees_near_table_edge_above_astra_minimum():
    """The current derived mount observes a near edge above 0.4 metres."""
    configured = TableEdgeConfig(
        camera_height_m=0.9070543261338818,
        camera_roll_rad=0.011934068034722573,
        camera_pitch_rad=0.46681472099722787,
        minimum_inliers=80,
        plane_distance_m=0.015,
        ransac_iterations=100,
    )
    depth, front_center, edge_basis = _render_table(
        configured, front_x_m=0.42, width_m=0.70, depth_m=0.70,
        background_depth_m=1.8)

    detection = detect_table_edge(depth, INTRINSICS, configured)

    assert detection is not None
    endpoints = _to_level(detection, configured)
    detected_center = np.mean(endpoints[:, :2], axis=0)
    inward = np.array([edge_basis[1], -edge_basis[0]])
    assert abs(float((detected_center - front_center) @ inward)) < 0.025
    assert detection.front_distance_m >= configured.minimum_depth_m


def test_rejects_vertical_wall_and_floor():
    """A wall or floor cannot pass the bounded tabletop-height gate."""
    configured = config()
    wall = np.full((INTRINSICS.height, INTRINSICS.width), 900,
                   dtype=np.uint16)
    floor, _, _ = _render_table(
        configured, table_height_m=0.0, front_x_m=0.1,
        width_m=3.0, depth_m=3.0)

    assert detect_table_edge(wall, INTRINSICS, configured) is None
    assert detect_table_edge(floor, INTRINSICS, configured) is None


def test_table_mode_rejects_a_thirty_degree_sloped_plane():
    """Table mode uses a stricter horizontal gate than generic box tops."""
    configured = config()
    normal = np.array([0.5, 0.0, math.cos(math.radians(30.0))])
    points = np.array([
        [0.5, -0.2, -0.25],
        [0.6, 0.0, -0.25],
        [0.7, 0.2, -0.25],
    ])
    assert configured.minimum_normal_y == pytest.approx(0.90)
    assert not _horizontal_plane_is_valid(normal, points, configured)


def test_rejects_image_boundary_instead_of_calling_it_a_table_edge():
    """An image crop without a depth discontinuity is not a table edge."""
    configured = config()
    # The physical front lies below the image. Only the top surface is visible.
    depth, _, _ = _render_table(
        configured, front_x_m=0.05, width_m=2.0, depth_m=2.0)
    assert detect_table_edge(depth, INTRINSICS, configured) is None


def test_visible_tabletop_reports_plane_when_front_edge_is_out_of_view():
    """A valid current tabletop fit survives a missing visible front edge."""
    configured = config()
    depth, _, _ = _render_table(
        configured, front_x_m=0.05, width_m=2.0, depth_m=2.0)
    plane_result = {'source': 'stale_previous_frame'}

    detection = detect_table_edge(
        depth, INTRINSICS, configured, plane_result=plane_result)

    assert detection is None
    assert plane_result['source'] == 'current_depth_tabletop_fit'
    assert plane_result['inlier_count'] >= configured.minimum_inliers
    assert plane_result['candidate_count'] >= plane_result['inlier_count']
    assert plane_result['plane_rms_m'] <= configured.plane_distance_m
    assert plane_result['table_height_m'] == pytest.approx(.75, abs=.015)
    normal_optical = np.asarray(plane_result['plane_normal_optical'])
    assert np.linalg.norm(normal_optical) == pytest.approx(1.)
    expected_normal = (
        _optical_to_level_rotation(configured).T @ np.array([0., 0., 1.]))
    assert normal_optical @ expected_normal > .99
    point_optical = np.asarray(plane_result['plane_point_optical_m'])
    point_level = (
        _optical_to_level_rotation(configured) @ point_optical)
    assert configured.camera_height_m + point_level[2] == pytest.approx(
        plane_result['table_height_m'])


def test_plane_output_does_not_change_visible_edge_detection():
    """Requesting plane provenance preserves the existing edge result."""
    configured = config()
    depth, _, _ = _render_table(configured, background_depth_m=1.8)
    plane_result = {}

    baseline = detect_table_edge(depth, INTRINSICS, configured)
    with_plane = detect_table_edge(
        depth, INTRINSICS, configured, plane_result=plane_result)

    assert baseline is not None
    assert with_plane == baseline
    assert plane_result['source'] == 'current_depth_tabletop_fit'
    assert plane_result['inlier_count'] == baseline.inlier_count
    assert plane_result['candidate_count'] == baseline.candidate_count
    assert plane_result['plane_rms_m'] == pytest.approx(baseline.plane_rms_m)


@pytest.mark.parametrize('invalid_kind', ['floor', 'no_inliers'])
def test_invalid_depth_clears_plane_result(invalid_kind):
    """A failed plane fit cannot leak a result from an earlier frame."""
    configured = config()
    if invalid_kind == 'floor':
        depth, _, _ = _render_table(
            configured, table_height_m=0.0, front_x_m=0.1,
            width_m=3.0, depth_m=3.0)
    else:
        depth = np.zeros(
            (INTRINSICS.height, INTRINSICS.width), dtype=np.uint16)
    plane_result = {'source': 'stale_previous_frame', 'inlier_count': 999}

    detection = detect_table_edge(
        depth, INTRINSICS, configured, plane_result=plane_result)

    assert detection is None
    assert plane_result == {}


def test_near_edge_uses_later_measured_point_below_the_apron():
    configured = replace(config(1.26, 36.64), plane_distance_m=.025)
    rotation = _optical_to_level_rotation(configured)
    depth = np.zeros((240, 320), dtype=np.uint16)
    column, row = 160, 210

    def ray(at_row):
        return rotation @ np.array([
            (column - INTRINSICS.center_x_px) / INTRINSICS.focal_x_px,
            (at_row - INTRINSICS.center_y_px) / INTRINSICS.focal_y_px, 1.0])

    for at_row in (row, row + 2):
        depth[at_row, column] = round(.36 / ray(at_row)[0] * 1000)
    depth[230, column] = round((.72 - 1.26) / ray(230)[2] * 1000)
    assert _has_depth_discontinuity_below(
        depth, row, column, INTRINSICS, rotation,
        np.array([0., 0., 1.]), 1.26 - .784, configured)


def test_near_table_front_with_vertical_apron_remains_detectable():
    configured = replace(config(1.26, 36.64), plane_distance_m=.025)
    depth, front, basis = _render_table(
        configured, table_height_m=.784, front_x_m=.36, background_depth_m=1.5)
    rows, columns = np.mgrid[:INTRINSICS.height, :INTRINSICS.width]
    rays = np.stack(((columns - INTRINSICS.center_x_px) / INTRINSICS.focal_x_px,
                     (rows - INTRINSICS.center_y_px) / INTRINSICS.focal_y_px,
                     np.ones_like(rows)), axis=-1)
    rays = rays @ _optical_to_level_rotation(configured).T
    scale = .36 / rays[..., 0]
    heights = 1.26 + scale * rays[..., 2]
    apron = ((heights >= .724) & (heights <= .784)
             & (np.abs(scale * rays[..., 1]) <= .4)
             & (scale > 0) & (scale < depth * .001))
    depth[apron] = np.rint(scale[apron] * 1000).astype(np.uint16)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    center = np.mean(_to_level(detection, configured)[:, :2], axis=0)
    inward = np.array([basis[1], -basis[0]])
    assert abs(float((center - front) @ inward)) < .01


def test_rejects_minimum_depth_clipping_as_an_edge():
    """The configured near-range cutoff cannot create a false edge."""
    configured = replace(
        config(), minimum_depth_m=0.72, minimum_edge_length_m=0.18)
    depth, _, _ = _render_table(
        configured, front_x_m=0.05, width_m=2.0, depth_m=2.0)
    assert detect_table_edge(depth, INTRINSICS, configured) is None


@pytest.mark.parametrize('front_x_m', [0.35, 0.37])
def test_default_near_range_preserves_a_measured_table_front(front_x_m):
    """A visible edge near 1--3 cm chassis clearance survives the depth filter."""
    configured = replace(
        config(0.907, 15.0),
        minimum_depth_m=TableEdgeConfig().minimum_depth_m,
        plane_distance_m=0.025)
    depth, _, _ = _render_table(
        configured, front_x_m=front_x_m, background_depth_m=1.5)
    detection = detect_table_edge(depth, INTRINSICS, configured)
    assert detection is not None
    assert 0.375 < detection.front_distance_m < 0.425
    assert detect_table_edge(
        depth, INTRINSICS, replace(configured, minimum_depth_m=0.40)) is None


@pytest.mark.parametrize('stripe_row', [140, 160, 180, 200, 220])
def test_rejects_invalid_depth_sentinel_as_edge_evidence(stripe_row):
    """A 65535 no-return stripe cannot manufacture a front edge."""
    configured = config()
    depth, _, _ = _render_table(
        configured, front_x_m=0.05, width_m=2.0, depth_m=2.0)
    depth[stripe_row:, :] = np.iinfo(np.uint16).max
    assert detect_table_edge(depth, INTRINSICS, configured) is None


def test_rejects_out_of_range_background_as_edge_evidence():
    """Depth beyond the configured maximum is not a measured background."""
    configured = config()
    depth, _, _ = _render_table(
        configured,
        background_depth_m=configured.maximum_background_depth_m + 0.5)
    assert detect_table_edge(depth, INTRINSICS, configured) is None


def test_candidate_roi_does_not_shrink_valid_background_sensor_range():
    """A 0.9m table ROI can use valid 1.8m background evidence."""
    configured = replace(config(), maximum_depth_m=0.9)
    depth, front_center, edge_basis = _render_table(
        configured, front_x_m=0.58, background_depth_m=1.8)

    detection = detect_table_edge(depth, INTRINSICS, configured)

    assert detection is not None
    endpoints = _to_level(detection, configured)
    detected_center = np.mean(endpoints[:, :2], axis=0)
    inward = np.array([edge_basis[1], -edge_basis[0]])
    assert abs(float((detected_center - front_center) @ inward)) < 0.025
    assert detection.front_distance_m < configured.maximum_depth_m
    assert 1.8 < configured.maximum_background_depth_m


def test_rejects_same_plane_gradient_at_the_astra_near_range():
    """A clipped infinite tabletop cannot become a false interior edge."""
    configured = replace(
        config(0.8, 5.0), pixel_step=4, minimum_depth_m=0.4)
    depth, _, _ = _render_table(
        configured, front_x_m=0.05, width_m=0.80, depth_m=0.75)
    assert detect_table_edge(depth, INTRINSICS, configured) is None


@pytest.mark.parametrize(
    ('camera_height_m', 'pitch_deg', 'roll_deg'),
    [(0.92, 24.0, 0.0), (1.0, 27.0, 2.0), (1.1, 32.0, -3.0)],
)
def test_rejects_when_floor_beyond_edge_is_outside_sensor_range(
        camera_height_m, pitch_deg, roll_deg):
    """An out-of-range floor cannot substantiate a visible table edge."""
    configured = config(camera_height_m, pitch_deg, roll_deg)
    depth, _, _ = _render_table(
        configured, background_height_m=0.0)

    detection = detect_table_edge(depth, INTRINSICS, configured)

    assert detection is None


def test_requires_a_positive_camera_height_calibration():
    """Unknown camera height disables metric table-height detection."""
    depth = np.full((INTRINSICS.height, INTRINSICS.width), 1000,
                    dtype=np.uint16)
    with pytest.raises(ValueError, match='camera_height_m'):
        detect_table_edge(depth, INTRINSICS, TableEdgeConfig())


@pytest.mark.parametrize('background_max', [math.nan, 0.8])
def test_rejects_invalid_background_sensor_range(background_max):
    """The background sensor ceiling must cover the candidate ROI."""
    configured = replace(
        config(), maximum_depth_m=0.9,
        maximum_background_depth_m=background_max)
    depth = np.full((INTRINSICS.height, INTRINSICS.width), 1000,
                    dtype=np.uint16)
    with pytest.raises(ValueError, match='maximum_background_depth_m'):
        detect_table_edge(depth, INTRINSICS, configured)
