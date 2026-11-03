"""Tests for table-edge docking target geometry."""

from copy import deepcopy
import math
from pathlib import Path

from jdamr_cube_navigation.table_edge_target import (
    compute_table_edge_docking_target,
    PROVENANCE,
)

import pytest
import yaml


def mount():
    """Return a tilted rear-camera mount document."""
    return {
        'camera_mount': {
            'status': 'derived',
            'parent_frame': 'base_link',
            'child_frame': 'camera_link',
            'transform': {
                'x_m': -0.275,
                'y_m': 0.0,
                'z_m': 1.0,
                'roll_rad': math.radians(2.0),
                'pitch_rad': math.radians(27.0),
                'yaw_rad': math.pi,
            },
        },
    }


def _base_to_optical(vector, roll_rad, pitch_rad, yaw_rad):
    yaw_cosine = math.cos(yaw_rad)
    yaw_sine = math.sin(yaw_rad)
    pitch_cosine = math.cos(pitch_rad)
    pitch_sine = math.sin(pitch_rad)
    roll_cosine = math.cos(roll_rad)
    roll_sine = math.sin(roll_rad)
    base_x, base_y, base_z = vector
    yaw_x = yaw_cosine * base_x + yaw_sine * base_y
    yaw_y = -yaw_sine * base_x + yaw_cosine * base_y
    pitch_x = pitch_cosine * yaw_x - pitch_sine * base_z
    pitch_z = pitch_sine * yaw_x + pitch_cosine * base_z
    camera_y = roll_cosine * yaw_y + roll_sine * pitch_z
    camera_z = -roll_sine * yaw_y + roll_cosine * pitch_z
    return -camera_y, -camera_z, pitch_x


def observation(mounted=None):
    """Project one physical horizontal edge into the configured camera."""
    mounted = mount() if mounted is None else mounted
    transform = mounted['camera_mount']['transform']
    rotation = tuple(transform[key] for key in (
        'roll_rad', 'pitch_rad', 'yaw_rad'))
    endpoints_base = ((-1.0, -0.4, 0.75), (-1.0, 0.4, 0.75))
    camera_position = tuple(transform[key] for key in ('x_m', 'y_m', 'z_m'))
    endpoints_optical = [
        _base_to_optical(tuple(
            point[index] - camera_position[index] for index in range(3)),
            *rotation)
        for point in endpoints_base
    ]
    normal_optical = _base_to_optical((0.0, 0.0, 1.0), *rotation)
    return {
        'stamp_s': 10.0,
        'detected': True,
        'stable': True,
        'surface_kind': 'table_edge',
        'edge_endpoints_optical_m': endpoints_optical,
        'plane_normal': normal_optical,
        'table_height_m': 0.75,
        'edge_rms_m': 0.003,
        'confidence': 0.9,
    }


def target(observed=None, mounted=None, robot=None, gap=0.05, extent=0.25):
    """Compute a target from default valid fixtures."""
    mounted = mount() if mounted is None else mounted
    return compute_table_edge_docking_target(
        observation(mounted) if observed is None else observed,
        mounted,
        ({'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
         if robot is None else robot),
        requested_gap_m=gap,
        front_extent_m=extent,
        now_s=10.2,
    )


def test_full_mount_geometry_produces_five_centimeter_table_gap():
    """Full roll, pitch and yaw recover the physical five-centimetre gap."""
    result = target()
    assert result['face_center_map_xy_m'] == pytest.approx((-1.0, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((1.0, 0.0))
    assert result['x_m'] == pytest.approx(-0.70)
    assert result['y_m'] == pytest.approx(0.0)
    assert abs(result['yaw_rad']) == pytest.approx(math.pi)
    assert result['estimate_gap_m'] == pytest.approx(0.05)
    assert result['provenance'] == PROVENANCE
    assert result['table_height_m'] == pytest.approx(0.75)
    assert result['surface_kind'] == 'table_edge'
    assert abs(result['edge_basis_map_xy'][1]) == pytest.approx(1.0)


def test_fresh_tracking_candidate_is_allowed_only_after_explicit_tracking_handoff():
    candidate = {**observation(), 'stable': False, 'reason': 'collecting_stability'}
    with pytest.raises(ValueError, match='stable detected'):
        target(observed=candidate)
    tracked = compute_table_edge_docking_target(
        candidate, mount(), {'x_m': 0., 'y_m': 0., 'yaw_rad': 0.},
        requested_gap_m=.05, front_extent_m=.25, now_s=10.2, require_stable=False)
    assert tracked == target()
    candidate['confidence'] = .1
    with pytest.raises(ValueError, match='confidence'):
        compute_table_edge_docking_target(
            candidate, mount(), {'x_m': 0., 'y_m': 0., 'yaw_rad': 0.},
            requested_gap_m=.05, front_extent_m=.25, now_s=10.2, require_stable=False)


def test_both_edge_ends_share_the_requested_line_gap():
    """The parallel robot-front line is five centimetres from both ends."""
    result = target()
    outward = result['outward_normal_map_xy']
    heading = (-outward[0], -outward[1])
    robot_front_center = (
        result['x_m'] + heading[0] * 0.25,
        result['y_m'] + heading[1] * 0.25,
    )
    for endpoint in ((-1.0, -0.4), (-1.0, 0.4)):
        signed_gap = (
            (robot_front_center[0] - endpoint[0]) * outward[0]
            + (robot_front_center[1] - endpoint[1]) * outward[1]
        )
        assert signed_gap == pytest.approx(0.05)


def test_robot_map_pose_rotates_edge_target_and_basis():
    """Robot map yaw rotates the target, normal and edge basis together."""
    result = target(robot={
        'x_m': 2.0, 'y_m': 1.0, 'yaw_rad': math.pi / 2.0})
    assert result['x_m'] == pytest.approx(2.0)
    assert result['y_m'] == pytest.approx(0.30)
    assert result['face_center_map_xy_m'] == pytest.approx((2.0, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((0.0, 1.0))
    assert abs(result['edge_basis_map_xy'][0]) == pytest.approx(1.0)
    assert result['yaw_rad'] == pytest.approx(-math.pi / 2.0)


@pytest.mark.parametrize(
    'mutate, message', [
        (lambda item: item.update(stamp_s=9.0), 'stale'),
        (lambda item: item.update(stable=False), 'stable detected'),
        (lambda item: item.update(surface_kind='front'), 'table_edge'),
        (lambda item: item.update(
            edge_endpoints_optical_m=[[0.0, 0.0, 1.0]]), 'two points'),
        (lambda item: item.update(
            edge_endpoints_optical_m=[
                [0.0, 0.0, 1.0], [math.nan, 0.0, 1.0]]), 'finite'),
        (lambda item: item.update(plane_normal=[0.0, 0.0, 2.0]),
         'unit optical'),
    ],
)
def test_rejects_invalid_or_stale_edge_observations(mutate, message):
    """Malformed, stale, unstable or wrongly typed observations fail closed."""
    observed = observation()
    mutate(observed)
    with pytest.raises(ValueError, match=message):
        target(observed=observed)


def test_rejects_short_edge_and_nonhorizontal_plane():
    """A short line or non-upward top normal cannot define table docking."""
    mounted = mount()
    observed = observation(mounted)
    first = observed['edge_endpoints_optical_m'][0]
    observed['edge_endpoints_optical_m'] = [first, first]
    with pytest.raises(ValueError, match='too short'):
        target(observed=observed, mounted=mounted)

    observed = observation(mounted)
    transform = mounted['camera_mount']['transform']
    observed['plane_normal'] = _base_to_optical(
        (1.0, 0.0, 0.0),
        transform['roll_rad'], transform['pitch_rad'], transform['yaw_rad'])
    with pytest.raises(ValueError, match='point upward'):
        target(observed=observed, mounted=mounted)


def test_camera_height_or_angle_change_is_consumed_from_mount():
    """Updated camera height and angles preserve the physical target."""
    adjusted = deepcopy(mount())
    transform = adjusted['camera_mount']['transform']
    transform.update({
        'z_m': 1.12,
        'roll_rad': math.radians(-4.0),
        'pitch_rad': math.radians(33.0),
        'yaw_rad': math.pi - 0.15,
    })
    result = target(mounted=adjusted)
    assert result['face_center_map_xy_m'] == pytest.approx((-1.0, 0.0))
    assert result['x_m'] == pytest.approx(-0.70)
    assert result['estimate_gap_m'] == pytest.approx(0.05)


def test_rejects_table_height_inconsistent_with_transformed_edge():
    """Reported and transformed table heights must describe one edge."""
    observed = observation()
    observed['table_height_m'] = 0.90
    with pytest.raises(ValueError, match='height disagrees'):
        target(observed=observed)


@pytest.mark.parametrize(
    'field, value, message', [
        ('confidence', 0.0, 'confidence'),
        ('edge_rms_m', 0.05, 'edge_rms_m'),
    ],
)
def test_rejects_observations_below_minimum_edge_quality(
        field, value, message):
    """A target is not produced from zero confidence or a poor line fit."""
    observed = observation()
    observed[field] = value
    with pytest.raises(ValueError, match=message):
        target(observed=observed)


def calibrated_mount(yaw=0.10, bias=0.02):
    mounted = mount()
    mounted['camera_mount']['table_edge_calibration'] = {
        'enabled': True, 'status': 'derived',
        'yaw_offset_rad': yaw, 'plane_offset_bias_m': bias,
        'reference_transform': deepcopy(mounted['camera_mount']['transform']),
    }
    return mounted


def test_task_calibration_rotates_and_offsets_one_face_before_map_projection():
    mounted = calibrated_mount()
    unchanged = deepcopy(mounted)
    robot = {'x_m': 2.0, 'y_m': -1.0, 'yaw_rad': 0.7}
    result = target(mounted=mounted, robot=robot)
    angle = robot['yaw_rad'] + 0.10
    normal = (math.cos(angle), math.sin(angle))
    face = (2.0 - 1.02 * normal[0], -1.0 - 1.02 * normal[1])
    assert result['outward_normal_map_xy'] == pytest.approx(normal)
    assert result['face_center_map_xy_m'] == pytest.approx(face)
    assert (result['x_m'], result['y_m']) == pytest.approx((
        face[0] + 0.30 * normal[0], face[1] + 0.30 * normal[1]))
    assert result['yaw_rad'] == pytest.approx(angle - math.pi)
    tangent = result['edge_basis_map_xy']
    assert sum(a * b for a, b in zip(normal, tangent)) == pytest.approx(0.0)
    calibration = result['table_edge_calibration']
    assert calibration['raw_plane_offset_m'] == pytest.approx(1.0)
    assert calibration['corrected_plane_offset_m'] == pytest.approx(1.02)
    # Observation calibration must not rewrite physical extrinsics or top height.
    assert mounted == unchanged
    assert result['table_height_m'] == pytest.approx(0.75)


def test_disabled_task_calibration_leaves_target_unchanged():
    mounted = calibrated_mount()
    mounted['camera_mount']['table_edge_calibration']['enabled'] = False
    assert target(mounted=mounted) == target()


@pytest.mark.parametrize('field', (
    'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad'))
def test_task_calibration_rejects_changed_physical_mount(field):
    mounted = calibrated_mount()
    mounted['camera_mount']['transform'][field] += 0.001
    with pytest.raises(ValueError, match='mount has changed'):
        target(mounted=mounted)


@pytest.mark.parametrize('calibration, message', [
    (None, 'mapping'),
    ({'enabled': 1, 'status': 'derived'}, 'enabled'),
    ({'enabled': True, 'status': 'candidate'}, 'derived or measured'),
    ({'enabled': True, 'status': 'derived', 'yaw_offset_rad': math.nan,
      'plane_offset_bias_m': 0.0}, 'finite'),
])
def test_task_calibration_rejects_malformed_input(calibration, message):
    mounted = mount()
    mounted['camera_mount']['table_edge_calibration'] = calibration
    with pytest.raises(ValueError, match=message):
        target(mounted=mounted)


def test_current_mount_disables_superseded_astra_s_task_calibration():
    """A replaced camera must not inherit the previous sensor's task correction.

    The 2026-10-27 correction was derived again for the current Astra mount
    (operator gap report plus a matching stationary depth observation).
    """
    mounted = yaml.safe_load((Path(__file__).resolve().parents[1]
                              / 'config/camera_mount_current.yaml').read_text())
    calibration = mounted['camera_mount']['table_edge_calibration']
    superseded = mounted['camera_mount']['provenance']['superseded_astra_s_profile']
    assert superseded['valid'] is False
    assert calibration['enabled'] is True
    assert calibration['status'] == 'derived'
    assert calibration['provenance']['valid_for'] == (
        'current_Astra_mount_water_station_front_edge')
    assert calibration['reference_transform'] == mounted['camera_mount']['transform']
    assert calibration['provenance']['millimeter_accuracy_validated'] is False

    # The enabled current-mount correction is applied (the superseded one was not).
    without_calibration = deepcopy(mounted)
    without_calibration['camera_mount'].pop('table_edge_calibration')
    assert target(mounted=mounted) != target(mounted=without_calibration)


def test_november_corner_recheck_uses_right_left_order_and_normal_distance():
    from jdamr_cube_navigation.table_edge_target import _calibrated_table_line
    current = yaml.safe_load((Path(__file__).parents[1] / 'config/camera_mount_current.yaml').read_text())
    calibration = current['camera_mount']['table_edge_calibration']
    data = calibration['field_recheck_20261101']
    right, left = data['internal_corner_gaps_m']
    angle = math.asin((right-left)/data['corner_spacing_m'])
    distance = (right+left)/2 + data['front_extent_m']*math.cos(angle)
    raw_angle = angle-data['prior_yaw_offset_rad']
    raw_distance = distance-data['prior_plane_offset_bias_m']
    normal = (-math.cos(raw_angle), -math.sin(raw_angle))
    center = tuple(-raw_distance*n for n in normal)
    corrected_center, corrected_normal, _ = _calibrated_table_line(center,normal,current)
    corners = [(data['front_extent_m'], -data['corner_spacing_m']/2),
               (data['front_extent_m'], data['corner_spacing_m']/2)]
    predicted = [sum((p[i]-corrected_center[i])*corrected_normal[i] for i in (0,1)) for p in corners]
    assert predicted == pytest.approx([data['external_right_gap_m'], data['external_left_gap_m']],abs=1e-9)


def test_live_tape_plane_compensates_rgb_exposure_translation_and_yaw():
    import numpy as np
    from jdamr_cube_navigation.table_edge_target import table_tape_surface_at_rgb
    mounted = mount()
    depth_pose, rgb_pose = (1., 2., .3), (1.05, 1.98, .34)
    observed = observation(mounted)
    result = table_tape_surface_at_rgb(
        observed, mounted, depth_pose, rgb_pose, 10.1, 10.2)
    rotation = np.array(result['R_base_from_optical'])
    point = rotation @ np.array(result['plane_point_optical_m'])
    point += np.array([-.275, 0., 1.])
    c, s = math.cos(rgb_pose[2]), math.sin(rgb_pose[2])
    world = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ point
    world[:2] += rgb_pose[:2]
    assert world[2] == pytest.approx(.75)
    # The carried plane still contains the original measured tabletop.
    assert (rotation @ np.array(result['plane_normal_optical'])) == pytest.approx(
        [0., 0., 1.], abs=1e-8)
    assert result['source'] == 'live_depth_table_plane'
    assert result['stamp_s'] == 10.
    assert result['rgb_stamp_s'] == 10.1


@pytest.mark.parametrize('stamp,now', [(9.6, 10.2), (10.3, 10.2)])
def test_live_tape_plane_rejects_mismatched_or_future_rgb(stamp, now):
    from jdamr_cube_navigation.table_edge_target import table_tape_surface_at_rgb
    with pytest.raises(ValueError, match='exposure mismatch'):
        table_tape_surface_at_rgb(observation(), mount(), (0.,0.,0.),
                                  (0.,0.,0.), stamp, now)


def test_live_tape_plane_does_not_consume_predicted_tracking_geometry():
    from jdamr_cube_navigation.table_edge_target import table_tape_surface_at_rgb
    observed = observation()
    observed.update(detected=False, stable=False, reason='predicted')
    with pytest.raises(ValueError, match='stable detected'):
        table_tape_surface_at_rgb(observed, mount(), (0.,0.,0.),
                                  (0.,0.,0.), 10.1, 10.2)


def test_live_tabletop_without_visible_front_edge_is_enough_for_tape_projection():
    from jdamr_cube_navigation.table_edge_target import table_tape_surface_at_rgb
    original = observation()
    point = [sum(end[i] for end in original['edge_endpoints_optical_m']) / 2
             for i in range(3)]
    doc = {'stamp_s': 10., 'detected': False, 'stable': False,
           'reason': 'no_table_edge_candidate', 'tabletop_plane': {
               'source': 'current_depth_tabletop_fit',
               'plane_normal_optical': original['plane_normal'],
               'plane_point_optical_m': point,
               'plane_rms_m': .002, 'table_height_m': .75, 'inlier_count': 500}}
    surface = table_tape_surface_at_rgb(doc, mount(), (0.,0.,0.),
                                        (0.,0.,0.), 10.1, 10.2)
    assert surface['source'] == 'live_depth_table_plane'
    assert surface['plane_point_optical_m'] == pytest.approx(point)
