"""Pure geometry for a table-edge docking target."""

import math

import numpy as np

from .box_docking_target import (
    _finite_fields,
    _finite_number,
    _normalize_angle,
    _rotate_rpy,
    _rotate_xy,
    _validated_mount,
    MAXIMUM_OBSERVATION_AGE_S,
)


PROVENANCE = 'TABLE_EDGE_CAMERA_MOUNT_ESTIMATE'
MINIMUM_EDGE_LENGTH_M = 0.10
MINIMUM_CONFIDENCE = 0.50
MINIMUM_UP_DOT = 0.80
MAXIMUM_EDGE_RMS_M = 0.01
MAXIMUM_EDGE_VERTICAL_DELTA_M = 0.05
MAXIMUM_HEIGHT_DISAGREEMENT_M = 0.05
# One 5 Hz depth-observer period plus half a 10 Hz RGB period.
MAXIMUM_TAPE_RGB_DEPTH_DELTA_S = 0.25


def table_tape_surface_at_rgb(observation, camera_mount, depth_pose_odom,
                              rgb_pose_odom, rgb_stamp_s, now_s):
    """Carry a measured tabletop plane to the RGB camera's exposure pose.

    Current tabletop up supplies roll/pitch; mount yaw and the measured planar
    camera lever remain explicit calibration inputs. No historical plane is used.
    """
    mount = _validated_mount(camera_mount)
    if not isinstance(observation, dict):
        raise ValueError('live tabletop observation must be a mapping')
    plane = observation.get('tabletop_plane')
    if plane is not None:
        if (not isinstance(plane, dict)
                or plane.get('source') != 'current_depth_tabletop_fit'):
            raise ValueError('live tabletop plane source invalid')
        up = np.asarray(_finite_vector(
            plane.get('plane_normal_optical'), 3, 'tabletop normal'))
        point = np.asarray(_finite_vector(
            plane.get('plane_point_optical_m'), 3, 'tabletop point'))
        rms = _finite_number(plane.get('plane_rms_m'), 'tabletop plane rms')
        height = _finite_number(plane.get('table_height_m'), 'tabletop height')
        support = plane.get('inlier_count')
        if (not math.isclose(float(np.linalg.norm(up)), 1., abs_tol=1e-3)
                or _optical_vector_to_base(up, mount)[2] < MINIMUM_UP_DOT
                or not 0. <= rms <= MAXIMUM_EDGE_RMS_M
                or not .5 <= height <= 1.05
                or not isinstance(support, int) or isinstance(support, bool)
                or support < 3):
            raise ValueError('live tabletop plane quality invalid')
    else:
        _validated_table_observation(observation, mount, now_s, require_stable=False)
        up = np.asarray(observation['plane_normal'], dtype=float)
        point = np.mean(observation['edge_endpoints_optical_m'], axis=0)
    rgb_stamp = _finite_number(rgb_stamp_s, 'rgb_stamp_s')
    depth_stamp = _finite_number(observation['stamp_s'], 'depth_stamp_s')
    if (not 0. <= now_s - depth_stamp <= MAXIMUM_OBSERVATION_AGE_S
            or not 0. <= now_s - rgb_stamp <= MAXIMUM_OBSERVATION_AGE_S
            or abs(rgb_stamp - depth_stamp) > MAXIMUM_TAPE_RGB_DEPTH_DELTA_S):
        raise ValueError('tape RGB/depth exposure mismatch')
    poses = [_finite_vector(pose, 3, 'base_pose_odom')
             for pose in (depth_pose_odom, rgb_pose_odom)]
    up /= np.linalg.norm(up)
    forward = np.array([0., 0., 1.]) - up[2] * up
    norm = np.linalg.norm(forward)
    if norm < 0.1:
        raise ValueError('tabletop forward axis unobservable')
    forward /= norm
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)

    def turn(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    rotation = turn(mount[5]) @ np.vstack([forward, left, up])
    world_rotations = [turn(pose[2]) @ rotation for pose in poses]
    origins = [np.array([pose[0], pose[1], 0.])
               + turn(pose[2]) @ np.asarray(mount[:3]) for pose in poses]
    world_point = world_rotations[0] @ point + origins[0]
    rgb_point = world_rotations[1].T @ (world_point - origins[1])
    rgb_normal = world_rotations[1].T @ world_rotations[0] @ up
    return {
        'source': 'live_depth_table_plane',
        'stamp_s': depth_stamp,
        'rgb_stamp_s': rgb_stamp,
        'plane_normal_optical': rgb_normal.tolist(),
        'plane_point_optical_m': rgb_point.tolist(),
        'R_base_from_optical': rotation.tolist(),
        'camera_lever_xy_m': list(mount[:2]),
    }


def _finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label} must contain {length} finite values')
    return tuple(
        _finite_number(item, f'{label}[{index}]')
        for index, item in enumerate(value))


def _optical_point_to_base(point, mount):
    optical_x, optical_y, optical_z = point
    mount_x, mount_y, mount_z, roll, pitch, yaw = mount
    base_x, base_y, base_z = _rotate_rpy(
        optical_z, -optical_x, -optical_y, roll, pitch, yaw)
    return base_x + mount_x, base_y + mount_y, base_z + mount_z


def _optical_vector_to_base(vector, mount):
    optical_x, optical_y, optical_z = vector
    _mount_x, _mount_y, _mount_z, roll, pitch, yaw = mount
    return _rotate_rpy(
        optical_z, -optical_x, -optical_y, roll, pitch, yaw)


def _validated_table_observation(document, mount, now_s, *, require_stable=True):
    if not isinstance(require_stable, bool):
        raise ValueError('require_stable must be a bool')
    tracking_candidate = (
        require_stable is False and isinstance(document, dict)
        and document.get('stable') is False
        and document.get('reason') == 'collecting_stability')
    if (not isinstance(document, dict)
            or document.get('detected') is not True
            or (document.get('stable') is not True and not tracking_candidate)
            or document.get('surface_kind') != 'table_edge'):
        raise ValueError('stable detected table_edge surface is required')
    stamp_s, confidence, table_height_m, edge_rms_m = _finite_fields(
        document,
        ('stamp_s', 'confidence', 'table_height_m', 'edge_rms_m'),
        'observation')
    now_s = _finite_number(now_s, 'now_s')
    age_s = now_s - stamp_s
    if age_s < 0.0 or age_s > MAXIMUM_OBSERVATION_AGE_S:
        raise ValueError('table-edge observation is stale or future-dated')
    if not MINIMUM_CONFIDENCE <= confidence <= 1.0:
        raise ValueError('table-edge confidence must be in [0.5, 1]')
    if table_height_m <= 0.0:
        raise ValueError('table_height_m must be positive')
    if not 0.0 <= edge_rms_m <= MAXIMUM_EDGE_RMS_M:
        raise ValueError('edge_rms_m exceeds the docking quality bound')

    endpoints = document.get('edge_endpoints_optical_m')
    if not isinstance(endpoints, (list, tuple)) or len(endpoints) != 2:
        raise ValueError('edge_endpoints_optical_m must contain two points')
    endpoints = tuple(
        _finite_vector(point, 3, f'edge_endpoints_optical_m[{index}]')
        for index, point in enumerate(endpoints))
    normal = _finite_vector(document.get('plane_normal'), 3, 'plane_normal')
    normal_magnitude = math.sqrt(sum(value * value for value in normal))
    if not math.isclose(normal_magnitude, 1.0, abs_tol=1e-3):
        raise ValueError('plane_normal must be a unit optical vector')

    endpoints_base = tuple(
        _optical_point_to_base(point, mount) for point in endpoints)
    edge_dx = endpoints_base[1][0] - endpoints_base[0][0]
    edge_dy = endpoints_base[1][1] - endpoints_base[0][1]
    edge_length_m = math.hypot(edge_dx, edge_dy)
    if edge_length_m < MINIMUM_EDGE_LENGTH_M:
        raise ValueError('table edge is too short for docking geometry')
    if abs(endpoints_base[1][2] - endpoints_base[0][2]) > (
            MAXIMUM_EDGE_VERTICAL_DELTA_M):
        raise ValueError('table edge is not level in the base frame')
    observed_height_m = 0.5 * (
        endpoints_base[0][2] + endpoints_base[1][2])
    if abs(observed_height_m - table_height_m) > (
            MAXIMUM_HEIGHT_DISAGREEMENT_M):
        raise ValueError('table height disagrees with transformed edge')
    normal_base = _optical_vector_to_base(normal, mount)
    if normal_base[2] < MINIMUM_UP_DOT:
        raise ValueError('table plane normal must point upward in base frame')
    return endpoints_base, table_height_m, edge_rms_m


def _calibrated_table_line(center, normal, camera_mount):
    """Apply a task calibration to line normal and signed base-origin distance."""
    calibration = camera_mount['camera_mount'].get('table_edge_calibration', {})
    if not isinstance(calibration, dict):
        raise ValueError('table-edge calibration must be a mapping')
    if calibration.get('enabled', False) is False:
        return center, normal, None
    if (calibration.get('enabled') is not True
            or calibration.get('status') not in {'derived', 'measured'}):
        raise ValueError('table-edge calibration must be enabled and derived or measured')
    yaw, bias = _finite_fields(
        calibration, ('yaw_offset_rad', 'plane_offset_bias_m'),
        'table_edge_calibration')
    reference_mount = _finite_fields(
        calibration.get('reference_transform', {}),
        ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad'),
        'table_edge_calibration.reference_transform')
    if not all(math.isclose(old, current, abs_tol=1e-9, rel_tol=0.0)
               for old, current in zip(reference_mount, _validated_mount(camera_mount))):
        raise ValueError('table-edge calibration mount has changed')
    raw_offset = -sum(center[i] * normal[i] for i in (0, 1))
    corrected_normal = _rotate_xy(normal[0], normal[1], yaw)
    corrected_offset = raw_offset + bias
    rotated_center = _rotate_xy(center[0], center[1], yaw)
    corrected_center = tuple(
        rotated_center[i] - bias * corrected_normal[i] for i in (0, 1))
    return corrected_center, corrected_normal, {
        'applied': True,
        'yaw_offset_rad': yaw,
        'plane_offset_bias_m': bias,
        'raw_normal_base_xy': normal,
        'raw_plane_offset_m': raw_offset,
        'corrected_plane_offset_m': corrected_offset,
        'scope': 'table_line_observation_not_physical_camera_transform',
    }


def compute_table_edge_docking_target(
        observation, camera_mount, robot_pose, *, requested_gap_m,
        front_extent_m, now_s, require_stable=True):
    """Estimate a map-frame base pose facing a visible table front edge."""
    gap_m = _finite_number(requested_gap_m, 'requested_gap_m')
    extent_m = _finite_number(front_extent_m, 'front_extent_m')
    if gap_m <= 0.0:
        raise ValueError('requested_gap_m must be positive')
    if extent_m <= 0.0:
        raise ValueError('front_extent_m must be positive')
    robot_x, robot_y, robot_yaw = _finite_fields(
        robot_pose, ('x_m', 'y_m', 'yaw_rad'), 'robot_pose')
    mount = _validated_mount(camera_mount)
    endpoints_base, table_height_m, edge_rms_m = (
        _validated_table_observation(observation, mount, now_s,
                                     require_stable=require_stable))

    center_x = 0.5 * (endpoints_base[0][0] + endpoints_base[1][0])
    center_y = 0.5 * (endpoints_base[0][1] + endpoints_base[1][1])
    edge_dx = endpoints_base[1][0] - endpoints_base[0][0]
    edge_dy = endpoints_base[1][1] - endpoints_base[0][1]
    edge_length = math.hypot(edge_dx, edge_dy)
    basis_x = edge_dx / edge_length
    basis_y = edge_dy / edge_length
    normal_x, normal_y = -basis_y, basis_x
    camera_x, camera_y = mount[0], mount[1]
    if ((camera_x - center_x) * normal_x
            + (camera_y - center_y) * normal_y) < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y

    (center_x, center_y), (normal_x, normal_y), calibration = _calibrated_table_line(
        (center_x, center_y), (normal_x, normal_y), camera_mount)
    if calibration is not None:
        # Keep the tangent and corrected plane normal perpendicular.
        basis_x, basis_y = -normal_y, normal_x

    center_offset_m = extent_m + gap_m
    target_base_x = center_x + normal_x * center_offset_m
    target_base_y = center_y + normal_y * center_offset_m
    target_map_dx, target_map_dy = _rotate_xy(
        target_base_x, target_base_y, robot_yaw)
    center_map_dx, center_map_dy = _rotate_xy(
        center_x, center_y, robot_yaw)
    normal_map_x, normal_map_y = _rotate_xy(
        normal_x, normal_y, robot_yaw)
    basis_map_x, basis_map_y = _rotate_xy(
        basis_x, basis_y, robot_yaw)
    estimate_gap_m = math.hypot(
        center_x - target_base_x,
        center_y - target_base_y) - extent_m
    heading_base = math.atan2(-normal_y, -normal_x)
    return {
        'x_m': robot_x + target_map_dx,
        'y_m': robot_y + target_map_dy,
        'yaw_rad': _normalize_angle(robot_yaw + heading_base),
        'estimate_gap_m': estimate_gap_m,
        'provenance': PROVENANCE,
        'face_center_map_xy_m': (
            robot_x + center_map_dx, robot_y + center_map_dy),
        'outward_normal_map_xy': (normal_map_x, normal_map_y),
        'table_height_m': table_height_m,
        'edge_basis_map_xy': (basis_map_x, basis_map_y),
        'edge_rms_m': edge_rms_m,
        'surface_kind': 'table_edge',
        'table_edge_calibration': calibration,
    }
