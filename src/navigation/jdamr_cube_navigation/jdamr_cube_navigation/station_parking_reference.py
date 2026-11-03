"""Pure fixed-station RGB-D reference and lateral anchoring helpers."""

from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Mapping

import cv2
import numpy as np

from jdamr_cube_navigation.rgbd_parking_reference import (
    CameraIntrinsics,
    PreparedRGBDReference,
    camera_reference_pose_to_planar_base_error,
    estimate_reference_planar_basis,
)


REGISTERED_LATERAL_TOLERANCE_M = 0.005
LATERAL_PRECISION_STATUS = 'NOT_VALIDATED'
DEFAULT_MINIMUM_LOOKAHEAD_M = 0.10
SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')
CAPTURE_EQUALS_GOAL = 'operator_confirmed_registered_station_goal'
# The robot was photographed where the operator measured it, not on the goal.
CAPTURE_OFFSET_FROM_GOAL = 'operator_measured_offset_from_capture'
MAXIMUM_GOAL_LEFT_OFFSET_M = 0.05


def capture_goal_left_offset(binding):
    """Return the measured lateral goal offset in the capture's base frame."""
    if not isinstance(binding, Mapping):
        raise ValueError('reference_goal_binding_missing')
    kind = binding.get('kind')
    if kind == CAPTURE_EQUALS_GOAL:
        if binding.get('robot_pose_equals_goal') is not True:
            raise ValueError('reference_goal_binding_pose_mismatch')
        return 0.0
    if kind != CAPTURE_OFFSET_FROM_GOAL or binding.get('robot_pose_equals_goal') is not False:
        raise ValueError('unsupported_reference_goal_binding')
    offset = binding.get('goal_offset_in_capture_base_m')
    if not isinstance(offset, Mapping) or set(offset) != {'forward_m', 'left_m'}:
        raise ValueError('reference_goal_offset_must_be_forward_left')
    # Depth owns the front gap; the image binding may move the goal sideways only.
    if _finite(offset['forward_m'], 'reference_goal_forward_offset') != 0.0:
        raise ValueError('reference_goal_offset_must_be_lateral_only')
    left = _finite(offset['left_m'], 'reference_goal_left_offset')
    if abs(left) > MAXIMUM_GOAL_LEFT_OFFSET_M:
        raise ValueError('reference_goal_left_offset_out_of_bounds')
    if not isinstance(binding.get('operator_measurement'), Mapping):
        raise ValueError('reference_goal_offset_requires_operator_measurement')
    return left


def _finite(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f'{label}_must_be_finite')
    return float(value)


def _pose(value, label):
    try:
        values = tuple(value)
    except TypeError as error:
        raise ValueError(f'{label}_must_be_xy_yaw') from error
    if len(values) != 3:
        raise ValueError(f'{label}_must_be_xy_yaw')
    return tuple(_finite(item, f'{label}_{index}')
                 for index, item in enumerate(values))


def _xy(value, label):
    try:
        values = tuple(value)
    except TypeError as error:
        raise ValueError(f'{label}_must_be_xy') from error
    if len(values) != 2:
        raise ValueError(f'{label}_must_be_xy')
    return tuple(_finite(item, f'{label}_{index}')
                 for index, item in enumerate(values))


def _wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def filter_station_goal_observations(observations, max_age_s):
    """Median one sample per image, expressed in the same odom frame.

    The first valid image is usable immediately. Later images can correct its
    error without changing the immutable taught reference or moving raw odom.
    """
    if _finite(max_age_s, 'max_age_s') <= 0.:
        raise ValueError('max_age_s_must_be_positive')
    by_stamp = {}
    for observation in observations:
        if observation.get('valid') is True:
            stamp = _finite(observation['stamp_s'], 'stamp_s')
            _pose(observation['goal_odom'], 'goal_odom')
            by_stamp[stamp] = observation
    if not by_stamp:
        return None
    latest_stamp = max(by_stamp)
    samples = [by_stamp[stamp] for stamp in sorted(by_stamp)
               if latest_stamp - stamp <= max_age_s]
    result = deepcopy(samples[-1])
    for frame in ('goal_odom', 'goal_map'):
        poses = [sample[frame] for sample in samples if frame in sample]
        if len(poses) != len(samples):
            # Never combine an old map goal with a newly filtered odom goal.
            result.pop(frame, None)
            continue
        heading = poses[-1][2]
        result[frame] = [
            float(np.median([pose[0] for pose in poses])),
            float(np.median([pose[1] for pose in poses])),
            _wrapped(heading + float(np.median([
                _wrapped(pose[2] - heading) for pose in poses]))),
        ]
    result['goal_filter'] = 'recent_valid_pose_median'
    result['goal_filter_sample_count'] = len(samples)
    result['goal_filter_xy_spread_m'] = max(
        math.dist(sample['goal_odom'][:2], result['goal_odom'][:2])
        for sample in samples)
    result['goal_filter_yaw_spread_rad'] = max(
        abs(_wrapped(sample['goal_odom'][2] - result['goal_odom'][2]))
        for sample in samples)
    return result


def _resolved_path(value, label):
    if not isinstance(value, (str, Path)) or not str(value):
        raise ValueError(f'{label}_path_missing')
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'{label}_not_found')
    return path


def _verified_file(path_value, hash_value, label):
    path = _resolved_path(path_value, label)
    if not isinstance(hash_value, str) or not SHA256_PATTERN.fullmatch(hash_value):
        raise ValueError(f'{label}_sha256_invalid')
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != hash_value:
        raise ValueError(f'{label}_sha256_mismatch')
    return path


def _camera_identity(intrinsics):
    value = CameraIntrinsics.from_value(intrinsics)
    return {
        'matrix': value.matrix,
        'distortion': value.distortion,
        'width': value.width,
        'height': value.height,
        'frame_id': value.frame_id,
    }


def _same_camera(left, right):
    if (left['width'] != right['width']
            or left['height'] != right['height']
            or left['frame_id'] != right['frame_id']
            or left['matrix'].shape != right['matrix'].shape
            or left['distortion'].shape != right['distortion'].shape):
        return False
    return (np.allclose(left['matrix'], right['matrix'], atol=1e-9, rtol=0)
            and np.allclose(left['distortion'], right['distortion'],
                            atol=1e-9, rtol=0))


def _invalid_observation(reason, stamp_s, assumptions, quality=None):
    return {
        'valid': False,
        'reason': str(reason),
        'goal_odom': None,
        'forward_m': None,
        'left_m': None,
        'yaw_rad': None,
        'stamp_s': stamp_s,
        'quality': quality or {},
        'assumptions': deepcopy(assumptions),
    }


class TaughtStationReference:
    """Load one immutable reference and observe its registered camera pose."""

    REQUIRED = (
        'reference_capture_path',
        'reference_capture_sha256',
        'reference_rgb_path',
        'reference_rgb_sha256',
        'reference_depth_path',
        'reference_depth_sha256',
        'reference_rgb_rois',
        'tabletop_depth_roi',
        'tabletop_depth_exclusion_rois',
        'camera_yaw_rad',
        'camera_lever_xy_m',
    )

    def __init__(self, config):
        if not isinstance(config, Mapping):
            raise ValueError('station_reference_config_must_be_mapping')
        missing = [key for key in self.REQUIRED if key not in config]
        if missing:
            raise ValueError(f'missing_station_reference_fields:{",".join(missing)}')
        if ('lateral_tolerance_m' in config
                and not math.isclose(
                    _finite(config['lateral_tolerance_m'], 'lateral_tolerance_m'),
                    REGISTERED_LATERAL_TOLERANCE_M, abs_tol=1e-12)):
            raise ValueError('registered_lateral_tolerance_must_be_0.005')
        if (config.get('lateral_precision_status', LATERAL_PRECISION_STATUS)
                != LATERAL_PRECISION_STATUS):
            raise ValueError('lateral_precision_status_must_be_NOT_VALIDATED')

        capture_path = _verified_file(
            config['reference_capture_path'],
            config['reference_capture_sha256'], 'reference_capture')
        rgb_path = _verified_file(
            config['reference_rgb_path'],
            config['reference_rgb_sha256'], 'reference_rgb')
        depth_path = _verified_file(
            config['reference_depth_path'],
            config['reference_depth_sha256'], 'reference_depth')
        capture = json.loads(capture_path.read_text(encoding='utf-8'))
        if not isinstance(capture, dict):
            raise ValueError('invalid_reference_capture')
        try:
            rgb_info = capture['camera_info']['rgb']
            depth_info = capture['camera_info']['depth']
            rgb_record = capture['rgb']
            depth_record = capture['depth']
            frame_counts = capture['frame_counts']
        except (KeyError, TypeError) as error:
            raise ValueError('invalid_reference_capture') from error
        self.reference_rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if self.reference_rgb is None:
            raise ValueError('invalid_reference_rgb')
        try:
            with np.load(depth_path, allow_pickle=False) as archive:
                if set(archive.files) != {'depth_raw_uint16'}:
                    raise ValueError('invalid_reference_depth_archive')
                self.reference_depth = np.asarray(
                    archive['depth_raw_uint16']).copy()
        except (OSError, ValueError) as error:
            raise ValueError('invalid_reference_depth_archive') from error
        if self.reference_depth.dtype != np.uint16 or self.reference_depth.ndim != 2:
            raise ValueError('invalid_reference_depth')

        self.reference_rgb_intrinsics = deepcopy(rgb_info)
        self.reference_depth_intrinsics = deepcopy(depth_info)
        self._rgb_identity = _camera_identity(rgb_info)
        depth_identity = _camera_identity(depth_info)
        expected_rgb_shape = tuple(rgb_record.get('shape', ()))
        expected_depth_shape = tuple(depth_record.get('shape', ()))
        if (expected_rgb_shape != self.reference_rgb.shape
                or expected_depth_shape != self.reference_depth.shape):
            raise ValueError('reference_capture_shape_mismatch')
        if (rgb_record.get('frame') != self._rgb_identity['frame_id']
                or depth_record.get('frame') != depth_identity['frame_id']
                or self._rgb_identity['frame_id'] != depth_identity['frame_id']):
            raise ValueError('reference_capture_frame_mismatch')
        if (self._rgb_identity['width'] != self.reference_rgb.shape[1]
                or self._rgb_identity['height'] != self.reference_rgb.shape[0]
                or depth_identity['width'] != self.reference_depth.shape[1]
                or depth_identity['height'] != self.reference_depth.shape[0]):
            raise ValueError('reference_capture_intrinsics_size_mismatch')
        try:
            rgb_stamp = _finite(rgb_record.get('stamp'), 'reference_rgb_stamp')
            depth_stamp = _finite(depth_record.get('stamp'), 'reference_depth_stamp')
            recorded_delta = _finite(
                capture.get('rgb_depth_delta_s'), 'rgb_depth_delta_s')
            datetime.fromisoformat(str(capture['captured_at']))
            rgb_frames = int(frame_counts['rgb'])
            depth_frames = int(frame_counts['depth'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('invalid_reference_capture_timestamps') from error
        if (rgb_stamp <= 0. or depth_stamp <= 0. or recorded_delta < 0.
                or rgb_frames <= 0 or depth_frames <= 0
                or not math.isclose(
                    abs(rgb_stamp - depth_stamp), recorded_delta,
                    abs_tol=1e-6, rel_tol=0.)):
            raise ValueError('invalid_reference_capture_timestamps')

        if (not isinstance(config['reference_rgb_rois'], Mapping)
                or len(config['reference_rgb_rois']) < 2):
            raise ValueError('at_least_two_named_reference_rois_required')
        self.reference_rgb_rois = deepcopy(config['reference_rgb_rois'])
        try:
            self.tabletop_depth_roi = tuple(config['tabletop_depth_roi'])
            self.tabletop_depth_exclusion_rois = tuple(
                tuple(roi) for roi in config['tabletop_depth_exclusion_rois'])
        except TypeError as error:
            raise ValueError('invalid_tabletop_roi_configuration') from error
        self.camera_yaw_rad = _finite(
            config['camera_yaw_rad'], 'camera_yaw_rad')
        self.camera_lever_xy_m = _xy(
            config['camera_lever_xy_m'], 'camera_lever_xy_m')
        self.basis = estimate_reference_planar_basis(
            self.reference_depth,
            self.reference_depth_intrinsics,
            self.tabletop_depth_roi,
            camera_yaw_rad=self.camera_yaw_rad,
            camera_lever_xy_m=self.camera_lever_xy_m,
            exclusion_depth_rois=self.tabletop_depth_exclusion_rois,
        )
        if not self.basis.valid:
            raise ValueError(f'invalid_reference_planar_basis:{self.basis.reason}')
        self.observation_method = config.get('observation_method', 'rgbd_pnp')
        if self.observation_method not in ('rgbd_pnp', 'table_contact_edges', 'table_front_tape'):
            raise ValueError('unsupported_station_observation_method')
        self.contact_reference = None
        self.prepared_reference = None
        if self.observation_method == 'table_front_tape':
            from jdamr_cube_navigation.table_tape_reference import TableTapeReference
            self.contact_reference = TableTapeReference(
                self.reference_rgb_intrinsics, self.basis, config['table_front_tape'])
        elif self.observation_method == 'table_contact_edges':
            from jdamr_cube_navigation.table_contact_reference import TableContactReference
            contact_config = capture.get('table_contact_reference')
            if not isinstance(contact_config, Mapping):
                raise ValueError('missing_bound_table_contact_reference')
            self.contact_reference = TableContactReference(
                self.reference_rgb, self.reference_rgb_intrinsics, self.basis, contact_config)
        else:
            self.prepared_reference = PreparedRGBDReference(
                self.reference_rgb,
                self.reference_depth,
                self.reference_rgb_intrinsics,
                self.reference_depth_intrinsics,
                self.reference_rgb_rois,
            )
        self.reference_capture = deepcopy(capture)
        self.goal_left_offset_m = capture_goal_left_offset(capture.get('goal_binding'))
        if self.observation_method == 'table_front_tape':
            # The user placed the strips symmetrically around the desired centre.
            # A historical cup/bottle capture offset does not move this new target.
            self.goal_left_offset_m = 0.0
        # A per-run operator correction, bound to this image frame by the caller.
        self.run_left_offset_m = 0.0
        self.R_base_from_optical = np.asarray(
            self.basis.R_base_from_optical, dtype=np.float64).copy()
        self.assumptions = {
            'observation_method': self.observation_method,
            'reference_capture_path': str(capture_path),
            'reference_capture_sha256': config['reference_capture_sha256'],
            'reference_rgb_path': str(rgb_path),
            'reference_rgb_sha256': config['reference_rgb_sha256'],
            'reference_depth_path': str(depth_path),
            'reference_depth_sha256': config['reference_depth_sha256'],
            'target_role': 'registered_reference_pose_not_object_centroid',
            'camera_yaw_rad': self.camera_yaw_rad,
            'camera_lever_xy_m': list(self.camera_lever_xy_m),
            'basis_source': self.basis.basis_source,
            'reference_captured_at': capture['captured_at'],
            'reference_rgb_stamp_s': rgb_stamp,
            'reference_depth_stamp_s': depth_stamp,
            'reference_rgb_depth_delta_s': recorded_delta,
            'reference_goal_binding': deepcopy(capture.get('goal_binding')),
            'goal_left_offset_from_capture_m': self.goal_left_offset_m,
            'mount_calibration_complete': self.basis.mount_calibration_complete,
            'millimeter_accuracy_validated': False,
            'registered_lateral_tolerance_m': REGISTERED_LATERAL_TOLERANCE_M,
            'lateral_precision_status': LATERAL_PRECISION_STATUS,
            'stale_age_checked_by_runtime_node': True,
        }

    def set_run_left_offset(self, offset_m):
        """Shift this run's goal sideways in the capture frame, never the capture."""
        offset = _finite(offset_m, 'run_left_offset')
        if abs(offset) > MAXIMUM_GOAL_LEFT_OFFSET_M:
            raise ValueError('run_left_offset_out_of_bounds')
        self.run_left_offset_m = offset

    def observe(self, current_rgb, current_info, odom_pose_at_image_stamp,
                stamp_s, *, surface=None):
        """Return the bound goal (capture pose plus lateral offset) in odom."""
        try:
            stamp = _finite(stamp_s, 'stamp_s')
            actual = _pose(odom_pose_at_image_stamp, 'odom_pose')
            current_identity = _camera_identity(current_info)
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            return _invalid_observation(str(error), stamp_s, self.assumptions)
        if not _same_camera(self._rgb_identity, current_identity):
            return _invalid_observation(
                'current_camera_intrinsics_identity_mismatch', stamp,
                self.assumptions)
        if self.contact_reference is not None:
            if self.observation_method == 'table_front_tape' and surface is not None:
                error, quality = self.contact_reference.observe(
                    current_rgb, surface=surface, rgb_info=current_info)
            else:
                error, quality = self.contact_reference.observe(current_rgb)
        else:
            estimate = self.prepared_reference.observe(current_rgb, current_info)
            quality = {
                'observation_method': 'rgbd_pnp',
                'match_count': estimate.match_count,
                'inlier_count': estimate.inlier_count,
                'reprojection_rmse_px': estimate.reprojection_rmse_px,
                'roi_inliers': dict(estimate.roi_inliers),
            }
            if not estimate.valid:
                return _invalid_observation(
                    estimate.reason, stamp, self.assumptions, quality)
            error = camera_reference_pose_to_planar_base_error(
                estimate.T_current_from_reference, self.basis)
        quality.update({
            'tabletop_point_count': self.basis.point_count,
            'tabletop_inlier_count': self.basis.inlier_count,
            'tabletop_inlier_ratio': self.basis.inlier_ratio,
            'tabletop_plane_rms_m': self.basis.plane_rms_m,
        })
        if not error.valid:
            return _invalid_observation(
                error.reason, stamp, self.assumptions, quality)
        # The offset is along the capture base's left axis, seen from current base.
        offset = self.goal_left_offset_m + self.run_left_offset_m
        forward_m = error.forward_m - math.sin(error.yaw_rad) * offset
        left_m = error.left_m + math.cos(error.yaw_rad) * offset
        cosine, sine = math.cos(actual[2]), math.sin(actual[2])
        goal = [
            actual[0] + cosine * forward_m - sine * left_m,
            actual[1] + sine * forward_m + cosine * left_m,
            _wrapped(actual[2] + error.yaw_rad),
        ]
        return {
            'valid': True,
            'reason': 'observed_registered_reference_pose',
            'observation_method': self.observation_method,
            'goal_odom': goal,
            'observation_odom_pose': list(actual),
            'forward_m': forward_m,
            'left_m': left_m,
            'yaw_rad': error.yaw_rad,
            'capture_forward_m': error.forward_m,
            'capture_left_m': error.left_m,
            'goal_left_offset_m': offset,
            'stamp_s': stamp,
            'quality': quality,
            'assumptions': deepcopy(self.assumptions),
        }


def anchor_face_to_taught_lateral(target, taught_xy):
    """Translate a detected face tangentially to the taught station position."""
    if not isinstance(target, Mapping):
        raise ValueError('target_must_be_mapping')
    face = _xy(target.get('face_center_map_xy_m'), 'face_center')
    normal = np.asarray(
        _xy(target.get('outward_normal_map_xy'), 'outward_normal'),
        dtype=np.float64)
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError('outward_normal_must_be_nonzero')
    unit_normal = normal / norm
    tangent = np.array((-unit_normal[1], unit_normal[0]))
    goal = np.asarray(
        (_finite(target.get('x_m'), 'target_x_m'),
         _finite(target.get('y_m'), 'target_y_m')),
        dtype=np.float64)
    taught = np.asarray(_xy(taught_xy, 'taught_xy'), dtype=np.float64)
    shift = float(np.dot(taught - goal, tangent))
    delta = shift * tangent
    result = deepcopy(dict(target))
    result['x_m'] = float(goal[0] + delta[0])
    result['y_m'] = float(goal[1] + delta[1])
    result['face_center_map_xy_m'] = [
        float(face[0] + delta[0]), float(face[1] + delta[1])]
    result['lateral_reference'] = 'operator_taught_physical_pose_and_heading'
    result['lateral_tolerance_m'] = REGISTERED_LATERAL_TOLERANCE_M
    result['lateral_precision_status'] = LATERAL_PRECISION_STATUS
    result['lateral_shift_m'] = shift
    return result


def lateral_approach_heading(
        face_heading, goal_odom, actual_odom, remaining_m, tolerance_m,
        max_heading_rad, minimum_lookahead_m=DEFAULT_MINIMUM_LOOKAHEAD_M):
    """Follow a smooth lateral path that ends parallel to the target face.

    A cubic Hermite path is rebuilt in the face-aligned frame on every
    update.  Its start tangent is the current chassis heading and its end
    tangent is parallel to the face.  A rate-led midpoint tangent therefore
    corrects lateral error early while progressively removing yaw error,
    instead of aiming straight at the lateral coordinate and turning back at
    the end of the approach.
    """
    heading = _finite(face_heading, 'face_heading')
    goal = _pose(goal_odom, 'goal_odom')
    actual = _pose(actual_odom, 'actual_odom')
    remaining = _finite(remaining_m, 'remaining_m')
    tolerance = _finite(tolerance_m, 'tolerance_m')
    maximum = _finite(max_heading_rad, 'max_heading_rad')
    lookahead_minimum = _finite(
        minimum_lookahead_m, 'minimum_lookahead_m')
    if remaining < 0.0:
        raise ValueError('remaining_m_must_be_nonnegative')
    if tolerance <= 0.0:
        raise ValueError('tolerance_m_must_be_positive')
    if not 0.0 < maximum <= math.pi:
        raise ValueError('max_heading_rad_out_of_bounds')
    if lookahead_minimum <= 0.0:
        raise ValueError('minimum_lookahead_m_must_be_positive')
    left = (-math.sin(heading), math.cos(heading))
    lateral_error = ((goal[0] - actual[0]) * left[0]
                     + (goal[1] - actual[1]) * left[1])
    if abs(lateral_error) <= tolerance:
        lateral_error = 0.0

    path_length = max(remaining, lookahead_minimum)
    heading_error = _wrapped(actual[2] - heading)
    bounded_start_heading = max(-maximum, min(maximum, heading_error))
    start_slope = math.tan(bounded_start_heading)

    # Midpoint Hermite tangent feedback is 1.5*error/path - .25*start_slope.
    # The lateral term receives 4/3 command lead so the existing 0.08 rad/s
    # yaw-rate limit and 0.35 s heading filter do not postpone straightening
    # until the final 5 cm band.  The final heading term still drives the path
    # tangent parallel to the face.
    correction = math.atan(
        2.0 * lateral_error / path_length - 0.25 * start_slope)
    correction = max(-maximum, min(maximum, correction))
    return _wrapped(heading + correction)
