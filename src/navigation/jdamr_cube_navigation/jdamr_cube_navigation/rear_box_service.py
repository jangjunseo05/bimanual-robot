"""Rear LiDAR face feedback on the existing guarded precision motion path."""

import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from jdamr_cube_navigation.box_lidar_witness import (
    MAX_CANDIDATE_PLANE_DISTANCE_M, MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD,
    MAXIMUM_RESIDUAL_RMS_M,
)
from jdamr_cube_navigation.corridor_route import (
    spin_node, TRANSIT_PLAN_END_TOLERANCE_M,
)
from jdamr_cube_navigation.parking import (
    MAXIMUM_FRONT_GAP_M, MINIMUM_FRONT_GAP_M,
)
from jdamr_cube_navigation.rear_box_docking import (
    derive_robot_pose_from_dock_reference, estimate_rear_box_face,
    MINIMUM_SUPPORT, MINIMUM_TANGENT_SPREAD_M, rear_corridor_clearance, RearBoxFaceTracker,
)
from jdamr_cube_navigation.rear_collision_profile import rear_collision_profile
from jdamr_cube_navigation.restaurant_service import (
    LIVE_APPROACH_MAX_M, STRAIGHT_ESCAPE_TOLERANCE_M,
)
from jdamr_cube_navigation.reverse_parking import (
    CachedStaticCorridorValidator, MAX_REVERSE_DISTANCE_M, reverse_curve_waypoints,
    reverse_waypoints, static_corridor_clear, path_sweep_hit, scan_points, to_base_frame,
)
from jdamr_cube_navigation.surface_approach import (
    FACE_APPROACH_TIME_S, FACE_HEADING_FILTER_S, filtered_face_heading,
)
from nav_msgs.msg import Path as RosPath
import rclpy
from tf2_ros import TransformException


REAR_COARSE_HANDOFF_M = 0.5
# The registered dock approach is 0.5 m on the deployed map.  Keep the
# compatibility name for callers/tests, but do not force a farther 1 m stage.
REAR_CENTER_HANDOFF_M = REAR_COARSE_HANDOFF_M
# Keep 2 cm of the fine controller's 0.6 m travel bound for heading correction.
REAR_STRAIGHT_HANDOFF_M = 0.58
REAR_CENTER_TOLERANCE_M = 0.02
REAR_COARSE_SEGMENT_M = 0.5
REAR_FACE_REOBSERVE_GAP_M = 0.2
ROTATION_SCAN_INPUT_GAPS = frozenset({
    'rotation scan unavailable',
    'rotation scan stale or future-dated',
})
# Planning candidates, not measured calibration: retain at least 10 cm for a
# stopped same-face observation before the final live-gap controller.
REAR_CENTER_REOBSERVE_GAPS_M = (REAR_FACE_REOBSERVE_GAP_M, 0.15, 0.1)
REAR_FINAL_TRIM_MAX_M = 0.01
# The saved mission ranges used by the live and stopped face fits lie on a
# 0.25 mm grid. Keep both confirmation paths on one boundary without rejecting
# a pose for a sub-bin corner-gap difference; this is an observed input bound,
# not a broader target-gap allowance.
REAR_GAP_CONFIRMATION_EPSILON_M = 0.00025
# Observation only: the swept footprint and gap calculation retain the chassis width.
REAR_FACE_OBSERVATION_HALF_WIDTH_M = 1.2
REAR_FACE_OBSERVATION_WIDTHS_M = (0.6, 0.9, REAR_FACE_OBSERVATION_HALF_WIDTH_M)


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _rear_gap_confirmed(gap_m, error_rad, half_width_m, target_gap_m):
    far_gap_m = gap_m + 2. * half_width_m * abs(math.sin(error_rad))
    lower_m = target_gap_m - .003 - REAR_GAP_CONFIRMATION_EPSILON_M
    upper_m = target_gap_m + .003 + REAR_GAP_CONFIRMATION_EPSILON_M
    return lower_m <= gap_m <= upper_m and lower_m <= far_gap_m <= upper_m


def _rear_center_target(observed, extent, gap_m=REAR_FACE_REOBSERVE_GAP_M):
    normal, midpoint = observed['normal_odom'], observed['midpoint_odom']
    return (*(midpoint[i] + (extent + gap_m) * normal[i]
              for i in (0, 1)), math.atan2(normal[1], normal[0]))


def _rear_center_curve_gap(observed, extent):
    """Prefer the farther handoff, then use a longer feasible reverse curve."""
    for gap_m in REAR_CENTER_REOBSERVE_GAPS_M:
        try:
            reverse_curve_waypoints(
                observed['base_pose_odom'], _rear_center_target(observed, extent, gap_m))
        except ValueError as error:
            if str(error) == 'already at goal':
                return gap_m
            if str(error) in ('reverse curve is tighter than min_radius_m',
                              'start_pose must be in front of target_pose'):
                continue
            raise
        return gap_m
    return None


def _rear_center_curve_available(observed, extent):
    return _rear_center_curve_gap(observed, extent) is not None


def _observe_rear_face_roi(scan, geometry, extent, half_width, expected_span_m=None):
    """Prefer the nearest useful view before including more side furniture."""
    candidate, failure = None, None
    for width_m in REAR_FACE_OBSERVATION_WIDTHS_M:
        try:
            observed = estimate_rear_box_face(
                scan, geometry=geometry, rear_extent_m=extent,
                rear_half_width_m=half_width, observation_half_width_m=width_m)
        except ValueError as error:
            failure = error
            continue
        if candidate is None:
            candidate = observed
        if observed['observation_roi_clipped']:
            continue
        if (expected_span_m is not None
                and abs(observed['span_m'] - expected_span_m) > max(.08, .25 * expected_span_m)):
            continue
        return observed
    if candidate is not None and expected_span_m is None:
        return candidate
    if candidate is not None:
        raise ValueError('rear dock reference face width or visibility changed')
    raise failure or ValueError('rear face unavailable in observation ROI')


def _same_target(reference, observed):
    normal, old_normal = observed['normal_odom'], reference['normal_odom']
    change = abs(_angle(math.atan2(normal[1], normal[0])
                        - math.atan2(old_normal[1], old_normal[0])))

    def local_support(item, item_normal):
        candidate = item.get('midpoint_odom')
        if candidate is None:
            return item['face_odom']
        residual = abs(sum((candidate[i] - item['face_odom'][i]) * item_normal[i]
                           for i in (0, 1)))
        return candidate if residual <= MAX_CANDIDATE_PLANE_DISTANCE_M else item['face_odom']

    old_support = local_support(reference, old_normal)
    support = local_support(observed, normal)
    # Compare each fitted plane at the other scan's local support centre.  A
    # closest-point-to-odom representation moves by origin * normal_noise even
    # when both fits pivot around the same physical box face.
    displacement = max(
        abs(sum((support[i] - reference['face_odom'][i]) * old_normal[i]
                for i in (0, 1))),
        abs(sum((old_support[i] - observed['face_odom'][i]) * normal[i]
                for i in (0, 1))),
    )
    if (change > MAXIMUM_NORMAL_YAW_DIFFERENCE_RAD
            or displacement > MAX_CANDIDATE_PLANE_DISTANCE_M):
        raise RuntimeError('rear box target changed')


def _set_collision_target(command_watch, observation, target_gap_m,
                          current_gap_m, source_point_filter=None):
    setter = getattr(command_watch, 'set_target', None)
    if callable(setter):
        setter(observation, target_gap_m, current_gap_m, source_point_filter)


def _clear_collision_target(command_watch):
    clearer = getattr(command_watch, 'clear_target', None)
    if callable(clearer):
        clearer()


def _stationary_face(observations, extent, half_width):
    """Use the stopped scan window rather than its last noisy line fit."""
    result = dict(observations[-1])
    for midpoint in ('face_midpoint_base_xy_m', 'midpoint_odom'):
        if all(midpoint in item for item in observations):
            result[midpoint] = tuple(sum(item[midpoint][i] for item in observations)
                                     / len(observations) for i in (0, 1))
    for normal_key, face_key, support_key in (
            ('normal_base_xy', 'face_center_base_xy_m', 'face_midpoint_base_xy_m'),
            ('normal_odom', 'face_odom', 'midpoint_odom')):
        normals = [item[normal_key] for item in observations]
        angle = math.atan2(sum(n[1] for n in normals), sum(n[0] for n in normals))
        normal = math.cos(angle), math.sin(angle)
        supports = [item.get(support_key, item[face_key]) for item in observations]
        support = tuple(sum(point[i] for point in supports) / len(supports)
                        for i in (0, 1))
        local_offset = sum(
            sum(item[normal_key][i] * (item[face_key][i] - supports[index][i])
                for i in (0, 1))
            for index, item in enumerate(observations)) / len(observations)
        result[normal_key] = normal
        result[face_key] = tuple(support[i] + local_offset * normal[i]
                                 for i in (0, 1))
    nx, ny = result['normal_base_xy']
    point = result['face_center_base_xy_m']
    offset = sum(point[i] * result['normal_base_xy'][i] for i in (0, 1))
    result.update(
        face_center_base_xy_m=(offset / nx, 0.),
        line_anchor_base_xy_m=point, line_offset_m=offset,
        tangent_base_xy=(-ny, nx), normal_yaw_error_rad=math.atan2(ny, nx),
        center_gap_m=-offset / nx - extent,
        gap_m=-offset - extent * nx - half_width * abs(ny),
        averaged_frames=len(observations))
    result['observation_roi_clipped'] = any(
        item.get('observation_roi_clipped', False) for item in observations)
    return result


class RearBoxParkingMixin:
    """Observe a rear box without starting the depth camera or a transit task."""

    def _reach_staging(self, stage):
        """Keep a completed staging position while transient scan gaps stop its turn."""
        staged = super()._reach_staging(
            stage,
            pause_on_rotation_failure=lambda failure:
            failure in ROTATION_SCAN_INPUT_GAPS)
        if staged is not None:
            return staged
        reason = getattr(self, '_last_direct_motion_failure', None)
        if reason not in ROTATION_SCAN_INPUT_GAPS:
            return None
        goal_yaw = float(stage['yaw'])
        previous_yaw = None
        rotation_travel_rad = 0.0
        rotation_limit_rad = None
        while reason in ROTATION_SCAN_INPUT_GAPS:
            if not self._rear_wait_for_input(
                    reason, preserve_motion_deadline=True):
                return None
            # The position action already reached staging.  Reissuing it here
            # can move the base after a sensor-only pause, so use only fresh
            # stopped odometry against the unchanged stage yaw.
            actual, _ = self.capture_stationary_pose(intermediate=True)
            current_yaw = actual[2]
            residual = _angle(goal_yaw - current_yaw)
            if previous_yaw is None:
                rotation_limit_rad = (
                    2.0 * abs(residual) + self.home_contract['yaw_tolerance_rad'])
            else:
                rotation_travel_rad += abs(_angle(current_yaw - previous_yaw))
                if rotation_travel_rad > rotation_limit_rad:
                    self._last_direct_motion_failure = (
                        'rear staging residual rotation exceeded bounded travel')
                    return None
            previous_yaw = current_yaw
            self.emit('rear_staging_rotation_resuming', reason=reason,
                      target_preserved=True, delta_yaw_rad=residual,
                      rotation_travel_rad=rotation_travel_rad,
                      rotation_limit_rad=rotation_limit_rad)
            if abs(residual) <= self.home_contract['yaw_tolerance_rad']:
                return actual
            if self.stop_requested or not self._operator_link_ready():
                return None
            turned = self._rotate_in_place(
                residual,
                pause_on_failure=lambda failure: failure in ROTATION_SCAN_INPUT_GAPS)
            self.emit('staging_turn_observed', requested_yaw_rad=residual,
                      observed_yaw_rad=turned, observation_frame='odom',
                      controller='damped')
            if turned is not None:
                actual, _ = self.capture_stationary_pose(intermediate=True)
                rotation_travel_rad += abs(_angle(actual[2] - previous_yaw))
                if rotation_travel_rad > rotation_limit_rad:
                    self._last_direct_motion_failure = (
                        'rear staging residual rotation exceeded bounded travel')
                    return None
                return actual
            reason = getattr(self, '_last_direct_motion_failure', None)
        return None

    def _execute_service_once(self, *, final_parking, alignment, staging=False, face=False):
        normal_departure = (
            not (final_parking or alignment or staging)
            and self.config.get('dock_departure_distance_m') is None
            and self.waypoints and self.waypoints[0]['id'] == 'home_exit'
            and getattr(self, '_resume_waypoint_index', 0) == 0
            and getattr(self, 'home_rear_geometry', None) is not None
            and self.registry.get('home', {}).get('rear_parking_reference') == 'lidar_box')
        if normal_departure and not getattr(self, '_rear_departure_capture_attempted', False):
            self._rear_departure_capture_attempted = True
            geometry = self.home_rear_geometry
            extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
            half_width = geometry['frame_width']['value'] / 2.
            try:
                observation = self._rear_box_observation(geometry, extent, half_width)
                if (observation['observation_roi_clipped']
                        or observation['support_count'] < 2 * MINIMUM_SUPPORT
                        or observation['span_m'] < 2 * MINIMUM_TANGENT_SPREAD_M):
                    raise RuntimeError('rear dock departure face support is insufficient')
                actual, _ = self.capture_stationary_pose(intermediate=True)
                current_odom = self._odom_pose()
                scan_odom = observation['base_pose_odom']
                turn = actual[2] - current_odom[2]
                cosine, sine = math.cos(turn), math.sin(turn)
                dx, dy = scan_odom[0] - current_odom[0], scan_odom[1] - current_odom[1]
                pose = {'x_m': actual[0] + cosine * dx - sine * dy,
                        'y_m': actual[1] + sine * dx + cosine * dy,
                        'yaw_rad': _angle(scan_odom[2] + turn)}
                self._rear_dock_reference = {
                    'observation': observation, 'pose_map': pose,
                    'start_pose_map': list(actual),
                    'approach_distance_m': self.registry['home'].get(
                        'approach_offset_m', REAR_COARSE_HANDOFF_M),
                    'observation_phase': 'before_nav2_departure',
                }
                self.emit('rear_dock_reference_recorded', reference=self._rear_dock_reference,
                          source='same_run_nav2_departure', internal_estimate_only=True)
            except (ValueError, RuntimeError) as error:
                self.emit('rear_dock_reference_unavailable', reason=str(error))
        return super()._execute_service_once(
            final_parking=final_parking, alignment=alignment, staging=staging, face=face)

    def _depart_dock_straight(self, distance_m):
        # Only the route's initial dock exit teaches a landmark. A terminal
        # clearance/retry must never replace the outbound reference.
        outbound = (getattr(self, 'home_rear_geometry', None) is not None
                    and self.config.get('dock_departure_distance_m') == distance_m
                    and self.registry.get('home', {}).get('rear_parking_reference') == 'lidar_box')
        if outbound and getattr(self, '_dock_departure', None) is None:
            self._rear_dock_reference = None
        teach = outbound and getattr(self, '_rear_dock_reference', None) is None
        before_exit = None
        if teach:
            geometry = self.home_rear_geometry
            extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
            half_width = geometry['frame_width']['value'] / 2.
            try:
                before_exit = self._rear_box_observation(geometry, extent, half_width)
            except (ValueError, RuntimeError):
                pass
        if not super()._depart_dock_straight(distance_m):
            return False
        if teach:
            geometry = self.home_rear_geometry
            extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
            half_width = geometry['frame_width']['value'] / 2.
            try:
                # The parent already confirmed stopped odometry at the exit.
                # Use its latest fresh stamped scan without another dwell.
                phase = 'after_straight_departure'
                fallback_reason = None
                try:
                    observation = self._rear_box_observation(geometry, extent, half_width)
                    if observation['observation_roi_clipped']:
                        fallback_reason = 'rear dock face is clipped by observation ROI'
                    elif (observation['support_count'] < 2 * MINIMUM_SUPPORT
                          or observation['span_m'] < 2 * MINIMUM_TANGENT_SPREAD_M):
                        fallback_reason = 'rear dock reference support is too small'
                except (ValueError, RuntimeError) as error:
                    if before_exit is None:
                        raise
                    fallback_reason = str(error)
                if fallback_reason is not None and before_exit is not None:
                    observation, phase = before_exit, 'before_straight_departure'
                if observation['observation_roi_clipped']:
                    raise RuntimeError('rear dock face is clipped by observation ROI')
                if (observation['support_count'] < 2 * MINIMUM_SUPPORT
                        or observation['span_m'] < 2 * MINIMUM_TANGENT_SPREAD_M):
                    raise RuntimeError('rear dock reference support is too small')
                departure = self._dock_departure
                end_map, end_odom = departure['end'], self._odom_pose()
                scan_odom = observation['base_pose_odom']
                turn = end_map[2] - end_odom[2]
                cosine, sine = math.cos(turn), math.sin(turn)
                dx, dy = scan_odom[0] - end_odom[0], scan_odom[1] - end_odom[1]
                # The scan can predate the stopped exit pose. Apply its stamped
                # odom displacement before associating the face with map.
                pose = {'x_m': end_map[0] + cosine * dx - sine * dy,
                        'y_m': end_map[1] + sine * dx + cosine * dy,
                        'yaw_rad': _angle(scan_odom[2] + turn)}
                self._rear_dock_reference = {
                    'observation': observation, 'pose_map': pose,
                    'start_pose_map': list(departure['start']),
                    'exit_pose_map': list(departure['end']),
                    'observation_phase': phase,
                }
                self.emit('rear_dock_reference_recorded', reference=self._rear_dock_reference,
                          source='same_run_straight_departure',
                          after_exit_failure=fallback_reason, internal_estimate_only=True)
            except (ValueError, RuntimeError) as error:
                self.emit('rear_dock_reference_unavailable', reason=str(error))
        return True

    def _rear_reference_stage(self, stage):
        reference = getattr(self, '_rear_dock_reference', None)
        if reference is None:
            return stage
        pose, observation = reference['pose_map'], reference['observation']
        angle = pose['yaw_rad'] + observation['normal_yaw_error_rad']
        start = reference['start_pose_map']
        distance = self.registry.get('home', {}).get(
            'approach_offset_m', reference.get(
                'approach_distance_m', REAR_COARSE_HANDOFF_M))
        if distance <= 0.0:
            raise RuntimeError('rear dock reference has no forward departure')
        # Preserve the outbound origin's tangent position; use the observed
        # box normal for the reverse axis instead of a separate map heading.
        return {**stage, 'x': start[0] + distance * math.cos(angle),
                'y': start[1] + distance * math.sin(angle), 'yaw': _angle(angle)}

    def _rear_reference_pose(self, observed, *, actual_pose_map=None):
        reference = self._rear_dock_reference
        taught = reference['observation']
        if observed.get('observation_roi_clipped', True):
            raise RuntimeError('rear dock reference face is clipped')
        if abs(observed['span_m'] - taught['span_m']) > max(.08, .25 * taught['span_m']):
            raise RuntimeError('rear dock reference face width changed')
        measured = derive_robot_pose_from_dock_reference(taught, observed, reference['pose_map'])
        if actual_pose_map is not None:
            inferred = tuple(measured[k] for k in ('x_m', 'y_m', 'yaw_rad'))
            if (math.dist(actual_pose_map[:2], inferred[:2]) > REAR_FACE_OBSERVATION_HALF_WIDTH_M
                    or abs(_angle(actual_pose_map[2] - inferred[2])) > math.radians(30.)):
                raise RuntimeError('rear dock landmark disagrees with return region')
        return measured

    def _align_rear_reference(self, stage, pose, *, observed=None, return_to_stage=True):
        """Correct observed dock drift, then center the existing reverse entry."""
        if getattr(self, '_rear_dock_reference', None) is None:
            return True
        reason = ('operator_stop' if self.stop_requested else
                  'operator_link_lost' if not self._operator_link_ready() else
                  self._guard_failure(False))
        if reason:
            raise RuntimeError(reason)
        geometry = self.home_rear_geometry
        extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
        half_width = geometry['frame_width']['value'] / 2.
        if observed is None:
            observed = self._observe_rear_box(geometry, extent, half_width)
        if not return_to_stage:
            # Validate the observed face identity; terminal control uses its
            # relative odom geometry and does not need another AMCL reset.
            measured = self._rear_reference_pose(observed)
            self.emit('rear_dock_landmark_relative_control',
                      observed_pose_map=[measured[k] for k in ('x_m', 'y_m', 'yaw_rad')],
                      control_frame='odom', localization_reset_requested=False)
            return True
        actual, _ = self.capture_stationary_pose(intermediate=True)
        measured = self._rear_reference_pose(observed, actual_pose_map=actual)
        inferred = tuple(measured[k] for k in ('x_m', 'y_m', 'yaw_rad'))
        shift = math.dist(actual[:2], inferred[:2])
        turn = abs(_angle(actual[2] - inferred[2]))
        # This seed comes from a measured landmark, not the registered dock pose.
        if shift > .02 or turn > math.radians(.5):
            from jdamr_cube_navigation.activate_navigation import initialize_localization

            initialize_localization(self, {
                'frame_id': self.registry['frame_id'], **measured,
                'covariance_x_m2': .0025, 'covariance_y_m2': .0025,
                'covariance_yaw_rad2': math.radians(3.) ** 2,
            })
            localized, _ = self.capture_stationary_pose(intermediate=True)
            if (math.dist(localized[:2], inferred[:2]) > .05
                    or abs(_angle(localized[2] - inferred[2])) > math.radians(2.)):
                raise RuntimeError('rear dock landmark localization not retained')
            self.emit('rear_dock_landmark_localized', prior_pose_map=list(actual),
                      observed_pose_map=list(inferred), translation_correction_m=shift,
                      yaw_correction_rad=turn, source='same_run_rear_face_midpoint_and_normal')
            actual = localized
        if return_to_stage and (math.dist(actual[:2], (stage['x'], stage['y'])) > .05
                or abs(_angle(actual[2] - stage['yaw'])) > math.radians(3.)):
            stage_pose = {**pose, 'id': stage['id'], 'x_m': stage['x'], 'y_m': stage['y'],
                          'yaw_rad': stage['yaw'], 'parking_direction': 'forward'}
            planned = self._plan_with_input_recovery(
                stage_pose, single=True, end_tolerance_m=TRANSIT_PLAN_END_TOLERANCE_M)
            if not planned['ok']:
                return False
            actual = self._reach_staging(stage)
            if actual is None:
                return False
            observed = self._observe_rear_box(geometry, extent, half_width)
            self._rear_reference_pose(observed, actual_pose_map=actual)
        return True

    def _center_rear_box(self, observed, extent, *, after_clearance=False):
        """Use one guarded reverse curve while there is room to remove lateral error."""
        self._rear_center_moved = False
        self._rear_center_skipped = False
        if observed.get('observation_roi_clipped', True):
            raise RuntimeError('rear box midpoint is clipped')
        odom = self._odom_pose()
        normal = observed['normal_odom']
        midpoint = observed['midpoint_odom']
        tangent = -normal[1], normal[0]
        lateral = sum((odom[i] - midpoint[i]) * tangent[i] for i in (0, 1))
        self.emit('rear_dock_center_observed', lateral_error_m=lateral,
                  angle_rad=observed['normal_yaw_error_rad'], basis='live_rear_face_midpoint')
        if (not after_clearance and abs(lateral) <= REAR_CENTER_TOLERANCE_M
                and observed['gap_m'] <= REAR_STRAIGHT_HANDOFF_M):
            self._rear_center_skipped = True
            return True
        # Re-evaluate from the current pose: a guarded clearance can have moved
        # the base since this fixed-in-odom face was observed.
        curve_gap = _rear_center_curve_gap(dict(observed, base_pose_odom=odom), extent)
        if curve_gap is None:
            raise RuntimeError('rear dock centering reverse curve unavailable')
        target_odom = _rear_center_target(observed, extent, curve_gap)
        _path, to_odom = self._frozen_in_odom(RosPath())
        anchor = to_odom(0., 0., 0.)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])

        def to_map(pose):
            dx, dy = pose[0] - anchor[0], pose[1] - anchor[1]
            return (cosine * dx + sine * dy, -sine * dx + cosine * dy,
                    _angle(pose[2] - anchor[2]))

        actual, target = to_map(odom), to_map(target_odom)
        path = self._make_reverse_path(actual, target, curve=True)
        points = [(item.pose.position.x, item.pose.position.y,
                   2. * math.atan2(item.pose.orientation.z, item.pose.orientation.w))
                  for item in path.poses]
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())
        if not check(points):
            exact_clear = static_corridor_clear(
                Path(self.registry['map']['yaml_path']),
                Path(self.registry['keepout']['yaml_path']),
                points, self._footprint_outline())
            self.emit('rear_dock_center_static_check',
                      conservative_blockage=getattr(check, 'last_blockage', None),
                      exact_corridor_clear=exact_clear, path_points=points)
            if not exact_clear:
                if not static_corridor_clear(
                        Path(self.registry['map']['yaml_path']),
                        Path(self.registry['keepout']['yaml_path']), points,
                        self._footprint_outline(), live_checked_dock_occupancy=True):
                    raise RuntimeError('rear dock centering keepout or unknown corridor blocked')
                # The local dock face is observed in odom. A stale map outline
                # must not override fresh swept-footprint evidence here.
                snapshot = self._scan_in_base()
                if snapshot is None:
                    raise RuntimeError('rear dock centering live scan unavailable')
                scan, laser, outline, memory = snapshot
                local_path = [to_base_frame(actual, point) for point in points]
                occupied = path_sweep_hit([*scan_points(scan, laser), *memory],
                                          outline, local_path)
                self.emit('rear_dock_center_live_check', occupied=occupied,
                          static_map_disagreement=True)
                if occupied:
                    raise RuntimeError('rear dock centering live corridor occupied')
        frozen = self._odom_path([to_odom(*point) for point in points])
        self.waypoints = [{'id': 'rear_box_center', 'x': target[0], 'y': target[1],
                           'yaw': target[2]}]
        self.emit('rear_dock_centering_started', target_odom=list(target_odom),
                  lateral_error_m=lateral, target_gap_m=curve_gap)
        # This is an observation waypoint, not the final dock pose.
        # Reobserve the same face and finish heading before terminal approach.
        completed = self._execute_reverse_once(
            frozen, goal_checker_id='entry_heading_checker', final=False,
            motion='rear_dock_centering', position_handoff_m=.015)
        self._rear_center_moved = bool(completed)
        return completed

    def _align_rear_face_before_clearance(self, observed, geometry, extent, half_width):
        """Remove large heading error before a straight clearance can add lateral drift."""
        initial = observed
        if observed['gap_m'] < 0.05:
            return observed
        gain = 1.0
        previous_yaw = observed['base_pose_odom'][2]
        rotation_travel_rad = 0.0
        rotation_limit_rad = (
            2.0 * abs(observed['normal_yaw_error_rad']) + math.radians(3.))
        completed_attempts = 0
        # Bound stationary heading correction to a primary command and one
        # measured residual command. Sensor pauses do not consume an attempt,
        # but fresh odometry still bounds total physical rotation.
        while completed_attempts < 2:
            error = observed['normal_yaw_error_rad']
            if abs(error) <= math.radians(3.):
                return observed
            reason = ('operator_stop' if self.stop_requested else
                      'operator_link_lost' if not self._operator_link_ready() else
                      self._guard_failure(False))
            if reason:
                raise RuntimeError(reason)
            correction = gain * error
            self.emit('rear_dock_face_heading_alignment', angle_rad=error,
                      correction_rad=correction, gain=gain, translation_requested=False)
            if self._rotate_in_place(
                    correction,
                    pause_on_failure=lambda failure:
                    failure in ROTATION_SCAN_INPUT_GAPS) is None:
                reason = (getattr(self, '_last_direct_motion_failure', None)
                          or 'rear dock heading alignment incomplete')
                if reason not in ROTATION_SCAN_INPUT_GAPS:
                    raise RuntimeError(reason)
                if not self._rear_wait_for_input(
                        reason, preserve_motion_deadline=True):
                    latest_failure = getattr(
                        self, '_last_direct_motion_failure', None)
                    raise RuntimeError(
                        'operator_stop' if self.stop_requested else
                        'operator_link_lost' if not self._operator_link_ready() else
                        latest_failure or reason)
                observed = self._observe_rear_box_recovering(
                    geometry, extent, half_width, allow_range_reapproach=True)
                _same_target(initial, observed)
                current_yaw = observed['base_pose_odom'][2]
                rotation_travel_rad += abs(_angle(current_yaw - previous_yaw))
                previous_yaw = current_yaw
                if rotation_travel_rad > rotation_limit_rad:
                    raise RuntimeError(
                        'rear dock face rotation exceeded bounded residual travel')
                self.emit('rear_dock_face_heading_alignment_resuming',
                          reason=reason, target_preserved=True,
                          rotation_travel_rad=rotation_travel_rad,
                          rotation_limit_rad=rotation_limit_rad)
                continue
            completed_attempts += 1
            observed = self._observe_rear_box_recovering(
                geometry, extent, half_width, allow_range_reapproach=True)
            _same_target(initial, observed)
            current_yaw = observed['base_pose_odom'][2]
            rotation_travel_rad += abs(_angle(current_yaw - previous_yaw))
            previous_yaw = current_yaw
            if rotation_travel_rad > rotation_limit_rad:
                raise RuntimeError(
                    'rear dock face rotation exceeded bounded residual travel')
            residual = observed['normal_yaw_error_rad']
            # A sign reversal measures overshoot against the same stationary face.
            # Reduce the next command; never amplify it or accept a bad heading.
            if error * residual < 0.0:
                gain = min(gain, max(.25, abs(correction / (error - residual))))
            self.emit('rear_dock_face_heading_alignment_result',
                      before_rad=error, residual_rad=residual, next_gain=gain)
        if abs(observed['normal_yaw_error_rad']) > math.radians(3.):
            raise RuntimeError('rear dock face heading alignment not converged')
        return observed

    def _rear_box_scan(self):
        scan = self.last_scan
        received = self.samples.get('scan')
        if scan is None or received is None or time.monotonic() - received > 0.5:
            raise RuntimeError('rear box scan unavailable')
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not 0.0 <= now_s - stamp <= 0.5:
            raise RuntimeError('rear box scan stale or future-dated')
        if scan.header.frame_id != 'laser_link':
            raise RuntimeError('rear box scan frame mismatch')
        return scan, stamp

    @staticmethod
    def _rear_scan_values(scan):
        return {
            'ranges': list(scan.ranges), 'angle_min': scan.angle_min,
            'angle_increment': scan.angle_increment,
            'range_min': scan.range_min, 'range_max': scan.range_max,
        }

    def _rear_box_observation(self, geometry, extent, half_width):
        scan, stamp = self._rear_box_scan()
        reference = getattr(self, '_rear_dock_reference', None)
        expected_span = (None if reference is None or getattr(self, '_rear_fine_face', None)
                         is not None else reference['observation']['span_m'])
        key = id(geometry), extent, half_width, expected_span
        failed = getattr(self, '_rear_box_detection_failure', None)
        if failed is not None and failed[:2] == (stamp, key):
            raise RuntimeError('rear box face unavailable')
        cached = getattr(self, '_rear_box_detection_cache', None)
        if (cached is not None and cached[0] == stamp
                and cached[1] == key):
            return cached[2]
        try:
            observation = _observe_rear_face_roi(
                self._rear_scan_values(scan), geometry, extent, half_width, expected_span)
        except ValueError as error:
            self._rear_box_detection_failure = stamp, key
            self.emit('rear_box_face_unavailable', reason=str(error))
            raise RuntimeError('rear box face unavailable') from error
        self._rear_box_detection_failure = None
        try:
            transform = self.parking_tf.lookup_transform(
                'odom', self.parking_contract['robot_base_frame'],
                rclpy.time.Time(seconds=stamp)).transform
        except TransformException as error:
            raise RuntimeError('rear box stamped odom TF unavailable') from error
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        x, y = observation['face_center_base_xy_m']
        nx, ny = observation['normal_base_xy']
        result = {**observation, 'stamp_s': stamp,
                  'base_pose_odom': (transform.translation.x, transform.translation.y, yaw),
                  'face_odom': (transform.translation.x + cosine * x - sine * y,
                                transform.translation.y + sine * x + cosine * y),
                  'normal_odom': (cosine * nx - sine * ny, sine * nx + cosine * ny)}
        mx, my = observation['face_midpoint_base_xy_m']
        result['midpoint_odom'] = (
            transform.translation.x + cosine * mx - sine * my,
            transform.translation.y + sine * mx + cosine * my)
        self._rear_box_detection_cache = stamp, key, result
        return result

    def _rear_corridor_observation(self, geometry, extent, half_width):
        scan, stamp = self._rear_box_scan()
        try:
            observed = rear_corridor_clearance(
                self._rear_scan_values(scan), geometry=geometry,
                rear_extent_m=extent, rear_half_width_m=half_width)
        except ValueError as error:
            raise RuntimeError('rear box corridor unavailable') from error
        return {**observed, 'stamp_s': stamp}

    def _rear_coarse_step(self, geometry, extent, half_width, distance_m,
                          command_watch=None, *, handoff_m=REAR_COARSE_HANDOFF_M,
                          stop_when_face_visible=False, face_half_width_m=None):
        start = self._odom_pose()
        target = (start[0] - distance_m * math.cos(start[2]),
                  start[1] - distance_m * math.sin(start[2]), start[2])
        anchor = self._frozen_in_odom(RosPath())[1](0., 0., 0.)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())

        def validate(actual):
            if math.dist(actual[:2], target[:2]) <= 0.05:
                poses = [actual, target]
            else:
                poses = reverse_waypoints(actual, target, max_distance_m=LIVE_APPROACH_MAX_M)
            mapped = []
            for x, y, yaw in poses:
                dx, dy = x - anchor[0], y - anchor[1]
                mapped.append((cosine * dx + sine * dy,
                               -sine * dx + cosine * dy, yaw - anchor[2]))
            if not check(mapped):
                raise RuntimeError('rear box coarse static corridor blocked')

        validate(start)
        validated_pose = [start]
        face_tracker = RearBoxFaceTracker(required_frames=8)
        last_face_stamp = [None]

        def stable_face_visible():
            """Require one odom-stationary face, not a one-frame base-frame fit.

            The base-frame line offset changes while reversing.  The same
            physical face is stationary in odom, so track that line instead.
            This keeps a transient furniture/edge fit from repeatedly stopping
            a bounded range reapproach after only a few millimetres.
            """
            try:
                observed = self._rear_box_observation(
                    geometry, extent,
                    half_width if face_half_width_m is None else face_half_width_m)
            except RuntimeError:
                face_tracker.update(None)
                return False
            if observed['stamp_s'] == last_face_stamp[0]:
                return False
            last_face_stamp[0] = observed['stamp_s']
            normal = observed['normal_odom']
            tracked = {
                'normal_base_xy': normal,
                'tangent_base_xy': (-normal[1], normal[0]),
                'line_offset_m': sum(
                    normal[i] * observed['face_odom'][i] for i in (0, 1)),
            }
            return face_tracker.update(tracked)

        def watch():
            try:
                observed = self._rear_corridor_observation(geometry, extent, half_width)
                if observed['gap_m'] <= handoff_m:
                    return 'rear box coarse handoff ready'
                if stop_when_face_visible and stable_face_visible():
                    return 'rear box face stable'
                actual = self._odom_pose()
                if (math.dist(actual[:2], validated_pose[0][:2]) >= 0.025
                        or abs(_angle(actual[2] - validated_pose[0][2])) >= math.radians(.3)):
                    validate(actual)
                    validated_pose[0] = actual
            except (RuntimeError, ValueError, TransformException) as error:
                return str(error)
            return None

        kwargs = {} if command_watch is None else {'command_watch': command_watch}
        travelled = self._drive_zero_turn(
            -distance_m, watch=watch, hold_heading=True, **kwargs)
        failure = getattr(self, '_last_direct_motion_failure', None)
        if failure in ('rear box coarse handoff ready', 'rear box face stable'):
            return True
        if travelled is None or failure or self.stop_requested:
            self.emit('rear_box_coarse_paused', reason=failure or 'coarse motion incomplete')
            return False
        return travelled < 0.0

    def _approach_rear_corridor(self, geometry, extent, half_width, command_watch=None,
                               *, handoff_m=REAR_COARSE_HANDOFF_M):
        """Range while reversing on the known dock axis; latch a face only nearby."""
        start = self._odom_pose()
        outline = self._footprint_outline()
        corridor_half_width = max(half_width, *(abs(point[1]) for point in outline))
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            try:
                observed = self._rear_corridor_observation(
                    geometry, extent, corridor_half_width)
            except RuntimeError as error:
                if not self._rear_wait_for_input(str(error), command_watch):
                    return False
                continue
            if observed['gap_m'] <= handoff_m:
                self.emit('rear_box_fine_handoff', gap_m=observed['gap_m'])
                return True
            travelled = math.dist(start[:2], self._odom_pose()[:2])
            budget = MAX_REVERSE_DISTANCE_M - travelled
            if budget <= 0.0:
                raise RuntimeError('rear box coarse approach exceeds bounded travel')
            distance = min(REAR_COARSE_SEGMENT_M, budget,
                           observed['gap_m'] - handoff_m
                           + STRAIGHT_ESCAPE_TOLERANCE_M)
            self.emit('rear_box_coarse_approach_started', gap_m=observed['gap_m'],
                      distance_m=distance, face_latched=False)
            if not self._rear_coarse_step(
                    geometry, extent, corridor_half_width, distance, command_watch,
                    **({'handoff_m': handoff_m} if handoff_m != REAR_COARSE_HANDOFF_M else {})):
                reason = getattr(self, '_last_direct_motion_failure', None)
                if not reason or not self._rear_wait_for_input(reason, command_watch):
                    return False
        return False

    def _rear_wait_for_input(self, reason, command_watch=None, *,
                             preserve_motion_deadline=None):
        if preserve_motion_deadline is None:
            preserve_motion_deadline = reason in {
                'rear box scan unavailable', 'rear box scan stale or future-dated',
            }
        recoverable = {
            'rear box scan unavailable', 'rear box scan stale or future-dated',
            'rear box corridor unavailable', 'rear docking swept footprint occupied',
            *ROTATION_SCAN_INPUT_GAPS,
        }
        if reason not in recoverable and not self._input_gap_recoverable(reason):
            return False
        started_s = time.monotonic()
        initial_received_s = self.samples.get('scan')
        initial_scan = getattr(self, 'last_scan', None)
        initial_stamp_s = None
        try:
            initial_stamp_s = (initial_scan.header.stamp.sec
                               + initial_scan.header.stamp.nanosec * 1e-9)
        except (AttributeError, TypeError, ValueError):
            pass
        self.emit('rear_box_waiting_for_input', reason=reason, phase='coarse')
        expected = Twist()
        expected.linear.x = -0.05
        previous_pause = getattr(self, '_input_recovery_paused', False)
        recovered = False
        paused_s = 0.0
        if preserve_motion_deadline:
            self._input_recovery_paused = True
        try:
            while not self.stop_requested:
                deadline_s = getattr(self, 'run_deadline_s', None)
                if (not preserve_motion_deadline and deadline_s is not None
                        and time.monotonic() >= deadline_s):
                    break
                if preserve_motion_deadline:
                    self._escape_publisher().publish(Twist())
                spin_node(self, timeout_sec=0.05)
                if self.stop_requested:
                    self._last_direct_motion_failure = 'operator_stop'
                    break
                if not self._operator_link_ready():
                    self._last_direct_motion_failure = 'operator_link_lost'
                    break
                guard_failure = self._guard_failure(False)
                if guard_failure:
                    if self._input_gap_recoverable(guard_failure):
                        continue
                    self._last_direct_motion_failure = guard_failure
                    break
                try:
                    _scan, stamp_s = self._rear_box_scan()
                except RuntimeError as error:
                    scan_failure = str(error)
                    if scan_failure in {
                            'rear box scan unavailable',
                            'rear box scan stale or future-dated',
                    }:
                        continue
                    self._last_direct_motion_failure = scan_failure
                    break
                received_s = self.samples.get('scan')
                # Recovery must observe a scan callback from this stopped wait,
                # not merely revalidate the sample which caused the pause.
                if (received_s is None or received_s < started_s
                        or (initial_received_s is not None
                            and received_s <= initial_received_s)
                        or (initial_stamp_s is not None
                            and stamp_s == initial_stamp_s)):
                    continue
                watch_failure = command_watch(expected) if command_watch is not None else None
                if watch_failure:
                    if (watch_failure in recoverable
                            or self._input_gap_recoverable(watch_failure)):
                        continue
                    self._last_direct_motion_failure = watch_failure
                    break
                recovered = True
                break
        finally:
            paused_s = max(0.0, time.monotonic() - started_s)
            if preserve_motion_deadline and recovered:
                deadline_s = getattr(self, 'run_deadline_s', None)
                if (isinstance(deadline_s, (int, float))
                        and not isinstance(deadline_s, bool)
                        and math.isfinite(deadline_s)):
                    self.run_deadline_s = deadline_s + paused_s
            if preserve_motion_deadline:
                self._input_recovery_paused = previous_pause
        if not recovered:
            return False
        self.emit('rear_box_input_recovered', phase='coarse', target_preserved=True,
                  paused_s=paused_s,
                  motion_deadline_preserved=preserve_motion_deadline)
        return True

    def _observe_rear_box(self, geometry, extent, half_width, *, timeout_s=5.0):
        tracker = RearBoxFaceTracker(required_frames=8, max_missing_frames=2)
        observations = []
        last_stamp = None
        failure = 'rear box unstable'
        deadline = min(self.run_deadline_s, time.monotonic() + timeout_s)
        while not self.stop_requested and time.monotonic() < deadline:
            iteration_started_s = time.monotonic()
            previous_pause = getattr(self, '_input_recovery_paused', False)
            # Classify this bounded read before the timer can convert a scan
            # gap into operator_stop. Only an actual input gap refunds time.
            self._input_recovery_paused = True
            try:
                spin_node(self, timeout_sec=0.02)
                if not self._operator_link_ready():
                    raise RuntimeError('operator_link_lost')
                motion = getattr(self, 'latest_motion', None)
                if (motion is None or not 0.0 <= time.monotonic() - motion[0] <= 0.5
                        or abs(motion[1]) >= 0.01 or abs(motion[2]) >= 0.02):
                    tracker.reset()
                    observations.clear()
                    failure = 'rear box waiting for stopped odometry'
                    continue
                try:
                    observation = self._rear_box_observation(geometry, extent, half_width)
                    if observation['stamp_s'] == last_stamp:
                        continue
                    last_stamp = observation['stamp_s']
                    observations.append(observation)
                    observations = observations[-8:]
                    if tracker.update(observation):
                        return _stationary_face(observations, extent, half_width)
                    failure = 'rear box unstable'
                    observations = observations[-tracker.frame_count:] if tracker.frame_count else []
                except (ValueError, RuntimeError) as error:
                    if str(error) in {
                            'rear box scan unavailable',
                            'rear box scan stale or future-dated'}:
                        paused_s = time.monotonic()
                        detection_s = max(0.0, paused_s - iteration_started_s)
                        self.run_deadline_s += detection_s
                        deadline += detection_s
                        if not self._rear_wait_for_input(str(error)):
                            raise RuntimeError(
                                getattr(self, '_last_direct_motion_failure', None)
                                or str(error)) from error
                        deadline += max(0.0, time.monotonic() - paused_s)
                        tracker.reset()
                        observations.clear()
                        continue
                    failed = getattr(self, '_rear_box_detection_failure', None)
                    if str(error) == 'rear box face unavailable' and failed is not None:
                        # Count a missing frame once, not once per polling tick.
                        if failed[0] == last_stamp:
                            continue
                        last_stamp = failed[0]
                        tracker.update(None)
                        observations = (observations[-tracker.frame_count:]
                                        if tracker.frame_count else [])
                    else:
                        tracker.reset()
                        observations.clear()
                    failure = str(error)
            finally:
                self._input_recovery_paused = previous_pause
        raise RuntimeError(f'rear box observation unavailable: {failure}')

    def _observe_rear_box_recovering(self, geometry, extent, half_width, command_watch=None,
                                     *, allow_range_reapproach=False,
                                     range_handoff_m=REAR_FACE_REOBSERVE_GAP_M):
        """Keep a temporary perception gap inside the current return task."""
        range_start = self._odom_pose()
        while not self.stop_requested and time.monotonic() < self.run_deadline_s:
            if not self._operator_link_ready():
                raise RuntimeError('operator_link_lost')
            try:
                return self._observe_rear_box(
                    geometry, extent, half_width,
                    **({'timeout_s': 1.0} if allow_range_reapproach else {}))
            except RuntimeError as error:
                if not str(error).startswith('rear box observation unavailable:'):
                    raise
                if self.stop_requested or time.monotonic() >= self.run_deadline_s:
                    break
                travelled_m = math.dist(range_start[:2], self._odom_pose()[:2])
                range_budget_m = MAX_REVERSE_DISTANCE_M - travelled_m
                if (allow_range_reapproach and range_budget_m > 0.0
                        and str(error).endswith('rear box face unavailable')):
                    # The raw range handoff does not prove a transverse box face.
                    # Reobserve closer on the same axis, bounded by every live return.
                    outline = self._footprint_outline()
                    corridor_width = max(half_width, *(abs(p[1]) for p in outline))
                    try:
                        observed = self._rear_corridor_observation(
                            geometry, extent, corridor_width)
                    except RuntimeError as range_error:
                        if self._rear_wait_for_input(str(range_error), command_watch):
                            continue
                        raise
                    # Keep reversing in one bounded command until a stable face
                    # appears or the live range handoff is reached.  Splitting
                    # this into fixed 20 cm moves caused stop-observe-stop cycles.
                    distance_m = min(range_budget_m,
                                     observed['gap_m'] - range_handoff_m)
                    if distance_m > 0:
                        self.emit('rear_box_range_reobserve', distance_m=distance_m,
                                  gap_m=observed['gap_m'],
                                  handoff_m=range_handoff_m)
                        if not self._rear_coarse_step(
                                geometry, extent, corridor_width, distance_m, command_watch,
                                handoff_m=range_handoff_m,
                                stop_when_face_visible=True,
                                face_half_width_m=half_width):
                            reason = ('operator_stop' if self.stop_requested else
                                      'operator_link_lost' if not self._operator_link_ready() else
                                      getattr(self, '_last_direct_motion_failure', None)
                                      or 'rear box range reobservation blocked')
                            recoverable_sensor_gap = reason in {
                                'rear box scan unavailable',
                                'rear box scan stale or future-dated',
                                'rear box corridor unavailable',
                            }
                            if (recoverable_sensor_gap
                                    and self._rear_wait_for_input(reason, command_watch)):
                                continue
                            raise RuntimeError(reason)
                        continue
                self.emit('rear_box_waiting_for_observation', reason=str(error),
                          remaining_s=max(0., self.run_deadline_s - time.monotonic()))
        raise RuntimeError('operator_stop' if self.stop_requested
                           else 'rear box observation deadline exceeded')

    def _rear_box_update(self, initial, geometry, extent, half_width, target_gap,
                         reference=None, command_watch=None,
                         source_point_filter=None):
        """Track the dock box in odom; live scan guards cover the final approach."""
        state = dict(initial)
        reference = initial if reference is None else reference
        reference = {'face_odom': tuple(reference['face_odom']),
                     'normal_odom': tuple(reference['normal_odom'])}
        start = self._odom_pose()

        def update(actual):
            observation = self._rear_box_observation(geometry, extent, half_width)
            if observation['stamp_s'] < state['stamp_s']:
                raise RuntimeError('rear box scan clock regressed')
            if observation['stamp_s'] != state['stamp_s']:
                _same_target(reference, observation)
                _set_collision_target(
                    command_watch, observation, target_gap,
                    observation['gap_m'], source_point_filter)
                old_heading = math.atan2(state['normal_odom'][1], state['normal_odom'][0])
                raw_heading = math.atan2(observation['normal_odom'][1],
                                         observation['normal_odom'][0])
                elapsed = observation['stamp_s'] - state['stamp_s']
                heading = filtered_face_heading(
                    old_heading, raw_heading, elapsed)
                old_offset = sum(state['normal_odom'][i] * state['face_odom'][i]
                                 for i in (0, 1))
                raw_offset = sum(observation['normal_odom'][i] * observation['face_odom'][i]
                                 for i in (0, 1))
                alpha = 1.0 - math.exp(-elapsed / FACE_HEADING_FILTER_S)
                offset = old_offset + alpha * (raw_offset - old_offset)
                state.update(observation)
                state['normal_odom'] = math.cos(heading), math.sin(heading)
                state['face_odom'] = tuple(offset*n for n in state['normal_odom'])
                self.emit('rear_box_face_updated', observation_stamp_s=state['stamp_s'],
                          gap_m=observation['gap_m'],
                          angle_rad=observation['normal_yaw_error_rad'])
            normal = state['normal_odom']
            heading = math.atan2(normal[1], normal[0])
            error = _angle(heading - actual[2])
            center_gap = (sum(
                (actual[i] - state['face_odom'][i]) * normal[i] for i in (0, 1))
                - extent * math.cos(error))
            gap = center_gap - half_width * abs(math.sin(error))
            if abs(error) > math.radians(30.0):
                raise RuntimeError('rear box heading outside approach range')
            # Center the two rear-corner gaps on the requested distance. Using
            # the nearest corner alone systematically leaves the far corner
            # long whenever a small residual yaw remains.
            remaining = (center_gap - target_gap) / math.cos(error)
            # A fresh same-face fit can already satisfy both final corners while
            # the filtered plane still asks for more travel. Finish this move;
            # the caller will confirm again from stopped scans.
            raw_normal = observation['normal_odom']
            raw_error = _angle(math.atan2(raw_normal[1], raw_normal[0]) - actual[2])
            raw_gap = (sum((actual[i] - observation['face_odom'][i]) * raw_normal[i]
                           for i in (0, 1)) - extent * math.cos(raw_error)
                       - half_width * abs(math.sin(raw_error)))
            target_confirmed = _rear_gap_confirmed(
                raw_gap, raw_error, half_width, target_gap)
            if target_confirmed:
                remaining = 0.
            if math.dist(start[:2], actual[:2]) + max(0., remaining) > 0.6:
                raise RuntimeError('rear box approach exceeds bounded travel')
            return {'remaining_m': remaining, 'heading_rad': heading,
                    'target_confirmed': target_confirmed,
                    'nearest_gap_m': gap,
                    'center_gap_m': center_gap,
                    'farthest_gap_m': center_gap + half_width * abs(math.sin(error)),
                    'allow_near_alignment': (gap > 0. and abs(error) < math.atan2(
                        half_width, extent))}

        return update

    def park_rear_box(self, geometry, route_path, *, execute, timeout_s,
                      target_gap_m=None):
        """Align to a live rear box and stop at the requested one-centimetre gap."""
        for key in ('frame_length', 'front_to_wheel_axis', 'frame_width'):
            item = geometry[key]
            if (item['unit'] != 'm' or isinstance(item['value'], bool)
                    or not isinstance(item['value'], (int, float))
                    or not math.isfinite(item['value']) or item['value'] <= 0):
                raise ValueError('invalid rear box chassis geometry')
        extent = geometry['frame_length']['value'] - geometry['front_to_wheel_axis']['value']
        half_width = geometry['frame_width']['value'] / 2.0
        if not all(math.isfinite(v) and v > 0 for v in (extent, half_width)):
            raise ValueError('invalid rear box chassis geometry')
        target_gap = self._final_gap_m() if target_gap_m is None else target_gap_m
        if (isinstance(target_gap, bool)
                or not isinstance(target_gap, (int, float))
                or not math.isfinite(target_gap)
                or not MINIMUM_FRONT_GAP_M <= target_gap <= MAXIMUM_FRONT_GAP_M):
            raise ValueError('target_front_gap_m must be in [0.01, 0.10] m')
        self.run_deadline_s = time.monotonic() + timeout_s
        self.table_id, self.pose_id = 'home_dock', 'rear_box'
        # A staging confirmation from the preceding home leg must not be reused as
        # evidence for this independent live-LiDAR terminal confirmation.
        self.confirmation = None
        self.verify_live_maps()
        if not self.wait_until_ready(timeout=10.0):
            self.emit('failed', phase='rear_box_readiness',
                      reason=self.readiness_failure_reason())
            return False
        if not execute:
            self.capture_stationary_pose()
            initial = self._observe_rear_box(geometry, extent, half_width)
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            return True
        if not self._reverse_smoother_ready() or not self._parking_parameters_ready(reverse=True):
            return False
        self._confirm_collision_monitor_on()
        if not self._approach_rear_corridor(
                geometry, extent, half_width, handoff_m=REAR_COARSE_HANDOFF_M):
            return False
        initial = self._observe_rear_box_recovering(
            geometry, extent, half_width, allow_range_reapproach=True)
        if getattr(self, '_rear_dock_reference', None) is not None:
            with self._localization_bound(True):
                if not self._align_rear_reference(
                        self._rear_reference_stage({'id': 'home_dock_approach'}),
                        self.registry['home'], observed=initial, return_to_stage=False):
                    return False
        # The registered approach point supplies lateral position.  Once the
        # face is visible, only heading and straight reverse distance are
        # corrected; curved centering caused the repeated wide approach.
        entry_face = initial
        fixed_midpoint = entry_face['midpoint_odom']
        fixed_normal = entry_face['normal_odom']
        tangent = -fixed_normal[1], fixed_normal[0]
        initial = self._align_rear_face_before_clearance(
            initial, geometry, extent, half_width)
        _same_target(entry_face, initial)
        if self.stop_requested or time.monotonic() >= self.run_deadline_s:
            return False
        actual = self._odom_pose()
        lateral = sum((actual[i] - fixed_midpoint[i]) * tangent[i] for i in (0, 1))
        center_inside_face = abs(lateral) <= initial['span_m'] / 2.
        self.emit('rear_dock_straight_entry_ready', lateral_error_m=lateral,
                  center_inside_face=center_inside_face,
                  angle_rad=initial['normal_yaw_error_rad'])
        if abs(lateral) > REAR_CENTER_TOLERANCE_M:
            self.emit('rear_dock_lateral_precision_unmet', lateral_error_m=lateral,
                      tolerance_m=REAR_CENTER_TOLERANCE_M,
                      continued_with_live_face=center_inside_face)
        if not center_inside_face:
            self.emit('failed', phase='rear_box_centering',
                      reason='registered dock approach is outside observed face',
                      lateral_error_m=lateral, forward_retry_requested=False)
            return False
        if abs(initial['normal_yaw_error_rad']) > math.radians(3.):
            self.emit('failed', phase='rear_box_centering',
                      reason='rear box final entry heading exceeds tolerance',
                      forward_retry_requested=False)
            return False
        with rear_collision_profile(self) as command_watch:
            self.emit('rear_box_observed', observation=initial, target_gap_m=target_gap)
            # Landmark localization needs visible endpoints. After latching the
            # plane, near-range clipping must not replace plane feedback with
            # a requirement to see the complete box width again.
            self._rear_fine_face = initial
            try:
                return self._park_rear_face(
                    initial, geometry, route_path, extent, half_width, target_gap, command_watch)
            finally:
                _clear_collision_target(command_watch)
                self._rear_fine_face = None

    def _park_rear_face(self, initial, geometry, route_path, extent, half_width,
                        target_gap, command_watch):
        """Keep the fine target fixed after the moving range-only handoff."""

        configure_rear = getattr(command_watch, 'set_rear_extent', None)
        if callable(configure_rear):
            configure_rear(extent)
        source_cache = {}

        def source_goal_face_point(pose, scan, point):
            """Classify memory in its acquisition frame, before odom drift."""
            key = id(scan)
            acquisition_pose = tuple(pose)
            cached = source_cache.get(key)
            if (cached is None or cached[0] is not scan
                    or cached[1] != acquisition_pose):
                try:
                    observed = _observe_rear_face_roi(
                        self._rear_scan_values(scan), geometry, extent,
                        half_width)
                    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
                    x_m, y_m = observed['face_center_base_xy_m']
                    nx, ny = observed['normal_base_xy']
                    source = {
                        **observed,
                        'face_odom': (
                            pose[0] + cosine * x_m - sine * y_m,
                            pose[1] + sine * x_m + cosine * y_m),
                        'normal_odom': (
                            cosine * nx - sine * ny,
                            sine * nx + cosine * ny),
                    }
                    _same_target(initial, source)
                    source_cache[key] = (scan, acquisition_pose, observed)
                except (KeyError, RuntimeError, TypeError, ValueError):
                    source_cache[key] = (scan, acquisition_pose, None)
            observed = source_cache[key][2]
            if observed is None:
                return False
            normal = observed['normal_base_xy']
            tangent = observed['tangent_base_xy']
            residual_m = (point[0] * normal[0] + point[1] * normal[1]
                          - observed['line_offset_m'])
            tangent_m = point[0] * tangent[0] + point[1] * tangent[1]
            lower_m, upper_m = observed['tangent_bounds_m']
            residual_limit_m = min(
                4.0 * observed['residual_rms_m'], MAXIMUM_RESIDUAL_RMS_M)
            near_limit_m = min(residual_limit_m, 0.003)
            if not (-residual_limit_m <= residual_m <= near_limit_m
                    and lower_m <= tangent_m <= upper_m):
                return False
            # A partial fit may legitimately see only the near-range portion of
            # the latched dock face. It is still the target only where its
            # verified source segment overlaps the original target segment.
            try:
                cosine, sine = math.cos(pose[2]), math.sin(pose[2])
                point_odom = (
                    pose[0] + cosine * point[0] - sine * point[1],
                    pose[1] + sine * point[0] + cosine * point[1])
                reference_point = to_base_frame(
                    initial['base_pose_odom'], (*point_odom, 0.0))
                reference_tangent = initial['tangent_base_xy']
                reference_tangent_m = sum(
                    reference_point[i] * reference_tangent[i] for i in (0, 1))
                reference_lower_m, reference_upper_m = initial['tangent_bounds_m']
                reference_normal = initial['normal_base_xy']
                reference_residual_m = (
                    sum(reference_point[i] * reference_normal[i] for i in (0, 1))
                    - initial['line_offset_m'])
                reference_residual_limit_m = min(
                    4.0 * (initial['residual_rms_m'] + observed['residual_rms_m']),
                    MAXIMUM_RESIDUAL_RMS_M)
            except (KeyError, TypeError, ValueError):
                return False
            # The source-frame check above already excludes returns in front
            # of that measured face. Across two fits, add their residual bounds;
            # applying the final-gap near-side limit again in odom reclassifies
            # the same box as an obstacle after a small registration difference.
            # Keep the target identity, overlapping segment and residual cap.
            return (
                abs(reference_residual_m) <= reference_residual_limit_m
                and reference_lower_m <= reference_tangent_m <= reference_upper_m)

        def approach(observation):
            _set_collision_target(
                command_watch, observation, target_gap, observation['gap_m'],
                source_goal_face_point)
            update = self._rear_box_update(
                observation, geometry, extent, half_width, target_gap,
                reference=initial, command_watch=command_watch,
                source_point_filter=source_goal_face_point)
            remaining = update(self._odom_pose())['remaining_m']
            if remaining <= 0.0:
                return True
            kwargs = {} if command_watch is None else {'command_watch': command_watch}
            return self._drive_zero_turn(
                -remaining, precision=True, hold_heading=True,
                target_update=update, approach_time_s=FACE_APPROACH_TIME_S, **kwargs) is not None

        # Final feedback owns one reverse approach. A near-range residual must
        # not trigger a new forward clearance or replay the parking manoeuvre.
        if not approach(initial):
            return False

        final = self._observe_rear_box_recovering(geometry, extent, half_width, command_watch)
        _same_target(initial, final)

        def confirmation(observation):
            error = observation['normal_yaw_error_rad']
            gap = observation['gap_m']
            far_gap = gap + 2.0 * half_width * abs(math.sin(error))
            confirmed = _rear_gap_confirmed(gap, error, half_width, target_gap)
            return confirmed, gap, far_gap, error

        confirmed, gap, far_gap, error = confirmation(final)
        # The feedback drive can stop a few millimetres early while braking.
        # Finish that residual inside this request instead of requiring a new
        # coarse approach.  Keep this to one reverse-only trim; a larger miss
        # or a face-angle miss still returns through the existing failure path.
        center_gap = gap + half_width * abs(math.sin(error))
        trim_m = (center_gap - target_gap) / math.cos(error)
        trim_allowed = (
            not confirmed
            and gap > target_gap
            and far_gap > target_gap + 0.003
            and 0.0 < trim_m <= REAR_FINAL_TRIM_MAX_M
            and far_gap - gap <= 0.006
            and not self.stop_requested
            and self._operator_link_ready()
        )
        if trim_allowed:
            self.emit('rear_box_final_trim_started', requested_distance_m=trim_m,
                      nearest_gap_m=gap, farthest_gap_m=far_gap,
                      angle_rad=error)
            if approach(final):
                final = self._observe_rear_box_recovering(
                    geometry, extent, half_width, command_watch)
                _same_target(initial, final)
                confirmed, gap, far_gap, error = confirmation(final)
                self.emit('rear_box_final_trim_finished', confirmed=confirmed,
                          nearest_gap_m=gap, farthest_gap_m=far_gap,
                          angle_rad=error)
        self.confirmation = {
            'confirmed': confirmed,
            'reason': ('confirmed' if confirmed
                       else 'rear_box_gap_out_of_tolerance'),
            'reference': 'live_rear_lidar_face',
            'angle_rad': error,
            'corner_gap_span_m': far_gap - gap,
            'maximum_corner_gap_span_m': 0.006,
            'nearest_gap_m': gap,
            'farthest_gap_m': far_gap,
            'internal_estimate_only': True,
            'external_accuracy_verified': False,
        }
        self.emit('rear_box_parking_finished', confirmed=confirmed,
                  nearest_gap_m=gap, farthest_gap_m=far_gap, angle_rad=error,
                  target_gap_m=target_gap, basis='live_rear_lidar_face',
                  internal_estimate_only=True, external_accuracy_verified=False)
        return confirmed
