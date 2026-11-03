"""Scoped Collision Monitor override for a live rear docking target.

The dock face is an intentional obstacle.  During the bounded rear approach,
``FootprintApproach`` would therefore predict the requested one-centimetre stop
as a collision.  This profile leaves the directional ``StopZone`` enabled and
replaces only that predictive action with the same live scan, footprint and
remembered blind-range points.
"""

from contextlib import contextmanager
import math
import time

from jdamr_cube_navigation.box_lidar_witness import MAXIMUM_RESIDUAL_RMS_M
from jdamr_cube_navigation.corridor_route import spin_node
from jdamr_cube_navigation.reverse_parking import path_sweep_hit, scan_points
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


SCAN_MAX_AGE_S = 0.5
PARAMETER_TIMEOUT_S = 2.0
PREDICTION_STEP_S = 0.05
# Keep these synchronized with the direct-motion guard and stop latency.  This
# module cannot import restaurant_service: rear_box_service is loaded before it.
DIRECT_GUARD_PERIOD_S = 0.1
DIRECT_STOP_LATENCY_S = 0.05
EXPECTED_LINEAR_DECEL_MPS2 = 0.5
TARGET_GAP_TOLERANCE_M = 0.003


def _wait_future(node, future, timeout_s=PARAMETER_TIMEOUT_S):
    """Wait through operator stop so a parameter restoration can still finish."""
    if not hasattr(future, 'done'):
        return future
    deadline_s = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline_s:
        spin_node(node, timeout_sec=0.05)
    if not future.done():
        try:
            future.cancel()
        except (AttributeError, RuntimeError):
            pass
        raise RuntimeError('rear collision parameter request timed out')
    if future.exception() is not None:
        raise RuntimeError(
            f'rear collision parameter request failed: {future.exception()}')
    return future.result()


def _parameter_client(node, attribute, remote_node):
    client = getattr(node, attribute, None)
    if client is None:
        client = AsyncParameterClient(node, remote_node)
        setattr(node, attribute, client)
    if not client.wait_for_services(timeout_sec=PARAMETER_TIMEOUT_S):
        raise RuntimeError(f'{remote_node} parameter service unavailable')
    return client


def _read_parameters(node, client, names):
    response = _wait_future(node, client.get_parameters(list(names)))
    values = getattr(response, 'values', None)
    if values is None or len(values) != len(names):
        raise RuntimeError('rear collision parameter response is incomplete')
    return {
        name: parameter_value_to_python(value)
        for name, value in zip(names, values)
    }


def _set_footprint_approach(node, client, enabled, expected_stop_zone=True):
    response = _wait_future(node, client.set_parameters_atomically([
        Parameter('FootprintApproach.enabled', value=bool(enabled)),
    ]))
    result = getattr(response, 'result', None)
    if result is None or result.successful is not True:
        reason = getattr(result, 'reason', '') if result is not None else ''
        raise RuntimeError(reason or 'FootprintApproach parameter change rejected')
    confirmed = _read_parameters(
        node, client, ('StopZone.enabled', 'FootprintApproach.enabled'))
    if confirmed['StopZone.enabled'] is not expected_stop_zone:
        raise RuntimeError('rear collision profile lost StopZone')
    if confirmed['FootprintApproach.enabled'] is not bool(enabled):
        raise RuntimeError('FootprintApproach parameter change not confirmed')


def _fail_closed(node, reason):
    """Latch the existing stop paths without hiding the parameter failure."""
    try:
        node.engage_emergency_stop(reason)
    except Exception:  # The original restoration error remains the caller's evidence.
        pass
    try:
        node.request_stop()
    except Exception:
        pass


def _stamp_s(scan):
    stamp = scan.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _prediction_poses(linear_mps, angular_radps, horizon_s):
    count = max(1, math.ceil(horizon_s / PREDICTION_STEP_S))
    poses = [(0.0, 0.0, 0.0)]
    for index in range(1, count + 1):
        elapsed_s = horizon_s * index / count
        yaw = angular_radps * elapsed_s
        if abs(angular_radps) <= 1e-9:
            poses.append((linear_mps * elapsed_s, 0.0, yaw))
            continue
        radius_m = linear_mps / angular_radps
        poses.append((radius_m * math.sin(yaw),
                      radius_m * (1.0 - math.cos(yaw)), yaw))
    return poses


def _larger_live_velocity(command, latest_motion, now_s):
    linear = float(command.linear.x)
    angular = float(command.angular.z)
    if not all(math.isfinite(value) for value in (linear, angular)):
        raise ValueError('rear collision command is not finite')
    if latest_motion is None or len(latest_motion) != 3:
        return linear, angular
    received_s, actual_linear, actual_angular = latest_motion
    if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value)
                for value in (received_s, actual_linear, actual_angular))
            or not 0.0 <= now_s - received_s <= SCAN_MAX_AGE_S):
        return linear, angular
    if abs(actual_linear) > abs(linear):
        # CorridorRoute stores planar odometry speed as a magnitude.  Preserve
        # the requested direction while accounting for the larger live speed.
        linear = math.copysign(abs(actual_linear), linear or 1.0)
    if abs(actual_angular) > abs(angular):
        angular = actual_angular
    return linear, angular


def _target_context(observation, target_gap_m, current_gap_m,
                    source_point_filter=None):
    """Freeze one verified raw rear-face fit for the current scan only."""
    if not isinstance(observation, dict):
        raise ValueError('rear collision target observation must be a mapping')
    try:
        normal = tuple(float(value) for value in observation['normal_base_xy'])
        tangent = tuple(float(value) for value in observation['tangent_base_xy'])
        bounds = tuple(float(value) for value in observation['tangent_bounds_m'])
        offset_m = float(observation['line_offset_m'])
        residual_rms_m = float(observation['residual_rms_m'])
        stamp_s = float(observation['stamp_s'])
        target_gap_m = float(target_gap_m)
        current_gap_m = float(current_gap_m)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('rear collision target geometry is incomplete') from error
    values = (*normal, *tangent, *bounds, offset_m, residual_rms_m,
              stamp_s, target_gap_m, current_gap_m)
    if (len(normal) != 2 or len(tangent) != 2 or len(bounds) != 2
            or not all(math.isfinite(value) for value in values)
            or not math.isclose(math.hypot(*normal), 1.0, abs_tol=1e-3)
            or not math.isclose(math.hypot(*tangent), 1.0, abs_tol=1e-3)
            or abs(normal[0] * tangent[0] + normal[1] * tangent[1]) > 1e-3
            or bounds[0] > bounds[1]
            or not 0.0 <= residual_rms_m <= MAXIMUM_RESIDUAL_RMS_M
            or target_gap_m < 0.0):
        raise ValueError('rear collision target geometry is invalid')
    return {
        'normal': normal,
        'tangent': tangent,
        'tangent_bounds_m': bounds,
        'line_offset_m': offset_m,
        'residual_limit_m': min(
            4.0 * residual_rms_m, MAXIMUM_RESIDUAL_RMS_M),
        # Near-side returns cannot consume more than the existing final-gap
        # tolerance, even when a rough face has an accepted larger fit RMS.
        'near_residual_limit_m': min(
            4.0 * residual_rms_m, TARGET_GAP_TOLERANCE_M),
        'stamp_s': stamp_s,
        'target_gap_m': target_gap_m,
        'current_gap_m': current_gap_m,
        'source_point_filter': source_point_filter,
    }


def _may_ignore_target(command, target, stamp_s):
    if target is None or not math.isclose(
            target['stamp_s'], stamp_s, rel_tol=0.0, abs_tol=1e-9):
        return False
    linear_mps = float(command.linear.x)
    angular_radps = float(command.angular.z)
    if linear_mps < -1e-12:
        return target['current_gap_m'] >= (
            target['target_gap_m'] - TARGET_GAP_TOLERANCE_M)
    # At contact the already-supported straight departure is allowed only
    # while its translation increases distance from the rear target.
    return (linear_mps > 1e-12 and abs(angular_radps) <= 1e-12
            and linear_mps * target['normal'][0] > 0.0)


def _is_target_face_point(point, target):
    normal = target['normal']
    tangent = target['tangent']
    residual_m = (
        point[0] * normal[0] + point[1] * normal[1]
        - target['line_offset_m'])
    tangent_m = point[0] * tangent[0] + point[1] * tangent[1]
    lower_m, upper_m = target['tangent_bounds_m']
    return (-target['residual_limit_m'] <= residual_m
            <= target['near_residual_limit_m']
            and lower_m <= tangent_m <= upper_m)


def _point_inside_outline(point, outline):
    """Return whether point is strictly inside a possibly stepped outline."""
    x_m, y_m = point
    inside = False
    previous = outline[-1]
    for current in outline:
        ax, ay = previous
        bx, by = current
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        fraction = 0.0 if length2 == 0.0 else max(
            0.0, min(1.0, ((x_m - ax) * dx + (y_m - ay) * dy) / length2))
        if math.hypot(
                x_m - ax - fraction * dx,
                y_m - ay - fraction * dy) <= 1e-9:
            return False
        if ((ay > y_m) != (by > y_m)
                and x_m < (bx - ax) * (y_m - ay) / (by - ay) + ax):
            inside = not inside
        previous = current
    return inside


def _impossible_turned_memory_projection(point, source, chassis_outline):
    """Identify an old external return now projected inside the physical chassis."""
    try:
        source_pose, _scan, source_point, current_pose = source
        heading_change = abs(math.atan2(
            math.sin(current_pose[2] - source_pose[2]),
            math.cos(current_pose[2] - source_pose[2])))
    except (IndexError, TypeError, ValueError):
        return False
    return (heading_change > math.radians(5.0)
            and not _point_inside_outline(source_point, chassis_outline)
            and _point_inside_outline(point, chassis_outline))


class _RearCollisionWatch:
    """Callable direct-motion guard with a scoped fine-target exception."""

    def __init__(self, node, state, linear_decel_mps2, angular_decel_radps2):
        self._node = node
        self._state = state
        self._linear_decel_mps2 = linear_decel_mps2
        self._angular_decel_radps2 = angular_decel_radps2

    def __call__(self, command):
        return _watch(
            self._node, command, self._state, self._linear_decel_mps2,
            self._angular_decel_radps2)

    def set_target(self, observation, target_gap_m, current_gap_m,
                   source_point_filter=None):
        self._state['target'] = _target_context(
            observation, target_gap_m, current_gap_m, source_point_filter)
        self._state['filtered_memory_key'] = None

    def set_rear_extent(self, rear_extent_m):
        """Use the measured rear edge only for fresh terminal reverse targets."""
        outline = self._state['outline']
        padded_extent = -min(point[0] for point in outline)
        if (isinstance(rear_extent_m, bool)
                or not isinstance(rear_extent_m, (int, float))
                or not math.isfinite(rear_extent_m)
                or rear_extent_m <= 0.0
                or not -1e-9 <= padded_extent - rear_extent_m <= .005 + 1e-9):
            raise ValueError('rear docking footprint differs from measured rear edge')
        self._state['terminal_outline'] = tuple(
            (max(x, -rear_extent_m), y) for x, y in outline)

    def clear_target(self):
        self._state['target'] = None
        self._state['filtered_memory'] = ()
        self._state['filtered_memory_key'] = None

    @property
    def target_active(self):
        return self._state['target'] is not None


def _watch(node, command, state, linear_decel_mps2, angular_decel_radps2):
    scan_context = state.get('scan_context', 'rear box')
    collision_context = state.get('collision_context', 'rear collision')
    requested = (float(command.linear.x), float(command.angular.z))
    if not all(math.isfinite(value) for value in requested):
        return f'{collision_context} command invalid'
    # A zero command must always pass so _direct_motion can stop in finally.
    if abs(requested[0]) <= 1e-12 and abs(requested[1]) <= 1e-12:
        return None

    scan = getattr(node, 'last_scan', None)
    received_s = (getattr(node, 'samples', None) or {}).get('scan')
    if scan is None or received_s is None:
        return f'{scan_context} scan unavailable'
    now_s = time.monotonic()
    receive_age_s = now_s - received_s
    try:
        stamp_s = _stamp_s(scan)
        ros_age_s = node.get_clock().now().nanoseconds * 1e-9 - stamp_s
    except (AttributeError, TypeError, ValueError, OverflowError):
        return f'{collision_context} scan timestamp invalid'
    if (not all(math.isfinite(value) for value in (receive_age_s, ros_age_s))
            or not 0.0 <= receive_age_s <= SCAN_MAX_AGE_S
            or not 0.0 <= ros_age_s <= SCAN_MAX_AGE_S):
        last_emit_s = state.get('last_scan_latency_emit_s', -math.inf)
        if now_s - last_emit_s >= 1.0:
            try:
                node.emit(
                    'rear_scan_latency_breakdown',
                    receive_age_s=receive_age_s, ros_age_s=ros_age_s,
                    scan_input_timing=dict(
                        getattr(node, 'scan_input_timing', None) or {}))
                state['last_scan_latency_emit_s'] = now_s
            except (AttributeError, TypeError, ValueError, RuntimeError):
                pass
        return f'{scan_context} scan stale or future-dated'
    if scan.header.frame_id != state['frame_id']:
        return f'{collision_context} scan frame changed'

    scan_key = (id(scan), stamp_s)
    if scan_key != state['scan_key']:
        try:
            state['memory'] = tuple(node._remembered_blind_points(state['laser'], scan))
            state['memory_sources'] = tuple(getattr(node, '_blind_point_sources', ()))
            state['memory_diagnostics'] = tuple(
                getattr(node, '_blind_point_diagnostics', ()))
        except (AttributeError, RuntimeError, ValueError):
            return f'{collision_context} blind-range memory unavailable'
        state['scan_key'] = scan_key
        state['filtered_memory_key'] = None
    try:
        linear_mps, angular_radps = _larger_live_velocity(
            command, getattr(node, 'latest_motion', None), now_s)
        reaction_s = (max(receive_age_s, ros_age_s)
                      + DIRECT_GUARD_PERIOD_S + DIRECT_STOP_LATENCY_S)
        horizon_s = reaction_s + max(
            abs(linear_mps) / linear_decel_mps2,
            abs(angular_radps) / angular_decel_radps2)
        live_points = scan_points(scan, state['laser'])
        remembered_points = state['memory']
        remembered_sources = state.get('memory_sources', ())
        remembered_diagnostics = state.get('memory_diagnostics', ())
        if len(remembered_diagnostics) != len(remembered_points):
            remembered_diagnostics = (None,) * len(remembered_points)
        target = state.get('target')
        outline = state['outline']
        if _may_ignore_target(command, target, stamp_s):
            # Nav2's polygon embeds 5 mm beyond the measured chassis. At the
            # final centimetre use its actual rear edge, without deleting any
            # obstacle returns or changing the wheel/side/front envelope.
            if (command.linear.x < 0.0
                    and target['current_gap_m'] <= .03
                    and 'terminal_outline' in state):
                outline = state['terminal_outline']
            live_points = [point for point in live_points
                           if not _is_target_face_point(point, target)]
            source_filter = target.get('source_point_filter')
            filtered_memory_key = (scan_key, id(source_filter))
            if state['filtered_memory_key'] != filtered_memory_key:
                filtered_memory = remembered_points
                filtered_sources = remembered_sources
                filtered_diagnostics = remembered_diagnostics
                if callable(source_filter):
                    if len(remembered_sources) == len(remembered_points):
                        kept = []
                        for point, source, diagnostic in zip(
                                remembered_points, remembered_sources,
                                remembered_diagnostics):
                            pose, source_scan, source_point, _current_pose = source
                            if not source_filter(pose, source_scan, source_point):
                                kept.append((point, source, diagnostic))
                        filtered_memory = tuple(
                            point for point, _source, _diagnostic in kept)
                        filtered_sources = tuple(
                            source for _point, source, _diagnostic in kept)
                        filtered_diagnostics = tuple(
                            diagnostic for _point, _source, diagnostic in kept)
                    else:
                        # Older/custom providers may not expose provenance. Preserve
                        # fail-closed behavior by recomputing instead of dropping an
                        # unverified remembered obstacle.
                        filtered_memory = tuple(node._remembered_blind_points(
                            state['laser'], scan,
                            source_point_filter=lambda pose, source_scan, point: not
                            source_filter(pose, source_scan, point)))
                        filtered_sources = tuple(
                            getattr(node, '_blind_point_sources', ()))
                        filtered_diagnostics = tuple(
                            getattr(node, '_blind_point_diagnostics', ()))
                state['filtered_memory'] = filtered_memory
                state['filtered_sources'] = filtered_sources
                state['filtered_diagnostics'] = filtered_diagnostics
                state['filtered_memory_key'] = filtered_memory_key
            remembered_points = state['filtered_memory']
            remembered_sources = state['filtered_sources']
            remembered_diagnostics = state['filtered_diagnostics']
            if len(remembered_diagnostics) != len(remembered_points):
                remembered_diagnostics = (None,) * len(remembered_points)
            # A return acquired outside the chassis before a substantial turn
            # can be projected inside the measured body by accumulated odometry
            # error. It cannot describe a collision that the next reverse step
            # will newly create. Drop only that provenance-backed impossible
            # projection; live returns and every point outside the physical
            # chassis continue through the swept-footprint guard.
            if ('terminal_outline' in state
                    and len(remembered_sources) == len(remembered_points)):
                kept = [
                    (point, source, diagnostic)
                    for point, source, diagnostic in zip(
                        remembered_points, remembered_sources,
                        remembered_diagnostics)
                    if not _impossible_turned_memory_projection(
                        point, source, state['terminal_outline'])
                ]
                remembered_points = tuple(item[0] for item in kept)
                remembered_sources = tuple(item[1] for item in kept)
                remembered_diagnostics = tuple(item[2] for item in kept)
        points = [*live_points, *remembered_points]
        poses = _prediction_poses(linear_mps, angular_radps, horizon_s)
        witness = {}
        if path_sweep_hit(
                points, outline, poses, push_tolerance_m=0.0, witness=witness):
            if not state['collision_active']:
                live_hit = witness['point_index'] < len(live_points)
                source = 'live_scan' if live_hit else 'remembered_blind_point'
                origin = None
                clearance_diagnostic = None
                index = witness['point_index'] - len(live_points)
                if not live_hit and 0 <= index < len(remembered_sources):
                    pose, source_scan, point, current_pose = remembered_sources[index]
                    origin = {
                        'pose_odom': list(pose), 'point_base_xy_m': list(point),
                        'current_pose_odom': list(current_pose),
                        'laser_pose_base': list(state['laser']),
                        'scan_stamp_s': _stamp_s(source_scan),
                        'angle_min': source_scan.angle_min,
                        'angle_increment': source_scan.angle_increment,
                        'range_min': source_scan.range_min, 'range_max': source_scan.range_max,
                        'ranges': [float(r) if math.isfinite(r) else None
                                   for r in source_scan.ranges],
                    }
                    if index < len(remembered_diagnostics):
                        clearance_diagnostic = remembered_diagnostics[index]
                node.emit(
                    'rear_collision_witness', source=source,
                    point_base_xy_m=witness['point_base_xy_m'], scan_stamp_s=stamp_s,
                    remembered_source=origin,
                    remembered_clearance_diagnostic=clearance_diagnostic,
                    live_point_count=len(live_points),
                    remembered_point_count=len(remembered_points),
                    target_face_filter_active=target is not None,
                    requested_linear_mps=requested[0],
                    requested_angular_radps=requested[1],
                    inside_initial_outline=witness.get('inside_initial_outline'),
                    collision_pose_base_xyyaw=witness.get('collision_pose_base_xyyaw'),
                    initial_boundary_distance_m=witness.get('initial_boundary_distance_m'),
                    footprint_base_xy_m=outline,
                    prediction_horizon_s=horizon_s,
                    effective_linear_mps=linear_mps,
                    effective_angular_radps=angular_radps)
            state['collision_active'] = True
            return state.get('collision_reason', 'rear docking swept footprint occupied')
        state['collision_active'] = False
    except (AttributeError, TypeError, ValueError, OverflowError):
        return f'{collision_context} scan geometry invalid'
    return None


def _motion_collision_watch(node, snapshot, *, rotation=False):
    scan, laser, outline, memory = snapshot
    if not outline:
        raise RuntimeError('rotation collision footprint unavailable' if rotation
                           else 'rear collision footprint unavailable')
    smoother = _parameter_client(
        node, 'rear_collision_smoother_parameters', 'velocity_smoother')
    max_decel = _read_parameters(node, smoother, ('max_decel',))['max_decel']
    if (not isinstance(max_decel, (list, tuple)) or len(max_decel) < 3
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in (max_decel[0], max_decel[2]))):
        raise RuntimeError('velocity_smoother max_decel is invalid')
    linear_decel_mps2, angular_decel_radps2 = abs(max_decel[0]), abs(max_decel[2])
    if (not math.isclose(linear_decel_mps2, EXPECTED_LINEAR_DECEL_MPS2,
                         rel_tol=0.0, abs_tol=1e-9)
            or angular_decel_radps2 <= 0.0):
        raise RuntimeError('velocity_smoother deceleration does not match direct motion')

    state = {
        'laser': tuple(laser), 'outline': tuple(tuple(point) for point in outline),
        'memory': tuple(memory), 'frame_id': scan.header.frame_id,
        'memory_sources': tuple(getattr(node, '_blind_point_sources', ())),
        'memory_diagnostics': tuple(getattr(node, '_blind_point_diagnostics', ())),
        'scan_key': (id(scan), _stamp_s(scan)),
        'target': None,
        'filtered_memory': (), 'filtered_sources': (), 'filtered_diagnostics': (),
        'filtered_memory_key': None,
        'collision_active': False,
    }
    if rotation:
        state.update(scan_context='rotation', collision_context='rotation collision',
                     collision_reason='rotation swept footprint occupied')
    return _RearCollisionWatch(node, state, linear_decel_mps2, angular_decel_radps2)


def rotation_collision_watch(node):
    """Watch live and remembered blind-range points without changing CM settings."""
    snapshot = node._scan_in_base()
    if snapshot is None:
        raise RuntimeError('rotation scan unavailable')
    return _motion_collision_watch(node, snapshot, rotation=True)


@contextmanager
def rear_collision_profile(node):
    """Disable only FootprintApproach while preserving an equivalent live guard."""
    snapshot = node._scan_in_base()
    if snapshot is None:
        raise RuntimeError('rear box scan unavailable')
    scan, laser, outline, memory = snapshot
    if not outline:
        raise RuntimeError('rear collision footprint unavailable')

    monitor = _parameter_client(
        node, 'precision_parameters', 'collision_monitor')
    enabled = _read_parameters(
        node, monitor, ('StopZone.enabled', 'FootprintApproach.enabled'))
    if enabled['StopZone.enabled'] is not True:
        raise RuntimeError('rear collision profile requires StopZone enabled')
    if type(enabled['FootprintApproach.enabled']) is not bool:
        raise RuntimeError('FootprintApproach enabled state is invalid')
    if enabled['FootprintApproach.enabled'] is not True:
        try:
            _set_footprint_approach(node, monitor, True)
        except RuntimeError:
            _fail_closed(
                node, 'FootprintApproach could not be restored before rear docking')
            raise
        node.emit('rear_collision_profile_recovered',
                  footprint_approach_enabled=True, stop_zone_enabled=True)
    if getattr(node, 'stop_requested', False):
        raise RuntimeError('operator_stop')

    watch = _motion_collision_watch(node, snapshot)
    try:
        _set_footprint_approach(node, monitor, False)
        yield watch
    finally:
        watch.clear_target()
        try:
            _set_footprint_approach(node, monitor, True)
        except RuntimeError:
            _fail_closed(
                node, 'FootprintApproach could not be restored after rear docking')
            raise
