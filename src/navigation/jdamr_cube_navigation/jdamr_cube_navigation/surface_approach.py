"""Plane-relative approach commands and their nominal footprint corridor."""

import math


FACE_HEADING_TOLERANCE_RAD = math.radians(0.3)
FACE_HEADING_DEADBAND_RAD = math.radians(0.1)
FACE_ALIGNMENT_GAP_M = 0.05
FACE_ALIGNMENT_ENTRY_MARGIN_M = 0.0005
FACE_HEADING_FILTER_S = 0.35
FACE_APPROACH_TIME_S = 0.5
TABLE_APPROACH_SPEED_MPS = 0.1
TABLE_ALIGNMENT_MAX_RADPS = 0.08
LATERAL_MIN_LOOKAHEAD_M = 0.02
LATERAL_ARRIVAL_STRIP_FRACTION = 0.8


def filtered_face_heading(previous, observed, elapsed_s):
    """Filter a circular heading with the same state law in control and preview."""
    if not math.isfinite(observed) or not math.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError('invalid table heading filter input')
    if previous is None:
        return observed
    if not math.isfinite(previous):
        raise ValueError('invalid previous table heading')
    delta = math.atan2(math.sin(observed - previous), math.cos(observed - previous))
    return previous + (1.0 - math.exp(-elapsed_s / FACE_HEADING_FILTER_S)) * delta


def face_approach_command(remaining_m, error_rad, target_gap_m=0.01, *,
                          approach_time_s=FACE_APPROACH_TIME_S, allow_near_alignment=False,
                          continuous_table=False, surface_gap_m=None,
                          lateral_tracking=False, face_error_rad=None):
    """Track a command heading while guarding clearance with face alignment.

    ``error_rad`` is the steering command error.  Normal approaches use it for
    both steering and clearance, preserving the original behaviour.  A
    lateral-tracking approach supplies ``face_error_rad`` separately so an
    intentional curved command is not mistaken for physical misalignment with
    the surface.
    """
    if not all(math.isfinite(v) for v in (
            remaining_m, error_rad, target_gap_m, approach_time_s)) or approach_time_s <= 0.0:
        raise ValueError('nonfinite table approach command')
    if not isinstance(lateral_tracking, bool):
        raise ValueError('invalid lateral tracking mode')
    if lateral_tracking:
        if (isinstance(face_error_rad, bool)
                or not isinstance(face_error_rad, (int, float))
                or not math.isfinite(face_error_rad)):
            raise ValueError('lateral tracking requires finite face error')
        geometry_error_rad = float(face_error_rad)
    else:
        geometry_error_rad = error_rad
    if surface_gap_m is None:
        surface_gap_m = remaining_m * math.cos(geometry_error_rad) + target_gap_m
    if isinstance(surface_gap_m, bool) or not math.isfinite(surface_gap_m):
        raise ValueError('invalid surface gap')
    early_alignment = continuous_table and surface_gap_m > (
        max(FACE_ALIGNMENT_GAP_M, target_gap_m) + FACE_ALIGNMENT_ENTRY_MARGIN_M)
    limit = TABLE_ALIGNMENT_MAX_RADPS if early_alignment else 0.02
    angular = max(-limit, min(limit, (3.0 if early_alignment else 1.0) * error_rad))
    if abs(error_rad) <= FACE_HEADING_DEADBAND_RAD:
        angular = 0.0
    cruise = TABLE_APPROACH_SPEED_MPS if continuous_table else 0.05
    # Scale the taper time with cruise speed so doubling cruise does not double
    # the demanded deceleration. The default/rear taper is unchanged.
    taper_time_s = approach_time_s * cruise / .05
    terminal_speed = max(0.01, remaining_m / taper_time_s)
    linear = min(cruise, terminal_speed)
    if remaining_m <= 0.0:
        linear = 0.0
    if abs(geometry_error_rad) > FACE_HEADING_TOLERANCE_RAD:
        clearance = remaining_m - max(0.0, FACE_ALIGNMENT_GAP_M - target_gap_m)
        if clearance <= 0.0:
            if allow_near_alignment:
                return 0.0, angular
            raise ValueError('table alignment requires more clearance')
        if continuous_table:
            # Lateral tracking intentionally carries a temporary face-angle
            # error while following a curve.  Fresh surface gap and corridor
            # validation guard that segment through the precision band.  The
            # hard no-clearance branch above remains active.  Normal approaches
            # keep the earlier predictive stop-and-align behaviour.
            if not lateral_tracking:
                # [unit-mismatch: paradigmatic] angle [rad] / rate [rad/s]
                # gives time [s].
                alignment_s = max(
                    approach_time_s,
                    2.0 * abs(geometry_error_rad) / abs(angular))
                if clearance < linear * alignment_s:
                    linear = 0.0
        else:
            # Remaining alignment distance [m] / time [s] bounds linear speed.
            alignment_s = max(
                approach_time_s,
                2.0 * abs(geometry_error_rad) / abs(angular))
            linear = min(linear, clearance / alignment_s)
    return linear, angular


def lateral_face_approach_command(remaining_m, face_error_rad, lateral_error_m,
                                  surface_gap_m, lateral_tolerance_m, *,
                                  target_gap_m=0.01, allow_near_alignment=False):
    """Track a lateral goal while reaching the near-face band parallel to it.

    The cubic path's curvature is limited by the available yaw rate, so a
    lateral correction cannot demand more steering than the base can produce.
    This is the same command used by the corridor preview and live controller.
    """
    values = (remaining_m, face_error_rad, lateral_error_m, surface_gap_m,
              lateral_tolerance_m, target_gap_m)
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in values)
            or lateral_tolerance_m <= 0.0 or target_gap_m <= 0.0):
        raise ValueError('invalid lateral table approach')
    if surface_gap_m <= 0.0:
        raise ValueError('table approach crosses surface')
    if abs(face_error_rad) > math.radians(30.0) + 1e-9:
        raise ValueError('table approach heading out of range')
    band = target_gap_m + FACE_ALIGNMENT_ENTRY_MARGIN_M
    if surface_gap_m <= band:
        if abs(lateral_error_m) > lateral_tolerance_m:
            raise ValueError('table lateral alignment requires more clearance')
        if abs(face_error_rad) > FACE_HEADING_TOLERANCE_RAD:
            if not allow_near_alignment:
                raise ValueError('table alignment requires more clearance')
            return 0.0, max(-0.02, min(0.02, face_error_rad))
        return face_approach_command(
            remaining_m, face_error_rad, target_gap_m, allow_near_alignment=True,
            continuous_table=True, surface_gap_m=surface_gap_m,
            lateral_tracking=True, face_error_rad=face_error_rad)

    # A Hermite curve with the current heading and a face-parallel end tangent
    # has start curvature 6*offset/L^2 - 4*tan(start_heading)/L.
    lookahead = max(LATERAL_MIN_LOOKAHEAD_M, remaining_m)
    # Keep tracking while the current heading could carry the base outside the
    # arrival strip. Inside it, straighten without an unnecessary tight S-curve.
    # Heading correction below stays active even when the offset is suppressed.
    heading_drift_m = max(0., remaining_m) * abs(math.tan(face_error_rad))
    arrival_strip_m = max(0., lateral_tolerance_m - heading_drift_m)
    offset = math.copysign(max(0., abs(lateral_error_m)
                              - LATERAL_ARRIVAL_STRIP_FRACTION * arrival_strip_m),
                           lateral_error_m)
    curvature = 6.0 * offset / lookahead ** 2 + 4.0 * math.tan(face_error_rad) / lookahead
    if not math.isfinite(curvature):
        raise ValueError('invalid lateral table curvature')
    linear = min(TABLE_APPROACH_SPEED_MPS, max(0.01, remaining_m))
    if abs(curvature) > 1e-12:
        linear = min(linear, TABLE_ALIGNMENT_MAX_RADPS / abs(curvature))
    if remaining_m <= 0.0:
        linear = 0.0
    return linear, linear * curvature


def face_approach_waypoints(start, face, normal, front_extent_m, half_width_m,
                            target_gap_m=0.01, *, command_heading_rad=None,
                            approach_time_s=FACE_APPROACH_TIME_S, allow_near_alignment=False,
                            continuous_table=False, lateral_tracking=False,
                            lateral_goal_odom=None, lateral_tolerance_m=None,
                            max_lateral_heading_rad=None, lateral_entry_gap_m=None):
    """Preview the same command law up to the existing final precision band.

    This ideal odometry rollout is for static corridor validation, not evidence
    of real traction or physical parking accuracy.
    """
    values = (*start, *face, *normal, front_extent_m, half_width_m,
              target_gap_m, approach_time_s)
    if (len(start) != 3 or len(face) != 2 or len(normal) != 2
            or not all(math.isfinite(v) for v in values)
            or not math.isclose(math.hypot(*normal), 1.0, abs_tol=1e-6)
            or front_extent_m <= 0.0 or half_width_m <= 0.0 or target_gap_m <= 0.0
            or approach_time_s <= 0.0):
        raise ValueError('invalid table approach preview geometry')
    if lateral_tracking:
        try:
            lateral_goal = tuple(lateral_goal_odom)
        except TypeError as error:
            raise ValueError('lateral preview requires goal pose') from error
        if (not continuous_table or len(lateral_goal) != 3
                or not all(isinstance(value, (int, float))
                           and not isinstance(value, bool)
                           and math.isfinite(value) for value in lateral_goal)
                or isinstance(lateral_tolerance_m, bool)
                or not isinstance(lateral_tolerance_m, (int, float))
                or not math.isfinite(lateral_tolerance_m)
                or lateral_tolerance_m <= 0.0
                or isinstance(max_lateral_heading_rad, bool)
                or not isinstance(max_lateral_heading_rad, (int, float))
                or not math.isfinite(max_lateral_heading_rad)
                or not 0.0 < max_lateral_heading_rad <= math.pi):
            raise ValueError('invalid lateral preview control contract')
    else:
        lateral_goal = None
    if lateral_entry_gap_m is not None and (
            not lateral_tracking or isinstance(lateral_entry_gap_m, bool)
            or not isinstance(lateral_entry_gap_m, (int, float))
            or not math.isfinite(lateral_entry_gap_m)
            or lateral_entry_gap_m <= target_gap_m):
        raise ValueError('invalid lateral alignment entry gap')
    acquisition_goal = start[:2]
    lateral_entered = lateral_entry_gap_m is None
    alignment_gap_m = (target_gap_m if lateral_tracking
                       else max(FACE_ALIGNMENT_GAP_M, target_gap_m))
    heading = math.atan2(-normal[1], -normal[0])
    command_heading = (None if lateral_tracking else
                       filtered_face_heading(command_heading_rad, heading, 0.0))
    pose = tuple(start)
    poses = [pose]
    for _step in range(1600):
        error = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        heading_limit = max_lateral_heading_rad if lateral_tracking else math.radians(30.0)
        if abs(error) > heading_limit + 1e-9:
            raise ValueError('table approach preview heading out of range')
        projected_distance = sum(
            (pose[i] - face[i]) * normal[i] for i in (0, 1))
        center_gap = projected_distance - front_extent_m * math.cos(error)
        gap = center_gap - half_width_m * abs(math.sin(error))
        if gap <= 0.0:
            raise ValueError('table approach preview crosses surface')
        remaining_gap = center_gap if lateral_tracking else gap
        remaining = (remaining_gap - target_gap_m) / math.cos(error)
        if lateral_tracking:
            lateral_entered = lateral_entered or gap <= lateral_entry_gap_m
            tracking_goal = lateral_goal if lateral_entered else acquisition_goal
            left = (-math.sin(heading), math.cos(heading))
            lateral_error = sum((tracking_goal[i] - pose[i]) * left[i] for i in (0, 1))
            command_error = None
        else:
            command_error = math.atan2(math.sin(command_heading - pose[2]),
                                       math.cos(command_heading - pose[2]))
        if gap <= alignment_gap_m + FACE_ALIGNMENT_ENTRY_MARGIN_M:
            if lateral_tracking:
                linear, angular = lateral_face_approach_command(
                    remaining, error, lateral_error, gap, lateral_tolerance_m,
                    target_gap_m=target_gap_m, allow_near_alignment=allow_near_alignment)
                if abs(error) <= FACE_HEADING_TOLERANCE_RAD:
                    return poses
                if abs(error) >= math.atan2(half_width_m, front_extent_m):
                    raise ValueError('table alignment requires more clearance')
                pose = (*pose[:2], pose[2] + angular * .05)
                poses.append(pose)
                continue
            if (target_gap_m <= FACE_ALIGNMENT_GAP_M and
                    (abs(error) if lateral_tracking else
                     max(abs(error), abs(command_error)))
                    > FACE_HEADING_TOLERANCE_RAD):
                # Turning toward the normal retracts the leading front corner
                # only below the rectangular support maximum atan(width/extent).
                if not allow_near_alignment or abs(error) >= math.atan2(
                        half_width_m, front_extent_m):
                    raise ValueError('table alignment requires more clearance')
                angular = max(-.02, min(.02, command_error))
                if angular * error <= 0.:
                    angular = 0.
                pose = (*pose[:2], pose[2] + angular * .05)
                poses.append(pose)
                command_heading = filtered_face_heading(command_heading, heading, .05)
                continue
            return poses
        if math.dist(start[:2], pose[:2]) + remaining > 0.6 + 1e-9:
            raise ValueError('table approach preview exceeds bounded travel')
        if lateral_tracking:
            linear, angular = lateral_face_approach_command(
                remaining, error, lateral_error, gap, lateral_tolerance_m,
                target_gap_m=target_gap_m, allow_near_alignment=allow_near_alignment)
        else:
            linear, angular = face_approach_command(
                remaining, command_error, target_gap_m, approach_time_s=approach_time_s,
                continuous_table=continuous_table, surface_gap_m=gap)
        closing_speed_bound = linear + abs(angular) * (front_extent_m + half_width_m)
        step_s = min(0.05, (gap - alignment_gap_m)
                     / max(closing_speed_bound, 1e-9))
        pose = (pose[0] + linear * math.cos(pose[2]) * step_s,
                pose[1] + linear * math.sin(pose[2]) * step_s,
                pose[2] + angular * step_s)
        poses.append(pose)
        if not lateral_tracking:
            command_heading = filtered_face_heading(
                command_heading, heading, step_s)
    raise ValueError('table approach preview did not converge')
