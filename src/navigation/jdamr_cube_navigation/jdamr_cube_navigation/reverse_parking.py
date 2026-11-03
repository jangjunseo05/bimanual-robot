"""
Pure helpers for a short Nav2 reverse-parking path.

Path generation validates geometry, and the static corridor check rejects
occupied, unknown and keepout cells. Live obstacle detection remains with
Nav2 costmaps and the collision monitor.
"""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from copy import deepcopy  # noqa: I100
from itertools import chain
from pathlib import Path
from typing import Any, Sequence

from jdamr_cube_navigation.keepout_mask import _map_metadata, _point_in_polygon


ALREADY_AT_GOAL_DISTANCE_M = 0.05
MAX_XY_TOLERANCE_M = 0.05
MAX_YAW_TOLERANCE_RAD = math.radians(3.0)
MAX_REVERSE_DISTANCE_M = 2.0
MAX_WAYPOINT_SPACING_M = 0.025
MAX_REVERSE_LINEAR_VELOCITY_MPS = 0.08
# Tightest turn the curved dock reverse may ask for: the transit curve slowdown
# radius (FollowPath regulated_linear_scaling_min_radius).
REVERSE_CURVE_MIN_RADIUS_M = 0.3
# Service-session transit speed: the configured 0.12 m/s transit (operator,
# 2026-10-01: 0.06 m/s was too slow). Parking controllers keep their own
# contract speeds.
SERVICE_TRANSIT_MAX_MPS = 0.12
CACHED_CORRIDOR_SUPPORT_DIRECTIONS = 16
# Inspect a point shortly before it reaches range_min. This matches the 5 cm
# spatial keyframe interval while leaving room for the 2 cm free-ray margin.
BLIND_CLEARANCE_LOOKAHEAD_M = 0.05
RPP_PLUGIN = (
    'nav2_regulated_pure_pursuit_controller::'
    'RegulatedPurePursuitController')


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _pose(value: Sequence[float], name: str) -> tuple[float, float, float]:
    try:
        if len(value) != 3:
            raise ValueError(f'{name} must contain x, y, and yaw')
    except TypeError as error:
        raise ValueError(f'{name} must contain x, y, and yaw') from error
    return tuple(_finite_number(item, name) for item in value)


def _shortest_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def reverse_waypoints(
        start_pose: Sequence[float], target_pose: Sequence[float],
        xy_tolerance_m: float = 0.05,
        yaw_tolerance_rad: float = math.radians(3.0),
        max_distance_m: float = 2.0,
        spacing_m: float = 0.025,
) -> list[tuple[float, float, float]]:
    """
    Interpolate an obstacle-unchecked, straight reverse-only path.

    The start must be in front of the target along the target heading, with
    lateral and yaw errors inside the supplied tolerances.  Both the measured
    start pose and the exact target pose are included in the returned path.
    """
    start = _pose(start_pose, 'start_pose')
    target = _pose(target_pose, 'target_pose')
    xy_tolerance = _finite_number(xy_tolerance_m, 'xy_tolerance_m')
    yaw_tolerance = _finite_number(
        yaw_tolerance_rad, 'yaw_tolerance_rad')
    max_distance = _finite_number(max_distance_m, 'max_distance_m')
    spacing = _finite_number(spacing_m, 'spacing_m')
    if not 0.0 <= xy_tolerance <= MAX_XY_TOLERANCE_M:
        raise ValueError('xy_tolerance_m exceeds the accepted bound')
    if not 0.0 <= yaw_tolerance <= MAX_YAW_TOLERANCE_RAD:
        raise ValueError('yaw_tolerance_rad exceeds the accepted bound')
    if not 0.0 < max_distance <= MAX_REVERSE_DISTANCE_M:
        raise ValueError('max_distance_m exceeds the accepted bound')
    if not 0.0 < spacing <= MAX_WAYPOINT_SPACING_M:
        raise ValueError('spacing_m exceeds the accepted bound')

    target_heading = (math.cos(target[2]), math.sin(target[2]))
    target_left = (-target_heading[1], target_heading[0])
    target_to_start = (start[0] - target[0], start[1] - target[1])
    forward_distance = sum(
        component * heading
        for component, heading in zip(target_to_start, target_heading))
    lateral_error = abs(sum(
        component * left
        for component, left in zip(target_to_start, target_left)))
    distance = math.hypot(*target_to_start)
    yaw_delta = _shortest_angle(target[2] - start[2])

    if distance <= ALREADY_AT_GOAL_DISTANCE_M:
        raise ValueError('already at goal')
    if distance > max_distance:
        raise ValueError('reverse path exceeds max_distance_m')
    if forward_distance <= 0.0:
        raise ValueError('start_pose must be in front of target_pose')
    if lateral_error > xy_tolerance:
        raise ValueError('start_pose lateral error exceeds xy_tolerance_m')
    if abs(yaw_delta) > yaw_tolerance:
        raise ValueError('start_pose yaw error exceeds yaw_tolerance_rad')

    segment_count = math.ceil(distance / spacing)
    waypoints = []
    for index in range(segment_count + 1):
        fraction = index / segment_count
        if index == segment_count:
            waypoints.append(target)
            continue
        x = start[0] + (target[0] - start[0]) * fraction
        y = start[1] + (target[1] - start[1]) * fraction
        yaw = start[2] if index == 0 else _shortest_angle(
            start[2] + yaw_delta * fraction)
        waypoints.append((x, y, yaw))

    for current, following in zip(waypoints, waypoints[1:]):
        displacement = (
            following[0] - current[0], following[1] - current[1])
        for yaw in (current[2], following[2]):
            rearward_dot = (
                displacement[0] * math.cos(yaw)
                + displacement[1] * math.sin(yaw))
            if rearward_dot >= 0.0:
                raise ValueError('path segment is not behind robot heading')
    return waypoints


def _base_points(scan, laser_pose):
    """Yield the valid scan returns as (x, y) in the base frame."""
    lx, ly, lyaw = laser_pose
    for index, distance in enumerate(scan.ranges):
        if not (math.isfinite(distance) and scan.range_min <= distance <= scan.range_max):
            continue
        angle = lyaw + scan.angle_min + index * scan.angle_increment
        yield lx + distance * math.cos(angle), ly + distance * math.sin(angle)


def scan_points(scan, laser_pose):
    """Return the valid scan returns as a list of (x, y) in the base frame."""
    return list(_base_points(scan, laser_pose))


def points_sweep_hit(points, outline, distance_m, step_m=0.01):
    """
    Whether a straight move of distance_m (negative: reverse) would meet a point.

    points are (x, y) in the base frame. A point is met when the outline shifted
    along the heading covers it. One already inside the outline counts only on the
    half facing the move: a straight move cannot close on a point beside the
    trailing half (2026-10-02 13:48: a bounding rectangle held a return 5 mm beside
    the rear corner as "behind" and refused the back-off).
    """
    count = max(1, math.ceil(abs(distance_m) / step_m))
    shifts = [math.copysign(abs(distance_m) * index / count, distance_m)
              for index in range(1, count + 1)]
    for x_m, y_m in points:
        if _point_in_polygon(x_m, y_m, outline):
            if (x_m >= 0.0) == (distance_m > 0.0):
                return True
            continue
        if any(_point_in_polygon(x_m - shift, y_m, outline) for shift in shifts):
            return True
    return False


def straight_sweep_hit(scan, laser_pose, outline, distance_m, step_m=0.01, extra_points=()):
    """
    Whether a straight move of distance_m (negative: reverse) would meet a scan return.

    laser_pose is the scan frame in the base frame as (x, y, yaw); extra_points are
    further (x, y) in the base frame, such as remembered returns the LiDAR can no
    longer see. See points_sweep_hit.
    """
    return points_sweep_hit(
        [*_base_points(scan, laser_pose), *extra_points], outline, distance_m, step_m)


def _edge_distance(x_m, y_m, outline):
    """Distance from (x, y) to the nearest outline edge."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(outline, list(outline[1:]) + list(outline[:1])):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        fraction = 0.0 if length2 == 0.0 else max(
            0.0, min(1.0, ((x_m - ax) * dx + (y_m - ay) * dy) / length2))
        best = min(best, math.hypot(x_m - ax - fraction * dx, y_m - ay - fraction * dy))
    return best


def path_sweep_hit(points, outline, poses, step_m=0.01, step_rad=math.radians(1.0),
                   push_tolerance_m=0.005, witness=None):
    """
    Whether moving along poses would bring the outline over a point, or push into one.

    poses are (x, y, yaw) in the current base frame, starting at the base, and are
    densified to step_m and step_rad. A point outside the outline is met when the
    outline covers it at any pose. A point already inside (it touches the body) is
    met when it gets more than push_tolerance_m deeper than it started: backing off
    it or sliding along it is the escape, driving into it is not (review 2026-10-06:
    a return inside the rear half was left out, and the way back pushed it).
    """
    dense = [tuple(poses[0])]
    for start, end in zip(poses, poses[1:]):
        turn = _shortest_angle(end[2] - start[2])
        count = max(1, math.ceil(max(math.dist(start[:2], end[:2]) / step_m,
                                     abs(turn) / step_rad)))
        for index in range(1, count + 1):
            fraction = index / count
            dense.append((start[0] + (end[0] - start[0]) * fraction,
                          start[1] + (end[1] - start[1]) * fraction,
                          start[2] + turn * fraction))
    reach_m = max(math.hypot(x_m, y_m) for x_m, y_m in outline)
    for point_index, (x_m, y_m) in enumerate(points):
        touching = _point_in_polygon(x_m, y_m, outline)
        allowed_m = _edge_distance(x_m, y_m, outline) + push_tolerance_m if touching else None
        for x, y, yaw in dense[1:]:
            dx, dy = x_m - x, y_m - y
            if math.hypot(dx, dy) > reach_m:
                continue
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            local = (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)
            if not _point_in_polygon(*local, outline):
                continue
            if allowed_m is None or _edge_distance(*local, outline) > allowed_m:
                if witness is not None:
                    witness.update(point_index=point_index, point_base_xy_m=(x_m, y_m),
                                   inside_initial_outline=touching,
                                   collision_pose_base_xyyaw=(x, y, yaw),
                                   initial_boundary_distance_m=_edge_distance(x_m, y_m, outline))
                return True
    return False


def to_base_frame(base_pose, pose):
    """Express pose (x, y, yaw), given in the frame of base_pose, in the base frame."""
    dx, dy = pose[0] - base_pose[0], pose[1] - base_pose[1]
    cos_yaw, sin_yaw = math.cos(base_pose[2]), math.sin(base_pose[2])
    return (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy,
            _shortest_angle(pose[2] - base_pose[2]))


def trail_retrace_points(current, trail, distance_m, max_gap_m, max_turn_rad,
                         settle_m=0.01):
    """
    Walk the trail back from current: the way the base came, as (x, y, yaw).

    trail holds the poses the base drove forward through, oldest first, in the frame
    of current. Every step back must lead backwards from the previous pose (within
    60 deg of its rear direction), be at most max_gap_m long and turn at most
    max_turn_rad; the walk ends at the first step that is not (an odom jump, a stretch
    driven in reverse, a turn on the spot) or once distance_m is covered. Trail
    poses within settle_m of the walk are skipped. Returns (points, length_m).
    """
    points, length_m = [tuple(current)], 0.0
    for pose in reversed(list(trail)):
        previous = points[-1]
        gap_m = math.dist(previous[:2], pose[:2])
        if gap_m < settle_m:
            continue
        along_m = ((pose[0] - previous[0]) * math.cos(previous[2])
                   + (pose[1] - previous[1]) * math.sin(previous[2]))
        if (gap_m > max_gap_m or along_m > -0.5 * gap_m
                or abs(_shortest_angle(pose[2] - previous[2])) > max_turn_rad):
            break
        points.append(tuple(pose))
        length_m += gap_m
        if length_m >= distance_m:
            break
    return points, length_m


def _later_scan_clearance(point_odom, pose, scan, laser_pose):
    """Return free-space proof plus a bounded explanation of a failed proof."""
    base = to_base_frame(pose, (*point_odom, 0.0))
    laser_point = to_base_frame(laser_pose, base)
    distance = math.hypot(*laser_point[:2])
    diagnostic = {'projected_distance_m': distance}
    # Do not infer free space inside range_min, from absent returns, or along
    # an occluding surface. Two centimetres separates the old hit from new rays.
    margin_m = 0.02
    if not scan.range_min + margin_m < distance < scan.range_max - margin_m:
        diagnostic['reason'] = 'point_outside_provable_range'
        return False, diagnostic
    increment = scan.angle_increment
    if not math.isfinite(increment) or increment == 0.0:
        diagnostic['reason'] = 'scan_angle_increment_invalid'
        return False, diagnostic
    bearing = math.atan2(laser_point[1], laser_point[0])
    # Cover both [-pi, pi] and [0, 2pi] scan conventions without wrapping
    # a partial scan across its unobserved boundary.
    for turn in (-2.0 * math.pi, 0.0, 2.0 * math.pi):
        index = round((bearing + turn - scan.angle_min) / increment)
        if 1 <= index < len(scan.ranges) - 1:
            rays = scan.ranges[index - 1:index + 2]
            diagnostic.update(
                ray_index=index,
                rays_m=[float(ray) if math.isfinite(ray) else None for ray in rays])
            if all(math.isfinite(r) and distance + margin_m < r <= scan.range_max
                   for r in rays):
                diagnostic['reason'] = 'three_finite_rays_beyond_point'
                return True, diagnostic
            diagnostic['reason'] = 'three_finite_rays_not_beyond_point'
            return False, diagnostic
    diagnostic['reason'] = 'point_bearing_outside_scan'
    return False, diagnostic


def _later_scan_clears_point(point_odom, pose, scan, laser_pose):
    """Require three finite rays beyond a formerly occupied, still visible point."""
    return _later_scan_clearance(point_odom, pose, scan, laser_pose)[0]


def _scan_stamp_s(scan):
    """Return a finite LaserScan header stamp, or None for incomplete evidence."""
    try:
        stamp = scan.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _newer_same_frame_scan(source_scan, current_scan):
    """Accept an explicit clearing scan only when its ordering and frame are proven."""
    if current_scan is None:
        return False
    try:
        source_frame = source_scan.header.frame_id
        current_frame = current_scan.header.frame_id
    except AttributeError:
        return False
    source_stamp = _scan_stamp_s(source_scan)
    current_stamp = _scan_stamp_s(current_scan)
    return (bool(source_frame) and current_frame == source_frame
            and source_stamp is not None and current_stamp is not None
            and current_stamp > source_stamp)


def remembered_blind_points(keyframes, current_pose, laser_pose, blind_radius_m,
                            source_point_filter=None, source_records=None,
                            current_scan=None, current_scan_pose=None,
                            cleared_sources=None, clearance_failures=None,
                            source_diagnostics=None,
                            current_evidence_failure=None):
    """
    Return remembered returns that now lie inside the LiDAR blind range, base frame.

    keyframes are (base pose in odom, scan) pairs, current_pose the base pose in odom
    and laser_pose the scan frame in the base frame. Inside blind_radius_m of the
    laser the G4 reports nothing (range_min), so an object the base drove up to
    vanishes from the live scan; outside it the live scan decides.  An optional
    source_point_filter(pose, scan, point) selects points using the scan and pose
    that acquired them, before odometry drift changes their current projection.
    current_scan and its stamped odom pose may contribute newer same-frame
    free-space evidence. When a mutable cleared_sources set is supplied, that
    evidence is retained by source scan and return index so a point cleared while
    still visible cannot reappear after it enters the blind range. Entries are
    pruned with their bounded keyframe source.
    """
    lx, ly, _lyaw = laser_pose
    found = []
    keyframes = list(keyframes)
    active_scan_keys = {(id(frame), _scan_stamp_s(frame))
                        for _pose, frame in keyframes}
    if cleared_sources is not None:
        cleared_sources.intersection_update(
            key for key in cleared_sources if key[:2] in active_scan_keys)
    if clearance_failures is not None:
        for key in tuple(clearance_failures):
            if key[:2] not in active_scan_keys:
                clearance_failures.pop(key, None)
    for frame_index, (pose, scan) in enumerate(keyframes):
        moved_m = math.dist(pose[:2], current_pose[:2])
        relative = to_base_frame(current_pose, pose)
        cos_yaw, sin_yaw = math.cos(relative[2]), math.sin(relative[2])
        # The laser itself moved at most moved_m plus its offset swung about the axle.
        limit_m = (blind_radius_m + BLIND_CLEARANCE_LOOKAHEAD_M + moved_m
                   + 2.0 * math.hypot(lx, ly))
        for point_index, (x_m, y_m) in enumerate(_base_points(scan, laser_pose)):
            source_key = (id(scan), _scan_stamp_s(scan), point_index)
            if cleared_sources is not None and source_key in cleared_sources:
                continue
            if math.hypot(x_m - lx, y_m - ly) > limit_m:
                continue
            if (source_point_filter is not None
                    and not source_point_filter(pose, scan, (x_m, y_m))):
                continue
            bx = relative[0] + cos_yaw * x_m - sin_yaw * y_m
            by = relative[1] + sin_yaw * x_m + cos_yaw * y_m
            cosine, sine = math.cos(pose[2]), math.sin(pose[2])
            point_odom = (pose[0] + cosine * x_m - sine * y_m,
                          pose[1] + sine * x_m + cosine * y_m)
            current_distance_m = math.hypot(bx - lx, by - ly)
            inside_blind_range = current_distance_m < blind_radius_m
            clearance_window = current_distance_m < (
                blind_radius_m + BLIND_CLEARANCE_LOOKAHEAD_M)
            explicitly_cleared = False
            if clearance_window:
                source_stamp_s = _scan_stamp_s(scan)
                current_stamp_s = _scan_stamp_s(current_scan)
                diagnostic = {
                    'source_stamp_s': source_stamp_s,
                    'current_stamp_s': current_stamp_s,
                    'source_frame': getattr(getattr(scan, 'header', None), 'frame_id', None),
                    'current_frame': getattr(
                        getattr(current_scan, 'header', None), 'frame_id', None),
                    'projected_distance_m': current_distance_m,
                }
                if current_scan_pose is None:
                    diagnostic['reason'] = (
                        current_evidence_failure or 'current_scan_pose_unavailable')
                elif not _newer_same_frame_scan(scan, current_scan):
                    diagnostic['reason'] = 'scan_not_newer_in_same_frame'
                else:
                    explicitly_cleared, evidence = _later_scan_clearance(
                        point_odom, current_scan_pose, current_scan, laser_pose)
                    diagnostic.update(evidence)
                if clearance_failures is not None:
                    if explicitly_cleared:
                        clearance_failures.pop(source_key, None)
                    elif (diagnostic.get('reason') != 'point_outside_provable_range'
                          or source_key not in clearance_failures):
                        clearance_failures[source_key] = diagnostic
            historically_cleared = inside_blind_range and any(
                _later_scan_clears_point(point_odom, later_pose, later_scan, laser_pose)
                for later_pose, later_scan in keyframes[frame_index + 1:])
            if explicitly_cleared or historically_cleared:
                if clearance_failures is not None:
                    clearance_failures.pop(source_key, None)
                if explicitly_cleared and cleared_sources is not None:
                    cleared_sources.add(source_key)
                continue
            if inside_blind_range:
                found.append((bx, by))
                if source_records is not None:
                    source_records.append((tuple(pose), scan, (x_m, y_m), tuple(current_pose)))
                if source_diagnostics is not None:
                    source_diagnostics.append(
                        None if clearance_failures is None
                        else clearance_failures.get(source_key))
    return found


def outline_intrusion(scan, laser_pose, outline, min_points, extra_points=()):
    """
    Return which half of the outline holds at least min_points returns.

    'front' (ahead of the axle), 'rear', 'both' or None. The Collision Monitor
    stops on min_points returns inside a polygon, and a return inside the padded
    footprint holds every polygon that contains it (2026-10-02 11:09). extra_points
    are further (x, y) in the base frame (remembered returns in the blind range).
    """
    counts = {'front': 0, 'rear': 0}
    for x_m, y_m in [*_base_points(scan, laser_pose), *extra_points]:
        if _point_in_polygon(x_m, y_m, outline):
            counts['front' if x_m >= 0.0 else 'rear'] += 1
    held = [half for half in ('front', 'rear') if counts[half] >= min_points]
    return 'both' if len(held) == 2 else (held[0] if held else None)


def rear_swing_clearance(scan, laser_pose, radius_m, extra_points=()):
    """
    Return the forward travel that takes every return behind the axle out of the turn circle.

    The base turns about its axle near the front, so its rear sweeps radius_m. A return
    at (x < 0, y) inside that circle leaves it after sqrt(radius_m^2 - y^2) + x of
    forward travel. Returns 0.0 when nothing behind the axle is inside the circle.
    """
    needed_m = 0.0
    for x_m, y_m in [*_base_points(scan, laser_pose), *extra_points]:
        if x_m < 0.0 and math.hypot(x_m, y_m) < radius_m:
            needed_m = max(needed_m, math.sqrt(radius_m ** 2 - y_m ** 2) + x_m)
    return needed_m


def reverse_curve_waypoints(
        start_pose: Sequence[float], target_pose: Sequence[float],
        min_radius_m: float = REVERSE_CURVE_MIN_RADIUS_M,
        max_distance_m: float = 2.0,
        spacing_m: float = 0.025,
) -> list[tuple[float, float, float]]:
    """
    Reverse from the start pose onto the target pose along one smooth curve.

    A cubic Hermite curve leaves the start along its rear direction and arrives
    along the target's rear direction, so a lateral or heading offset is taken
    out while backing up instead of by turn-move-turn in front of the dock
    (2026-10-01: a 10 cm staging offset cost a full extra turn). The robot faces
    away from its motion at every point. Rejected when any part of the curve is
    tighter than min_radius_m, so only small offsets are absorbed this way.
    """
    start = _pose(start_pose, 'start_pose')
    target = _pose(target_pose, 'target_pose')
    min_radius = _finite_number(min_radius_m, 'min_radius_m')
    max_distance = _finite_number(max_distance_m, 'max_distance_m')
    spacing = _finite_number(spacing_m, 'spacing_m')
    if not 0.0 < max_distance <= MAX_REVERSE_DISTANCE_M:
        raise ValueError('max_distance_m exceeds the accepted bound')
    if not 0.0 < spacing <= MAX_WAYPOINT_SPACING_M:
        raise ValueError('spacing_m exceeds the accepted bound')
    chord = (target[0] - start[0], target[1] - start[1])
    distance = math.hypot(*chord)
    if distance <= ALREADY_AT_GOAL_DISTANCE_M:
        raise ValueError('already at goal')
    if distance > max_distance:
        raise ValueError('reverse path exceeds max_distance_m')
    if -sum(c * h for c, h in zip(chord, (math.cos(target[2]), math.sin(target[2])))) <= 0.0:
        raise ValueError('start_pose must be in front of target_pose')
    # Motion directions (rearward) scaled by the chord length.
    d0 = (-math.cos(start[2]) * distance, -math.sin(start[2]) * distance)
    d1 = (-math.cos(target[2]) * distance, -math.sin(target[2]) * distance)

    def point(t):
        h00, h10 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t
        h01, h11 = -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
        return tuple(h00 * start[i] + h10 * d0[i] + h01 * target[i] + h11 * d1[i]
                     for i in (0, 1))

    def velocity(t):
        a, b = 6 * t ** 2 - 6 * t, 3 * t ** 2 - 4 * t + 1
        c, d = -6 * t ** 2 + 6 * t, 3 * t ** 2 - 2 * t
        return tuple(a * start[i] + b * d0[i] + c * target[i] + d * d1[i] for i in (0, 1))

    def acceleration(t):
        a, b, c, d = 12 * t - 6, 6 * t - 4, -12 * t + 6, 6 * t - 2
        return tuple(a * start[i] + b * d0[i] + c * target[i] + d * d1[i] for i in (0, 1))

    count = max(2, math.ceil(distance / spacing))
    waypoints = []
    for index in range(count + 1):
        t = index / count
        vx, vy = velocity(t)
        ax, ay = acceleration(t)
        speed = math.hypot(vx, vy)
        if speed <= 1e-9 or abs(vx * ay - vy * ax) > speed ** 3 / min_radius:
            raise ValueError('reverse curve is tighter than min_radius_m')
        if index == 0:
            waypoints.append(start)
        elif index == count:
            waypoints.append(target)
        else:
            x, y = point(t)
            waypoints.append((x, y, math.atan2(-vy, -vx)))
    return waypoints


def reverse_controller_overrides(controller_dict: dict) -> dict:
    """Append an isolated, collision-enabled reverse Parking controller."""
    if not isinstance(controller_dict, dict):
        raise ValueError('controller parameters must be a mapping')
    controllers = controller_dict.get('controller_plugins')
    if not isinstance(controllers, list):
        raise ValueError('controller_plugins must be a list')
    if controllers.count('Parking') != 1:
        raise ValueError('controller_plugins must contain Parking once')
    if ('ParkingReverse' in controllers
            or 'ParkingReverse' in controller_dict):
        raise ValueError('ParkingReverse controller already exists')

    parking = controller_dict.get('Parking')
    if not isinstance(parking, dict):
        raise ValueError('Parking configuration must be a mapping')
    if parking.get('plugin') != RPP_PLUGIN:
        raise ValueError('reverse parking requires Regulated Pure Pursuit')
    desired_velocity = _finite_number(
        parking.get('desired_linear_vel'), 'Parking.desired_linear_vel')
    if desired_velocity <= 0.0:
        raise ValueError('Parking.desired_linear_vel must be positive')

    configured = deepcopy(controller_dict)
    configured['controller_plugins'].append('ParkingReverse')
    reverse = deepcopy(configured['Parking'])
    reverse.update({
        'use_rotate_to_heading': False,
        'allow_reversing': True,
        'desired_linear_vel': min(
            desired_velocity, MAX_REVERSE_LINEAR_VELOCITY_MPS),
    })
    configured['ParkingReverse'] = reverse
    graceful = configured.get('GracefulParking')
    if isinstance(graceful, dict):
        configured['controller_plugins'].append('GracefulReverse')
        configured['GracefulReverse'] = {**graceful, 'allow_backward': True}
    return configured


def _clearly_free(metadata: dict, row: int, column: int) -> bool:
    pixel = metadata['pixels'][row * metadata['width'] + column]
    if pixel == 205:
        return False
    occupancy = (
        pixel / 255.0 if metadata['negate'] else (255 - pixel) / 255.0)
    return occupancy < metadata['free_thresh']


def _convex_hull(
        points: Sequence[Sequence[float]],
) -> list[tuple[float, float]]:
    ordered = sorted({
        (float(point[0]), float(point[1])) for point in points
    })
    if len(ordered) < 3:
        raise ValueError('footprint must enclose an area')

    def cross(origin, first, second):
        return ((first[0] - origin[0]) * (second[1] - origin[1])
                - (first[1] - origin[1]) * (second[0] - origin[0]))

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError('footprint must enclose an area')
    return hull


def _convex_footprint_pieces(
        points: Sequence[Sequence[float]],
) -> list[list[tuple[float, float]]]:
    """Validate an ordered simple footprint and split concavities into ears."""
    polygon = [(float(point[0]), float(point[1])) for point in points]
    if len(polygon) > 1 and polygon[0] == polygon[-1]:
        polygon.pop()
    if len(polygon) < 3 or len(set(polygon)) != len(polygon):
        raise ValueError('footprint must be a simple polygon')

    def cross(first, second, third):
        return ((second[0] - first[0]) * (third[1] - first[1])
                - (second[1] - first[1]) * (third[0] - first[0]))

    def on_segment(first, second, point):
        return (abs(cross(first, second, point)) <= 1e-12
                and min(first[0], second[0]) - 1e-12 <= point[0]
                <= max(first[0], second[0]) + 1e-12
                and min(first[1], second[1]) - 1e-12 <= point[1]
                <= max(first[1], second[1]) + 1e-12)

    def intersects(a, b, c, d):
        turns = (cross(a, b, c), cross(a, b, d),
                 cross(c, d, a), cross(c, d, b))
        if ((turns[0] > 1e-12 and turns[1] < -1e-12
             or turns[0] < -1e-12 and turns[1] > 1e-12)
                and (turns[2] > 1e-12 and turns[3] < -1e-12
                     or turns[2] < -1e-12 and turns[3] > 1e-12)):
            return True
        return any((abs(turns[index]) <= 1e-12
                    and on_segment(*pair)) for index, pair in enumerate((
                        (a, b, c), (a, b, d), (c, d, a), (c, d, b))))

    edge_count = len(polygon)
    for first in range(edge_count):
        for second in range(first + 1, edge_count):
            if (second == (first + 1) % edge_count
                    or first == (second + 1) % edge_count):
                continue
            if intersects(
                    polygon[first], polygon[(first + 1) % edge_count],
                    polygon[second], polygon[(second + 1) % edge_count]):
                raise ValueError('footprint must be a simple polygon')

    changed = True
    while changed and len(polygon) >= 3:
        changed = False
        for index in range(len(polygon)):
            if abs(cross(polygon[index - 1], polygon[index],
                         polygon[(index + 1) % len(polygon)])) <= 1e-12:
                polygon.pop(index)
                changed = True
                break
    if len(polygon) < 3:
        raise ValueError('footprint must enclose an area')
    area = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(polygon, polygon[1:] + polygon[:1])) / 2.0
    if abs(area) <= 1e-12:
        raise ValueError('footprint must enclose an area')
    if area < 0.0:
        polygon.reverse()
    if all(cross(polygon[index - 1], polygon[index],
                 polygon[(index + 1) % len(polygon)]) > 1e-12
           for index in range(len(polygon))):
        return [polygon]

    def inside_triangle(point, triangle):
        return all(cross(triangle[index], triangle[(index + 1) % 3], point) >= -1e-12
                   for index in range(3))

    remaining = list(polygon)
    pieces = []
    while len(remaining) > 3:
        for index in range(len(remaining)):
            previous = (index - 1) % len(remaining)
            following = (index + 1) % len(remaining)
            triangle = [remaining[previous], remaining[index], remaining[following]]
            if cross(*triangle) <= 1e-12:
                continue
            if any(inside_triangle(point, triangle)
                   for candidate, point in enumerate(remaining)
                   if candidate not in (previous, index, following)):
                continue
            pieces.append(triangle)
            remaining.pop(index)
            break
        else:
            raise ValueError('footprint cannot be decomposed safely')
    pieces.append(remaining)
    return pieces


def _circumscribed_support_polygon(
        points: Sequence[Sequence[float]],
        direction_count: int = CACHED_CORRIDOR_SUPPORT_DIRECTIONS,
        additional_directions: Sequence[Sequence[float]] = (),
) -> list[tuple[float, float]]:
    """Bound a point cloud by a bounded-size outer support polygon."""
    if direction_count < 8 or direction_count % 4:
        raise ValueError('direction_count must be a multiple of four and at least eight')
    directions = [
        (math.cos(2.0 * math.pi * index / direction_count),
         math.sin(2.0 * math.pi * index / direction_count))
        for index in range(direction_count)]
    for direction in additional_directions:
        normal_x = _finite_number(direction[0], 'support direction')
        normal_y = _finite_number(direction[1], 'support direction')
        length = math.hypot(normal_x, normal_y)
        if length <= 1e-12:
            raise ValueError('support direction must have positive length')
        unit = (normal_x / length, normal_y / length)
        if not any(
                abs(unit[0] - existing[0]) <= 1e-12
                and abs(unit[1] - existing[1]) <= 1e-12
                for existing in directions):
            directions.append(unit)
    supports = []
    for normal_x, normal_y in directions:
        support = max(
            x_value * normal_x + y_value * normal_y
            for x_value, y_value in points)
        supports.append(support + max(1e-12, abs(support) * 1e-12))

    minimum_x = min(point[0] for point in points)
    maximum_x = max(point[0] for point in points)
    minimum_y = min(point[1] for point in points)
    maximum_y = max(point[1] for point in points)
    epsilon = max(
        1e-12, max(abs(minimum_x), abs(maximum_x),
                   abs(minimum_y), abs(maximum_y)) * 1e-12)
    polygon = [
        (minimum_x - epsilon, minimum_y - epsilon),
        (maximum_x + epsilon, minimum_y - epsilon),
        (maximum_x + epsilon, maximum_y + epsilon),
        (minimum_x - epsilon, maximum_y + epsilon),
    ]
    for (normal_x, normal_y), support in zip(directions, supports):
        clipped = []
        previous = polygon[-1]
        previous_projection = previous[0] * normal_x + previous[1] * normal_y
        previous_inside = previous_projection <= support
        for current in polygon:
            current_projection = current[0] * normal_x + current[1] * normal_y
            current_inside = current_projection <= support
            if current_inside != previous_inside:
                denominator = current_projection - previous_projection
                fraction = (support - previous_projection) / denominator
                clipped.append((
                    previous[0] + fraction * (current[0] - previous[0]),
                    previous[1] + fraction * (current[1] - previous[1])))
            if current_inside:
                clipped.append(current)
            previous = current
            previous_projection = current_projection
            previous_inside = current_inside
        polygon = clipped
    if len(polygon) < 3:
        raise ValueError('swept footprint must enclose an area')
    return polygon


def _world_footprint(
        pose: Sequence[float], footprint: Sequence[Sequence[float]],
) -> list[tuple[float, float]]:
    x_pose, y_pose, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (x_pose + x_local * cosine - y_local * sine,
         y_pose + x_local * sine + y_local * cosine)
        for x_local, y_local in footprint
    ]


def _polygon_intersects_cell(
        polygon: Sequence[Sequence[float]], padding: float,
        cell_left: float, cell_bottom: float, resolution: float,
) -> bool:
    projections = _polygon_projections(polygon)
    return _prepared_polygon_intersects_cell(
        projections, padding, cell_left, cell_bottom, resolution)


def _polygon_projections(
        polygon: Sequence[Sequence[float]],
) -> list[tuple[float, float, float, float]]:
    """Return SAT axes and polygon extents reused for every candidate cell."""
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length = math.hypot(delta_x, delta_y)
        if length > 0.0:
            axes.append((-delta_y / length, delta_x / length))
    projections = []
    for axis_x, axis_y in axes:
        values = [
            x_value * axis_x + y_value * axis_y
            for x_value, y_value in polygon]
        projections.append((axis_x, axis_y, min(values), max(values)))
    return projections


def _prepared_polygon_intersects_cell(
        projections: Sequence[tuple[float, float, float, float]],
        padding: float, cell_left: float, cell_bottom: float,
        resolution: float,
) -> bool:
    """SAT test using polygon projections prepared once for a grid scan."""
    cell_right = cell_left + resolution
    cell_top = cell_bottom + resolution
    for axis_x, axis_y, polygon_minimum, polygon_maximum in projections:
        projections_at_corners = (
            cell_left * axis_x + cell_bottom * axis_y,
            cell_right * axis_x + cell_bottom * axis_y,
            cell_right * axis_x + cell_top * axis_y,
            cell_left * axis_x + cell_top * axis_y,
        )
        if (polygon_maximum + padding < min(projections_at_corners)
                or max(projections_at_corners) < polygon_minimum - padding):
            return False
    return True


def _corridor_blockage(
        source: dict, keepout: dict,
        pieces: Sequence[tuple[Sequence[Sequence[float]], float]],
        ignored_keepout_cells=frozenset(),
        *, skip_clearly_free_cells: bool = False,
        ignored_unknown_cells=frozenset(),
        nonfree_columns_by_row=None,
) -> dict | None:
    resolution = source['resolution']
    origin_x, origin_y, _origin_yaw = source['origin']
    width = source['width']
    height = source['height']
    for polygon, padding in pieces:
        projections = _polygon_projections(polygon)
        minimum_column = math.floor(
            (min(point[0] for point in polygon) - padding - origin_x)
            / resolution) - 1
        maximum_column = math.floor(
            (max(point[0] for point in polygon) + padding - origin_x)
            / resolution) + 1
        minimum_map_row = math.floor(
            (min(point[1] for point in polygon) - padding - origin_y)
            / resolution) - 1
        maximum_map_row = math.floor(
            (max(point[1] for point in polygon) + padding - origin_y)
            / resolution) + 1
        for map_row in range(minimum_map_row, maximum_map_row + 1):
            cell_bottom = origin_y + map_row * resolution
            columns = range(minimum_column, maximum_column + 1)
            if (skip_clearly_free_cells and nonfree_columns_by_row is not None
                    and 0 <= map_row < height):
                row = height - 1 - map_row
                nonfree_columns = nonfree_columns_by_row[row]
                first = bisect_left(nonfree_columns, max(0, minimum_column))
                last = bisect_right(
                    nonfree_columns, min(width - 1, maximum_column))
                columns = chain(
                    range(minimum_column, min(0, maximum_column + 1)),
                    nonfree_columns[first:last],
                    range(max(width, minimum_column), maximum_column + 1))
            for column in columns:
                cell_left = origin_x + column * resolution
                inside_grid = (
                    0 <= map_row < height and 0 <= column < width)
                if skip_clearly_free_cells and inside_grid:
                    row = height - 1 - map_row
                    if (_clearly_free(source, row, column)
                            and _clearly_free(keepout, row, column)):
                        continue
                if not _prepared_polygon_intersects_cell(
                        projections, padding, cell_left, cell_bottom, resolution):
                    continue
                if not inside_grid:
                    return {
                        'reason': 'outside_grid',
                        'column': column,
                        'map_row': map_row,
                    }
                row = height - 1 - map_row
                for grid_name, metadata in (
                        ('map', source), ('keepout', keepout)):
                    if (grid_name == 'keepout'
                            and (column, map_row) in ignored_keepout_cells):
                        continue
                    if (grid_name == 'map'
                            and (column, map_row) in ignored_unknown_cells
                            and metadata['pixels'][row * width + column] == 205):
                        continue
                    if not _clearly_free(metadata, row, column):
                        index = row * width + column
                        return {
                            'reason': f'{grid_name}_not_free',
                            'grid': grid_name,
                            'column': column,
                            'map_row': map_row,
                            'image_row': row,
                            'pixel': metadata['pixels'][index],
                            'cell_bounds': (
                                cell_left, cell_bottom,
                                cell_left + resolution,
                                cell_bottom + resolution),
                        }
    return None


def _exact_corridor_pieces(poses, footprint):
    footprints = [_world_footprint(pose, footprint) for pose in poses]
    radius_m = max(math.hypot(x, y) for x, y in footprint)
    pieces = [(polygon, 0.) for polygon in footprints]
    for index, (start, end) in enumerate(zip(poses, poses[1:])):
        turn_rad = abs(_shortest_angle(end[2] - start[2]))
        pieces.append((_convex_hull(footprints[index] + footprints[index + 1]),
                       radius_m * (1. - math.cos(turn_rad / 2.))))
    return pieces


def _cell_overlap_polygon(polygon, bounds, padding):
    """Clip a swept piece to a cell expanded by its conservative rotation pad."""
    left, bottom, right, top = bounds
    result = list(polygon)
    for axis, sign, limit in ((0, -1., -left + padding), (0, 1., right + padding),
                              (1, -1., -bottom + padding), (1, 1., top + padding)):
        if not result:
            break
        clipped = []
        previous = result[-1]
        previous_value = sign * previous[axis]
        for current in result:
            value = sign * current[axis]
            if (value <= limit) != (previous_value <= limit):
                fraction = (limit - previous_value) / (value - previous_value)
                clipped.append(tuple(previous[i] + fraction * (current[i] - previous[i])
                                     for i in (0, 1)))
            if value <= limit:
                clipped.append(current)
            previous, previous_value = current, value
        result = clipped
    return result


def _observed_surface_corridor_clear(
        source, keepout, poses, footprint, surface,
        *, nonfree_columns_by_row=None):
    """Resolve target raster cells outside a measured plane, never occupied cells."""
    face = tuple(_finite_number(v, 'surface point') for v in surface['point_xy_m'])
    normal = tuple(_finite_number(v, 'surface normal') for v in surface['normal_xy'])
    half_span_m = _finite_number(surface['half_span_m'], 'surface half span')
    if (len(face) != 2 or len(normal) != 2 or half_span_m <= 0.
            or not math.isclose(math.hypot(*normal), 1., abs_tol=1e-6)):
        raise ValueError('invalid observed surface')
    tangent = -normal[1], normal[0]
    length_m = 0.
    for start, end in zip(poses, poses[1:]):
        step = end[0] - start[0], end[1] - start[1]
        length_m += math.hypot(*step)
        if sum(step[i] * normal[i] for i in (0, 1)) > 1e-9:
            return False
    if length_m > .6 + 1e-9:
        return False
    pieces = _exact_corridor_pieces(poses, footprint)
    if any(min(sum((point[i] - face[i]) * normal[i] for i in (0, 1))
               for point in polygon) <= padding for polygon, padding in pieces):
        return False
    ignored = set()
    ignored_unknown = set()
    while True:
        blocked = _corridor_blockage(source, keepout, pieces, ignored,
                                     skip_clearly_free_cells=True,
                                     ignored_unknown_cells=ignored_unknown,
                                     nonfree_columns_by_row=nonfree_columns_by_row)
        if blocked is None:
            return True
        target_mask = (blocked.get('grid') == 'keepout'
                       and keepout['negate'] == 0 and blocked.get('pixel') == 0)
        target_unknown = (surface.get('allow_precision_unknown') is True
                          and blocked.get('grid') == 'map'
                          and source['negate'] == 0 and blocked.get('pixel') == 205)
        if not (target_mask or target_unknown):
            return False
        outward_m, lateral_m = [], []
        for polygon, padding in pieces:
            overlap = _cell_overlap_polygon(polygon, blocked['cell_bounds'], padding)
            outward_m.extend(
                sum((p[i] - face[i]) * normal[i] for i in (0, 1)) + padding for p in overlap)
            lateral_m.extend(
                abs(sum((p[i] - face[i]) * tangent[i] for i in (0, 1))) + padding for p in overlap)
        # This is a grid-resolution allowance, not a new physical parking gap.
        half_cell_tangent_m = .5 * source['resolution'] * (abs(tangent[0]) + abs(tangent[1]))
        if (not outward_m or max(outward_m) <= 0.
                or max(outward_m) > 2. * source['resolution']
                or max(lateral_m) > half_span_m + half_cell_tangent_m):
            return False
        (ignored_unknown if target_unknown else ignored).add(
            (blocked['column'], blocked['map_row']))


class CachedStaticCorridorValidator:
    """Reuse one verified map pair and footprint for conservative path checks.

    Each call reduces all sampled footprint poses to one outer support polygon.
    Fixed map axes bound its cost, while start/end footprint-edge normals avoid
    excess at the actual body angle.  Maximum between-sample rotation sagitta
    padding keeps every piece checked by :func:`static_corridor_clear` inside.
    It may therefore reject extra free space, but cannot make a blocked exact
    corridor free.
    """

    def __init__(
            self, map_yaml: Path, keepout_yaml: Path,
            footprint: Sequence[Sequence[float]],
    ) -> None:
        source = _map_metadata(Path(map_yaml).resolve())
        keepout = _map_metadata(Path(keepout_yaml).resolve())
        if (not math.isclose(
                source['resolution'], keepout['resolution'], abs_tol=1e-12)
                or source['origin'] != keepout['origin']
                or source['width'] != keepout['width']
                or source['height'] != keepout['height']):
            raise ValueError('map and keepout grids must have identical geometry')
        resolution = source['resolution']
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError('map resolution must be positive and finite')
        for metadata in (source, keepout):
            if (metadata['negate'] not in (0, 1)
                    or not math.isfinite(metadata['free_thresh'])
                    or not 0.0 <= metadata['free_thresh'] <= 1.0):
                raise ValueError('map occupancy metadata is invalid')

        try:
            footprint_values = tuple(footprint)
        except TypeError as error:
            raise ValueError(
                'footprint must contain at least three points') from error
        if len(footprint_values) < 3:
            raise ValueError('footprint must contain at least three points')
        footprint_points = []
        for point in footprint_values:
            try:
                if len(point) != 2:
                    raise ValueError('footprint points must contain x and y')
            except TypeError as error:
                raise ValueError(
                    'footprint points must contain x and y') from error
            footprint_points.append((
                _finite_number(point[0], 'footprint'),
                _finite_number(point[1], 'footprint')))

        self._source = source
        self._keepout = keepout
        self._nonfree_columns_by_row = tuple(
            tuple(column for column in range(source['width'])
                  if (not _clearly_free(source, row, column)
                      or not _clearly_free(keepout, row, column)))
            for row in range(source['height']))
        self._footprint = tuple(_convex_hull(footprint_points))
        edge_normals = []
        for start, end in zip(
                self._footprint, self._footprint[1:] + self._footprint[:1]):
            delta_x = end[0] - start[0]
            delta_y = end[1] - start[1]
            length = math.hypot(delta_x, delta_y)
            edge_normals.append((-delta_y / length, delta_x / length))
        self._footprint_edge_normals = tuple(edge_normals)
        self._footprint_radius = max(
            math.hypot(x_value, y_value)
            for x_value, y_value in self._footprint)

    def __call__(self, poses: Sequence[Sequence[float]], *, target_surface=None) -> bool:
        """Return whether the conservative hull of ``poses`` is clearly free."""
        try:
            checked_poses = tuple(_pose(pose, 'pose') for pose in poses)
        except TypeError as error:
            raise ValueError('poses must be a non-empty sequence') from error
        if not checked_poses:
            raise ValueError('poses must be a non-empty sequence')

        swept_points = []
        for pose in checked_poses:
            swept_points.extend(_world_footprint(pose, self._footprint))
        support_directions = []
        for pose in (checked_poses[0], checked_poses[-1]):
            cosine = math.cos(pose[2])
            sine = math.sin(pose[2])
            support_directions.extend(
                (cosine * normal_x - sine * normal_y,
                 sine * normal_x + cosine * normal_y)
                for normal_x, normal_y in self._footprint_edge_normals)
        swept_hull = _circumscribed_support_polygon(
            swept_points, additional_directions=support_directions)
        maximum_sagitta = 0.0
        for current, following in zip(checked_poses, checked_poses[1:]):
            yaw_change = abs(_shortest_angle(following[2] - current[2]))
            maximum_sagitta = max(
                maximum_sagitta,
                self._footprint_radius * (1.0 - math.cos(yaw_change / 2.0)))
        self.last_blockage = _corridor_blockage(
            self._source, self._keepout,
            [(swept_hull, maximum_sagitta)],
            skip_clearly_free_cells=True,
            nonfree_columns_by_row=self._nonfree_columns_by_row)
        clear = self.last_blockage is None
        if clear or target_surface is None:
            return clear
        clear = _observed_surface_corridor_clear(
            self._source, self._keepout, checked_poses, self._footprint,
            target_surface,
            nonfree_columns_by_row=self._nonfree_columns_by_row)
        if clear:
            self.last_blockage = None
        return clear


def static_corridor_clear(
        map_yaml: Path, keepout_yaml: Path,
        poses: Sequence[Sequence[float]],
        footprint: Sequence[Sequence[float]],
        *, allow_rear_keepout_escape: bool = False,
        live_checked_dock_occupancy: bool = False,
) -> bool:
    """
    Check a sampled footprint corridor against static map pixels.

    This complements Nav2 path validation by treating unknown pixels as
    blocked.  It checks only the supplied static map and keepout mask, not
    live obstacles or the robot's runtime costmaps.
    """
    source = _map_metadata(Path(map_yaml).resolve())
    keepout = _map_metadata(Path(keepout_yaml).resolve())
    if (not math.isclose(
            source['resolution'], keepout['resolution'], abs_tol=1e-12)
            or source['origin'] != keepout['origin']
            or source['width'] != keepout['width']
            or source['height'] != keepout['height']):
        raise ValueError('map and keepout grids must have identical geometry')
    if live_checked_dock_occupancy:
        # Only fully occupied map pixels may yield to a live-checked dock path.
        # Unknown pixels, keepout, map boundaries and the on-disk map stay intact.
        occupied, free = (0, 255) if source['negate'] == 0 else (255, 0)
        source = dict(source, pixels=bytes(
            free if pixel == occupied else pixel for pixel in source['pixels']))
    resolution = source['resolution']
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('map resolution must be positive and finite')
    for metadata in (source, keepout):
        if (metadata['negate'] not in (0, 1)
                or not math.isfinite(metadata['free_thresh'])
                or not 0.0 <= metadata['free_thresh'] <= 1.0):
            raise ValueError('map occupancy metadata is invalid')

    try:
        checked_poses = tuple(_pose(pose, 'pose') for pose in poses)
    except TypeError as error:
        raise ValueError('poses must be a non-empty sequence') from error
    if not checked_poses:
        raise ValueError('poses must be a non-empty sequence')
    try:
        if len(footprint) < 3:
            raise ValueError('footprint must contain at least three points')
    except TypeError as error:
        raise ValueError(
            'footprint must contain at least three points') from error
    footprint_points = []
    for point in footprint:
        try:
            if len(point) != 2:
                raise ValueError('footprint points must contain x and y')
        except TypeError as error:
            raise ValueError(
                'footprint points must contain x and y') from error
        footprint_points.append((
            _finite_number(point[0], 'footprint'),
            _finite_number(point[1], 'footprint')))

    footprint_pieces = _convex_footprint_pieces(footprint_points)
    footprint_radius = max(
        math.hypot(x_value, y_value)
        for x_value, y_value in footprint_points)
    world_footprints = [[
        _world_footprint(pose, piece) for piece in footprint_pieces]
        for pose in checked_poses]
    ignored_keepout_cells = set()
    if allow_rear_keepout_escape:
        start = checked_poses[0]
        cosine, sine = math.cos(start[2]), math.sin(start[2])
        previous_forward = 0.0
        for pose in checked_poses:
            dx, dy = pose[0] - start[0], pose[1] - start[1]
            forward = dx * cosine + dy * sine
            lateral = -dx * sine + dy * cosine
            if (abs(_shortest_angle(pose[2] - start[2])) > 1e-9
                    or abs(lateral) > 1e-9 or forward < previous_forward - 1e-9):
                return False
            previous_forward = forward
        while True:
            blockage = _corridor_blockage(
                source, keepout,
                [(polygon, 0.0) for polygon in world_footprints[0]],
                ignored_keepout_cells)
            if blockage is None:
                break
            if (blockage.get('grid') != 'keepout' or keepout['negate'] != 0
                    or blockage.get('pixel') != 0):
                return False
            left, bottom, right, top = blockage['cell_bounds']
            corners = ((left, bottom), (right, bottom), (right, top), (left, top))
            if (max((x - start[0]) * cosine + (y - start[1]) * sine
                    for x, y in corners) >= 0.0
                    or any(_polygon_intersects_cell(
                        polygon, 0.0, left, bottom, resolution)
                        for polygon in world_footprints[-1])):
                return False
            ignored_keepout_cells.add((blockage['column'], blockage['map_row']))
    pieces = [(polygon, 0.0) for footprints in world_footprints
              for polygon in footprints]
    for index, (current, following) in enumerate(zip(
            checked_poses, checked_poses[1:])):
        yaw_change = abs(_shortest_angle(following[2] - current[2]))
        sagitta = footprint_radius * (1.0 - math.cos(yaw_change / 2.0))
        for current_piece, following_piece in zip(
                world_footprints[index], world_footprints[index + 1]):
            swept_hull = _convex_hull(current_piece + following_piece)
            pieces.append((swept_hull, sagitta))
    return _corridor_blockage(source, keepout, pieces, ignored_keepout_cells) is None
