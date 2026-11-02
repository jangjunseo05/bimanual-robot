"""Pure parameter validation for the measured new-base navigation contract."""

import json
import math

from .docking_stop_profile import USER_APPROVED_CLEARANCE_M


# Approved selection policy for the five directional StopZone entries. These
# are software policy limits, not newly measured chassis values.
VELOCITY_POLICY_RANGES = {
    'rotation': (-0.005, 0.005, 0.005, 1.0),
    'rotation_clockwise': (-0.005, 0.005, -1.0, -0.005),
    'translation_forward_straight': (0.005, 0.2, -0.005, 0.005),
    'translation_backward_straight': (-0.2, -0.005, -0.005, 0.005),
    'translation_forward': (0.005, 0.2, -1.0, 1.0),
    'translation_backward': (-0.2, -0.005, -1.0, 1.0),
    'stopped': (-1.0, 1.0, -1.0, 1.0),
}


def _inside(polygon, point, strict):
    """Whether point lies in polygon; on an edge counts only when not strict."""
    x_m, y_m = point
    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        cross = ((end[0] - start[0]) * (y_m - start[1])
                 - (end[1] - start[1]) * (x_m - start[0]))
        if (abs(cross) <= 1e-9
                and min(start[0], end[0]) - 1e-9 <= x_m <= max(start[0], end[0]) + 1e-9
                and min(start[1], end[1]) - 1e-9 <= y_m <= max(start[1], end[1]) + 1e-9):
            return not strict
        if (start[1] > y_m) != (end[1] > y_m):
            crossing_x = start[0] + (y_m - start[1]) * (end[0] - start[0]) / (end[1] - start[1])
            if crossing_x > x_m:
                inside = not inside
    return inside


def _crossing(a, b, c, d):
    """Whether segments ab and cd intersect at more than a shared endpoint."""
    def side(p, q, r):
        value = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return 0 if abs(value) <= 1e-12 else (1 if value > 0 else -1)
    if len({tuple(a), tuple(b)} & {tuple(c), tuple(d)}):
        return False
    sides = (side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b))
    if not any(sides):
        # Collinear: only overlapping spans cross (the stepped sides share lines).
        return all(max(min(a[k], b[k]), min(c[k], d[k]))
                   <= min(max(a[k], b[k]), max(c[k], d[k])) for k in (0, 1))
    return sides[0] * sides[1] <= 0 and sides[2] * sides[3] <= 0


def _proper_crossing(a, b, c, d):
    """Whether segments ab and cd cross at a point interior to both."""
    def side(p, q, r):
        value = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return 0 if abs(value) <= 1e-12 else (1 if value > 0 else -1)
    return (side(a, b, c) * side(a, b, d) < 0
            and side(c, d, a) * side(c, d, b) < 0)


def polygon_contains(outer, inner):
    """
    Whether the polygon outer covers the polygon inner, boundaries may touch.

    Exact for simple polygons, unlike sampling the inner perimeter (review
    2026-10-06: a 1 mm groove between 5 mm samples passed): every inner vertex
    and edge midpoint lies in or on outer, no outer vertex or edge midpoint lies
    strictly inside inner, and no two edges cross in their interiors.
    """
    outer_edges = list(zip(outer, list(outer[1:]) + list(outer[:1])))
    inner_edges = list(zip(inner, list(inner[1:]) + list(inner[:1])))

    def middle(edge):
        return ((edge[0][0] + edge[1][0]) / 2.0, (edge[0][1] + edge[1][1]) / 2.0)

    return (all(_inside(outer, point, False) for point in inner)
            and all(_inside(outer, middle(edge), False) for edge in inner_edges)
            and not any(_inside(inner, point, True) for point in outer)
            and not any(_inside(inner, middle(edge), True) for edge in outer_edges)
            and not any(_proper_crossing(*a, *b)
                        for a in outer_edges for b in inner_edges))


def _clip_half_plane(polygon, keep):
    """Clip polygon to the half plane x >= 0 (keep 'front') or x <= 0 ('rear')."""
    sign = 1.0 if keep == 'front' else -1.0
    clipped = []
    for start, end in zip(polygon, list(polygon[1:]) + list(polygon[:1])):
        start_in, end_in = sign * start[0] >= 0.0, sign * end[0] >= 0.0
        if start_in:
            clipped.append(tuple(start))
        if start_in != end_in:
            fraction = start[0] / (start[0] - end[0])
            clipped.append((0.0, start[1] + (end[1] - start[1]) * fraction))
    unique = []
    for point in clipped:
        if not unique or point != unique[-1]:
            unique.append(point)
    return unique


# Operator request 2026-09-30: transit +50 % and less slowdown.
NEW_BASE_MAX_FORWARD_MPS = 0.12
# Reverse at most at the parking reverse speed (parking_contract desired_linear_mps).
NEW_BASE_MAX_REVERSE_MPS = 0.08
# In-place turns stop on the footprint swept this far in the turn direction; it
# covers the stop from the smoother's turn-rate cap (operator 2026-10-02).
ROTATION_SWEEP_DEG = 6.0
ROTATION_SWEEP_MAX_RADPS = 0.3
NEW_BASE_SLOWDOWN_RATIO = 0.8


def validate_new_base_params(params, geometry, precision_parking=False):
    """Reject navigation parameters that violate the measured base contract."""
    front = geometry['front_to_wheel_axis']['value']
    rear = front - geometry['frame_length']['value']
    half_width = geometry['wheel_outer_width']['value'] / 2.0

    def polygon_points(value):
        polygon = json.loads(value) if isinstance(value, str) else value
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise RuntimeError('new-base polygon needs at least three points')
        if any(not isinstance(point, (list, tuple)) or len(point) != 2
               or any(type(coordinate) not in (int, float)
                      or not math.isfinite(coordinate)
                      for coordinate in point)
               for point in polygon):
            raise RuntimeError(
                'new-base polygon points must be finite numeric pairs')
        return polygon

    def bounds(points):
        """
        Return (front, rear, half width) of a rectilinear polygon mirrored about X.

        The footprint and StopZones follow the measured body: the 0.45 m frame is
        narrower than the 0.54 m wheels, so they are stepped polygons (2026-10-02);
        a rectangle is the four-corner case.
        """
        polygon = polygon_points(points)
        corners = {tuple(point) for point in polygon}
        if (len(polygon) < 4 or len(corners) != len(polygon)
                or any((x, -y) not in corners for x, y in corners)):
            raise RuntimeError('new-base polygon must be mirrored about the base axis')
        edges = [(polygon[index], polygon[(index + 1) % len(polygon)])
                 for index in range(len(polygon))]
        if any(start[0] != end[0] and start[1] != end[1] for start, end in edges):
            raise RuntimeError('new-base polygon edges must be axis-aligned')
        if any(_crossing(*edges[i], *edges[j])
               for i in range(len(edges)) for j in range(i + 2, len(edges))
               if not (i == 0 and j == len(edges) - 1)):
            raise RuntimeError('new-base polygon corners must follow the perimeter')
        return (max(x for x, _y in corners), min(x for x, _y in corners),
                max(abs(y) for _x, y in corners))

    def contains(outer, inner_points, strict, label):
        outer_polygon = polygon_points(outer)
        if not all(_inside(outer_polygon, point, strict) for point in inner_points):
            raise RuntimeError(label)

    costmaps = [params[key][key]['ros__parameters']
                for key in ('local_costmap', 'global_costmap')]
    footprints = [bounds(costmap['footprint']) for costmap in costmaps]
    if footprints[0] != footprints[1]:
        raise RuntimeError('new-base costmap footprints differ')
    footprint = footprints[0]
    if not (footprint[0] > front and footprint[1] < rear
            and footprint[2] > half_width):
        raise RuntimeError('new-base footprint is smaller than measured base')
    # Every measured corner (frame, and wheels around the axle) lies strictly
    # inside the padded footprint.
    try:
        frame_half_width = geometry['frame_width']['value'] / 2.0
        wheel_radius = geometry['wheel_radius_initial']['value']
        wheelbase_m = geometry['wheelbase']['value']
    except (KeyError, TypeError) as error:
        raise RuntimeError('new-base geometry lacks frame width, wheel radius or wheelbase') from error
    if (geometry['wheelbase'].get('unit') != 'm' or isinstance(wheelbase_m, bool)
            or not isinstance(wheelbase_m, (int, float))
            or not math.isfinite(wheelbase_m) or not 0.0 < wheelbase_m < front - rear):
        raise RuntimeError('new-base wheelbase must fit the four-wheel frame in meters')
    body = [(x, y) for x in (front, rear) for y in (-frame_half_width, frame_half_width)]
    body += [(axle_m + offset_m, y_m) for axle_m in (0.0, -wheelbase_m)
             for offset_m in (-wheel_radius, wheel_radius)
             for y_m in (-half_width, half_width)]
    for costmap in costmaps:
        contains(costmap['footprint'], body, True,
                 'new-base footprint does not cover the measured body')
    if any(costmap['robot_base_frame'] != 'base_footprint'
           for costmap in costmaps):
        raise RuntimeError('new-base costmaps need base_footprint')
    for costmap in costmaps:
        layer = costmap['obstacle_layer']
        scan = layer['scan']
        sources = layer.get('observation_sources')
        if (not layer['enabled'] or sources != 'scan'
                or scan['topic'] != '/scan' or not scan['marking']):
            raise RuntimeError('new-base costmap scan layer is not active')
    depth_enabled = [
        'depth_obstacle_layer' in costmap for costmap in costmaps]
    if any(depth_enabled) and not all(depth_enabled):
        raise RuntimeError('new-base depth layer must cover both costmaps')
    if all(depth_enabled):
        for costmap in costmaps:
            plugins = costmap.get('plugins', [])
            if ('depth_obstacle_layer' not in plugins
                    or plugins.index('depth_obstacle_layer')
                    != plugins.index('inflation_layer') - 1):
                raise RuntimeError(
                    'new-base depth layer must run before inflation')
            depth_layer = costmap['depth_obstacle_layer']
            expected_layer = {
                'plugin': 'nav2_costmap_2d::VoxelLayer',
                'enabled': True,
                'combination_method': 1,
                'origin_z': 0.0,
                'z_resolution': 0.1,
                'z_voxels': 16,
                'mark_threshold': 0,
                'unknown_threshold': 15,
                'publish_voxel_map': False,
                'min_obstacle_height': -0.05,
                'max_obstacle_height': 1.5,
                'observation_sources': 'depth_marks depth_rays',
            }
            if any(depth_layer.get(key) != value
                   for key, value in expected_layer.items()):
                raise RuntimeError(
                    'new-base depth voxel layer violates approved policy')
            marks = depth_layer.get('depth_marks', {})
            rays = depth_layer.get('depth_rays', {})
            common = (
                marks.get('sensor_frame') == 'camera_color_optical_frame'
                and rays.get('sensor_frame') == 'camera_color_optical_frame'
                and marks.get('data_type') == 'PointCloud2'
                and rays.get('data_type') == 'PointCloud2')
            marking = (
                marks.get('topic') == '/depth_navigation/obstacles'
                and marks.get('marking') is True
                and marks.get('clearing') is False
                and marks.get('min_obstacle_height') == 0.05
                and marks.get('max_obstacle_height') == 1.5
                and marks.get('obstacle_min_range') == 0.4
                and marks.get('obstacle_max_range') == 2.5)
            clearing = (
                rays.get('topic') == '/depth_navigation/rays'
                and rays.get('marking') is False
                and rays.get('clearing') is True
                and rays.get('min_obstacle_height') == -0.05
                and rays.get('max_obstacle_height') == 1.5
                and rays.get('raytrace_min_range') == 0.4
                and rays.get('raytrace_max_range') == 2.5)
            if not (common and marking and clearing):
                raise RuntimeError(
                    'new-base depth costmap sources violate approved policy')

    monitor = params['collision_monitor']['ros__parameters']
    if monitor.get('base_frame_id') != 'base_footprint':
        raise RuntimeError('new-base collision monitor needs base_footprint')
    if not {'StopZone', 'SlowdownZone', 'FootprintApproach'} <= set(
            monitor['polygons']):
        raise RuntimeError('new-base collision polygons are incomplete')
    approved_sources = (['scan'], ['scan', 'depth_obstacles'])
    observation_sources = monitor.get('observation_sources')
    if all(depth_enabled) != (
            observation_sources == ['scan', 'depth_obstacles']):
        raise RuntimeError(
            'new-base depth costmaps and collision source must be enabled together')
    if observation_sources not in approved_sources:
        if (not isinstance(observation_sources, list)
                or 'scan' not in observation_sources):
            raise RuntimeError(
                'new-base scan collision source is not active')
        raise RuntimeError(
            'new-base collision sources must be scan with optional depth')
    for name in ('StopZone', 'SlowdownZone', 'FootprintApproach'):
        effective_sources = monitor[name].get(
            'sources_names', observation_sources)
        if effective_sources != observation_sources:
            raise RuntimeError(
                f'new-base {name} must observe the scan source and all '
                'approved sources')
    stop_zone = monitor['StopZone']
    if (stop_zone['type'] != 'velocity_polygon'
            or stop_zone['action_type'] != 'stop'
            or not stop_zone['enabled']
            or type(stop_zone.get('min_points')) is not int
            or stop_zone['min_points'] != 3
            or stop_zone.get('holonomic') is not False):
        raise RuntimeError('new-base StopZone is not active')
    expected_velocity_polygons = [
        'rotation', 'rotation_clockwise', 'translation_forward_straight',
        'translation_backward_straight', 'translation_forward',
        'translation_backward', 'stopped']
    if stop_zone.get('velocity_polygons') != expected_velocity_polygons:
        raise RuntimeError(
            'new-base directional StopZone order is incomplete or unsafe')
    for name in expected_velocity_polygons:
        polygon = stop_zone[name]
        for minimum, maximum in (
                ('linear_min', 'linear_max'), ('theta_min', 'theta_max')):
            lower = polygon.get(minimum)
            upper = polygon.get(maximum)
            if (type(lower) not in (int, float)
                    or type(upper) not in (int, float)
                    or not math.isfinite(lower) or not math.isfinite(upper)
                    or lower > upper):
                raise RuntimeError(
                    f'new-base {name} {minimum}/{maximum} range is invalid')
        observed_range = (
            polygon['linear_min'], polygon['linear_max'],
            polygon['theta_min'], polygon['theta_max'])
        if ((name == 'rotation' and polygon['theta_min'] <= 0.0)
                or (name == 'rotation_clockwise'
                    and polygon['theta_max'] >= 0.0)):
            raise RuntimeError(
                'new-base rotation StopZone must exclude zero velocity')
        if observed_range != VELOCITY_POLICY_RANGES[name]:
            raise RuntimeError(
                f'new-base {name} velocity range violates approved policy')
    slow_zone = monitor['SlowdownZone']
    # The precision session switches the slowdown ring off: it always holds the
    # target face during the 5 cm approach (docking_stop_profile).
    if (slow_zone['type'] != 'polygon'
            or slow_zone['action_type'] != 'slowdown'
            or (slow_zone['enabled'] is not True and not precision_parking)
            or type(slow_zone.get('min_points')) is not int
            or slow_zone['min_points'] != 3):
        raise RuntimeError('new-base SlowdownZone is not active')
    ratio = slow_zone.get('slowdown_ratio')
    if type(ratio) not in (int, float) or ratio != NEW_BASE_SLOWDOWN_RATIO:
        raise RuntimeError('new-base slowdown ratio violates approved policy')
    approach = monitor['FootprintApproach']
    if (approach.get('type') != 'polygon'
            or approach['action_type'] != 'approach' or not approach['enabled']
            or type(approach.get('min_points')) is not int
            or approach['min_points'] != 3
            or approach.get('footprint_topic') !=
            '/local_costmap/published_footprint'):
        raise RuntimeError('new-base approach monitor is not active')
    for parameter, approved_value in (
            ('time_before_collision', 0.5),
            ('simulation_time_step', 0.1)):
        value = approach.get(parameter)
        if (type(value) not in (int, float) or not math.isfinite(value)
                or value != approved_value):
            raise RuntimeError(
                f'new-base approach {parameter} violates approved policy')
    if ('scan' not in monitor['observation_sources']
            or not monitor['scan']['enabled']
            or monitor['scan']['type'] != 'scan'
            or monitor['scan']['topic'] != '/scan'):
        raise RuntimeError('new-base scan collision source is not active')
    if observation_sources == ['scan', 'depth_obstacles']:
        depth = monitor.get('depth_obstacles', {})
        if (depth.get('type') != 'pointcloud'
                or depth.get('topic') != '/depth_navigation/obstacles'
                or depth.get('enabled') is not True
                or depth.get('source_timeout') != 1.0
                or depth.get('min_height') != 0.05
                or depth.get('max_height') != 1.5):
            raise RuntimeError(
                'new-base depth collision source violates approved policy')

    rotation_points = polygon_points(stop_zone['rotation']['points'])
    clockwise_points = polygon_points(
        stop_zone['rotation_clockwise']['points'])
    counterclockwise = stop_zone['rotation']
    clockwise = stop_zone['rotation_clockwise']
    if (counterclockwise['theta_min'] <= 0.0
            or clockwise['theta_max'] >= 0.0):
        raise RuntimeError(
            'new-base rotation StopZone must exclude zero velocity')
    if sorted(map(tuple, clockwise_points)) != sorted(
            (x, -y) for x, y in rotation_points):
        raise RuntimeError('new-base rotation StopZones must mirror each other')
    footprint_polygon = [tuple(point) for point in polygon_points(costmaps[0]['footprint'])]
    for name, sign in (('rotation', 1.0), ('rotation_clockwise', -1.0)):
        points = polygon_points(stop_zone[name]['points'])
        edges = list(zip(points, points[1:] + points[:1]))
        if any(_crossing(*edges[i], *edges[j])
               for i in range(len(edges)) for j in range(i + 2, len(edges))
               if not (i == 0 and j == len(edges) - 1)):
            raise RuntimeError(f'new-base {name} StopZone edges must not cross')
        # The footprint turned every 0.5 deg; between steps the corners move on
        # arcs whose sag (3.4 um at 0.36 m) the 3 mm buffer covers.
        for step in range(int(ROTATION_SWEEP_DEG * 2) + 1):
            angle = math.radians(sign * step / 2.0)
            turned = [(math.cos(angle) * x - math.sin(angle) * y,
                       math.sin(angle) * x + math.cos(angle) * y)
                      for x, y in footprint_polygon]
            if not polygon_contains(points, turned):
                raise RuntimeError(f'new-base {name} StopZone misses the swept footprint')
    forward_stop = bounds(stop_zone['translation_forward']['points'])
    backward_stop = bounds(stop_zone['translation_backward']['points'])
    stopped_stop = bounds(stop_zone['stopped']['points'])
    slow = bounds(slow_zone['points'])
    if not (stopped_stop[0] > footprint[0]
            and stopped_stop[1] <= footprint[1] + 1e-6
            and stopped_stop[2] >= footprint[2] - 1e-6):
        raise RuntimeError('new-base StopZone does not contain footprint')
    for name in ('stopped', 'translation_forward', 'translation_backward'):
        if not polygon_contains(polygon_points(stop_zone[name]['points']), footprint_polygon):
            raise RuntimeError(f'new-base {name} StopZone does not contain footprint')
    # Zero-turn polygons hold the footprint half on the side of travel, cut at the
    # axle, with the same leading edge as the general polygon (2026-10-02 11:09).
    for name, general, keep in (
            ('translation_forward_straight', forward_stop, 'front'),
            ('translation_backward_straight', backward_stop, 'rear')):
        straight = bounds(stop_zone[name]['points'])
        forward = name == 'translation_forward_straight'
        if not ((straight[0] == general[0] and straight[1] == 0.0) if forward
                else (straight[0] == 0.0 and straight[1] == general[1])):
            raise RuntimeError(f'new-base {name} StopZone must end at the axle')
        if not polygon_contains(polygon_points(stop_zone[name]['points']),
                                _clip_half_plane(footprint_polygon, keep)):
            raise RuntimeError(
                f'new-base {name} StopZone does not contain its footprint half')
    front_reference = front
    # All sessions retain the approved physical-front margin; sides and rear
    # keep the padded footprint.
    if stopped_stop[0] - front_reference < USER_APPROVED_CLEARANCE_M - 1e-6:
        raise RuntimeError('new-base StopZone margin is below approved clearance')
    if not (forward_stop[0] - front_reference >= USER_APPROVED_CLEARANCE_M - 1e-6
            and forward_stop[1] <= footprint[1]
            and abs(forward_stop[2] - footprint[2]) <= 1e-6):
        raise RuntimeError('new-base forward StopZone shape is invalid')
    if precision_parking and (
            not math.isclose(forward_stop[0], front + USER_APPROVED_CLEARANCE_M, abs_tol=1e-6)
            or not math.isclose(stopped_stop[0], front + USER_APPROVED_CLEARANCE_M, abs_tol=1e-6)
            or forward_stop[0] - footprint[0] < 0.02 - 1e-6):
        raise RuntimeError('precision parking requires physical front clearance and padding')
    # The approved leading band spans the whole front edge of the footprint, in every
    # polygon that can stop forward motion (review 2026-10-06: a band narrowed to
    # |y| <= 0.05 passed the front-X check).
    front_half_width = max(abs(y) for x, y in footprint_polygon if x == footprint[0])
    band_margin_m = USER_APPROVED_CLEARANCE_M
    band = [(footprint[0], -front_half_width), (front_reference + band_margin_m, -front_half_width),
            (front_reference + band_margin_m, front_half_width), (footprint[0], front_half_width)]
    for name in ('stopped', 'translation_forward', 'translation_forward_straight'):
        if not polygon_contains(polygon_points(stop_zone[name]['points']), band):
            raise RuntimeError(f'new-base {name} StopZone misses the approved front band')
    if not (footprint[1] - backward_stop[1] >= -1e-6
            and backward_stop[0] >= footprint[0]
            and abs(backward_stop[2] - footprint[2]) <= 1e-6):
        raise RuntimeError('new-base backward StopZone shape is invalid')
    rotation_front = max(point[0] for point in rotation_points)
    rotation_rear = min(point[0] for point in rotation_points)
    rotation_half_width = max(abs(point[1]) for point in rotation_points)
    if not (slow[0] > max(stopped_stop[0], rotation_front)
            and slow[1] < min(stopped_stop[1], rotation_rear)
            and slow[2] > max(stopped_stop[2], rotation_half_width)):
        raise RuntimeError('new-base SlowdownZone does not contain StopZone')
    if (type(monitor.get('source_timeout')) not in (int, float)
            or monitor['source_timeout'] != 1.0):
        raise RuntimeError('new-base scan source timeout must be 1.0s')
    scan_timeout = monitor['scan'].get('source_timeout', 1.0)
    if (type(scan_timeout) not in (int, float) or scan_timeout != 1.0):
        raise RuntimeError(
            'new-base scan timeout override must be exactly 1.0s')
    if params['amcl']['ros__parameters']['set_initial_pose']:
        raise RuntimeError('new-base AMCL cannot force the old map origin')
    amcl_tf_tolerance_s = params['amcl']['ros__parameters'][
        'transform_tolerance']
    if (type(amcl_tf_tolerance_s) not in (int, float)
            or amcl_tf_tolerance_s != 1.0):
        raise RuntimeError('new-base AMCL transform tolerance must be 1.0s')
    smoother = params['velocity_smoother']['ros__parameters']
    if smoother['max_velocity'][0] > NEW_BASE_MAX_FORWARD_MPS:
        raise RuntimeError('new-base forward speed exceeds uncalibrated limit')
    if not (abs(smoother['max_velocity'][2]) <= ROTATION_SWEEP_MAX_RADPS
            and abs(smoother['min_velocity'][2]) <= ROTATION_SWEEP_MAX_RADPS):
        raise RuntimeError('new-base turn rate exceeds the rotation StopZone sweep')
    controller = params['controller_server']['ros__parameters']
    reverse = controller.get('ParkingReverse')
    minimum_velocity = smoother['min_velocity'][0]
    if (type(minimum_velocity) not in (int, float)
            or not math.isfinite(minimum_velocity)):
        raise RuntimeError('new-base reverse velocity must be finite')
    if reverse is None:
        if minimum_velocity < 0.0:
            raise RuntimeError('new-base autonomous profile cannot reverse')
    elif (not isinstance(reverse, dict)
          or 'ParkingReverse' not in controller.get('controller_plugins', [])
          or reverse.get('allow_reversing') is not True
          or reverse.get('use_rotate_to_heading') is not False
          or type(reverse.get('desired_linear_vel')) not in (int, float)
          or not math.isfinite(reverse['desired_linear_vel'])
          or not -0.08 <= minimum_velocity < 0.0
          or not math.isclose(minimum_velocity, -reverse['desired_linear_vel'],
                              abs_tol=1e-9)):
        raise RuntimeError('new-base reverse parking controller/smoother mismatch')
    progress = controller['progress_checker']
    if (progress['plugin'] != 'nav2_controller::PoseProgressChecker'
            or progress.get('required_movement_angle') != 0.10):
        raise RuntimeError('new-base progress checker must count rotation')
    controller_plugins = controller.get('controller_plugins')
    if (not isinstance(controller_plugins, list) or not controller_plugins
            or 'FollowPath' not in controller_plugins):
        raise RuntimeError(
            'new-base controller plugins must include FollowPath')
    # The default transit and staging trees follow with controller_id MPPI
    # (since 2026-10-05); without it every leg fails with INVALID_CONTROLLER.
    if 'MPPI' not in controller_plugins:
        raise RuntimeError('new-base controller plugins must include MPPI')
    supported_controller = (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController')
    graceful_controller = 'nav2_graceful_controller::GracefulController'
    mppi_controller = 'nav2_mppi_controller::MPPIController'
    for plugin_name in controller_plugins:
        if plugin_name not in controller:
            raise RuntimeError(
                f'new-base registered controller {plugin_name} is missing')
        plugin = controller[plugin_name]
        if plugin.get('plugin') == graceful_controller:
            # Graceful has no collision-detection switch in 1.3.12; the
            # collision monitor still gates its output like every controller.
            top = plugin.get('v_linear_max')
            if (type(top) not in (int, float) or not math.isfinite(top)
                    or not 0.0 < top <= NEW_BASE_MAX_FORWARD_MPS):
                raise RuntimeError(
                    'new-base controller speed exceeds uncalibrated limit')
            continue
        if plugin.get('plugin') == mppi_controller:
            # MPPI must score rollouts with the measured footprint and keep the
            # uncalibrated speed limits; it may reverse at most as fast as reverse
            # parking (operator 2026-10-06).
            top, bottom = plugin.get('vx_max'), plugin.get('vx_min')
            cost = plugin.get('CostCritic') or {}
            if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (top, bottom))
                    or not 0.0 < top <= NEW_BASE_MAX_FORWARD_MPS
                    or not -NEW_BASE_MAX_REVERSE_MPS <= bottom <= 0.0
                    or plugin.get('motion_model') != 'DiffDrive'
                    or type(plugin.get('wz_max')) not in (int, float)
                    or not 0.0 < plugin['wz_max'] <= ROTATION_SWEEP_MAX_RADPS
                    or 'CostCritic' not in (plugin.get('critics') or [])
                    or cost.get('enabled') is not True
                    or cost.get('consider_footprint') is not True):
                raise RuntimeError('new-base MPPI controller violates the approved profile')
            continue
        if plugin.get('plugin') != supported_controller:
            raise RuntimeError(
                f'new-base controller {plugin_name} plugin is unsupported')
        desired_velocity = plugin.get('desired_linear_vel')
        if (type(desired_velocity) not in (int, float)
                or not math.isfinite(desired_velocity)
                or not 0.0 < desired_velocity <= NEW_BASE_MAX_FORWARD_MPS):
            raise RuntimeError(
                'new-base controller speed exceeds uncalibrated limit')
    if params['bt_navigator']['ros__parameters'][
            'robot_base_frame'] != 'base_footprint':
        raise RuntimeError('new-base BT needs base_footprint')
    return []
