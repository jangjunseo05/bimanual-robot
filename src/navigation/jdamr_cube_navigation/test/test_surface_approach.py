"""Ideal approach rollout and static corridor regressions without hardware."""

import math

from jdamr_cube_navigation.reverse_parking import static_corridor_clear
from jdamr_cube_navigation.surface_approach import (
    FACE_ALIGNMENT_ENTRY_MARGIN_M,
    face_approach_command, face_approach_waypoints, filtered_face_heading,
    lateral_face_approach_command,
)
import pytest
import yaml


def test_lateral_steering_is_continuous_across_arrival_tolerance():
    errors = (.004999, .005, .005001)
    commands = [lateral_face_approach_command(.3, 0., error, .31, .005)
                for error in errors]
    assert all(angular > 0. for _linear, angular in commands)
    assert commands[-1][1] - commands[0][1] < 1e-4


@pytest.mark.parametrize('angle_deg', [-30.0, -12.0, 12.0, 30.0])
@pytest.mark.parametrize('heading', [0.0, math.pi - 0.01])
def test_preview_aligns_before_precision_band(angle_deg, heading):
    face = (0.5 * math.cos(heading), 0.5 * math.sin(heading))
    normal = (-math.cos(heading), -math.sin(heading))
    poses = face_approach_waypoints(
        (0.0, 0.0, heading + math.radians(angle_deg)), face, normal, 0.065, 0.225)
    assert len(poses) > 1
    final = poses[-1]
    residual = math.atan2(math.sin(heading - final[2]), math.cos(heading - final[2]))
    gap = (sum((final[i] - face[i]) * normal[i] for i in (0, 1))
           - 0.065 * math.cos(residual) - 0.225 * abs(math.sin(residual)))
    assert abs(residual) <= math.radians(0.3)
    assert 0.049 <= gap <= 0.0505
    assert math.dist(poses[0][:2], final[:2]) <= 0.6


def test_close_aligned_face_keeps_existing_precision_band():
    assert face_approach_waypoints(
        (0.4, 0.0, 0.0), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225) == [(0.4, 0.0, 0.0)]
    with pytest.raises(ValueError, match='more clearance'):
        face_approach_waypoints(
            (0.4, 0.0, 0.1), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225)


def test_close_table_alignment_turns_toward_face_without_backing_out():
    error = .014061414934757988
    nearest = .033676181083231255
    face = (.065 * math.cos(error) + .225 * math.sin(error) + nearest, 0.)
    poses = face_approach_waypoints(
        (0., 0., error), face, (-1., 0.), .065, .225,
        allow_near_alignment=True)
    assert len(poses) > 1
    assert all(pose[:2] == (0., 0.) for pose in poses)
    assert abs(poses[-1][2]) <= math.radians(.3)
    gaps = [face[0] - .065 * math.cos(p[2]) - .225 * abs(math.sin(p[2])) for p in poses]
    assert min(gaps) >= nearest - 1e-9
    assert all(a <= b + 1e-9 for a, b in zip(gaps, gaps[1:]))
    linear, angular = face_approach_command(.02368, -error, allow_near_alignment=True)
    assert linear == 0.
    assert angular < 0.


@pytest.mark.parametrize('angle', [-.23, 0., .23])
def test_rear_coarse_preview_stops_at_intermediate_gap(angle):
    normal = math.cos(angle), math.sin(angle)
    offset = -1.45 - .275 * normal[0] - .225 * abs(normal[1])
    face = offset / normal[0], 0.
    target_gap = 1.45 - .5 * math.cos(angle)
    poses = face_approach_waypoints(
        (0., 0., math.pi), face, normal, .275, .225, target_gap,
        command_heading_rad=angle + math.pi)
    final = poses[-1]
    error = math.atan2(math.sin(angle + math.pi - final[2]),
                       math.cos(angle + math.pi - final[2]))
    gap = (sum((final[i] - face[i]) * normal[i] for i in (0, 1))
           - .275 * math.cos(error) - .225 * abs(math.sin(error)))
    assert target_gap - .001 <= gap <= target_gap + .0005
    assert math.dist(poses[0][:2], final[:2]) <= .6


@pytest.mark.parametrize('remaining,error', [(math.nan, 0.0), (0.1, math.inf)])
def test_command_rejects_nonfinite_inputs(remaining, error):
    with pytest.raises(ValueError):
        face_approach_command(remaining, error)


def test_command_preserves_straight_speed_and_requires_early_alignment():
    linear, angular = face_approach_command(0.3, math.radians(12.0))
    assert 0.0 < linear < 0.05
    assert angular == 0.02
    with pytest.raises(ValueError, match='more clearance'):
        face_approach_command(0.02, math.radians(12.0))
    assert face_approach_command(0.3, 0.0) == (0.05, 0.0)


def test_command_tapers_only_inside_the_final_approach_band():
    assert face_approach_command(0.026, 0.0) == (0.05, 0.0)
    assert face_approach_command(0.020, 0.0) == pytest.approx((0.04, 0.0))
    assert face_approach_command(0.005, 0.0) == (0.01, 0.0)


@pytest.mark.parametrize('angle_deg', [-3., 0., 3.])
def test_continuous_table_uses_cruise_instead_of_heading_crawl(angle_deg):
    linear, angular = face_approach_command(
        .3, math.radians(angle_deg), continuous_table=True)
    assert linear == .1
    assert abs(angular) <= .08


def test_large_table_error_aligns_once_without_slow_translation():
    linear, angular = face_approach_command(.3, math.radians(12.), continuous_table=True)
    assert linear == 0.
    assert angular == .08
    assert face_approach_command(.1, 0., continuous_table=True) == (.1, 0.)
    assert face_approach_command(.025, 0., continuous_table=True) == (.025, 0.)
    assert face_approach_command(.005, 0., continuous_table=True) == (.01, 0.)
    # The rear/default profile retains its earlier alignment law.
    assert 0. < face_approach_command(.3, math.radians(12.))[0] < .05


def test_lateral_tracking_steers_without_treating_command_as_face_error():
    command_error = math.radians(12.)

    linear, angular = face_approach_command(
        .3, command_error, continuous_table=True,
        lateral_tracking=True, face_error_rad=0.)

    assert linear == .1
    assert angular == .08
    # The same angle is still a stop-and-align request in the default mode.
    assert face_approach_command(
        .3, command_error, continuous_table=True)[0] == 0.


def test_lateral_tracking_keeps_face_clearance_gate():
    linear, angular = face_approach_command(
        .03, math.radians(-4.), continuous_table=True,
        surface_gap_m=.04, allow_near_alignment=True,
        lateral_tracking=True, face_error_rad=math.radians(12.))

    assert linear == 0.
    assert angular < 0.


@pytest.mark.parametrize('face_error', [None, math.nan])
def test_lateral_tracking_requires_a_finite_face_error(face_error):
    with pytest.raises(ValueError, match='finite face error'):
        face_approach_command(
            .3, 0., continuous_table=True, lateral_tracking=True,
            face_error_rad=face_error)


@pytest.mark.parametrize('filtered_deg', [3., 12., 30.])
def test_continuous_table_preserves_near_rate_with_raw_filtered_heading_difference(filtered_deg):
    gap = .049
    remaining = (gap - .01) / math.cos(math.radians(30.))
    linear, angular = face_approach_command(
        remaining, math.radians(filtered_deg), continuous_table=True,
        surface_gap_m=gap, allow_near_alignment=True)
    assert linear == 0.
    assert abs(angular) <= .02


def test_near_table_preview_matches_command_with_filtered_heading():
    yaw = math.radians(30.)
    gap = .049
    plane_x = .065*math.cos(yaw) + .225*math.sin(yaw) + gap
    remaining = (gap-.01) / math.cos(yaw)
    filtered_heading = yaw-math.radians(3.)
    linear, angular = face_approach_command(
        remaining, filtered_heading-yaw, continuous_table=True,
        surface_gap_m=gap, allow_near_alignment=True)
    poses = face_approach_waypoints(
        (0.,0.,yaw), (plane_x,0.), (-1.,0.), .065, .225,
        continuous_table=True, allow_near_alignment=True,
        command_heading_rad=filtered_heading)
    assert linear == 0.
    assert poses[1] == pytest.approx((0.,0.,yaw+angular*.05))


def test_lateral_preview_does_not_stop_for_intentional_command_offset():
    command_heading = math.radians(8.)
    poses = face_approach_waypoints(
        (0., 0., 0.), (.5, 0.), (-1., 0.), .065, .225,
        continuous_table=True, allow_near_alignment=True,
        command_heading_rad=command_heading, lateral_tracking=True,
        lateral_goal_odom=(.425, .03, 0.), lateral_tolerance_m=.005,
        max_lateral_heading_rad=math.radians(30.))

    assert len(poses) > 2
    assert poses[1][0] > poses[0][0]
    assert abs(poses[1][2]) > 0.


@pytest.mark.parametrize(('field', 'value'), [
    ('lateral_goal_odom', None),
    ('lateral_tolerance_m', None),
    ('max_lateral_heading_rad', None),
])
def test_lateral_preview_requires_the_live_control_contract(field, value):
    kwargs = {
        'lateral_goal_odom': (.425, 0., 0.),
        'lateral_tolerance_m': .005,
        'max_lateral_heading_rad': math.radians(30.),
    }
    kwargs[field] = value
    with pytest.raises(ValueError, match='lateral preview'):
        face_approach_waypoints(
            (0., 0., 0.), (.5, 0.), (-1., 0.), .065, .225,
            continuous_table=True, lateral_tracking=True, **kwargs)


def _actual_lateral_rollout(start_y, start_yaw):
    face_x = .5
    x, y, yaw = 0., start_y, start_yaw
    step_s = .05
    poses = [(x, y, yaw)]
    stopped_before_band = False

    for _step in range(200):
        face_error = -yaw
        center_gap = face_x - x - .065 * math.cos(face_error)
        gap = center_gap - .225 * abs(math.sin(face_error))
        # Lateral tracking runs to the target-gap precision band (b4e5cdd),
        # not the older 5 cm face-alignment band.
        if gap <= .01 + FACE_ALIGNMENT_ENTRY_MARGIN_M:
            return poses, stopped_before_band
        remaining = (center_gap - .01) / math.cos(face_error)
        linear, angular = lateral_face_approach_command(
            remaining, face_error, -y, gap, .005, allow_near_alignment=True)
        stopped_before_band |= linear == 0.
        x += linear * math.cos(yaw) * step_s
        y += linear * math.sin(yaw) * step_s
        yaw += angular * step_s
        poses.append((x, y, yaw))
    pytest.fail('lateral tracking rollout did not reach precision band')


@pytest.mark.parametrize(('start_y', 'start_yaw'), [
    (-.028, 0.),
    (-.04, math.radians(8.)),
    (.04, math.radians(-8.)),
])
def test_lateral_tracking_reaches_precision_band_aligned_without_stop_turn(
        start_y, start_yaw):
    """Exercise the real filter, 0.08 rad/s limit and command law together."""
    poses, stopped_before_band = _actual_lateral_rollout(start_y, start_yaw)
    _x, y, yaw = poses[-1]

    assert not stopped_before_band
    assert abs(y) <= .005
    assert abs(yaw) <= math.radians(.3)


def test_lateral_preview_matches_live_rollout_swept_range_and_endpoint():
    start = (0., -.04, math.radians(8.))
    goal = (.425, 0., 0.)
    actual, stopped = _actual_lateral_rollout(start[1], start[2])
    preview = face_approach_waypoints(
        start, (.5, 0.), (-1., 0.), .065, .225,
        continuous_table=True, allow_near_alignment=True,
        lateral_tracking=True, lateral_goal_odom=goal,
        lateral_tolerance_m=.005,
        max_lateral_heading_rad=math.radians(30.))

    assert not stopped
    assert min(pose[1] for pose in preview) == pytest.approx(
        min(pose[1] for pose in actual), abs=.001)
    assert max(pose[1] for pose in preview) == pytest.approx(
        max(pose[1] for pose in actual), abs=.001)
    assert preview[-1][:2] == pytest.approx(actual[-1][:2], abs=.002)
    assert preview[-1][2] == pytest.approx(actual[-1][2], abs=math.radians(.2))
    assert abs(preview[-1][1] - start[1]) >= .035


def test_logged_water_station_lateral_offset_reaches_precision_band():
    # 2026-10-27: the prior preview exhausted its iteration budget at this pose.
    start = (-0.36201116062983996, -0.11271175325312832, -0.05774233784610361)
    face = (-0.002282984304820865, -0.07559402843585278)
    normal = (-0.9987171987184517, 0.05063553084513732)
    goal = (0.024608312304077895, -0.07695743249122788, -0.025996507601880273)
    poses = face_approach_waypoints(
        start, face, normal, .065, .225, continuous_table=True,
        allow_near_alignment=True, lateral_tracking=True,
        lateral_goal_odom=goal, lateral_tolerance_m=.005,
        max_lateral_heading_rad=math.radians(30.))
    heading = math.atan2(-normal[1], -normal[0])
    left = (-math.sin(heading), math.cos(heading))
    lateral_error = sum((goal[i] - poses[-1][i]) * left[i] for i in (0, 1))
    assert abs(lateral_error) <= .005
    assert abs(poses[-1][2] - heading) <= math.radians(.3)
    assert all(sum((p[i] - face[i]) * normal[i] for i in (0, 1))
               - .065 * math.cos(heading - p[2])
               - .225 * abs(math.sin(heading - p[2])) > 0 for p in poses)


@pytest.mark.parametrize(('remaining_m', 'heading_error_rad', 'lateral_error_m'), [
    # mission_20261102_192956_dac49049 table_alignment_transit_skipped
    (.28248187259350843, -.3162350320844097, .010362658669407773),
    # mission_20261102_191252_65d8accb table_alignment_transit_skipped
    (.2936478404024305, -.16447163408767784, .07210256520408),
])
def test_logged_alignment_transit_reaches_final_precision_band(
        remaining_m, heading_error_rad, lateral_error_m):
    yaw = -heading_error_rad
    face_x = (remaining_m + .065) * math.cos(heading_error_rad) + .01
    start = (0., -lateral_error_m, yaw)
    goal = (face_x - .065 - .01, 0., 0.)

    poses = face_approach_waypoints(
        start, (face_x, 0.), (-1., 0.), .065, .225,
        continuous_table=True, allow_near_alignment=True,
        lateral_tracking=True, lateral_goal_odom=goal,
        lateral_tolerance_m=.005,
        max_lateral_heading_rad=math.radians(30.))

    final = poses[-1]
    assert abs(final[1]) <= .005
    assert abs(final[2]) <= math.radians(.3)


def test_logged_near_lateral_error_requests_clearance_instead_of_looping():
    start = (-0.08223210939063952, -0.09532518952372127, -0.030208263284322398)
    face = (0.003239895298168459, -0.11427502896258854)
    normal = (-0.99994042141429, 0.010915751087856827)
    goal = (-0.08234056, -0.1052593, -0.010915967874693955)
    with pytest.raises(ValueError, match='lateral alignment requires more clearance'):
        face_approach_waypoints(
            start, face, normal, .065, .225, continuous_table=True,
            allow_near_alignment=True, lateral_tracking=True,
            lateral_goal_odom=goal, lateral_tolerance_m=.005,
            max_lateral_heading_rad=math.radians(30.))


@pytest.mark.parametrize('angle_deg', [-30., -12., -3., 3., 12., 30.])
@pytest.mark.parametrize('distance_m', [.25, .5])
def test_continuous_table_rollout_aligns_without_crossing_surface(angle_deg, distance_m):
    poses = face_approach_waypoints(
        (0., 0., math.radians(angle_deg)), (distance_m, 0.), (-1., 0.), .065, .225,
        allow_near_alignment=True, continuous_table=True)
    for x_m, _y_m, yaw_rad in poses:
        nearest_m = distance_m - x_m - .065*math.cos(yaw_rad) - .225*abs(math.sin(yaw_rad))
        assert nearest_m > 0.
    assert abs(poses[-1][2]) <= math.radians(.3)


def test_heading_filter_crosses_pi_along_shortest_arc():
    filtered = filtered_face_heading(math.pi - 0.01, -math.pi + 0.01, 0.35)
    assert filtered == pytest.approx(math.pi - 0.01 + (1.0 - math.exp(-1.0)) * 0.02)


def test_preview_uses_current_filtered_heading_then_converges_to_raw_normal():
    start = (0.0, 0.0, math.radians(12.0))
    filtered = math.radians(-5.0)
    poses = face_approach_waypoints(
        start, (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225,
        command_heading_rad=filtered)
    remaining = (0.5 - 0.065 * math.cos(start[2])
                 - 0.225 * abs(math.sin(start[2])) - 0.01) / math.cos(start[2])
    linear, angular = face_approach_command(remaining, filtered - start[2])
    assert poses[1] == pytest.approx((linear * math.cos(start[2]) * 0.05,
                                     linear * math.sin(start[2]) * 0.05,
                                     start[2] + angular * 0.05))
    assert abs(poses[-1][2]) <= math.radians(0.3)


def test_curved_corridor_rejects_keepout_missed_by_straight_plan(tmp_path):
    poses = face_approach_waypoints(
        (0.0, 0.0, math.radians(30.0)), (0.5, 0.0), (-1.0, 0.0), 0.065, 0.225)
    point = max(poses, key=lambda pose: pose[1])
    resolution, width, height = 0.01, 80, 80
    origin = (-0.1, -0.4, 0.0)
    pixels = bytearray([254] * (width * height))
    column = math.floor((point[0] - origin[0]) / resolution)
    map_row = math.floor((point[1] - origin[1]) / resolution)

    def write(name, data):
        image = tmp_path / (name + '.pgm')
        image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes(data))
        path = tmp_path / (name + '.yaml')
        path.write_text(yaml.safe_dump({
            'image': image.name, 'resolution': resolution, 'origin': list(origin),
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
        return path

    source = write('map', pixels)
    pixels[(height - 1 - map_row) * width + column] = 0
    mask = write('mask', pixels)
    footprint = [(-0.003, -0.003), (0.003, -0.003), (0.003, 0.003), (-0.003, 0.003)]
    assert static_corridor_clear(source, mask, [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0)], footprint)
    assert not static_corridor_clear(source, mask, poses, footprint)
