"""Replay rejected real-run geometry without robot actions or copied sensor validity."""
import contextlib
import json
import math
from collections import deque
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from jdamr_cube_navigation import box_service as module
from jdamr_cube_navigation.box_service import BoxServiceRoute

CASES = json.loads((Path(__file__).parent / 'fixtures/kitchen_tape_restage_20261101.json').read_text())


@pytest.mark.parametrize('case', CASES, ids=lambda c: c['run'])
@pytest.mark.parametrize('map_shift', [0., .16])
@pytest.mark.parametrize('reported_errors', [False, True])
@pytest.mark.parametrize('rebase', [False, True])
def test_rejected_tape_goal_survives_reverse_and_drives_sideways_restage(monkeypatch, case, map_shift, reported_errors, rebase):
    node = object.__new__(BoxServiceRoute)
    actual = list(case['start_pose_odom'])
    goal = case['lateral_goal_odom']
    face = case['face_center_odom']
    normal = case['outward_normal_odom']
    heading = math.atan2(-normal[1], -normal[0])
    left = (-math.sin(heading), math.cos(heading))
    if reported_errors:
        # User-reported summaries supplement the exact recorded preview inputs.
        lateral, angle_deg = ((.078, -8.0) if '125748' in case['run'] else (.098, -6.2))
        previous = sum((goal[i]-actual[i])*left[i] for i in (0,1))
        actual[0] += (previous-lateral)*left[0]
        actual[1] += (previous-lateral)*left[1]
        actual[2] = heading - math.radians(angle_deg)
        # The summaries omit distance. Use an explicitly synthetic 20 cm
        # nearest-corner gap; exact logged distances are covered above.
        distance = sum((actual[i]-face[i])*normal[i] for i in (0,1))
        wanted = .20 + case['front_extent_m']*math.cos(math.radians(angle_deg)) + case['half_width_m']*abs(math.sin(math.radians(angle_deg)))
        actual[0] += (wanted-distance)*normal[0]
        actual[1] += (wanted-distance)*normal[1]
    extent, half_width = case['front_extent_m'], case['half_width_m']
    node.box_front_half_width_m = half_width
    node.station_reference = SimpleNamespace(observation_method='table_front_tape')
    node.station_reference_mode = 'rgbd'
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_goal = {'valid': True, 'goal_odom': list(goal), 'stamp_s': 1.}
    node.station_reference_control_goal = dict(node.station_reference_goal)
    node.station_reference_goal_frozen = True
    node._station_reference_lock = Lock()
    node._station_reference_goal_samples = deque([dict(node.station_reference_goal)])
    node._station_terminal_goal = None
    node.surface_mode = 'table_edge'
    node.final_gap_override_m = .01
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._navigation_ready = Mock(return_value=True)
    node._table_observation_context = ({'camera_mount': {'table_observation': {'base_offset_m': .5}}}, (0., 0.), 1.)
    node._odom_pose = lambda: tuple(actual)
    # A changed AMCL origin must not move the carried physical destination.
    def to_odom(x, y, yaw):
        return x, y + map_shift, yaw
    node._frozen_in_odom = lambda _p: (None, to_odom)
    node.registry = {'map': {'yaml_path': 'map'}, 'keepout': {'yaml_path': 'mask'}}
    node._footprint_outline = lambda: [(-.275, -.225), (.065, .225)]
    corridor = Mock(return_value=True)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', lambda *_a: corridor)
    node.emit = Mock()
    node._localization_bound = lambda _v: contextlib.nullcontext()
    node._station_lateral_status = lambda: {'internal_lateral_within_tolerance': False}
    order = []
    planned = []

    def observation(*_a):
        # One tape stripe is visible: the coarse map/face goal has almost no
        # lateral error, even though the accepted tape goal is far off-centre.
        center = [face[0], face[1] - map_shift]
        target = {'x_m': center[0] + normal[0] * (extent + .01),
                  'y_m': center[1] + normal[1] * (extent + .01),
                  'yaw_rad': heading, 'face_center_map_xy_m': center,
                  'outward_normal_map_xy': normal,
                  'observation_robot_pose_map': [actual[0], actual[1]-map_shift, actual[2]]}
        target = module.anchor_face_to_taught_lateral(target, (actual[0] + .006*left[0], actual[1]-map_shift + .006*left[1]))
        order.append('observe')
        return target

    def reverse(**_kwargs):
        order.append('reverse')
        actual[0] -= .06 * math.cos(actual[2])
        actual[1] -= .06 * math.sin(actual[2])
        return True
    node._leave_parked_pose = reverse
    node.observe_target = observation
    def plan(target, **_kwargs):
        order.append('restage')
        planned.append(target)
        return {'ok': True}
    node.plan_pose = plan
    def execute(**_kwargs):
        pose = planned[-1]
        actual[:] = to_odom(pose['x_m'], pose['y_m'], pose['yaw_rad'])
        return True
    node.execute = execute
    attempts = []
    def approach(_target, _extent, **_kwargs):
        attempts.append(tuple(actual))
        validator = node._table_approach_validator(extent, lambda p: (p[0], p[1]-map_shift, p[2]))
        held = node.station_reference_goal
        try:
            if rebase and len(attempts) == 1:
                # A near three-frame consensus arrives after the old target froze.
                fresh = dict(held, goal_filter_sample_count=3)
                node._fresh_station_reference = lambda: fresh
                node._station_reference_rebase_goal = fresh
                node._station_reference_goal_challenged = True
                node._station_goal_reference(freeze=True)
                assert node._table_rebase_requires_restage
                raise RuntimeError('table lateral alignment requires more clearance')
            validator(tuple(actual), face, normal, heading,
                      lateral_goal_odom=held['goal_odom'])
        except RuntimeError as error:
            node._last_direct_motion_failure = str(error)
            return False, None
        node._last_direct_motion_failure = None
        return True, 'preview_passed'
    node._zero_turn_final_approach = approach
    initial = observation()
    initial.pop('observation_robot_pose_map')
    assert node._table_fine_approach(initial, extent) == (True, 'preview_passed')
    assert len(attempts) == 2 and len(planned) == 1
    assert order.index('reverse') < order.index('restage')
    held = node._table_restage_tape_goal
    assert held['goal_odom'] == goal
    rejection_left = (-math.sin(goal[2]), math.cos(goal[2])) if rebase else left
    assert held['rejection_lateral_m'] == pytest.approx(sum((goal[i]-attempts[0][i])*rejection_left[i] for i in (0,1)))
    assert node.station_reference_goal_frozen
    assert len(node._station_reference_goal_samples) == 1
    assert abs(sum((goal[i]-actual[i])*left[i] for i in (0,1))) < 1e-9
    assert actual[2] == pytest.approx(heading)
    assert abs(sum((attempts[0][i]-actual[i])*left[i] for i in (0,1))) > .07
    assert corridor.called  # real preview succeeded; static checking was retained
    assert node._station_terminal_goal is None  # preview pass is not final arrival
    assert node._final_gap_m() == .01
