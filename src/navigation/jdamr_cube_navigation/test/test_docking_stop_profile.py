"""Tests for the bounded forward docking StopZone profile."""

from copy import deepcopy
import json
import math
from pathlib import Path

from jdamr_cube_navigation.docking_stop_profile import (
    apply_docking_stop_profile,
    MINIMUM_PADDED_STOP_MARGIN_M,
    USER_APPROVED_CLEARANCE_M,
)
from jdamr_cube_navigation.new_base_contract import validate_new_base_params
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / 'jdamr_cube_navigation/config/new_base_nav2_params.yaml'
GEOMETRY = ROOT / 'jdamr_cube_description/config/new_base_geometry.yaml'


def documents():
    return (
        yaml.safe_load(PARAMS.read_text(encoding='utf-8')),
        yaml.safe_load(GEOMETRY.read_text(encoding='utf-8')),
    )


def points(value):
    return json.loads(value) if isinstance(value, str) else value


def changed_paths(before, after, prefix=()):
    if isinstance(before, dict) and isinstance(after, dict):
        paths = set()
        for key in before.keys() | after.keys():
            paths.update(changed_paths(
                before.get(key), after.get(key), (*prefix, key)))
        return paths
    if before != after:
        return {prefix}
    return set()


def test_only_stopzone_front_and_slowdown_switch_change_and_input_is_untouched():
    nav2, geometry = documents()
    original = deepcopy(nav2)
    output = apply_docking_stop_profile(nav2, geometry)
    assert nav2 == original
    assert changed_paths(nav2, output) == {
        ('collision_monitor', 'ros__parameters', 'SlowdownZone', 'enabled'),
    }
    assert USER_APPROVED_CLEARANCE_M == pytest.approx(0.40)



@pytest.mark.parametrize('name', [
    'translation_forward', 'translation_forward_straight', 'stopped'])
def test_only_front_x_coordinates_change_inside_target_polygons(name):
    nav2, geometry = documents()
    output = apply_docking_stop_profile(nav2, geometry)
    before = points(nav2['collision_monitor']['ros__parameters'][
        'StopZone'][name]['points'])
    after = points(output['collision_monitor']['ros__parameters'][
        'StopZone'][name]['points'])
    expected_front_m = 0.065 + USER_APPROVED_CLEARANCE_M
    for old_point, new_point in zip(before, after):
        assert new_point[1] == old_point[1]
        if old_point[0] == max(point[0] for point in before):
            assert new_point[0] == pytest.approx(expected_front_m)
        else:
            assert new_point[0] == old_point[0]


def test_backward_side_rotation_scan_timeouts_and_approach_are_identical():
    nav2, geometry = documents()
    nav2['unknown_future_nav2_key'] = {'preserve': [1, 2, 3]}
    output = apply_docking_stop_profile(nav2, geometry)
    before = nav2['collision_monitor']['ros__parameters']
    after = output['collision_monitor']['ros__parameters']
    assert after['StopZone']['translation_backward'] == (
        before['StopZone']['translation_backward'])
    assert after['StopZone']['rotation'] == before['StopZone']['rotation']
    assert after['StopZone']['rotation_clockwise'] == (
        before['StopZone']['rotation_clockwise'])
    assert after['SlowdownZone'] == {**before['SlowdownZone'], 'enabled': False}
    assert after['FootprintApproach'] == before['FootprintApproach']
    assert after['scan'] == before['scan']
    assert after['source_timeout'] == before['source_timeout']
    assert after['stop_pub_timeout'] == before['stop_pub_timeout']
    assert after['StopZone']['min_points'] == before['StopZone']['min_points']
    assert output['unknown_future_nav2_key'] == {'preserve': [1, 2, 3]}


def test_target_remains_ahead_of_padded_footprint_by_required_margin():
    nav2, geometry = documents()
    output = apply_docking_stop_profile(nav2, geometry)
    footprint_front = max(point[0] for point in points(nav2[
        'local_costmap']['local_costmap']['ros__parameters']['footprint']))
    target_front = max(point[0] for point in points(output[
        'collision_monitor']['ros__parameters']['StopZone'][
            'translation_forward']['points']))
    assert target_front - footprint_front >= MINIMUM_PADDED_STOP_MARGIN_M
    assert USER_APPROVED_CLEARANCE_M >= footprint_front - 0.065


@pytest.mark.parametrize(
    'mutate, message', [
        (lambda _nav, geometry: geometry['front_to_wheel_axis'].update(
            value=math.nan), 'finite and positive'),
        (lambda _nav, geometry: geometry['front_to_wheel_axis'].update(
            value=geometry['frame_length']['value']), 'smaller than'),
        (lambda nav, _geometry: nav['local_costmap']['local_costmap'][
            'ros__parameters'].update(footprint=[
                [0.13, 0.29], [0.13, -0.29], [-0.295, -0.29],
                [-0.295, 0.29]]), 'footprint fronts differ'),
        (lambda nav, _geometry: [
            nav[name][name]['ros__parameters'].update(footprint=[
                [0.45, 0.29], [0.45, -0.29], [-0.295, -0.29],
                [-0.295, 0.29]])
            for name in ('local_costmap', 'global_costmap')],
         'lacks padded footprint margin'),
    ],
)
def test_invalid_geometry_or_clearance_contract_is_rejected(mutate, message):
    nav2, geometry = documents()
    mutate(nav2, geometry)
    with pytest.raises(ValueError, match=message):
        apply_docking_stop_profile(nav2, geometry)


def test_unknown_or_nonrectangular_target_is_rejected():
    nav2, geometry = documents()
    nav2['collision_monitor']['ros__parameters']['StopZone'][
        'translation_forward']['points'] = '[[0.135, 0.29], [0.1, -0.29]]'
    with pytest.raises(ValueError, match='four finite XY points'):
        apply_docking_stop_profile(nav2, geometry)


def test_precision_profile_requires_explicit_launch_contract():
    nav2, geometry = documents()
    profile = apply_docking_stop_profile(nav2, geometry)
    validate_new_base_params(nav2, geometry)
    with pytest.raises(RuntimeError, match='margin|SlowdownZone is not active'):
        validate_new_base_params(profile, geometry)
    validate_new_base_params(profile, geometry, precision_parking=True)


def test_t37_standard_session_still_requires_the_slowdown_ring():
    """Only the precision session may run without the slowdown ring."""
    nav2, geometry = documents()
    nav2['collision_monitor']['ros__parameters']['SlowdownZone']['enabled'] = False
    with pytest.raises(RuntimeError, match='SlowdownZone is not active'):
        validate_new_base_params(nav2, geometry)


def test_precision_contract_still_rejects_smaller_than_requested_clearance():
    nav2, geometry = documents()
    profile = apply_docking_stop_profile(nav2, geometry)
    for name in ('translation_forward', 'translation_forward_straight', 'stopped'):
        polygon = points(profile['collision_monitor']['ros__parameters'][
            'StopZone'][name]['points'])
        front = max(point[0] for point in polygon)
        for point in polygon:
            if point[0] == front:
                point[0] = 0.105
        profile['collision_monitor']['ros__parameters'][
            'StopZone'][name]['points'] = json.dumps(polygon)
    with pytest.raises(RuntimeError, match='margin|shape|clearance'):
        validate_new_base_params(profile, geometry, precision_parking=True)


def test_t37_precision_final_approach_outruns_the_progress_checker():
    """
    The slowest final-approach command must clear the progress checker twice over.

    On 2026-09-30 the water-station approach stopped 8 cm short: the Parking
    minimum 0.01 m/s was cut to 0.004 m/s by the 0.6 SlowdownZone, below the
    0.05 m / 10 s progress requirement (FAILED_TO_MAKE_PROGRESS, 105).
    """
    nav2, geometry = documents()
    output = apply_docking_stop_profile(nav2, geometry)
    monitor = output['collision_monitor']['ros__parameters']
    contract = yaml.safe_load((ROOT / 'jdamr_cube_navigation/config/'
                               'box_parking_contract.yaml').read_text(encoding='utf-8'))
    progress = output['controller_server']['ros__parameters']['progress_checker']
    required_mps = (progress['required_movement_radius']
                    / progress['movement_time_allowance'])
    slowdown = monitor['SlowdownZone']
    factor = (1.0 - slowdown['slowdown_ratio']) if slowdown.get('enabled', True) else 1.0
    assert contract['min_approach_linear_mps'] * factor >= 2.0 * required_mps


def test_t37_precision_profile_keeps_stop_and_approach_protection():
    """Disabling the slowdown ring leaves the approved StopZone and approach active."""
    nav2, geometry = documents()
    monitor = apply_docking_stop_profile(nav2, geometry)['collision_monitor']['ros__parameters']
    assert monitor['StopZone']['enabled'] is True
    assert monitor['FootprintApproach']['enabled'] is True
    assert monitor['SlowdownZone']['enabled'] is False
    assert 'SlowdownZone' in monitor['polygons']


def test_standard_session_rejects_legacy_front_margin():
    nav2, geometry = documents()
    for name in ('translation_forward', 'translation_forward_straight', 'stopped'):
        polygon = points(nav2['collision_monitor']['ros__parameters']['StopZone'][name]['points'])
        for point in polygon:
            if point[0] == 0.465:
                point[0] = 0.12
        nav2['collision_monitor']['ros__parameters']['StopZone'][name]['points'] = json.dumps(polygon)
    with pytest.raises(RuntimeError, match='margin'):
        validate_new_base_params(nav2, geometry)
