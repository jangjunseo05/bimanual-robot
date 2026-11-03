"""Tests for preparing a current manual-map runtime asset bundle."""

import hashlib
import json
import math
from pathlib import Path

import pytest
import yaml

from jdamr_cube_navigation.keepout_mask import _read_pgm
from jdamr_cube_navigation.map_session_assets import prepare_map_session
from jdamr_cube_navigation.service_destinations import load_registry, map_identity


def _write_fixture(tmp_path):
    source = tmp_path / 'capture'
    source.mkdir()
    pixels = bytes([0, 205, 254, 205, 0, 254])
    image = source / 'refined.pgm'
    image.write_bytes(b'P5\n3 2\n255\n' + pixels)
    map_yaml = source / 'refined.yaml'
    map_yaml.write_text(yaml.safe_dump({
        'image': 'refined.pgm', 'mode': 'trinary', 'resolution': 0.05,
        'origin': [-1.0, -2.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8')
    identity = map_identity(map_yaml)
    home = {
        'id': 'home_dock', 'frame_id': 'map', 'robot_base_frame': 'base_link',
        'x_m': -0.8, 'y_m': 0.4, 'yaw_rad': 0.1,
        'parking_direction': 'reverse', 'source': 'operator_dock',
        'captured_at': '2026-10-26T07:17:28+00:00',
        'physical_accuracy': 'NOT_EXTERNALLY_MEASURED', 'table_gap_m': None,
        'scan_path': str(source / 'dock_scan.json'), 'map': identity,
    }
    (source / 'home.yaml').write_text(
        yaml.safe_dump(home, sort_keys=False), encoding='utf-8')
    dock_scan = {
        'scans': [{
            'stamp': 1., 'frame': 'laser_link', 'angle_min': -1.,
            'angle_increment': .1, 'range_min': .05, 'range_max': 12.,
            'ranges': [1., 1.1],
        }],
        'odom': [{'stamp': 1., 'x': -.8, 'y': .4, 'qz': .05, 'qw': .99,
                  'v': 0., 'w': 0.}],
        'battery': [{'stamp': 1., 'voltage': 12.}],
        'tf_static': [{'parent': 'base_link', 'child': 'laser_link'}],
        'map_to_base_before_finish': {'parent': 'map', 'child': 'base_link'},
        'base_to_laser': {'parent': 'base_link', 'child': 'laser_link'},
    }
    (source / 'dock_scan.json').write_text(
        json.dumps(dock_scan), encoding='utf-8')
    dock_reference = {
        'scan_path': str(source / 'dock_scan.json'),
        'pose_map': [-.8, .4, .1], 'home_pose': [-.8, .4, .1],
        'map_identity': [identity['yaml_sha256'], identity['image_sha256']],
    }
    (source / 'dock_scan_reference.json').write_text(
        json.dumps(dock_reference), encoding='utf-8')
    station = {
        'id': 'water_station', 'frame_id': 'map', 'robot_base_frame': 'base_link',
        'x_m': 1.2, 'y_m': -0.3, 'yaw_rad': math.pi / 2,
        'source': 'operator_station', 'captured_at': '2026-10-26T08:17:18+00:00',
        'target_front_gap_m': 0.01, 'lateral_tolerance_m': 0.005,
        'physical_accuracy': 'NOT_EXTERNALLY_MEASURED', 'map': identity,
        'capture_folder': str(source / 'stations/water_station'),
        'reference_capture': str(source / 'stations/water_station/reference.json'),
        'camera_mount_calibration': 'NEW_ASTRA_MOUNT_NOT_YET_RECALIBRATED',
        'object_reference_role': 'auxiliary_only_do_not_redefine_destination',
    }
    (source / 'water_station.yaml').write_text(
        yaml.safe_dump(station, sort_keys=False), encoding='utf-8')
    table = {
        'id': 'table_01', 'frame_id': 'map', 'robot_base_frame': 'base_link',
        'x_m': -0.734910483363684, 'y_m': -1.6068820918282494,
        'yaw_rad': -0.03841021709958648,
        'source': 'operator_taught_physical_pose_global_lidar_match_then_tf',
        'captured_at': '2026-10-27T00:57:14+00:00',
        'target_front_gap_m': 0.01,
        'physical_accuracy': 'NOT_EXTERNALLY_MEASURED', 'map': identity,
        'capture_folder': str(source / 'stations/table_01'),
        'registration_status': 'CONFIRMED_BY_OPERATOR',
        'lateral_reference': 'operator_taught_map_pose_without_visual_landmark',
        'lateral_precision_status': 'NOT_VALIDATED',
        'object_reference_role': 'none_featureless_table',
        'operating_registry_activated': False,
        'pose_receipt': str(source / 'stations/table_01/current_pose_receipt.json'),
        'pose_receipt_sha256': '1' * 64,
    }
    (source / 'table_01.yaml').write_text(
        yaml.safe_dump(table, sort_keys=False), encoding='utf-8')
    pointer = tmp_path / 'current.json'
    pointer.write_text(json.dumps({
        'directory': str(source), 'map_yaml': str(map_yaml),
        'home_dock': str(source / 'home.yaml'),
        'dock_scan_reference': str(source / 'dock_scan_reference.json'),
    }), encoding='utf-8')
    geometry = tmp_path / 'geometry.yaml'
    geometry.write_text(
        'front_to_wheel_axis:\n'
        '  value: 0.065\n'
        '  unit: m\n'
        '  source: fixture\n'
        '  measured_at: 2026-09-15\n',
        encoding='utf-8')
    return pointer, geometry, source, map_yaml, image, pixels


def test_prepares_loadable_runtime_without_mutating_sources(tmp_path):
    pointer, geometry, source, map_yaml, image, source_pixels = _write_fixture(tmp_path)
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in source.iterdir() if path.is_file()}

    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    after = {path: hashlib.sha256(path.read_bytes()).hexdigest()
             for path in source.iterdir() if path.is_file()}
    assert before == after
    assert result['session_directory'] == str(source)
    assert result['runtime_directory'] == str(tmp_path / 'runtime')
    assert result['dock_departure_mode'] == 'straight'
    assert result['station_ids'] == ['water_station', 'table_01']
    assert set(Path(result['output_dir']).iterdir()) == {
        tmp_path / 'runtime/map.yaml', tmp_path / 'runtime/map.pgm',
        tmp_path / 'runtime/refined.pgm', tmp_path / 'runtime/keepout.yaml',
        tmp_path / 'runtime/keepout.pgm',
        tmp_path / 'runtime/service_destinations.yaml',
        tmp_path / 'runtime/destinations.json',
        tmp_path / 'runtime/station_profiles.json',
    }
    assert (tmp_path / 'runtime/map.yaml').read_bytes() == map_yaml.read_bytes()
    assert (tmp_path / 'runtime/map.pgm').read_bytes() == image.read_bytes()
    assert (tmp_path / 'runtime/refined.pgm').read_bytes() == image.read_bytes()
    assert result['map_hashes']['source']['yaml_sha256'] == result['map_hashes']['runtime']['yaml_sha256']
    assert result['map_hashes']['source']['image_sha256'] == result['map_hashes']['runtime']['image_sha256']

    width, height, keepout = _read_pgm(tmp_path / 'runtime/keepout.pgm')
    assert (width, height) == (3, 2)
    assert keepout == bytes(0 if pixel == 205 else 254 for pixel in source_pixels)
    keepout_yaml = yaml.safe_load((tmp_path / 'runtime/keepout.yaml').read_text())
    assert keepout_yaml['generation']['scope'] == 'unknown-only'
    assert keepout_yaml['generation']['no_table_keepout_inferred'] is True


def test_map_session_can_select_guarded_nav2_departure(tmp_path):
    pointer, geometry, _, _, _, _ = _write_fixture(tmp_path)
    data = json.loads(pointer.read_text())
    data['dock_departure_mode'] = 'nav2_waypoint'
    pointer.write_text(json.dumps(data))
    assert prepare_map_session(pointer, tmp_path / 'runtime', geometry)[
        'dock_departure_mode'] == 'nav2_waypoint'
    data['dock_departure_mode'] = 'invalid'
    pointer.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='dock_departure_mode'):
        prepare_map_session(pointer, tmp_path / 'other-runtime', geometry)


def test_registry_preserves_taught_poses_and_goal_metadata(tmp_path):
    pointer, geometry, _, _, _, _ = _write_fixture(tmp_path)
    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    registry = load_registry(result['assets']['service_destinations'])
    assert registry['home']['x_m'] == pytest.approx(-0.8)
    assert registry['home']['y_m'] == pytest.approx(0.4)
    assert registry['home']['yaw_rad'] == pytest.approx(0.1)
    assert registry['home']['parking_direction'] == 'reverse'
    assert registry['home']['target_rear_gap_m'] == pytest.approx(0.01)
    assert registry['home']['target_status'] == 'NOT_VALIDATED'
    dock = registry['home']['teaching']['observation']['dock_scan']
    assert dock['pose_map'] == pytest.approx([-.8, .4, .1])
    assert dock['scan_sha256'] == hashlib.sha256(
        (tmp_path / 'capture/dock_scan.json').read_bytes()).hexdigest()
    assert dock['schema']['scan_count'] == 1
    assert [table['table_id'] for table in registry['tables']] == [
        'water_station', 'table_01']
    pose = registry['tables'][0]['service_poses'][0]
    assert (pose['x_m'], pose['y_m'], pose['yaw_rad']) == pytest.approx(
        (1.2, -0.3, math.pi / 2))
    assert pose['approach_offset_m'] == pytest.approx(0.5)
    assert pose['target_front_gap_m'] == pytest.approx(0.01)
    assert pose['lateral_tolerance_m'] == pytest.approx(0.005)
    assert pose['teaching']['table_gap_m'] is None
    assert pose['target_gap_status'] == 'NOT_VALIDATED'

    table_pose = registry['tables'][1]['service_poses'][0]
    assert (table_pose['x_m'], table_pose['y_m'], table_pose['yaw_rad']) == pytest.approx(
        (-0.734910483363684, -1.6068820918282494, -0.03841021709958648))
    assert table_pose['target_front_gap_m'] == pytest.approx(0.01)
    assert table_pose['lateral_tolerance_m'] == pytest.approx(0.005)
    assert table_pose['lateral_precision_status'] == 'NOT_VALIDATED'
    assert table_pose['registration_status'] == 'CONFIRMED_BY_OPERATOR'
    assert table_pose['runtime_registry_activated'] is True
    assert table_pose['source_record_operating_registry_activated'] is False
    assert table_pose['teaching']['observation']['source_record'][
        'operating_registry_activated'] is False

    destinations = json.loads(Path(result['assets']['destinations']).read_text())
    target = destinations['destinations']['water_station']
    assert target['face_xy'] == pytest.approx([1.2, -0.225])
    assert target['normal_deg'] == pytest.approx(270.0)
    assert target['provenance']['status'] == 'DERIVED_NOT_MEASURED'
    assert target['provenance']['front_extent']['measured_at'] == '2026-09-15'
    table_target = destinations['destinations']['table_01']
    table_xyz = (-0.734910483363684, -1.6068820918282494, -0.03841021709958648)
    assert table_target['face_xy'] == pytest.approx([
        table_xyz[0] + 0.075 * math.cos(table_xyz[2]),
        table_xyz[1] + 0.075 * math.sin(table_xyz[2]),
    ])
    assert table_target['provenance']['taught_pose_xy_yaw'] == pytest.approx(table_xyz)
    assert table_target['provenance']['requested_gap_m'] == pytest.approx(0.01)
    assert table_target['provenance']['status'] == 'DERIVED_NOT_MEASURED'
    profiles = json.loads(Path(result['assets']['station_profiles']).read_text())
    profile = profiles['stations']['water_station']
    assert profile['target_priority'] == 'operator_taught_map_pose_and_heading'
    assert profile['actual_measured_gap_m'] is None
    assert profile['approach_offset_role'] == 'route_observation_only_not_final_parking_gap'
    assert profile['reference_mode'] == 'rgbd_auxiliary'
    table_profile = profiles['stations']['table_01']
    assert table_profile['reference_mode'] == 'map_pose_only'
    assert table_profile['rgbd_reference'] is None
    assert table_profile['object_reference_role'] == 'none_featureless_table'
    assert table_profile['lateral_tolerance_m'] == pytest.approx(0.005)
    assert table_profile['lateral_precision_status'] == 'NOT_VALIDATED'
    assert table_profile['actual_measured_gap_m'] is None
    assert table_profile['runtime_registry_activated'] is True
    assert table_profile['source_record_operating_registry_activated'] is False
    assert profiles['keepout']['scope'] == 'unknown-only'
    assert profiles['home']['dock_scan']['reference_sha256'] == hashlib.sha256(
        (tmp_path / 'capture/dock_scan_reference.json').read_bytes()).hexdigest()


def test_repeated_preparation_is_byte_stable_and_preserves_runtime_state(tmp_path):
    pointer, geometry, _, _, _, _ = _write_fixture(tmp_path)
    runtime = tmp_path / 'runtime'
    state = runtime / 'state.json'
    runtime.mkdir()
    state.write_text('{"active_task":"operator-owned"}\n', encoding='utf-8')

    first = prepare_map_session(pointer, runtime, geometry)
    generated = [Path(path) for path in first['assets'].values()]
    first_hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in generated}
    first_home = load_registry(first['assets']['service_destinations'])['home']
    second = prepare_map_session(pointer, runtime, geometry)
    second_hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in generated}

    assert first_hashes == second_hashes
    assert load_registry(second['assets']['service_destinations'])['home'] == first_home
    assert state.read_text(encoding='utf-8') == '{"active_task":"operator-owned"}\n'


@pytest.mark.parametrize(('mutation', 'message'), [
    (lambda pointer, source: (source / 'water_station.yaml').unlink(), 'water_station is missing'),
    (lambda pointer, source: _rewrite_yaml(
        source / 'water_station.yaml', lambda value: value['map'].update(yaml_sha256='0' * 64)),
     'stale map yaml_sha256'),
    (lambda pointer, source: _rewrite_yaml(
        source / 'water_station.yaml', lambda value: value.update(x_m=float('nan'))),
     'x_m must be finite'),
    (lambda pointer, source: _rewrite_yaml(
        source / 'water_station.yaml', lambda value: value.update(id='table_01')),
     'water_station is missing'),
])
def test_rejects_missing_stale_or_invalid_station(tmp_path, mutation, message):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    mutation(pointer, source)
    with pytest.raises(ValueError, match=message):
        prepare_map_session(pointer, tmp_path / 'runtime', geometry)


@pytest.mark.parametrize(('mutation', 'message'), [
    (lambda source: _rewrite_yaml(
        source / 'table_01.yaml', lambda value: value['map'].update(image_sha256='0' * 64)),
     'stale map image_sha256'),
    (lambda source: _rewrite_yaml(
        source / 'table_01.yaml', lambda value: value.update(id='table_02')),
     'ID must match its filename'),
    (lambda source: _rewrite_yaml(
        source / 'table_01.yaml', lambda value: value.update(target_front_gap_m=0.03)),
     'table_01 target front gap must be 0.01 m'),
])
def test_rejects_stale_or_invalid_taught_table(tmp_path, mutation, message):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    mutation(source)
    with pytest.raises(ValueError, match=message):
        prepare_map_session(pointer, tmp_path / 'runtime', geometry)


def test_keeps_water_only_sessions_backward_compatible(tmp_path):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    (source / 'table_01.yaml').unlink()

    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    assert result['station_ids'] == ['water_station']
    registry = load_registry(result['assets']['service_destinations'])
    assert [table['table_id'] for table in registry['tables']] == ['water_station']


def test_ignores_route_files_that_share_the_table_prefix(tmp_path):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    (source / 'table_01_route.yaml').write_text('waypoints: []\n', encoding='utf-8')

    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    assert result['station_ids'] == ['water_station', 'table_01']


def _write_table_02(source):
    table = yaml.safe_load((source / 'table_01.yaml').read_text(encoding='utf-8'))
    table.update(id='table_02', x_m=2.0, y_m=3.0, yaw_rad=0.4)
    (source / 'table_02.yaml').write_text(
        yaml.safe_dump(table, sort_keys=False), encoding='utf-8')


def test_unselected_table_draft_is_not_activated_by_default(tmp_path):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    _write_table_02(source)

    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    assert result['station_ids'] == ['water_station', 'table_01']
    registry = load_registry(result['assets']['service_destinations'])
    assert [table['table_id'] for table in registry['tables']] == [
        'water_station', 'table_01']


def test_pointer_can_select_a_different_explicit_table(tmp_path):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    _write_table_02(source)
    _rewrite_json(pointer, lambda value: value.update(enabled_table_ids=['table_02']))

    result = prepare_map_session(pointer, tmp_path / 'runtime', geometry)

    assert result['station_ids'] == ['water_station', 'table_02']
    registry = load_registry(result['assets']['service_destinations'])
    pose = registry['tables'][1]['service_poses'][0]
    assert (pose['x_m'], pose['y_m'], pose['yaw_rad']) == pytest.approx((2.0, 3.0, 0.4))


@pytest.mark.parametrize(('enabled_table_ids', 'message'), [
    ('table_01', 'must be a list'),
    (['table_01', 'table_01'], 'must not contain duplicates'),
    (['../table_01'], 'must contain table IDs'),
    (['table_02'], 'enabled table record is missing'),
])
def test_rejects_invalid_or_missing_explicit_table_selection(
        tmp_path, enabled_table_ids, message):
    pointer, geometry, _, _, _, _ = _write_fixture(tmp_path)
    _rewrite_json(
        pointer, lambda value: value.update(enabled_table_ids=enabled_table_ids))

    with pytest.raises(ValueError, match=message):
        prepare_map_session(pointer, tmp_path / 'runtime', geometry)


@pytest.mark.parametrize(('mutation', 'message'), [
    (lambda pointer, source: _rewrite_json(
        pointer, lambda value: value.pop('dock_scan_reference')),
     'asset path must be a non-empty string'),
    (lambda pointer, source: _rewrite_json(
        pointer, lambda value: value.update(
            dock_scan_reference=str(source / 'missing_reference.json'))),
     'dock scan reference does not exist'),
    (lambda pointer, source: _rewrite_json(
        source / 'dock_scan_reference.json',
        lambda value: value.update(map_identity=['0' * 64, '1' * 64])),
     'stale map identity'),
    (lambda pointer, source: _rewrite_json(
        source / 'dock_scan_reference.json',
        lambda value: value.update(home_pose=[-.7, .4, .1])),
     'does not match home pose'),
    (lambda pointer, source: (source / 'dock_scan.json').unlink(),
     'dock scan does not exist'),
])
def test_rejects_missing_stale_or_mismatched_dock_scan_reference(
        tmp_path, mutation, message):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    mutation(pointer, source)
    with pytest.raises(ValueError, match=message):
        prepare_map_session(pointer, tmp_path / 'runtime', geometry)


def _rewrite_yaml(path, mutation):
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    mutation(value)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')


def _rewrite_json(path, mutation):
    value = json.loads(path.read_text(encoding='utf-8'))
    mutation(value)
    path.write_text(json.dumps(value), encoding='utf-8')


def test_refuses_output_over_source_capture(tmp_path):
    pointer, geometry, source, _, _, _ = _write_fixture(tmp_path)
    before = {path: path.read_bytes() for path in source.iterdir() if path.is_file()}
    with pytest.raises(ValueError, match='must not overwrite'):
        prepare_map_session(pointer, source, geometry)
    assert {path: path.read_bytes() for path in source.iterdir() if path.is_file()} == before


def test_operator_water_station_tolerance_survives_runtime_generation(tmp_path):
    pointer = _write_fixture(tmp_path)[0]
    source = tmp_path / 'capture/water_station.yaml'
    station = yaml.safe_load(source.read_text())
    station['lateral_tolerance_m'] = .01
    source.write_text(yaml.safe_dump(station))
    result = prepare_map_session(pointer)
    runtime = Path(result['runtime_directory'])
    profiles = json.loads((runtime/'station_profiles.json').read_text())
    registry = load_registry(runtime/'service_destinations.yaml')
    assert profiles['stations']['water_station']['lateral_tolerance_m'] == .01
    assert registry['tables'][0]['service_poses'][0]['lateral_tolerance_m'] == .01
    assert registry['tables'][0]['service_poses'][0]['target_front_gap_m'] == .01
