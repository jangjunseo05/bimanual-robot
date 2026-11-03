"""Mocked flow checks for jdamr_depart.py: no ssh, no robot, temporary state file.

PC-only like the tool itself (it reads the map data at import):
    python3 -m pytest -q scripts/map_20260930_manual/test_jdamr_depart_flow.py
"""
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import pytest
import yaml

HERE = Path(__file__).resolve().parent
# dock_survey and scan_match are the Phase 2 tools the departure tool imports.
sys.path[:0] = [str(HERE), str(HERE.parent / 'phase2_20260930'),
                str(HERE.parents[1] / 'jdamr_cube_navigation')]
# Import with a disposable copy of bundled inputs; CI never needs a host's map
# or live localization state, and every SSH interaction remains mocked below.
_test_data = tempfile.TemporaryDirectory(prefix='jdamr-departure-flow-')
_old_data_root = os.environ.get('JDAMR_DATA_ROOT')
os.environ['JDAMR_DATA_ROOT'] = _test_data.name
shutil.copytree(
    HERE.parents[3] / 'config/navigation/jdamr_runtime',
    Path(_test_data.name) / 'map_20260930_manual/aligned',
)
try:
    import jdamr_depart as d  # noqa: E402
finally:
    if _old_data_root is None:
        os.environ.pop('JDAMR_DATA_ROOT', None)
    else:
        os.environ['JDAMR_DATA_ROOT'] = _old_data_root


def is_submission(command):
    return 'mv ' in command and '.part' in command and '.request' in command


class Pi:
    """Scripted Pi: answers by command substring, records every command."""

    def __init__(self, busy='', linger='yes', session='active', restart_moves=True):
        self.calls, self.busy, self.linger, self.session = [], busy, linger, session
        self.restart_moves = restart_moves
        self.start = {u: 100 for u in d.SENSOR_UNITS}

    def __call__(self, command, timeout=120, check=True, stdin=None):
        self.calls.append(command)
        out = ''
        if 'jdamr-table-cycle-*' in command:
            out = self.busy
        elif 'loginctl show-user' in command:
            out = self.linger
        elif 'is-active jdamr-restaurant-navigation' in command:
            out = self.session
        elif command.startswith('systemctl show'):
            unit = command.split()[-1]
            out = f'ActiveState=active\nActiveEnterTimestampMonotonic={self.start[unit]}\n'
        elif command.startswith('sudo -n systemctl restart'):
            unit = command.split()[-1]
            if self.restart_moves:
                self.start[unit] += 1
        return SimpleNamespace(returncode=0, stdout=out)


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setattr(d, 'STATE', tmp_path / 'state.json')
    monkeypatch.setattr(d, 'P2', tmp_path)
    monkeypatch.setattr(d, 'CLICK', tmp_path / 'click.json')
    monkeypatch.setattr(d.time, 'sleep', lambda _s: None)
    monkeypatch.setattr(d, 'sh', lambda *a, **k: SimpleNamespace(returncode=0, stdout='active'))
    regions = {k: {'xy': [0.0, 0.0], 'radius_m': 0.4} for k in ('water_station', 'table_02')}
    d.save_state({'regions': regions, 'pose': [0, 0, 0], 'localized': True,
                  'init_local_only': False})
    return tmp_path


def go_args(**extra):
    base = dict(table_id='table_02', route=None, region=None, via_route=None, skip_via=False,
                resume_at_observation=False, resume_parked_log=None, dock_only=False, graceful_final=False,
                rpp_transit=False, record_diagnostics=False)
    base.update(extra)
    return SimpleNamespace(**base)


def _map_session_fixture(tmp_path):
    data_root = tmp_path / 'data'
    session = data_root / 'maps' / 'manual-session'
    runtime = session / 'runtime'
    runtime.mkdir(parents=True)
    map_image = runtime / 'map.pgm'
    map_image.write_bytes(b'P5\n2 2\n255\n\x00\xfe\xcd\x00')
    map_yaml = runtime / 'map.yaml'
    map_yaml.write_text(yaml.safe_dump({
        'image': map_image.name, 'mode': 'trinary', 'resolution': .05,
        'origin': [-1., -2., 0.], 'negate': 0,
        'occupied_thresh': .65, 'free_thresh': .196,
    }, sort_keys=False))
    keepout = runtime / 'keepout.pgm'
    keepout.write_bytes(b'P5\n2 2\n255\n\xfe\x00\x00\xfe')
    (runtime / 'keepout.yaml').write_text(yaml.safe_dump({
        'image': keepout.name, 'mode': 'trinary', 'resolution': .05,
        'origin': [-1., -2., 0.], 'negate': 0,
        'occupied_thresh': .65, 'free_thresh': .196,
    }, sort_keys=False))
    destinations = {
        'destinations': {
            'water_station': {
                'face_xy': [1.2, -.225], 'normal_deg': 270.,
                'source': 'fixture derived search face',
            },
        },
    }
    (runtime / 'destinations.json').write_text(json.dumps(destinations))
    registry = {
        'home': {'x_m': -.8, 'y_m': .4, 'yaw_rad': .1,
                 'parking_direction': 'reverse'},
        'tables': [{
            'table_id': 'water_station',
            'service_poses': [{
                'x_m': 1.2, 'y_m': -.3, 'yaw_rad': math.pi / 2,
                'approach_offset_m': .5,
            }],
        }],
    }
    (runtime / 'service_destinations.yaml').write_text(
        yaml.safe_dump(registry, sort_keys=False))
    profiles = runtime / 'station_profiles.json'
    profiles.write_text(json.dumps({
        'stations': {'water_station': {'target_priority': 'operator_taught'}},
    }))
    map_identity = [d.sha256(map_yaml), d.sha256(map_image)]
    scan = session / 'dock_scan.json'
    scan.write_text('{"scans": []}\n')
    reference = session / 'dock_scan_reference.json'
    reference.write_text(json.dumps({
        'map_identity': map_identity,
        'home_pose': [-.8, .4, .1], 'pose_map': [-.8, .4, .1],
        'scan_path': str(scan),
    }))
    profile_document = json.loads(profiles.read_text())
    home_record = session / 'dock.json'
    station_record = session / 'water_station.json'
    home_record.write_text('{}\n')
    station_record.write_text('{}\n')
    profile_document['home'] = {
        'source_record': str(home_record),
        'dock_scan': {'reference_path': str(reference), 'scan_path': str(scan)}}
    profile_document['stations']['water_station']['source_record'] = str(station_record)
    profiles.write_text(json.dumps(profile_document))
    pointer = data_root / 'maps/current_manual_mapping.json'
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(json.dumps({
        'directory': str(session), 'map_yaml': str(map_yaml),
        'dock_scan_reference': str(reference),
    }))
    (runtime / 'state.json').write_text(json.dumps({
        'operator_owned': 'preserve',
        'localized': True,
        'regions': {'water_station': {'xy': [1.2, -.3], 'radius_m': .4}},
    }, indent=1))
    profile = {
        'session_directory': str(session),
        'runtime_directory': str(runtime),
        'station_ids': ['water_station'],
        'map_hashes': {'source': {'yaml_path': str(map_yaml)}, 'runtime': {
            'yaml_sha256': map_identity[0], 'image_sha256': map_identity[1],
        }},
    }
    return SimpleNamespace(
        data_root=data_root, session=session, runtime=runtime,
        pointer=pointer, profile=profile, map_identity=map_identity,
        station_profiles=profiles,
    )


def _configure_fixture_session(tmp_path, monkeypatch):
    fixture = _map_session_fixture(tmp_path)
    for name in (
            'ACTIVE_MAP_SESSION', 'BASE', 'P2', 'MAP_YAML', 'KEEPOUT_YAML',
            'REGISTRY', 'ANNOTATION', 'CLICK', 'STATE', 'PI_BASE', 'PI_P2',
            'PI_REGISTRY', 'ACTIVE_MAP_POINTER', 'PI_TOOLS', 'CAMERA_MOUNT_CONFIG',
            'PI_MOUNT', 'DESTINATIONS', 'TABLES',
            'STATION_PROFILES'):
        monkeypatch.setattr(d, name, getattr(d, name))
    monkeypatch.setattr(d, 'DATA_ROOT', fixture.data_root)
    monkeypatch.setattr(d, 'prepare_reference_mount', lambda: False)
    fake_assets = SimpleNamespace(
        prepare_map_session=lambda pointer: fixture.profile)
    monkeypatch.setitem(
        sys.modules, 'jdamr_cube_navigation.map_session_assets', fake_assets)
    assert d.configure_map_session(fixture.pointer) == fixture.profile
    return fixture


def _add_taught_table_01(fixture):
    registry_path = fixture.runtime / 'service_destinations.yaml'
    registry = yaml.safe_load(registry_path.read_text())
    registry['tables'].append({
        'table_id': 'table_01',
        'service_poses': [{
            'x_m': -.735, 'y_m': -1.607, 'yaw_rad': -.0384,
            'approach_offset_m': .5,
        }],
    })
    registry_path.write_text(yaml.safe_dump(registry, sort_keys=False))
    destinations_path = fixture.runtime / 'destinations.json'
    destinations = json.loads(destinations_path.read_text())
    destinations['destinations']['table_01'] = {
        'face_xy': [-.660, -1.610], 'normal_deg': 177.8,
        'source': 'fixture taught table pose',
    }
    destinations_path.write_text(json.dumps(destinations))
    fixture.profile['station_ids'] = ['water_station', 'table_01']


def test_prepare_reference_mount_copies_selected_mount_independent_of_teaching_reference(
        tmp_path, monkeypatch):
    monkeypatch.setattr(d, 'P2', tmp_path)
    station_profiles = tmp_path / 'station_profiles.json'
    monkeypatch.setattr(d, 'STATION_PROFILES', station_profiles)
    canonical = yaml.safe_load((HERE.parents[1] / 'jdamr_cube_navigation/config' /
                                'camera_mount_current.yaml').read_text())

    generated = []
    for pitch in (0.1, 1.4):
        station_profiles.write_text(json.dumps({
            'stations': {'water_station': {'rgbd_reference': {
                'camera_position_m': [-0.1, 0.2, 0.8],
                'camera_yaw_rad': pitch,
                'reference_capture_path': f'/changed/reference/{pitch}.json',
            }}},
        }))
        assert d.prepare_reference_mount()
        generated.append(yaml.safe_load((tmp_path / 'camera_mount.yaml').read_text()))

    assert generated == [canonical, canonical]
    mount = generated[0]['camera_mount']
    assert mount['provenance']['mount_calibration_complete'] is False
    assert mount['provenance']['millimeter_accuracy_validated'] is False
    assert mount['table_edge_calibration'] == canonical['camera_mount']['table_edge_calibration']


def test_prepare_reference_mount_preserves_future_measured_validation_state(
        tmp_path, monkeypatch):
    tools = tmp_path / 'scripts' / 'map_tools'
    tools.mkdir(parents=True)
    canonical_path = (tools.parents[1] / 'jdamr_cube_navigation/config' /
                      'camera_mount_current.yaml')
    canonical_path.parent.mkdir(parents=True)
    measured = {
        'camera_mount': {
            'status': 'measured',
            'parent_frame': 'base_link',
            'child_frame': 'camera_link',
            'transform': {
                'x_m': -0.275, 'y_m': 0.0, 'z_m': 1.26,
                'roll_rad': 0.01, 'pitch_rad': 0.84, 'yaw_rad': 0.0,
            },
            'table_edge_calibration': {'enabled': True, 'status': 'measured'},
            'provenance': {
                'mount_calibration_complete': True,
                'millimeter_accuracy_validated': True,
            },
        },
    }
    canonical_path.write_text(yaml.safe_dump(measured, sort_keys=False))
    monkeypatch.setattr(d, 'TOOLS', tools)
    monkeypatch.setattr(d, 'P2', tmp_path / 'runtime')
    d.P2.mkdir()

    assert d.prepare_reference_mount()
    assert yaml.safe_load((d.P2 / 'camera_mount.yaml').read_text()) == measured


@pytest.mark.parametrize('mode,has_straight_departure', [
    ('straight', True), ('nav2_waypoint', False),
])
def test_taught_map_route_uses_its_configured_departure_mode(
        tmp_path, monkeypatch, mode, has_straight_departure):
    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    fixture.profile['dock_departure_mode'] = mode
    d.write_routes((-2., .5, 0.))
    generated = yaml.safe_load((fixture.runtime / 'water_station_route.yaml').read_text())
    assert generated['waypoints'][0]['id'] == 'home_exit'
    assert ('dock_departure_distance_m' in generated) is has_straight_departure


def test_go_refuses_without_a_completed_init(env, monkeypatch):
    state = d.load_state()
    state['localized'] = False
    d.save_state(state)
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert pi.calls == []


def test_go_during_a_cycle_stops_nothing(env, monkeypatch):
    pi = Pi(busy='jdamr-table-cycle-x.service loaded active running')
    monkeypatch.setattr(d, 'pi', pi)
    runs = []
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or [])
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert runs == []
    assert not any('stop' in c or 'restart' in c for c in pi.calls)


@pytest.mark.parametrize('action', ['recover', 'session-stop'])
def test_recover_and_session_stop_refuse_during_a_cycle(env, monkeypatch, action):
    pi = Pi(busy='jdamr-table-cycle-x.service loaded active running')
    monkeypatch.setattr(d, 'pi', pi)
    handler = d.cmd_recover if action == 'recover' else d.cmd_session_stop
    with pytest.raises(SystemExit):
        handler(SimpleNamespace(seed=None, local_only=False))
    assert not any('restart' in c or 'session.sh stop' in c for c in pi.calls)
    assert d.load_state()['localized'] is True


MAP_FAIL = [{'event': 'failed', 'reason': 'live map data unavailable: map'}]


def test_map_wait_failure_restarts_only_executor_once_and_keeps_localization(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs, recovered = [], []
    results = iter([MAP_FAIL, [{'event': 'home_arrived', 'confirmation': {'confirmed': True}}]])
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or next(results))
    before = d.load_state()
    monkeypatch.setattr(d, 'restart_executor', lambda: recovered.append('executor'))
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('full recovery during go'))
    d.cmd_go(go_args())
    assert len(runs) == 2 and recovered == ['executor']
    assert d.load_state() == before


def test_later_failures_are_not_retried(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs = []
    moved = [{'event': 'observation_route_selected'},
             {'event': 'failed', 'reason': 'live map data unavailable: map'}]
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or moved)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    # A cycle that did not end docked exits non-zero, without a retry.
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert len(runs) == 1


def test_recover_local_only_needs_a_seed(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_recover(SimpleNamespace(seed=None, local_only=True))
    assert pi.calls == []


@pytest.mark.parametrize('extra', [{'skip_via': True}, {'resume_at_observation': True},
                                   {'resume_parked_log': '/x y/log.jsonl'}, 'local_init'])
def test_map_retry_from_current_pose_never_reinitializes_or_repeats_forever(env, monkeypatch, extra):
    if extra == 'local_init':
        state = d.load_state()
        state['init_local_only'] = True
        d.save_state(state)
        extra = {}
    monkeypatch.setattr(d, 'pi', Pi())
    attempts, restarts = [], []
    before = d.load_state()
    monkeypatch.setattr(d, 'run_cycle', lambda *a: attempts.append(a) or MAP_FAIL)
    monkeypatch.setattr(d, 'restart_executor', lambda: restarts.append(True))
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(**extra))
    assert len(attempts) == 2 and restarts == [True]
    assert d.load_state() == before


def test_recover_requires_linger(env, monkeypatch):
    pi = Pi(linger='no')
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_recover(SimpleNamespace(seed=None, local_only=False))
    assert not any('restart' in c for c in pi.calls)


def test_restart_must_show_a_new_start_time(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi(restart_moves=False))
    with pytest.raises(SystemExit):
        d.restart_sensor_services()


def _inactive(pi, unit):
    real = pi.__call__

    def answer(command, **kw):
        result = real(command, **kw)
        if command.startswith('systemctl show') and command.endswith(unit):
            result.stdout = 'ActiveState=inactive\nActiveEnterTimestampMonotonic=0\n'
        return result
    return answer


def test_restart_refuses_an_inactive_base_but_skips_the_on_demand_camera(env, monkeypatch):
    """2026-10-06: the executor starts camera and observer for the box stops only."""
    pi = Pi()
    monkeypatch.setattr(d, 'pi', _inactive(pi, 'jdamr-base.service'))
    with pytest.raises(SystemExit):
        d.restart_sensor_services()
    pi = Pi()
    monkeypatch.setattr(d, 'pi', _inactive(pi, 'jdamr-box-observer.service'))
    d.restart_sensor_services()
    restarted = [c.split()[-1] for c in pi.calls if c.startswith('sudo -n systemctl restart')]
    assert 'jdamr-box-observer.service' not in restarted
    assert 'jdamr-base.service' in restarted


def test_a_session_start_leaves_the_camera_to_the_executor(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', lambda command, **kw: pi(command, **kw) if not command.startswith(
        'systemctl is-active') else SimpleNamespace(returncode=3, stdout='inactive'))
    d.start_sensor_services()
    started = [c for c in pi.calls if c.startswith('sudo -n systemctl start')]
    assert started == ['sudo -n systemctl start jdamr-base.service']


def test_restart_covers_all_sensor_services(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    d.restart_sensor_services()
    restarted = [c.split()[-1] for c in pi.calls if c.startswith('sudo -n systemctl restart')]
    assert restarted == list(d.SENSOR_UNITS)


def test_recover_seed_does_not_imply_local_only(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    for name in ('cmd_display_stop', 'cmd_session_stop', 'restart_sensor_services',
                 'cmd_display_start', 'cmd_session_start'):
        monkeypatch.setattr(d, name, lambda *a: None)
    seen = []
    monkeypatch.setattr(d, 'cmd_init', lambda a: seen.append(
        (a.keep_home, a.seed_from_state, a.local_only, a.click)))
    monkeypatch.setattr(d, 'dds_health', lambda: ('ok', 'PROBE health ok {} 1.0 s'))
    d.cmd_recover(SimpleNamespace(seed=[1.0, 2.0, 90.0], local_only=False))
    assert seen == [(True, False, False, True)]
    assert json.loads(d.CLICK.read_text())['x_m'] == 1.0


def test_init_seeds_from_the_registered_dock_and_keeps_it(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'pull_registry', lambda: {'home': {
        'x_m': -0.204, 'y_m': 0.184, 'yaw_rad': 0.0}})
    monkeypatch.setattr(d, 'capture_scan', lambda _path: None)
    seen = []

    def refine(_scan, click):
        seen.append(click)
        raise SystemExit('stop after seeding')
    monkeypatch.setattr(d, 'refine_pose', refine)
    args = SimpleNamespace(seed_from_state=False, click=False, local_only=False, keep_home=False)
    with pytest.raises(SystemExit):
        d.cmd_init(args)
    assert (seen[0]['x_m'], seen[0]['y_m'], seen[0]['source']) == (-0.204, 0.184, 'registered_dock')
    assert args.keep_home is True
    assert not d.CLICK.exists()


def test_failed_init_leaves_go_refused(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi(session='inactive'))
    with pytest.raises(SystemExit):
        d.cmd_init(SimpleNamespace(seed_from_state=True, local_only=False, keep_home=True))
    assert d.load_state()['localized'] is False


@pytest.mark.parametrize('source', ['anchored_dock_scan', 'registered_dock_local_map'])
def test_registered_dock_init_refuses_previous_state_seed_without_revoking_localization(
        env, monkeypatch, source):
    state = d.load_state()
    state['init_pose_source'] = source
    d.save_state(state)
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)

    with pytest.raises(SystemExit):
        d.cmd_init(SimpleNamespace(seed_from_state=True, local_only=False, keep_home=True,
                                   min_inlier=None))

    assert d.load_state()['localized'] is True
    assert pi.calls == []


@pytest.mark.parametrize('stdout,expected', [
    ("PROBE health ok {'map': 1} 1.2 s\n", 'ok'),
    ("PROBE health MISSING {'map': 0} 30.0 s\n", 'missing'),
    ('ssh: connect to host: No route to host\n', 'error'),
])
def test_probe_result_is_classified(env, monkeypatch, stdout, expected):
    monkeypatch.setattr(d, 'pi', lambda *a, **k: SimpleNamespace(returncode=0, stdout=stdout))
    assert d.dds_health()[0] == expected


def test_probe_timeout_is_an_error(env, monkeypatch):
    def slow(*_a, **_k):
        raise subprocess.TimeoutExpired('ssh', 60)
    monkeypatch.setattr(d, 'pi', slow)
    assert d.dds_health()[0] == 'error'


def test_session_restart_revokes_localization(env, monkeypatch):
    monkeypatch.setattr(d, 'start_executor', lambda: True)
    monkeypatch.setattr(d, 'pi', lambda c, **k: SimpleNamespace(
        returncode=0, stdout='' if 'jdamr-table-cycle-*' in c else 'STARTED'))
    d.cmd_session_start(None)
    assert d.load_state()['localized'] is False


def test_session_reuse_keeps_localization(env, monkeypatch):
    monkeypatch.setattr(d, 'start_executor', lambda: True)
    monkeypatch.setattr(d, 'pi', lambda c, **k: SimpleNamespace(
        returncode=0, stdout='' if 'jdamr-table-cycle-*' in c else 'REUSED'))
    d.cmd_session_start(None)
    assert d.load_state()['localized'] is True


def test_display_start_starts_only_missing_units(env, monkeypatch):
    started = []

    def sh(command, **_k):
        if command.startswith('systemd-run'):
            started.append(command.split('--unit=')[1].split()[0])
        if 'is-active' in command:
            unit = command.split()[-1]
            active = unit != 'jdamr-p2-rviz' or 'jdamr-p2-rviz' in started
            return SimpleNamespace(returncode=0, stdout='active' if active else 'inactive')
        return SimpleNamespace(returncode=0, stdout='')
    monkeypatch.setattr(d, 'sh', sh)
    d.cmd_display_start(None)
    assert started == ['jdamr-p2-rviz']


def test_taught_map_display_passes_saved_station_directory(env, monkeypatch):
    commands = []
    started = set()
    monkeypatch.setattr(d, 'ACTIVE_MAP_SESSION', {'session_directory': str(env)})
    monkeypatch.setattr(d, 'BASE', env / 'map session')

    def sh(command, **_kwargs):
        commands.append(command)
        if command.startswith('systemd-run'):
            started.add(command.split('--unit=')[1].split()[0])
        if command.startswith('systemctl --user is-active'):
            unit = command.split()[-1]
            active = unit in started
            return SimpleNamespace(
                returncode=0 if active else 3,
                stdout='active' if active else 'inactive')
        return SimpleNamespace(returncode=0, stdout='active')

    monkeypatch.setattr(d, 'sh', sh)
    d.cmd_display_start(None)
    marker = next(command for command in commands if '--unit=jdamr-p2-markers ' in command)
    assert '--station-directory' in marker
    assert str(env / 'map session') in marker


def test_stop_during_the_map_wait_prevents_recovery(env, monkeypatch):
    """A stop makes the executor fail with the same map message; do not re-depart."""
    monkeypatch.setattr(d, 'pi', Pi())

    def stopped_run(*_a):
        d.cmd_stop(None)
        return MAP_FAIL
    monkeypatch.setattr(d, 'run_cycle', stopped_run)
    monkeypatch.setattr(d, 'cmd_recover', lambda a: pytest.fail('recovered'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())


def test_stop_during_recovery_prevents_the_second_departure(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    runs = []
    monkeypatch.setattr(d, 'run_cycle', lambda *a: runs.append(a) or MAP_FAIL)
    monkeypatch.setattr(d, 'restart_executor', lambda: d.cmd_stop(None))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert len(runs) == 1


class ExecutorPi(Pi):
    """The resident executor is up; requests finish at once."""

    def __init__(self, result='{"code": 0}'):
        super().__init__()
        self.stdins = []
        self.result = result

    def __call__(self, command, timeout=120, check=True, stdin=None):
        self.stdins.append(stdin)
        if command == f'systemctl is-active {d.EXECUTOR_UNIT}':
            self.calls.append(command)
            return SimpleNamespace(returncode=0, stdout='active')
        if 'if test -e ' in command and '.result' in command:
            self.calls.append(command)
            return SimpleNamespace(returncode=0, stdout=self.result)
        return super().__call__(command, timeout, check, stdin)


def test_go_hands_the_run_to_the_resident_executor(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    events = d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert events == [{'event': 'home_arrived'}]
    request = next(s for s in pi.stdins if s)
    argv = json.loads(request)['argv']
    assert argv[argv.index('--table-id') + 1] == 'table_02'
    assert not any('systemd-run --unit=jdamr-table-cycle' in c for c in pi.calls)
    assert not any(name in command for command in pi.calls for name in (
        'jdamr-onboard-bag-', 'jdamr-udp-drops-', 'jdamr-load-profile-'))
    state = d.load_state()
    assert state['last_unit'].startswith('executor:')
    assert 'onboard_bag_unit' not in state and 'monitor_units' not in state
    d.cmd_stop(SimpleNamespace())
    assert any(f'sudo -n systemctl kill --signal=SIGINT {d.EXECUTOR_UNIT}' in c
               for c in pi.calls)


def test_default_run_interruption_does_not_stop_or_copy_unstarted_diagnostics(
        env, monkeypatch):
    pi = ExecutorPi()
    copies = []
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'sh', lambda command, **_kwargs: copies.append(command) or
                        SimpleNamespace(returncode=0, stdout=''))

    def interrupted(_path):
        raise KeyboardInterrupt
    monkeypatch.setattr(d, 'read_events', interrupted)

    with pytest.raises(KeyboardInterrupt):
        d.run_cycle(go_args(), d.load_state(), 'table_02')

    assert not any(name in command for command in pi.calls for name in (
        'jdamr-onboard-bag-', 'jdamr-udp-drops-', 'jdamr-load-profile-',
        'ros2 bag reindex'))
    assert not any(command.startswith('rsync ') for command in copies)
    state = d.load_state()
    assert 'onboard_bag_unit' not in state and 'monitor_units' not in state


def test_onboard_bag_brackets_the_run_and_is_copied(env, monkeypatch):
    pi = ExecutorPi()
    copies = []
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'sh', lambda command, **k: copies.append(command) or
                        SimpleNamespace(returncode=0, stdout=''))
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(record_diagnostics=True), d.load_state(), 'table_02')
    start = next(i for i, c in enumerate(pi.calls) if 'systemd-run --unit=jdamr-onboard-bag-' in c)
    request = next(i for i, c in enumerate(pi.calls) if is_submission(c))
    assert start < request
    assert all(topic in pi.calls[start] for topic in ('/odom', '/imu/data_raw', '/scan'))
    assert 'RuntimeMaxSec=' in pi.calls[start] and 'MemoryMax=' in pi.calls[start]
    unit = pi.calls[start].split('--unit=')[1].split()[0]
    stop = pi.calls.index(f'sudo -n systemctl stop {unit}')
    assert stop > request
    assert any(c.startswith(f'rsync -a --partial {d.HOST}:') for c in copies)
    assert 'onboard_bag_unit' not in d.load_state()
    d.cmd_stop(SimpleNamespace())
    assert pi.calls.count(f'sudo -n systemctl stop {unit}') == 1


def test_onboard_bag_is_closed_when_the_run_is_interrupted(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)

    def interrupted(_path):
        raise KeyboardInterrupt
    monkeypatch.setattr(d, 'read_events', interrupted)
    with pytest.raises(KeyboardInterrupt):
        d.run_cycle(go_args(record_diagnostics=True), d.load_state(), 'table_02')
    assert any(c.startswith('sudo -n systemctl stop jdamr-onboard-bag-') for c in pi.calls)
    assert 'onboard_bag_unit' not in d.load_state()


HOME_EVENTS = [{'event': 'box_approach_finished'}, {'event': 'parked_dwell_complete'},
               {'event': 'box_escape_finished'},
               {'event': 'home_arrived', 'confirmation': {'confirmed': True}}]


def test_sequence_runs_each_table_from_the_dock_with_an_init_between(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    order = []
    monkeypatch.setattr(d, 'run_cycle', lambda args, state, table: order.append(table) or HOME_EVENTS)
    monkeypatch.setattr(d, 'cmd_init', lambda init_args: order.append(
        ('init', init_args.keep_home, init_args.click, init_args.seed_from_state)))
    d.cmd_go(go_args(table_ids=['table_01', 'table_02'], dock_wait_s=0.0))
    assert order == ['table_01', ('init', True, False, False), 'table_02']


def test_sequence_stops_at_the_first_cycle_that_does_not_dock(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    order = []
    failed = [{'event': 'failed'}]
    monkeypatch.setattr(d, 'run_cycle', lambda args, state, table: order.append(table) or failed)
    monkeypatch.setattr(d, 'cmd_init', lambda init_args: order.append('init'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(table_ids=['table_02', 'table_01'], dock_wait_s=0.0))
    assert order == ['table_02']


@pytest.mark.parametrize('extra', [{'skip_via': True}, {'resume_at_observation': True},
                                   {'dock_only': True}, {'resume_parked_log': '/log'}])
def test_sequence_refuses_single_cycle_options(env, monkeypatch, extra):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'run_cycle', lambda *a: pytest.fail('must not depart'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(table_ids=['table_01', 'table_02'], **extra))


def test_poll_pi_tells_a_failed_link_from_a_finished_cycle(monkeypatch):
    """ssh exit 255 or a hung ssh is 'unknown', never 'the cycle ended'."""
    import subprocess
    answers = iter([SimpleNamespace(returncode=255, stdout=''),
                    subprocess.TimeoutExpired('ssh', 30),
                    SimpleNamespace(returncode=0, stdout='running\n'),
                    SimpleNamespace(returncode=0, stdout='')])

    def pi(*_args, **_kwargs):
        answer = next(answers)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    monkeypatch.setattr(d, 'pi', pi)
    assert d.poll_pi('test -e result || echo running') is None
    assert d.poll_pi('test -e result || echo running') is None
    assert d.poll_pi('test -e result || echo running').stdout.strip() == 'running'
    assert d.poll_pi('test -e result || echo running').stdout.strip() == ''


def test_historical_operator_call_never_sends_notifications(env, monkeypatch, capsys):
    sent, messages = [], []
    monkeypatch.setattr(d, 'sh', lambda command, **kwargs: sent.append(command)
                        or SimpleNamespace(returncode=0, stdout=''))
    monkeypatch.setattr(d, 'log', messages.append)
    monkeypatch.setenv('JDAMR_OPERATOR_NOTIFY_CMD', 'removed-notification-hook')
    state = d.load_state()
    state['operator_alerts'] = True
    d.save_state(state)
    d.show_event({'event': 'operator_call', 'level': 'FATAL', 'reason': 'old run'})
    assert sent == []
    assert messages == []
    assert '\a' not in capsys.readouterr().out
    d.show_event({'event': 'failed', 'reason': 'scan stale'})
    assert len(messages) == 1
    assert json.loads(messages[0])['reason'] == 'scan stale'
    assert sent == []


def test_go_passes_the_rpp_transit_switch_only_when_asked(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'rpp_transit': True})):
        # One run directory per second; keep the two runs apart.
        monkeypatch.setattr(d.time, 'strftime', lambda _fmt, s=second: f'20261002_11000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert '--rpp-transit' not in seen[0]
    assert '--rpp-transit' in seen[1]


def test_go_passes_the_pointcloud_switch_only_when_asked(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'pointcloud_perception': True})):
        monkeypatch.setattr(d.time, 'strftime', lambda _fmt, s=second: f'20261002_11000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert '--pointcloud-perception' not in seen[0]
    assert '--pointcloud-perception' in seen[1]


@pytest.mark.parametrize('extra', [
    ['--surface-mode', 'front'], ['--dock-only'], ['--rear-box-only'],
])
def test_pointcloud_cli_rejects_tasks_that_do_not_use_table_observation(env, monkeypatch, extra):
    monkeypatch.setattr(d.sys, 'argv', ['jdamr_depart.py', '--legacy-map', 'go', 'table_01',
                                      '--pointcloud-perception', *extra])
    monkeypatch.setattr(d, 'cmd_go', lambda _args: pytest.fail('invalid task was dispatched'))
    with pytest.raises(SystemExit) as error:
        d.main()
    assert error.value.code == 2


def test_go_passes_the_transit_variant(env, monkeypatch):
    monkeypatch.setattr(d.time, 'strftime', lambda _fmt: '20261006_110000')
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(transit='navfn-mppi'), d.load_state(), 'table_02')
    argv = json.loads(next(s for s in pi.stdins if s))['argv']
    assert argv[argv.index('--transit') + 1] == 'navfn-mppi'


def test_rear_box_request_has_no_transit_via_or_home_task(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [
        {'event': 'rear_box_parking_finished', 'confirmed': True}])
    d.run_cycle(go_args(rear_box_only=True, final_gap_m=.01), d.load_state(), 'table_02')
    argv = json.loads(next(s for s in pi.stdins if s))['argv']
    assert '--rear-box-only' in argv
    assert '--return-home' not in argv
    assert '--via-id' not in argv
    assert '--search' not in argv
    assert argv[argv.index('--final-gap-m') + 1] == '0.01'


def test_rear_box_success_does_not_require_map_home_arrival(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'run_cycle', lambda *a: [
        {'event': 'rear_box_parking_finished', 'confirmed': True}])
    d.cmd_go(go_args(rear_box_only=True))


def test_go_passes_the_zero_turn_final_switch_only_when_asked(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'zero_turn_final': True})):
        monkeypatch.setattr(d.time, 'strftime', lambda _fmt, s=second: f'20261005_10000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert '--zero-turn-final' not in seen[0]
    assert '--zero-turn-final' in seen[1]


def test_go_passes_the_final_gap_and_dwell_only_when_asked(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'final_gap_m': 0.02, 'dwell_s': 30.0},
                                    {'dwell_s': 0.0})):
        monkeypatch.setattr(d.time, 'strftime', lambda _fmt, s=second: f'20261006_10000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert '--final-gap-m' not in seen[0] and '--dwell-s' not in seen[0]
    argv = seen[1]
    assert argv[argv.index('--final-gap-m') + 1] == '0.02'
    assert argv[argv.index('--dwell-s') + 1] == '30.0'
    assert seen[2][seen[2].index('--dwell-s') + 1] == '0.0'


def test_go_spools_table_edge_by_default_and_front_only_when_requested(env, monkeypatch):
    seen = []
    for second, extra in enumerate(({}, {'surface_mode': 'front'})):
        monkeypatch.setattr(d.time, 'strftime',
                            lambda _fmt, s=second: f'20261013_10000{s}')
        pi = ExecutorPi()
        monkeypatch.setattr(d, 'pi', pi)
        monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
        d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
        seen.append(json.loads(next(s for s in pi.stdins if s))['argv'])
    assert seen[0][seen[0].index('--surface-mode') + 1] == 'table_edge'
    assert '--final-gap-m' not in seen[0]
    assert seen[0][seen[0].index('--via-id') + 1] == d.VIA_ID
    assert seen[1][seen[1].index('--surface-mode') + 1] == 'front'
    assert seen[1][seen[1].index('--via-id') + 1] == d.VIA_ID


@pytest.mark.parametrize('with_via', [False, True])
def test_table_edge_via_is_spooled_by_default_and_when_requested(
        env, monkeypatch, with_via):
    monkeypatch.setattr(d.time, 'strftime', lambda _fmt: '20261013_100010')
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(surface_mode='table_edge', with_via=with_via),
                d.load_state(), 'table_02')
    argv = json.loads(next(s for s in pi.stdins if s))['argv']
    assert argv[argv.index('--via-id') + 1] == d.VIA_ID
    assert argv[argv.index('--surface-mode') + 1] == 'table_edge'


@pytest.mark.parametrize('extra,route_name', [
    ({}, 'table_02_route.yaml'),
    ({'surface_mode': 'table_edge', 'with_via': True}, 'table_02_route.yaml'),
    ({'surface_mode': 'front'}, 'table_02_route.yaml'),
    ({'surface_mode': 'front', 'skip_via': True}, 'table_02_direct_route.yaml'),
    ({'surface_mode': 'table_edge', 'skip_via': True}, 'table_02_direct_route.yaml'),
    ({'route': '/operator/current_table_route.yaml'}, '/operator/current_table_route.yaml'),
])
def test_spooled_route_start_matches_the_selected_via_mode(
        env, monkeypatch, extra, route_name):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(**extra), d.load_state(), 'table_02')
    argv = json.loads(next(s for s in pi.stdins if s))['argv']
    expected = Path(route_name) if Path(route_name).is_absolute() else env / route_name
    assert argv[argv.index('--approach-route') + 1] == str(expected)


def test_generated_table_routes_keep_distinct_dock_and_via_starts(env, monkeypatch):
    mask = env / 'keepout.pgm'
    mask.write_bytes(b'P5\n2 2\n255\n\x00\xff\xff\x00')
    mask_yaml = env / 'keepout.yaml'
    mask_yaml.write_text(yaml.safe_dump({'image': mask.name}))
    monkeypatch.setattr(d, 'KEEPOUT_YAML', mask_yaml)
    monkeypatch.setattr(d, 'MAP_YAML', env / 'map.yaml')
    home = (0.1, 0.2, -0.3)
    d.write_routes(home)
    water_x, water_y, _ = d.along_normal(d.VIA_ID, d.ESCAPE_M)
    for table_id in d.TABLES:
        via = yaml.safe_load((env / f'{table_id}_route.yaml').read_text())
        dock = yaml.safe_load((env / f'{table_id}_direct_route.yaml').read_text())
        assert via['start_pose'] == {'x': round(water_x, 3), 'y': round(water_y, 3)}
        assert dock['start_pose'] == {'x': home[0], 'y': home[1]}
        assert dock['waypoints'][0]['id'] == 'home_exit'
        assert dock['waypoints'][1:] == via['waypoints']
        assert len({wp['id'] for wp in dock['waypoints']}) == len(dock['waypoints'])
        for field in ('frame_id', 'map_yaml', 'keepout_mask_yaml',
                      'expected_mask_sha256', 'max_route_start_distance_m'):
            assert dock[field] == via[field]


def test_configured_map_session_is_idempotent_and_generates_only_taught_routes(
        tmp_path, monkeypatch):
    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    first = d.load_state()

    assert d.TABLES == ('water_station',)
    assert d.STATION_PROFILES == fixture.station_profiles
    assert d.dock_map_identity() == fixture.map_identity
    assert first['operator_owned'] == 'preserve'
    assert first['localized'] is False
    assert first['map_session_identity'] == fixture.map_identity
    assert first['dock_scan_reference']['home_pose'] == [-.8, .4, .1]

    first['localized'] = True
    d.save_state(first)
    stable_state = d.STATE.read_bytes()
    d.configure_map_session(fixture.pointer)
    assert d.STATE.read_bytes() == stable_state

    d.write_routes((-.8, .4, .1))
    routes = sorted(path.name for path in fixture.runtime.glob('*_route.yaml'))
    assert routes == ['water_station_direct_route.yaml', 'water_station_route.yaml']
    route = yaml.safe_load(
        (fixture.runtime / 'water_station_direct_route.yaml').read_text())
    assert route['station_profiles'] == str(fixture.station_profiles)
    assert route['map_yaml'] == str(fixture.runtime / 'map.yaml')
    assert route['expected_mask_sha256'] == d.sha256(
        fixture.runtime / 'keepout.pgm')
    assert [point['id'] for point in route['waypoints']] == [
        'home_exit', 'water_station_pre', 'water_station_observation']
    assert len({point['id'] for point in route['waypoints']}) == 3


def test_taught_session_restores_registered_dock_anchor_after_derived_scan(tmp_path, monkeypatch):
    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    state = d.load_state()
    original = state['dock_scan_reference']
    state['dock_scan_reference'] = {
        **original, 'scan_path': str(fixture.runtime / 'later_local_scan.json'),
        'pose_map': [-.79, .35, .12],
    }
    state['localized'] = True
    d.save_state(state)

    d.configure_map_session(fixture.pointer)

    restored = d.load_state()
    assert restored['dock_scan_reference'] == original
    assert restored['localized'] is False
    assert restored['operator_owned'] == 'preserve'


def test_taught_session_map_fallback_keeps_original_dock_anchor(tmp_path, monkeypatch):
    import numpy as np

    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    original = d.load_state()['dock_scan_reference']
    home = {'x_m': -.8, 'y_m': .4, 'yaw_rad': .1}
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'sh', lambda *_a, **_kw: None)
    monkeypatch.setattr(d, 'pull_registry', lambda: {'home': home})
    monkeypatch.setattr(d, 'capture_scan', lambda _path: None)
    monkeypatch.setattr(d, 'refine_at_registered_dock', lambda *_a, **_k: None)
    monkeypatch.setattr(d, 'refine_pose', lambda *_a: {
        'pose': [-.79, .35, .12], 'inlier_5cm': .5, 'global_agrees': False,
        'click_offset_m': .05, 'click_offset_deg': 1., 'click': [-.8, .4, .1],
    })
    monkeypatch.setattr(d, 'read_events', lambda _path: [{
        'event': 'navigation_activated_without_motion',
        'stationary_pose': [-.79, .35, .12],
    }])
    monkeypatch.setattr(d.scan_match, 'load_map', lambda *_a: (
        None, np.zeros((3, 3), dtype=bool), None, .05, [0., 0., 0.]))
    monkeypatch.setattr(d.dock_survey, 'raw_points_map', lambda *_a: ([], 1))
    monkeypatch.setattr(d.dock_survey, 'clusters', lambda *_a: [])

    d.cmd_init(SimpleNamespace(
        seed_from_state=False, click=False, local_only=True, keep_home=True))

    assert d.load_state()['localized'] is True
    assert d.load_state()['dock_scan_reference'] == original
    d.configure_map_session(fixture.pointer)
    assert d.load_state()['localized'] is True
    assert d.load_state()['dock_scan_reference'] == original


def test_new_water_station_request_uses_profile_and_has_no_via(
        tmp_path, monkeypatch):
    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    d.write_routes((-.8, .4, .1))
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])

    events = d.run_cycle(
        go_args(table_id='water_station'), d.load_state(), 'water_station')

    assert events == [{'event': 'home_arrived'}]
    argv = json.loads(next(value for value in pi.stdins if value))['argv']
    assert argv.count('--table-id') == 1
    assert argv[argv.index('--table-id') + 1] == 'water_station'
    assert '--station-profiles' in argv
    assert argv[argv.index('--station-profiles') + 1] == str(
        d.pi_path(fixture.station_profiles))
    assert argv[argv.index('--approach-route') + 1] == str(
        d.pi_path(fixture.runtime / 'water_station_direct_route.yaml'))
    assert '--via-id' not in argv
    assert '--via-route' not in argv
    assert '--via-region-xy' not in argv


def test_taught_map_table_cycle_visits_water_then_table_and_returns(
        tmp_path, monkeypatch, capsys):
    fixture = _map_session_fixture(tmp_path)
    _add_taught_table_01(fixture)
    for name in (
            'ACTIVE_MAP_SESSION', 'BASE', 'P2', 'MAP_YAML', 'KEEPOUT_YAML',
            'REGISTRY', 'ANNOTATION', 'CLICK', 'STATE', 'PI_BASE', 'PI_P2',
            'PI_REGISTRY', 'ACTIVE_MAP_POINTER', 'PI_TOOLS', 'CAMERA_MOUNT_CONFIG',
            'PI_MOUNT', 'DESTINATIONS', 'TABLES',
            'STATION_PROFILES'):
        monkeypatch.setattr(d, name, getattr(d, name))
    monkeypatch.setattr(d, 'DATA_ROOT', fixture.data_root)
    monkeypatch.setattr(d, 'prepare_reference_mount', lambda: False)
    monkeypatch.setitem(
        sys.modules, 'jdamr_cube_navigation.map_session_assets',
        SimpleNamespace(prepare_map_session=lambda _pointer: fixture.profile))
    d.configure_map_session(fixture.pointer)
    d.write_routes((-.8, .4, .1))

    water = json.loads((fixture.runtime / 'destinations.json').read_text())[
        'destinations']['water_station']
    normal = math.radians(water['normal_deg'])
    expected_escape = {
        'x': round(water['face_xy'][0] + d.ESCAPE_M * math.cos(normal), 3),
        'y': round(water['face_xy'][1] + d.ESCAPE_M * math.sin(normal), 3),
    }
    table_route = yaml.safe_load((fixture.runtime / 'table_01_route.yaml').read_text())
    direct_route = yaml.safe_load(
        (fixture.runtime / 'table_01_direct_route.yaml').read_text())
    assert table_route['start_pose'] == expected_escape
    assert [point['id'] for point in table_route['waypoints']] == [
        'table_01_pre', 'table_01_observation']
    assert direct_route['start_pose'] == {'x': -.8, 'y': .4}
    assert direct_route['waypoints'][0]['id'] == 'home_exit'

    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(table_id='table_01'), d.load_state(), 'table_01')
    argv = json.loads(next(value for value in pi.stdins if value))['argv']
    assert argv[argv.index('--table-id') + 1] == 'table_01'
    assert argv[argv.index('--approach-route') + 1] == str(
        d.pi_path(fixture.runtime / 'table_01_route.yaml'))
    assert argv[argv.index('--via-id') + 1] == 'water_station'
    assert argv[argv.index('--via-route') + 1] == str(
        d.pi_path(fixture.runtime / 'water_station_route.yaml'))
    table_face = json.loads((fixture.runtime / 'destinations.json').read_text())[
        'destinations']['table_01']['face_xy']
    assert argv[argv.index('--region-xy') + 1:argv.index('--region-xy') + 3] == [
        str(table_face[0]), str(table_face[1])]
    assert 'SUBMITTED water_station -> table_01 -> dock:' in capsys.readouterr().out

    direct_pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', direct_pi)
    d.run_cycle(
        go_args(table_id='table_01', skip_via=True), d.load_state(), 'table_01')
    direct_argv = json.loads(next(value for value in direct_pi.stdins if value))['argv']
    assert direct_argv[direct_argv.index('--approach-route') + 1] == str(
        d.pi_path(fixture.runtime / 'table_01_direct_route.yaml'))
    assert '--via-id' not in direct_argv


def test_sync_skips_map_pose_only_null_reference_and_keeps_water_rgbd(
        tmp_path, monkeypatch):
    fixture = _configure_fixture_session(tmp_path, monkeypatch)
    references = []
    for name, content in (
            ('capture.json', b'{}\n'), ('rgb.ppm', b'P6\n1 1\n255\n\x00\x00\x00'),
            ('depth.npz', b'fixture-depth')):
        path = fixture.session / 'stations' / 'water_station' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        references.append(path)
    profiles = json.loads(fixture.station_profiles.read_text())
    profiles['stations']['water_station']['rgbd_reference'] = dict(zip(
        ('reference_capture_path', 'reference_rgb_path', 'reference_depth_path'),
        map(str, references)))
    table_record = fixture.session / 'table_01.json'
    table_record.write_text('{}\n')
    profiles['stations']['table_01'] = {
        'source_record': str(table_record),
        'reference_mode': 'map_pose_only', 'rgbd_reference': None,
    }
    fixture.station_profiles.write_text(json.dumps(profiles))

    shell_calls, pi_calls = [], []

    def fake_sh(command, **_kwargs):
        shell_calls.append(command)
        return SimpleNamespace(returncode=0, stdout='')

    local_by_name = {
        name: fixture.runtime / name
        for name in ('service_destinations.yaml', 'keepout.pgm', 'keepout.yaml')
    }
    reference_sources = [*references, fixture.runtime / 'map.yaml',
                         fixture.runtime / 'map.pgm', fixture.session / 'dock.json',
                         fixture.session / 'water_station.json', table_record,
                         fixture.session / 'dock_scan_reference.json',
                         fixture.session / 'dock_scan.json', fixture.pointer]
    local_by_name.update({path.name: path for path in reference_sources})

    def fake_pi(command, **_kwargs):
        pi_calls.append(command)
        if command.startswith('sha256sum '):
            remote_name = Path(command.split()[-1]).name
            return SimpleNamespace(
                returncode=0, stdout=d.sha256(local_by_name[remote_name]) + '  remote\n')
        return SimpleNamespace(returncode=0, stdout='')

    monkeypatch.setattr(d, 'sh', fake_sh)
    monkeypatch.setattr(d, 'pi', fake_pi)
    d.cmd_sync(None)

    reference_syncs = [call for call in shell_calls if '--protect-args' in call]
    assert len(reference_syncs) == len(reference_sources)
    assert all(any(str(path) in call for call in reference_syncs) for path in references)
    assert sum(command.startswith('sha256sum ') for command in pi_calls) == 3 + len(reference_sources)


def test_observation_waypoint_uses_mount_profile_without_changing_aisle_point(
        tmp_path, monkeypatch):
    mount = tmp_path / 'camera.yaml'
    mount.write_text(yaml.safe_dump({
        'camera_mount': {'table_observation': {'base_offset_m': .37}}}))
    monkeypatch.setattr(d, 'CAMERA_MOUNT_CONFIG', mount)
    points = d.approach_waypoints('table_01')
    assert points[-1]['x'] == round(d.along_normal('table_01', .37)[0], 3)
    assert points[0]['x'] == round(d.along_normal('table_01', d.OBSERVE_M)[0], 3)


def test_departure_log_reports_the_spooled_stops(env, monkeypatch, capsys):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert 'SUBMITTED water_station -> table_02 -> dock:' in capsys.readouterr().out


@pytest.mark.parametrize('extra', [
    {'surface_mode': 'table_edge', 'with_via': True, 'skip_via': True},
    {'surface_mode': 'front', 'with_via': True},
])
def test_go_rejects_conflicting_or_front_with_via_before_pi_work(env, monkeypatch, extra):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(**extra))
    assert pi.calls == []


@pytest.mark.parametrize('extra', [{'final_gap_m': 0.005}, {'final_gap_m': 0.2},
                                   {'dwell_s': -1.0}, {'dwell_s': 500.0}])
def test_go_refuses_out_of_range_gap_or_dwell_before_anything_starts(env, monkeypatch, extra):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(**extra))
    assert pi.calls == []


def test_go_profiles_the_pi_load_and_stops_the_profiler_at_the_end(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    d.run_cycle(go_args(record_diagnostics=True), d.load_state(), 'table_02')
    start = next(c for c in pi.calls if 'systemd-run --unit=jdamr-load-profile-' in c)
    assert 'RuntimeMaxSec=' in start and 'MemoryMax=' in start and 'load_profile.jsonl' in start
    unit = start.split('--unit=')[1].split()[0]
    assert any(c.startswith('sudo -n systemctl stop ') and unit in c for c in pi.calls)


def test_monitor_units_are_stopped_when_the_run_is_interrupted(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)

    def interrupted(_path):
        assert d.load_state()['monitor_units'].startswith('jdamr-udp-drops-')
        raise KeyboardInterrupt
    monkeypatch.setattr(d, 'read_events', interrupted)
    with pytest.raises(KeyboardInterrupt):
        d.run_cycle(go_args(record_diagnostics=True), d.load_state(), 'table_02')
    stop = next(c for c in pi.calls if c.startswith('sudo -n systemctl stop jdamr-udp-drops-'))
    assert 'jdamr-load-profile-' in stop
    assert 'monitor_units' not in d.load_state()


def test_stop_closes_monitor_units_left_by_a_dead_go(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    state = d.load_state()
    state['monitor_units'] = 'jdamr-udp-drops-x jdamr-load-profile-x'
    d.save_state(state)
    d.cmd_stop(SimpleNamespace())
    assert 'sudo -n systemctl stop jdamr-udp-drops-x jdamr-load-profile-x' in pi.calls
    assert 'monitor_units' not in d.load_state()


def test_stop_reaches_the_robot_before_closing_the_bag_and_monitors(env, monkeypatch):
    """Review 2026-10-06: a slow bag reindex held the stop back for up to 120 s."""
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    state = d.load_state()
    state.update(last_unit='executor:table_02', onboard_bag_unit='jdamr-onboard-bag-x',
                 onboard_bag_run=str(d.P2 / 'run'),
                 monitor_units='jdamr-udp-drops-x jdamr-load-profile-x')
    d.save_state(state)
    d.cmd_stop(SimpleNamespace())
    stop = next(i for i, c in enumerate(pi.calls) if 'kill --signal=SIGINT' in c)
    bag = next(i for i, c in enumerate(pi.calls) if 'jdamr-onboard-bag-x' in c)
    monitors = next(i for i, c in enumerate(pi.calls) if 'jdamr-udp-drops-x' in c)
    assert stop < bag and stop < monitors


def test_a_failing_monitor_start_does_not_end_run_tracking(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])

    def broken(_run_dir, _run):
        raise OSError('script missing')
    monkeypatch.setattr(d, 'start_drop_monitor', broken)
    events = d.run_cycle(go_args(record_diagnostics=True), d.load_state(), 'table_02')
    assert events == [{'event': 'home_arrived'}]
    assert any(c.startswith('sudo -n systemctl stop jdamr-udp-drops-') for c in pi.calls)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), 0.2, 1.5])
def test_init_min_inlier_is_validated_before_any_ssh(env, monkeypatch, value):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    with pytest.raises(SystemExit):
        d.cmd_init(SimpleNamespace(seed_from_state=True, local_only=True, keep_home=True,
                                   min_inlier=value))
    assert pi.calls == [] and d.load_state()['localized'] is True


def test_init_records_the_inlier_threshold_actually_used(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'capture_scan', lambda _path: None)
    refined = {'pose': [0.0, 0.0, 0.0], 'inlier_5cm': 0.2, 'global_agrees': False,
               'click_offset_m': 0.0, 'click_offset_deg': 0.0, 'click': {}}
    monkeypatch.setattr(d, 'refine_pose', lambda _scan, _click: dict(refined))
    args = SimpleNamespace(seed_from_state=True, local_only=True, keep_home=True,
                           min_inlier=0.3)
    with pytest.raises(SystemExit):                       # 0.2 < 0.3: refused, but recorded
        d.cmd_init(args)
    written = json.loads(next(env.glob('runs/init_*/refine.json')).read_text())
    assert written['min_inlier_used'] == 0.3


def test_status_and_departure_ignore_obsolete_operator_alert_settings(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    messages = []
    monkeypatch.setattr(d, 'log', messages.append)
    monkeypatch.setattr(d, 'estop_state', lambda: 'released')
    monkeypatch.setattr(d, 'read_events', lambda _path: [{'event': 'home_arrived'}])
    state = d.load_state()
    state['operator_alerts'] = True
    d.save_state(state)
    d.cmd_status(SimpleNamespace())
    d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert not any('operator alerts' in m for m in messages)
    assert any(m.startswith('SUBMITTED') for m in messages)


@pytest.mark.parametrize('result', ['{"code": 1}', '{"code": 2}', '', '{', '{}',
                                   '[]', '{"code": false}', '{"code": "0"}',
                                   'executor_unavailable'])
def test_failed_or_invalid_executor_result_cannot_be_hidden_by_home_event(
        env, monkeypatch, result):
    pi = ExecutorPi(result=result)
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    monkeypatch.setattr(d, 'cmd_init', lambda _args: pytest.fail('started next table'))
    with pytest.raises(SystemExit):
        d.cmd_go(go_args(table_ids=['table_02', 'table_01']))
    assert len([c for c in pi.calls if is_submission(c)]) == 1
    assert any('cat ' in c and '.result;' in c for c in pi.calls)


def test_map_retry_requires_an_initial_map_failure_not_invalid_completion():
    assert d.map_wait_failed([*MAP_FAIL, {'event': 'executor_failed', 'code': 1,
                                        'reason': 'executor_exit_nonzero'}])
    assert not d.map_wait_failed([*MAP_FAIL, {'event': 'executor_failed', 'code': None,
                                            'reason': 'executor_result_invalid'}])


MAP_FAILURE_EVENTS = [
    *MAP_FAIL,
    {'event': 'operator_call', 'reason': 'failed: live map data unavailable: map'},
    {'event': 'box_camera', 'action': 'stop', 'ok': True},
]


@pytest.mark.parametrize('recovered', [True, False])
def test_real_map_failure_metadata_retries_once_without_notification_side_effects(
        env, monkeypatch, recovered):
    pi = ExecutorPi(result='{"code": 1}')
    monkeypatch.setattr(d, 'pi', pi)
    restarts = []
    monkeypatch.setattr(d, 'read_events', lambda _path: (
        HOME_EVENTS.copy() if pi.result == '{"code": 0}' else MAP_FAILURE_EVENTS.copy()))

    def restart():
        restarts.append(True)
        if recovered:
            pi.result = '{"code": 0}'

    monkeypatch.setattr(d, 'restart_executor', restart)
    if recovered:
        d.cmd_go(go_args())
    else:
        with pytest.raises(SystemExit):
            d.cmd_go(go_args())
    assert restarts == [True]
    assert len([c for c in pi.calls if is_submission(c)]) == 2


@pytest.mark.parametrize('extra', [
    {'event': 'observation_route_selected'},
    {'event': 'failed', 'phase': 'cleanup', 'reason': 'camera_stop_failed'},
    {'event': 'box_camera', 'action': 'stop', 'ok': False},
    {'event': 'operator_call', 'reason': 'emergency stop engaged'},
])
def test_map_retry_never_hides_motion_or_other_failures(extra):
    assert not d.map_wait_failed([*MAP_FAILURE_EVENTS, extra])


def test_predeparture_ssh_reads_and_submission_have_short_timeouts(env, monkeypatch):
    pi = ExecutorPi()
    requests = []

    def record(command, **kwargs):
        requests.append((command, kwargs.get('timeout', 120)))
        return pi(command, **kwargs)

    monkeypatch.setattr(d, 'pi', record)
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    d.cmd_go(go_args())
    submitted = next(i for i, (c, _) in enumerate(requests) if is_submission(c))
    assert all(timeout <= 10 for _, timeout in requests[:submitted + 1])


def test_departure_durations_use_monotonic_clock_not_stop_timestamp(env, monkeypatch):
    pi = ExecutorPi()
    clock = {'now': 123.0}
    monkeypatch.setattr(d.time, 'time', lambda: 1800000000.0)
    monkeypatch.setattr(d.time, 'monotonic', lambda: clock['now'])
    messages = []
    monkeypatch.setattr(d, 'log', messages.append)

    def timed_pi(command, **kwargs):
        if is_submission(command):
            clock['now'] += 1.5
        return pi(command, **kwargs)

    monkeypatch.setattr(d, 'pi', timed_pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    d.cmd_go(go_args())
    assert any('request_submit_s=1.500' in m for m in messages)
    assert 'executor first event after 1.500 s' in messages


@pytest.mark.parametrize('reply', ['timeout', 'ssh_error'])
def test_ambiguous_submission_follows_same_run_without_reissuing(env, monkeypatch, reply):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())

    def uncertain(command, **kwargs):
        result = pi(command, **kwargs)
        if is_submission(command):
            assert d.load_state()['last_unit'].startswith('executor:')
            if reply == 'timeout':
                raise subprocess.TimeoutExpired('ssh', 10)
            return SimpleNamespace(returncode=255, stdout='lost reply')
        return result

    monkeypatch.setattr(d, 'pi', uncertain)
    d.cmd_go(go_args())
    assert len([c for c in pi.calls if is_submission(c)]) == 1
    assert any('.result' in c and not is_submission(c) for c in pi.calls)


def test_cancelled_missing_submission_cannot_arrive_later(env, monkeypatch):
    spool = env / 'spool'
    spool.mkdir()
    monkeypatch.setattr(d, 'EXECUTOR_SPOOL', str(spool))
    pi = ExecutorPi(result='request_missing')
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: [])
    events = d.run_cycle(go_args(), d.load_state(), 'table_02')
    assert events[-1]['reason'] == 'request_missing'
    publish = next(c for c in pi.calls if is_submission(c))
    status = next(c for c in pi.calls if 'if test -e ' in c and '.result' in c)
    # Execute only the generated spool protocol against temporary local paths.
    # No request exists, so this branch never invokes systemctl.
    missing = subprocess.run(['bash', '-c', status], capture_output=True, text=True, check=True)
    assert missing.stdout.strip() == 'request_missing'
    late = subprocess.run(['bash', '-c', publish], input='{"argv": []}',
                          capture_output=True, text=True)
    assert late.returncode == 1
    assert list(spool.glob('*.cancelled'))
    assert not list(spool.glob('*.request')) and not list(spool.glob('*.part'))


@pytest.mark.parametrize('pending_suffix', ['part', 'request', 'running'])
def test_stop_cancels_pending_publication_and_signals_claimed_request(
        env, monkeypatch, pending_suffix):
    spool = env / 'spool'
    spool.mkdir()
    monkeypatch.setattr(d, 'EXECUTOR_SPOOL', str(spool))
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    d.run_cycle(go_args(), d.load_state(), 'table_02')
    run = d.load_state()['last_unit'].split(':', 1)[1]
    (spool / f'{run}.{pending_suffix}').write_text('{"argv": []}')
    publish = next(c for c in pi.calls if is_submission(c))
    d.cmd_stop(None)
    stop = pi.calls[-1]
    # Stub only systemctl/sudo; run the actual lock and file operations locally.
    commands = 'sudo() { return 0; }; ' + stop
    subprocess.run(['bash', '-c', commands], capture_output=True, text=True, check=True)
    late = subprocess.run(['bash', '-c', publish], input='{"argv": []}',
                          capture_output=True, text=True)
    assert late.returncode == 1
    assert (spool / f'{run}.cancelled').exists()
    assert not list(spool.glob('*.request')) and not list(spool.glob('*.part'))
    assert (spool / f'{run}.running').exists() is (pending_suffix == 'running')
    assert stop.count('kill --signal=SIGINT') == 2
    assert stop.index('.cancelled') < stop.rindex('kill --signal=SIGINT')


@pytest.mark.parametrize('reply', ['timeout', 'ssh_error'])
def test_stop_never_reports_success_without_remote_reply(env, monkeypatch, reply, capsys):
    state = d.load_state()
    state['last_unit'] = 'executor:run'
    d.save_state(state)

    def unavailable(command, **kwargs):
        assert kwargs['timeout'] == d.SSH_READ_TIMEOUT_S
        if reply == 'timeout':
            raise subprocess.TimeoutExpired('ssh', 10)
        return SimpleNamespace(returncode=255, stdout='')

    monkeypatch.setattr(d, 'pi', unavailable)
    with pytest.raises(SystemExit) as raised:
        d.cmd_stop(None)
    assert raised.value.code == 1
    output = capsys.readouterr().out
    assert 'cancellation unconfirmed' in output and 'stop sent' not in output


def test_restart_executor_never_falls_back_or_restarts_sensors(env, monkeypatch):
    pi = Pi()
    def stuck_executor(command, **kwargs):
        result = pi(command, **kwargs)
        if command.startswith('sudo -n systemctl stop'):
            return SimpleNamespace(returncode=1, stdout='stop failed')
        return result
    monkeypatch.setattr(d, 'pi', stuck_executor)
    with pytest.raises(SystemExit):
        d.restart_executor()
    assert not any(unit in c for c in pi.calls for unit in d.SENSOR_UNITS)
    assert d.load_state()['localized'] is True


def test_restart_of_collected_executor_starts_even_when_stop_reports_missing(env, monkeypatch):
    calls = []
    monkeypatch.setattr(d, 'refuse_during_cycle', lambda action: None)
    monkeypatch.setattr(d, 'pi', lambda *a, **k: SimpleNamespace(returncode=1, stdout='not loaded'))
    monkeypatch.setattr(d, 'executor_active', lambda: False)
    monkeypatch.setattr(d, 'start_executor', lambda: calls.append('start') or True)
    d.restart_executor()
    assert calls == ['start']


def test_executor_start_failure_cannot_dispatch_one_shot_or_revoke_pose(env, monkeypatch):
    pi = Pi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'start_executor', lambda: False)
    with pytest.raises(SystemExit):
        d.cmd_go(go_args())
    assert not any(is_submission(c) or 'jdamr-table-cycle-' in c for c in pi.calls
                   if 'list-units' not in c)
    assert d.load_state()['localized'] is True


def test_session_start_reports_executor_failure(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', lambda c, **k: SimpleNamespace(
        returncode=0, stdout='' if 'jdamr-table-cycle-*' in c else 'REUSED'))
    monkeypatch.setattr(d, 'start_executor', lambda: False)
    with pytest.raises(SystemExit):
        d.cmd_session_start(None)
    assert d.load_state()['localized'] is True


def test_prepared_go_has_no_sleep_or_setup_before_submit_and_first_poll(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    monkeypatch.setattr(d.time, 'sleep', lambda _s: pytest.fail('delayed first poll'))
    for name in ('cmd_recover', 'cmd_init', 'cmd_session_start', 'cmd_sync',
                 'cmd_display_start', 'start_onboard_bag', 'start_drop_monitor'):
        monkeypatch.setattr(d, name, lambda *a, **kw: pytest.fail('setup during prepared go'))
    d.cmd_go(go_args())
    assert len([c for c in pi.calls if is_submission(c)]) == 1


def test_fast_runs_have_distinct_spool_names_within_one_second(env, monkeypatch):
    pi = ExecutorPi()
    monkeypatch.setattr(d, 'pi', pi)
    monkeypatch.setattr(d.time, 'strftime', lambda _fmt: '20261015_120000')
    monkeypatch.setattr(d, 'read_events', lambda _path: HOME_EVENTS.copy())
    for _ in range(2):
        d.run_cycle(go_args(), d.load_state(), 'table_02')
    requests = [c for c in pi.calls if is_submission(c)]
    assert len(requests) == 2 and requests[0] != requests[1]


def test_display_relay_uses_configured_host_and_recording_is_opt_in(env, monkeypatch):
    commands = []
    monkeypatch.setattr(d, 'HOST', 'robot@192.0.2.10')
    started = set()

    def sh(command, **_kw):
        commands.append(command)
        if command.startswith('systemd-run'):
            started.add(command.split('--unit=')[1].split()[0])
        active = command.split()[-1] in started
        return SimpleNamespace(returncode=0, stdout='active' if active else 'inactive')

    monkeypatch.setattr(d, 'sh', sh)
    records = []
    monkeypatch.setattr(d, 'record_start', lambda: records.append(True))
    d.cmd_display_start(None)
    relay = next(c for c in commands if '--unit=jdamr-p2-relay ' in c)
    assert '--host robot@192.0.2.10 ' in relay and 'jdamr.local' not in relay
    assert records == []
    d.cmd_display_start(SimpleNamespace(record_diagnostics=True))
    assert records == [True]


@pytest.mark.parametrize('display_active', [False, True])
def test_manual_recovery_does_not_require_display_or_recording(env, monkeypatch, display_active):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'sh', lambda *a, **k: SimpleNamespace(
        returncode=0 if display_active else 3, stdout='active' if display_active else 'inactive'))
    calls = []
    for name in ('cmd_display_stop', 'cmd_session_stop', 'restart_sensor_services',
                 'cmd_session_start', 'cmd_init'):
        monkeypatch.setattr(d, name, lambda *a, n=name: calls.append(n))

    def broken_display(args):
        calls.append('display_restore')
        assert args.record_diagnostics is False
        raise SystemExit('X11 unavailable')

    monkeypatch.setattr(d, 'cmd_display_start', broken_display)
    monkeypatch.setattr(d, 'dds_health', lambda: ('ok', 'healthy'))
    monkeypatch.setattr(d, 'record_start', lambda: pytest.fail('recording during recovery'))
    d.cmd_recover(SimpleNamespace(seed=None, local_only=False))
    assert 'cmd_init' in calls and 'cmd_session_start' in calls
    assert ('display_restore' in calls) is display_active
    assert ('cmd_display_stop' in calls) is display_active


@pytest.mark.parametrize('events,expected', [
    ([{'event': 'precision_straight_finished', 'target_failure': 'rear box target changed'},
      {'event': 'operator_call', 'reason': 'cycle failed'},
      {'event': 'executor_failed', 'reason': 'executor_exit_nonzero', 'code': 1}],
     'rear box target changed'),
    ([{'event': 'failed', 'reason': 'rear box observation deadline exceeded'}],
     'rear box observation deadline exceeded'),
    ([{'event': 'straight_drive_stopped', 'reason': 'table approach depth stale or future-dated'},
      {'event': 'trim_rotation_stopped', 'reason': 'rotation scan stale or future-dated'},
      {'event': 'executor_failed', 'reason': 'executor_exit_nonzero'}],
     'rotation scan stale or future-dated'),
    ([{'event': 'straight_drive_stopped', 'reason': 'rear box coarse handoff ready'}], None),
    ([{'event': 'precision_straight_finished', 'target_failure': None}], None),
])
def test_executor_failure_retains_the_action_cause(events, expected):
    assert d.execution_failure_cause(events) == expected


def _dock_scan_pair(monkeypatch, *, delta=(.03, -.02, .04), shape='room'):
    import numpy as np
    rng = np.random.default_rng(17)
    angles = np.linspace(-math.pi, math.pi, 120, endpoint=False)
    radii = rng.uniform(.5, 4., len(angles))
    if shape == 'circle':
        radii[:] = 2.
    anchored = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    if shape == 'line':
        anchored = np.column_stack((np.linspace(-2., 2., 120), np.ones(120)))
    if shape == 'two_sector_unique':
        angles = np.concatenate((rng.uniform(.2, 1.35, 60),
                                 rng.uniform(1.8, 2.9, 60)))
        radii = rng.uniform(.5, 4., len(angles))
        anchored = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    if shape == 'one_sector_unique':
        angles = rng.uniform(.2, 1.35, 120)
        radii = rng.uniform(.5, 4., len(angles))
        anchored = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    if shape == 'two_sector_translation_ambiguous':
        angles = np.concatenate((rng.uniform(.2, 1.35, 60),
                                 rng.uniform(1.8, 2.9, 60)))
        radii = rng.uniform(.5, 4., len(angles))
        base = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
        anchored = np.concatenate((base, base + np.array([.06, 0.])))
    c, s = math.cos(delta[2]), math.sin(delta[2])
    current = (anchored - delta[:2]) @ np.array([[c, -s], [s, c]])
    if shape == 'clutter':
        current = rng.uniform(-5., 5., (120, 2))
    if shape == 'sparse':
        current = current[:40]
    if shape == 'occluded_reference':
        current = current[:80]
    if shape == 'weak_matched_support':
        current = np.concatenate((current[np.linspace(0, 119, 75, dtype=int)],
                                  rng.uniform(-5., 5., (25, 2))))
    if shape == 'new_current_returns':
        current = np.concatenate((current, rng.uniform(5., 8., (60, 2))))

    def laser_points(path):
        points = current if path == 'current' else anchored
        return points, None, None, None, (0., 0., math.pi), {}

    monkeypatch.setattr(d.scan_match, 'laser_points', laser_points)
    monkeypatch.setattr(d, 'dock_map_identity', lambda: ['yaml', 'image'])
    home = [.12, .165, .134]
    click = dict(zip(('x_m', 'y_m', 'yaw_rad'), home), source='registered_dock')
    reference = {'scan_path': 'reference', 'pose_map': home, 'home_pose': home,
                 'map_identity': ['yaml', 'image']}
    return click, reference


def test_registered_dock_reference_retains_its_map_anchor(monkeypatch):
    delta = (.03, -.02, .04)
    click, reference = _dock_scan_pair(monkeypatch, delta=delta)
    result = d.refine_at_registered_dock('current', click, reference)
    assert result is not None
    assert result['basis'] == 'anchored_dock_scan'
    assert result['global_agrees'] is False
    assert result['relative_pose'] == pytest.approx(delta, abs=.006)
    c, s = math.cos(.134), math.sin(.134)
    assert result['pose'] == pytest.approx(
        [.12 + c * delta[0] - s * delta[1], .165 + s * delta[0] + c * delta[1],
         .134 + delta[2]], abs=.006)
    assert reference['pose_map'] == [.12, .165, .134]
    # The accepted numeric result is persisted as the activation seed and state.
    assert yaml.safe_load(yaml.safe_dump(result)) == result


@pytest.mark.parametrize('shape', ['occluded_reference', 'new_current_returns'])
def test_registered_dock_accepts_changed_visibility_with_enough_matched_support(
        monkeypatch, shape):
    click, reference = _dock_scan_pair(monkeypatch, shape=shape)
    result = d.refine_at_registered_dock('current', click, reference)
    assert result is not None
    assert min(result['inlier_5cm'], result['reference_inlier_5cm']) < .70
    assert min(result['current_inlier_count'], result['reference_inlier_count']) >= 80
    assert result['relative_pose'] == pytest.approx((.03, -.02, .04), abs=.006)


@pytest.mark.parametrize(('shape', 'expected_sectors'),
                         [('one_sector_unique', 1), ('two_sector_unique', 2)])
def test_registered_dock_accepts_unique_fixed_support_regardless_of_sector_bins(
        monkeypatch, shape, expected_sectors):
    diagnostics = {}
    click, reference = _dock_scan_pair(monkeypatch, shape=shape)
    result = d.refine_at_registered_dock(
        'current', click, reference, diagnostics=diagnostics)
    assert result is not None
    assert diagnostics['supported_sectors'] == expected_sectors
    assert diagnostics['ambiguous_translation_trials'] == []
    assert len(diagnostics['uniqueness_trials']) == 6


def test_registered_dock_rejects_two_sector_translation_ambiguity(monkeypatch):
    diagnostics = {}
    click, reference = _dock_scan_pair(
        monkeypatch, shape='two_sector_translation_ambiguous')
    result = d.refine_at_registered_dock(
        'current', click, reference, diagnostics=diagnostics)
    assert result is None
    assert diagnostics['reason'] == 'dock_match_limited_angular_support_not_unique'
    assert diagnostics['supported_sectors'] == 2
    assert diagnostics['ambiguous_translation_trials']
    assert len(diagnostics['uniqueness_trials']) == 6


@pytest.mark.parametrize('case', ['click', 'previous_seed', 'new_map', 'new_home',
                                  'translation', 'rotation', 'circle', 'line',
                                  'sparse', 'clutter', 'weak_matched_support'])
def test_dock_reference_does_not_accept_a_different_or_ambiguous_placement(monkeypatch, case):
    delta = (.3, 0., 0.) if case == 'translation' else (
        (0., 0., .3) if case == 'rotation' else (.03, -.02, .04))
    click, reference = _dock_scan_pair(monkeypatch, delta=delta, shape=case)
    if case == 'click':
        click['source'] = 'rviz_click'
    elif case == 'previous_seed':
        click['source'] = 'previous_init_scan_match'
    elif case == 'new_map':
        reference['map_identity'] = ['different', 'image']
    elif case == 'new_home':
        reference['home_pose'] = [3., 2., .134]
    assert d.refine_at_registered_dock('current', click, reference) is None


@pytest.mark.parametrize('activation', ['failed', 'amcl_disagrees'])
def test_dock_reference_still_requires_activation_and_amcl_agreement(
        env, monkeypatch, activation):
    monkeypatch.setattr(d, 'pi', Pi())
    monkeypatch.setattr(d, 'pull_registry', lambda: {'home': {
        'x_m': .12, 'y_m': .165, 'yaw_rad': .134}})
    monkeypatch.setattr(d, 'capture_scan', lambda _path: None)
    monkeypatch.setattr(d, 'sh', lambda *_a, **_k: None)
    monkeypatch.setattr(d, 'refine_pose', lambda *_a: pytest.fail('unexpected map fallback'))
    monkeypatch.setattr(d, 'refine_at_registered_dock', lambda *_a, **_k: {
        'pose': [.12, .165, .134], 'inlier_5cm': .9, 'global_agrees': False,
        'basis': 'anchored_dock_scan',
        'click_offset_m': 0., 'click_offset_deg': 0., 'click': [.12, .165, .134]})
    events = [] if activation == 'failed' else [{
        'event': 'navigation_activated_without_motion', 'stationary_pose': [.5, .165, .134]}]
    monkeypatch.setattr(d, 'read_events', lambda _path: events)
    args = SimpleNamespace(seed_from_state=False, click=False, local_only=False, keep_home=True)
    with pytest.raises(SystemExit) as caught:
        d.cmd_init(args)
    assert caught.value.code == 1
    assert d.load_state()['localized'] is False


CONTROLLER_FAIL = [
    {'event': 'station_reference_configured', 'table_id': 'water_station'},
    {'event': 'observation_route_selected'},
    {'event': 'failed', 'phase': 'preflight',
     'reason': 'controller_server parameter service unavailable'},
    {'event': 'box_camera', 'action': 'stop', 'ok': True},
    {'event': 'executor_failed', 'code': 1, 'reason': 'executor_exit_nonzero'},
]


def test_controller_preflight_read_failure_recovers_once_from_registered_dock(env, monkeypatch):
    monkeypatch.setattr(d, 'pi', Pi())
    state=d.load_state();state['init_pose_source']='anchored_dock_scan';d.save_state(state)
    calls=[]
    results=iter([CONTROLLER_FAIL, HOME_EVENTS.copy()])
    monkeypatch.setattr(d,'run_cycle',lambda *a: calls.append('run') or next(results))
    monkeypatch.setattr(d,'recover_controller_before_departure',lambda start: calls.append('recover'))
    d.cmd_go(go_args())
    assert calls==['run','recover','run']


@pytest.mark.parametrize('extra,events', [
    ({'resume_at_observation':True},CONTROLLER_FAIL),
    ({'dock_only':True},CONTROLLER_FAIL),
    ({},[*CONTROLLER_FAIL,{'event':'box_escape_finished'}]),
])
def test_controller_recovery_never_resets_a_resume_or_moved_cycle(env, monkeypatch, extra, events):
    monkeypatch.setattr(d,'pi',Pi())
    state=d.load_state();state['init_pose_source']='anchored_dock_scan';d.save_state(state)
    monkeypatch.setattr(d,'run_cycle',lambda *a:events)
    monkeypatch.setattr(d,'recover_controller_before_departure',lambda *a:pytest.fail('unsafe reset'))
    with pytest.raises(SystemExit):d.cmd_go(go_args(**extra))


def test_controller_recovery_is_bounded_to_one_attempt(env,monkeypatch):
    monkeypatch.setattr(d,'pi',Pi())
    state=d.load_state();state['init_pose_source']='anchored_dock_scan';d.save_state(state)
    calls=[]
    monkeypatch.setattr(d,'run_cycle',lambda *a:calls.append('run') or CONTROLLER_FAIL)
    monkeypatch.setattr(d,'recover_controller_before_departure',lambda *a:calls.append('recover'))
    with pytest.raises(SystemExit):d.cmd_go(go_args())
    assert calls==['run','recover','run']


def test_controller_recovery_stop_before_restart_never_starts_services(env,monkeypatch):
    state=d.load_state();state['stop_unix']=100.;d.save_state(state)
    monkeypatch.setattr(d,'cmd_session_stop',lambda *a:pytest.fail('restart after stop'))
    with pytest.raises(SystemExit):d.recover_controller_before_departure(99.)


def test_controller_recovery_orders_stop_start_init_without_motion(env,monkeypatch):
    calls=[]
    monkeypatch.setattr(d,'refuse_during_cycle',lambda *a:None)
    monkeypatch.setattr(d,'cmd_session_stop',lambda *a:calls.append('stop'))
    monkeypatch.setattr(d,'cmd_session_start',lambda *a:calls.append('start'))
    def init(args):
        assert args.keep_home and not args.seed_from_state and not args.click
        calls.append('init')
    monkeypatch.setattr(d,'cmd_init',init)
    d.recover_controller_before_departure(1.)
    assert calls==['stop','start','init']


@pytest.mark.parametrize('stage', ['stop', 'start', 'init'])
def test_controller_recovery_stop_between_steps_prevents_remaining_work(env, monkeypatch, stage):
    calls = []
    monkeypatch.setattr(d, 'refuse_during_cycle', lambda *a: None)
    def step(name):
        def run(*args):
            calls.append(name)
            if name == stage:
                state = d.load_state()
                state['stop_unix'] = 100.
                d.save_state(state)
        return run
    for name in ('stop', 'start', 'init'):
        monkeypatch.setattr(d, 'cmd_session_' + name if name != 'init' else 'cmd_init',
                            step(name))
    with pytest.raises(SystemExit):
        d.recover_controller_before_departure(99.)
    assert calls == ['stop', 'start', 'init'][:['stop', 'start', 'init'].index(stage) + 1]


@pytest.mark.parametrize('extra', [
    {'event': 'box_camera', 'action': 'start', 'ok': True},
    {'event': 'navigation_result'}, {'event': 'precision_straight_finished'},
])
def test_controller_recovery_rejects_motion_or_unknown_execution_evidence(extra):
    assert not d.controller_wait_failed([*CONTROLLER_FAIL, extra])


@pytest.mark.parametrize('command,handler', [
    ('stop', 'cmd_stop'), ('estop', 'cmd_estop'),
    ('session-stop', 'cmd_session_stop'), ('display-stop', 'cmd_display_stop'),
])
def test_stop_dispatch_survives_broken_taught_map(env, monkeypatch, tmp_path, command, handler):
    pointer = tmp_path / 'broken-pointer.json'
    pointer.write_text('{broken json')
    calls = []
    monkeypatch.setattr(d, 'configure_map_session', lambda *_: pytest.fail('perception before stop'))
    monkeypatch.setattr(d, handler, lambda args: calls.append(args.command))
    monkeypatch.setattr(sys, 'argv', ['jdamr_depart.py', '--map-session', str(pointer), command])
    d.main()
    assert calls == [command]


def test_taught_stop_reads_current_request_without_loading_reference(env, monkeypatch, tmp_path):
    base = tmp_path / 'new-session'
    pointer = tmp_path / 'current.json'
    pointer.write_text(json.dumps({'directory': str(base)}))
    monkeypatch.setattr(d, 'BASE', d.BASE)
    monkeypatch.setattr(d, 'P2', d.P2)
    monkeypatch.setattr(d, 'STATE', d.STATE)
    d.select_stop_state(pointer)
    assert d.STATE == base / 'runtime/state.json'


def test_stop_signals_robot_when_taught_state_directory_is_missing(env, monkeypatch, tmp_path):
    remote = Pi()
    monkeypatch.setattr(d, 'pi', remote)
    monkeypatch.setattr(d, 'STATE', tmp_path / 'missing/runtime/state.json')
    d.cmd_stop(None)
    assert 'kill --signal=SIGINT ' + d.EXECUTOR_UNIT in remote.calls[0]
    assert not d.STATE.exists()


def test_stop_without_request_state_still_signals_resident_executor(env, monkeypatch):
    remote = Pi()
    monkeypatch.setattr(d, 'pi', remote)
    d.save_state({})
    d.cmd_stop(None)
    assert any('kill --signal=SIGINT ' + d.EXECUTOR_UNIT in call for call in remote.calls)


def test_reference_camera_uses_single_driver_for_registered_rgb_and_depth(env, monkeypatch):
    monkeypatch.setattr(d, 'CAMERA_TRANSPORT', 'usb')
    calls = []
    monkeypatch.setattr(d, 'ACTIVE_MAP_SESSION', {'session_directory': 'new'})
    def remote(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout='inactive' if 'is-active' in command else '')
    monkeypatch.setattr(d, 'pi', remote)
    d.prepare_reference_camera()
    content = next(kwargs['stdin'] for _, kwargs in calls if 'stdin' in kwargs)
    assert content.count('ros2 launch astra_camera') == 1
    assert 'enable_color:=true' in content and 'enable_depth:=true' in content
    assert 'depth_registration:=true' in content
    assert not any('restart' in command for command, _ in calls)
