import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import Mock

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools/jdamr'
sys.path.insert(0, str(TOOLS))
from water_service import WaterServiceController
import water_service_api as api


@pytest.fixture
def controller(tmp_path, monkeypatch):
    data = tmp_path / 'data'
    session = data / 'maps/session'
    runs = session / 'runtime/runs'
    runs.mkdir(parents=True)
    pointer = tmp_path / 'pointer.json'
    pointer.write_text(json.dumps({'directory': str(session)}))
    monkeypatch.setenv('JDAMR_DATA_ROOT', str(data))
    node = WaterServiceController(inference_url='http://order.invalid',
                                  state_directory=tmp_path / 'state', map_session=pointer)
    mission = node.state_directory / 'runs/current'
    mission.mkdir(parents=True)
    node._log_path = mission / 'controller.log'
    node._state.update(mission_id='current', mission_directory=str(mission),
                       phase='parking', state='failed', started_unix_s=100.)
    return node, runs


def make_run(runs, name, stamp=101.):
    run = runs / name
    run.mkdir()
    command = run / 'command.txt'
    command.write_text('--table-id water_station --search')
    os.utime(command, (stamp, stamp))
    (runs.parent / 'state.json').write_text(json.dumps({'last_run': str(run)}))
    return run


def test_failed_helper_records_current_run_and_specific_reason(controller):
    node, runs = controller
    run = make_run(runs, 'water_station_current')
    node._log_path.write_text(f'SUBMITTED hold, log /pi/maps/{run.name}/cycle_events.jsonl\nSTOP: execution stopped: table_lateral_restage_not_confirmed (code=1)\n')
    node._state['parking_log_on_pi'] = '/old/run/cycle_events.jsonl'
    node._record_navigation_run(['go', 'water_station', '--park-only'], 100.)
    state = node.status()
    assert state['navigation_run'] == str(run)
    assert state['navigation_failure_reason'] == 'table_lateral_restage_not_confirmed'
    assert state['parking_log_on_pi'].endswith('/water_station_current/cycle_events.jsonl')


def test_previous_run_is_not_associated_with_new_failure(controller):
    node, runs = controller
    make_run(runs, 'water_station_old', stamp=50.)
    node._log_path.write_text('failure before submission\n')
    node._record_navigation_run(['go', 'water_station', '--park-only'], 100.)
    assert node.status().get('navigation_run') is None


@pytest.mark.parametrize("state", ["failed", "running"])
def test_failed_legacy_mission_logs_use_own_submission_not_later_manual_run(controller, monkeypatch, state):
    node, runs = controller
    node._state['state'] = state
    own = make_run(runs, 'water_station_own')
    make_run(runs, 'water_station_later_manual', stamp=150.)
    node._log_path.write_text(f'SUBMITTED hold, log /pi/maps/{own.name}/cycle_events.jsonl, request_submit_s=1\n')
    command = Mock(return_value=SimpleNamespace(returncode=0, stdout='{"event":"failed","reason":"alignment"}\n', stderr=''))
    monkeypatch.setattr('water_service.subprocess.run', command)
    result = node.logs()
    assert result['navigation_run'] == str(own)
    assert result['navigation_events'][0]['reason'] == 'alignment'
    assert command.call_args.args[0][-1].endswith('water_station_own/cycle_events.jsonl')


def test_log_read_does_not_authorize_missing_submission(controller, monkeypatch):
    node, runs = controller
    make_run(runs, 'water_station_unrelated')
    node._log_path.write_text('failure before submission\n')
    command = Mock()
    monkeypatch.setattr('water_service.subprocess.run', command)
    assert node.logs()['navigation_events'] == []
    command.assert_not_called()


def test_logs_endpoint_requires_authentication_and_only_reads():
    controller = SimpleNamespace(logs=Mock(return_value={'navigation_events': [{'event': 'failed'}]}))
    server = api._MissionServer(('127.0.0.1', 0), controller, 'test-token', None)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    address = f'http://127.0.0.1:{server.server_port}/api/mission/logs'
    try:
        with pytest.raises(HTTPError) as error:
            urlopen(address)
        assert error.value.code == 401
        controller.logs.assert_not_called()
        with urlopen(Request(address, headers={'Authorization': 'Bearer test-token'})) as response:
            assert json.load(response)['navigation_events'][0]['event'] == 'failed'
        controller.logs.assert_called_once_with()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)


def test_record_uses_submission_instead_of_later_manual_run(controller):
    node, runs = controller
    own = make_run(runs, 'water_station_own')
    make_run(runs, 'water_station_manual', stamp=150.)
    node._log_path.write_text(f'SUBMITTED hold, log /pi/{own.name}/cycle_events.jsonl\n')
    node._record_navigation_run(['go', 'water_station'], 100.)
    assert node.status()['navigation_run'] == str(own)


def test_long_running_mission_keeps_submission_after_display_tail(controller, monkeypatch):
    node, runs = controller
    own = make_run(runs, 'water_station_own')
    make_run(runs, 'water_station_manual', stamp=150.)
    node._state['state'] = 'running'
    node._log_path.write_text(
        f'SUBMITTED hold, log /pi/{own.name}/cycle_events.jsonl\n'
        + ('waiting for input ' + 'x' * 200 + '\n') * 600)
    command = Mock(return_value=SimpleNamespace(returncode=0, stdout='{"event":"waiting"}\n', stderr=''))
    monkeypatch.setattr('water_service.subprocess.run', command)
    result = node.logs()
    assert len(result['controller_lines']) == 160
    assert result['navigation_run'] == str(own)
    assert result['navigation_events'] == [{'event': 'waiting'}]
    node._record_navigation_run(['go', 'water_station'], 100.)
    assert node.status()['navigation_run'] == str(own)
