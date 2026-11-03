"""The drive PC departs only for a started order and obeys the control page STOP."""

import importlib.util
from pathlib import Path
import threading
from unittest.mock import Mock
from urllib.request import Request, urlopen

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'tools/jdamr/water_service.py'
SPEC = importlib.util.spec_from_file_location('water_service_follow', SOURCE)
water_service = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(water_service)


@pytest.fixture
def controller(tmp_path):
    node = water_service.WaterServiceController(
        inference_url='http://order.invalid', state_directory=tmp_path)
    node.start = Mock()
    node.stop = Mock()
    return node


def order_state(order_id='A', arms_ready=True, station='kitchen', emergency=False):
    return {
        'emergency_stop': {'active': emergency, 'at': 't1' if emergency else None},
        'execution_backend': {
            'drive': {'arms_ready': arms_ready, 'next': station},
            'waiting_alignment': {'station': station, 'order_id': order_id},
        },
    }


def mark_running(node):
    release = threading.Event()
    node._worker_thread = threading.Thread(target=release.wait, daemon=True)
    node._worker_thread.start()
    return release


def test_started_order_departs_once(controller):
    assert controller.follow_web(order_state()) == 'start A'
    controller.start.assert_called_once_with('web-A')
    controller._state['requests']['web-A'] = {'state': 'succeeded'}
    assert controller.follow_web(order_state()) is None
    controller.start.assert_called_once()


@pytest.mark.parametrize('state', [
    {},
    order_state(arms_ready=False),
    order_state(station='table'),
    order_state(order_id=None),
])
def test_nothing_departs_without_the_folded_arms_signal(controller, state):
    assert controller.follow_web(state) is None
    controller.start.assert_not_called()


def test_running_mission_is_not_started_again(controller):
    release = mark_running(controller)
    try:
        assert controller.follow_web(order_state()) is None
        controller.start.assert_not_called()
    finally:
        release.set()


def test_stop_ends_a_running_mission(controller):
    release = mark_running(controller)
    try:
        assert controller.follow_web(order_state(emergency=True)) == 'stop'
        controller.stop.assert_called_once()
    finally:
        release.set()


def test_stop_outranks_a_waiting_departure(controller):
    assert controller.follow_web(order_state(emergency=True)) is None
    controller.start.assert_not_called()


def test_dry_run_reports_once_and_touches_nothing(controller):
    assert controller.follow_web(order_state(), dry_run=True) == 'would_start A'
    assert controller.follow_web(order_state(), dry_run=True) is None
    assert controller.follow_web(order_state(emergency=True), dry_run=True) == 'would_stop'
    assert controller.follow_web(order_state(emergency=True), dry_run=True) is None
    controller.start.assert_not_called()
    controller.stop.assert_not_called()


def test_driving_mission_stops_on_emergency_even_after_the_order_succeeded(controller):
    state = {'emergency_stop': {'active': True},
             'orders': [{'order_id': 'A', 'state': 'SUCCEEDED'}], 'execution_backend': {}}
    controller._state['order_id'] = 'A'
    with pytest.raises(RuntimeError, match='emergency stop'):
        controller._drive_ready(state, 'dock')


def test_rehearsal_service_folds_and_stops_without_hardware():
    source = Path(__file__).resolve().parents[1] / 'tools/rollout/rehearsal_service.py'
    spec = importlib.util.spec_from_file_location('rehearsal_service', source)
    rehearsal = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rehearsal)
    service = rehearsal.Rehearsal()
    server = rehearsal.Server(('127.0.0.1', 0), rehearsal.handler_factory(service))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        url = f'http://127.0.0.1:{server.server_port}/stow'
        with urlopen(Request(url, method='POST'), timeout=5) as response:
            assert response.status == 200
        assert service.arms == 'stowed'
    finally:
        server.shutdown()
        server.server_close()


def init_state(request_id='i1', state='requested', **order):
    return {**order_state(**order), 'drive_init': {'id': request_id, 'state': state}}


@pytest.fixture
def helper(tmp_path, monkeypatch):
    """Replace jdamr_depart.py; it records its arguments and ends with the given line and code."""
    calls = tmp_path / 'calls.txt'

    def install(last_line, code=0):
        script = tmp_path / 'jdamr_depart.py'
        script.write_text(
            'import sys\n'
            f'open({str(calls)!r}, "a").write(" ".join(sys.argv[1:]) + "\\n")\n'
            f'print({last_line!r})\n'
            f'raise SystemExit({code})\n')
        monkeypatch.setattr(water_service, 'DEPART_HELPER', script)
        return calls
    return install


def finish_init(node):
    node._init_thread.join(timeout=10)
    assert not node._init_thread.is_alive()
    return node._init_report


def test_requested_init_runs_once_and_reports_success(controller, helper):
    calls = helper('init complete: robot localized')
    assert controller.follow_web(init_state()) == 'init i1'
    assert finish_init(controller) == {
        'id': 'i1', 'state': 'succeeded', 'message': 'init complete: robot localized'}
    assert calls.read_text().split()[-1] == 'init'
    assert controller.follow_web(init_state()) is None
    assert len(calls.read_text().splitlines()) == 1


def test_init_failure_reports_the_stop_line(controller, helper):
    helper('STOP: activation failed', code=1)
    controller.follow_web(init_state())
    assert finish_init(controller) == {
        'id': 'i1', 'state': 'failed', 'message': 'STOP: activation failed'}


def test_init_result_goes_out_with_the_heartbeat(controller, helper):
    helper('init complete')
    controller.follow_web(init_state())
    finish_init(controller)
    controller._http_json = Mock()
    controller.report_web(dry_run=False)
    assert controller._http_json.call_args.kwargs['body']['init']['state'] == 'succeeded'


def test_init_does_not_run_during_a_mission(controller, helper):
    calls = helper('init complete')
    release = mark_running(controller)
    try:
        assert controller.follow_web(init_state()) is None
    finally:
        release.set()
    assert controller._init_thread is None
    assert not calls.exists()


def test_stop_outranks_a_requested_init(controller, helper):
    helper('init complete')
    assert controller.follow_web(init_state(emergency=True)) is None
    assert controller._init_thread is None


def test_dry_run_reports_init_and_runs_nothing(controller, helper):
    helper('init complete')
    assert controller.follow_web(init_state(), dry_run=True) == 'would_init i1'
    assert controller.follow_web(init_state(), dry_run=True) is None
    assert controller._init_thread is None


def test_departure_waits_for_a_running_init(controller):
    release = threading.Event()
    controller._init_thread = threading.Thread(target=release.wait, daemon=True)
    controller._init_thread.start()
    try:
        assert controller.follow_web(order_state()) is None
        controller.start.assert_not_called()
    finally:
        release.set()


def test_mission_start_is_refused_during_init(tmp_path):
    pointer = tmp_path / 'map.json'
    pointer.write_text('{}')
    node = water_service.WaterServiceController(
        inference_url='http://order.invalid', state_directory=tmp_path / 'state', map_session=pointer)
    release = threading.Event()
    node._init_thread = threading.Thread(target=release.wait, daemon=True)
    node._init_thread.start()
    try:
        with pytest.raises(RuntimeError, match='init is running'):
            node.start('web-A')
    finally:
        release.set()


def test_failed_manual_order_is_not_restarted_by_follow_web(controller):
    controller._state.update(order_id='A', request_id='manual-A', state='failed')
    controller._record_request_terminal()
    assert controller._state['requests']['manual-A']['order_id'] == 'A'
    controller._state['order_id'] = None
    assert controller.follow_web(order_state('A')) is None
    controller.start.assert_not_called()
    assert controller.follow_web(order_state('B')) == 'start B'
