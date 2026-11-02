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
