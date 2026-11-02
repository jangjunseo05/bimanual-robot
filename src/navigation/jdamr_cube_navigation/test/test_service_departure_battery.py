"""Exercise departure reserve independently from the running cutoff."""

import io
import json
from pathlib import Path
from unittest.mock import Mock

from jdamr_cube_navigation import corridor_route
from jdamr_cube_navigation.restaurant_service import load_service_contract, ServiceRoute
import pytest
from rclpy.task import Future
import yaml


CONTRACT = Path(__file__).resolve().parents[1] / 'config/restaurant_service_contract.yaml'


def readiness_node(monkeypatch, *, guard=None, protection=None):
    """Exercise the real readiness methods with a bounded simulated clock."""
    clock = [0.0]
    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])

    def spin(*_args, **_kwargs):
        clock[0] += .1

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    node = object.__new__(ServiceRoute)
    node.stop_requested = False
    node.navigation_profile = 'new_base_revisit_candidate'
    node._guard_failure = Mock(return_value=guard)
    node._revisit_protection_ready = Mock(return_value=protection)
    node.travel_pose_status = None
    node.get_logger = Mock(return_value=Mock())
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.battery_voltage = 12.0
    node.emit = Mock()
    return node


@pytest.mark.parametrize('reason', [
    'battery low: voltage=10.192V limit=10.500V',
    'scan stale: age=2.000s limit=1.000s',
    'AMCL pose missing',
])
def test_readiness_preserves_guard_failure(monkeypatch, reason):
    node = readiness_node(monkeypatch, guard=reason)
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == reason
    node.get_logger().error.assert_called_once_with(
        'navigation readiness timeout: ' + reason)


def test_readiness_preserves_protection_failure(monkeypatch):
    reason = '/cmd_vel publisher must be collision_monitor only'
    node = readiness_node(monkeypatch, protection=reason)
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == reason


@pytest.mark.parametrize('new_pose,ready', [(True, True), (False, False)])
def test_readiness_requires_new_amcl_after_stationary_nomotion_update(
        monkeypatch, new_pose, ready):
    clock = [0.0]
    node = readiness_node(monkeypatch)
    node.navigation_profile = 'corridor'
    node.start_check_pending = True
    node.latest_motion = (clock[0], 0.0, 0.0)
    node.amcl_seen = None
    node._guard_failure.side_effect = (
        lambda *_args, **_kwargs:
        'AMCL pose missing' if node.amcl_seen is None else None)
    future = Future()
    future.set_result(object())
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    node.create_client = Mock(return_value=client)
    node.destroy_client = Mock()

    def spin(*_args, **_kwargs):
        clock[0] += .05
        if new_pose and client.call_async.called:
            node.amcl_seen = clock[0]

    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(corridor_route, 'spin_node', spin)

    assert node.wait_until_ready(timeout=.5) is ready
    assert node.start_check_pending is (not ready)
    assert node.create_client.call_args.args[1] == '/request_nomotion_update'
    node.destroy_client.assert_called_once_with(client)


def test_readiness_does_not_request_nomotion_update_while_moving(monkeypatch):
    node = readiness_node(monkeypatch, guard='AMCL pose missing')
    node.navigation_profile = 'corridor'
    node.latest_motion = (0.0, 0.02, 0.0)
    node.create_client = Mock()

    assert not node.wait_until_ready(timeout=.2)
    node.create_client.assert_not_called()
    assert node.readiness_failure_reason() == 'AMCL pose missing'


def test_readiness_defers_nomotion_update_until_odometry_stops(monkeypatch):
    clock = [0.0]
    node = readiness_node(monkeypatch, guard='AMCL pose missing')
    node.navigation_profile = 'corridor'
    node.latest_motion = (clock[0], 0.02, 0.0)
    node.amcl_seen = None
    future = Future()
    future.set_result(object())
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    node.create_client = Mock(return_value=client)
    node.destroy_client = Mock()

    def spin(*_args, **_kwargs):
        clock[0] += .05
        if clock[0] >= .15:
            node.latest_motion = (clock[0], 0.0, 0.0)
        if client.call_async.called:
            node.amcl_seen = clock[0]
            node._guard_failure.return_value = None

    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(corridor_route, 'spin_node', spin)

    assert node.wait_until_ready(timeout=.5)
    client.call_async.assert_called_once()


@pytest.mark.parametrize('latest_motion', [
    (0.1, 0.02, 0.0),
    (-1.0, 0.0, 0.0),
])
def test_readiness_rejects_nomotion_refresh_if_odometry_changes_after_request(
        monkeypatch, latest_motion):
    clock = [0.0]
    node = readiness_node(monkeypatch)
    node.navigation_profile = 'corridor'
    node.latest_motion = (clock[0], 0.0, 0.0)
    node.amcl_seen = None
    node._guard_failure.side_effect = (
        lambda *_args, **_kwargs:
        'AMCL pose missing' if node.amcl_seen is None else None)
    future = Future()
    future.set_result(object())
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    node.create_client = Mock(return_value=client)
    node.destroy_client = Mock()

    def spin(*_args, **_kwargs):
        clock[0] += .1
        if client.call_async.called:
            node.latest_motion = latest_motion
            node.amcl_seen = clock[0]

    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(corridor_route, 'spin_node', spin)

    assert not node.wait_until_ready(timeout=.7)
    client.call_async.assert_called_once()
    assert node.readiness_failure_reason() == 'navigation readiness timeout'


def test_readiness_retries_after_motion_and_requires_another_new_amcl_pose(
        monkeypatch):
    clock = [0.0]
    node = readiness_node(monkeypatch)
    node.navigation_profile = 'corridor'
    node.start_check_pending = True
    node.latest_motion = (clock[0], 0.0, 0.0)
    node.amcl_seen = None
    node._guard_failure.side_effect = (
        lambda *_args, **_kwargs:
        'AMCL pose missing' if node.amcl_seen is None else None)
    futures = []
    client = Mock()
    client.wait_for_service.return_value = True

    def request(_request):
        future = Future()
        future.set_result(object())
        futures.append(future)
        return future

    client.call_async.side_effect = request
    node.create_client = Mock(return_value=client)
    node.destroy_client = Mock()
    movement_injected = [False]

    def spin(*_args, **_kwargs):
        clock[0] += .05
        if len(futures) == 1 and not movement_injected[0]:
            node.latest_motion = (clock[0], 0.02, 0.0)
            node.amcl_seen = clock[0]
            movement_injected[0] = True
        elif len(futures) < 2:
            node.latest_motion = (clock[0], 0.0, 0.0)
        elif len(futures) == 2:
            node.latest_motion = (clock[0], 0.0, 0.0)
            node.amcl_seen = clock[0]

    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(corridor_route, 'spin_node', spin)

    assert node.wait_until_ready(timeout=.7)
    assert client.call_async.call_count == 2


def test_readiness_stop_takes_priority_and_success_clears_old_failure(monkeypatch):
    node = readiness_node(monkeypatch, guard='battery low: voltage=10.192V limit=10.500V')
    node.stop_requested = True
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == 'operator stop'
    node.stop_requested = False
    node._guard_failure.return_value = None
    assert node.wait_until_ready(timeout=.2)
    assert node.last_readiness_failure is None


@pytest.mark.parametrize('voltage,expected', [
    (10.6, 'battery departure reserve: voltage=10.600V limit=10.800V'),
    (None, 'battery voltage missing'),
    (float('nan'), 'battery voltage non-finite: value=nan'),
])
def test_readiness_reports_departure_reserve_after_running_guard_passes(
        monkeypatch, voltage, expected):
    node = readiness_node(monkeypatch)
    node.battery_voltage = voltage
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == expected


def test_readiness_preserves_lost_operator_link(monkeypatch, tmp_path):
    node = readiness_node(monkeypatch)
    node.operator_heartbeat = tmp_path / 'missing-heartbeat'
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == 'operator_link_lost'


@pytest.mark.parametrize('voltage_v,ready', [
    (None, False), (float('nan'), False), (float('inf'), False),
    (10.464, False), (10.6, False), (10.8, True), (12.0, True),
])
def test_departure_reserve(voltage_v, ready):
    """A near-cutoff idle reading must not authorize a new mission."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.battery_voltage = voltage_v
    node.emit = Mock()
    assert node._departure_battery_ready() is ready
    assert node.minimum_battery_v == 10.5
    if not ready:
        assert node.emit.call_args.kwargs['reason'] == 'battery_departure_reserve'
        json.dumps(node.emit.call_args.kwargs, allow_nan=False)


def test_reserve_is_not_reapplied_between_steps_but_running_cutoff_remains():
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = 10.5
    node.emit = Mock()
    node.battery_voltage = 11.2
    assert node._departure_battery_ready()
    node._mission_started = True
    node.battery_voltage = 10.7
    assert node._departure_battery_ready()
    node.battery_voltage = 10.4
    assert not node._departure_battery_ready()


@pytest.mark.parametrize('method', ['execute', '_execute_reverse_path'])
def test_low_reserve_never_dispatches_forward_or_reverse(method):
    """Both action entrypoints reject low reserve before send_goal_async."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = 10.5
    node.battery_voltage = 10.6
    node.emit = Mock()
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node.navigate, node.follow_reverse = Mock(), Mock()
    node.waypoints = [{'id': 'home'}]
    node.config = {}
    node.result_stream = io.StringIO()
    assert not getattr(node, method)(*([None] if method.startswith('_') else []))
    node.navigate.send_goal_async.assert_not_called()
    node.follow_reverse.send_goal_async.assert_not_called()


@pytest.mark.parametrize('value', [None, True, float('nan'), -1.0, 10.4])
def test_invalid_departure_contract(tmp_path, value):
    """Reject missing, nonfinite and weaker-than-running reserve settings."""
    document = yaml.safe_load(CONTRACT.read_text())
    document['minimum_start_battery_v'] = value
    path = tmp_path / 'contract.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_service_contract(path)
