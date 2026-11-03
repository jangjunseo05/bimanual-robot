"""통합 준비가 구동 중 재시작하거나 일부 서비스만으로 성공하지 않게 한다."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

path = Path(__file__).resolve().parents[2] / 'tools/jdamr/scenario_runtime.py'
spec = importlib.util.spec_from_file_location('scenario_runtime', path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


@pytest.mark.parametrize('state', [
    {'active_order_id': 'order'},
    {'drive_link': {'state': 'running'}},
    {'drive_init': {'state': 'running'}},
])
def test_prepare_preserves_active_work(state):
    with pytest.raises(RuntimeError):
        m.ensure_idle(state)


def test_ready_arms_do_not_hide_dead_camera():
    state = {'execution_backend': {'manipulation_service_state': 'ready'},
             'drive_link': {'connected': True}, 'drive_init': {'state': 'succeeded'}}
    remote = {key: {'state': 'running', 'pid': 1} for key in ('arms', 'order')}
    remote['camera'] = {'state': 'stopped', 'pid': None}
    state['drive_init']['id'] = 'fresh'
    with patch.object(m, 'get_state', return_value=state), patch.object(m, 'windows', return_value=remote), \
            patch.object(m, 'urlopen') as opened:
        opened.return_value.__enter__.return_value.read.return_value = b'{"drive_init": {"id": "fresh"}}'
        with pytest.raises(RuntimeError, match='Windows 서비스'):
            m.wait_ready('user@host', 'http://host:8765', 5)


def test_all_roles_and_init_required_for_ready():
    state = {'execution_backend': {'manipulation_service_state': 'ready'},
             'drive_link': {'connected': True}, 'drive_init': {'state': 'succeeded'}}
    remote = {key: {'state': 'running', 'pid': 1} for key in ('arms', 'order', 'camera')}
    remote['services'] = {key: {'reachable': True} for key in ('rollout_8770', 'order_8765')}
    state['drive_init']['id'] = 'fresh'
    with patch.object(m, 'get_state', return_value=state), patch.object(m, 'windows', return_value=remote), \
            patch.object(m, 'urlopen') as opened:
        opened.return_value.__enter__.return_value.read.return_value = b'{"drive_init": {"id": "fresh"}}'
        assert m.wait_ready('user@host', 'http://host:8765', 5) == state


def test_old_init_success_is_not_reused():
    old = {'execution_backend': {'manipulation_service_state': 'ready'},
           'drive_link': {'connected': True},
           'drive_init': {'id': 'old', 'state': 'succeeded'}}
    fresh = {**old, 'drive_init': {'id': 'new', 'state': 'succeeded'}}
    remote = {key: {'state': 'running', 'pid': 1} for key in ('arms', 'order', 'camera')}
    remote['services'] = {key: {'reachable': True} for key in ('rollout_8770', 'order_8765')}
    with patch.object(m, 'get_state', side_effect=[old, fresh]) as states, \
            patch.object(m, 'windows', return_value=remote), patch.object(m, 'urlopen') as opened:
        opened.return_value.__enter__.return_value.read.return_value = b'{"drive_init": {"id": "new"}}'
        assert m.wait_ready('user@host', 'http://host:8765', 5) == fresh
        assert states.call_count == 2
        opened.assert_called_once()
