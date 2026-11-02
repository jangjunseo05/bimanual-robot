"""Shell-mocked contract tests for the Nav2-only restaurant session wrapper."""

import os
from pathlib import Path
import re
import subprocess

import pytest


PACKAGE = Path(__file__).parents[1]
SCRIPT = PACKAGE / 'scripts/restaurant_session.sh'


def _executable(path, body):
    path.write_text('#!/usr/bin/env bash\n' + body, encoding='utf-8')
    path.chmod(0o755)


@pytest.fixture
def session_env(tmp_path):
    home = tmp_path / 'home'
    workspace = home / 'jdamr_ws'
    bin_dir = tmp_path / 'bin'
    install = workspace / 'install'
    share = install / 'jdamr_cube_navigation/share/jdamr_cube_navigation'
    registry = home / 'registry.yaml'
    ros_setup = tmp_path / 'ros_setup.bash'
    log = tmp_path / 'commands.log'
    bin_dir.mkdir()
    (share / 'config').mkdir(parents=True)
    registry.parent.mkdir(parents=True, exist_ok=True)
    registry.write_text('schema_version: 1\n', encoding='utf-8')
    params = share / 'config/new_base_nav2_params.yaml'
    params.write_text('controller_server: {}\n', encoding='utf-8')
    for contract in ('parking_contract.yaml', 'box_parking_contract.yaml'):
        (share / 'config' / contract).write_text('schema_version: 1\n', encoding='utf-8')
    ros_setup.write_text(
        'test -z "$MOCK_SETUP_UNDEFINED"\nexport MOCK_ROS_SETUP=1\n',
        encoding='utf-8',
    )
    install.mkdir(exist_ok=True)
    (install / 'setup.bash').write_text(
        'export MOCK_OVERLAY_SETUP=1\n', encoding='utf-8')

    _executable(bin_dir / 'systemctl', r"""
printf 'systemctl %s\n' "$*" >> "$MOCK_LOG"
if [ "$1" = status ]; then exit "${MOCK_STATUS_RC:-0}"; fi
if [ "$1" = stop ]; then exit 0; fi
if [ "$1" = daemon-reload ]; then touch "$MOCK_LOG.reloaded"; exit 0; fi
if [ "$1" = start ]; then
  touch "$MOCK_LOG.started"
  exit "${MOCK_START_RC:-0}"
fi
if [ "$1" = show ] && [ "${@: -1}" = --value ]; then
  if [[ "$*" == *--property=ActiveState* ]]; then
    echo "${MOCK_SHOW_ACTIVE_STATE:-inactive}"
    exit 0
  fi
  if [ -e "$MOCK_LOG.reloaded" ] && [ "${MOCK_RELEASE_FAIL:-0}" != 1 ]; then
    echo not-found
  else
    echo loaded
  fi
  exit 0
fi
if [ "$1" = show ]; then
  active_state="${MOCK_SHOW_ACTIVE_STATE:-active}"
  sub_state="${MOCK_SHOW_SUB_STATE:-running}"
  if [ -e "$MOCK_LOG.started" ]; then
    active_state="${MOCK_SHOW_ACTIVE_STATE_AFTER_START:-active}"
    sub_state="${MOCK_SHOW_SUB_STATE_AFTER_START:-running}"
  fi
  printf 'LoadState=%s\nActiveState=%s\nSubState=%s\nFragmentPath=%s\nEnvironment=%s\n' \
    "${MOCK_SHOW_LOAD_STATE:-not-found}" \
    "$active_state" \
    "$sub_state" \
    "${MOCK_SHOW_FRAGMENT_PATH:-}" \
    "${MOCK_UNIT_ENV:-}"
  exit "${MOCK_SHOW_RC:-0}"
fi
if [ "$1" = is-active ]; then
  service="${@: -1}"
  case " ${MOCK_ACTIVE_SERVICES:-jdamr-base.service} " in
    *" $service "*) exit 0 ;;
    *) exit 3 ;;
  esac
fi
exit 0
""")
    _executable(bin_dir / 'ros2', r"""
printf 'ros2 %s\n' "$*" >> "$MOCK_LOG"
if [ "$1 $2 $3" = "node list --no-daemon" ]; then
  printf '%b' "${MOCK_NODES:-/robot_state_publisher\\n/base_driver\\n}"
elif [ "$1 $2 $3" = "topic list --no-daemon" ]; then
  if [ "${MOCK_TOPICS_DELAYED:-0}" = 1 ] && [ ! -e "$MOCK_LOG.topics_seen" ]; then
    touch "$MOCK_LOG.topics_seen"
    printf '/odom\n/tf\n'
    exit 0
  fi
  printf '%b' "${MOCK_TOPICS:-/scan\\n/odom\\n/tf\\n}"
elif [ "$1" = launch ]; then
  printf 'launch-env %s %s %s %s\n' "$ROS_DOMAIN_ID" \
    "$ROS_AUTOMATIC_DISCOVERY_RANGE" "$FASTDDS_BUILTIN_TRANSPORTS" \
    "${MOCK_OVERLAY_SETUP:-}" >> "$MOCK_LOG"
  printf 'localhost %s\n' "${ROS_LOCALHOST_ONLY:-unset}" >> "$MOCK_LOG"
fi
""")
    _executable(bin_dir / 'python3', r"""
printf 'python3 %s\n' "$*" >> "$MOCK_LOG"
exit "${MOCK_REGISTRY_RC:-0}"
""")
    _executable(bin_dir / 'systemd-run', r"""
printf 'systemd-run %s\n' "$*" >> "$MOCK_LOG"
exit "${MOCK_SYSTEMD_RUN_RC:-0}"
""")
    _executable(bin_dir / 'cp', r"""
printf 'cp %s\n' "$*" >> "$MOCK_LOG"
exit 0
""")
    _executable(bin_dir / 'rm', r"""
printf 'rm %s\n' "$*" >> "$MOCK_LOG"
exit 0
""")
    _executable(bin_dir / 'sudo', r"""
printf 'sudo %s\n' "$*" >> "$MOCK_LOG"
[ "$1" = -n ] && shift
exec "$@"
""")

    env = os.environ.copy()
    env.update({
        'HOME': str(home),
        'PATH': f'{bin_dir}:{env["PATH"]}',
        'JDAMR_ROS_SETUP': str(ros_setup),
        'MOCK_LOG': str(log),
        'TMPDIR': str(tmp_path),
    })
    return {
        'env': env, 'home': home, 'workspace': workspace,
        'registry': registry, 'params': params, 'log': log,
    }


def _run(session_env, *args, **updates):
    env = session_env['env'].copy()
    env.update(updates)
    return subprocess.run(
        ['/bin/bash', str(SCRIPT), *map(str, args)], env=env,
        text=True, capture_output=True, check=False,
    )


def _start(session_env, **updates):
    return _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], **updates,
    )


def test_help_states_nav2_only_and_readiness_boundary(session_env):
    result = _run(session_env, '--help')
    assert result.returncode == 0
    assert 'Nav2 서버만 시작' in result.stdout
    assert '초기 pose, NavigateToPose, FollowPath' in result.stdout
    assert '준비 완료를 뜻하지 않는다' in result.stdout
    assert '--precision-parking' in result.stdout
    assert '5 cm 박스 주차 시험' in result.stdout
    assert '--use-composition true|false' in result.stdout
    assert '비교 진단용 폴백' in result.stdout
    assert not session_env['log'].exists()


def test_start_uses_singleton_unit_and_composed_nav2_by_default(session_env):
    result = _start(session_env)
    assert result.returncode == 0, result.stderr
    assert '기동 요청을 수락했다' in result.stdout
    assert '주행 준비 완료가 아니다' in result.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert '--unit=jdamr-restaurant-navigation.service --collect' in commands
    assert '__run' in commands
    assert '--workspace ' + str(session_env['workspace']) in commands
    assert '--use-composition true' in commands
    assert '--precision-parking' not in commands
    assert 'NavigateToPose' not in commands
    assert 'FollowPath' not in commands


def test_internal_run_sources_overlay_and_launches_servers_without_action(session_env):
    result = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], JDAMR_RESTAURANT_INTERNAL='1',
    )
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'ros2 launch jdamr_cube_navigation restaurant_service.launch.py' in commands
    assert 'use_composition:=true' in commands
    assert 'coordinated_startup:=true' in commands
    assert 'precision_parking:=false' in commands
    assert '/config/parking_contract.yaml' in commands
    assert '/config/box_parking_contract.yaml' not in commands
    assert 'use_box_observer:=false' in commands
    assert 'discovery_range:=LOCALHOST' in commands
    assert 'launch-env 12 LOCALHOST UDPv4 1' in commands
    assert 'initial_pose' not in commands


def test_precision_parking_is_explicitly_propagated_and_uses_box_contract(
        session_env):
    started = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--precision-parking')
    assert started.returncode == 0, started.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert '__run ' in commands
    assert '--precision-parking' in commands

    session_env['log'].write_text('', encoding='utf-8')
    internal = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--precision-parking',
        JDAMR_RESTAURANT_INTERNAL='1')
    assert internal.returncode == 0, internal.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'precision_parking:=true' in commands
    assert '/config/box_parking_contract.yaml' in commands
    assert '/config/parking_contract.yaml' not in commands


@pytest.mark.parametrize(
    ('updates', 'message'), [
        ({'MOCK_ACTIVE_SERVICES':
          'jdamr-base.service jdamr-cartographer-session.service'},
         'jdamr-cartographer-session.service 실행 중'),
        ({'MOCK_NODES': '/robot_state_publisher\\n/amcl\\n'},
         '기존 localization/Cartographer/Nav2 노드'),
        ({'MOCK_TOPICS': '/scan\\n/tf\\n'},
         '센서 진단 실패'),
    ],
)
def test_start_fails_closed_without_stopping_conflicts(
        session_env, updates, message):
    result = _start(session_env, **updates)
    assert result.returncode == 4
    assert message in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl stop jdamr-cartographer-session.service' not in commands
    assert 'systemd-run ' not in commands


def test_start_requires_active_base_service(session_env):
    result = _start(session_env, MOCK_ACTIVE_SERVICES='none.service')
    assert result.returncode == 4
    assert 'jdamr-base.service가 active가 아니다' in result.stderr


def test_start_rejects_existing_singleton_unit(session_env):
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES=(
            'jdamr-base.service jdamr-restaurant-navigation.service'),
    )
    assert result.returncode == 4
    assert '실행 설정을 확인할 수 없거나 요청과 다르다' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands


def _started_identity(session_env, *args):
    result = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], *args,
    )
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    match = re.search(r'--setenv=(JDAMR_RESTAURANT_SESSION_ID=[a-f0-9]{64})', commands)
    assert match is not None
    session_env['log'].write_text('', encoding='utf-8')
    return match.group(1)


def test_repeated_start_reuses_matching_active_session_without_restart(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV='HOME=/tmp ' + identity,
    )
    assert result.returncode == 0, result.stderr
    assert '같은 설정으로 실행 중' in result.stdout
    assert '주행 준비 완료 판정이 아니다' in result.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands
    assert 'ros2 ' not in commands


def test_repeated_start_reuses_matching_inactive_transient_unit(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_SHOW_LOAD_STATE='loaded',
        MOCK_SHOW_ACTIVE_STATE='inactive',
        MOCK_SHOW_SUB_STATE='dead',
        MOCK_SHOW_FRAGMENT_PATH=(
            '/run/systemd/transient/jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV='HOME=/tmp ' + identity,
    )
    assert result.returncode == 0, result.stderr
    assert '같은 설정의 중지된 transient unit을 다시 시작했다' in result.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl start jdamr-restaurant-navigation.service' in commands
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_inactive_transient_unit_with_changed_identity_is_recreated(session_env):
    identity = _started_identity(session_env)
    session_env['params'].write_text(
        'controller_server: {changed: true}\n', encoding='utf-8')
    result = _start(
        session_env,
        MOCK_SHOW_LOAD_STATE='loaded',
        MOCK_SHOW_ACTIVE_STATE='inactive',
        MOCK_SHOW_SUB_STATE='dead',
        MOCK_SHOW_FRAGMENT_PATH=(
            '/run/systemd/transient/jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV=identity + " JDAMR_RESTAURANT_INTERNAL=1",
    )
    assert result.returncode == 0, result.stderr
    assert '새 설정으로 생성한다' in result.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl start ' not in commands
    assert 'systemd-run ' in commands
    assert 'systemctl stop ' in commands
    assert commands.index('cp -- /run/systemd/transient/') < commands.index('systemctl stop ')



@pytest.mark.parametrize('owned,release_fails', [(False, False), (True, True)])
def test_changed_inactive_session_requires_ownership_and_release(
        session_env, owned, release_fails):
    identity = _started_identity(session_env)
    session_env['params'].write_text('changed: true\n', encoding='utf-8')
    result = _start(
        session_env, MOCK_SHOW_LOAD_STATE='loaded',
        MOCK_SHOW_ACTIVE_STATE='inactive', MOCK_SHOW_SUB_STATE='dead',
        MOCK_SHOW_FRAGMENT_PATH=(
            '/run/systemd/transient/jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV=identity + (
            ' JDAMR_RESTAURANT_INTERNAL=1' if owned else ''),
        MOCK_RELEASE_FAIL='1' if release_fails else '0',
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl start ' not in commands
    if not owned:
        assert 'systemctl stop ' not in commands


def test_matching_inactive_persistent_unit_is_not_reused(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_SHOW_LOAD_STATE='loaded',
        MOCK_SHOW_ACTIVE_STATE='inactive',
        MOCK_SHOW_SUB_STATE='dead',
        MOCK_SHOW_FRAGMENT_PATH=(
            '/etc/systemd/system/jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    assert 'transient unit인지 확인할 수 없다' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl start ' not in commands
    assert 'systemd-run ' not in commands


def test_matching_inactive_unit_still_requires_clean_ros_graph(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_SHOW_LOAD_STATE='loaded',
        MOCK_SHOW_ACTIVE_STATE='inactive',
        MOCK_SHOW_SUB_STATE='dead',
        MOCK_SHOW_FRAGMENT_PATH=(
            '/run/systemd/transient/jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV=identity,
        MOCK_NODES='/robot_state_publisher\n/controller_server\n',
    )
    assert result.returncode == 4
    assert '기존 localization/Cartographer/Nav2 노드' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl start ' not in commands
    assert 'systemd-run ' not in commands


def test_matching_active_session_does_not_bypass_conflicting_mapping_service(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES=(
            'jdamr-base.service jdamr-restaurant-navigation.service '
            'jdamr-cartographer-session.service'),
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    assert 'jdamr-cartographer-session.service 실행 중' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_active_session_with_changed_params_is_not_reused(session_env):
    identity = _started_identity(session_env)
    session_env['params'].write_text('controller_server: {changed: true}\n', encoding='utf-8')
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_active_session_with_changed_registry_is_not_reused(session_env):
    identity = _started_identity(session_env)
    session_env['registry'].write_text('schema_version: 1\nhome: changed\n', encoding='utf-8')
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_active_session_with_changed_parking_contract_is_not_reused(session_env):
    identity = _started_identity(session_env)
    contract = session_env['params'].parent / 'parking_contract.yaml'
    contract.write_text('schema_version: 1\nxy_tolerance_m: 0.01\n', encoding='utf-8')
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_active_session_show_failure_never_restarts(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity, MOCK_SHOW_RC='1',
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_active_session_stopped_during_inspection_is_not_reused(session_env):
    identity = _started_identity(session_env)
    result = _start(
        session_env,
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity, MOCK_SHOW_ACTIVE_STATE='inactive',
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_precision_request_does_not_reuse_normal_active_session(session_env):
    identity = _started_identity(session_env)
    result = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--precision-parking',
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity,
    )
    assert result.returncode == 4
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


def test_composition_selection_survives_systemd_dispatch_and_internal_run(
        session_env):
    started = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--use-composition', 'false')
    assert started.returncode == 0, started.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert '__run ' in commands
    assert '--use-composition false' in commands

    session_env['log'].write_text('', encoding='utf-8')
    internal = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--use-composition', 'false',
        JDAMR_RESTAURANT_INTERNAL='1')
    assert internal.returncode == 0, internal.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'use_composition:=false' in commands


def test_composition_selection_changes_session_identity(session_env):
    composed_identity = _started_identity(session_env)
    standalone_identity = _started_identity(
        session_env, '--use-composition', 'false')
    assert standalone_identity != composed_identity


def test_active_session_with_different_composition_is_not_reused(session_env):
    composed_identity = _started_identity(session_env)
    result = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--use-composition', 'false',
        MOCK_ACTIVE_SERVICES=(
            'jdamr-base.service jdamr-restaurant-navigation.service'),
        MOCK_UNIT_ENV=composed_identity,
    )
    assert result.returncode == 4
    assert '요청과 다르다' in result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


@pytest.mark.parametrize('value', ['', 'yes', 'TRUE', '0'])
def test_composition_selection_rejects_invalid_value(session_env, value):
    args = ['start', '--workspace', session_env['workspace'],
            '--registry', session_env['registry'], '--params-file',
            session_env['params'], '--use-composition']
    if value:
        args.append(value)
    result = _run(session_env, *args)
    assert result.returncode == 4
    assert '--use-composition' in result.stderr
    assert not session_env['log'].exists()


def test_transport_default_overrides_inherited_environment(session_env):
    result = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], JDAMR_RESTAURANT_INTERNAL='1',
        FASTDDS_BUILTIN_TRANSPORTS='DEFAULT')
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'launch-env 12 LOCALHOST UDPv4 1' in commands


def test_internal_run_clears_inherited_localhost_isolation(session_env):
    result = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], JDAMR_RESTAURANT_INTERNAL='1', ROS_LOCALHOST_ONLY='1',
    )
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'localhost 0' in commands


def test_prepare_only_starts_localization_without_navigation_activation(session_env):
    result = _run(
        session_env, '__run', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--prepare-only',
        JDAMR_RESTAURANT_INTERNAL='1')
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'coordinated_startup:=false' in commands
    assert 'navigation_autostart:=false' in commands
    assert 'NavigateToPose' not in commands
    assert 'FollowPath' not in commands


@pytest.mark.parametrize('scope', ['LOCALHOST', 'SUBNET'])
def test_discovery_selection_survives_dispatch_and_internal_run(session_env, scope):
    arguments = ('--workspace', session_env['workspace'], '--registry',
                 session_env['registry'], '--params-file', session_env['params'],
                 '--discovery-range', scope)
    started = _run(session_env, 'start', *arguments)
    assert started.returncode == 0, started.stderr
    assert '--discovery-range ' + scope in session_env['log'].read_text()
    internal = _run(session_env, '__run', *arguments,
                    JDAMR_RESTAURANT_INTERNAL='1', ROS_LOCALHOST_ONLY='1')
    assert internal.returncode == 0, internal.stderr
    commands = session_env['log'].read_text()
    assert 'discovery_range:=' + scope in commands
    assert f'launch-env 12 {scope} UDPv4 1' in commands
    assert 'localhost 0' in commands


def test_discovery_mismatch_does_not_reuse_active_session(session_env):
    identity = _started_identity(session_env)
    result = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--discovery-range', 'SUBNET',
        MOCK_ACTIVE_SERVICES='jdamr-base.service jdamr-restaurant-navigation.service',
        MOCK_UNIT_ENV=identity)
    assert result.returncode == 4
    assert '요청과 다르다' in result.stderr
    commands = session_env['log'].read_text()
    assert 'systemd-run ' not in commands
    assert 'systemctl stop ' not in commands


@pytest.mark.parametrize('value', ['', 'local', 'localHost', 'ALL'])
def test_discovery_selection_rejects_invalid_values(session_env, value):
    args = ['start', '--discovery-range']
    if value:
        args.append(value)
    result = _run(session_env, *args)
    assert result.returncode == 4
    assert '--discovery-range' in result.stderr
    assert not session_env['log'].exists()


def test_prepare_flag_survives_systemd_dispatch(session_env):
    result = _run(
        session_env, 'start', '--workspace', session_env['workspace'],
        '--registry', session_env['registry'], '--params-file',
        session_env['params'], '--prepare-only')
    assert result.returncode == 0, result.stderr
    assert '--prepare-only' in session_env['log'].read_text(encoding='utf-8')


def test_missing_first_discovery_retries_before_rejecting_sensor(session_env):
    result = _start(session_env, MOCK_TOPICS_DELAYED='1')
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'topic list --no-daemon --spin-time 5' in commands
    assert sum(line.startswith('systemd-run --unit=')
               for line in commands.splitlines()) == 1


def test_registry_validation_failure_prevents_start(session_env):
    result = _start(session_env, MOCK_REGISTRY_RC='1')
    assert result.returncode == 4
    assert (
        'registry 스키마·지도·keepout 경로 또는 해시 검증 실패'
        in result.stderr
    )
    assert 'systemd-run ' not in session_env['log'].read_text(encoding='utf-8')


def test_overlay_is_required_and_checked_before_ros_queries(session_env):
    (session_env['workspace'] / 'install/setup.bash').unlink()
    result = _start(session_env)
    assert result.returncode == 4
    assert 'workspace overlay를 읽을 수 없다' in result.stderr
    assert not session_env['log'].exists()


def test_overlay_source_failure_is_reported(session_env):
    (session_env['workspace'] / 'install/setup.bash').write_text(
        'return 1\n', encoding='utf-8')
    result = _start(session_env)
    assert result.returncode == 4
    assert 'workspace overlay 적용 실패' in result.stderr
    assert not session_env['log'].exists()


def test_status_and_stop_target_only_navigation_unit(session_env):
    status = _run(session_env, 'status')
    stop = _run(session_env, 'stop')
    assert status.returncode == 0
    assert '준비 완료 판정이 아니다' in status.stdout
    assert stop.returncode == 0
    assert '베이스·센서 서비스는 변경하지 않았다' in stop.stdout
    commands = session_env['log'].read_text(encoding='utf-8')
    assert 'systemctl status --no-pager jdamr-restaurant-navigation.service' in commands
    assert 'systemctl stop jdamr-restaurant-navigation.service' in commands
    assert 'systemctl stop jdamr-base.service' not in commands


def test_transport_profile_is_forwarded_and_changes_identity(session_env):
    profile = session_env['home'] / 'dds.xml'
    profile.write_text('<profiles/>', encoding='utf-8')
    result = _start(session_env, FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
    assert result.returncode == 0, result.stderr
    commands = session_env['log'].read_text()
    assert '--setenv=FASTRTPS_DEFAULT_PROFILES_FILE=' + str(profile) in commands
    identity = re.search(r'JDAMR_RESTAURANT_SESSION_ID=([0-9a-f]+)', commands)[1]
    session_env['log'].write_text('')
    profile.write_text('<profiles><!-- changed --></profiles>', encoding='utf-8')
    result = _start(session_env, FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
    assert result.returncode == 0, result.stderr
    changed = re.search(
        r'JDAMR_RESTAURANT_SESSION_ID=([0-9a-f]+)', session_env['log'].read_text())[1]
    assert changed != identity
