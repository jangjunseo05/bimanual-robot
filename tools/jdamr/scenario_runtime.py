#!/usr/bin/env python3
"""주행 PC에서 Windows 서비스와 시나리오 준비를 관리한다. prepare는 주문하지 않는다."""
from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
REMOTE_SCRIPT = Path(__file__).with_name('windows_runtime.ps1')
UNIT = 'jdamr-water-service-api.service'


def powershell(peer: str, script: str) -> str:
    encoded = base64.b64encode(script.encode('utf-16-le')).decode('ascii')
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', peer,
         'powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass -EncodedCommand ' + encoded],
        capture_output=True, timeout=45)
    output = result.stdout.decode('utf-8-sig', errors='replace').strip()
    if result.returncode:
        raise RuntimeError(output or result.stderr.decode('utf-8', errors='replace'))
    return output


def install_remote(peer: str) -> None:
    powershell(peer, "New-Item -ItemType Directory -Force ($HOME+'\\jdamr_remote') | Out-Null")
    subprocess.run(['scp', '-q', str(REMOTE_SCRIPT), peer + ':jdamr_remote/windows_runtime.ps1'],
                   check=True, timeout=25)


def windows(peer: str, action: str, component: str = 'all') -> dict:
    # Action/component are parser-owned literals; remote paths are resolved on Windows.
    text = powershell(peer, "$ProgressPreference='SilentlyContinue';"
                      "& ($HOME+'\\jdamr_remote\\windows_runtime.ps1')"
                      f" -Action {action} -Component {component}")
    result = json.loads(text)
    if result.get('error') or result.get('ok') is False:
        raise RuntimeError(result.get('error') or json.dumps(result, ensure_ascii=False))
    for role in result.get('roles', []):
        result[role['role']] = role
    return result


def get_state(url: str) -> dict:
    with urlopen(url + '/api/state', timeout=5) as response:
        return json.load(response)


def ensure_idle(state: dict) -> None:
    if (state.get('active_order_id')
            or (state.get('drive_link') or {}).get('state') == 'running'
            or (state.get('drive_init') or {}).get('state') in ('requested', 'running')):
        raise RuntimeError('주문·복귀 주행 또는 초기화가 진행 중이어서 준비 설정을 바꾸지 않습니다.')


def service(*arguments: str) -> None:
    env = os.environ.copy()
    runtime = env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    env.setdefault('DBUS_SESSION_BUS_ADDRESS', 'unix:path=' + runtime + '/bus')
    subprocess.run(['systemctl', '--user', *arguments], check=True, env=env)


def prepare_bridge(url: str) -> None:
    # Keep local overrides and the existing authenticated API; only bind the order peer.
    directory = Path.home() / '.config/systemd/user' / (UNIT + '.d')
    directory.mkdir(parents=True, exist_ok=True)
    unit = directory.parent / UNIT
    created = not unit.exists()
    if created:
        unit.write_text('[Unit]\nDescription=JD-AMR scenario bridge\n[Service]\nType=simple\n'
                        + 'WorkingDirectory=' + str(ROOT) + '\nRestart=on-failure\n'
                        + '[Install]\nWantedBy=default.target\n')
    override = directory / '30-scenario-runtime.conf'
    content = ('[Service]\nExecStart=\nExecStart=/usr/bin/python3 '
               + str(ROOT / 'tools/jdamr/water_service_api.py')
               + ' serve --order-url ' + url + ' --follow-web\n')
    changed = created or not override.exists() or override.read_text() != content
    if changed:
        override.write_text(content)
        service('daemon-reload')
    # An existing unit can be restarted only while the remote mission is idle.
    if changed:
        service('restart', UNIT)
    else:
        service('start', UNIT)


def wait_ready(peer: str, url: str, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    last = None
    requested_init_id = None
    while time.monotonic() < deadline:
        try:
            state = get_state(url)
        except OSError as error:
            print('관제 응답 재확인 중: ' + str(error), flush=True)
            time.sleep(3)
            continue
        backend = state.get('execution_backend') or {}
        arm = backend.get('manipulation_service_state')
        summary = (arm, (state.get('drive_link') or {}).get('connected'))
        if summary != last:
            print(json.dumps({'arms': arm, 'drive_connected': summary[1]}, ensure_ascii=False), flush=True)
            last = summary
        init = state.get('drive_init') or {}
        if summary[1] and requested_init_id is None:
            if init.get('state') in ('requested', 'running'):
                raise RuntimeError('다른 초기화가 진행 중입니다. 중복 초기화를 보내지 않습니다.')
            request = Request(url + '/api/drive/init', data=b'{}',
                              headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(request, timeout=5) as response:
                requested_init_id = json.load(response)['drive_init']['id']
            if not requested_init_id:
                raise RuntimeError('초기화 요청 ID를 받지 못했습니다.')
            continue
        own_init = requested_init_id is not None and init.get('id') == requested_init_id
        if own_init and init.get('state') == 'failed':
            raise RuntimeError('주행 초기화 실패: ' + str(init.get('message')))
        try:
            remote = windows(peer, 'status')
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            print('Windows 상태 재확인 중: ' + str(error)[:180], flush=True)
            time.sleep(3)
            continue
        if arm == 'ready' and summary[1] and own_init and init.get('state') == 'succeeded':
            roles = [remote.get(name) or {} for name in ('camera', 'order', 'arms')]
            if not all(role.get('state') == 'running' and role.get('pid') for role in roles):
                raise RuntimeError('Windows 서비스가 준비 중 종료됐습니다: ' + json.dumps(remote, ensure_ascii=False))
            services = remote.get('services') or {}
            if not all((services.get(name) or {}).get('reachable')
                       for name in ('rollout_8770', 'order_8765')):
                raise RuntimeError('Windows 서비스 응답이 확인되지 않았습니다.')
            return state
        if not (remote.get('arms') or {}).get('pid'):
            raise RuntimeError('팔 실행기가 종료됐습니다: ' + json.dumps(remote.get('arms'), ensure_ascii=False))
        time.sleep(3)
    raise RuntimeError('준비 대기시간 초과; 서비스는 유지됩니다. status로 준비 상태와 로그를 확인하세요.')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'status', 'stop'))
    parser.add_argument('--windows', default=os.environ.get('JDAMR_WINDOWS_SSH', 'user@192.168.0.157'))
    parser.add_argument('--order-url', default=os.environ.get('JDAMR_ORDER_URL', 'http://192.168.0.157:8765'))
    parser.add_argument('--component', choices=('all', 'arms', 'camera', 'order'), default='all')
    parser.add_argument('--wait-s', type=float, default=600.0)
    args = parser.parse_args()
    if not re.fullmatch(r'https?://[A-Za-z0-9.:-]+', args.order_url):
        parser.error('--order-url must contain only scheme, host and port')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', args.windows):
        parser.error('--windows must be user@host')
    if args.wait_s <= 0:
        parser.error('--wait-s must be positive')
    try:
        install_remote(args.windows)
        if args.action == 'status':
            print(json.dumps(windows(args.windows, 'status'), ensure_ascii=False, indent=2))
            return 0
        if args.action == 'stop':
            # Stop the mission via its existing API before stopping its sensor/services.
            if args.component == 'all':
                stopped = subprocess.run([sys.executable, str(ROOT / 'tools/jdamr/water_service_api.py'), 'stop'],
                                         timeout=30, stdout=subprocess.DEVNULL)
                if stopped.returncode:
                    subprocess.run([sys.executable, str(ROOT / 'tools/jdamr/jdamr_depart.py'), 'stop'],
                                   check=True, timeout=60)
            result = windows(args.windows, 'stop', args.component)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            if args.component == 'all':
                service('stop', UNIT)
            return 0
        # Start only daemons. No order, align/confirm, stow, policy or base motion request.
        if args.component == 'all':
            try:
                before = get_state(args.order_url)
            except OSError:
                before = {}  # A stopped order server is one component to prepare.
            ensure_idle(before)
        print(json.dumps(windows(args.windows, 'start', args.component), ensure_ascii=False), flush=True)
        if args.component != 'all':
            return 0
        state = get_state(args.order_url)
        ensure_idle(state)
        prepare_bridge(args.order_url)
        state = wait_ready(args.windows, args.order_url, args.wait_s)
        print(json.dumps({'prepared': True, 'arms': 'ready',
                          'drive_connected': state['drive_link']['connected'],
                          'drive_init': state.get('drive_init'),
                          'control_url': args.order_url + '/control',
                          'order_url': args.order_url + '/order?table=1'}, ensure_ascii=False), flush=True)
        return 0
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
