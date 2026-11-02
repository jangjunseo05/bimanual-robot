#!/usr/bin/env python3
"""JD-AMR taught-station departure runbook as one PC-side command.

A call runs: dock -> water_station (face approach, escape) -> called table
(face approach, escape) -> reverse docking. The operator's "출발 table_0N" stands in
for a web call.

The robot only moves in `go`, which starts the existing `box_service` executor on the Pi.
Every other subcommand is no-motion: PC display, Nav2 session in prepare-only mode,
localization seeding through the existing `activate_navigation`, home teaching.

    jdamr_depart.py sync            copy Phase 2 assets to the Pi (same paths)
    jdamr_depart.py display-start   PC map_server + relay import + markers + RViz + click capture
    jdamr_depart.py session-start   Pi Nav2 session (precision, prepare-only, LOCALHOST)
    jdamr_depart.py init            registered dock pose -> scan-matched pose -> activation
                                    (--click: RViz click instead, re-teaches home unless --keep-home)
    jdamr_depart.py go table_01     tabletop face: observe -> align -> contract gap (1 cm) -> dwell -> escape -> dock
    jdamr_depart.py health          read-only: does a fresh Pi process receive map/TF/scan
    jdamr_depart.py recover         no-motion restart of sensors/session + init; restore used display
    jdamr_depart.py status | stop | display-stop | session-stop
    jdamr_depart.py estop           latch the base driver stop (wheels at zero), stop the attempt
    jdamr_depart.py estop-reset     release it (refused while a velocity command still arrives)
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
from types import SimpleNamespace
from uuid import uuid4

import yaml

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import dock_survey  # noqa: E402
import scan_match  # noqa: E402

# mDNS (jdamr.local) failed twice on 2026-09-30; the Pi keeps 192.168.0.159 on this LAN.
HOST = os.environ.get('JDAMR_HOST', 'lim@192.168.0.159')
PI_USER = os.environ.get('JDAMR_PI_USER', HOST.split('@', 1)[0])
HOME_DIR = Path(os.path.expanduser('~'))
DATA_ROOT = Path(os.environ.get('JDAMR_DATA_ROOT', HOME_DIR / 'jdamr_data')).expanduser()
BASE = DATA_ROOT / 'map_20260930_manual'
# Wall-aligned copy of the 14:23 Cartographer map (aligned/alignment.json).
P2 = BASE / 'aligned'
MAP_YAML = P2 / 'map.yaml'
# Only the map's unknown cells: satisfies the session's non-empty mask and keeps
# KeepoutFilter from turning unknown cells into free space.
KEEPOUT_YAML = P2 / 'keepout.yaml'
REGISTRY = P2 / 'service_destinations.yaml'
ANNOTATION = P2 / 'annotation.json'
CLICK = P2 / 'rviz_initialpose.json'
STATE = P2 / 'state.json'
PI_HOME = os.environ.get('JDAMR_PI_HOME', '/home/lim').rstrip('/')
PI_WS = os.environ.get('JDAMR_PI_WS', f'{PI_HOME}/jdamr_ws').rstrip('/')
PI_BASE = Path(PI_HOME) / 'jdamr_data/map_20260930_manual'
PI_P2 = PI_BASE / 'aligned'
PI_REGISTRY = PI_P2 / 'service_destinations.yaml'
PI_SESSION = f'{PI_WS}/src/jdamr_cube_ros/jdamr_cube_navigation/scripts/restaurant_session.sh'
# Resident box executor (2026-10-01): one DDS participant for every go, started with
# the session. Every go uses its result protocol, including after a restart.
EXECUTOR_UNIT = 'jdamr-box-executor'
EXECUTOR_SPOOL = f'{PI_HOME}/jdamr_data/box_executor_spool'
SSH_READ_TIMEOUT_S = 10
PI_TOOLS = f'{PI_HOME}/jdamr_data/map_20260930_manual/tools'
PI_SHARE = f'{PI_WS}/install/jdamr_cube_navigation/share/jdamr_cube_navigation'
PI_BOX_CONTRACT = f'{PI_SHARE}/config/box_parking_contract.yaml'
PI_GEOMETRY = (f'{PI_WS}/install/jdamr_cube_description/share/jdamr_cube_description/'
               'config/new_base_geometry.yaml')
PI_MOUNT = f'{PI_HOME}/jdamr_deploy/camera_mount_current.yaml'
ROS_ENV = ('export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 '
           'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4')
# Fast DDS LOCALHOST adds SHM even when UDPv4 is requested. Use the existing
# loopback-only XML with SUBNET discovery so Pi peers keep UDP on 127.0.0.1.
PI_DDS_PROFILE = f'{PI_SHARE}/config/fastdds_udp_loopback.xml'
PI_SOURCE = (f'set +u; source /opt/ros/jazzy/setup.bash; '
             f'source {PI_WS}/install/setup.bash; {ROS_ENV}; '
             'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET '
             f'FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(PI_DDS_PROFILE)}')
SESSION_DISCOVERY = 'SUBNET'
# The shared Astra is owned by Windows in the current serving scenario.
CAMERA_TRANSPORT = os.environ.get('JDAMR_CAMERA_TRANSPORT', 'windows_tcp')
if CAMERA_TRANSPORT not in ('windows_tcp', 'usb'):
    raise ValueError('JDAMR_CAMERA_TRANSPORT must be windows_tcp or usb')
# 9/29's last session ran without composition (plan v4.1 E4); a composed container
# deadlocked when a participant joined (three reproductions on 2026-09-30).
SESSION_COMPOSITION = 'false'
PC_WS = os.environ.get('JDAMR_PC_WS', str(HOME_DIR / 'jdamr_rgbd_ws')).rstrip('/')
PC_SOURCE = ('set +u; source /opt/ros/jazzy/setup.bash; '
             f'source {PC_WS}/install/setup.bash; {ROS_ENV}')
DISPLAY_UNITS = ('jdamr-p2-mapserver', 'jdamr-p2-relay', 'jdamr-p2-markers',
                 'jdamr-p2-rviz', 'jdamr-p2-click')
# PC-side bag of what the display relay already brings over (no Pi disk or load:
# the Pi had 844 MB free on 2026-09-30). /tf carries the odom trajectory and the
# AMCL corrections; the relay never carries command topics by design.
RECORD_UNIT = 'jdamr-p2-record'
RECORD_TOPICS = ('/tf', '/tf_static', '/scan', '/plan', '/amcl_pose')
# Box front-face centres and outward normals (destinations.json, operator RViz clicks).
DESTINATIONS = (json.loads((P2 / 'destinations.json').read_text())['destinations']
                if (P2 / 'destinations.json').exists() else {})
VIA_ID = 'water_station'
TABLES = ('table_01', 'table_02')
ACTIVE_MAP_SESSION = None
ACTIVE_MAP_POINTER = None
STATION_PROFILES = None
# Nominal observation and escape distances from destinations.json's face.
# Table_01 joins its observation point along the aisle instead of returning
# toward the opposing water table's old box-based pre-point.
PRE_M, OBSERVE_M, ESCAPE_M = 1.00, 0.58, 0.565
CAMERA_MOUNT_CONFIG = (
    TOOLS.parents[1] / 'jdamr_cube_navigation/config/camera_mount_current.yaml')
HOME_EXIT_M = 0.70


def configure_map_session(pointer):
    """Bind the existing executor to one taught map without changing the captures."""
    from jdamr_cube_navigation.map_session_assets import prepare_map_session

    profile = prepare_map_session(pointer)
    base = Path(profile['session_directory'])
    runtime = Path(profile['runtime_directory'])
    relative = base.relative_to(DATA_ROOT.resolve())
    pi_base = Path(PI_HOME) / 'jdamr_data' / relative
    pi_runtime = pi_base / runtime.relative_to(base)
    paths = {
        'ACTIVE_MAP_SESSION': profile, 'ACTIVE_MAP_POINTER': Path(pointer).expanduser().resolve(),
        'BASE': base, 'P2': runtime,
        'MAP_YAML': runtime / 'map.yaml', 'KEEPOUT_YAML': runtime / 'keepout.yaml',
        'REGISTRY': runtime / 'service_destinations.yaml',
        'ANNOTATION': runtime / 'annotation.json', 'CLICK': runtime / 'rviz_initialpose.json',
        'STATE': runtime / 'state.json', 'PI_BASE': pi_base, 'PI_P2': pi_runtime,
        'PI_REGISTRY': pi_runtime / 'service_destinations.yaml',
        'DESTINATIONS': json.loads((runtime / 'destinations.json').read_text())['destinations'],
        'TABLES': tuple(profile['station_ids']),
        'STATION_PROFILES': runtime / 'station_profiles.json',
        'PI_TOOLS': f'{PI_WS}/src/jdamr_cube_ros/scripts/map_20260930_manual',
    }
    globals().update(paths)
    if prepare_reference_mount():
        globals().update(CAMERA_MOUNT_CONFIG=runtime / 'camera_mount.yaml',
                         PI_MOUNT=str(pi_runtime / 'camera_mount.yaml'))
    # Shared display/transport inputs carry no coordinates from the old map.
    bundled = TOOLS.parents[3] / 'config/navigation/jdamr_runtime'
    for name in ('fastdds_udp_loopback.xml', 'restaurant_phase2.rviz'):
        source = bundled / name
        if source.exists() and not (runtime / name).exists():
            (runtime / name).write_bytes(source.read_bytes())
    annotation = json.loads(ANNOTATION.read_text()) if ANNOTATION.exists() else {}
    if annotation.get('source_map_sha256') != profile['map_hashes']['runtime']['image_sha256']:
        ANNOTATION.write_text(json.dumps({
            'source_map_sha256': profile['map_hashes']['runtime']['image_sha256'],
            'markers': {},
        }) + '\n')
    state = load_state()
    profiles = json.loads(STATION_PROFILES.read_text())
    reference_path = Path(profiles['home']['dock_scan']['reference_path'])
    reference = json.loads(reference_path.read_text())
    if reference['map_identity'] != dock_map_identity():
        raise ValueError('dock scan reference belongs to a different map')
    if (state.get('map_session_identity') != reference['map_identity']
            or state.get('dock_scan_reference') != reference):
        # A taught session's registered capture is the fixed dock anchor.
        # A later local scan-match must not silently replace that anchor.
        state['dock_scan_reference'] = reference
        state['localized'] = False
        state['map_session_identity'] = reference['map_identity']
        save_state(state)
    return profile


def select_stop_state(pointer):
    """Locate cancellation state without validating any perception or map assets."""
    try:
        document = json.loads(Path(pointer).expanduser().read_text())
        base = Path(document['directory']).expanduser().resolve()
        globals().update(BASE=base, P2=base / 'runtime',
                         STATE=base / 'runtime/state.json')
    except (OSError, ValueError, KeyError, TypeError) as error:
        log(f'cannot locate taught-map state: {error}; stop handler remains available')


def prepare_reference_mount():
    """Copy the selected mount; teaching references must not redefine hardware TF."""
    from jdamr_cube_navigation.box_docking_target import _validated_mount

    canonical = (TOOLS.parents[1]
                 / 'jdamr_cube_navigation/config/camera_mount_current.yaml')
    document = yaml.safe_load(canonical.read_text())
    _validated_mount(document)
    (P2 / 'camera_mount.yaml').write_text(yaml.safe_dump(document, sort_keys=False))
    return True


def prepare_reference_camera():
    """Prepare the selected sensor source without restarting it between tasks."""
    if CAMERA_TRANSPORT == 'windows_tcp':
        # The user unit owns the sole receiver. The USB unit must stay inactive.
        active = pi('systemctl is-active jdamr-box-rgbd.service', check=False).stdout.strip()
        if active == 'active':
            pi('sudo -n systemctl stop jdamr-box-rgbd.service')
        pi('export XDG_RUNTIME_DIR=/run/user/$(id -u); '
           'systemctl --user start jdamr-astra-tcp.service && '
           'sudo -n systemctl start jdamr-box-observer.service')
        log('camera: continuous Windows AST1 receiver and observer; no USB launch')
        return
    if ACTIVE_MAP_SESSION is None:
        return
    source = (f'source /opt/ros/jazzy/setup.bash; '
              f'source {shlex.quote(PI_HOME + "/astra_ws/install/setup.bash")}; '
              f'exec {shlex.quote(PI_WS + "/install/jdamr_cube_bringup/lib/"
                                 "jdamr_cube_bringup/jdamr-launch-supervisor")} -- '
              'ros2 launch astra_camera astra.launch.xml '
              'enable_color:=true color_width:=640 color_height:=480 color_fps:=30 '
              'enable_depth:=true enable_ir:=false enable_point_cloud:=false '
              'enable_colored_point_cloud:=false depth_registration:=true '
              'color_depth_synchronization:=false tf_publish_rate:=0.0 '
              'depth_width:=320 depth_height:=240 depth_fps:=30')
    content = '[Service]\nExecStart=\nExecStart=/bin/bash -c ' + shlex.quote(source) + '\n'
    target = '/etc/systemd/system/jdamr-box-rgbd.service.d/96-taught-station-reference.conf'
    result = pi(f'cat {shlex.quote(target)}', check=False)
    if result.returncode == 0 and result.stdout == content:
        return
    active = pi('systemctl is-active jdamr-box-rgbd.service', check=False).stdout.strip()
    pi(f'sudo -n mkdir -p {shlex.quote(str(Path(target).parent))} && '
       f'sudo -n tee {shlex.quote(target)} >/dev/null', stdin=content)
    pi('sudo -n systemctl daemon-reload')
    if active == 'active':
        pi('sudo -n systemctl restart jdamr-box-rgbd.service', timeout=90)


def along_normal(dest_id, distance_m):
    spec = DESTINATIONS[dest_id]
    yaw = math.radians(spec['normal_deg'])
    return (spec['face_xy'][0] + distance_m * math.cos(yaw),
            spec['face_xy'][1] + distance_m * math.sin(yaw), yaw)


def table_observation_offset_m():
    if not CAMERA_MOUNT_CONFIG.exists():
        return OBSERVE_M
    mount = yaml.safe_load(CAMERA_MOUNT_CONFIG.read_text())['camera_mount']
    value = mount.get('table_observation', {}).get('base_offset_m', OBSERVE_M)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not .2 <= value <= OBSERVE_M):
        raise ValueError('invalid camera table observation offset')
    return value


def approach_waypoints(dest_id):
    if ACTIVE_MAP_SESSION is not None:
        registry = yaml.safe_load(REGISTRY.read_text())
        table = next(item for item in registry['tables'] if item['table_id'] == dest_id)
        pose = table['service_poses'][0]
        yaw = pose['yaw_rad']
        offset = pose['approach_offset_m']
        return [
            {'id': f'{dest_id}_pre',
             'x': pose['x_m'] - PRE_M * math.cos(yaw),
             'y': pose['y_m'] - PRE_M * math.sin(yaw)},
            {'id': f'{dest_id}_observation',
             'x': pose['x_m'] - offset * math.cos(yaw),
             'y': pose['y_m'] - offset * math.sin(yaw), 'yaw': yaw},
        ]
    pre_x, pre_y, normal = along_normal(dest_id, PRE_M)
    obs_x, obs_y, _ = along_normal(dest_id, table_observation_offset_m())
    facing = math.atan2(-math.sin(normal), -math.cos(normal))
    if dest_id == 'table_01':
        pre_x = along_normal(dest_id, OBSERVE_M)[0]
        pre_y = along_normal(VIA_ID, ESCAPE_M)[1]
    return [{'id': f'{dest_id}_pre', 'x': round(pre_x, 3), 'y': round(pre_y, 3)},
            {'id': f'{dest_id}_observation', 'x': round(obs_x, 3), 'y': round(obs_y, 3),
             'yaw': round(facing, 4)}]


def station_escape_xy(dest_id):
    """Expected base position after the face-relative straight escape."""
    spec = DESTINATIONS[dest_id]
    normal = math.radians(spec['normal_deg'])
    return (spec['face_xy'][0] + ESCAPE_M * math.cos(normal),
            spec['face_xy'][1] + ESCAPE_M * math.sin(normal))


def station_region(state, dest_id):
    """Derive current-map regions from current teaching, not a cached old pose."""
    if ACTIVE_MAP_SESSION is not None and dest_id in DESTINATIONS:
        return {
            'xy': list(DESTINATIONS[dest_id]['face_xy']),
            'radius_m': REGION_RADIUS_M,
            'basis': 'taught-map destination face',
        }
    region = state.get('regions', {}).get(dest_id)
    if region is not None:
        return region
    fail(f'{dest_id} region is missing')


REGION_RADIUS_M = 0.40
# A box face seen from the dock appears as a 0.25-0.7 m unmapped LiDAR line; smaller
# clusters (chair legs, bags) never move the region away from the user's marker.
UNMAPPED_REGION_MAX_M = 0.5
BOX_FACE_LENGTH_M = (0.25, 0.7)
# Budgets are not safety gates (the progress checker ends real stalls). Conservative
# upper bounds from plan v4.1 section 1.7: return about 500 s, table leg above 240 s.
TASK_TIMEOUT_S = 600.0
RETURN_TIMEOUT_S = 600.0
INIT_COVARIANCE = {'covariance_x_m2': 0.0025, 'covariance_y_m2': 0.0025,
                   'covariance_yaw_rad2': math.radians(5.0) ** 2}
# Three unmapped boxes near the dock lower the inlier share (0.48-0.49 at 15:20 with a
# consistent global match); the global agreement and the AMCL agreement stay the gates.
MATCH_MIN_INLIER = 0.45
MATCH_OVERRIDE_FLOOR = 0.25
MATCH_CLICK_XY_M = 0.5
MATCH_CLICK_YAW_RAD = math.radians(30.0)
AMCL_AGREE_XY_M = 0.05
AMCL_AGREE_YAW_RAD = math.radians(2.0)
CLICK_MAX_AGE_S = 1800.0
# The 2026-09-30 morning fix for new processes receiving nothing restarted all three.
SENSOR_UNITS = ('jdamr-base.service', 'jdamr-box-rgbd.service', 'jdamr-box-observer.service')
# The executor starts these for the box stops and stops them afterwards (2026-10-06:
# they held the Pi at 80 deg C); a session or recovery does not start them.
CAMERA_UNITS = ('jdamr-box-rgbd.service', 'jdamr-box-observer.service')
# Recorded on the Pi for every go: wheel odom, raw IMU and scan for the odom/IMU EKF
# comparison (rotation_truth.py), velocity commands for the Spin overshoot.
ONBOARD_BAG_CHECK_S = 25.0
# No wait at the dock between cycles of a sequence (operator, 2026-10-01); the
# init that re-localizes at the dock still runs.
DOCK_WAIT_S = 0.0
ONBOARD_TOPICS = ('/odom', '/imu/data_raw', '/scan', '/tf', '/tf_static', '/amcl_pose',
                  '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel')
# box_service's first check (verify_live_maps, 30 s) runs before any motion command.
MAP_WAIT_FAILURE = 'live map data unavailable'


def log(message):
    print(time.strftime('%H:%M:%S'), message, flush=True)


def fail(message):
    log('STOP: ' + message)
    raise SystemExit(1)


def sh(command, timeout=120, check=True, capture=True):
    result = subprocess.run(['bash', '-c', command], text=True, timeout=timeout,
                            stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.STDOUT if capture else None)
    if check and result.returncode != 0:
        fail(f'command failed ({result.returncode}): {command}\n{result.stdout}')
    return result


def pi(command, timeout=120, check=True, stdin=None):
    args = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', HOST, command]
    result = subprocess.run(args, text=True, timeout=timeout, input=stdin,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if check and result.returncode != 0:
        fail(f'pi command failed ({result.returncode}): {command}\n{result.stdout}')
    return result


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pi_path(path):
    """Map a local runbook data path to the matching path below PI_HOME."""
    path = Path(path)
    try:
        return PI_BASE / path.relative_to(BASE)
    except ValueError:
        return path


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save_state(state):
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, indent=1))
    os.replace(temporary, STATE)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def mark_localized(value):
    """Only a completed init localizes; a session restart or a failed init revokes it."""
    state = load_state()
    state['localized'] = value
    save_state(state)


def session_active():
    for _attempt in range(2):
        result = pi('systemctl is-active jdamr-restaurant-navigation.service',
                    timeout=SSH_READ_TIMEOUT_S, check=False)
        if result.returncode != 255:
            return result.returncode == 0
    return False


def refuse_during_cycle(action):
    """Session stops and sensor restarts would end a running cycle mid-drive."""
    result = pi("systemctl list-units --type=service --state=active,activating,deactivating "
                "--no-legend 'jdamr-table-cycle-*'; "
                f"ls {EXECUTOR_SPOOL}/*.request {EXECUTOR_SPOOL}/*.running 2>/dev/null; true",
                timeout=SSH_READ_TIMEOUT_S, check=False)
    if result.returncode == 255:
        fail(f'ssh to the Pi failed; cannot tell whether a cycle is running: {result.stdout[-200:]}')
    busy = result.stdout.strip()
    if busy:
        fail(f'a table cycle is running; {action} would stop it (use stop first): {busy}')


# ---------------------------------------------------------------- assets
def write_routes(home):
    """Generate table approaches from the dock and from the water stop's escape pose."""
    keepout_image = KEEPOUT_YAML.parent / yaml.safe_load(KEEPOUT_YAML.read_text())['image']
    hx, hy, hyaw = home
    exit_wp = {'id': 'home_exit', 'x': round(hx + HOME_EXIT_M * math.cos(hyaw), 3),
               'y': round(hy + HOME_EXIT_M * math.sin(hyaw), 3)}
    if ACTIVE_MAP_SESSION is not None:
        for dest_id in TABLES:
            is_via = dest_id == VIA_ID
            if is_via:
                start = (hx, hy)
                waypoints = [exit_wp, *approach_waypoints(dest_id)]
                start_basis = 'registered dock pose'
            else:
                start = station_escape_xy(VIA_ID)
                waypoints = approach_waypoints(dest_id)
                start_basis = f'{VIA_ID} face-relative escape pose'
            route = {
                'schema_version': 1, 'frame_id': 'map',
                'map_yaml': str(MAP_YAML), 'keepout_mask_yaml': str(KEEPOUT_YAML),
                'expected_mask_sha256': sha256(keepout_image),
                'start_pose': {'x': round(start[0], 3), 'y': round(start[1], 3)},
                'max_route_start_distance_m': 0.3,
                'waypoints': waypoints,
                'purpose': 'taught station observation; final parking follows in the same run',
                'target_basis': 'operator taught pose; no old-map destinations',
                'start_basis': start_basis,
                'station_profiles': str(STATION_PROFILES),
            }
            if is_via:
                if ACTIVE_MAP_SESSION.get('dock_departure_mode', 'straight') == 'straight':
                    route['dock_departure_distance_m'] = HOME_EXIT_M
                direct_route = route
            else:
                direct_route = {
                    **route,
                    'start_pose': {'x': round(hx, 3), 'y': round(hy, 3)},
                    'waypoints': [exit_wp, *waypoints],
                    'start_basis': 'registered dock pose; water stop omitted',
                }
                if ACTIVE_MAP_SESSION.get('dock_departure_mode', 'straight') == 'straight':
                    direct_route['dock_departure_distance_m'] = HOME_EXIT_M
            (P2 / f'{dest_id}_route.yaml').write_text(
                yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
            (P2 / f'{dest_id}_direct_route.yaml').write_text(
                yaml.safe_dump(direct_route, sort_keys=False, allow_unicode=True))
        log(f'taught-map routes written: {", ".join(TABLES)}; reverse home retained')
        return
    escape = along_normal(VIA_ID, ESCAPE_M)
    plans = {VIA_ID: ((hx, hy), [exit_wp, *approach_waypoints(VIA_ID)],
                      'registry home taught at the operator placement (start = dock)')}
    for table_id in TABLES:
        plans[table_id] = ((escape[0], escape[1]), approach_waypoints(table_id),
                           'water_station escape pose (face + 0.565 m along its normal)')
    for dest_id, (start, waypoints, basis) in plans.items():
        route = {
            'schema_version': 1, 'frame_id': 'map',
            'map_yaml': str(MAP_YAML), 'keepout_mask_yaml': str(KEEPOUT_YAML),
            'expected_mask_sha256': sha256(keepout_image),
            'start_pose': {'x': round(float(start[0]), 3), 'y': round(float(start[1]), 3)},
            'max_route_start_distance_m': 0.3,
            'waypoints': waypoints,
            'purpose': f'{dest_id} box observation approach; not a final parking goal',
            'target_basis': f'destinations.json {DESTINATIONS[dest_id]["source"]}',
            'start_basis': basis,
            'camera_observation_base_offset_m': table_observation_offset_m(),
        }
        if dest_id == VIA_ID:
            route['dock_departure_distance_m'] = HOME_EXIT_M
        path = P2 / f'{dest_id}_route.yaml'
        path.write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
        if dest_id in TABLES:
            dock_route = {
                **route,
                'start_pose': {'x': round(hx, 3), 'y': round(hy, 3)},
                'waypoints': [exit_wp, *waypoints],
                'start_basis': 'registered dock pose; water stop omitted',
                'dock_departure_distance_m': HOME_EXIT_M,
            }
            (P2 / f'{dest_id}_direct_route.yaml').write_text(
                yaml.safe_dump(dock_route, sort_keys=False, allow_unicode=True))
    log(f'routes written: home ({hx:.3f}, {hy:.3f}, {math.degrees(hyaw):.1f} deg), '
        f'water escape ({escape[0]:.3f}, {escape[1]:.3f})')


def cmd_sync(_args):
    pi(f'mkdir -p {shlex.quote(str(PI_P2))}/runs')
    sh(f'rsync -a --exclude runs --exclude state.json --exclude rviz_initialpose.json '
       f'{shlex.quote(str(P2))}/ {HOST}:{shlex.quote(str(PI_P2))}/')
    for name in ('service_destinations.yaml', 'keepout.pgm', 'keepout.yaml'):
        local = sha256(P2 / name)
        remote = pi(f'sha256sum {shlex.quote(str(PI_P2 / name))}').stdout.split()[0]
        if local != remote:
            fail(f'{name} differs after sync')
    if ACTIVE_MAP_SESSION is not None:
        profiles = json.loads(STATION_PROFILES.read_text())
        source_map = Path(ACTIVE_MAP_SESSION['map_hashes']['source']['yaml_path'])
        reference_paths = {
            source_map,
            source_map.parent / yaml.safe_load(source_map.read_text())['image'],
            Path(profiles['home']['source_record']),
            Path(profiles['home']['dock_scan']['reference_path']),
            Path(profiles['home']['dock_scan']['scan_path']),
        }
        for station_id, station in profiles['stations'].items():
            reference_paths.add(Path(station['source_record']))
            config = station.get('rgbd_reference')
            if config is None:
                continue
            if not isinstance(config, dict):
                fail(f'{station_id} rgbd_reference must be a mapping or null')
            for key in ('reference_capture_path', 'reference_rgb_path', 'reference_depth_path'):
                if config.get(key):
                    reference_paths.add(Path(config[key]))
        for source in sorted(reference_paths):
            remote = Path(PI_HOME) / 'jdamr_data' / source.relative_to(DATA_ROOT)
            pi(f'mkdir -p {shlex.quote(str(remote.parent))}')
            sh(f'rsync -a --protect-args {shlex.quote(str(source))} '
               f'{HOST}:{shlex.quote(str(remote))}')
            if pi(f'sha256sum {shlex.quote(str(remote))}').stdout.split()[0] != sha256(source):
                fail(f'{source.name} reference differs after sync')
        # Publish selection after its inputs, including departure mode and IDs.
        remote_pointer = Path(PI_HOME) / 'jdamr_data' / ACTIVE_MAP_POINTER.relative_to(DATA_ROOT)
        pi(f'mkdir -p {shlex.quote(str(remote_pointer.parent))}')
        sh(f'rsync -a --protect-args {shlex.quote(str(ACTIVE_MAP_POINTER))} '
           f'{HOST}:{shlex.quote(str(remote_pointer))}')
        if pi(f'sha256sum {shlex.quote(str(remote_pointer))}').stdout.split()[0] != sha256(
                ACTIVE_MAP_POINTER):
            fail('map session pointer differs after sync')
    log('map assets and taught references synced to the Pi (hashes match)')


def pull_registry():
    """The Pi registry is authoritative once teach-home ran there."""
    text = pi(f'cat {shlex.quote(str(PI_REGISTRY))}').stdout
    REGISTRY.write_text(text)
    return yaml.safe_load(text)


# ---------------------------------------------------------------- display (PC)
def cmd_display_start(args):
    units = getattr(args, '_display_units', DISPLAY_UNITS)
    record = getattr(args, 'record_diagnostics', False)
    missing = [u for u in units
               if sh(f'systemctl --user is-active {u}', check=False).stdout.strip() != 'active']
    if not missing:
        log('display already running')
        if record:
            record_start()
        return
    env = '--setenv=DISPLAY=:1 --setenv=XAUTHORITY=/run/user/1000/gdm/Xauthority'
    commands = {
        'jdamr-p2-mapserver': (
            f'{PC_SOURCE}; ros2 run nav2_map_server map_server --ros-args '
            f'-p yaml_filename:={MAP_YAML} -p frame_id:=map & '
            'sleep 3; ros2 run nav2_util lifecycle_bringup map_server; wait'),
        'jdamr-p2-relay': (
            f'{PC_SOURCE}; exec python3 -m jdamr_cube_navigation.rviz_display_relay import '
            f'--host {shlex.quote(HOST)} --workspace {shlex.quote(PI_WS)}'),
        'jdamr-p2-markers': (
            f'{PC_SOURCE}; exec ros2 run jdamr_cube_navigation service_visualization '
            f'--registry {REGISTRY} --annotation {ANNOTATION} --selected {TABLES[0]}'
            + (f' --station-directory {shlex.quote(str(BASE))}'
               if ACTIVE_MAP_SESSION is not None else '')),
        'jdamr-p2-rviz': f'{PC_SOURCE}; exec rviz2 -d {P2}/restaurant_phase2.rviz',
        'jdamr-p2-click': f'{PC_SOURCE}; exec python3 {TOOLS}/initialpose_capture.py {CLICK}',
    }
    for unit in missing:
        command = commands[unit]
        # The relay rides on ssh to the Pi; one timeout at 09:08 on 2026-10-02 ended
        # it and RViz stayed empty for an hour. It restarts itself now.
        restart = ('--property=Restart=on-failure --property=RestartSec=5 '
                   if unit == 'jdamr-p2-relay' else '')
        sh(f'systemd-run --user --unit={unit} --collect {env} {restart}'
           f'--property=KillSignal=SIGINT --property=TimeoutStopSec=10 '
           f'/bin/bash -c {shlex.quote(command)}')
    time.sleep(4)
    states = {u: sh(f'systemctl --user is-active {u}', check=False).stdout.strip()
              for u in units}
    (P2 / 'display_units.txt').write_text(
        json.dumps({'started': time.time(), 'units': states,
                    'stop': 'jdamr_depart.py display-stop'}, indent=1))
    log(f'display units: {states}')
    if any(v != 'active' for v in states.values()):
        fail('a display unit is not active; see journalctl --user -u <unit>')
    if record:
        record_start()


def record_start():
    """Start the PC bag once and confirm that it actually writes messages."""
    if sh(f'systemctl --user is-active {RECORD_UNIT}', check=False).stdout.strip() == 'active':
        log('bag recorder already running')
        return
    bag = P2 / 'runs' / f'bag_{time.strftime("%Y%m%d_%H%M%S")}'
    command = (f'{PC_SOURCE}; exec ros2 bag record -s mcap -o {shlex.quote(str(bag))} '
               + ' '.join(RECORD_TOPICS))
    sh(f'systemd-run --user --unit={RECORD_UNIT} --collect '
       f'--property=KillSignal=SIGINT --property=TimeoutStopSec=15 '
       f'/bin/bash -c {shlex.quote(command)}')
    # Remember the bag before the write check: rosbag2 caches messages, so a slow
    # first flush must not leave record-stop pointing at the previous bag.
    state = load_state()
    state['bag'] = str(bag)
    save_state(state)
    for _ in range(15):
        time.sleep(2)
        written = sum(f.stat().st_size for f in bag.glob('*.mcap')) if bag.exists() else 0
        if written > 4096:
            log(f'bag recording: {bag} ({written} bytes so far)')
            return
    log(f'bag recorder started but nothing written yet: {bag} '
        '(the relay may still be connecting; check with status)')


def record_stop():
    sh(f'systemctl --user stop {RECORD_UNIT}', check=False)
    bag = load_state().get('bag')
    if bag and Path(bag).exists():
        size = sum(f.stat().st_size for f in Path(bag).glob('*'))
        log(f'bag closed: {bag} ({size} bytes)')


def cmd_display_stop(_args):
    record_stop()
    for unit in DISPLAY_UNITS:
        sh(f'systemctl --user stop {unit}', check=False)
    left = [u for u in DISPLAY_UNITS
            if sh(f'systemctl --user is-active {u}', check=False).stdout.strip() == 'active']
    log(f'display stopped; still active: {left}')


# ---------------------------------------------------------------- session (Pi)
SESSION_SCRIPT = r'''
{source}
was_active=$(systemctl is-active jdamr-restaurant-navigation.service)
start_at=$(date '+%Y-%m-%d %H:%M:%S')
if ! bash {session} start --registry {registry} --discovery-range {discovery} \
    --use-composition {composition} --precision-parking --prepare-only; then
  [ "$was_active" = active ] || exit 3
  # Identity mismatch (registry, params or contract changed): no-motion restart once.
  bash {session} stop || exit 3
  start_at=$(date '+%Y-%m-%d %H:%M:%S')
  was_active=inactive
  bash {session} start --registry {registry} --discovery-range {discovery} \
    --use-composition {composition} --precision-parking --prepare-only || exit 3
fi
[ "$was_active" = active ] && {{ echo REUSED; exit 0; }}
for i in $(seq 1 40); do
  J=$(journalctl -a -u jdamr-restaurant-navigation.service --since "$start_at" -o cat --no-pager)
  if grep -q 'lifecycle_manager_localization\].*Managed nodes are active' <<<"$J" &&
     grep -q 'lifecycle_manager_keepout\].*Managed nodes are active' <<<"$J"; then
    echo STARTED; exit 0
  fi
  sleep 3
done
exit 4
'''


def start_sensor_services():
    """After a reboot only jdamr-base starts by itself; the camera starts on demand."""
    for unit in SENSOR_UNITS:
        if unit in CAMERA_UNITS:
            continue
        if pi(f'systemctl is-active {unit}', check=False).stdout.strip() != 'active':
            pi(f'sudo -n systemctl start {unit}', timeout=120, check=False)
            log(f'{unit}: ' + pi(f'systemctl is-active {unit}', check=False).stdout.strip())


def cmd_session_start(_args):
    refuse_during_cycle('session-start')
    prepare_reference_camera()
    start_sensor_services()
    script = SESSION_SCRIPT.format(source=PI_SOURCE, session=PI_SESSION, registry=PI_REGISTRY,
                                   discovery=SESSION_DISCOVERY,
                                   composition=SESSION_COMPOSITION)
    try:
        result = pi('bash -s', stdin=script, timeout=240, check=False)
    except subprocess.TimeoutExpired:
        mark_localized(False)
        fail('Nav2 session start did not answer within 240 s; run status, then init')
    tail = result.stdout.strip().splitlines()[-3:]
    if not tail or tail[-1] != 'REUSED':
        # A new or restarted session holds no localization until init.
        mark_localized(False)
    if result.returncode != 0:
        fail(f'Nav2 session start failed (code {result.returncode}): {tail}')
    log(f'Nav2 session {tail[-1] if tail else ""}: precision, prepare-only, '
        f'{SESSION_DISCOVERY}, composition={SESSION_COMPOSITION}; '
        'AMCL waits for init, motion servers inactive')
    if not start_executor():
        fail('Nav2 session is ready but the executor did not start; no motion requested')


def executor_active():
    for _attempt in range(2):
        result = pi(f'systemctl is-active {EXECUTOR_UNIT}',
                    timeout=SSH_READ_TIMEOUT_S, check=False)
        if result.returncode != 255:
            return result.returncode == 0
    return False


def start_executor():
    """Start the resident box executor once the Nav2 session is up (no motion)."""
    if executor_active():
        return True
    command = (f'{PI_SOURCE}; exec python3 -m jdamr_cube_navigation.box_service '
               f'--serve {EXECUTOR_SPOOL}')
    pi(f'mkdir -p {EXECUTOR_SPOOL} && sudo -n systemd-run --unit={EXECUTOR_UNIT} --collect '
       f'--property=User={PI_USER} --property=KillSignal=SIGTERM --property=TimeoutStopSec=30 '
       f'--setenv=HOME={PI_HOME} --setenv=OPENBLAS_NUM_THREADS=1 --setenv=OMP_NUM_THREADS=1 '
       f'--working-directory={PI_WS} /bin/bash -c {shlex.quote(command)}',
       check=False)
    active = executor_active()
    log(f'{EXECUTOR_UNIT}: ' + ('active' if active else 'not active'))
    return active


def restart_executor():
    """Retry initial map discovery without resetting sensors, localization or displays."""
    refuse_during_cycle('executor restart')
    result = pi(f'sudo -n systemctl stop {EXECUTOR_UNIT}', timeout=60, check=False)
    if result.returncode != 0 and executor_active():
        fail('executor did not stop; no restart or departure was submitted')
    if not start_executor():
        fail('executor restart failed; no departure was submitted')


def cmd_session_stop(_args):
    refuse_during_cycle('session-stop')
    mark_localized(False)
    pi(f'sudo -n systemctl stop {EXECUTOR_UNIT}', timeout=60, check=False)
    pi(f'{PI_SOURCE}; bash {PI_SESSION} stop', timeout=90, check=False)
    log('session: ' + pi('systemctl is-active jdamr-restaurant-navigation.service',
                         check=False).stdout.strip())


# ---------------------------------------------------------------- init
def capture_motion_failure(odometry):
    """Reject sustained motion and an unsettled capture end, not isolated spikes."""
    if not odometry:
        return 'scan capture returned no odometry'
    moving_since_s = None
    previous_stamp_s = None
    for sample in odometry:
        stamp_s, linear_mps, angular_radps = (
            sample['stamp'], sample['v'], sample['w'])
        if (not all(math.isfinite(value) for value in (
                stamp_s, linear_mps, angular_radps))
                or previous_stamp_s is not None and stamp_s < previous_stamp_s):
            return 'scan capture odometry is invalid or time-regressed'
        previous_stamp_s = stamp_s
        moving = abs(linear_mps) > 0.02 or abs(angular_radps) > 0.02
        if not moving:
            moving_since_s = None
            continue
        if moving_since_s is None:
            moving_since_s = stamp_s
        # Match the existing precision stop confirmation interval. A lone EKF
        # angular-velocity spike must not invalidate otherwise stationary scans.
        if stamp_s - moving_since_s >= 0.08:
            return ('robot is moving during capture '
                    f'(linear={linear_mps:.3f} m/s, angular={angular_radps:.3f} rad/s)')
    if moving_since_s is not None:
        return 'scan capture ended before odometry confirmed stop'
    return None


def capture_scan(path):
    script = (TOOLS / 'capture_scan2.py').read_text()
    result = pi(f'{PI_SOURCE}; python3 - 40', stdin=script, timeout=60)
    line = next((l for l in result.stdout.splitlines() if l.startswith('JSON:')), None)
    if line is None:
        fail('scan capture returned no data (check linger and sensor services)')
    Path(path).write_text(line[5:])
    data = json.loads(line[5:])
    if len(data['scans']) < 20:
        fail(f'only {len(data["scans"])} scans captured')
    motion_failure = capture_motion_failure(data['odom'])
    if motion_failure:
        fail(motion_failure)
    return data


def refine_pose(scan_path, click):
    img, occ, free, res, origin = scan_match.load_map(str(MAP_YAML))
    pts, *_ = scan_match.laser_points(str(scan_path))
    score, _ = scan_match.make_score(occ, res, origin)
    seed = (click['x_m'], click['y_m'], click['yaw_rad'])
    local_score, pose = scan_match.search(score, pts, seed, xy_half=MATCH_CLICK_XY_M,
                                          yaw_half=MATCH_CLICK_YAW_RAD)
    inlier = float((score(pts, *pose)[1] < 0.05).mean())
    registered_dock = (ACTIVE_MAP_SESSION is not None
                       and click.get('source') == 'registered_dock')
    best, agree = None, False
    if not registered_dock:
        ranked = dock_survey.global_pose(score, pts, free, res, origin)
        best = ranked[0]['pose']
        agree = (math.dist(best[:2], pose[:2]) <= 0.1
                 and abs(wrap(best[2] - pose[2])) <= math.radians(3.0))
    return {'pose': [float(v) for v in pose], 'mean_dist_m': float(local_score),
            'inlier_5cm': float(inlier),
            'global_best': None if best is None else [float(v) for v in best],
            'global_search_performed': not registered_dock,
            'global_agrees': bool(agree), 'click': [float(v) for v in seed],
            'click_offset_m': float(math.dist(seed[:2], pose[:2])),
            'click_offset_deg': float(math.degrees(wrap(seed[2] - pose[2])))}


def dock_map_identity():
    meta = yaml.safe_load(MAP_YAML.read_text())
    return [sha256(MAP_YAML), sha256(MAP_YAML.parent / meta['image'])]


def refine_at_registered_dock(scan_path, click, reference, *, diagnostics=None):
    """Match a stated dock placement to its anchored scan, never to a rolling seed."""
    def reject(reason, **values):
        if diagnostics is not None:
            diagnostics.update(accepted=False, reason=reason, **values)
        return None

    if click.get('source') != 'registered_dock' or not isinstance(reference, dict):
        return reject('registered_dock_reference_not_selected')
    import numpy as np
    try:
        for key in ('home_pose', 'pose_map'):
            values = reference[key]
            if (not isinstance(values, (tuple, list)) or len(values) != 3
                    or not all(isinstance(v, (float, int)) and not isinstance(v, bool)
                               and math.isfinite(v) for v in values)):
                return reject('dock_reference_pose_invalid', field=key)
        if (reference['map_identity'] != dock_map_identity()
                or math.dist(reference['home_pose'][:2], (click['x_m'], click['y_m'])) > .001
                or abs(wrap(reference['home_pose'][2] - click['yaw_rad'])) > .001):
            return reject('dock_reference_identity_mismatch')
        current, *_c, laser, _data = scan_match.laser_points(str(scan_path))
        anchored, *_a, old_laser, _old = scan_match.laser_points(reference['scan_path'])
        minimum_matched_points = 80
        if (min(len(current), len(anchored)) < minimum_matched_points
                or math.dist(laser[:2], old_laser[:2]) > .001
                or abs(wrap(laser[2] - old_laser[2])) > .001
                or not np.isfinite(current).all() or not np.isfinite(anchored).all()):
            return reject('dock_scan_geometry_or_support_invalid',
                          current_points=len(current), reference_points=len(anchored),
                          current_laser=list(laser), reference_laser=list(old_laser))

        def transformed(pose):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            return current @ np.array([[c, s], [-s, c]]) + pose[:2]

        def support_distances(relative_pose):
            moved = transformed(relative_pose)
            squared = np.sum(
                (moved[:, None, :] - anchored[None, :, :]) ** 2, axis=2)
            distances = np.sqrt(np.min(squared, axis=1))
            reverse_distances = np.sqrt(np.min(squared, axis=0))
            return distances, reverse_distances

        def score(_points, x_m, y_m, yaw_rad):
            distances, reverse_distances = support_distances((x_m, y_m, yaw_rad))
            # Fit the common support required below in both directions. New
            # chairs/occlusions can change most returns without moving the dock;
            # scoring 65% of all current returns forced those outliers into the fit.
            common_mean = max(
                np.mean(np.sort(distances)[:minimum_matched_points]),
                np.mean(np.sort(reverse_distances)[:minimum_matched_points]))
            return float(common_mean), distances

        best, relative = scan_match.search(
            score, current, (0., 0., 0.), xy_half=.15, yaw_half=math.radians(10))
        if (math.hypot(*relative[:2]) >= .145 or abs(relative[2]) >= math.radians(9.5)
                or best > .02):
            return reject('dock_match_residual_or_search_boundary',
                          relative_pose=list(relative), common_mean_m=best)
        distances, reverse_distances = support_distances(relative)
        inliers = distances < .05
        sectors = np.floor((np.arctan2(current[inliers, 1], current[inliers, 0])
                            + math.pi) / (math.pi / 2)).astype(int) % 4
        supported_sectors = sum(np.count_nonzero(sectors == i) >= 8 for i in range(4))
        reference_inliers = reverse_distances < .05
        # Occlusion and new returns change both denominators at a fixed dock.
        # Use matched support in both scans; residual, sectors and uniqueness
        # still distinguish the anchor from another placement.
        if (min(np.count_nonzero(inliers), np.count_nonzero(reference_inliers))
                < minimum_matched_points
                or supported_sectors < 3):
            return reject('dock_match_common_support_insufficient',
                          current_support=int(np.count_nonzero(inliers)),
                          reference_support=int(np.count_nonzero(reference_inliers)),
                          supported_sectors=supported_sectors)
        # Compare the same support in every uniqueness trial. Reselecting the
        # nearest 80 points lets a different wall fragment replace the dock and
        # makes a distinct anchored placement appear ambiguous.
        fixed_support_mean = max(
            float(np.mean(distances[inliers])),
            float(np.mean(reverse_distances[reference_inliers])))
        uniqueness_trials = []
        ambiguous_translations = []
        origin = reference['pose_map']
        c, s = math.cos(origin[2]), math.sin(origin[2])

        def registered_pose(relative_pose):
            return [float(origin[0] + c * relative_pose[0] - s * relative_pose[1]),
                    float(origin[1] + s * relative_pose[0] + c * relative_pose[1]),
                    float(wrap(origin[2] + relative_pose[2]))]

        def inside_registered_region(candidate):
            return (math.dist(candidate[:2], (click['x_m'], click['y_m'])) <= .2
                    and abs(wrap(candidate[2] - click['yaw_rad'])) <= math.radians(15))

        for axis, delta in ((0, .06), (1, .06), (2, math.radians(3))):
            for sign in (-1, 1):
                trial = list(relative)
                trial[axis] += sign * delta
                forward_trial, reverse_trial = support_distances(trial)
                trial_mean = max(
                    float(np.mean(forward_trial[inliers])),
                    float(np.mean(reverse_trial[reference_inliers])))
                uniqueness_trials.append({
                    'axis': axis, 'delta': sign * delta,
                    'fixed_support_mean_m': trial_mean})
                if trial_mean <= fixed_support_mean + .003:
                    # The stated registered dock supplies the location prior.
                    # A bounded translation along a wall is seed uncertainty,
                    # not proof of a unique millimetre pose. Keep rotational
                    # ambiguity and candidates outside that prior as failures.
                    if axis == 2 or not inside_registered_region(registered_pose(trial)):
                        return reject('dock_match_ambiguous_fixed_support',
                                      fixed_support_mean_m=fixed_support_mean,
                                      trial=uniqueness_trials[-1])
                    ambiguous_translations.append(uniqueness_trials[-1])
        pose = registered_pose(relative)
        if not inside_registered_region(pose):
            return reject('dock_match_outside_registered_region', matched_pose=pose)
        seed_covariance = dict(INIT_COVARIANCE)
        if ambiguous_translations:
            deltas = [(entry['delta'] if entry['axis'] == 0 else 0.,
                       entry['delta'] if entry['axis'] == 1 else 0.)
                      for entry in ambiguous_translations]
            seed_covariance['covariance_x_m2'] += max(
                (c * dx - s * dy) ** 2 for dx, dy in deltas)
            seed_covariance['covariance_y_m2'] += max(
                (s * dx + c * dy) ** 2 for dx, dy in deltas)
        basis = ('anchored_dock_scan_registered_prior' if ambiguous_translations
                 else 'anchored_dock_scan')
        if diagnostics is not None:
            diagnostics.update(accepted=True, reason=basis,
                               current_support=int(np.count_nonzero(inliers)),
                               reference_support=int(np.count_nonzero(reference_inliers)),
                               supported_sectors=supported_sectors,
                               fixed_support_mean_m=fixed_support_mean,
                               uniqueness_trials=uniqueness_trials,
                               ambiguous_translation_trials=ambiguous_translations,
                               seed_covariance=seed_covariance)
        return {'pose': pose, 'mean_dist_m': best, 'inlier_5cm': float(inliers.mean()),
                'score_basis': 'bidirectional_common_dock_support',
                'reference_inlier_5cm': float(reference_inliers.mean()),
                'current_inlier_count': int(np.count_nonzero(inliers)),
                'reference_inlier_count': int(np.count_nonzero(reference_inliers)),
                'minimum_matched_points': minimum_matched_points,
                'uniqueness_basis': 'fixed_bidirectional_inlier_support',
                'fixed_support_mean_m': fixed_support_mean,
                'uniqueness_trials': uniqueness_trials,
                'ambiguous_translation_trials': ambiguous_translations,
                'seed_covariance': seed_covariance,
                'global_agrees': False, 'basis': basis,
                'reference': reference, 'relative_pose': [float(v) for v in relative],
                'click': [click['x_m'], click['y_m'], click['yaw_rad']],
                'click_offset_m': math.dist(pose[:2], (click['x_m'], click['y_m'])),
                'click_offset_deg': math.degrees(wrap(pose[2] - click['yaw_rad']))}
    except (OSError, ValueError, KeyError, TypeError, StopIteration) as error:
        return reject('dock_reference_read_or_calculation_error', detail=str(error))


# The cycle runs on the Pi; a PC that loses ssh keeps polling this long before it
# gives up following (the robot is not stopped by that).
LINK_LOST_GIVE_UP_S = 600.0


def poll_pi(command):
    """Run a read on the Pi; None when ssh itself failed or hung (exit 255 or timeout)."""
    try:
        result = pi(command, timeout=SSH_READ_TIMEOUT_S, check=False)
    except subprocess.TimeoutExpired:
        return None
    return None if result.returncode == 255 else result


def read_events(pi_path):
    result = poll_pi(f'cat {shlex.quote(pi_path)}')
    text = result.stdout if result is not None else ''
    events = []
    for line in text.splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return events


def cmd_init(args):
    override = getattr(args, 'min_inlier', None)
    # argparse accepts 'nan' and 'inf'; fail before any ssh or state change.
    if override is not None and not (math.isfinite(override)
                                     and MATCH_OVERRIDE_FLOOR <= override <= 1.0):
        fail(f'--min-inlier must be between {MATCH_OVERRIDE_FLOOR} and 1.0, got {override}')
    state = load_state()
    if (args.seed_from_state and args.keep_home
            and state.get('init_pose_source') in (
                'anchored_dock_scan', 'registered_dock_local_map')):
        fail('the previous init already used the registered dock reference; omit '
             '--seed-from-state to keep that reference, or save a fresh RViz click and use '
             '--click --keep-home if the robot is now elsewhere')
    # Until this init completes, go must not depart on an earlier localization.
    state['localized'] = False
    save_state(state)
    refuse_during_cycle('init')
    if not session_active():
        fail('Nav2 session is not active; run session-start first')
    if args.seed_from_state:
        # Same placement only: the scan match below must agree globally with this seed.
        if 'pose' not in state:
            fail('no previous init pose; click 2D Pose Estimate in RViz')
        x, y, yaw = state['pose']
        click = {'x_m': x, 'y_m': y, 'yaw_rad': yaw, 'received_unix_s': time.time(),
                 'source': 'previous_init_scan_match'}
    elif getattr(args, 'click', False):
        if not CLICK.exists():
            fail(f'no RViz 2D Pose Estimate click saved yet ({CLICK})')
        click = json.loads(CLICK.read_text())
    else:
        # A new run starts at the dock: seed with the registered dock and keep it, so the
        # dock does not drift to wherever the last docking stopped.
        home = pull_registry().get('home') or state.get('home')
        if home is None:
            fail('no registered dock yet; run init --click at the dock once')
        click = {'x_m': home['x_m'], 'y_m': home['y_m'], 'yaw_rad': home['yaw_rad'],
                 'received_unix_s': time.time(), 'source': 'registered_dock'}
        args.keep_home = True
    age = time.time() - click['received_unix_s']
    if age > CLICK_MAX_AGE_S:
        fail(f'RViz click is {age:.0f} s old; click 2D Pose Estimate again')
    run = time.strftime('%Y%m%d_%H%M%S')
    run_dir = P2 / 'runs' / f'init_{run}'
    pi_run_dir = pi_path(run_dir)
    run_dir.mkdir(parents=True)
    pi(f'mkdir -p {shlex.quote(str(pi_run_dir))}')
    log(f'click ({click["x_m"]:.3f}, {click["y_m"]:.3f}, '
        f'{math.degrees(click["yaw_rad"]):.1f} deg), age {age:.0f} s')
    capture_scan(run_dir / 'scan.json')
    anchor_diagnostic = {}
    refined = refine_at_registered_dock(
        run_dir / 'scan.json', click, state.get('dock_scan_reference'),
        diagnostics=anchor_diagnostic)
    (run_dir / 'dock_anchor_diagnostic.json').write_text(
        json.dumps(anchor_diagnostic, indent=1))
    dock_reference_used = refined is not None
    if refined is None:
        log('dock anchor: ' + anchor_diagnostic.get('reason', 'not_available'))
        refined = refine_pose(run_dir / 'scan.json', click)
    pose = refined['pose']
    basis = ('bounded dock-prior scan' if refined.get('ambiguous_translation_trials')
             else 'anchored dock scan' if dock_reference_used else 'map scan')
    global_result = ('not evaluated for dock scan' if dock_reference_used else
                     'not required for registered dock'
                     if refined.get('global_search_performed') is False else
                     refined['global_agrees'])
    log(f'{basis} match ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg) '
        f'inlier {refined["inlier_5cm"]:.2f}, click offset {refined["click_offset_m"]:.2f} m / '
        f'{refined["click_offset_deg"]:.1f} deg, global agrees {global_result}')
    # --local-only: the operator stated the placement (e.g. pulled straight back and
    # turned to face a box); nearby boxes can outrank it globally. AMCL agreement stays.
    min_inlier = 0.35 if args.local_only else MATCH_MIN_INLIER
    if override is not None:
        # Operator override for a cluttered spot full of unmapped objects (2026-10-02
        # 18:34: 0.349 twice at the same pose); never below MATCH_OVERRIDE_FLOOR.
        if not args.local_only or override < MATCH_OVERRIDE_FLOOR:
            fail(f'--min-inlier needs --local-only and >= {MATCH_OVERRIDE_FLOOR}')
        min_inlier = override
    # A stated dock placement already supplies the global location. Keep the
    # normal map inlier bound and the anchor's existing 20 cm / 15 deg window;
    # an unrelated global winner must not veto this bounded local match.
    dock_local_match = (
        ACTIVE_MAP_SESSION is not None and click['source'] == 'registered_dock'
        and refined['inlier_5cm'] >= MATCH_MIN_INLIER
        and refined['click_offset_m'] <= .2
        and abs(refined['click_offset_deg']) <= 15.)
    refined['registered_dock_local_match'] = dock_local_match
    refined['min_inlier_used'] = None if dock_reference_used else min_inlier
    refined['threshold_basis'] = (refined['basis'] if dock_reference_used else
                                  'registered_dock_local_map' if dock_local_match else 'map_scan')
    (run_dir / 'refine.json').write_text(json.dumps(refined, indent=1))
    if not dock_reference_used and (
            refined['inlier_5cm'] < min_inlier
            or not (refined['global_agrees'] or args.local_only or dock_local_match)):
        fail('scan does not match the map near the click; check placement or click again')
    initial = {'frame_id': 'map', 'x_m': pose[0], 'y_m': pose[1], 'yaw_rad': pose[2],
               **refined.get('seed_covariance', INIT_COVARIANCE),
               'provenance': {'purpose': ('localization_seed_from_anchored_dock_scan'
                                          if dock_reference_used else
                                          'localization_seed_from_scan_match'),
                              'click': refined['click'], 'refine': 'refine.json',
                              'covariance_basis': (
                                  'seed spread plus bounded dock translation ambiguity, not measured'
                                  if refined.get('ambiguous_translation_trials') else
                                  'seed spread (5 cm, 5 deg), not measured')}}
    (run_dir / 'initial_pose.yaml').write_text(yaml.safe_dump(initial, sort_keys=False))
    sh(f'rsync -a {shlex.quote(str(run_dir))}/ {HOST}:{shlex.quote(str(pi_run_dir))}/')
    log('activating navigation (no motion) with the scan-matched seed')
    result = pi(f'{PI_SOURCE}; cd {PI_WS}; python3 -m jdamr_cube_navigation.activate_navigation '
                f'--registry {PI_REGISTRY} --parking-contract {PI_BOX_CONTRACT} '
                f'--log {pi_run_dir}/activation.jsonl '
                f'--initial-pose {pi_run_dir}/initial_pose.yaml',
                timeout=240, check=False)
    events = read_events(str(pi_run_dir / 'activation.jsonl'))
    activated = [e for e in events if e.get('event') == 'navigation_activated_without_motion']
    if result.returncode != 0 or not activated:
        reasons = [e.get('reason') for e in events if e.get('event') == 'activation_failed']
        fail(f'activation failed: {reasons or result.stdout[-400:]}')
    amcl = activated[-1]['stationary_pose']
    dxy, dyaw = math.dist(amcl[:2], pose[:2]), wrap(amcl[2] - pose[2])
    log(f'AMCL ({amcl[0]:.3f}, {amcl[1]:.3f}, {math.degrees(amcl[2]):.1f} deg) vs scan match: '
        f'{dxy:.3f} m, {math.degrees(dyaw):.1f} deg')
    if dxy > AMCL_AGREE_XY_M or abs(dyaw) > AMCL_AGREE_YAW_RAD:
        fail('AMCL disagrees with the scan match after activation; do not depart')
    if not args.keep_home and ACTIVE_MAP_SESSION is None:
        pi(f'{PI_SOURCE}; cd {PI_WS}; python3 -m jdamr_cube_navigation.restaurant_service '
           f'teach-home --registry {PI_REGISTRY} --log {pi_run_dir}/teach_home.jsonl '
           '--pose-id home_dock --approach-offset-m 0.7 --parking-direction reverse --replace',
           timeout=90)
    registry = pull_registry()
    home = registry.get('home')
    if home is None:
        fail('registry has no home; run init without --keep-home')
    log(f'home_dock ({home["x_m"]:.3f}, {home["y_m"]:.3f}, '
        f'{math.degrees(home["yaw_rad"]):.1f} deg)')
    write_routes((home['x_m'], home['y_m'], home['yaw_rad']))
    route_paths = [P2 / f'{VIA_ID}_route.yaml']
    route_paths.extend(P2 / f'{table_id}{suffix}_route.yaml'
                       for table_id in TABLES for suffix in ('', '_direct'))
    sh(f'rsync -a {" ".join(shlex.quote(str(path)) for path in route_paths)} '
       f'{HOST}:{shlex.quote(str(PI_P2))}/')
    # Box candidates: persistent unmapped LiDAR returns near each marker.
    raw, n_scans = dock_survey.raw_points_map(str(run_dir / 'scan.json'), pose)
    img, occ, _free, res, origin = scan_match.load_map(str(MAP_YAML))
    import cv2
    import numpy as np
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    groups = dock_survey.clusters(raw, n_scans, dist, res, origin, *occ.shape)
    regions = {}
    for table_id, spec in DESTINATIONS.items():
        marker = spec['face_xy']
        near = sorted((g for g in groups
                       if math.dist(g['centroid'], marker) <= UNMAPPED_REGION_MAX_M
                       and BOX_FACE_LENGTH_M[0] <= g['length_m'] <= BOX_FACE_LENGTH_M[1]),
                      key=lambda g: math.dist(g['centroid'], marker))
        center = near[0]['centroid'] if near else list(marker)
        regions[table_id] = {'xy': center, 'radius_m': REGION_RADIUS_M,
                             'basis': ('unmapped LiDAR cluster near marker' if near
                                       else 'marker (no box-sized unmapped LiDAR cluster within 0.5 m)'),
                             'candidates': near[:3]}
        log(f'{table_id} region ({center[0]:.3f}, {center[1]:.3f}) r={REGION_RADIUS_M}: '
            f'{regions[table_id]["basis"]}')
    (run_dir / 'regions.json').write_text(json.dumps(regions, indent=1))
    state.update({'init_run': str(run_dir), 'pose': pose, 'amcl': amcl, 'home': home,
                  'regions': regions, 'init_time': time.time(), 'localized': True,
                  'init_local_only': bool(args.local_only),
                  'init_pose_source': refined['threshold_basis']})
    if (click['source'] == 'registered_dock' and not dock_reference_used
            and ACTIVE_MAP_SESSION is None):
        state['dock_scan_reference'] = {
            'scan_path': str(run_dir / 'scan.json'), 'pose_map': list(pose),
            'home_pose': [home['x_m'], home['y_m'], home['yaw_rad']],
            'map_identity': dock_map_identity()}
    save_state(state)
    log('init complete: robot localized at the scan-matched placement, '
        'navigation active, no motion sent')


# ---------------------------------------------------------------- go
def map_wait_failed(events):
    """Only the executor's first check ran and it received no map: nothing moved."""
    events = [event for event in events if not (
        event.get('event') == 'executor_failed' and event.get('code') == 1
        and event.get('reason') == 'executor_exit_nonzero')
        and not (event.get('event') == 'operator_call'
                 and str(event.get('reason', '')).startswith('failed: ' + MAP_WAIT_FAILURE))
        and not (event.get('event') == 'box_camera' and event.get('action') == 'stop'
                 and event.get('ok') is True)]
    return (len(events) == 1 and events[0].get('event') == 'failed'
            and str(events[0].get('reason', '')).startswith(MAP_WAIT_FAILURE))


def controller_wait_failed(events):
    """Recover only an unavailable controller proven to fail before any route goal."""
    allowed = {'station_reference_configured', 'observation_route_selected',
               'failed', 'operator_call',
               'box_camera', 'executor_failed'}
    failures = [event for event in events if event.get('event') == 'failed']
    if len(failures) != 1 or failures[0].get('phase') != 'preflight':
        return False
    reason = str(failures[0].get('reason', ''))
    if not reason.startswith((
            'controller_server parameter service unavailable',
            'controller_server parameter read timed out:')):
        return False
    completion = [event for event in events if event.get('event') == 'executor_failed']
    if len(completion) != 1 or completion[0].get('code') != 1 or \
            completion[0].get('reason') != 'executor_exit_nonzero':
        return False
    return all(event.get('event') in allowed and not (
        event.get('event') == 'box_camera' and not (
            event.get('action') == 'stop' and event.get('ok') is True))
        for event in events)


def recover_controller_before_departure(started):
    """Refresh a hung Nav2 session once while the original dock departure is unmoved."""
    stop_requested_since(started)
    refuse_during_cycle('controller connection recovery before motion')
    cmd_session_stop(None)
    stop_requested_since(started)
    cmd_session_start(None)
    stop_requested_since(started)
    cmd_init(SimpleNamespace(keep_home=True, local_only=False, min_inlier=None,
                             seed_from_state=False, click=False))
    stop_requested_since(started)


def show_event(event):
    # Historical runs may still contain removed notification events.
    if event.get('event') == 'operator_call':
        return
    keep = {k: event[k] for k in ('event', 'level', 'category', 'phase', 'reason', 'waypoint_id',
                                  'terminal_status_code', 'nav2_error_code') if k in event}
    log('  ' + json.dumps(keep, ensure_ascii=False))


def execution_failure_cause(events):
    """Keep the action failure instead of replacing it with the process exit code."""
    for event in reversed(events):
        if event.get('event') == 'failed' and event.get('reason'):
            return str(event['reason'])
        if (event.get('event') == 'precision_straight_finished'
                and event.get('target_failure')):
            return str(event['target_failure'])
        if (event.get('event') in ('straight_drive_stopped', 'rear_box_coarse_paused')
                and event.get('reason') not in (None, 'rear box coarse handoff ready')):
            return str(event['reason'])
    return None


def run_cycle(args, state, table_id):
    """Start one box_service run on the Pi and follow its log until the unit ends."""
    requested_s = getattr(args, '_request_started_monotonic_s', time.monotonic())
    region = station_region(state, table_id)
    surface_mode = getattr(args, 'surface_mode', 'table_edge')
    rear_box_only = getattr(args, 'rear_box_only', False)
    include_via = (not rear_box_only and not args.dock_only and not args.skip_via
                   and table_id != VIA_ID)
    via = station_region(state, VIA_ID) if include_via else None
    if args.region:
        region = {'xy': args.region[:2], 'radius_m': args.region[2],
                  'basis': f'operator-confirmed {surface_mode}'}
    run = time.strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8]
    run_dir = P2 / 'runs' / f'{table_id}_{run}'
    pi_run_dir = pi_path(run_dir)
    run_dir.mkdir(parents=True)
    record_diagnostics = getattr(args, 'record_diagnostics', False)
    bag_unit, monitors = None, None
    if record_diagnostics:
        pi(f'mkdir -p {shlex.quote(str(pi_run_dir))}')
        # Start before the request so the bag includes the still pre-departure stretch.
        bag_unit = start_onboard_bag(run_dir, run)
        monitors = monitor_units(run)
        early = load_state()
        early.update({'onboard_bag_unit': bag_unit, 'onboard_bag_run': str(run_dir),
                      'monitor_units': monitors})
        save_state(early)
    try:
        route_suffix = '' if include_via else '_direct'
        route = (Path(args.route) if args.route
                 else P2 / f'{table_id}{route_suffix}_route.yaml')
        # Refreshed by every poll below; the executor starts no further goal once it is
        # older than 30 s (the PC stop is the only remote stop without a physical one).
        heartbeat = f'{EXECUTOR_SPOOL}/{run}.heartbeat'
        pi(f'mkdir -p {shlex.quote(str(pi_run_dir))} {EXECUTOR_SPOOL} && touch {heartbeat}',
           timeout=SSH_READ_TIMEOUT_S)
        pi_route = pi_path(route)
        options = (f'--registry {PI_REGISTRY} --approach-route {pi_route} '
                   f'--camera-mount {PI_MOUNT} '
                   f'--geometry {PI_GEOMETRY} --parking-contract {PI_BOX_CONTRACT} '
                   f'--table-id {table_id} --region-xy {region["xy"][0]} {region["xy"][1]} '
                   f'--region-radius-m {region["radius_m"]} '
                   f'--log {pi_run_dir}/cycle_events.jsonl '
                   f'--candidate-trial --execute --task-timeout-s {TASK_TIMEOUT_S} '
                   + ('--rear-box-only ' if rear_box_only else
                      '--search ' if getattr(args, 'park_only', False) else
                      f'--search --return-home --return-timeout-s {RETURN_TIMEOUT_S} ')
                   + ('--camera-always-on ' if CAMERA_TRANSPORT == 'windows_tcp' else '')
                   + f'--operator-heartbeat {heartbeat} '
                   f'--surface-mode {surface_mode} '
                   + (' --home-only ' if args.dock_only else '')
                   + (' --graceful-final ' if args.graceful_final else '')
                   + (' --rpp-transit ' if getattr(args, 'rpp_transit', False) else '')
                   + (' --pointcloud-perception '
                      if getattr(args, 'pointcloud_perception', False) else '')
                   + (f' --station-profiles {shlex.quote(str(pi_path(STATION_PROFILES)))} '
                      if STATION_PROFILES is not None else '')
                   + (f' --transit {args.transit} ' if getattr(args, 'transit', None) else '')
                   + (f' --final-gap-m {args.final_gap_m} '
                      if getattr(args, 'final_gap_m', None) is not None else '')
                   + (f' --dwell-s {args.dwell_s} '
                      if getattr(args, 'dwell_s', None) is not None else '')
                   + (' --zero-turn-final ' if getattr(args, 'zero_turn_final', False) else '')
                   + (' --resume-at-observation ' if args.resume_at_observation else '')
                   + (f' --reapproach-m {args.reapproach_m} '
                      if getattr(args, 'reapproach_m', None) is not None else '')
                   + (f' --station-left-correction-m {args.station_left_correction_m} '
                      if getattr(args, 'station_left_correction_m', 0.0) else '')
                   + (f' --reapproach-from-log {shlex.quote(str(pi_path(args.reapproach_from_log)))} '
                      if getattr(args, 'reapproach_from_log', None) else '')
                   + (f' --resume-parked-from-log {shlex.quote(args.resume_parked_log)} '
                      if args.resume_parked_log else '')
                   + ('' if not include_via else
                      f'--via-id {VIA_ID} '
                      f'--via-route {pi_path(args.via_route or P2 / (VIA_ID + "_route.yaml"))} '
                      f'--via-region-xy {via["xy"][0]} {via["xy"][1]} '
                      f'--via-region-radius-m {via["radius_m"]}'))
        completion_error = None
        if not start_executor():
            fail('executor unavailable; no departure request was submitted')
        # The resident executor already knows the graph: no rediscovery per run.
        unit = f'executor:{run}'
        (run_dir / 'command.txt').write_text(f'{EXECUTOR_UNIT} request: {options}\n')
        request = json.dumps({'argv': shlex.split(options)})
        prefix = f'{EXECUTOR_SPOOL}/{run}'
        # Publish and an absent-request cancellation use the same lock: a late SSH
        # command cannot dispatch motion after the PC has reported no submission.
        publish = (f'set -e; '
                   f'if test -e {prefix}.cancelled; then rm -f -- {prefix}.part; exit 1; fi; '
                   f'mv {prefix}.part {prefix}.request')
        fresh = load_state()
        fresh.update({'last_unit': unit, 'last_run': str(run_dir)})
        save_state(fresh)
        submission_uncertain = False
        try:
            submitted = pi(f'cat > {prefix}.part && '
                           f'flock -x {prefix}.lock bash -c {shlex.quote(publish)}',
                           stdin=request, timeout=SSH_READ_TIMEOUT_S, check=False)
            submission_uncertain = submitted.returncode != 0
        except subprocess.TimeoutExpired:
            submission_uncertain = True
        if submission_uncertain:
            log('submission reply unavailable; following this run ID without resubmitting')

        def running():
            nonlocal completion_error
            status = (f'touch {heartbeat}; '
                      f'if test -e {prefix}.result; then cat {prefix}.result; '
                      f'elif test -e {prefix}.request || test -e {prefix}.running; then '
                      f'if systemctl is-active --quiet {EXECUTOR_UNIT}; then echo running; '
                      f'else rm -f -- {prefix}.request; echo executor_unavailable; fi; '
                      f'else touch {prefix}.cancelled; echo request_missing; fi')
            result = poll_pi(f'flock -x {prefix}.lock bash -c {shlex.quote(status)}')
            if result is None:
                return None
            if result.returncode == 0 and result.stdout.strip() == 'running':
                return True
            if result.stdout.strip() in ('executor_unavailable', 'request_missing'):
                completion_error = {'event': 'executor_failed',
                                    'reason': result.stdout.strip(), 'code': None}
                return False
            try:
                document = json.loads(result.stdout)
                code = document.get('code') if isinstance(document, dict) else None
                if result.returncode != 0 or type(code) is not int or not 0 <= code <= 255:
                    raise ValueError('invalid executor exit code')
            except (ValueError, TypeError):
                completion_error = {'event': 'executor_failed',
                                    'reason': 'executor_result_invalid', 'code': None}
            else:
                if code != 0:
                    completion_error = {'event': 'executor_failed',
                                        'reason': 'executor_exit_nonzero', 'code': code}
            return False
        if record_diagnostics:
            try:
                start_drop_monitor(run_dir, run)
            except Exception as error:  # diagnostics must not end run tracking mid-drive
                log(f'monitor start failed ({error}); the run continues without it')
        stops = ('rear box only' if rear_box_only else 'dock only' if args.dock_only else
                 f'{VIA_ID} -> {table_id}' if include_via else f'{table_id}')
        submission_label = 'SUBMISSION_UNCONFIRMED' if submission_uncertain else 'SUBMITTED'
        ending = ' (hold at station)' if getattr(args, 'park_only', False) else ' -> dock'
        log(f'{submission_label} {stops}{ending}: unit {unit}, log {run_dir}/cycle_events.jsonl, '
            f'request_submit_s={time.monotonic() - requested_s:.3f}')
        seen, bag_checked, departed_s, lost_since = 0, False, time.monotonic(), None
        first_event_reported = False

        def report_new(events):
            for event in events[seen:]:
                show_event(event)

        while True:
            # The recorder needs ~13 s to subscribe on a loaded Pi (2026-10-01).
            if (record_diagnostics and not bag_checked
                    and time.monotonic() - departed_s >= ONBOARD_BAG_CHECK_S):
                bag_checked = True
                log('onboard bag: ' + (onboard_bag_bytes(run_dir) or 'nothing written yet'))
            events = read_events(str(pi_run_dir / 'cycle_events.jsonl'))
            if events and not first_event_reported:
                first_event_reported = True
                log(f'executor first event after {time.monotonic() - requested_s:.3f} s')
            report_new(events)
            # A failed read returns nothing; keep the count so nothing is repeated.
            seen = max(seen, len(events))
            alive = running()
            if alive is None:
                # 2026-10-02 inventory: a failed ssh read used to end the follow-up
                # (monitor and bag stopped, go failed) while the robot drove on.
                if lost_since is None:
                    lost_since = time.monotonic()
                    log('ssh to the Pi failed; the cycle runs on the Pi regardless, still polling')
                elif time.monotonic() - lost_since > LINK_LOST_GIVE_UP_S:
                    fail(f'no ssh to the Pi for {LINK_LOST_GIVE_UP_S:.0f} s; the robot may still '
                         'be in its cycle: run status (and stop or estop) once the link is back')
                time.sleep(1.0 if not seen else 10.0)
                continue
            if lost_since is not None:
                log(f'ssh to the Pi back after {time.monotonic() - lost_since:.0f} s')
                lost_since = None
            if not alive:
                # The last lines may land between the read above and this check.
                final = read_events(str(pi_run_dir / 'cycle_events.jsonl'))
                report_new(final)
                if completion_error is not None:
                    final.append(completion_error)
                    show_event(completion_error)
                if record_diagnostics:
                    stop_monitors(monitors)
                    finish_onboard_bag(bag_unit, run_dir)
                return final
            time.sleep(1.0 if not seen else 10.0)
    except BaseException:
        # Ctrl-C, ssh or request failures: close and copy what was recorded.
        if record_diagnostics:
            try:
                stop_monitors(monitors)
            finally:
                finish_onboard_bag(bag_unit, run_dir)
        raise


def start_onboard_bag(run_dir, run):
    """Record the Pi's raw odom, IMU and scan for this run (bounded, no motion)."""
    unit = f'jdamr-onboard-bag-{run.replace("_", "-")}'
    bag = shlex.quote(str(pi_path(run_dir / 'onboard_bag')))
    # A 1 MB cache flushes every few seconds; the default 100 MB held a whole run.
    command = (f'{PI_SOURCE}; exec ros2 bag record -s mcap --max-cache-size 1000000 '
               f'-o {bag} --topics ' + ' '.join(ONBOARD_TOPICS))
    pi(f'sudo -n systemd-run --unit={unit} --collect --property=User={PI_USER} '
       '--property=KillSignal=SIGINT --property=TimeoutStopSec=30 '
       '--property=RuntimeMaxSec=3600 --property=MemoryMax=400M '
       f'--setenv=HOME={PI_HOME} --setenv=ROS_LOG_DIR={PI_HOME}/.ros/log '
       f'--working-directory={PI_WS} /bin/bash -c {shlex.quote(command)}', check=False)
    return unit


def onboard_bag_bytes(run_dir):
    """Return the bytes written so far as text, or '' when the recorder wrote nothing."""
    bag = shlex.quote(str(pi_path(run_dir / 'onboard_bag')))
    out = pi(f'du -cb {bag}/*.mcap 2>/dev/null | tail -1', check=False).stdout.split()
    return f'{out[0]} bytes' if out and out[0].isdigit() and int(out[0]) > 4096 else ''


def finish_onboard_bag(unit, run_dir):
    """Close the onboard bag and copy the whole run directory to the PC."""
    pi(f'sudo -n systemctl stop {unit}', timeout=60, check=False)
    bag = shlex.quote(str(pi_path(run_dir / 'onboard_bag')))
    # The 11:52 PC bag closed without metadata.yaml; reindex rebuilds it from the mcap.
    pi(f'test -e {bag}/metadata.yaml || ({PI_SOURCE}; ros2 bag reindex {bag} -s mcap)',
       timeout=120, check=False)
    sh(f'rsync -a --partial {HOST}:{shlex.quote(str(pi_path(run_dir)))}/ '
       f'{shlex.quote(str(run_dir))}/',
       timeout=600, check=False)
    copied = (run_dir / 'onboard_bag' / 'metadata.yaml').exists()
    log(f'onboard bag: {run_dir / "onboard_bag"} ({"copied" if copied else "not copied"})')
    state = load_state()
    if state.get('onboard_bag_unit') == unit:
        # Finished: a later stop has no recorder to close.
        state.pop('onboard_bag_unit')
        state.pop('onboard_bag_run', None)
        save_state(state)


def monitor_units(run):
    """Names of the per-run monitor units (UDP drops, load profile), space separated."""
    tag = run.replace('_', '-')
    return f'jdamr-udp-drops-{tag} jdamr-load-profile-{tag}'


def stop_monitors(units):
    """Stop the per-run monitor units and forget them in the state."""
    pi(f'sudo -n systemctl stop {units}', check=False)
    state = load_state()
    if state.get('monitor_units') == units:
        state.pop('monitor_units')
        save_state(state)


def start_drop_monitor(run_dir, run):
    """Sample per-process UDP drops on the Pi for this run (read-only, bounded)."""
    unit, profile = monitor_units(run).split()
    script = shlex.quote(f'{PI_TOOLS}/udp_drop_monitor.py')
    pi(f'mkdir -p {PI_TOOLS} && cat > {script}', stdin=(TOOLS / 'udp_drop_monitor.py').read_text(),
       check=False)
    pi(f'sudo -n systemd-run --unit={unit} --collect --property=User={PI_USER} '
       f'--property=RuntimeMaxSec=3600 /usr/bin/python3 {script} '
       f'{shlex.quote(str(pi_path(run_dir / "udp_drops.jsonl")))} 5', check=False)
    # Per-process CPU, memory and temperature beside it: does the Pi 4 keep up (10-05)?
    profiler = shlex.quote(f'{PI_TOOLS}/pi_load_profile.py')
    pi(f'cat > {profiler}', stdin=(TOOLS / 'pi_load_profile.py').read_text(), check=False)
    pi(f'sudo -n systemd-run --unit={profile} --collect --property=User={PI_USER} '
       f'--property=RuntimeMaxSec=3600 --property=MemoryMax=100M /usr/bin/python3 {profiler} '
       f'{shlex.quote(str(pi_path(run_dir / "load_profile.jsonl")))} 2', check=False)


def stop_requested_since(started):
    """An operator stop ends this go: no recovery, no re-departure."""
    if load_state().get('stop_unix', 0.0) >= started:
        fail('stop requested; no re-departure')


def cmd_go(args):
    """One cycle per table: `go table_01 table_02` docks and re-inits between them."""
    # The executor's own bounds, checked before any recorder or monitor starts.
    gap = getattr(args, 'final_gap_m', None)
    if gap is not None and not (math.isfinite(gap) and 0.01 <= gap <= 0.10):
        fail('--final-gap-m must be in [0.01, 0.10] m')
    dwell = getattr(args, 'dwell_s', None)
    if dwell is not None and not (math.isfinite(dwell) and 0.0 <= dwell <= 120.0):
        fail('--dwell-s must be in [0, 120] s')
    surface_mode = getattr(args, 'surface_mode', 'table_edge')
    with_via = getattr(args, 'with_via', False)
    if with_via and args.skip_via:
        fail('--with-via cannot be combined with --skip-via')
    if with_via and surface_mode != 'table_edge':
        fail('--with-via is only valid with --surface-mode table_edge')
    tables = list(getattr(args, 'table_ids', None) or [args.table_id])
    reapproach_m = getattr(args, 'reapproach_m', None)
    correction_m = getattr(args, 'station_left_correction_m', 0.0)
    if reapproach_m is not None:
        if not (math.isfinite(reapproach_m) and 0.2 <= reapproach_m <= 0.45
                and len(tables) == 1 and args.park_only and surface_mode == 'table_edge'
                and not (args.dock_only or args.rear_box_only or with_via
                         or args.resume_parked_log or tables[0] != VIA_ID and not args.skip_via)):
            fail('--reapproach-m requires one --park-only table_edge station, 0.2–0.45 m')
    if (not math.isfinite(correction_m) or abs(correction_m) > 0.05
            or correction_m and reapproach_m is None):
        fail('--station-left-correction-m requires --reapproach-m and at most 0.05 m')
    if getattr(args, 'reapproach_from_log', None) is not None and reapproach_m is None:
        fail('--reapproach-from-log requires --reapproach-m')
    if getattr(args, 'rear_box_only', False) and (
            len(tables) != 1 or args.dock_only or with_via or args.resume_parked_log):
        fail('--rear-box-only requires one standalone parking attempt')
    if len(tables) > 1 and (args.skip_via or args.resume_at_observation
                            or args.resume_parked_log
                            or args.dock_only or args.route or args.region):
        fail('a sequence starts every cycle at the dock; resume, route and region options '
             'apply to a single table')
    sequence_started = time.time()
    for index, table_id in enumerate(tables):
        if index:
            # Charging is connected by hand and is not detected; --dock-wait-s adds a
            # charging stop before the standard dock init (no motion).
            wait_s = float(getattr(args, 'dock_wait_s', DOCK_WAIT_S))
            log(f'docked after {tables[index - 1]}; {table_id} departs after a fresh init '
                f'(no motion){f" and {wait_s:.0f} s at the dock" if wait_s > 0 else ""}')
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))
                stop_requested_since(sequence_started)
            cmd_init(SimpleNamespace(seed_from_state=False, click=False, local_only=False,
                                     keep_home=True))
            stop_requested_since(sequence_started)
        go_one(args, table_id)
    if len(tables) > 1:
        log(f'sequence complete: {" -> dock -> ".join(tables)} -> dock')


def go_one(args, table_id):
    """Run one dock-to-dock cycle for table_id; fail unless it ends confirmed at the dock."""
    started = time.time()
    args._request_started_monotonic_s = time.monotonic()
    state = load_state()
    if 'regions' not in state or state.get('localized') is not True:
        fail('not localized: the session restarted or an init failed since the last init; '
             'run init')
    if not session_active():
        fail('Nav2 session is not active')
    refuse_during_cycle('a new departure')
    args._map_retry_allowed = True
    events = run_cycle(args, state, table_id)
    if map_wait_failed(events):
        # The completed attempt failed before motion. Refresh only its DDS context;
        # keep the current pose, sensors and optional PC displays intact.
        stop_requested_since(started)
        log('no map before motion; restarting only the executor and retrying once')
        restart_executor()
        stop_requested_since(started)
        refuse_during_cycle('the re-departure')
        args._map_retry_allowed = False
        events = run_cycle(args, load_state(), table_id)
        if map_wait_failed(events):
            fail('map still unavailable after one executor restart; no further departure '
                 'or full recovery was started; current localization is preserved')
    fresh_departure = not any(getattr(args, flag, False) for flag in (
        'resume_at_observation', 'resume_at_final_approach', 'resume_parked_log',
        'dock_only', 'rear_box_only', 'route', 'region', 'skip_via'))
    if (fresh_departure and state.get('init_pose_source') in (
            'anchored_dock_scan', 'registered_dock_local_map')
            and state.get('init_local_only') is not True
            and controller_wait_failed(events)):
        log('controller unavailable before motion; restoring only Nav2/executor '
            'and the registered dock localization once')
        recover_controller_before_departure(started)
        refuse_during_cycle('the restored departure')
        events = run_cycle(args, load_state(), table_id)
    names = [e.get('event') for e in events]
    executor_failure = next((e for e in events if e.get('event') == 'executor_failed'), None)
    if executor_failure is not None:
        reason = execution_failure_cause(events) or executor_failure['reason']
        fail(f'execution stopped: {reason} '
             f'(code={executor_failure["code"]}); see the run log')
    if getattr(args, 'rear_box_only', False):
        parked = [e for e in events if e.get('event') == 'rear_box_parking_finished']
        if not parked or parked[-1].get('confirmed') is not True:
            fail('rear box parking not confirmed; see the run log')
        log('후면 박스 1cm 주차 완료(라이다 내부 추정)')
        return
    if getattr(args, 'park_only', False):
        if not any(e.get('event') == 'box_approach_finished'
                   and e.get('table_id') == table_id for e in events):
            fail('selected station parking not confirmed; holding position')
        log(f'{table_id}: 정밀주차 완료(센서 내부 추정), 현 위치 유지')
        return
    arrived = [e for e in events if e.get('event') == 'home_arrived']
    home = bool(arrived) and arrived[-1].get('confirmation', {}).get('confirmed') is True
    log(f'cycle ended: parked={"box_approach_finished" in names} '
        f'dwell={"parked_dwell_complete" in names} escape={"box_escape_finished" in names} '
        f'home={home}')
    if not home:
        fail('the cycle did not end confirmed at the dock; see the last events above')


def dds_health():
    """Read-only probe: returns ('ok' | 'missing' | 'error', report line)."""
    script = (TOOLS / 'dds_probe.py').read_text()
    try:
        result = pi(f'{PI_SOURCE}; timeout 45 python3 - health', stdin=script, timeout=60,
                    check=False)
    except subprocess.TimeoutExpired:
        return 'error', 'probe did not finish within 60 s'
    line = next((text for text in result.stdout.splitlines() if text.startswith('PROBE ')), None)
    if line is None:
        return 'error', f'probe did not run (code {result.returncode}): {result.stdout[-300:]}'
    return ('ok' if line.split()[2:3] == ['ok'] else 'missing'), line


def cmd_health(_args):
    status, line = dds_health()
    log(f'DDS {status}: {line}')
    if status == 'missing':
        fail('fresh processes do not receive map/TF/scan; run recover')
    if status == 'error':
        fail('the probe could not run; check ssh and the Pi ROS environment')


def unit_state(unit):
    text = pi(f'systemctl show -p ActiveState -p ActiveEnterTimestampMonotonic {unit}',
              check=False).stdout
    state = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    return state.get('ActiveState'), state.get('ActiveEnterTimestampMonotonic')


def restart_sensor_services():
    """Restart each running sensor service and confirm it really started again."""
    for unit in SENSOR_UNITS:
        before = unit_state(unit)
        if unit in CAMERA_UNITS and before[0] == 'inactive':
            # Started by the executor for the box stops only.
            log(f'{unit}: inactive (on demand), not restarted')
            continue
        if before[0] != 'active':
            # None means the ssh read failed.
            fail(f'{unit} is {before[0]}; start it before recovering')
        pi(f'sudo -n systemctl restart {unit}', timeout=200, check=False)
        after = unit_state(unit)
        if after[0] != 'active' or after[1] == before[1]:
            fail(f'{unit} did not restart (before {before}, after {after})')
        log(f'{unit}: restarted')
    time.sleep(8)


def cmd_recover(args):
    """Fresh DDS state without motion: sensor services, session, display, then init.

    Without --seed the robot must still be where the last init placed it (the dock
    before a departure): the scan match must then agree globally with that pose.
    --local-only (operator-stated placement) is never implied.
    """
    if args.local_only and args.seed is None:
        fail('--local-only needs --seed: without it the last init pose must agree globally')
    refuse_during_cycle('recover')
    linger = pi(f'loginctl show-user {PI_USER} -p Linger --value', check=False).stdout.strip()
    if linger != 'yes':
        # 2026-09-30 morning: without linger the last ssh logout removed DDS shared memory.
        fail(f'Pi user {PI_USER} linger is {linger or "unknown"}; '
             f'run sudo loginctl enable-linger {PI_USER}')
    mark_localized(False)
    used_display = tuple(u for u in DISPLAY_UNITS
                         if sh(f'systemctl --user is-active {u}', check=False).stdout.strip()
                         == 'active')
    if used_display:
        cmd_display_stop(args)
    cmd_session_stop(args)
    restart_sensor_services()
    if used_display:
        try:
            cmd_display_start(SimpleNamespace(
                _display_units=used_display, record_diagnostics=False))
        except (SystemExit, subprocess.TimeoutExpired, OSError) as error:
            log(f'optional display restore failed ({error}); continuing Pi recovery')
    cmd_session_start(args)
    args.keep_home, args.seed_from_state = True, args.seed is None
    args.click = args.seed is not None
    if args.seed is not None:
        x, y, yaw_deg = args.seed
        CLICK.write_text(json.dumps({'x_m': x, 'y_m': y, 'yaw_rad': math.radians(yaw_deg),
                                     'received_unix_s': time.time(), 'source': 'recover seed'}))
    try:
        cmd_init(args)
    except SystemExit:
        log('recover: services are fresh but init stopped. If the robot is not at the last '
            'init pose, click 2D Pose Estimate and run init --click, or run recover --seed X Y YAW_DEG')
        raise
    status, line = dds_health()
    if status != 'ok':
        fail(f'DDS {status} after recovery ({line})')
    log('DDS OK after recovery: ' + line)


def cmd_stop(_args):
    signal = f'sudo -n systemctl kill --signal=SIGINT {EXECUTOR_UNIT}'
    try:
        first_signal = pi(signal, timeout=SSH_READ_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired):
        first_signal = None
    try:
        state = load_state()
    except (OSError, ValueError, TypeError):
        state = {}
    state['stop_unix'] = time.time()
    try:
        save_state(state)
    except (OSError, ValueError, TypeError) as error:
        log(f'stop state could not be saved: {error}')
    # The robot first: closing the bag (reindex up to 120 s) or the monitors on a
    # slow Pi must not hold back the stop (review 2026-10-06).
    unit = state.get('last_unit')
    if unit and unit.startswith('executor:'):
        prefix = f'{EXECUTOR_SPOOL}/{unit.split(":", 1)[1]}'
        cancel = (f'set -e; touch {prefix}.cancelled; '
                  f'rm -f -- {prefix}.part {prefix}.request')
        # Signal existing motion first; cancel pending/late publication under the
        # claim lock, then signal again to cover a concurrent request claim.
        try:
            result = pi(f'{signal} || true; '
                        f'flock -x {prefix}.lock bash -c {shlex.quote(cancel)} && {signal}',
                        timeout=SSH_READ_TIMEOUT_S, check=False)
        except subprocess.TimeoutExpired:
            fail(f'{unit}: stop reply timed out; cancellation unconfirmed')
        if result.returncode != 0:
            fail(f'{unit}: cancellation unconfirmed (exit {result.returncode})')
        log(f'{unit}: pending request cancelled; stop sent to {EXECUTOR_UNIT}')
    elif unit:
        pi(f'sudo -n systemctl stop {unit}', check=False)
        log(f'{unit}: ' + pi(f'systemctl is-active {unit}', check=False).stdout.strip())
    else:
        if first_signal is None or first_signal.returncode != 0:
            fail('resident executor stop reply unavailable; cancellation unconfirmed')
        log(f'stop sent to {EXECUTOR_UNIT}; no recorded request was available')
    if state.get('onboard_bag_unit'):
        # go closes and copies it; this covers a go that is no longer running.
        finish_onboard_bag(state['onboard_bag_unit'], Path(state['onboard_bag_run']))
    if state.get('monitor_units'):
        stop_monitors(state['monitor_units'])


ESTOP_STATE_READ = ('timeout 8 ros2 topic echo --once --qos-durability transient_local '
                    '--qos-reliability reliable /emergency_stop_state std_msgs/msg/Bool')
# ros2 CLI calls leave a ros2 daemon on domain 12; long-lived participants there have
# blocked Nav2 lifecycle bringup before (2026-09-03; 2026-10-02 10:50 get_state timeout
# with a daemon up since 09:01), so each call here stops it again.
STOP_ROS2_DAEMON = 'ros2 daemon stop >/dev/null 2>&1 || true'


def estop_state():
    """'engaged', 'released', or 'unknown' when the base driver did not answer."""
    out = pi(f'{PI_SOURCE}; {ESTOP_STATE_READ}; {STOP_ROS2_DAEMON}', timeout=30,
             check=False).stdout
    return ('engaged' if 'data: true' in out else 'released' if 'data: false' in out
            else 'unknown')


def cmd_estop(_args):
    """Latch the base driver stop (wheels held at zero until estop-reset), then stop the attempt.

    A software stop over ssh: it needs the Wi-Fi link and takes a few seconds. It is
    not a safety-rated emergency stop.
    """
    pi(f'{PI_SOURCE}; timeout 10 ros2 topic pub --once -w 1 /emergency_stop '
       f'std_msgs/msg/Bool "{{data: true}}"; {STOP_ROS2_DAEMON}', timeout=40, check=False)
    log(f'emergency stop: {estop_state()}')
    cmd_stop(_args)


def cmd_estop_reset(_args):
    """Release the latch; refused while a velocity command is still arriving."""
    out = pi(f'{PI_SOURCE}; timeout 10 ros2 service call /emergency_stop_reset '
             f'std_srvs/srv/Trigger; {STOP_ROS2_DAEMON}', timeout=40, check=False).stdout
    log(('released' if 'success=True' in out else 'NOT released: '
         + (out.strip().splitlines() or ['no answer'])[-1]) + f'; state: {estop_state()}')


def cmd_status(_args):
    state = load_state()
    log('session: ' + pi('systemctl is-active jdamr-restaurant-navigation.service',
                         check=False).stdout.strip())
    for unit in (*DISPLAY_UNITS, RECORD_UNIT):
        log(f'{unit}: ' + sh(f'systemctl --user is-active {unit}', check=False).stdout.strip())
    log(f'{EXECUTOR_UNIT}: ' + pi(f'systemctl is-active {EXECUTOR_UNIT}',
                                  check=False).stdout.strip())
    if state.get('last_unit') and not state['last_unit'].startswith('executor:'):
        log(f'{state["last_unit"]}: ' + pi(f'systemctl is-active {state["last_unit"]}',
                                           check=False).stdout.strip())
    log(f'emergency stop: {estop_state()}')
    log('state: ' + json.dumps({k: state[k] for k in ('pose', 'home', 'last_run', 'bag')
                                if k in state}))


def main():
    context = argparse.ArgumentParser(add_help=False)
    context.add_argument('--map-session', type=Path,
                         default=os.environ.get('JDAMR_MAP_SESSION', str(
                             DATA_ROOT / 'maps/current_manual_mapping.json')))
    context.add_argument('--legacy-map', action='store_true',
                         help='Explicit archival access to the previous map; never selected by default')
    selected, remaining = context.parse_known_args()
    if (selected.map_session is not None and not selected.legacy_map
            and not any(arg in ('--help', '-h') for arg in sys.argv[1:])):
        if remaining and remaining[0] in (
                'stop', 'estop', 'session-stop', 'display-stop', 'status'):
            select_stop_state(selected.map_session)
        elif ACTIVE_MAP_POINTER != selected.map_session.expanduser().resolve():
            configure_map_session(selected.map_session)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     parents=[context])
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('sync', 'display-stop', 'session-start', 'session-stop',
                 'stop', 'status', 'record-start', 'record-stop', 'estop', 'estop-reset'):
        sub.add_parser(name)
    sub.add_parser('prepare', help='write and sync taught-map assets; no motion')
    display = sub.add_parser('display-start')
    display.add_argument('--record-diagnostics', action='store_true',
                         help='also start the optional PC bag recorder')
    init = sub.add_parser('init')
    init.add_argument('--keep-home', action='store_true',
                      help='do not re-teach home at this placement')
    init.add_argument('--local-only', action='store_true',
                      help='operator-stated placement: skip the global agreement check')
    init.add_argument('--min-inlier', type=float,
                      help='with --local-only: accept a lower scan-match inlier ratio '
                           '(cluttered spot; floor 0.25)')
    init.add_argument('--seed-from-state', action='store_true',
                      help='robot not moved since the last init: seed with that pose')
    init.add_argument('--click', action='store_true',
                      help='robot placed away from the registered dock: seed with the RViz click')
    go = sub.add_parser('go')
    go.add_argument('table_ids', nargs='+', choices=TABLES, metavar='table_id',
                    help='one or more tables; several run as dock-to-dock cycles in order')
    go.add_argument('--dock-wait-s', type=float, default=DOCK_WAIT_S,
                    help='wait at the dock between cycles of a sequence (charging stop; default none)')
    go.add_argument('--route',
                    help='route file (default: <table_id>_route.yaml after water station; '
                         '<table_id>_direct_route.yaml with --skip-via)')
    go.add_argument('--region', nargs=3, type=float, metavar=('X', 'Y', 'R'))
    go.add_argument('--via-route', help='water_station route (default: <P2>/water_station_route.yaml)')
    go.add_argument('--skip-via', action='store_true',
                    help='skip the water-station stop and go straight to the selected table')
    go.add_argument('--with-via', action='store_true',
                    help='explicitly include the water-station stop (already the default)')
    go.add_argument('--dock-only', action='store_true', help='return to the dock only (re-dock)')
    go.add_argument('--park-only', action='store_true',
                    help='park at the selected station and hold there without returning home')
    go.add_argument('--reapproach-m', type=float,
                    help='retreat from the current observed table, then approach it again')
    go.add_argument('--station-left-correction-m', type=float, default=0.0,
                    help='this reapproach only: shift the current lateral position left')
    go.add_argument('--reapproach-from-log', type=Path,
                    help='carry the previous confirmed parked face to the retreat')
    go.add_argument('--rear-box-only', action='store_true',
                    help='current-location rear LiDAR box parking, without transit')
    go.add_argument('--surface-mode', choices=('front', 'table_edge'), default='table_edge',
                    help='parking surface (default table_edge; use front for legacy box parking)')
    go.add_argument('--graceful-final', action='store_true',
                    help='Graceful box approach and dock leg instead of RPP (default)')
    go.add_argument('--zero-turn-final', action='store_true',
                    help='final box approach: trim the residual in place, then zero-turn straight')
    go.add_argument('--pointcloud-perception', action='store_true',
                    help='use the optional depth pointcloud observer and selected table region')
    go.add_argument('--rpp-transit', action='store_true',
                    help='RPP on NavFn paths for the transit and staging legs (the default)')
    go.add_argument('--transit', choices=('lattice-mppi', 'navfn-mppi', 'navfn-rpp'),
                    help='planner + controller for the transit and staging legs '
                         '(default navfn-rpp, 2026-10-06 algorithm selection)')
    go.add_argument('--final-gap-m', type=float,
                    help='front gap override (both surface modes use the box parking '
                         'contract target by default)')
    go.add_argument('--dwell-s', type=float,
                    help='optional service hold at each box (default 0 s; longer to measure)')
    go.add_argument('--record-diagnostics', action='store_true',
                    help='record the onboard bag, UDP drops and Pi load for this run')
    go.add_argument('--resume-at-observation', action='store_true',
                    help='robot already at an observation point: the table one skips the '
                         'water stop, the water one resumes there (--skip-via forces the table)')
    go.add_argument('--resume-parked-log',
                    help='cycle_events.jsonl of a run that stopped at the first box: hold, escape, continue')
    sub.add_parser('health')
    recover = sub.add_parser('recover')
    recover.add_argument('--seed', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                         help='placement when the robot is not at the last init pose')
    recover.add_argument('--local-only', action='store_true',
                         help='operator-stated placement: skip the global agreement check')
    routes = sub.add_parser('routes')
    routes.add_argument('--start', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW_RAD'))
    args = parser.parse_args()
    if args.command == 'go' and args.pointcloud_perception and (
            args.surface_mode != 'table_edge' or args.dock_only or args.rear_box_only):
        parser.error('--pointcloud-perception requires a table_edge approach task')
    handlers = {'sync': cmd_sync, 'display-start': cmd_display_start,
                'display-stop': cmd_display_stop, 'session-start': cmd_session_start,
                'session-stop': cmd_session_stop, 'init': cmd_init, 'go': cmd_go,
                'stop': cmd_stop, 'status': cmd_status, 'health': cmd_health,
                'estop': cmd_estop, 'estop-reset': cmd_estop_reset,
                'recover': cmd_recover,
                'record-start': lambda _a: record_start(),
                'record-stop': lambda _a: record_stop(),
                'routes': lambda a: write_routes(a.start)}
    def prepare(args):
        if ACTIVE_MAP_SESSION is None:
            parser.error('prepare requires --map-session')
        home = yaml.safe_load(REGISTRY.read_text())['home']
        write_routes([home[key] for key in ('x_m', 'y_m', 'yaw_rad')])
        cmd_sync(args)
        log('taught map prepared; session-start and init are no-motion setup')
    handlers['prepare'] = prepare
    handlers[args.command](args)


if __name__ == '__main__':
    main()
