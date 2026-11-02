"""Launch composed Nav2 with a separate collision-monitor process."""

# Run the corridor Nav2 subset in a composed container and keep graph-loss
# detection separate.  Composition correlated with lower load in earlier
# runs, but it does not prove intra-process delivery: rclcpp defaults that
# option to false and this launch does not override it.  Run-specific evidence
# and remaining causal uncertainty live in evaluation/20260904_HANDOFF.md.

import hashlib
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.keepout_mask import validate_mask
from jdamr_cube_navigation.mobile_manipulator_protection import (
    load_base_obstacle_protection,
    load_mobile_manipulator_protection,
)
from jdamr_cube_navigation.nav2_liveness_guard import DEFAULT_REQUIRED
from jdamr_cube_navigation.new_base_contract import validate_new_base_params
from jdamr_cube_navigation.service_destinations import load_registry, verify_identity
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


REVISIT_REFERENCE = {
    'map': ('autonomous_20260826T161908.yaml',
            '3ddadf69e8f29a2ac4ec17f2d1bfc71f56cda0e805d65c792ddb5f5d46e0d652',
            'autonomous_20260826T161908.pgm',
            'ee9b0911f41a7da31a92eca67d261b2a96c286f6f49d65b3d6c884fb5937fc6e'),
    'mask': ('autonomous_20260826T161908_keepout_multi.yaml',
             '945b904f254864baab2a61f2cbf602cf69a1c8e8a71da1b7f07c0de84b9344f9',
             'autonomous_20260826T161908_keepout_multi.pgm',
             '40a99481046b2ad55fd4c5d3ad3e8b3a6d245fd7c808d32571f2828d4837df28'),
}


def _validate_keepout(context):
    """Reject missing or misaligned physical map inputs before Nav2 starts."""
    map_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('map').perform(context))))
    mask_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration('keepout_mask').perform(context))))
    if not map_path.is_file():
        raise RuntimeError(f'saved map YAML not found: {map_path}')
    if not mask_path.is_file():
        raise RuntimeError(f'keepout mask YAML not found: {mask_path}')
    validate_mask(mask_path, map_path)
    return []


def _reject_legacy_profile_on_new_base(context):
    """Do not let the direct core launch bypass the physical wrapper."""
    if LaunchConfiguration('use_sim_time').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        return []
    description_share = Path(get_package_share_directory(
        'jdamr_cube_description'))
    new_base_model = description_share / 'urdf' / 'new_base_real.urdf'
    if (new_base_model.is_file()
            and LaunchConfiguration('navigation_profile').perform(context)
            not in ('new_base_candidate', 'new_base_revisit_candidate')):
        raise RuntimeError('new physical base rejects legacy navigation profiles')
    return []


def _validate_revisit_reference(context):
    """Pin the old map and mask used only to collect a new autonomous scan bag."""
    for label, argument in (('map', 'map'), ('mask', 'keepout_mask')):
        path = Path(os.path.expanduser(
            LaunchConfiguration(argument).perform(context))).resolve()
        yaml_name, expected_yaml_hash, image_name, expected_hash = (
            REVISIT_REFERENCE[label])
        if path.name != yaml_name or not path.is_file():
            raise RuntimeError(f'new-base revisit {label} YAML is not pinned')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected_yaml_hash:
            raise RuntimeError(f'new-base revisit {label} YAML hash mismatch')
        metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
        image = Path(metadata['image'])
        if image.is_absolute() or image.name != image_name:
            raise RuntimeError(f'new-base revisit {label} image is not pinned')
        image_path = path.parent / image
        if (not image_path.is_file()
                or hashlib.sha256(image_path.read_bytes()).hexdigest()
                != expected_hash):
            raise RuntimeError(f'new-base revisit {label} image hash mismatch')
    return []


def _validate_revisit_isolation(context):
    """Guard the direct physical launch as well as the autorun entrypoint."""
    if LaunchConfiguration('navigation_profile').perform(context) != (
            'new_base_revisit_candidate'):
        return []
    if LaunchConfiguration('use_sim_time').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        return []
    if os.environ.get('ROS_DOMAIN_ID') != '12':
        raise RuntimeError('new-base revisit requires physical ROS_DOMAIN_ID=12')
    description_share = Path(get_package_share_directory(
        'jdamr_cube_description'))
    if not (description_share / 'urdf' / 'new_base_real.urdf').is_file():
        raise RuntimeError('new-base revisit model is not installed')
    for service in ('jdamr-cartographer-session.service',
                    'jdamr-webteleop.service'):
        state = subprocess.run(['systemctl', 'is-active', '--quiet', service],
                               check=False, timeout=5)
        if state.returncode == 0:
            raise RuntimeError(f'new-base revisit rejects active {service}')
        if state.returncode not in (3, 4):
            raise RuntimeError(f'new-base revisit cannot verify {service}')
    nodes = subprocess.run(['ros2', 'node', 'list', '--no-daemon',
                            '--spin-time', '2'], check=False,
                           capture_output=True, text=True, timeout=15)
    if nodes.returncode != 0:
        raise RuntimeError('new-base revisit cannot read ROS nodes')
    if any('cartographer' in node or 'web_teleop' in node or node in (
            '/amcl', '/map_server', '/collision_monitor', '/bt_navigator')
           for node in nodes.stdout.splitlines()):
        raise RuntimeError('new-base revisit rejects preexisting control nodes')
    topics = subprocess.run(['ros2', 'topic', 'list', '--no-daemon',
                             '--spin-time', '2'], check=False,
                            capture_output=True, text=True, timeout=15)
    if topics.returncode != 0:
        raise RuntimeError('new-base revisit cannot read ROS topics')
    if '/cmd_vel' in topics.stdout.splitlines():
        command = None
        for _ in range(3):
            try:
                command = subprocess.run(
                    ['ros2', 'topic', 'info', '--no-daemon', '--spin-time',
                     '3', '/cmd_vel'], check=False, capture_output=True,
                    text=True, timeout=20)
            except subprocess.TimeoutExpired:
                continue
            if command.returncode == 0 and 'Publisher count:' in command.stdout:
                break
        if command is None or command.returncode != 0 or (
                'Publisher count: 0' not in command.stdout):
            raise RuntimeError('new-base revisit rejects existing cmd_vel publisher')
    return []


def _shutdown_unless_already_stopping(reason):
    """Fail closed on a required process exit without duplicate shutdowns."""
    def handler(_event, context):
        if context.is_shutdown:
            return []
        return [Shutdown(reason=reason)]

    return handler


def _validate_new_base_params(context, revisit=False):
    """Reject old footprint/StopZone parameters for the new physical base."""
    map_name = Path(LaunchConfiguration('map').perform(context)).name
    mask_name = Path(LaunchConfiguration('keepout_mask').perform(context)).name
    if revisit:
        _validate_revisit_reference(context)
    elif not (map_name.startswith('new_base_')
              and mask_name.startswith('new_base_')):
        registry_path = LaunchConfiguration('asset_registry', default='').perform(context)
        if not registry_path:
            raise RuntimeError('new-base profile requires a new-base map and mask')
        registry = load_registry(registry_path)
        verify_identity(registry['map'], LaunchConfiguration('map').perform(context))
        verify_identity(registry['keepout'],
                        LaunchConfiguration('keepout_mask').perform(context))
    params_path = Path(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context)))
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    geometry_path = (Path(get_package_share_directory('jdamr_cube_description'))
                     / 'config' / 'new_base_geometry.yaml')
    geometry = yaml.safe_load(geometry_path.read_text(encoding='utf-8'))
    validate_new_base_params(params, geometry, precision_parking=(
        LaunchConfiguration('precision_parking', default='false').perform(context) == 'true'))
    return []


def _launch_navigation(context):
    """Resolve the profile before selecting its BT and required components."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    profile = LaunchConfiguration('navigation_profile').perform(context)
    behavior_tree = {
        'corridor': 'navigate_to_pose_corridor_fail_fast.xml',
        'obstacle_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
        'obstacle_base_candidate': (
            'navigate_to_pose_dynamic_obstacle_eval.xml'),
        'new_base_candidate': 'navigate_to_pose_dynamic_obstacle_eval.xml',
        'new_base_revisit_candidate': (
            'navigate_to_pose_dynamic_obstacle_eval.xml'),
    }[profile]
    if profile in ('new_base_candidate', 'new_base_revisit_candidate'):
        _validate_new_base_params(
            context, revisit=profile == 'new_base_revisit_candidate')
    box_search = LaunchConfiguration(
        'enable_box_search', default='false').perform(context) == 'true'
    if box_search and (profile not in ('new_base_candidate', 'new_base_revisit_candidate')
                       or LaunchConfiguration('precision_parking', default='false').perform(
                           context) != 'true'):
        raise RuntimeError('box search requires the precision-parking new-base profile')
    map_yaml = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    navigation_autostart = LaunchConfiguration(
        'navigation_autostart', default=autostart)
    coordinated_startup = LaunchConfiguration(
        'coordinated_startup', default='false').perform(context).lower() == 'true'
    if (coordinated_startup
            and navigation_autostart.perform(context) != autostart.perform(context)):
        raise RuntimeError('separate navigation autostart requires independent lifecycle managers')
    selected_bt = os.path.join(package_share, 'behavior_trees', behavior_tree)
    protection = None
    if profile == 'obstacle_candidate':
        protection = load_mobile_manipulator_protection(Path(
            package_share) / 'config' / 'mobile_manipulator_protection.yaml')
    elif profile == 'obstacle_base_candidate':
        protection = load_base_obstacle_protection(Path(
            package_share) / 'config' / 'base_obstacle_protection.yaml')

    param_rewrites = {
        'default_nav_to_pose_bt_xml': selected_bt,
        'yaml_filename': map_yaml,
        ('local_costmap.local_costmap.ros__parameters.'
         'keepout_filter.enabled'): 'true',
        ('global_costmap.global_costmap.ros__parameters.'
         'keepout_filter.enabled'): 'true',
        'use_sim_time': use_sim_time,
    }
    if profile == 'new_base_revisit_candidate':
        # Resume starts at the observed physical pose, not the old home.
        param_rewrites.update({
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x':
                LaunchConfiguration('revisit_initial_x'),
            'amcl.ros__parameters.initial_pose.y':
                LaunchConfiguration('revisit_initial_y'),
            'amcl.ros__parameters.initial_pose.yaw':
                LaunchConfiguration('revisit_initial_yaw'),
        })
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_bond = {
        # Bonds off (as in the mapping launches): on 2026-09-30 a departing
        # executor stalled heartbeats past 10 s on the loaded Pi and the managers
        # shut the stack down (15:54, 16:57). Collision stop, command timeouts and
        # stale-sensor checks are unaffected.
        'bond_timeout': 0.0,
        'bond_respawn_max_duration': 20.0,
    }

    def composable(package, plugin, name, parameters, remap=True):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[*parameters, {'use_sim_time': use_sim_time}],
            remappings=remappings if remap else [],
        )

    keepout_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'keepout_filter_mask_server', [{
                       'use_sim_time': use_sim_time,
                       'yaml_filename': keepout_mask,
                       'topic_name': '/keepout_filter_mask',
                       'frame_id': 'map',
                   }], remap=False),
        composable('nav2_map_server',
                   'nav2_map_server::CostmapFilterInfoServer',
                   'keepout_costmap_filter_info_server', [{
                       'use_sim_time': use_sim_time,
                       'type': 0,
                       'filter_info_topic': '/keepout_costmap_filter_info',
                       'mask_topic': '/keepout_filter_mask',
                       'base': 0.0,
                       'multiplier': 1.0,
                   }], remap=False),
    ]
    collision_monitor_parameters = [configured_params]
    if protection is not None:
        collision_monitor_parameters.append(
            protection['collision_monitor_overrides'])
    nav2_components = [
        composable('nav2_map_server', 'nav2_map_server::MapServer',
                   'map_server', [configured_params,
                                  {'yaml_filename': map_yaml}]),
        composable('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl',
                   [configured_params]),
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_planner', 'nav2_planner::PlannerServer',
                   'planner_server', [configured_params]),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        composable('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                   'bt_navigator', [configured_params]),
    ]
    navigation_nodes = [
        'controller_server', 'planner_server', 'velocity_smoother',
        'collision_monitor', 'bt_navigator',
    ]
    required_nodes = list(DEFAULT_REQUIRED)
    if profile in {'corridor', 'obstacle_candidate', 'obstacle_base_candidate',
                   'new_base_candidate', 'new_base_revisit_candidate'}:
        # Ordinary transit keeps Wait only. Precision service can expose Spin
        # for explicit bounded search actions, never automatic backup/recovery.
        behavior_options = {'behavior_plugins': ['wait'], 'use_sim_time': use_sim_time}
        if box_search:
            behavior_options.update(behavior_plugins=['wait', 'spin'],
                                    spin={'plugin': 'nav2_behaviors::Spin'})
        nav2_components.insert(-1, ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params, behavior_options, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ))
        navigation_nodes.insert(-1, 'behavior_server')
        required_nodes.append('behavior_server')

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        namespace='',
        # Costmaps are child nodes created inside planner/controller, not
        # LoadNode targets.  They inherit process-level ROS arguments, so
        # component parameters alone silently leave them on Nav2 defaults.
        # RewrittenYaml only substitutes existing leaf keys.  The production
        # YAML omits use_sim_time, so pass it explicitly to components and
        # their child nodes instead of relying on that rewrite alone.
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        composable_node_descriptions=keepout_components + nav2_components,
        output='screen',
    )

    # Keep scan processing out of the planner/controller process so planner
    # load cannot delay the monitor callback and cause an invalid-source stop.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[*collision_monitor_parameters,
                    {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    keepout_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
            **lifecycle_bond,
        }],
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            **lifecycle_bond,
        }],
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': navigation_autostart,
            'node_names': navigation_nodes,
            **lifecycle_bond,
        }],
    )
    coordinated_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coordinated',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
                'map_server',
                'amcl',
                *navigation_nodes,
            ],
            **lifecycle_bond,
        }],
    )

    # The container process surviving is not evidence that the nodes inside it
    # are alive, so the graph-level guard keeps that failure observable while
    # Nav2 remains composed.
    liveness_guard = Node(
        package='jdamr_cube_navigation',
        executable='nav2_liveness_guard',
        name='nav2_liveness_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--required', ','.join(required_nodes)],
    )

    navigation_processes = [container]
    if LaunchConfiguration('use_composition', default='true').perform(context).lower() == 'false':
        executables = {
            'keepout_filter_mask_server': 'map_server',
            'keepout_costmap_filter_info_server': 'costmap_filter_info_server',
            'map_server': 'map_server', 'amcl': 'amcl',
            'controller_server': 'controller_server', 'planner_server': 'planner_server',
            'velocity_smoother': 'velocity_smoother', 'bt_navigator': 'bt_navigator',
            'behavior_server': 'behavior_server',
        }
        navigation_processes = []
        for component in keepout_components + nav2_components:
            name = perform_substitutions(context, component.node_name)
            navigation_processes.append(Node(
                package=component.package, executable=executables[name],
                name=name, output='screen',
                # Child costmaps need the complete YAML, as in the container.
                parameters=[configured_params, *component.parameters],
                remappings=component.remappings))
    lifecycle_processes = ([coordinated_lifecycle] if coordinated_startup else [
        keepout_lifecycle,
        localization_lifecycle,
        navigation_lifecycle,
    ])
    required_processes = [
        *navigation_processes, collision_monitor, *lifecycle_processes,
    ]
    required_exit_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=_shutdown_unless_already_stopping(
                'required Nav2 process exited; stopping navigation')))
        for process in required_processes
    ]

    # Discovery diagnostics do not own motor or lifecycle shutdown.
    return [*required_exit_handlers, *required_processes, liveness_guard]


def generate_launch_description():
    """Keep the proven corridor profile as the default onboard launch."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    discovery_range = LaunchConfiguration('discovery_range')
    return LaunchDescription([
        DeclareLaunchArgument('precision_parking', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_box_search', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_composition', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'coordinated_startup', default_value='false',
            choices=['true', 'false'],
            description=(
                'Activate keepout, localization and navigation in one '
                'ordered lifecycle transaction')),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument(
            'discovery_range', default_value='LOCALHOST',
            choices=['LOCALHOST', 'SUBNET'],
            description=(
                'Use the same discovery scope as onboard sensors and the '
                'route executor; LOCALHOST excludes remote ROS clients')),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', discovery_range),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908.yaml')),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.expanduser(
                '~/maps/autonomous_20260826T161908_keepout_multi.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('asset_registry', default_value='',
                              description='Hash-bound named map and keepout registry'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('navigation_autostart',
                              default_value=LaunchConfiguration('autostart'),
                              choices=['true', 'false']),
        DeclareLaunchArgument('revisit_initial_x', default_value='0.0'),
        DeclareLaunchArgument('revisit_initial_y', default_value='-0.1'),
        DeclareLaunchArgument('revisit_initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'navigation_profile', default_value='corridor',
            choices=[
                'corridor', 'obstacle_candidate',
                'obstacle_base_candidate', 'new_base_candidate',
                'new_base_revisit_candidate'],
            description='Candidate enables online replanning and Wait recovery'),
        OpaqueFunction(function=_reject_legacy_profile_on_new_base),
        OpaqueFunction(function=_validate_revisit_isolation),
        OpaqueFunction(function=_validate_keepout),
        OpaqueFunction(function=_launch_navigation),
    ])
