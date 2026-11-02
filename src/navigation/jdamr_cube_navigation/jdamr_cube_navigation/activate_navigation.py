"""Activate prepared Nav2 servers after stationary localization is confirmed."""

import argparse
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from jdamr_cube_navigation.corridor_route import AMCL_QOS, spin_node
from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from jdamr_cube_navigation.service_destinations import load_registry
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes, SetInitialPose
import rclpy
from std_srvs.srv import Empty
import yaml

LOCALIZATION_NODES = (
    'map_server', 'amcl', 'keepout_filter_mask_server',
    'keepout_costmap_filter_info_server')
NAVIGATION_NODES = (
    'controller_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'velocity_smoother', 'collision_monitor')
INITIAL_POSE_FIELDS = (
    'frame_id', 'x_m', 'y_m', 'yaw_rad', 'covariance_x_m2',
    'covariance_y_m2', 'covariance_yaw_rad2')


def load_initial_pose(path):
    """Load one explicit measured pose without filling missing values."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('initial pose document must be a mapping')
    missing = [name for name in INITIAL_POSE_FIELDS if name not in document]
    if missing:
        raise ValueError('initial pose fields missing: ' + ', '.join(missing))
    if document['frame_id'] != 'map':
        raise ValueError('initial pose frame_id must be map')
    for name in INITIAL_POSE_FIELDS[1:]:
        value = document[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError(name + ' must be finite')
    for name in ('covariance_x_m2', 'covariance_y_m2',
                 'covariance_yaw_rad2'):
        if document[name] <= 0.0:
            raise ValueError(name + ' must be positive')
    return {name: document[name] for name in INITIAL_POSE_FIELDS}


def require_active(node, names, state_id=State.PRIMARY_STATE_ACTIVE, wait=None,
                   transition_timeout_s=0.0):
    """Check lifecycle responses, optionally polling a bounded transition."""
    for name in names:
        client = node.create_client(GetState, '/' + name + '/get_state')
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(name + ' lifecycle service unavailable')
            waiter = wait or node._wait
            transition_deadline_s = (
                time.monotonic() + transition_timeout_s
                if transition_timeout_s > 0.0 else None)
            while True:
                response_budget_s = 5.0
                if transition_deadline_s is not None:
                    response_budget_s = min(
                        response_budget_s,
                        transition_deadline_s - time.monotonic())
                    if response_budget_s <= 0.0:
                        raise RuntimeError(
                            name + ' lifecycle transition timed out')
                deadline_s = time.monotonic() + response_budget_s
                response = None
                for attempt in range(2):
                    remaining_s = deadline_s - time.monotonic()
                    if remaining_s <= 0.0:
                        raise RuntimeError(
                            name + ' lifecycle response timed out')
                    timeout_s = (min(2.0, remaining_s)
                                 if attempt == 0 else remaining_s)
                    future = client.call_async(GetState.Request())
                    try:
                        response = waiter(future, timeout_s)
                    except RuntimeError as error:
                        incomplete = not future.done()
                        if incomplete:
                            client.remove_pending_request(future)
                        if (attempt == 0 and incomplete
                                and not node.stop_requested
                                and time.monotonic() < deadline_s):
                            node.get_logger().warning(
                                f'lifecycle read timeout: node={name} '
                                f'attempt=1/2; '
                                f'retrying GetState within the original '
                                f'{response_budget_s:g}s response budget; '
                                f'error={error}')
                            continue
                        if (incomplete and transition_deadline_s is not None
                                and not node.stop_requested
                                and time.monotonic() < transition_deadline_s):
                            node.get_logger().warning(
                                f'lifecycle read still pending: node={name}; '
                                f'polling again within the bounded transition '
                                f'budget; error={error}')
                            response = None
                            break
                        raise
                    break
                if (response is not None
                        and response.current_state.id == state_id):
                    break
                remaining_transition_s = (
                    transition_deadline_s - time.monotonic()
                    if transition_deadline_s is not None else 0.0)
                if (transition_deadline_s is None or node.stop_requested):
                    raise RuntimeError(name + ' lifecycle state unconfirmed')
                if remaining_transition_s <= 0.0:
                    raise RuntimeError(
                        name + ' lifecycle transition timed out')
                spin_node(node, timeout_sec=min(0.1, remaining_transition_s))
        finally:
            node.destroy_client(client)


def initialize_localization(
        node, initial_pose, timeout_s=8.0, covariance_limits=None):
    """Set an explicit pose and require a usable post-ACK AMCL sample."""
    if covariance_limits is not None:
        try:
            covariance_limits = tuple(covariance_limits)
        except TypeError as error:
            raise ValueError(
                'covariance_limits must contain x, y, and yaw limits') from error
        if len(covariance_limits) != 3 or any(
                isinstance(limit, bool)
                or not isinstance(limit, (int, float))
                or not math.isfinite(limit) or limit <= 0.0
                for limit in covariance_limits):
            raise ValueError(
                'covariance_limits must contain three finite positive values')
    require_active(node, LOCALIZATION_NODES, transition_timeout_s=15.0)
    node.verify_live_maps(require_command_path=False)
    latest_sample = [None]

    def observe(message):
        stamp = message.header.stamp
        covariance = (None if covariance_limits is None else (
            float(message.pose.covariance[0]),
            float(message.pose.covariance[7]),
            float(message.pose.covariance[35])))
        latest_sample[0] = (
            stamp.sec + stamp.nanosec * 1e-9,
            covariance,
        )

    subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', observe, AMCL_QOS)
    initial_client = node.create_client(SetInitialPose, '/set_initial_pose')
    update_client = node.create_client(Empty, '/request_nomotion_update')
    try:
        if node.stop_requested:
            raise RuntimeError('initialization interrupted')
        if not initial_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('AMCL set_initial_pose service unavailable')
        if not update_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('AMCL nomotion update service unavailable')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = initial_pose['frame_id']
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.pose.position.x = initial_pose['x_m']
        message.pose.pose.position.y = initial_pose['y_m']
        message.pose.pose.orientation.z = math.sin(
            initial_pose['yaw_rad'] / 2.0)
        message.pose.pose.orientation.w = math.cos(
            initial_pose['yaw_rad'] / 2.0)
        message.pose.covariance[0] = initial_pose['covariance_x_m2']
        message.pose.covariance[7] = initial_pose['covariance_y_m2']
        message.pose.covariance[35] = initial_pose['covariance_yaw_rad2']
        request = SetInitialPose.Request()
        request.pose = message
        response = node._wait(initial_client.call_async(request), timeout_s)
        if response is None:
            raise RuntimeError('AMCL initialization response unconfirmed')
        ack_ros_s = node.get_clock().now().nanoseconds * 1e-9
        deadline_s = time.monotonic() + timeout_s
        next_update_s = 0.0
        update_future = None
        fresh_pose_seen = False
        while time.monotonic() < deadline_s:
            if node.stop_requested:
                raise RuntimeError('initialization interrupted')
            if latest_sample[0] is not None:
                stamp_s, covariance = latest_sample[0]
                if stamp_s > ack_ros_s:
                    fresh_pose_seen = True
                    if covariance_limits is None or all(
                            math.isfinite(value) and value >= 0.0
                            and value <= limit
                            for value, limit in zip(
                                covariance, covariance_limits)):
                        return
            now_s = time.monotonic()
            if update_future is not None and update_future.done():
                if (update_future.exception() is not None
                        or update_future.result() is None):
                    raise RuntimeError('AMCL nomotion update unconfirmed')
                update_future = None
            if update_future is None and now_s >= next_update_s:
                update_future = update_client.call_async(Empty.Request())
                next_update_s = now_s + .2
            # rclpy.spin_once() would move the node off its own events executor
            # for good (2026-10-06: every later reply was dropped).
            spin_node(node, timeout_sec=.05)
        if fresh_pose_seen and covariance_limits is not None:
            raise RuntimeError(
                'AMCL covariance did not converge after initialization ACK')
        raise RuntimeError('fresh AMCL pose missing after initialization ACK')
    finally:
        node.destroy_client(initial_client)
        node.destroy_client(update_client)
        node.destroy_subscription(subscription)


def rollback_navigation(node, client):
    """Resolve lifecycle reset even when the ordinary request was interrupted."""
    def wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            spin_node(node, timeout_sec=0.05)
        if not future.done() or future.exception() is not None or future.result() is None:
            raise RuntimeError('navigation rollback response unconfirmed')
        return future.result()

    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.RESET
    response = wait(client.call_async(request), 45.0)
    if not response.success:
        raise RuntimeError('navigation rollback rejected')
    require_active(node, NAVIGATION_NODES,
                   state_id=State.PRIMARY_STATE_UNCONFIGURED, wait=wait)
    node.emit('navigation_activation_rolled_back')


def navigation_active(node):
    """Return True when every navigation server already answers active."""
    try:
        require_active(node, NAVIGATION_NODES)
    except RuntimeError:
        return False
    return True


def require_parking_controller(node):
    """Prove the controller answers before reporting preparation as complete."""
    if not node._parking_parameters_ready():
        raise RuntimeError(node.last_parking_parameters_failure)


def activate_prepared(node):
    """Send lifecycle startup only; never set a pose or send a motion goal."""
    require_active(node, LOCALIZATION_NODES, transition_timeout_s=15.0)
    node.verify_live_maps(require_command_path=False)
    if not node.wait_until_ready(timeout=10.0):
        raise RuntimeError(node.readiness_failure_reason())
    client = node.create_client(
        ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
    startup_requested = False
    startup_confirmed = False
    try:
        if node.stop_requested or not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('navigation lifecycle startup unavailable')
        pose, evidence = node.capture_stationary_pose(timeout_s=10.0)
        # Recheck inputs after service discovery; readiness is not a cached grant.
        failure = node._guard_failure(require_fresh_amcl=True)
        if failure or node.stop_requested:
            raise RuntimeError(failure or 'activation interrupted')
        if navigation_active(node):
            # Re-init after a cycle (2026-10-01 16:18): STARTUP on active servers
            # failed at configure and the rollback reset the whole stack.
            node.verify_live_maps()
            require_parking_controller(node)
            node.emit('navigation_activated_without_motion', stationary_pose=pose,
                      stationary_evidence=evidence, already_active=True)
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        startup_requested = True
        response = node._wait(client.call_async(request), 45.0)
        if response is None or not response.success:
            raise RuntimeError('navigation lifecycle startup failed')
        require_active(node, NAVIGATION_NODES, transition_timeout_s=15.0)
        startup_confirmed = True
        node.verify_live_maps()
        require_parking_controller(node)
        node.emit('navigation_activated_without_motion',
                  stationary_pose=pose, stationary_evidence=evidence)
    except Exception as verification_error:
        if startup_requested and not startup_confirmed:
            try:
                rollback_navigation(node, client)
            except Exception as error:
                node.emit('navigation_activation_rollback_unconfirmed',
                          reason=str(error), navigation_ready=False)
                raise RuntimeError('navigation rollback unconfirmed; do not depart') from error
        elif startup_confirmed:
            # A failed read/identity check still prevents departure, but is not
            # evidence that healthy lifecycle servers need a destructive reset.
            node.emit('navigation_activation_verification_failed',
                      reason=str(verification_error), navigation_ready=False,
                      servers_kept_active=True)
        raise
    finally:
        node.destroy_client(client)


def initialize_and_activate(node, initial_pose=None):
    """Optionally initialize AMCL before the unchanged activation gates."""
    if initial_pose is not None:
        covariance_limits = (
            node.max_amcl_covariance[0],
            node.max_amcl_covariance[1],
            node.service_contract['max_yaw_covariance_rad2'],
        )
        initialize_localization(
            node, initial_pose, covariance_limits=covariance_limits)
    activate_prepared(node)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', required=True, type=Path)
    parser.add_argument('--parking-contract', required=True, type=Path)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--initial-pose', type=Path)
    args = parser.parse_args(argv)
    registry = load_registry(args.registry)
    contract = load_parking_contract(args.parking_contract)
    initial_pose = (load_initial_pose(args.initial_pose)
                    if args.initial_pose is not None else None)
    with args.log.open('x', encoding='utf-8') as stream:
        rclpy.init()
        node = ServiceRoute(registry, contract, stream)
        try:
            initialize_and_activate(node, initial_pose)
            return 0
        except RuntimeError as error:
            node.emit('activation_failed', reason=str(error))
            return 1
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
