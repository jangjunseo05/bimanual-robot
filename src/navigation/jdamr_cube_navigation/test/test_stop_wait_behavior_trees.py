"""Contracts for stop-and-wait Nav2 behavior trees."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).parents[1]
TREE_CASES = {
    'navigate_through_poses_transit_rpp.xml': {
        'planner': 'ComputePathThroughPoses',
        'planner_id': 'GridBased',
        'controller_id': 'FollowPath',
        'goal_checker_id': 'general_goal_checker',
        'prefix_children': [],
    },
    'navigate_to_pose_staging_rpp.xml': {
        'planner': 'ComputePathToPose',
        'planner_id': 'GridBased',
        'controller_id': 'FollowPath',
        'goal_checker_id': 'staging_position_checker',
        'prefix_children': [],
    },
    'navigate_to_pose_corridor_fail_fast.xml': {
        'planner': 'ComputePathToPose',
        'planner_id': '{selected_planner}',
        'controller_id': '{selected_controller}',
        'goal_checker_id': '{selected_goal_checker}',
        'prefix_children': [
            'ControllerSelector', 'PlannerSelector', 'GoalCheckerSelector'],
    },
}
FORBIDDEN_RECOVERIES = {'ClearEntireCostmap', 'Spin', 'BackUp'}


def _main_sequence(name):
    root = ET.parse(PACKAGE_ROOT / 'behavior_trees' / name).getroot()
    behavior_tree = root.find("./BehaviorTree[@ID='MainTree']")
    assert behavior_tree is not None
    children = list(behavior_tree)
    assert len(children) == 1
    assert children[0].tag == 'Sequence'
    return root, children[0]


@pytest.mark.parametrize(('name', 'contract'), TREE_CASES.items())
def test_tree_plans_then_retries_the_same_path(name, contract):
    """Keep planning ahead of bounded same-path controller retries."""
    root, sequence = _main_sequence(name)
    children = list(sequence)
    prefix_count = len(contract['prefix_children'])

    assert [child.tag for child in children[:prefix_count]] == contract[
        'prefix_children']
    planner_retry, follower_retry = children[prefix_count:]
    assert planner_retry.tag == follower_retry.tag == 'RecoveryNode'
    assert planner_retry.attrib['number_of_retries'] == '60'
    assert follower_retry.attrib['number_of_retries'] == '60'

    planner, planner_recovery = list(planner_retry)
    assert planner.tag == contract['planner']
    assert planner.attrib['path'] == '{path}'
    assert planner.attrib['planner_id'] == contract['planner_id']
    assert [node.tag for node in planner_recovery] == [
        'WouldAPlannerRecoveryHelp', 'Wait']
    assert planner_recovery[-1].attrib['wait_duration'] == '1.0'

    follower, follower_recovery = list(follower_retry)
    assert follower.tag == 'FollowPath'
    assert follower.attrib['path'] == '{path}'
    assert follower.attrib['controller_id'] == contract['controller_id']
    assert follower.attrib['goal_checker_id'] == contract['goal_checker_id']
    assert [node.tag for node in follower_recovery] == [
        'WouldAControllerRecoveryHelp', 'Wait']
    assert follower_recovery[-1].attrib['wait_duration'] == '1.0'

    tags = {node.tag for node in root.iter()}
    assert tags.isdisjoint(FORBIDDEN_RECOVERIES)
    assert len(list(root.iter(contract['planner']))) == 1
    assert len(list(root.iter('FollowPath'))) == 1


def _run_without_motion(sequence, planner_failures, follower_failures):
    """Execute the relevant BT control flow with deterministic action stubs."""
    recoveries = list(sequence.iter('RecoveryNode'))
    planner_retry, follower_retry = recoveries
    planner_limit = int(planner_retry.attrib['number_of_retries']) + 1
    follower_limit = int(follower_retry.attrib['number_of_retries']) + 1
    state = {'planner_calls': 0, 'followed_paths': [], 'waits': 0}
    path = None

    for attempt in range(planner_limit):
        state['planner_calls'] += 1
        if state['planner_calls'] > planner_failures:
            path = object()
            break
        if attempt + 1 < planner_limit:
            state['waits'] += 1
    if path is None:
        return False, state

    for attempt in range(follower_limit):
        state['followed_paths'].append(path)
        if len(state['followed_paths']) > follower_failures:
            return True, state
        if attempt + 1 < follower_limit:
            state['waits'] += 1
    return False, state


@pytest.mark.parametrize('name', TREE_CASES)
def test_no_motion_execution_reuses_path_after_controller_failures(name):
    """A deterministic no-motion run keeps the first successful path."""
    _, sequence = _main_sequence(name)

    succeeded, state = _run_without_motion(
        sequence, planner_failures=2, follower_failures=3)

    assert succeeded
    assert state['planner_calls'] == 3
    assert len(state['followed_paths']) == 4
    assert len({id(path) for path in state['followed_paths']}) == 1
    assert state['waits'] == 5


@pytest.mark.parametrize('name', TREE_CASES)
def test_exhausted_controller_retries_do_not_reenter_planner(name):
    """Controller exhaustion fails the tree without planning another path."""
    _, sequence = _main_sequence(name)

    succeeded, state = _run_without_motion(
        sequence, planner_failures=0, follower_failures=61)

    assert not succeeded
    assert state['planner_calls'] == 1
    assert len(state['followed_paths']) == 61
    assert state['waits'] == 60
