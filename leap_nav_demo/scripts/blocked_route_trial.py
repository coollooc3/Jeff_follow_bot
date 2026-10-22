#!/usr/bin/env python3
"""Fail on a closed exit, open it, then retry the same goal without restarting Nav2."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.parameter import Parameter
from closed_route import ClosedRoute
from nav_recording import Recorder, pose
from task_checks import task_failures
from task_config import encode_task, read_task, task_hash


def settle(nav, log):
    deadline = time.monotonic() + 2.
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=.1)
    odom, command = log.last_odom, log.last_cmd
    stopped = (odom is not None and time.monotonic() - log.odom_received < .5 and
               abs(odom.twist.twist.linear.x) < .005 and abs(odom.twist.twist.angular.z) < .02)
    zero = command is not None and abs(command.linear.x) < 1e-4 and abs(command.angular.z) < 1e-4
    return bool(stopped and (zero or command is None)), bool(zero)


def action(nav, log, goal, tree, timeout, phase):
    start = time.monotonic()
    path, commands, aborted = log.path_length, log.nonzero_commands, len(log.planner_aborted_goals)
    result = {'goal': list(goal), 'phase': phase}
    log.event('route_goal_requested', phase=phase, goal=goal)
    if not nav.goToPose(pose(nav, *goal), behavior_tree=str(tree)):
        raise RuntimeError(phase + ' goal rejected')
    result['action_id'] = bytes(nav.goal_handle.goal_id.uuid).hex()
    recoveries = 0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback is not None:
            recoveries = max(recoveries, feedback.number_of_recoveries)
        if time.monotonic() - start > timeout:
            nav.cancelTask()
            result['result'] = 'task_timeout'
            break
    else:
        result['result'] = nav.getResult().name
    result.update(task_wall_seconds=time.monotonic() - start, recoveries=recoveries)
    result['stopped_and_fresh'], result['last_command_zero'] = settle(nav, log)
    result.update(odom_path_length_m=log.path_length - path, nonzero_commands=log.nonzero_commands - commands,
                  planner_aborted_goals=len(log.planner_aborted_goals) - aborted)
    log.event('route_action_finished', **result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task-config', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires simulation domain178 and localhost only')
    task = read_task(args.task_config, 'blocked_route')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'task_config_used.json').write_bytes(encode_task(task))
    trees = [Path(get_package_share_directory('leap_nav_demo')) / 'behavior_trees/fail_fast.xml',
             Path(get_package_share_directory('nav2_bt_navigator')) /
             'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml']
    tree_hashes = []
    for tree, name in zip(trees, ('behavior_tree_used.xml', 'retry_behavior_tree_used.xml')):
        data = tree.read_bytes()
        (args.output / name).write_bytes(data)
        tree_hashes.append(hashlib.sha256(data).hexdigest())
    rclpy.init()
    nav = BasicNavigator(node_name='leap_blocked_route_task')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    log = Recorder(nav, args.output, task)
    fixture = ClosedRoute(nav, log, args.output, task['start'], task['goal'])
    evidence = {'same_nav2_session': True, 'gate_deleted': False}
    report = {'ok': False, 'scenario': 'blocked_route', 'result': 'not_started', 'expected_result': 'SUCCEEDED',
        'route': task['route'], 'initial_map_pose': task['start'], 'goal_map_pose': task['goal'],
        'required_path_length_m': task['required_path_length_m'], 'task_config_sha256': task_hash(task),
        'record_expert': False, 'behavior_tree_sha256': tree_hashes[0], 'retry_behavior_tree_sha256': tree_hashes[1],
        'route_recovery': evidence, 'limits': 'Controlled closed-exit fixture; explicit gate removal and costmap clear. '
            'Topology is point-grid connectivity, not whole-scene contact certification or autonomous environment recovery.'}
    started = time.monotonic()
    try:
        while nav.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(nav, timeout_sec=.1)
        nav.setInitialPose(pose(nav, *task['start']))
        nav.waitUntilNav2Active()
        evidence['original_connectivity'] = fixture.wait_original()
        evidence['closed_connectivity'] = fixture.prepare()
        closed = evidence['closed_attempt'] = action(nav, log, task['goal'], trees[0], 10., 'exit_closed')
        if (closed['result'] != 'FAILED' or closed['planner_aborted_goals'] == 0 or
                closed['nonzero_commands'] != 0 or closed['odom_path_length_m'] >= .02 or
                not closed['stopped_and_fresh']):
            raise RuntimeError('closed-phase rejection/stop contract failed; do not start retry')
        evidence['gate_evidence_before_removal'] = fixture.gate.result()
        if not evidence['gate_evidence_before_removal']['ok']:
            raise RuntimeError('closed gate evidence invalid; do not start retry')
        evidence['opened_connectivity'] = fixture.open()
        evidence.update(gate_deleted=True, explicit_costmap_clear=True)
        retry = evidence['retry_attempt'] = action(nav, log, task['goal'], trees[1], 75., 'exit_open_retry')
        report['result'] = retry['result']
        final = log.map_pose()
        report.update(task_wall_seconds=time.monotonic() - started, final_map_pose=final,
            position_error_m=math.hypot(final[0] - task['goal'][0], final[1] - task['goal'][1]) if final else None,
            yaw_error_rad=abs(math.atan2(math.sin(final[2] - task['goal'][2]), math.cos(final[2] - task['goal'][2]))) if final else None,
            odom_path_length_m=log.path_length, plans_received=log.plan_count,
            nonzero_commands_received=log.nonzero_commands, planner_aborted_goal_count=len(log.planner_aborted_goals),
            recoveries_reported_by_nav2=retry['recoveries'], command_observed=log.last_cmd is not None,
            last_command_zero=retry['last_command_zero'], odom_stopped_and_fresh=retry['stopped_and_fresh'])
        report['check_failures'] = task_failures(report)
        report['ok'] = not report['check_failures']
    except (Exception, KeyboardInterrupt) as exc:
        report.update(ok=False, result=type(exc).__name__, error=str(exc))
        if nav.goal_handle is not None and rclpy.ok() and not nav.isTaskComplete():
            nav.cancelTask()
            settle(nav, log)
    finally:
        if rclpy.ok():
            try:
                nav.lifecycleShutdown()
                report['nav2_lifecycle_shutdown_complete'] = True
            except Exception as exc:
                report.update(ok=False, shutdown_error=str(exc))
        log.close(report)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
