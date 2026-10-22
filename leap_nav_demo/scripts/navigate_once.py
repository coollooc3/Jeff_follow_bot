#!/usr/bin/env python3
"""One simulation navigation task using Nav2's BasicNavigator.

Run through run_demo: that parent bounds startup and blocking upstream API calls.
"""

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from obstacle_scenario import (ObstacleScenario, path_intersects_box, estimated_box_clearance,
                               BLOCKED_GOAL_CENTER, BLOCKED_GOAL_SIZE)
from task_checks import SCENARIOS, StopProbe, task_failures
from task_config import make_task, read_task, encode_task, task_hash


from nav_recording import Recorder, pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--scenario', choices=SCENARIOS, default='baseline')
    parser.add_argument('--task-config', type=Path)
    parser.add_argument('--record-expert', action='store_true')
    args = parser.parse_args()
    if args.scenario == 'blocked_route':
        parser.error('use run_demo --scenario blocked_route for the two-action recovery task')
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    if not math.isfinite(args.timeout) or not 10 <= args.timeout <= 180:
        parser.error('--timeout must be 10..180 wall seconds')
    try:
        configured_task = read_task(args.task_config, args.scenario) if args.task_config else make_task(args.scenario)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'task_config_used.json').write_bytes(encode_task(configured_task))
    rclpy.init()
    nav = BasicNavigator(node_name='leap_navigation_task')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    log = Recorder(nav, args.output, configured_task, args.record_expert)
    report = {'ok': False, 'scenario': args.scenario, 'route': configured_task['route'],
              'initial_map_pose': configured_task['start'], 'goal_map_pose': configured_task['goal'],
              'required_path_length_m': configured_task['required_path_length_m'],
              'task_config_sha256': task_hash(configured_task), 'record_expert': args.record_expert,
              'result': 'not_started',
              'limits': 'AMCL/odom estimates; injected-box contacts only; no whole-scene or LEAP validation.'}
    report['expected_result'] = {'cancel': TaskResult.CANCELED.name,
                                 'blocked_goal': TaskResult.FAILED.name}.get(
                                     args.scenario, TaskResult.SUCCEEDED.name)
    pending = False
    obstacle = None
    recoveries = 0
    injection = {'triggered': False}
    cancellation = {'triggered': False}
    try:
        log.event('waiting_for_clock')
        while nav.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(nav, timeout_sec=0.1)
        nav.setInitialPose(pose(nav, *report['initial_map_pose']))
        log.event('waiting_for_nav2_and_localization')
        nav.waitUntilNav2Active()
        log.event('nav2_active')
        report['localized_initial_map_pose'] = log.map_pose()
        if args.scenario == 'obstacle':
            obstacle = ObstacleScenario(nav, log)
            obstacle.prepare()
        elif args.scenario == 'blocked_goal':
            obstacle = ObstacleScenario(nav, log, center=BLOCKED_GOAL_CENTER,
                size=BLOCKED_GOAL_SIZE, model_file='blocked_goal.sdf',
                required_occupied_point=report['goal_map_pose'][:2])
            obstacle.prepare()
            log.event('occupied_goal_verified', cost=obstacle.required_point_cost)
        elif args.scenario == 'sudden_obstacle':
            obstacle = ObstacleScenario(nav, log)
            obstacle.wait_for_clear_region()
        start = time.monotonic()
        behavior_tree = (str(Path(get_package_share_directory('leap_nav_demo')) /
                             'behavior_trees/fail_fast.xml') if args.scenario == 'blocked_goal' else '')
        if behavior_tree:
            tree = Path(behavior_tree).read_bytes()
            (args.output / 'behavior_tree_used.xml').write_bytes(tree)
            report['behavior_tree_sha256'] = hashlib.sha256(tree).hexdigest()
        if not nav.goToPose(pose(nav, *report['goal_map_pose']), behavior_tree=behavior_tree):
            report['result'] = 'goal_rejected'
        else:
            pending = True
            if log.expert is not None:
                log.expert.active = True
            log.event('goal_accepted')
            while not nav.isTaskComplete():
                now = time.monotonic()
                feedback = nav.getFeedback()
                if feedback is not None and feedback.number_of_recoveries > recoveries:
                    recoveries = feedback.number_of_recoveries
                    log.event('navigation_recovery', count=recoveries)
                if obstacle is not None:
                    obstacle.observe(log.map_pose())
                    if obstacle.spawned:
                        obstacle.observe_plan(log.latest_plan, log.latest_plan_wall)
                        if obstacle.collisions:
                            nav.cancelTask()
                            report['result'] = 'injected_obstacle_contact'
                            break
                    elif args.scenario == 'sudden_obstacle' and log.path_length >= 0.15:
                        now = time.monotonic()
                        cmd, odom, current = log.last_cmd, log.last_odom, log.map_pose()
                        moving = log.moving_forward(now) and current is not None
                        if moving:
                            if not path_intersects_box(log.first_plan):
                                raise RuntimeError('first plan does not intersect intended obstacle')
                            clearance = estimated_box_clearance(current)
                            if clearance < 0.25:
                                raise RuntimeError('too close to inject obstacle at fixed location')
                            injection = {'triggered': True, 'goal_elapsed_wall_s': now - start,
                                         'odom_path_at_trigger_m': log.path_length,
                                         'map_pose_at_trigger': current,
                                         'command_linear_at_trigger_mps': cmd.linear.x,
                                         'odom_linear_at_trigger_mps': odom.twist.twist.linear.x,
                                         'estimated_clearance_at_trigger_m': clearance,
                                         'initial_plan_intersects_box': True}
                            log.event('moving_obstacle_trigger', **injection)
                            obstacle.spawn()
                    if (obstacle.spawned and not obstacle.evidence_ready() and
                            time.monotonic() - obstacle.spawn_requested_wall > 8.0):
                        raise RuntimeError('post-injection obstacle evidence missing')
                if (args.scenario == 'cancel' and not cancellation['triggered'] and
                        log.path_length >= 0.15 and log.moving_forward(now)):
                    p = log.last_odom.pose.pose.position
                    log.stop_probe = StopProbe(time.monotonic(), (p.x, p.y))
                    cancellation = {'triggered': True, 'odom_path_at_request_m': log.path_length,
                        'odom_receive_age_at_request_s': log.stop_probe.requested_wall - log.odom_received,
                        'command_linear_at_request_mps': log.last_cmd.linear.x,
                        'odom_linear_at_request_mps': log.last_odom.twist.twist.linear.x}
                    log.event('cancel_requested', **cancellation)
                    nav.cancelTask()
                    cancellation['request_return_after_s'] = time.monotonic() - log.stop_probe.requested_wall
                if (cancellation['triggered'] and
                        time.monotonic() - log.stop_probe.requested_wall > 2.0):
                    report['result'] = 'cancel_confirmation_timeout'
                    break
                timeout = min(args.timeout, 10.0) if args.scenario == 'blocked_goal' else args.timeout
                if time.monotonic() - start > timeout:
                    log.event('task_timeout_cancel_requested')
                    nav.cancelTask()
                    report['result'] = 'task_timeout'
                    break
            else:
                report['result'] = nav.getResult().name
            pending = False
            if log.expert is not None:
                log.expert.active = False
            report['task_wall_seconds'] = time.monotonic() - start
            if cancellation['triggered']:
                cancellation['result_after_request_s'] = time.monotonic() - log.stop_probe.requested_wall
            report['recoveries_reported_by_nav2'] = recoveries
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(nav, timeout_sec=0.1)
            final = log.map_pose()
            report['final_map_pose'] = final
            goal = report['goal_map_pose']
            report['position_error_m'] = (math.hypot(final[0] - goal[0], final[1] - goal[1])
                                          if final else None)
            report['yaw_error_rad'] = (abs(math.atan2(math.sin(final[2] - goal[2]),
                                                     math.cos(final[2] - goal[2])))
                                       if final else None)
            report['odom_path_length_m'] = log.path_length
            report['plans_received'] = log.plan_count
            report['nonzero_commands_received'] = log.nonzero_commands
            report['planner_aborted_goal_count'] = len(log.planner_aborted_goals)
            cmd = log.last_cmd
            odom = log.last_odom
            report['command_observed'] = cmd is not None
            report['last_command_zero'] = (cmd is not None and
                abs(cmd.linear.x) < 1e-4 and abs(cmd.angular.z) < 1e-4)
            report['odom_stopped_and_fresh'] = (odom is not None and
                time.monotonic() - log.odom_received < 0.5 and
                abs(odom.twist.twist.linear.x) < 0.005 and abs(odom.twist.twist.angular.z) < 0.02)
            if obstacle is not None:
                report['obstacle'] = obstacle.result()
            if args.scenario == 'sudden_obstacle':
                report['injection'] = injection
            if args.scenario == 'cancel':
                cancellation['stop'] = log.stop_probe.result() if log.stop_probe is not None else {}
                report['cancellation'] = cancellation
            report['check_failures'] = task_failures(report)
            report['ok'] = not report['check_failures']
        log.event('task_finished', result=report['result'], ok=report['ok'])
        # Deactivate while Gazebo's clock and ROS services are still available.
        # Sending SIGINT to all nodes at once can strand lifecycle shutdown.
        log.event('nav2_shutdown_requested')
        nav.lifecycleShutdown()
        report['nav2_lifecycle_shutdown_complete'] = True
        log.event('nav2_shutdown_complete')
    except (Exception, KeyboardInterrupt) as exc:
        report['ok'] = False
        report['result'] = type(exc).__name__
        report['error'] = str(exc)
        if pending and rclpy.ok():
            nav.cancelTask()
    finally:
        report['first_odom_pose'] = log.first_odom_pose
        if log.expert is not None:
            log.expert.active = False
        if args.scenario == 'sudden_obstacle':
            report['injection'] = injection
        if args.scenario == 'cancel':
            if log.stop_probe is not None:
                cancellation['stop'] = log.stop_probe.result()
            report['cancellation'] = cancellation
        if obstacle is not None and 'obstacle' not in report:
            report['obstacle'] = obstacle.result()
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        log.close(report)
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
