#!/usr/bin/env python3
"""Simulation-only learned local controller: Nav2 planning, no expert takeover."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from bc_policy import Policy, vectorize, guard_reason
from nav_recording import Recorder, pose
from observation_buffer import ObservationBuffer
from task_config import read_task, task_hash


class Planner:
    """One outstanding asynchronous planner request; never block the control loop."""
    def __init__(self, node, observations, goal):
        self.node, self.observations, self.goal = node, observations, goal
        self.future = self.handle = None
        self.phase, self.last_request, self.count = None, -math.inf, 0

    def tick(self, now):
        if self.future is not None:
            if now - self.last_request > 2.:
                raise RuntimeError('planner response timeout')
            if not self.future.done():
                return
            response = self.future.result()
            if self.phase == 'accept':
                if not response.accepted:
                    raise RuntimeError('planner rejected goal')
                self.handle = response
                self.future, self.phase = response.get_result_async(), 'result'
                return
            if response.status != GoalStatus.STATUS_SUCCEEDED or len(response.result.path.poses) < 2:
                raise RuntimeError('planner failed to return a path')
            self.observations.on_plan(response.result.path)
            self.count += 1
            self.future = self.handle = None
        if now - self.last_request >= 1.:
            if not self.node.compute_path_to_pose_client.server_is_ready():
                raise RuntimeError('planner unavailable')
            message = ComputePathToPose.Goal()
            message.goal = pose(self.node, *self.goal)
            message.planner_id = 'GridBased'
            message.use_start = False
            self.future = self.node.compute_path_to_pose_client.send_goal_async(message)
            self.last_request, self.phase = now, 'accept'

    def cancel(self):
        if self.handle is not None and rclpy.ok():
            self.handle.cancel_goal_async()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task-config', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=40.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires isolated simulation domain 178 and localhost only')
    if not math.isfinite(args.timeout) or not 10 <= args.timeout <= 90:
        parser.error('timeout must be 10..90 wall seconds')
    configured = read_task(args.task_config, 'baseline')
    policy = Policy.load(args.policy)
    args.output.mkdir(parents=True, exist_ok=True)
    policy_bytes = args.policy.read_bytes()
    (args.output / 'policy_used.json').write_bytes(policy_bytes)
    rclpy.init()
    nav = BasicNavigator(node_name='leap_bc_trial')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    log = Recorder(nav, args.output, configured)
    observations = ObservationBuffer(nav, log.buffer, configured['goal'])
    subscriptions = [
        nav.create_subscription(LaserScan, '/scan', observations.on_scan, qos_profile_sensor_data),
        nav.create_subscription(Odometry, '/odom', observations.on_odom, qos_profile_sensor_data)]
    competing = {'active': False}

    def on_follow_status(message):
        if any(s.status == GoalStatus.STATUS_EXECUTING for s in message.status_list):
            competing['active'] = True

    subscriptions.append(nav.create_subscription(GoalStatusArray, '/follow_path/_action/status', on_follow_status, 10))
    publisher = nav.create_publisher(Twist, '/cmd_vel_nav', 10)
    planner = Planner(nav, observations, configured['goal'])
    report = {'ok': False, 'result': 'not_started', 'expected_result': 'GOAL_REACHED',
              'result_origin': 'bc_trial_goal_check', 'simulation_only': True,
              'scenario': 'baseline', 'controller': 'bc', 'task_config_sha256': task_hash(configured),
              'policy_sha256': hashlib.sha256(policy_bytes).hexdigest(), 'guard_stops': 0,
              'inference_steps': 0, 'clipped_steps': 0, 'expert_takeovers': 0,
              'limits': 'Nav2 global planner and velocity smoother retained; no FollowPath goal or recovery. '
                        'One-map experiment. Proximity guard is not a collision checker; whole-scene contacts unverified.'}
    commands = (args.output / 'policy_commands.jsonl').open('w')
    try:
        log.event('waiting_for_clock')
        while nav.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(nav, timeout_sec=.1)
        nav.setInitialPose(pose(nav, *configured['start']))
        nav.waitUntilNav2Active()
        log.event('bc_started_planner_only_no_follow_path_goal')
        started, next_tick, has_moved = time.monotonic(), time.monotonic(), False
        while time.monotonic() - started <= args.timeout:
            rclpy.spin_once(nav, timeout_sec=.01)
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = now + .05
            if competing['active']:
                raise RuntimeError('unexpected active FollowPath controller')
            planner.tick(now)
            action = {'wall_s': now, 'sim_s': nav.get_clock().now().nanoseconds * 1e-9, 'velocity': [0., 0.]}
            sample = observations.sample(action, True, False, require_follow_path=False)
            reason = guard_reason(sample)
            raw = bounded = [0., 0.]
            if reason is None:
                current = sample['features']['map_pose_at_odom_stamp']
                goal = configured['goal']
                position_error = math.hypot(current[0] - goal[0], current[1] - goal[1])
                yaw_error = abs(math.atan2(math.sin(current[2] - goal[2]), math.cos(current[2] - goal[2])))
                if position_error <= .20 and yaw_error <= .25:
                    report['result'] = 'GOAL_REACHED'
                    break
                raw_array, bounded_array = policy.predict(vectorize(sample['features']))
                raw, bounded = raw_array.tolist(), bounded_array.tolist()
                report['inference_steps'] += 1
                report['clipped_steps'] += int(any(abs(a - b) > 1e-10 for a, b in zip(raw, bounded)))
                has_moved |= abs(bounded[0]) + abs(bounded[1]) > 1e-4
            command = Twist()
            command.linear.x, command.angular.z = bounded
            publisher.publish(command)
            commands.write(json.dumps({'wall_s': now, 'sim_s': action['sim_s'], 'raw': raw,
                'bounded': bounded, 'guard_reason': reason, 'sample': sample}, allow_nan=False) + '\n')
            if reason is not None and (has_moved or now - started > 8.):
                report['result'], report['guard_reason'] = 'GUARD_STOP', reason
                report['guard_stops'] += 1
                break
        else:
            report['result'] = 'TIMEOUT'
        report['control_wall_seconds'] = time.monotonic() - started
    except (Exception, KeyboardInterrupt) as exc:
        report['result'], report['error'] = type(exc).__name__, str(exc)
    finally:
        planner.cancel()
        # Keep zero command input alive through the smoother while observing the stop.
        if rclpy.ok():
            deadline = time.monotonic() + 2.
            while time.monotonic() < deadline:
                publisher.publish(Twist())
                rclpy.spin_once(nav, timeout_sec=.05)
        final = log.map_pose()
        goal = configured['goal']
        report['final_map_pose'] = final
        report['position_error_m'] = math.hypot(final[0] - goal[0], final[1] - goal[1]) if final else None
        report['yaw_error_rad'] = abs(math.atan2(math.sin(final[2] - goal[2]), math.cos(final[2] - goal[2]))) if final else None
        report['odom_path_length_m'] = log.path_length
        report['plans_completed'] = planner.count
        report['follow_path_executing_detected'] = competing['active']
        odom = log.last_odom
        report['stopped_and_fresh'] = bool(odom is not None and time.monotonic() - log.odom_received < .5 and
            abs(odom.twist.twist.linear.x) < .005 and abs(odom.twist.twist.angular.z) < .02)
        report['ok'] = bool(report['result'] == 'GOAL_REACHED' and final and report['position_error_m'] <= .25 and
            report['yaw_error_rad'] <= .25 and report['stopped_and_fresh'] and not competing['active'] and
            report['guard_stops'] == 0)
        commands.close()
        if rclpy.ok():
            try:
                nav.lifecycleShutdown()
                report['nav2_lifecycle_shutdown_complete'] = True
            except Exception as exc:
                report['ok'] = False
                report['shutdown_error'] = str(exc)
        # lifecycleShutdown spins this node: subscriptions must retain open writers.
        log.close(report)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
