#!/usr/bin/env python3
"""Start, record and clean up one isolated simulation navigation demonstration."""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

from session_process import stop, check_empty_simulation
from task_checks import SCENARIOS
from task_config import make_task, encode_task, task_hash



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--scenario', choices=SCENARIOS, default='baseline')
    parser.add_argument('--controller', choices=['dwb', 'rpp', 'bc'], default='dwb')
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--record-expert', action='store_true')
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'leap_nav_data')
    args = parser.parse_args()
    if args.scenario == 'blocked_route' and args.record_expert:
        parser.error('blocked_route is a task-recovery scenario, not an expert-data episode')
    if args.controller == 'bc':
        if args.scenario != 'baseline' or args.record_expert or args.policy is None:
            parser.error('bc requires --policy, baseline, and no --record-expert')
        from bc_policy import Policy
        Policy.load(args.policy)
    elif args.policy is not None:
        parser.error('--policy is only supported with --controller bc')
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    try:
        configured_task = make_task(args.scenario, args.start, args.goal)
    except ValueError as exc:
        parser.error(str(exc))
    check_empty_simulation(parser)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-%fZ')
    directory = args.output_root / ('nav-task-' + stamp)
    directory.mkdir(parents=True)
    print('SAVED:', directory, flush=True)
    (directory / 'task_config_used.json').write_bytes(encode_task(configured_task))
    sim, task = None, None
    report = {'ok': False, 'ros_domain_id': 178, 'result': 'starting', 'simulation_only': True,
              'scenario': args.scenario, 'controller': args.controller,
              'task_config_sha256': task_hash(configured_task), 'record_expert': args.record_expert}
    from ament_index_python.packages import get_package_share_directory
    base = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    report['upstream_params_sha256'] = hashlib.sha256(base.read_bytes()).hexdigest()
    report['versions'] = subprocess.check_output([
        'dpkg-query', '-W', '-f=${Package} ${Version}\n', 'ros-humble-nav2-bringup',
        'ros-humble-nav2-simple-commander', 'ros-humble-turtlebot3-gazebo',
        'ros-humble-nav2-regulated-pure-pursuit-controller'], text=True).splitlines()
    with (directory / 'launch.log').open('w') as launch_log, (directory / 'task.log').open('w') as task_log:
        try:
            sim = subprocess.Popen(['ros2', 'launch', 'leap_nav_demo', 'navigation.launch.py',
                                    'gui:=' + str(args.gui).lower(),
                                    'controller:=' + ('rpp' if args.controller == 'bc' else args.controller),
                                    'strict_goal:=' + str(args.scenario == 'blocked_goal').lower(),
                                    'start_x:=' + str(configured_task['start'][0]),
                                    'start_y:=' + str(configured_task['start'][1]),
                                    'start_yaw:=' + str(configured_task['start'][2]),
                                    'params_output:=' + str(directory / 'nav2_params_used.yaml')],
                                   stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            if args.controller == 'bc':
                worker = ['bc_trial', '--policy', str(args.policy.resolve())]
            elif args.scenario == 'blocked_route':
                worker = ['blocked_route_trial']
            else:
                worker = ['navigate_once', '--scenario', args.scenario,
                          *(['--record-expert'] if args.record_expert else [])]
            task = subprocess.Popen(['ros2', 'run', 'leap_nav_demo', *worker,
                                     '--output', str(directory),
                                     '--task-config', str(directory / 'task_config_used.json')], stdout=task_log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            # Includes 60s startup plus 90s task and cancellation/settling overhead.
            report['task_exit'] = task.wait(timeout=240 if args.scenario == 'blocked_route' else 165)
            result_file = directory / 'result.json'
            task_report = json.loads(result_file.read_text()) if result_file.is_file() else {}
            report['task_result'] = task_report.get('result')
            report['expected_task_result'] = task_report.get('expected_result')
            report['ok'] = report['task_exit'] == 0 and task_report.get('ok', False)
            report['result'] = 'passed' if report['ok'] else 'task_failed'
            report['unexpected_launch_exit'] = sim.poll()
            if report['unexpected_launch_exit'] is not None:
                report['ok'], report['result'] = False, 'launch_exited'
        except subprocess.TimeoutExpired:
            report['result'] = 'session_wall_timeout'
        except (Exception, KeyboardInterrupt) as exc:
            report['result'], report['error'] = type(exc).__name__, str(exc)
        finally:
            # Interrupt the actual task executable, not only the ros2 run wrapper,
            # before stopping the simulation. Its ROS context may already be closing.
            report['cleanup'] = {'task': stop(task, interrupt_group=True), 'simulation': stop(sim)}
            params_file = directory / 'nav2_params_used.yaml'
            if params_file.is_file():
                report['params_used_sha256'] = hashlib.sha256(params_file.read_bytes()).hexdigest()
            if any(item['survivors'] for item in report['cleanup'].values()):
                report['ok'], report['result'] = False, 'cleanup_incomplete'
            (directory / 'session.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print((directory / 'task.log').read_text(errors='replace'), flush=True)
    print(json.dumps(report), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
