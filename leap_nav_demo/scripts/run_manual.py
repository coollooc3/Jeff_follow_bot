#!/usr/bin/env python3
"""Keep one isolated Nav2 simulation alive for manual goal/status/cancel commands."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from nav_task import require_domain
from session_process import check_empty_simulation, stop


def observe(directory):
    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from nav_task import TaskClient
    from nav_recording import Recorder, pose
    from task_config import make_task
    from action_msgs.msg import GoalStatusArray
    from rclpy.qos import QoSProfile, DurabilityPolicy
    # Keep the ROS context alive during ordered cancellation/lifecycle shutdown.
    quitting = False

    def interrupt(signum, frame):
        nonlocal quitting
        quitting = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator('leap_manual_observer')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    recorder = Recorder(nav, directory, make_task('baseline'))
    client = TaskClient(nav)
    history = nav.create_publisher(GoalStatusArray, '/leap_nav_demo/task_history',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    report = {'ok': False, 'result': 'starting', 'simulation_only': True}
    try:
        while nav.get_clock().now().nanoseconds == 0 and not quitting:
            rclpy.spin_once(nav, timeout_sec=0.1)
        if quitting:
            return 1
        nav.setInitialPose(pose(nav, -2.0, -0.5, 0.0))
        nav.waitUntilNav2Active()
        (directory / 'ready.json').write_text(json.dumps({'ready': True, 'initialization_calls': 1}) + '\n')
        previous = None
        while not quitting:
            client.spin(0.1)
            if client.states != previous:
                recorder.event('action_states', goals=client.states.copy())
                message = GoalStatusArray()
                message.status_list = list(client.history.values())
                history.publish(message)
                previous = client.states.copy()
        recorder.event('session_cancel_requested')
        report['cancellation'] = client.cancel()
        nav.lifecycleShutdown()
        report.update(ok=True, result='closed', goals=client.states)
    except Exception as exc:
        report.update(result=type(exc).__name__, error=str(exc))
    finally:
        (directory / 'observer.json').write_text(json.dumps(report, indent=2) + '\n')
        recorder.close(report)
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report['ok'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--seconds', type=int, default=1800, help='wall time after READY, 30..3600')
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'leap_nav_data')
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        require_domain()
    except ValueError as exc:
        parser.error(str(exc))
    if args.worker:
        return observe(args.worker)
    if not 30 <= args.seconds <= 3600:
        parser.error('--seconds must be 30..3600')
    check_empty_simulation(parser)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-%fZ')
    directory = args.output_root / ('nav-manual-' + stamp)
    directory.mkdir(parents=True)
    print('SAVED:', directory, flush=True)
    sim = observer = None
    report = {'ok': False, 'result': 'starting', 'simulation_only': True, 'ros_domain_id': 178,
              'controller': 'rpp', 'seconds_after_ready': args.seconds}
    with (directory / 'launch.log').open('w') as launch_log, (directory / 'observer.log').open('w') as observer_log:
        try:
            sim = subprocess.Popen(['ros2', 'launch', 'leap_nav_demo', 'navigation.launch.py',
                'gui:=' + str(args.gui).lower(), 'controller:=rpp',
                'params_output:=' + str(directory / 'nav2_params_used.yaml')],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            observer = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker', str(directory)],
                stdout=observer_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline, ready = time.monotonic() + 90, False
            while time.monotonic() < deadline:
                if sim.poll() is not None or observer.poll() is not None:
                    raise RuntimeError('simulation or observer exited; see logs')
                if not ready and (directory / 'ready.json').is_file():
                    ready = True
                    deadline = time.monotonic() + args.seconds
                    print('READY: use nav_task goal/status/cancel in another terminal; Ctrl+C closes this session.', flush=True)
                time.sleep(0.1)
            if not ready:
                raise TimeoutError('Nav2 startup exceeded 90s')
            report['result'] = 'session_time_limit'
        except KeyboardInterrupt:
            report['result'] = 'operator_exit'
        except Exception as exc:
            report.update(result=type(exc).__name__, error=str(exc))
        finally:
            report['cleanup'] = {'observer': stop(observer, interrupt_group=True), 'simulation': stop(sim)}
            observer_result = directory / 'observer.json'
            report['observer'] = json.loads(observer_result.read_text()) if observer_result.is_file() else {}
            report['ok'] = (report['result'] in ('operator_exit', 'session_time_limit') and
                            report['observer'].get('ok', False) and
                            not any(item['survivors'] for item in report['cleanup'].values()))
            params = directory / 'nav2_params_used.yaml'
            if params.is_file():
                report['params_used_sha256'] = hashlib.sha256(params.read_bytes()).hexdigest()
            (directory / 'session.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
