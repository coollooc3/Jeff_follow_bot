#!/usr/bin/env python3
"""Run bounded simulation sessions sequentially, retaining every attempted result."""
import argparse
import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

from suite_report import evaluate_attempt, write_reports
from task_checks import SCENARIOS
from task_config import make_task, task_hash


def run_child(command, log_path):
    with log_path.open('w') as log:
        # ros2 run assumes its executable also receives terminal SIGINT. Signal
        # this owned group, not only the CLI wrapper; run_demo owns other groups.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            return process.wait(), False
        except KeyboardInterrupt:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                try:
                    return process.wait(timeout=max(0.01, deadline - time.monotonic())), True
                except KeyboardInterrupt:
                    continue  # Allow run_demo to finish its owned-process cleanup.
                except subprocess.TimeoutExpired:
                    break
            return None, True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenarios', choices=SCENARIOS, nargs='+', default=list(SCENARIOS))
    parser.add_argument('--controller', choices=['dwb', 'rpp'], default='rpp')
    parser.add_argument('--repetitions', type=int, choices=range(1, 11), default=1)
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--record-expert', action='store_true')
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'leap_nav_data')
    args = parser.parse_args()
    if args.record_expert and 'blocked_route' in args.scenarios:
        parser.error('choose explicit expert-data scenarios without blocked_route')
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=178 and ROS_LOCALHOST_ONLY=1')
    if len(set(args.scenarios)) != len(args.scenarios):
        parser.error('list each scenario once; use --repetitions to repeat')
    try:
        configurations = {scenario: make_task(scenario, args.start, args.goal) for scenario in args.scenarios}
    except ValueError as exc:
        parser.error(str(exc))
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-%fZ')
    directory = args.output_root / ('nav-suite-' + stamp)
    directory.mkdir(parents=True)
    plan = [{'repetition': repetition, 'scenario': scenario, 'controller': args.controller,
             'task_config': configurations[scenario], 'task_config_sha256': task_hash(configurations[scenario]),
             'record_expert': args.record_expert}
            for repetition in range(1, args.repetitions + 1) for scenario in args.scenarios]
    rows = []
    interrupted, stop_reason = False, None
    write_reports(directory, rows, plan)
    print('SUITE:', directory, flush=True)
    try:
        for index, item in enumerate(plan, 1):
            attempt = directory / f'attempt-{index:03d}-{item["scenario"]}'
            attempt.mkdir()
            print(f'RUN {index}/{len(plan)}: {item["scenario"]}', flush=True)
            command = ['ros2', 'run', 'leap_nav_demo', 'run_demo', '--scenario', item['scenario'],
                       '--controller', args.controller, '--output-root', str(attempt),
                       '--start', *map(str, item['task_config']['start']),
                       '--goal', *map(str, item['task_config']['goal']),
                       *(['--record-expert'] if args.record_expert else [])]
            code, interrupted = run_child(command, attempt / 'runner.log')
            row = evaluate_attempt(attempt, item['scenario'], args.controller, code, item['task_config_sha256'])
            rows.append(row)
            if interrupted:
                stop_reason = 'interrupt requested; remaining attempts skipped'
            elif not row['cleanup_verified']:
                stop_reason = 'session evidence or process cleanup unconfirmed; remaining attempts skipped'
            write_reports(directory, rows, plan, interrupted, stop_reason)
            print(f"RESULT {index}: {row['actual_result']}; scenario_passed={row['passed']}", flush=True)
            if stop_reason:
                break
    except KeyboardInterrupt:
        interrupted, stop_reason = True, 'interrupted between sessions'
    except OSError as exc:
        stop_reason = 'runner could not start: ' + str(exc)
        # Preserve a denominator entry for the attempted startup failure.
        rows.append(evaluate_attempt(attempt, item['scenario'], args.controller, None, item['task_config_sha256']))
    report = write_reports(directory, rows, plan, interrupted, stop_reason)
    print('REPORT:', directory / 'summary.md', flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
