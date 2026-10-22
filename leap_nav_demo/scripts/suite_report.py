"""Recheck recorded contracts and summarize attempts without pooling configurations."""
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

from task_checks import task_failures
from task_config import read_task


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def reject_nonfinite(value):
    raise ValueError('non-finite JSON number: ' + value)


def evaluate_attempt(directory, scenario, controller, runner_exit, expected_task_hash=None):
    row = {'attempt': str(directory), 'scenario': scenario, 'controller': controller,
           'runner_exit': runner_exit, 'passed': False, 'cleanup_verified': False,
           'actual_result': None, 'failure_reasons': [], 'versions': []}
    failures = row['failure_reasons']
    try:
        sessions = sorted(Path(directory).glob('nav-task-*'))
        if len(sessions) != 1:
            raise ValueError('expected exactly one recorded session')
        path = sessions[0]
        row['session'] = str(path)
        task = json.loads((path / 'result.json').read_text(), parse_constant=reject_nonfinite)
        session = json.loads((path / 'session.json').read_text(), parse_constant=reject_nonfinite)
        if not isinstance(task['result'], str) or not isinstance(task['expected_result'], str):
            raise ValueError('task results must be strings')
        if not isinstance(session['versions'], list) or not all(isinstance(v, str) for v in session['versions']):
            raise ValueError('software versions must be a list of strings')
        row.update(actual_result=task['result'], expected_result=task['expected_result'],
                   params_sha256=session.get('params_used_sha256'),
                   behavior_tree_sha256=task.get('behavior_tree_sha256'),
                   retry_behavior_tree_sha256=task.get('retry_behavior_tree_sha256'),
                   task_config_sha256=task.get('task_config_sha256'),
                   versions=session['versions'])
        if scenario == 'blocked_route':
            phases = task['route_recovery']
            row['phase_results'] = [phases.get(key, {}).get('result') for key in ('closed_attempt', 'retry_attempt')]
        cleanup = session['cleanup']
        row['cleanup_verified'] = all(
            cleanup[name]['survivors'] == [] and cleanup[name]['exit_code'] is not None
            for name in ('task', 'simulation'))
        if session.get('result') == 'KeyboardInterrupt':
            failures.append('session_interrupted')
            return row
        for condition, reason in (
                (runner_exit == 0, 'runner_exit_nonzero'),
                (task['ok'] is True and session['ok'] is True, 'recorded_verdict_failed'),
                (task['scenario'] == scenario == session['scenario'], 'scenario_mismatch'),
                (task['expected_result'] == session['expected_task_result'] ==
                 {'cancel': 'CANCELED', 'blocked_goal': 'FAILED'}.get(scenario, 'SUCCEEDED'),
                 'expected_result_mismatch'),
                (session['controller'] == controller, 'controller_mismatch'),
                (session['simulation_only'] is True and session['ros_domain_id'] == 178,
                 'simulation_isolation_not_recorded'),
                (session['task_exit'] == 0 and session['unexpected_launch_exit'] is None,
                 'task_or_launch_exit_failed'),
                (session['task_result'] == task['result'], 'result_mismatch'),
                (row['cleanup_verified'], 'cleanup_unconfirmed')):
            if not condition:
                failures.append(reason)
        failures.extend(task_failures(task))
        if row['task_config_sha256'] is not None or session.get('task_config_sha256') is not None:
            configuration_path = path / 'task_config_used.json'
            configuration = read_task(configuration_path, scenario)
            digest = hashlib.sha256(configuration_path.read_bytes()).hexdigest()
            if digest != row['task_config_sha256'] or digest != session.get('task_config_sha256'):
                failures.append('task_config_snapshot_mismatch')
            if (task['initial_map_pose'] != configuration['start'] or task['goal_map_pose'] != configuration['goal']
                    or task['required_path_length_m'] != configuration['required_path_length_m']):
                failures.append('task_config_result_mismatch')
        if expected_task_hash is not None and row['task_config_sha256'] != expected_task_hash:
            failures.append('requested_task_config_mismatch')
        params = (path / 'nav2_params_used.yaml').read_bytes()
        if hashlib.sha256(params).hexdigest() != row['params_sha256']:
            failures.append('params_snapshot_mismatch')
        if scenario in ('blocked_goal', 'blocked_route'):
            tree = (path / 'behavior_tree_used.xml').read_bytes()
            if hashlib.sha256(tree).hexdigest() != row['behavior_tree_sha256']:
                failures.append('behavior_tree_snapshot_mismatch')
        if scenario == 'blocked_route':
            retry_tree = (path / 'retry_behavior_tree_used.xml').read_bytes()
            if hashlib.sha256(retry_tree).hexdigest() != task['retry_behavior_tree_sha256']:
                failures.append('retry_behavior_tree_snapshot_mismatch')
            from route_checks import connectivity
            for name in ('original', 'closed', 'opened'):
                snapshot = json.loads((path / ('route-' + name + '.json')).read_text(), parse_constant=reject_nonfinite)
                recomputed = connectivity(snapshot['grid'], snapshot['start'], snapshot['goal'])
                if (snapshot['start'] != task['initial_map_pose'][:2] or snapshot['goal'] != task['goal_map_pose'][:2] or
                        any(snapshot['proof'].get(key) != value or
                            phases[name + '_connectivity'].get(key) != value for key, value in recomputed.items())):
                    failures.append('route_grid_proof_mismatch:' + name)
        for field in ('task_wall_seconds', 'odom_path_length_m', 'recoveries_reported_by_nav2'):
            value = task[field]
            row[field] = value if finite_number(value) and value >= 0 else None
            if row[field] is None:
                failures.append('invalid_metric:' + field)
        row['robot_contact_messages'] = task.get('obstacle', {}).get('robot_contact_messages')
        stop = task.get('cancellation', {}).get('stop', {})
        row['first_low_speed_after_cancel_s'] = stop.get('first_stopped_after_request_s')
        row['post_cancel_displacement_m'] = stop.get('max_odom_displacement_after_request_m')
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        failures.append('evidence_error:' + str(exc))
    row['passed'] = not failures
    return row


def summarize(rows):
    groups = {}
    for row in rows:
        key = (row['scenario'], row['controller'], row.get('params_sha256'),
               row.get('behavior_tree_sha256'), row.get('retry_behavior_tree_sha256'), row.get('task_config_sha256'),
               json.dumps(row['versions'], sort_keys=True))
        groups.setdefault(key, []).append(row)
    result = []
    for (scenario, controller, params, tree, retry_tree, task_config, versions), attempts in groups.items():
        passed = [row for row in attempts if row['passed']]
        times = [row['task_wall_seconds'] for row in passed]
        recoveries = [row['recoveries_reported_by_nav2'] for row in passed]
        result.append({'scenario': scenario, 'controller': controller, 'params_sha256': params,
                       'behavior_tree_sha256': tree, 'versions': json.loads(versions),
                       'retry_behavior_tree_sha256': retry_tree,
                       'task_config_sha256': task_config,
                       'attempts': len(attempts), 'passed': len(passed),
                       'actual_results': dict(Counter(row['actual_result'] or 'NO_RESULT' for row in attempts)),
                       'action_phase_sequences': dict(Counter(' -> '.join(row['phase_results']) for row in attempts
                                                             if row.get('phase_results') and all(row['phase_results']))),
                       'passed_task_seconds': ({'count': len(times), 'median': statistics.median(times),
                                              'min': min(times), 'max': max(times)} if times else None),
                       'passed_with_recovery': sum(value > 0 for value in recoveries),
                       'max_recovery_feedback_passed': max(recoveries) if recoveries else None})
    return result


def write_reports(directory, rows, plan, interrupted=False, stop_reason=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    groups = summarize(rows)
    report = {'schema_version': 1, 'simulation_only': True, 'plan': plan,
              'planned_attempts': len(plan), 'completed_attempts': len(rows),
              'passed_attempts': sum(row['passed'] for row in rows),
              'interrupted': interrupted, 'stop_reason': stop_reason,
              'ok': len(rows) == len(plan) and all(row['passed'] for row in rows) and not interrupted,
              'groups': groups, 'attempts': rows,
              'limits': 'Repeated fixed scenes, not randomized success rates. '
                        'Times describe passing attempts only. Arrival/termination contracts '
                        'do not certify smoothness. Injected-box contacts only; no LEAP validation.'}
    # Publish each completed report atomically, retaining partial-batch evidence.
    temporary = directory / 'suite.json.tmp'
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    temporary.replace(directory / 'suite.json')
    fields = ['scenario', 'controller', 'passed', 'actual_result', 'expected_result', 'phase_results',
              'task_wall_seconds', 'odom_path_length_m', 'recoveries_reported_by_nav2',
              'robot_contact_messages', 'first_low_speed_after_cancel_s',
              'post_cancel_displacement_m', 'cleanup_verified', 'params_sha256',
              'behavior_tree_sha256', 'task_config_sha256', 'runner_exit', 'session', 'attempt', 'failure_reasons']
    fields.insert(fields.index('behavior_tree_sha256') + 1, 'retry_behavior_tree_sha256')
    with (directory / 'attempts.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows({**row, 'failure_reasons': json.dumps(row['failure_reasons']),
                          'phase_results': json.dumps(row.get('phase_results', []))} for row in rows)
    lines = ['# Nav2 fixed-scene evaluation', '',
             f"Completed {len(rows)}/{len(plan)}; scenario checks passed {report['passed_attempts']}/{len(rows)}.",
             '', '| Scenario | Controller | Params / task hash prefixes | Passed / attempts | Actual Nav2 results | Passing task seconds median [min, max] | Max recovery feedback (passing) |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for group in groups:
        times = group['passed_task_seconds']
        duration = f"{times['median']:.3f} [{times['min']:.3f}, {times['max']:.3f}]" if times else 'unavailable'
        lines.append(f"| {group['scenario']} | {group['controller']} | {(group['params_sha256'] or 'missing')[:12]} / {(group['task_config_sha256'] or 'legacy-default')[:12]} "
                     f"| {group['passed']}/{group['attempts']} | {json.dumps(group['actual_results'])}"
                     f"{(' phases: ' + json.dumps(group['action_phase_sequences'])) if group['action_phase_sequences'] else ''} "
                     f"| {duration} | {group['max_recovery_feedback_passed']} |")
    lines += ['', report['limits'], '', 'Full parameter, behavior-tree hashes and software versions are in suite.json.', '']
    for row in rows:
        if not row['passed']:
            lines.append(f"- Failed attempt {row['attempt']}: {'; '.join(row['failure_reasons'])}")
    if stop_reason:
        lines += ['', 'Batch stopped: ' + stop_reason]
    (directory / 'summary.md').write_text('\n'.join(lines) + '\n')
    return report
