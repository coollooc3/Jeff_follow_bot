"""Batch denominators, result semantics, configuration isolation and interruption."""
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import run_suite
from suite_report import evaluate_attempt, summarize, write_reports


class SuiteTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def fixture(self, name='attempt', params=b'configuration A'):
        attempt = self.root / name
        session = attempt / 'nav-task-fixture'
        session.mkdir(parents=True)
        task = {'scenario': 'blocked_goal', 'ok': True, 'result': 'FAILED',
                'expected_result': 'FAILED', 'command_observed': False,
                'last_command_zero': False, 'odom_stopped_and_fresh': True,
                'nonzero_commands_received': 0, 'planner_aborted_goal_count': 1,
                'task_wall_seconds': 0.1, 'odom_path_length_m': 0.0,
                'recoveries_reported_by_nav2': 0,
                'obstacle': {'ok': True, 'required_point_cost': 100},
                'behavior_tree_sha256': hashlib.sha256(b'tree').hexdigest()}
        report = {'ok': True, 'scenario': 'blocked_goal', 'controller': 'rpp',
                  'simulation_only': True, 'ros_domain_id': 178, 'task_exit': 0,
                  'task_result': 'FAILED', 'expected_task_result': 'FAILED',
                  'unexpected_launch_exit': None,
                  'params_used_sha256': hashlib.sha256(params).hexdigest(),
                  'versions': ['nav2 fixture'],
                  'cleanup': {key: {'survivors': [], 'exit_code': 0}
                              for key in ('task', 'simulation')}}
        (session / 'nav2_params_used.yaml').write_bytes(params)
        (session / 'behavior_tree_used.xml').write_bytes(b'tree')
        self.save(session, task, report)
        return attempt, session, task, report

    def save(self, path, task, session):
        (path / 'result.json').write_text(json.dumps(task))
        (path / 'session.json').write_text(json.dumps(session))

    def test_expected_failed_navigation_is_a_passing_scenario(self):
        attempt, *_ = self.fixture()
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        self.assertTrue(row['passed'], row['failure_reasons'])
        self.assertEqual(summarize([row])[0]['actual_results'], {'FAILED': 1})

    def route_fixture(self):
        from route_checks import connectivity
        from task_config import encode_task, make_task, task_hash
        from test_route_checks import recovered_report
        attempt, path, task, session = self.fixture()
        config = make_task('blocked_route')
        task.update(recovered_report(), expected_result='SUCCEEDED', initial_map_pose=config['start'],
                    goal_map_pose=config['goal'], required_path_length_m=config['required_path_length_m'],
                    task_config_sha256=task_hash(config))
        task.pop('obstacle')
        phases = task['route_recovery']
        for name in ('closed_attempt', 'retry_attempt'):
            phases[name]['goal'] = list(config['goal'])
        grid = {'frame': 'map', 'width': 21, 'height': 31, 'resolution': .1,
                'origin': [-3., -1.5, 0.], 'data': [0] * 651}
        for name in ('original', 'closed', 'opened'):
            data = list(grid['data'])
            if name != 'original':
                for index in range(8, 13):
                    for x, y in ((8, index), (12, index), (index, 8), (index, 12)):
                        data[y * 21 + x] = 100
            if name == 'opened':
                for x in (9, 10, 11):
                    data[12 * 21 + x] = 0
            snapshot = {**grid, 'data': data}
            proof = connectivity(snapshot, config['start'][:2], config['goal'][:2])
            phases[name + '_connectivity'] = proof
            (path / ('route-' + name + '.json')).write_text(json.dumps(
                {'grid': snapshot, 'start': config['start'][:2], 'goal': config['goal'][:2], 'proof': proof}))
        task['retry_behavior_tree_sha256'] = hashlib.sha256(b'retry tree').hexdigest()
        (path / 'retry_behavior_tree_used.xml').write_bytes(b'retry tree')
        (path / 'task_config_used.json').write_bytes(encode_task(config))
        session.update(scenario='blocked_route', expected_task_result='SUCCEEDED', task_result='SUCCEEDED',
                       task_config_sha256=task_hash(config))
        self.save(path, task, session)
        return attempt, path, task, session

    def test_route_retry_preserves_failed_first_action_and_rechecks_saved_grids(self):
        attempt, *_ = self.route_fixture()
        result = evaluate_attempt(attempt, 'blocked_route', 'rpp', 0)
        self.assertTrue(result['passed'], result['failure_reasons'])
        self.assertEqual(result['phase_results'], ['FAILED', 'SUCCEEDED'])

    def test_modified_closed_grid_cannot_keep_a_recorded_no_route_verdict(self):
        attempt, path, *_ = self.route_fixture()
        file = path / 'route-closed.json'
        snapshot = json.loads(file.read_text())
        snapshot['grid']['data'][12 * 21 + 10] = 0
        file.write_text(json.dumps(snapshot))
        result = evaluate_attempt(attempt, 'blocked_route', 'rpp', 0)
        self.assertIn('route_grid_proof_mismatch:closed', result['failure_reasons'])

    def test_modified_retry_tree_fails_the_two_phase_scenario(self):
        attempt, path, *_ = self.route_fixture()
        (path / 'retry_behavior_tree_used.xml').write_bytes(b'changed')
        result = evaluate_attempt(attempt, 'blocked_route', 'rpp', 0)
        self.assertIn('retry_behavior_tree_snapshot_mismatch', result['failure_reasons'])

    def test_different_retry_behavior_trees_are_not_pooled(self):
        attempt, *_ = self.route_fixture()
        first = evaluate_attempt(attempt, 'blocked_route', 'rpp', 0)
        second = {**first, 'retry_behavior_tree_sha256': 'different-retry-tree'}
        groups = summarize([first, second])
        self.assertEqual(len(groups), 2)
        self.assertEqual(groups[0]['action_phase_sequences'], {'FAILED -> SUCCEEDED': 1})

    def test_stored_ok_cannot_override_the_actual_contract(self):
        attempt, path, task, session = self.fixture()
        task['result'] = session['task_result'] = 'SUCCEEDED'
        self.save(path, task, session)
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        self.assertIn('occupied_goal_not_rejected', row['failure_reasons'])

    def test_modified_parameter_snapshot_fails(self):
        attempt, path, *_ = self.fixture()
        (path / 'nav2_params_used.yaml').write_bytes(b'other configuration')
        self.assertIn('params_snapshot_mismatch',
                      evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)['failure_reasons'])

    def test_modified_behavior_tree_snapshot_fails(self):
        attempt, path, *_ = self.fixture()
        (path / 'behavior_tree_used.xml').write_bytes(b'other tree')
        self.assertIn('behavior_tree_snapshot_mismatch',
                      evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)['failure_reasons'])

    def test_wrong_expected_label_cannot_redefine_the_contract(self):
        attempt, path, task, session = self.fixture()
        task['expected_result'] = session['expected_task_result'] = 'SUCCEEDED'
        self.save(path, task, session)
        self.assertIn('expected_result_mismatch',
                      evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)['failure_reasons'])

    def test_missing_evidence_remains_in_denominator(self):
        row = evaluate_attempt(self.root / 'missing', 'blocked_goal', 'rpp', 2)
        group = summarize([row])[0]
        self.assertEqual((group['attempts'], group['passed']), (1, 0))
        self.assertEqual(group['actual_results'], {'NO_RESULT': 1})
        self.assertFalse(row['cleanup_verified'])

    def test_nonfinite_json_fails_without_breaking_report_output(self):
        attempt, path, task, session = self.fixture()
        task['task_wall_seconds'] = float('nan')
        self.save(path, task, session)
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        report = write_reports(self.root / 'report', [row], [{}])
        self.assertFalse(report['ok'])
        self.assertTrue(row['failure_reasons'][0].startswith('evidence_error:'))

    def test_different_configurations_and_versions_are_not_pooled(self):
        first, *_ = self.fixture('first')
        second, path, task, session = self.fixture('second', b'configuration B')
        third, third_path, third_task, third_session = self.fixture('third')
        third_session['versions'] = ['nav2 other version']
        self.save(third_path, third_task, third_session)
        rows = [evaluate_attempt(path, 'blocked_goal', 'rpp', 0) for path in (first, second, third)]
        self.assertEqual(len(summarize(rows)), 3)

    def test_failures_count_but_do_not_enter_passing_time_statistics(self):
        attempt, path, task, session = self.fixture()
        first = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        task['task_wall_seconds'] = 99
        self.save(path, task, session)
        second = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        group = summarize([first, second])[0]
        self.assertEqual((group['passed'], group['attempts']), (1, 2))
        self.assertEqual(group['passed_task_seconds']['median'], 0.1)

    def test_unconfirmed_cleanup_and_nonzero_runner_cannot_pass(self):
        attempt, path, task, session = self.fixture()
        session['cleanup']['simulation']['survivors'] = [123]
        self.save(path, task, session)
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 1)
        self.assertFalse(row['cleanup_verified'])
        self.assertIn('cleanup_unconfirmed', row['failure_reasons'])
        self.assertIn('runner_exit_nonzero', row['failure_reasons'])

    def test_partial_batch_cannot_report_success(self):
        attempt, *_ = self.fixture()
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        report = write_reports(self.root / 'report', [row], [{}, {}])
        self.assertFalse(report['ok'])
        self.assertEqual(report['completed_attempts'], 1)

    def test_different_task_configurations_are_not_pooled(self):
        attempt, *_ = self.fixture()
        first = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0)
        second = {**first, 'task_config_sha256': 'other-task'}
        self.assertEqual(len(summarize([first, second])), 2)

    def test_legacy_default_cannot_satisfy_a_new_requested_task(self):
        attempt, *_ = self.fixture()
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 0, 'requested-new-task')
        self.assertIn('requested_task_config_mismatch', row['failure_reasons'])

    def test_interruption_is_preserved_even_with_partial_task_metadata(self):
        attempt, path, task, session = self.fixture()
        session['result'] = task['result'] = 'KeyboardInterrupt'
        del session['expected_task_result']
        self.save(path, task, session)
        row = evaluate_attempt(attempt, 'blocked_goal', 'rpp', 1)
        self.assertFalse(row['passed'])
        self.assertTrue(row['cleanup_verified'])
        self.assertEqual(row['failure_reasons'], ['session_interrupted'])

    def test_interrupt_is_forwarded_and_cleanup_is_awaited(self):
        with patch.object(run_suite.subprocess, 'Popen') as popen, \
                patch.object(run_suite.os, 'killpg') as signal_group:
            process = popen.return_value
            process.pid = 4321
            process.wait.side_effect = [KeyboardInterrupt(), 1]
            process.poll.return_value = None
            code, interrupted = run_suite.run_child(['fixture'], self.root / 'runner.log')
            self.assertEqual(code, 1)
            self.assertTrue(interrupted)
            signal_group.assert_called_once_with(4321, signal.SIGINT)
            process.send_signal.assert_not_called()
            self.assertTrue(popen.call_args.kwargs['start_new_session'])

    def test_robot_domain_is_rejected_before_starting_children(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}), \
                patch.object(sys, 'argv', ['run_suite']), \
                patch.object(run_suite, 'run_child') as start, self.assertRaises(SystemExit):
            run_suite.main()
        start.assert_not_called()


if __name__ == '__main__':
    unittest.main()
