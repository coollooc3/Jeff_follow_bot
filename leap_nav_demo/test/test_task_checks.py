"""Terminal status must match the request and be backed by fresh stopping evidence."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from task_checks import StopProbe, task_failures


def report(scenario='baseline'):
    return {'scenario': scenario, 'result': 'SUCCEEDED', 'command_observed': True,
            'last_command_zero': True, 'odom_stopped_and_fresh': True,
            'position_error_m': 0.1, 'yaw_error_rad': 0.1, 'odom_path_length_m': 1.0,
            'plans_received': 1, 'nonzero_commands_received': 1, 'task_wall_seconds': 5.0,
            'planner_aborted_goal_count': 0}


def canceled_report():
    value = report('cancel')
    value['result'] = 'CANCELED'
    value['cancellation'] = {'triggered': True, 'result_after_request_s': 0.1,
        'stop': {'first_stopped_after_request_s': 0.2,
                 'max_odom_displacement_after_request_m': 0.01, 'invalid_samples': 0}}
    return value


def occupied_report():
    value = report('blocked_goal')
    value.update(result='FAILED', command_observed=False, last_command_zero=False,
                 nonzero_commands_received=0, odom_path_length_m=0.0,
                 planner_aborted_goal_count=1,
                 obstacle={'ok': True, 'required_point_cost': 100})
    return value


class StopProbeTests(unittest.TestCase):
    def test_invalid_request_position_cannot_produce_a_distance_bound(self):
        with self.assertRaises(ValueError):
            StopProbe(10.0, (float('nan'), 0.0))

    def test_pre_request_and_invalid_samples_do_not_prove_a_stop(self):
        probe = StopProbe(10.0, (0.0, 0.0))
        probe.observe(9.9, (0.0, 0.0), 0.0, 0.0)
        probe.observe(10.1, (0.0, 0.0), float('nan'), 0.0)
        self.assertIsNone(probe.result()['first_stopped_after_request_s'])
        self.assertEqual(probe.result()['invalid_samples'], 1)

    def test_rotation_cannot_be_counted_as_stopped(self):
        probe = StopProbe(10.0, (0.0, 0.0))
        probe.observe(10.1, (0.0, 0.0), 0.0, 0.4)
        self.assertIsNone(probe.result()['first_stopped_after_request_s'])
        probe.observe(10.2, (0.01, 0.0), 0.0, 0.0)
        self.assertAlmostEqual(probe.result()['first_stopped_after_request_s'], 0.2)

    def test_motion_after_first_stop_remains_visible(self):
        probe = StopProbe(10.0, (0.0, 0.0))
        probe.observe(10.1, (0.01, 0.0), 0.0, 0.0)
        probe.observe(10.4, (0.08, 0.0), 0.1, 0.0)
        self.assertAlmostEqual(probe.result()['max_odom_displacement_after_request_m'], 0.08)


class TaskContractTests(unittest.TestCase):
    def test_normal_goal_still_requires_success_and_position_limit(self):
        self.assertEqual(task_failures(report()), [])
        for changed in ({'result': 'FAILED'}, {'position_error_m': 0.26}):
            value = report()
            value.update(changed)
            self.assertTrue(task_failures(value))

    def test_unknown_scenario_fails_closed(self):
        self.assertIn('unknown_scenario', task_failures(report('unknown')))

    def test_canceled_and_stopped_passes(self):
        self.assertEqual(task_failures(canceled_report()), [])

    def test_success_does_not_satisfy_a_cancellation_request(self):
        value = canceled_report()
        value['result'] = 'SUCCEEDED'
        self.assertIn('cancel_result_not_canceled', task_failures(value))

    def test_invalid_sample_cannot_be_hidden_by_a_later_valid_stop(self):
        value = canceled_report()
        value['cancellation']['stop']['invalid_samples'] = 1
        self.assertIn('invalid_cancel_stop_samples', task_failures(value))

    def test_late_stop_result_or_excess_displacement_fails(self):
        cases = [('result_after_request_s', 2.1),
                 ('first_stopped_after_request_s', 2.1),
                 ('first_stopped_after_request_s', None),
                 ('max_odom_displacement_after_request_m', 0.051)]
        for key, value in cases:
            with self.subTest(key=key, value=value):
                changed = canceled_report()
                target = changed['cancellation'] if key == 'result_after_request_s' else changed['cancellation']['stop']
                target[key] = value
                self.assertTrue(task_failures(changed))

    def test_stale_odom_or_absent_motion_cannot_pass_cancellation(self):
        for changed in ({'odom_stopped_and_fresh': False}, {'nonzero_commands_received': 0}):
            value = canceled_report()
            value.update(changed)
            self.assertTrue(task_failures(value))

    def test_rejected_occupied_goal_can_pass_without_ever_receiving_commands(self):
        self.assertEqual(task_failures(occupied_report()), [])

    def test_success_or_unrelated_failure_does_not_prove_occupied_goal_rejection(self):
        for changed in ({'result': 'SUCCEEDED'}, {'planner_aborted_goal_count': 0},
                        {'result': 'RuntimeError'}):
            value = occupied_report()
            value.update(changed)
            self.assertTrue(task_failures(value))

    def test_missing_occupied_cell_or_sensor_evidence_fails(self):
        for changed in ({'required_point_cost': None}, {'required_point_cost': 0}, {'ok': False}):
            value = occupied_report()
            value['obstacle'].update(changed)
            self.assertTrue(task_failures(value))

    def test_occupied_goal_cannot_pass_after_motion_or_timeout(self):
        for changed in ({'nonzero_commands_received': 1}, {'odom_path_length_m': 0.03},
                        {'task_wall_seconds': 10.1}):
            value = occupied_report()
            value.update(changed)
            self.assertTrue(task_failures(value))


if __name__ == '__main__':
    unittest.main()
