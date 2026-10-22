"""Start/goal validation, canonical snapshots and non-default task thresholds."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from task_config import DEFAULT_START, DEFAULT_GOAL, make_task, encode_task, read_task, task_hash
from task_checks import task_failures
from test_task_checks import report


class TaskConfigTests(unittest.TestCase):
    def test_default_task_and_motion_requirement_stay_the_same(self):
        task = make_task('baseline')
        self.assertEqual(task['start'], list(DEFAULT_START))
        self.assertEqual(task['goal'], list(DEFAULT_GOAL))
        self.assertEqual(task['required_path_length_m'], .5)

    def test_nonfinite_boolean_and_nearby_poses_are_rejected(self):
        for start in ([float('nan'), 0, 0], [False, 0, 0], [-2, 1, 0]):
            with self.subTest(start=start), self.assertRaises(ValueError):
                make_task('baseline', start=start)

    def test_custom_angles_are_normalized_and_task_identity_changes(self):
        task = make_task('baseline', [-2, .8, -5 * math.pi / 2], [-2, -.8, -math.pi / 2])
        self.assertAlmostEqual(task['start'][2], -math.pi / 2)
        self.assertNotEqual(task_hash(task), task_hash(make_task('baseline')))

    def test_fixed_obstacle_scenes_do_not_silently_change_geometry(self):
        for scenario in ('obstacle', 'sudden_obstacle', 'blocked_goal', 'blocked_route'):
            with self.subTest(scenario=scenario), self.assertRaises(ValueError):
                make_task(scenario, goal=[-2, -.8, 0])

    def test_round_trip_and_modified_motion_threshold(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'task.json'
            task = make_task('baseline')
            path.write_bytes(encode_task(task))
            self.assertEqual(read_task(path, 'baseline'), task)
            task['required_path_length_m'] = .01
            path.write_bytes(encode_task(task))
            with self.assertRaises(ValueError):
                read_task(path, 'baseline')

    def test_in_range_yaws_survive_exact_serialization_round_trips(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'task.json'
            for yaw in (.1, -.1, .3, 2.8, math.pi, -math.pi, .09999999999999999):
                with self.subTest(yaw=yaw):
                    task = make_task('baseline', [-1.95, -.8, yaw], [-1.95, .75, 2.8])
                    self.assertEqual(task['start'][2], yaw)
                    payload = encode_task(task)
                    path.write_bytes(payload)
                    self.assertEqual(read_task(path, 'baseline'), task)
                    self.assertEqual(path.read_bytes(), payload)

    def test_wrapped_angles_are_idempotent(self):
        for yaw in (17 * math.pi, -.1 - 6 * math.pi, .1 + 6 * math.pi, 1e6):
            with self.subTest(yaw=yaw):
                first = make_task('baseline', [-2, -.8, yaw], [-2, .75, -yaw])
                second = make_task('baseline', first['start'], first['goal'])
                self.assertTrue(-math.pi <= first['start'][2] <= math.pi)
                self.assertEqual(encode_task(first), encode_task(second))

    def test_original_rejected_snapshot_becomes_readable_with_same_hash(self):
        # Exact serialized yaw from the first failed holdout, retained as evidence.
        task = {'schema_version': 1, 'map': 'turtlebot3_world',
                'start': [-1.95, -.8, .09999999999999999], 'goal': [-1.95, .75, 2.8],
                'route': 'custom', 'required_path_length_m': .5}
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'task.json'
            path.write_bytes(encode_task(task))
            restored = read_task(path, 'baseline')
            self.assertEqual(task_hash(restored), 'e12e7ae8580558c5bce02cad5fe65cefef01e1caef5dd1dfeb4be26ee4957c32')

    def test_shorter_trials_scale_motion_evidence_without_changing_goal_tolerance(self):
        task = make_task('baseline', goal=[-2, .1, 0])
        value = report()
        value.update(required_path_length_m=task['required_path_length_m'], odom_path_length_m=.3)
        self.assertEqual(task_failures(value), [])
        value['position_error_m'] = .26
        self.assertIn('position_outside_limit', task_failures(value))


if __name__ == '__main__':
    unittest.main()
