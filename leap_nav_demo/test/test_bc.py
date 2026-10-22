"""Data leakage, artifact validity, action limits and guard behavior."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from bc_policy import FEATURE_SCHEMA, FEATURE_SIZE, Policy, guard_reason, vectorize
from bc_dataset import load_dataset
from task_config import make_task, encode_task, task_hash


def features():
    return {'scan_laser_frame': {'sector_min_m': [1.] * 36, 'finite_ray_counts': [10] * 36,
            'positive_infinite_ray_counts': [0] * 36, 'invalid_ray_counts': [0] * 36},
            'goal_xy_base_m': [1., .5], 'goal_yaw_error_sin_cos': [0., 1.],
            'odom_velocity': [.1, .2], 'path_reference_xy_base_m': [.3, .1]}


def artifact():
    return {'schema_version': 1, 'feature_schema': FEATURE_SCHEMA, 'mean': [0.] * FEATURE_SIZE,
            'std': [1.] * FEATURE_SIZE, 'action_scale': [.22, .6],
            'layers': [{'weight': np.zeros((2, FEATURE_SIZE)).tolist(), 'bias': [2., -2.]}]}


def episode(directory, start_y):
    directory.mkdir()
    task = make_task('baseline', [-2., start_y, 0.], [-2., 1., 1.57])
    (directory / 'task_config_used.json').write_bytes(encode_task(task))
    report = {'ok': True, 'result': 'SUCCEEDED', 'recoveries_reported_by_nav2': 0,
              'task_config_sha256': task_hash(task)}
    for filename in ('result.json', 'session.json'):
        (directory / filename).write_text(json.dumps(report))
    (directory / 'expert_summary.json').write_text(json.dumps({'episode_candidate': True}))
    (directory / 'expert_metadata.json').write_text(json.dumps({'schema_version': 2,
        'source_limits_sim_s': {'scan': .2, 'odom': .2, 'plan': 2.},
        'receive_limits_wall_s': {'scan': .4, 'odom': .4, 'plan': 3.}}))
    sample = {'candidate': True, 'reasons': [], 'task_active': True, 'follow_path_active': True,
        'input_ages': {k: {'source_sim_s': .1, 'receive_wall_s': .1} for k in ('scan', 'odom', 'plan')},
        'features': features(), 'action': {'velocity': [.2, 0.]}}
    (directory / 'expert_samples.jsonl').write_text((json.dumps(sample) + '\n') * 20)


class PolicyTests(unittest.TestCase):
    def test_features_cannot_leak_absolute_pose_or_teacher(self):
        clean = features()
        dirty = {**clean, 'map_pose_at_odom_stamp': [99., 99., 99.], 'teacher': [4., 4.]}
        np.testing.assert_array_equal(vectorize(clean), vectorize(dirty))
        self.assertEqual(vectorize(clean).shape, (80,))

    def test_unknown_sector_and_nan_velocity_rejected(self):
        for change in ('sector', 'velocity'):
            value = features()
            if change == 'sector':
                value['scan_laser_frame']['sector_min_m'][0] = None
            else:
                value['odom_velocity'][0] = float('nan')
            with self.assertRaises(ValueError):
                vectorize(value)

    def test_inference_bounds_and_artifact_roundtrip(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'policy.json'
            path.write_text(json.dumps(artifact()))
            raw, bounded = Policy.load(path).predict(vectorize(features()))
        np.testing.assert_allclose(raw, [.44, -1.2])
        np.testing.assert_allclose(bounded, [.22, -.6])

    def test_bad_schema_scale_normalization_and_network_rejected(self):
        mutations = [('feature_schema', 'old'), ('action_scale', [1., 1.]),
                     ('std', [0.] * 80), ('layers', [])]
        for key, value in mutations:
            broken = artifact()
            broken[key] = value
            with self.assertRaises(ValueError):
                Policy(broken)

    def test_bounded_output_has_forward_velocity_without_a_posthoc_floor(self):
        value = artifact()
        value.update(schema_version=2, output_transform='sigmoid_tanh')
        value['layers'][0]['bias'] = [-2., 2.]
        raw, bounded = Policy(value).predict(vectorize(features()))
        self.assertAlmostEqual(raw[0], .22 / (1. + np.exp(2.)))
        self.assertAlmostEqual(raw[1], .6 * np.tanh(2.))
        np.testing.assert_array_equal(raw, bounded)

    def test_bounded_output_stays_finite_at_extreme_logits(self):
        for bias in ([1000., -1000.], [-1000., 1000.]):
            value = artifact()
            value.update(schema_version=2, output_transform='sigmoid_tanh')
            value['layers'][0]['bias'] = bias
            raw, bounded = Policy(value).predict(vectorize(features()))
            self.assertTrue(np.isfinite(raw).all())
            self.assertTrue(0 <= raw[0] <= .22 and -.6 <= raw[1] <= .6)
            np.testing.assert_array_equal(raw, bounded)

    def test_output_contract_cannot_be_silently_ignored(self):
        for version, head in ((1, 'sigmoid_tanh'), (2, 'unknown'), (2, None)):
            value = artifact()
            value.update(schema_version=version, output_transform=head)
            with self.assertRaises(ValueError):
                Policy(value)

    def test_schema_two_linear_matches_original_policy(self):
        old = artifact()
        new = {**old, 'schema_version': 2, 'output_transform': 'linear'}
        for original, explicit in zip(Policy(old).predict(vectorize(features())),
                                      Policy(new).predict(vectorize(features()))):
            np.testing.assert_array_equal(original, explicit)

    def test_guard_stops_invalid_observation_and_close_return(self):
        sample = {'candidate': False, 'reasons': ['scan_future_or_stale'], 'features': features()}
        self.assertEqual(guard_reason(sample), 'scan_future_or_stale')
        sample.update(candidate=True, reasons=[])
        self.assertIsNone(guard_reason(sample))
        sample['features']['scan_laser_frame']['sector_min_m'][3] = .19
        self.assertEqual(guard_reason(sample), 'laser_proximity_below_0.20m')


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.manifest = self.root / 'manifest.json'
        self.entries = []
        for index, split in enumerate(('train', 'validation', 'test')):
            episode(self.root / split, -.5 - .1 * index)
            self.entries.append({'path': split, 'split': split})
        self.save()

    def tearDown(self):
        self.temp.cleanup()

    def save(self):
        self.manifest.write_text(json.dumps({'schema_version': 1, 'episodes': self.entries}))

    def test_loads_separate_whole_tasks_and_records_checksums(self):
        data, audit = load_dataset(self.manifest)
        self.assertEqual([len(x) for x, _ in data.values()], [20, 20, 20])
        self.assertEqual(len({r['task_sha256'] for r in audit}), 3)
        self.assertEqual(len(audit[0]['sha256']['expert_samples.jsonl']), 64)

    def test_repeated_task_cannot_cross_partitions(self):
        for filename in ('task_config_used.json', 'result.json', 'session.json'):
            (self.root / 'test' / filename).write_bytes((self.root / 'train' / filename).read_bytes())
        with self.assertRaisesRegex(ValueError, 'leaks'):
            load_dataset(self.manifest)

    def test_duplicate_episode_rejected(self):
        self.entries.append(copy.copy(self.entries[0]))
        self.save()
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            load_dataset(self.manifest)

    def test_recovery_is_excluded_and_stale_candidate_is_error(self):
        path = self.root / 'test/result.json'
        value = json.loads(path.read_text())
        value['recoveries_reported_by_nav2'] = 1
        path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'partition'):
            load_dataset(self.manifest)
        value['recoveries_reported_by_nav2'] = 0
        path.write_text(json.dumps(value))
        path = self.root / 'test/expert_samples.jsonl'
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        rows[0]['input_ages']['scan']['source_sim_s'] = -.01
        path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        with self.assertRaisesRegex(ValueError, 'future/stale'):
            load_dataset(self.manifest)


if __name__ == '__main__':
    unittest.main()
