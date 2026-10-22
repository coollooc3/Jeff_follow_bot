"""TF failures remain rejected and identify the failed chain without substituting latest TF."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformException
    from observation_buffer import ObservationBuffer, query_pose
except ImportError:
    ObservationBuffer = None

from test_expert_data import inputs


@unittest.skipIf(ObservationBuffer is None, 'ROS2 Python runtime required')
class ObservationBufferTests(unittest.TestCase):
    def transform(self, seconds=9.95):
        transform = TransformStamped()
        transform.header.stamp.sec = int(seconds)
        transform.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
        transform.transform.rotation.w = 1.
        return transform

    def observations(self, buffer):
        observations = ObservationBuffer(None, buffer, [2., 0., 0.])
        for kind, record in zip(('scan', 'odom', 'plan'), inputs()[:3]):
            observations.history[kind].append(record)
            observations.latest[kind] = record
        return observations

    def test_success_preserves_exact_time_pose_and_candidate(self):
        buffer = Mock()
        buffer.lookup_transform.return_value = self.transform()
        sample = self.observations(buffer).sample(inputs()[3], True, True)
        self.assertTrue(sample['candidate'], sample['reasons'])
        self.assertEqual(sample['features']['map_pose_at_odom_stamp'], [0., 0., 0.])
        self.assertEqual(sample['transform_diagnostics']['map_to_base']['query_sim_s'], 9.95)
        self.assertEqual(buffer.lookup_transform.call_args_list[0].args[2].nanoseconds, 9950000000)
        json.dumps(sample, allow_nan=False)

    def test_map_failure_records_exception_and_latest_but_still_rejects(self):
        buffer = Mock()
        buffer.lookup_transform.side_effect = [TransformException('future extrapolation'), self.transform(9.94)]
        sample = self.observations(buffer).sample(inputs()[3], True, True)
        self.assertFalse(sample['candidate'])
        self.assertIn('transform_missing', sample['reasons'])
        self.assertNotIn('map_pose_at_odom_stamp', sample['features'])
        diagnostic = sample['transform_diagnostics']['map_to_base']
        self.assertEqual(diagnostic['error'], 'future extrapolation')
        self.assertEqual(diagnostic['exception_type'], 'TransformException')
        self.assertAlmostEqual(diagnostic['latest_available_stamp_s'], 9.94)
        self.assertNotIn('base_to_laser', sample['transform_diagnostics'])

    def test_laser_failure_is_distinct_from_map_failure(self):
        buffer = Mock()
        buffer.lookup_transform.side_effect = [self.transform(), TransformException('laser frame missing'),
                                               TransformException('still missing')]
        sample = self.observations(buffer).sample(inputs()[3], True, True)
        self.assertFalse(sample['candidate'])
        self.assertEqual(sample['transform_diagnostics']['map_to_base']['status'], 'available')
        diagnostic = sample['transform_diagnostics']['base_to_laser']
        self.assertEqual((diagnostic['target'], diagnostic['source']), ('base_footprint', 'base_scan'))
        self.assertEqual(diagnostic['latest_error'], 'still missing')

    def test_invalid_query_is_reported_without_enabling_latest_fallback(self):
        buffer = Mock()
        buffer.lookup_transform.side_effect = [ValueError('invalid timestamp'), self.transform()]
        pose, diagnostic = query_pose(buffer, 'map', 'base_footprint', 9.95)
        self.assertIsNone(pose)
        self.assertEqual(diagnostic['exception_type'], 'ValueError')

    def test_missing_inputs_do_not_query_tf(self):
        buffer = Mock()
        observations = self.observations(buffer)
        observations.history['odom'].clear()
        sample = observations.sample(inputs()[3], True, True)
        self.assertFalse(sample['candidate'])
        buffer.lookup_transform.assert_not_called()
        self.assertEqual(sample['transform_diagnostics'], {})

    def delayed_observations(self):
        buffer = Mock()
        observations = self.observations(buffer)
        newer = {**inputs()[1], 'id': 2, 'stamp_s': 9.99}
        observations.history['odom'].append(newer)
        observations.latest['odom'] = newer

        def lookup(target, source, query):
            if target == 'map' and query.nanoseconds == 9990000000:
                raise TransformException('newest exact TF not received yet')
            transform = self.transform(9.95)
            if target == 'map':
                transform.transform.translation.x = .1
            return transform

        buffer.lookup_transform.side_effect = lookup
        return observations, buffer

    def test_temporarily_unavailable_latest_uses_fresh_exact_time_history(self):
        observations, _ = self.delayed_observations()
        sample = observations.sample(inputs()[3], True, True)
        self.assertTrue(sample['candidate'], sample['reasons'])
        self.assertEqual(sample['input_ids']['odom'], 1)
        self.assertEqual(sample['latest_received_input_ids']['odom'], 2)
        self.assertEqual(sample['pose_query_sim_s'], 9.95)
        self.assertEqual(sample['features']['map_pose_at_odom_stamp'], [.1, 0., 0.])
        self.assertAlmostEqual(sample['features']['goal_xy_base_m'][0], 1.9)
        self.assertTrue(sample['odom_selection']['used_history'])

    def test_history_cannot_bypass_source_or_receipt_freshness_limits(self):
        for field, stale in [('stamp_s', 9.79), ('received_wall_s', 19.59)]:
            observations, buffer = self.delayed_observations()
            observations.history['odom'][0][field] = stale
            sample = observations.sample(inputs()[3], True, True)
            self.assertFalse(sample['candidate'])
            self.assertIn('transform_missing', sample['reasons'])
            self.assertEqual(sample['input_ids']['odom'], 2)
            self.assertEqual(buffer.lookup_transform.call_count, 2)  # exact plus diagnostic latest

    def test_invalid_latest_odometry_is_rejected_instead_of_hidden_by_history(self):
        observations, buffer = self.delayed_observations()
        observations.history['odom'][-1]['frame'] = 'wrong_frame'
        sample = observations.sample(inputs()[3], True, True)
        self.assertFalse(sample['candidate'])
        self.assertIn('invalid_odom_frame_or_velocity', sample['reasons'])
        self.assertEqual(sample['input_ids']['odom'], 2)
        self.assertEqual(buffer.lookup_transform.call_count, 2)

    def test_history_alignment_never_uses_future_source_or_future_receipt(self):
        observations, _ = self.delayed_observations()
        for record in ({**inputs()[1], 'id': 3, 'stamp_s': 10.01},
                       {**inputs()[1], 'id': 4, 'stamp_s': 9.98, 'received_wall_s': 20.01}):
            observations.history['odom'].append(record)
        sample = observations.sample(inputs()[3], True, True)
        self.assertTrue(sample['candidate'], sample['reasons'])
        self.assertEqual(sample['input_ids']['odom'], 1)

    def test_unavailable_entire_history_remains_a_guard_stop(self):
        observations, buffer = self.delayed_observations()
        buffer.lookup_transform.side_effect = TransformException('no exact TF')
        sample = observations.sample(inputs()[3], True, True)
        self.assertFalse(sample['candidate'])
        self.assertIn('transform_missing', sample['reasons'])
        self.assertEqual(sample['input_ids']['odom'], 2)

    def test_native_tf_buffer_reproduces_odom_arriving_before_its_transform(self):
        buffer = Buffer()
        for seconds in (9., 10.):
            transform = self.transform(seconds)
            transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
            buffer.set_transform(transform, 'test')
        transform = self.transform(9.95)
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
        transform.transform.translation.x = .1
        buffer.set_transform(transform, 'test')
        transform = self.transform(0.)
        transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'base_scan'
        buffer.set_transform_static(transform, 'test')
        observations = self.observations(buffer)
        observations.history['odom'].append({**inputs()[1], 'id': 2, 'stamp_s': 9.99})
        sample = observations.sample(inputs()[3], True, True)
        self.assertTrue(sample['candidate'], sample['reasons'])
        self.assertEqual(sample['pose_query_sim_s'], 9.95)
        self.assertEqual(sample['features']['map_pose_at_odom_stamp'], [.1, 0., 0.])
        self.assertEqual(sample['odom_selection']['failed_queries'][0]['exception_type'], 'ExtrapolationException')


if __name__ == '__main__':
    unittest.main()
