"""Causal sample eligibility, laser unknowns and explicit coordinate transforms."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from expert_data import SECTORS, scan_features, body_vector, build_sample, latest_past


def inputs():
    common = {'id': 1, 'stamp_s': 9.95, 'received_wall_s': 19.95}
    scan = {**common, 'frame': 'base_scan', 'angle_min': 0., 'angle_increment': math.pi / 180,
            'range_min': .1, 'range_max': 3.5, 'time_increment': 0., 'ranges': [2.] * 360}
    odom = {**common, 'frame': 'odom', 'child_frame': 'base_footprint', 'velocity': [.2, 0.]}
    plan = {**common, 'frame': 'map', 'points': [[0., 0.], [1., 0.], [2., 0.]]}
    action = {'sim_s': 10., 'wall_s': 20., 'velocity': [.2, 0.]}
    return [scan, odom, plan, action, [0., 0., 0.], [-.032, 0., 0.], [2., 0., 0.], True, True]


class ExpertDataTests(unittest.TestCase):
    def test_history_selects_past_state_when_latest_header_is_ahead_of_clock(self):
        action = inputs()[3]
        older = {**inputs()[1], 'id': 1}
        ahead = {**older, 'stamp_s': 10.03, 'id': 2}
        selected = latest_past([older, ahead], action)
        self.assertEqual(selected['id'], 1)
        self.assertIsNone(latest_past([ahead], action))

    def test_history_does_not_use_later_receipts_or_assume_receipt_order_is_source_order(self):
        action = inputs()[3]
        newest = {**inputs()[1], 'stamp_s': 9.99, 'id': 1}
        delayed_old = {**newest, 'stamp_s': 9.90, 'id': 2}
        not_received = {**newest, 'stamp_s': 10., 'received_wall_s': 20.01, 'id': 3}
        self.assertEqual(latest_past([newest, delayed_old, not_received], action)['id'], 1)

    def test_history_scan_selection_checks_last_ray_time(self):
        scan, _, _, action, *_ = inputs()
        scan['time_increment'] = .001
        self.assertIsNone(latest_past([scan], action, scan=True))

    def test_fresh_inputs_produce_features_with_past_source_ages(self):
        sample = build_sample(*inputs())
        self.assertTrue(sample['candidate'], sample['reasons'])
        self.assertEqual(sample['features']['goal_xy_base_m'], [2., 0.])
        self.assertAlmostEqual(sample['features']['path_reference_xy_base_m'][0], .35)
        self.assertEqual(len(sample['features']['scan_laser_frame']['sector_min_m']), SECTORS)

    def test_policy_mode_does_not_claim_an_executing_expert(self):
        values = inputs()
        values[-1] = False
        self.assertFalse(build_sample(*values)['candidate'])
        policy_sample = build_sample(*values, require_follow_path=False)
        self.assertTrue(policy_sample['candidate'], policy_sample['reasons'])
        self.assertFalse(policy_sample['follow_path_active'])

    def test_future_source_or_receive_time_never_qualifies(self):
        for name, index in [('scan', 0), ('odom', 1), ('plan', 2)]:
            for field, future in [('stamp_s', 10.01), ('received_wall_s', 20.01)]:
                data = inputs()
                data[index][field] = future
                self.assertIn(name + '_future_or_stale', build_sample(*data)['reasons'])

    def test_stale_observations_are_not_silently_reused(self):
        for index in (0, 1, 2):
            data = inputs()
            data[index]['stamp_s'] = 7.
            self.assertFalse(build_sample(*data)['candidate'])

    def test_receipt_after_action_is_rejected_even_if_source_time_is_old(self):
        data = inputs()
        data[0]['received_wall_s'] = 20.001
        self.assertFalse(build_sample(*data)['candidate'])

    def test_scan_with_future_end_rays_is_rejected(self):
        data = inputs()
        data[0]['time_increment'] = .001
        self.assertIn('scan_future_rays', build_sample(*data)['reasons'])

    def test_positive_infinity_is_censored_but_unknown_rays_are_distinct(self):
        scan = inputs()[0]
        scan['ranges'] = [math.inf] * 360
        compressed = scan_features(scan)
        self.assertEqual(compressed['sector_min_m'], [3.5] * SECTORS)
        self.assertEqual(sum(compressed['positive_infinite_ray_counts']), 360)
        scan['ranges'] = [float('nan')] * 360
        self.assertEqual(scan_features(scan)['sector_min_m'], [None] * SECTORS)

    def test_out_of_range_finite_rays_do_not_become_free_space(self):
        scan = inputs()[0]
        scan['ranges'] = [0.] * 180 + [4.] * 180
        self.assertEqual(sum(scan_features(scan)['invalid_ray_counts']), 360)

    def test_body_reference_accounts_for_rotated_robot(self):
        vector = body_vector([0., 1.], [0., 0., math.pi / 2])
        self.assertAlmostEqual(vector[0], 1.)
        self.assertAlmostEqual(vector[1], 0.)

    def test_invalid_frames_transforms_and_paths_fail_sample(self):
        variants = []
        data = inputs(); data[2]['frame'] = 'odom'; variants.append(data)
        data = inputs(); data[2]['points'] = []; variants.append(data)
        data = inputs(); data[1]['child_frame'] = 'unexpected'; variants.append(data)
        data = inputs(); data[4] = None; variants.append(data)
        data = inputs(); data[5][0] = float('nan'); variants.append(data)
        for data in variants:
            self.assertFalse(build_sample(*data)['candidate'])

    def test_startup_settling_and_unknown_teacher_state_do_not_qualify(self):
        for index in (7, 8):
            data = inputs()
            data[index] = False
            self.assertFalse(build_sample(*data)['candidate'])


if __name__ == '__main__':
    unittest.main()
