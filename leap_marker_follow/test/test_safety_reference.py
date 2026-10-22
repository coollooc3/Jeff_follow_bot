from dataclasses import asdict
import math
from pathlib import Path
import tempfile
import unittest

from leap_marker_follow.reference import Reference, teach
from leap_marker_follow.safety import InputAge, LASER_X, obstacle_blocked, scan_points


class SafetyTests(unittest.TestCase):
    def test_repeated_and_invalid_publications_cannot_refresh_age(self):
        age = InputAge()
        self.assertTrue(age.accept(1., 100, .01, True, .4))
        self.assertFalse(age.accept(1.3, 100, .01, True, .4))
        self.assertEqual(age.reason(1.41, .4), 'timeout')
        self.assertFalse(age.accept(1.5, 200, -.1, True, .4))
        self.assertEqual(age.reason(1.5, .4), 'invalid')
        self.assertTrue(age.accept(1.6, 300, .01, True, .4))
        self.assertEqual(age.reason(1.6, .4), '')

    def scan(self, ranges, frame='laser_frame'):
        return scan_points(ranges, 0., math.radians(359), math.pi/180, .05, 12., frame)

    def test_scan_coverage_is_required_and_infinity_is_clear(self):
        self.assertEqual(self.scan([math.inf]*360), [])
        self.assertIsNone(self.scan([0.]*360))
        self.assertIsNone(self.scan([math.inf]*360, 'wrong_frame'))
        front_invalid = [math.inf]*360
        for i in range(25):
            front_invalid[i] = math.nan
        self.assertIsNone(self.scan(front_invalid))

    def test_leap_transform_and_front_stop_zone(self):
        ranges = [math.inf]*360
        ranges[0] = .25
        points = self.scan(ranges)
        self.assertAlmostEqual(points[0][0], LASER_X+.25)
        self.assertTrue(obstacle_blocked(points, .12, 0., .18, .04))
        self.assertFalse(obstacle_blocked([(.07489+.25, 0.)], .12, 0., .18, .04))

    def test_side_turn_zone_does_not_block_straight_motion(self):
        points = [(0., .17)]
        self.assertFalse(obstacle_blocked(points, .12, 0., .18, .04))
        self.assertTrue(obstacle_blocked(points, .12, .3, .18, .04))


class ReferenceTests(unittest.TestCase):
    def samples(self):
        return [(i*.05, .20, .01, 800, 600) for i in range(10)]

    def test_capture_and_atomic_roundtrip(self):
        reference = teach(self.samples(), .5, 0)
        # Keep temporary writes inside the checkout for restricted host runners.
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            path = Path(directory)/'reference.json'
            reference.save(path)
            self.assertEqual(Reference.load(path), reference)
            reference.save(path)
            self.assertEqual(list(Path(directory).iterdir()), [path])
        self.assertAlmostEqual(reference.side, .20)

    def test_stale_offcenter_unstable_and_mixed_resolution_are_rejected(self):
        with self.assertRaises(ValueError):
            teach(self.samples(), 2., 0)
        for column, value in ((1, .30), (2, .5), (3, 640)):
            samples = self.samples()
            last = list(samples[-1])
            last[column] = value
            samples[-1] = tuple(last)
            with self.assertRaises(ValueError):
                teach(samples, .5, 0)

    def test_invalid_file_fields_are_rejected(self):
        values = asdict(Reference(0, 800, 600, .2))
        for key, value in (('side', math.nan), ('width', 0), ('marker_id', 50)):
            with self.assertRaises(ValueError):
                Reference(**dict(values, **{key: value}))


if __name__ == '__main__':
    unittest.main()
