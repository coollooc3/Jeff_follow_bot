import math
import unittest

from leap_marker_follow.controller import Follower, Settings


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.f = Follower()
        self.f.set_reference(.20)

    def track(self, start=0., side=.10, offset=0., frames=8):
        command = None
        for i in range(frames):
            now = start+i*.05
            self.f.observe(now, side, offset)
            command = self.f.step(now)
        return command

    def test_starts_disabled_and_manual_off_stays_off(self):
        self.assertEqual(self.track().linear, 0.)
        self.f.set_enabled(True)
        self.assertGreater(self.track(.4).linear, 0.)
        self.f.set_enabled(False)
        self.assertEqual(self.track(10.).linear, 0.)
        self.assertFalse(self.f.enabled)

    def test_no_trial_time_or_distance_budget(self):
        self.f.set_enabled(True)
        for i in range(1601):  # Eighty seconds of fresh observations, > one metre commanded.
            self.f.observe(i*.05, .10, 0.)
            command = self.f.step(i*.05)
        self.assertEqual(command.reason, 'following')
        self.assertAlmostEqual(command.linear, .12)

    def test_curved_motion_has_both_components(self):
        self.f.set_enabled(True)
        command = self.track(offset=.2, frames=20)
        self.assertGreater(command.linear, .05)
        self.assertLess(command.angular, 0.)

    def test_left_turn_and_large_offset_stops_forward(self):
        self.f.set_enabled(True)
        command = self.track(offset=-.8, frames=20)
        self.assertEqual(command.linear, 0.)
        self.assertGreater(command.angular, 0.)

    def test_raw_size_stops_without_waiting_for_filter(self):
        self.f.set_enabled(True)
        self.track()
        self.f.observe(.4, .20, 0.)
        self.assertEqual(self.f.step(.4).linear, 0.)
        self.assertEqual(self.track(.45, side=.195).linear, 0.)

    def test_near_speed_has_starting_floor_and_speed_cap(self):
        self.f.set_enabled(True)
        self.assertAlmostEqual(self.track(side=.183, frames=20).linear, .05)
        f = Follower(Settings(max_forward_mps=.03))
        f.set_reference(.20)
        f.set_enabled(True)
        for i in range(20):
            f.observe(i*.05, .10, 0.)
            self.assertLessEqual(f.step(i*.05).linear, .03)

    def test_missing_frame_brakes_without_reconfirming(self):
        self.f.set_enabled(True)
        before = self.track(frames=20)
        self.f.observe(1.)
        missing = self.f.step(1.)
        self.assertLess(missing.linear, before.linear)
        self.assertEqual(missing.angular, 0.)
        self.f.observe(1.05, .10, 0.)
        self.assertEqual(self.f.count, 3)
        self.assertEqual(self.f.step(1.05).reason, 'following')

    def test_gap_stops_then_automatically_reacquires(self):
        self.f.set_enabled(True)
        self.track()
        self.assertEqual(self.f.step(.65).linear, 0.)
        self.assertTrue(self.f.enabled)
        self.f.observe(2., .10, 0.)
        self.assertEqual(self.f.step(2.).reason, 'acquiring')
        self.assertGreater(self.track(2.05).linear, 0.)

    def test_sensor_or_obstacle_block_zeroes_both_axes(self):
        self.f.set_enabled(True)
        self.track(offset=.2)
        command = self.f.step(.4, 'scan_timeout')
        self.assertEqual((command.linear, command.angular), (0., 0.))
        self.assertTrue(self.f.enabled)
        self.assertGreater(self.track(.45, offset=.2).linear, 0.)

    def test_clock_reversal_disables(self):
        self.f.set_enabled(True)
        self.track()
        self.assertEqual(self.f.step(.1).reason, 'control_clock_invalid')
        self.assertFalse(self.f.enabled)

    def test_ideal_camera_feedback_approaches_reference(self):
        # A toy planar camera/kinematics check; NOT Gazebo or real-world evidence.
        self.f.set_enabled(True)
        x = y = yaw = 0.
        for i in range(500):
            dx, dy = .8-x, .1-y
            depth = dx*math.cos(yaw)+dy*math.sin(yaw)
            lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
            self.f.observe(i*.05, .20*.25/depth, -lateral/depth/math.tan(.6))
            command = self.f.step(i*.05)
            x += command.linear*math.cos(yaw)*.05
            y += command.linear*math.sin(yaw)*.05
            yaw += command.angular*.05
        self.assertTrue(.24 <= depth <= .28, depth)
        self.assertLess(abs(self.f.offset), .06)
        self.assertEqual(command.linear, 0.)


if __name__ == '__main__':
    unittest.main()
