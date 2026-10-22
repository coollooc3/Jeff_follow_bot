"""Operator command validation and cancellation completion contracts."""
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import nav_task
import run_manual


class OperatorTests(unittest.TestCase):
    def test_nearby_goal_is_allowed_without_trial_minimum_distance(self):
        self.assertEqual(nav_task.goal_values([-2, -0.49, 0]), [-2, -0.49, 0])

    def test_invalid_goal_rejected(self):
        for values in ([0, 1], [0, 0, math.nan], [math.inf, 0, 0]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                nav_task.goal_values(values)

    def test_yaw_wraps_in_radians(self):
        self.assertAlmostEqual(nav_task.goal_values([0, 0, 3 * math.pi])[2], math.pi)

    def test_terminal_history_survives_late_execution_and_empty_native_list(self):
        from action_msgs.msg import GoalStatus, GoalStatusArray
        client = nav_task.TaskClient.__new__(nav_task.TaskClient)
        client.history, client.states = {}, {}
        def update(status):
            item = GoalStatus()
            item.goal_info.goal_id.uuid = list(range(16))
            item.status = status
            msg = GoalStatusArray()
            msg.status_list = [item]
            client.on_status(msg)
        update(GoalStatus.STATUS_EXECUTING)
        update(GoalStatus.STATUS_SUCCEEDED)
        update(GoalStatus.STATUS_EXECUTING)
        client.on_status(GoalStatusArray())
        self.assertEqual(client.states[bytes(range(16)).hex()], 'SUCCEEDED')
        self.assertEqual(client.history[bytes(range(16)).hex()].status, GoalStatus.STATUS_SUCCEEDED)

    def test_goal_acceptance_does_not_claim_arrival_and_sets_map_pose(self):
        from builtin_interfaces.msg import Time
        client = nav_task.TaskClient.__new__(nav_task.TaskClient)
        client.node = MagicMock()
        client.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        client.action = MagicMock()
        handle = MagicMock(accepted=True)
        handle.goal_id.uuid = list(range(16))
        client.wait = MagicMock(return_value=handle)
        report = client.send([-2, 1, math.pi / 2])
        self.assertTrue(report['accepted'])
        self.assertNotIn('result', report)
        goal = client.action.send_goal_async.call_args.args[0]
        self.assertEqual(goal.pose.header.frame_id, 'map')
        self.assertEqual(goal.pose.pose.position.x, -2)
        self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sqrt(0.5))
        handle.accepted = False
        self.assertFalse(client.send([0, 0, 0])['accepted'])

    def test_both_entries_reject_real_robot_domain_before_ros_initialization(self):
        for entry, argv in ((nav_task, ['nav_task', 'cancel']), (run_manual, ['run_manual'])):
            with patch.dict(os.environ, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}), \
                    patch.object(sys, 'argv', argv), self.assertRaises(SystemExit) as exc:
                entry.main()
            self.assertEqual(exc.exception.code, 2)

    def test_stop_requires_fresh_post_request_zero_command_and_low_odom(self):
        self.assertTrue(nav_task.stopped(10, 9.7, (9.9, 0, 0), (9.95, 0, 0)))
        cases = [((9.6, 0, 0), (9.95, 0, 0)),  # before cancellation
                 ((9.0, 0, 0), (9.0, 0, 0)),   # stale
                 ((9.9, 0.05, 0), (9.95, 0, 0)),
                 ((9.9, 0, 0.1), (9.95, 0, 0)),
                 ((9.9, 0, 0), (9.95, 0.01, 0)),
                 ((9.9, 0, 0), (9.95, 0, 0.2)),
                 ((9.9, 0, 0), None)]
        for odom, command in cases:
            with self.subTest(odom=odom, command=command):
                self.assertFalse(nav_task.stopped(10, 9.7, odom, command))

    def cancel_client(self, terminal='CANCELED', motion=0, identities=True):
        client = nav_task.TaskClient.__new__(nav_task.TaskClient)
        client.cancel_service = MagicMock()
        identity = MagicMock()
        identity.goal_id.uuid = list(range(16))
        response = MagicMock(return_code=0, goals_canceling=[identity] if identities else [])
        client.wait = MagicMock(return_value=response)
        client.states = {bytes(range(16)).hex(): terminal}
        clock = [10.0]

        def spin(seconds):
            clock[0] += seconds
            client.odom = (clock[0], motion, 0)
            client.command = (clock[0], 0, 0)

        client.spin = spin
        client.odom = client.command = None
        return client, clock

    def test_cancel_waits_for_terminal_state_and_stopping(self):
        client, clock = self.cancel_client()
        with patch.object(nav_task.time, 'monotonic', side_effect=lambda: clock[0]):
            report = client.cancel()
        self.assertEqual(report['result'], 'CANCELED')
        self.assertTrue(report['stop_verified'])

    def test_cancel_acknowledgement_alone_cannot_pass(self):
        for terminal, motion in [('CANCELING', 0), ('CANCELED', 0.1), ('ABORTED', 0)]:
            client, clock = self.cancel_client(terminal, motion)
            with patch.object(nav_task.time, 'monotonic', side_effect=lambda: clock[0]), \
                    self.assertRaises(TimeoutError):
                client.cancel()

    def test_cancel_no_active_goal_does_not_claim_stop_verified(self):
        client, clock = self.cancel_client(identities=False)
        with patch.object(nav_task.time, 'monotonic', side_effect=lambda: clock[0]):
            report = client.cancel()
        self.assertEqual(report['result'], 'nothing_to_cancel')
        self.assertFalse(report['stop_verified'])

    def test_service_rejection_and_unavailability_are_errors(self):
        client, clock = self.cancel_client()
        client.wait.return_value.return_code = 1
        with self.assertRaises(RuntimeError):
            client.cancel()
        client.cancel_service.wait_for_service.return_value = False
        with self.assertRaises(TimeoutError):
            client.cancel()


if __name__ == '__main__':
    unittest.main()
