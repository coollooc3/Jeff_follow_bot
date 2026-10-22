"""Test the supplied transport framework, not an answer to the student exercise."""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from leap_patrol.client import PatrolClient
from leap_patrol.exercise_check import check_exercise
from leap_patrol.mission import Mission


class ScriptedMission(Mission):
    """Fixture with explicit expected outcomes; no scheduling algorithm is supplied."""
    def __init__(self, outcomes=()):
        super().__init__(('A', 'B', 'C'))
        self.outcomes = list(outcomes)

    def on_result(self, status):
        expected, state, index, completed = self.outcomes.pop(0)
        if status != expected:
            raise AssertionError(f'Expected {expected}, received {status}')
        self.state, self.index, self.completed = state, index, completed
        self.reason = f'{self.current_name}: {status}' if state == 'FAILED' else ''


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancel_reply = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_reply


class FakeAction:
    def __init__(self):
        self.indices, self.responses = [], []

    def send(self, index):
        response = Future()
        self.indices.append(index)
        self.responses.append(response)
        return response

    def accept(self, position=-1, accepted=True):
        handle = FakeHandle(accepted)
        self.responses[position].set_result(handle)
        return handle


class FrameworkTests(unittest.TestCase):
    def make_client(self, outcomes=()):
        mission, action = ScriptedMission(outcomes), FakeAction()
        return PatrolClient(mission, action.send), action

    def test_start_is_idle_until_requested_and_rejects_duplicate_start(self):
        client, action = self.make_client()
        self.assertEqual(client.mission.state, 'IDLE')
        self.assertEqual(action.indices, [])
        client.start()
        self.assertEqual(action.indices, [0])
        with self.assertRaises(ValueError):
            client.start()

    def test_only_final_success_sends_next_and_complete_sends_no_fourth(self):
        outcomes = [('SUCCEEDED', 'RUNNING', 1, 1), ('SUCCEEDED', 'RUNNING', 2, 2),
                    ('SUCCEEDED', 'COMPLETED', 2, 3)]
        client, action = self.make_client(outcomes)
        client.start()
        for index in range(3):
            handle = action.accept()
            self.assertEqual(action.indices, list(range(index + 1)))
            handle.result.set_result(SimpleNamespace(status=4))
        self.assertEqual((client.mission.state, client.mission.completed), ('COMPLETED', 3))
        self.assertEqual(action.indices, [0, 1, 2])
        self.assertIsNone(client.active)

    def test_rejected_b_stops_without_sending_c(self):
        client, action = self.make_client([('SUCCEEDED', 'RUNNING', 1, 1), ('REJECTED', 'FAILED', 1, 1)])
        client.start()
        action.accept().result.set_result(SimpleNamespace(status=4))
        action.accept(accepted=False)
        self.assertEqual((client.mission.state, client.mission.current_name), ('FAILED', 'B'))
        self.assertEqual(action.indices, [0, 1])

    def test_aborted_a_stops_without_sending_b(self):
        client, action = self.make_client([('ABORTED', 'FAILED', 0, 0)])
        client.start()
        action.accept().result.set_result(SimpleNamespace(status=6))
        self.assertEqual(action.indices, [0])
        self.assertEqual(client.mission.state, 'FAILED')
        self.assertIsNone(client.active)

    def test_cancel_while_waiting_for_acceptance_cancels_late_handle(self):
        client, action = self.make_client()
        client.start()
        self.assertTrue(client.cancel())
        self.assertEqual(client.mission.state, 'CANCELING')
        with self.assertRaises(ValueError):
            client.start()
        handle = action.accept()
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_reply.set_result(SimpleNamespace(return_code=0))
        self.assertEqual(client.mission.state, 'CANCELING')
        handle.result.set_result(SimpleNamespace(status=5))
        self.assertEqual((client.mission.state, action.indices), ('CANCELED', [0]))

    def test_cancel_wins_over_queued_success_and_old_cancel_reply_is_ignored(self):
        client, action = self.make_client()
        client.start()
        handle = action.accept()
        client.cancel()
        client.cancel()
        self.assertEqual(handle.cancel_calls, 1)
        handle.result.set_result(SimpleNamespace(status=4))
        self.assertEqual(client.mission.state, 'CANCELED')
        self.assertEqual(client.last_nav_status, 'SUCCEEDED')
        client.start()
        handle.cancel_reply.set_result(SimpleNamespace(return_code=1))
        self.assertEqual((client.mission.state, client.mission.index, client.mission.completed), ('RUNNING', 0, 0))
        self.assertEqual(action.indices, [0, 0])

    def test_rejection_after_cancel_finishes_without_progress(self):
        client, action = self.make_client()
        client.start()
        client.cancel()
        action.accept(accepted=False)
        self.assertEqual(client.mission.state, 'CANCELED')
        self.assertEqual(action.indices, [0])
        self.assertIsNone(client.active)

    def test_cancel_rejection_keeps_unknown_goal_owned_and_never_advances(self):
        client, action = self.make_client()
        client.start()
        handle = action.accept()
        client.cancel()
        handle.cancel_reply.set_result(SimpleNamespace(return_code=1))
        self.assertEqual(client.mission.state, 'FAILED')
        self.assertIsNotNone(client.active)
        with self.assertRaises(ValueError):
            client.start()
        handle.result.set_result(SimpleNamespace(status=4))
        self.assertEqual(client.mission.state, 'FAILED')
        self.assertEqual(action.indices, [0])
        self.assertIsNone(client.active)

    def test_response_error_does_not_allow_another_goal(self):
        client, action = self.make_client()
        client.start()
        action.responses[0].set_exception(RuntimeError('connection lost'))
        self.assertEqual(client.mission.state, 'FAILED')
        with self.assertRaises(ValueError):
            client.start()
        self.assertEqual(action.indices, [0])

    def test_result_error_preserves_uncertainty(self):
        client, action = self.make_client()
        client.start()
        action.accept().result.set_exception(RuntimeError('result lost'))
        self.assertEqual(client.mission.state, 'FAILED')
        self.assertIsNotNone(client.active)
        self.assertEqual(action.indices, [0])

    def test_old_result_cannot_finish_the_new_goal(self):
        client, action = self.make_client([('SUCCEEDED', 'RUNNING', 1, 1)])
        client.start()
        old_token = client.active
        action.accept().result.set_result(SimpleNamespace(status=4))
        duplicate = Future()
        duplicate.set_result(SimpleNamespace(status=6))
        client._result(duplicate, old_token)
        self.assertEqual((client.mission.state, client.mission.index, action.indices), ('RUNNING', 1, [0, 1]))

    def test_cancel_when_idle_does_not_claim_a_stop(self):
        client, action = self.make_client()
        self.assertFalse(client.cancel())
        self.assertEqual((client.mission.state, action.indices), ('IDLE', []))

    def test_student_exception_is_reported_without_dispatching_another_goal(self):
        client, action = self.make_client()
        client.start()
        action.accept().result.set_result(SimpleNamespace(status=4))  # Fixture has no scheduled outcome.
        self.assertEqual(client.mission.state, 'FAILED')
        self.assertIn('scheduler_error', client.mission.reason)
        self.assertEqual(action.indices, [0])

    def test_exercise_check_reports_a_missing_implementation(self):
        with patch.object(Mission, 'on_result', side_effect=NotImplementedError('exercise')):
            errors = check_exercise()
        self.assertEqual(len(errors), 5)
        self.assertTrue(all('NotImplementedError' in error for error in errors))


if __name__ == '__main__':
    unittest.main()
