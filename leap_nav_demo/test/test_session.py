"""Linux process ownership and simulation-domain rejection checks."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

source = Path(__file__).parents[1] / 'scripts/run_demo.py'
sys.path.insert(0, str(source.parent))
spec = importlib.util.spec_from_file_location('run_demo', source)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
import session_process as process_helpers


class SessionTests(unittest.TestCase):
    def test_default_robot_domain_is_rejected(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}), \
                patch.object(sys, 'argv', ['run_demo']), \
                self.assertRaises(SystemExit) as exc:
            runner.main()
        self.assertEqual(exc.exception.code, 2)

    def test_task_interrupt_reaches_owned_executable_group(self):
        process = MagicMock(pid=4321, returncode=0)
        process.poll.return_value = None
        with patch.object(process_helpers, 'live_group_members', side_effect=[[4321], [], [], []]), \
                patch.object(process_helpers.os, 'killpg') as signal_group:
            report = runner.stop(process, interrupt_group=True)
        signal_group.assert_called_once_with(4321, process_helpers.signal.SIGINT)
        process.send_signal.assert_not_called()
        self.assertEqual(report['survivors'], [])

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux process groups')
    def test_cleanup_removes_owned_child_and_preserves_unrelated_process(self):
        owned = subprocess.Popen([sys.executable, '-c',
            'import subprocess,time; subprocess.Popen(["sleep","60"]); time.sleep(60)'],
            start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        unrelated = subprocess.Popen(['sleep', '60'], start_new_session=True)
        try:
            deadline = time.monotonic() + 3
            while len(process_helpers.live_group_members(owned.pid)) < 2 and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertEqual(len(process_helpers.live_group_members(owned.pid)), 2)
            result = runner.stop(owned, grace_periods=(0.2, 0.2, 1))
            self.assertEqual(result['survivors'], [])
            self.assertEqual(process_helpers.live_group_members(owned.pid), [])
            self.assertIsNone(unrelated.poll())
        finally:
            runner.stop(owned, grace_periods=(0.1, 0.1, 1))
            runner.stop(unrelated, grace_periods=(0.1, 0.1, 1))


if __name__ == '__main__':
    unittest.main()
