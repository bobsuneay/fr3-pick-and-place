"""The high-rate point-cloud queue must not block ROS callbacks or grow."""
from pathlib import Path
import sys
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_grasp'))
from fr3_dual_arm_grasp.latest_message import LatestMessageWorker


class LatestMessageWorkerTests(unittest.TestCase):
    def test_submit_is_non_blocking_and_only_latest_pending_message_survives(self):
        started = threading.Event()
        release = threading.Event()
        processed = []

        def process(value):
            processed.append(value)
            if value == 1:
                started.set()
                release.wait(2.0)

        worker = LatestMessageWorker(process)
        try:
            worker.submit(1)
            self.assertTrue(started.wait(1.0))
            begin = time.monotonic()
            self.assertTrue(worker.submit(2))
            self.assertTrue(worker.submit(3))
            self.assertLess(time.monotonic() - begin, 0.1)
            release.set()
            deadline = time.monotonic() + 1.0
            while processed != [1, 3] and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(processed, [1, 3])
        finally:
            release.set()
            self.assertTrue(worker.close())

    def test_worker_survives_processing_error_and_rejects_after_close(self):
        errors = []
        processed = threading.Event()

        def process(value):
            if value == 'bad':
                raise ValueError('bad cloud')
            processed.set()

        worker = LatestMessageWorker(process, on_error=errors.append)
        worker.submit('bad')
        deadline = time.monotonic() + 1.0
        while not errors and time.monotonic() < deadline:
            time.sleep(0.01)
        worker.submit('good')
        self.assertTrue(processed.wait(1.0))
        self.assertIsInstance(errors[0], ValueError)
        self.assertTrue(worker.close())
        self.assertFalse(worker.submit('late'))


if __name__ == '__main__':
    unittest.main()
