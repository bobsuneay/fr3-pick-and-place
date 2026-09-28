"""Bounded background processing for high-rate sensor messages."""
import threading
import time


class LatestMessageWorker:
    """Process at most one current message without ever building a backlog."""

    def __init__(self, process, min_interval=0.0, on_error=None,
                 name='latest-message-worker'):
        self._process = process
        self._min_interval = max(0.0, float(min_interval))
        self._on_error = on_error
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(self, message):
        """Replace any queued message and return immediately."""
        with self._condition:
            if self._closed:
                return False
            self._pending = message
            self._condition.notify()
        return True

    def _run(self):
        next_allowed = 0.0
        while True:
            with self._condition:
                while not self._closed and self._pending is None:
                    self._condition.wait()
                if self._closed:
                    return
                delay = next_allowed - time.monotonic()
                if delay > 0.0:
                    self._condition.wait(delay)
                    continue
                message, self._pending = self._pending, None
            try:
                self._process(message)
            except Exception as exc:
                if self._on_error is not None:
                    self._on_error(exc)
            next_allowed = time.monotonic() + self._min_interval

    def close(self, timeout=5.0):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(0.0, float(timeout)))
        return not self._thread.is_alive()
