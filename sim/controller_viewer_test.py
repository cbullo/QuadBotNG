"""Headless checks for renderer lifetime and real process/signal cleanup."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
import unittest
import tempfile
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import patch

from sim import controller_viewer as app


class AsyncViewer:
    """Like MuJoCo's handle: close requests exit before native teardown ends."""

    def __init__(self):
        self.closed = threading.Event()
        self.destroyed = threading.Event()
        self.thread = threading.Thread(target=self.render, daemon=True)
        self.thread.start()

    def render(self):
        self.closed.wait()
        time.sleep(0.05)
        self.destroyed.set()

    def close(self):
        self.closed.set()

    def is_running(self):
        return not self.closed.is_set()


def interrupt_worker():
    # Simulate inherited SIGINT-ignore, which must not prevent explicit cleanup.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    created = []

    def launch(*_):
        viewer = AsyncViewer()
        created.append(viewer)
        return viewer

    with app.ShutdownSignals() as shutdown:
        with patch.object(app.mujoco.viewer, "launch_passive", side_effect=launch):
            with app.joined_passive_viewer(None, None):
                # This child deliberately ignores TERM to exercise the fallback.
                child = subprocess.Popen(
                    [sys.executable, "-u", "-c",
                     "import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                     "print('ready',flush=True); time.sleep(60)"],
                    start_new_session=True, stdout=subprocess.PIPE, text=True)
                try:
                    if child.stdout.readline().strip() != "ready":
                        raise RuntimeError("child did not start")
                    print(f"ready {child.pid}", flush=True)
                    while not shutdown.signum:
                        time.sleep(0.01)
                finally:
                    app.stop_child(child, timeout=0.1)
                    child.stdout.close()
        assert created[0].destroyed.is_set()
        assert not created[0].thread.is_alive()
        assert child.returncode == -signal.SIGKILL
        print("renderer joined; child reaped", flush=True)
        return 128 + shutdown.signum


class ViewerLifecycleTest(unittest.TestCase):
    def test_attached_view_reads_existing_trace_without_a_child(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "state.csv"
            trace.write_text("time,q0,q1\n1.25,0.1,-0.2\n")
            viewer = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)), lock=nullcontext,
                                     sync=lambda: None)
            with patch.object(app.mujoco, "mj_forward") as forward:
                from unittest.mock import Mock
                viewer.is_running = Mock(side_effect=[True, False])
                data = SimpleNamespace(qpos=np.zeros(2), time=0)
                app.display(viewer, SimpleNamespace(nq=2), data, trace, None,
                            SimpleNamespace(suspended=False, show_collisions=False), SimpleNamespace(signum=0))
                np.testing.assert_array_equal(data.qpos, [0.1, -0.2])
                self.assertEqual(data.time, 1.25)
                forward.assert_called_once()

    def test_waits_for_renderer_on_completion_window_close_and_error(self):
        for reason in ("completion", "window", "error"):
            with self.subTest(reason=reason):
                created = []

                def launch(*_):
                    viewer = AsyncViewer()
                    created.append(viewer)
                    return viewer

                def session():
                    with patch.object(app.mujoco.viewer, "launch_passive", side_effect=launch):
                        with app.joined_passive_viewer(None, None) as viewer:
                            if reason == "window":
                                viewer.close()
                                self.assertFalse(viewer.is_running())
                            if reason == "error":
                                raise RuntimeError("bad snapshot")

                if reason == "error":
                    with self.assertRaisesRegex(RuntimeError, "bad snapshot"):
                        session()
                else:
                    session()
                self.assertTrue(created[0].destroyed.is_set())
                self.assertFalse(created[0].thread.is_alive())

    def test_signals_stop_and_reap_child_then_join_renderer(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig):
                worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--interrupt-worker"],
                                          start_new_session=True, stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, text=True)
                try:
                    ready, _, _ = select.select([worker.stdout], [], [], 10)
                    self.assertTrue(ready, "worker did not become ready")
                    line = worker.stdout.readline().strip()
                    self.assertTrue(line.startswith("ready "), line)
                    child_pid = int(line.split()[1])
                    # Send to the foreground-equivalent process group, as a
                    # terminal does, then interrupt again during cleanup.
                    os.killpg(worker.pid, sig)
                    time.sleep(0.02)
                    os.killpg(worker.pid, sig)
                    out, err = worker.communicate(timeout=5)
                    self.assertEqual(worker.returncode, 128 + sig, out + err)
                    self.assertIn("renderer joined; child reaped", out)
                    self.assertNotIn("Traceback", err)
                    with self.assertRaises(ProcessLookupError):
                        os.kill(child_pid, 0)
                finally:
                    if worker.poll() is None:
                        worker.kill()
                        worker.communicate(timeout=5)

    def test_restores_existing_signal_handlers(self):
        before = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        with app.ShutdownSignals() as shutdown:
            shutdown.request(signal.SIGINT, None)
            shutdown.request(signal.SIGTERM, None)
            self.assertEqual(shutdown.signum, signal.SIGINT)
        self.assertEqual(before, {sig: signal.getsignal(sig) for sig in before})


if __name__ == "__main__":
    if "--interrupt-worker" in sys.argv:
        raise SystemExit(interrupt_worker())
    unittest.main()
