"""Exercise native Qt telemetry/commands against the real simulated controller."""
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest

import numpy as np
from PySide6 import QtTest, QtWidgets

from dashboard.native_client import ControllerClient, History
from dashboard.native_view import InspectorWindow
from dashboard.telemetry import MOTORS


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class NativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)
        cls.root = Path(os.environ["TEST_SRCDIR"]) / "_main"
        cls.env = dict(os.environ, RUNFILES_DIR=str(cls.root.parent))

    def until(self, predicate, timeout=7):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            QtTest.QTest.qWait(10)
            if predicate():
                return
        self.fail("timed out waiting for native inspector state")

    def test_history_keeps_samples_between_refreshes_and_wraps(self):
        history = History(4)
        for tick in range(7):
            history.append({"time": tick, "motors": {name: {"rotor_rad": tick+i, "voltage_command_v": i}
                                                       for i, name in enumerate(MOTORS)}})
        times, values = history.arrays()
        np.testing.assert_array_equal(times, [3, 4, 5, 6])
        np.testing.assert_array_equal(values[:, 11, 1], [14, 15, 16, 17])
        self.assertTrue(np.isnan(values[:, :, 4]).all())

    def test_native_ui_feedback_commands_and_lease(self):
        port = free_port()
        with tempfile.TemporaryFile(mode="w+") as log:
            child = subprocess.Popen([str(self.root / "src/odroid_controller/controller/controller"),
                                      "--backend=sim", "--scenario=bob", "--duration=18", "--realtime",
                                      f"--dashboard-port={port}", "--dashboard-control"], env=self.env, stdout=log, stderr=log)
            client = ControllerClient(port)
            window = InspectorWindow(client, startup_stand=True, enable_3d=False)
            acknowledgements = []
            client.ack_received.connect(acknowledgements.append)
            try:
                window.show()
                client.start()
                self.until(lambda: client.latest and client.latest["time"] > 4.5)
                self.assertFalse(window.scene.invalidated)
                self.assertIsNotNone(window.scene.reference)
                self.assertGreater(window.samples, window.refreshes)
                self.assertEqual(window.history.count, window.samples)
                self.assertGreater(len(window.plots[0][2][0].xData), 100)
                self.assertFalse(window.hold.isEnabled())  # Script still owns control.
                window.stop.click()
                self.until(lambda: window.hold.isEnabled())
                window.hold.click()
                self.until(lambda: client.owns_hold and window.send_target.isEnabled())
                self.assertTrue(acknowledgements[-1]["accepted"])
                pose = client.latest["legs"]["leg_FR"]
                window.targets["theta"].setValue(pose["theta"] + .01)
                old = len(acknowledgements)
                window.send_target.click()
                self.until(lambda: len(acknowledgements) > old)
                self.assertTrue(acknowledgements[-1]["accepted"], acknowledgements[-1])
                QtTest.QTest.qWait(700)
                self.assertTrue(client.latest["remote_hold"])  # GUI timer renews the lease.
                client.heartbeat.stop()
                self.until(lambda: not client.latest["remote_hold"] and
                           all(v["mode"] == "stopped" for v in client.latest["legs"].values()), timeout=2)
                self.assertIn("lease", client.latest["dashboard_reason"])
                client.socket.abort()
                self.until(lambda: window.scene.invalidated)
                self.assertFalse(client.owns_hold)
                self.until(lambda: not window.send_target.isEnabled())
            finally:
                window.close()
                client.close()
                if child.poll() is None:
                    child.terminate()
                child.wait(timeout=3)

    def test_launcher_duration_and_cleanup(self):
        port = free_port()
        with tempfile.TemporaryDirectory() as temp:
            profile = Path(temp) / "profile.json"
            result = subprocess.run([str(self.root / "dashboard/native"), "--scenario=bob", "--duration=5",
                                     "--no-viewer", "--no-3d", f"--controller-port={port}",
                                     f"--profile-output={profile}"], env=self.env, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            metrics = json.loads(profile.read_text())
            self.assertGreater(metrics["samples"], 150)
            self.assertGreater(metrics["refreshes"], 50)
            self.assertEqual(metrics["samples"], metrics["history_samples"])
            with self.assertRaises(OSError):
                socket.create_connection(("127.0.0.1", port), timeout=.2)


if __name__ == "__main__":
    unittest.main()
