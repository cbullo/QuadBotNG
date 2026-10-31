"""Walking UI: real worker lifecycle, checkpoint refresh and live state replay."""
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import numpy as np
from PySide6 import QtWidgets
import torch

from sim.walk_training import WalkConfig, load_walk_policy, make_walk_env, train_walk
from sim.walk_training_view import PREVIEW_SECONDS, WalkTrainingWindow
from sim.training_view import TrainingWindow
from types import SimpleNamespace


class WalkWindowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)
        cls.root = Path(os.environ["TEST_SRCDIR"]) / "_main"
        cls.binary = cls.root / "sim/walk_train"
        cls.process_env = dict(os.environ, RUNFILES_DIR=str(cls.root.parent))

    def test_qualification_progress_is_separate_from_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            status = dict(state="evaluating", steps=8192, updates=16,
                          compute=dict(physics_device="gpu", num_envs=128), qualification=dict(
                completed=9, total=28, goal="right", kind="policy", initial_temperature_c=40,
                simulated_seconds=3.2, case_seconds=10, elapsed_seconds=24))
            (directory / "status.json").write_text(json.dumps(status))
            window = WalkTrainingWindow(directory, render=False)
            window.poll_timer.stop()
            window.step_timer.stop()
            try:
                self.assertFalse(window.failed, window.preview_status.text())
                self.assertIn("Policy preview", window.mode.itemText(0))
                self.assertIn("9/28", window.qualification_status.text())
                self.assertIn("3.2/10 simulated s", window.qualification_status.text())
                self.assertIn("24 s elapsed", window.qualification_status.text())
                self.assertIn("GPU physics ×128; CPU PPO", window.status.text())
                status["qualification"] = dict(completed=7, total=28, physics_device="gpu", parallel_cases=28,
                    phase="running", simulated_seconds=2.5, case_seconds=10, elapsed_seconds=8)
                (directory / "status.json").write_text(json.dumps(status))
                window.poll()
                self.assertIn("GPU qualification", window.qualification_status.text())
                self.assertIn("28 cases in parallel", window.qualification_status.text())
                self.assertIn("7/28", window.qualification_status.text())
                status.update(state="optimizing", policy_kind="droq", actor_updates=40,
                              replay_transitions=1040, optimization=dict(completed=17, total=160))
                (directory / "status.json").write_text(json.dumps(status))
                window.poll()
                self.assertIn("DroQ critic updates", window.status.text())
                self.assertIn("CPU DroQ", window.status.text())
                self.assertIn("actor updates: 40", window.status.text())
                self.assertIn("17/160", window.qualification_status.text())
                status["state"] = "training"
                status.pop("qualification")
                (directory / "status.json").write_text(json.dumps(status))
                window.poll()
                self.assertNotIn("9/28", window.qualification_status.text())
            finally:
                window.close()

    def test_policy_preview_matches_headless_and_refreshes_at_loop_boundary(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "walk"
            config = WalkConfig(rollout_steps=64, episode_seconds=.4, evaluation_interval=0)
            train_walk(run, 128, config)
            window = WalkTrainingWindow(run, render=False)
            window.poll_timer.stop()
            window.step_timer.stop()
            try:
                self.assertFalse(window.failed, window.preview_status.text())
                self.assertEqual(window.pointer["steps"], 128)
                policy, _, _ = load_walk_policy(run)
                with make_walk_env(replace(config, episode_seconds=PREVIEW_SECONDS),
                                   run / "motor_model.yaml", command_provider=window.command) as reference:
                    obs, _ = reference.reset(seed=12345, options={"initial_temperature_c": 40.})
                    np.testing.assert_array_equal(window.observation, obs)
                    for _ in range(10):
                        action = policy.predict(obs, deterministic=True)[0]
                        obs, _, terminated, truncated, _ = reference.step(action)
                        self.assertFalse(terminated or truncated)
                        window.tick()
                        self.assertFalse(window.failed, window.preview_status.text())
                        np.testing.assert_array_equal(window.observation, obs)
                        np.testing.assert_array_equal(window.env.unwrapped.data.qpos, reference.unwrapped.data.qpos)
                train_walk(run, 64, resume=True)
                window.poll()
                self.assertEqual(window.pointer["steps"], 128)
                window.loop_elapsed = PREVIEW_SECONDS-window.env.unwrapped.dt
                window.tick()
                self.assertEqual(window.pointer["steps"], 192)
                self.assertEqual(window.loop_count, 2)
                self.assertEqual(window.loop_elapsed, 0)
                window.env.unwrapped.plant.temperature[:] = 56
                window.tick()
                self.assertEqual(window.loop_resets, 1)
                self.assertEqual(window.pointer["steps"], 192)
                window.mode.setCurrentIndex(1)
                window.tick()
                frame = json.loads((run / "rollout.json").read_text())
                np.testing.assert_array_equal(window.env.unwrapped.data.qpos, frame["qpos"])
                self.assertIn("Copper heating", window.telemetry.text())
                self.assertIn("Live training rollout", window.preview_status.text())
                self.assertEqual(len(window.rows), 3)
                window.mode.setCurrentIndex(0)
                window.manual_command()
                np.testing.assert_allclose(window.env.unwrapped.velocity_command, [.1, 0, 0])
            finally:
                window.close()

    def test_stop_button_saves_during_qualification_and_reports_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "button-stop"
            with tempfile.TemporaryFile(mode="w+") as output:
                child = subprocess.Popen([str(self.binary), "--headless", f"--run-dir={run}",
                    "--num-envs=2", "--rollout-steps=64", "--episode-seconds=.4",
                    "--evaluation-interval=64", "--steps=100000"],
                    env=self.process_env, stdout=output, stderr=output)
                window = None
                try:
                    deadline = time.monotonic()+30
                    while time.monotonic() < deadline and child.poll() is None:
                        status = run / "status.json"
                        if status.exists() and json.loads(status.read_text())["state"] == "evaluating":
                            break
                        time.sleep(.05)
                    self.assertIsNone(child.poll())
                    self.assertEqual(json.loads(status.read_text())["state"], "evaluating")
                    window = WalkTrainingWindow(run, child, render=False)
                    window.poll_timer.stop()
                    window.step_timer.stop()
                    window.stop.click()  # Exercise the button without closing the window.
                    self.assertEqual(window.stop.text(), "Saving checkpoint…")
                    child.wait(timeout=15)
                    window.poll()
                    output.seek(0)
                    self.assertEqual(child.returncode, 0, output.read())
                    self.assertFalse(window.failed, window.preview_status.text())
                    self.assertEqual(window.stop.text(), "Stopped — checkpoint saved")
                    self.assertFalse(window.stop.isEnabled())
                    self.assertIn(str(run), window.stop.toolTip())
                    status = json.loads((run / "status.json").read_text())
                    self.assertEqual(status["state"], "stopped")
                    policy, _, pointer = load_walk_policy(run)
                    self.assertEqual(policy.num_timesteps, status["steps"])
                    self.assertEqual(pointer["updates"], status["updates"])
                finally:
                    if window is not None:
                        window.close()
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=15)

    def test_both_windows_distinguish_saved_completion_and_failure(self):
        for cls in (TrainingWindow, WalkTrainingWindow):
            for state, code, label in (("stopped", 0, "Stopped — checkpoint saved"),
                                       ("complete", 0, "Complete — checkpoint saved"),
                                       ("failed", 1, "Trainer failed — check log")):
                with self.subTest(window=cls.__name__, state=state), tempfile.TemporaryDirectory() as temp:
                    run = Path(temp)
                    (run / "status.json").write_text(json.dumps(dict(state=state, steps=64)))
                    child = SimpleNamespace(poll=lambda: code, returncode=code)
                    window = cls(run, child, render=False)
                    try:
                        window.poll()
                        self.assertEqual(window.stop.text(), label)
                        self.assertFalse(window.stop.isEnabled())
                    finally:
                        window.close()

    def test_one_command_starts_worker_closing_saves_and_watch_is_read_only(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "window"
            with tempfile.TemporaryFile(mode="w+") as output:
                child = subprocess.Popen([str(self.binary), f"--run-dir={run}", "--no-render",
                                          "--rollout-steps=64", "--episode-seconds=.4", "--evaluation-interval=0",
                                          "--max-copper-power-w=13", "--max-temperature-c=49", "--steps=100000"],
                                         env=self.process_env, stdout=output, stderr=output)
                try:
                    deadline = time.monotonic()+30
                    while time.monotonic() < deadline and child.poll() is None:
                        pointer = run / "latest.json"
                        if pointer.exists() and json.loads(pointer.read_text())["updates"] > 0:
                            break
                        time.sleep(.05)
                    self.assertIsNone(child.poll())
                    child.send_signal(signal.SIGTERM)  # Qt closes, then its worker checkpoints.
                    child.wait(timeout=15)
                    output.seek(0)
                    log = Path(str(run)+".log")
                    self.assertEqual(child.returncode, 0, output.read()+(log.read_text() if log.exists() else ""))
                finally:
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=15)
            status = json.loads((run / "status.json").read_text())
            self.assertEqual(status["state"], "stopped")
            policy, config, _ = load_walk_policy(run)
            self.assertGreater(policy._n_updates, 0)
            self.assertEqual(config.max_copper_power_w, 13)
            self.assertEqual(config.max_temperature_c, 49)
            before = {p.name: p.read_bytes() for p in run.glob("*.json")}
            result = subprocess.run([str(self.binary), f"--watch={run}", "--no-render", "--exit-after=1"],
                                    env=self.process_env, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertEqual(before, {p.name: p.read_bytes() for p in run.glob("*.json")})


if __name__ == "__main__":
    unittest.main()
