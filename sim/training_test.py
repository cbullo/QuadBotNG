"""Real PPO updates, checkpoint replay, resumption and native viewer lifecycle."""
import json
from dataclasses import replace
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
from PySide6 import QtTest, QtWidgets
from stable_baselines3 import PPO
import torch

from sim.training import (TrainingConfig, load_policy, make_env, read_manifest, train,
                          evaluate_tracking, record_evaluation, atomic_json, tracking_success)
from sim.training_view import TrainingWindow


class TrainingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)
        cls.root = Path(os.environ["TEST_SRCDIR"]) / "_main"
        cls.binary = cls.root / "sim/train"
        cls.process_env = dict(os.environ, RUNFILES_DIR=str(cls.root.parent))

    def test_actual_updates_checkpoint_reload_resume_and_preview(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "learning"
            config = TrainingConfig(rollout_steps=64, episode_seconds=.4, initialization="random", evaluation_interval=0)
            train(run, 128, config)
            self.assertEqual(json.loads((run / "status.json").read_text())["state"], "complete")
            initial = PPO.load(run / "policy-000000000-000000.zip", device="cpu")
            np.testing.assert_allclose(torch.exp(initial.policy.log_std).detach().numpy() * config.action_scale,
                                       np.exp(-2), rtol=1e-6)
            trained, saved_config, pointer = load_policy(run)
            self.assertEqual(saved_config, config)
            self.assertEqual(pointer["steps"], 128)
            self.assertIsInstance(trained.policy.mlp_extractor.policy_net[1], torch.nn.ELU)
            self.assertGreater(pointer["updates"], 0)
            self.assertTrue(any(not torch.equal(value, trained.policy.state_dict()[key])
                                for key, value in initial.policy.state_dict().items()))
            with make_env(config, run / "motor_model.yaml") as env:
                obs, _ = env.reset(seed=10)
                self.assertEqual(obs.shape, (240,))
                self.assertTrue(env.observation_space.contains(obs))
                action, _ = trained.predict(obs, deterministic=True)
                reloaded, _, _ = load_policy(run)
                np.testing.assert_array_equal(action, reloaded.predict(obs, deterministic=True)[0])
                obs, _, _, _, _ = env.step(action)
                self.assertTrue(np.isfinite(obs).all())
                obs, _ = env.reset(options={"command": [0, 0, -.15, 0, 0, 0]})
                self.assertAlmostEqual(obs[-58], -1)
                self.assertEqual(env.unwrapped.action_scale, 12)
                self.assertEqual(env.unwrapped.min_base_height, .06)
            train(run, 64, resume=True)
            resumed, _, pointer = load_policy(run)
            self.assertEqual(pointer["steps"], 192)
            self.assertGreater(resumed._n_updates, trained._n_updates)
            rows = [json.loads(line) for line in (run / "metrics.jsonl").read_text().splitlines()]
            self.assertEqual([row["steps"] for row in rows], [64, 128, 192])
            self.assertTrue(all(row["position_error_mm"] is not None for row in rows))
            window = TrainingWindow(run, render=False)
            try:
                self.assertEqual(window.mode.currentIndex(), 1)
                self.assertEqual(window.env.unwrapped.max_steps, 1500)
                window.mode.setCurrentIndex(0)
                window.show()
                window.poll()
                for _ in range(8):
                    QtTest.QTest.qWait(20)
                self.assertFalse(window.failed, window.preview_status.text())
                self.assertGreater(window.ticks, 0)
                self.assertIn("training rollout", window.preview_status.text())
                frame = json.loads((run / "rollout.json").read_text())
                np.testing.assert_array_equal(window.env.unwrapped.data.qpos, frame["qpos"])
                self.assertIn("Training episode", window.goal_status.text())
                window.mode.setCurrentIndex(1)
                self.assertEqual(window.pointer["steps"], 192)
                self.assertEqual(len(window.rows), 3)
                self.assertIn("Evaluation checkpoint", window.preview_status.text())
                first_goal = window.env.unwrapped.command.copy()
                window.restart()
                self.assertFalse(np.array_equal(first_goal, window.env.unwrapped.command))
                window.manual_goal()
                self.assertFalse(window.cycle_goals.isChecked())
                # A Qt preview must execute the same saved policy and physics
                # as a headless rollout, including the selected sampling mode.
                window.step_timer.stop()
                window.poll_timer.stop()
                for mode in (0, 1):
                    window.action_mode.setCurrentIndex(mode)
                    window.restart()
                    with make_env(replace(config, episode_seconds=30), run / "motor_model.yaml",
                                  window.command) as reference:
                        obs, _ = reference.reset(seed=12345)
                        np.testing.assert_array_equal(window.observation, obs)
                        for _ in range(10):
                            rng = torch.get_rng_state()
                            action = resumed.predict(obs, deterministic=mode == 1)[0]
                            obs, _, terminated, truncated, _ = reference.step(action)
                            self.assertFalse(terminated or truncated)
                            torch.set_rng_state(rng)
                            window.tick()
                            self.assertFalse(window.failed, window.preview_status.text())
                            np.testing.assert_array_equal(window.observation, obs)
                            np.testing.assert_array_equal(window.env.unwrapped.data.qpos,
                                                          reference.unwrapped.data.qpos)
                        self.assertIn("Applied motor target offset", window.evaluation.text())
            finally:
                window.close()
            with make_env(config, run / "motor_model.yaml") as env:
                summary = dict(steps=resumed.num_timesteps, goal_scale=config.goal_scale,
                               success_pct=75., survival_pct=100., tracking_gain_pct=60., goals=[])
                with patch("sim.training.evaluate_tracking", return_value=summary):
                    record_evaluation(resumed, config, run, env, SimpleNamespace(stop_requested=lambda: False))
                    self.assertEqual(resumed.goal_scale, config.goal_scale)
                    record_evaluation(resumed, config, run, env, SimpleNamespace(stop_requested=lambda: False))
                    self.assertAlmostEqual(resumed.goal_scale, config.goal_scale * 1.5)
                    self.assertEqual(env.unwrapped.goal_scale, resumed.goal_scale)
                resumed.save(run / "curriculum-check.zip")
                atomic_json(run / "latest.json", dict(pointer, file="curriculum-check.zip"))
                _, restored_config, _ = load_policy(run)
                self.assertEqual(restored_config.goal_scale, resumed.goal_scale)
            # Runs made before reward versioning keep their original objective.
            manifest = json.loads((run / "config.json").read_text())
            del manifest["training"]["reward_version"]
            del manifest["training"]["min_base_height"]
            del manifest["training"]["observation_version"]
            for key in ("action_filter_seconds", "target_rate_limit", "goal_scale",
                        "learning_rate", "target_kl", "evaluation_interval", "initialization",
                        "policy_version", "command_interval", "robustness", "evaluation_suite"):
                del manifest["training"][key]
            manifest["training"]["action_scale"] = 1
            manifest["training"]["goal_mode"] = "poses"
            (run / "config.json").write_text(json.dumps(manifest))
            old = read_manifest(run)
            self.assertEqual(old.reward_version, "legacy_v1")
            self.assertEqual(old.observation_version, "legacy_v1")
            self.assertEqual(old.min_base_height, .12)
            self.assertEqual(old.action_filter_seconds, 0)
            self.assertEqual(old.target_rate_limit, 0)
            self.assertEqual(old.goal_scale, 1)
            self.assertEqual(old.policy_version, "small_v1")
            self.assertEqual(old.robustness, "none")
            self.assertEqual(old.command_interval, 0)
            self.assertEqual(old.evaluation_suite, "legacy_v1")
            with make_env(old, run / "motor_model.yaml") as env:
                obs, _ = env.reset(options={"command": [0, 0, -.005, 0, 0, 0]})
                self.assertEqual(env.unwrapped.action_scale, 1)
                self.assertEqual(env.unwrapped.min_base_height, .12)
                self.assertAlmostEqual(obs[-58], -.5)

    def test_evaluation_distinguishes_holding_from_tracking(self):
        config = TrainingConfig(evaluation_suite="legacy_v1", robustness="none")
        hold = SimpleNamespace(num_timesteps=0, predict=lambda *a, **kw: (np.zeros(12), None))
        # Short rollouts isolate scorer semantics; physics and real PPO are
        # tested separately. Holding a nonzero goal cannot count as success.
        from sim.robot_model import SIM_DIR
        report = evaluate_tracking(hold, config, SIM_DIR / "motor_model.yaml", seconds=.04)
        self.assertAlmostEqual(report["tracking_gain_pct"], 0)
        self.assertEqual(report["success_pct"], 100 / 16)
        self.assertEqual(report["survival_pct"], 100)
        self.assertEqual(len(report["goals"]), 16)

        report = evaluate_tracking(hold, replace(config, evaluation_suite="pose_suite_v2",
                                                 robustness="mild_v1"),
                                   SIM_DIR / "motor_model.yaml", seconds=.04)
        self.assertEqual(len(report["goals"]), 32)
        self.assertEqual({r["seed"] for r in report["goals"]}, {12345, 54321})
        self.assertAlmostEqual(report["tracking_gain_pct"], 0)

    def test_success_requires_sustained_contact_and_no_earlier_slip(self):
        command = np.zeros(6)
        samples = np.array([[.001, .004, 1], [.001, .004, 1]])
        self.assertTrue(tracking_success(command, samples, .004, False))
        self.assertFalse(tracking_success(command, samples, .006, False))
        self.assertFalse(tracking_success(command, samples, .004, True))
        samples[0, 2] = 0
        self.assertFalse(tracking_success(command, samples, .004, False))
        samples[0, 2] = 1
        samples[0, 0] = .003  # Mean meets 2 mm, but not sustained tracking.
        self.assertFalse(tracking_success(command, samples, .004, False))

    def test_training_command_transitions_preserve_anchors_and_reward_command(self):
        from sim.robot_model import SIM_DIR
        config = TrainingConfig(command_interval=.04, robustness="none", episode_seconds=.4)
        with make_env(config, SIM_DIR / "motor_model.yaml") as env:
            obs, _ = env.reset(seed=5)
            anchors = env.unwrapped.foot_anchors.copy()
            changed = False
            for _ in range(8):
                command = env.unwrapped.command.copy()
                obs, _, terminated, truncated, info = env.step(np.zeros(12))
                self.assertFalse(terminated or truncated)
                expected = env.unwrapped.reference_position + env.unwrapped.reference_rotation @ command[:3]
                self.assertAlmostEqual(info["position_error_m"],
                                       np.linalg.norm(expected-env.unwrapped.data.body("base_link").xpos))
                changed |= not np.array_equal(command, env.unwrapped.command)
                np.testing.assert_array_equal(env.unwrapped.foot_anchors, anchors)
                np.testing.assert_allclose(obs[-60:-54], env.unwrapped.command / [.06, .03, .15, .2, .25, .25])
            self.assertTrue(changed)
            command = np.array([0, 0, -.01, 0, 0, 0])
            env.reset(seed=5, options={"command": command})
            for _ in range(8):
                env.step(np.zeros(12))
                np.testing.assert_array_equal(env.unwrapped.command, command)

    def test_thirty_second_loop_goal_changes_and_checkpoint_boundary(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "loops"
            train(run, 64, TrainingConfig(rollout_steps=64, episode_seconds=.4, initialization="random", evaluation_interval=0))
            window = TrainingWindow(run, render=False)
            window.step_timer.stop()
            window.poll_timer.stop()
            try:
                self.assertEqual(window.pointer["steps"], 64)
                # Isolate preview scheduling from policy quality; updates and
                # inference with the real PPO policy are checked above.
                window.policy.predict = lambda *a, **kw: (np.zeros(12), None)
                first_command = window.env.unwrapped.command.copy()
                anchors = window.env.unwrapped.foot_anchors.copy()
                train(run, 64, resume=True)  # A newer checkpoint arrives mid-loop.
                for _ in range(205):
                    window.tick()
                self.assertFalse(window.failed, window.preview_status.text())
                self.assertEqual(window.pointer["steps"], 64)
                self.assertAlmostEqual(window.loop_elapsed, 4.1)
                self.assertFalse(np.array_equal(first_command, window.env.unwrapped.command))
                np.testing.assert_array_equal(anchors, window.env.unwrapped.foot_anchors)
                np.testing.assert_allclose(window.observation[-60:-54],
                    window.env.unwrapped.command / [.06, .03, .15, .2, .25, .25])
                # A thermal failure resets the episode but keeps the loop's policy.
                window.env.unwrapped.plant.temperature[:] = 56
                window.tick()
                self.assertEqual(window.loop_resets, 1)
                self.assertEqual(window.pointer["steps"], 64)
                for _ in range(1500 - 206):
                    window.tick()
                self.assertFalse(window.failed, window.preview_status.text())
                self.assertEqual(window.loop_count, 2)
                self.assertEqual(window.pointer["steps"], 128)
                self.assertEqual(window.loop_elapsed, 0)
                loads = window.loaded_checkpoints
                window.restart()  # No newer checkpoint: retain the loaded policy.
                self.assertEqual(window.loaded_checkpoints, loads)
                self.assertEqual(json.loads((run / "config.json").read_text())["training"]["episode_seconds"], .4)
            finally:
                window.close()

    def test_cli_window_owns_trainer_and_saves_on_close(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "window-run"
            result = subprocess.run([
                str(self.binary), f"--run-dir={run}", "--no-render", "--exit-after=6", "--num-envs=1",
                "--rollout-steps=64", "--episode-seconds=.4", "--steps=100000", "--from-scratch", "--evaluation-interval=0",
            ], env=self.process_env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr +
                             (Path(str(run)+".log").read_text() if Path(str(run)+".log").exists() else ""))
            status = json.loads((run / "status.json").read_text())
            self.assertEqual(status["state"], "stopped")
            self.assertGreater(status["updates"], 0)
            load_policy(run)  # A complete final checkpoint is readable.

    def test_headless_sigterm_checkpoint_and_existing_run_protection(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp) / "headless"
            with tempfile.TemporaryFile(mode="w+") as log:
                child = subprocess.Popen([
                    str(self.binary), "--headless", f"--run-dir={run}", "--rollout-steps=64",
                    "--episode-seconds=.4", "--steps=100000", "--from-scratch", "--evaluation-interval=0",
                ], env=self.process_env, stdout=log, stderr=log)
                try:
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        pointer = run / "latest.json"
                        if pointer.exists() and json.loads(pointer.read_text())["updates"] > 0:
                            break
                        if child.poll() is not None:
                            break
                        time.sleep(.05)
                    self.assertIsNone(child.poll())
                    child.send_signal(signal.SIGTERM)
                    child.wait(timeout=10)
                    log.seek(0)
                    self.assertEqual(child.returncode, 0, log.read())
                    self.assertEqual(json.loads((run / "status.json").read_text())["state"], "stopped")
                    load_policy(run)
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.wait()
            result = subprocess.run([str(self.binary), "--headless", f"--run-dir={run}", "--steps=64"],
                                    env=self.process_env, capture_output=True, text=True, timeout=15)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("run already exists", result.stderr)


if __name__ == "__main__":
    unittest.main()
