"""Explicit GPU integration tests; run //sim:gpu_walk_test on a CUDA PC."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from functools import partial
from types import SimpleNamespace

import numpy as np
import torch
import warp as wp

from sim.robot_model import SIM_DIR
from sim.training_compute import training_envs, set_training_scale
from sim.walk_training import WalkConfig, make_walk_env, train_walk, load_walk_policy, evaluate_walk, GpuWalkQualification


class GpuWalkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_qualification_freezes_completed_worlds(self):
        from sim.gpu_walk import GpuWalkVecEnv
        config = WalkConfig(physics_device="gpu", episode_seconds=.06)
        env = GpuWalkVecEnv([partial(make_walk_env, config)]*2, auto_reset=False)
        try:
            env.reset()
            env.bases[0].max_steps = 1
            _, _, done, infos = env.step(np.zeros((2, 12)))
            self.assertEqual(done.tolist(), [True, False])
            terminal = infos[0]
            qpos, temperature = env.bases[0].data.qpos.copy(), env.bases[0].plant.temperature.copy()
            for _ in range(2):
                _, rewards, done, infos = env.step(np.zeros((2, 12)))
                np.testing.assert_array_equal(env.bases[0].data.qpos, qpos)
                np.testing.assert_array_equal(env.bases[0].plant.temperature, temperature)
                np.testing.assert_array_equal(env.physics.data.qpos.numpy()[0], qpos)
                self.assertEqual(env.result["counts"][0], 0)
                self.assertEqual(rewards[0], 0.)
                self.assertIs(infos[0], terminal)
            self.assertTrue(done.all())
        finally:
            env.close()

    def test_qualification_reuse_progress_and_cancel(self):
        config = WalkConfig(physics_device="gpu")
        policy = SimpleNamespace(num_timesteps=0, predict=lambda obs, **kw: (np.zeros((len(obs), 12)), None))
        with GpuWalkQualification(config, None, .04) as evaluator:
            progress = []
            first = evaluate_walk(policy, config, None, seconds=.04, gpu_evaluator=evaluator,
                                  progress_callback=progress.append)
            graph = evaluator.env.physics.graph
            second = evaluate_walk(policy, config, None, seconds=.04, gpu_evaluator=evaluator)
            self.assertIs(graph, evaluator.env.physics.graph)
            self.assertEqual(first["success_pct"], second["success_pct"])
            for first_row, second_row in zip(first["goals"], second["goals"]):
                for kind in ("hold", "policy"):
                    for key, value in first_row[kind].items():
                        if isinstance(value, (float, np.floating)):
                            self.assertAlmostEqual(value, second_row[kind][key], delta=1e-4)
                        else:
                            self.assertEqual(value, second_row[kind][key])
            self.assertEqual(progress[0]["phase"], "initializing")
            self.assertEqual(progress[-1]["completed"], 28)
            self.assertTrue(all(p["parallel_cases"] == 28 for p in progress))
            cancelled = evaluate_walk(policy, config, None, seconds=.04, gpu_evaluator=evaluator,
                                     stop_requested=lambda: True)
            self.assertIsNone(cancelled)

    def test_cpu_parity_selective_reset_and_thermal_stop(self):
        config = WalkConfig(physics_device="gpu", num_envs=2, rollout_steps=64,
                            episode_seconds=2, evaluation_interval=0)
        refs = [make_walk_env(config) for _ in range(2)]
        try:
            with training_envs(make_walk_env, config, SIM_DIR / "motor_model.yaml") as vec:
                observations = vec.reset()
                for i, ref in enumerate(refs):
                    obs, _ = ref.reset(seed=config.seed+i)
                    np.testing.assert_array_equal(observations[i], obs)
                # Only world zero reaches its time limit. World one must continue
                # undisturbed while zero gets new gains, friction and temperature.
                vec.bases[0].max_steps = refs[0].unwrapped.max_steps = 7
                set_training_scale(vec, "command_scale", .25)
                for ref in refs:
                    ref.unwrapped.command_scale = .25
                max_obs_error = max_reward_error = 0.
                first_observation = first_info = None
                for step in range(30):
                    if step == 20:
                        vec.bases[1].plant.temperature[:] = 55.001
                        refs[1].unwrapped.plant.temperature[:] = 55.001
                        with wp.ScopedDevice(vec.device):
                            vec.physics.temperature.assign(np.array([b.plant.temperature for b in vec.bases]))
                    actions = .05*np.sin(step*.3 + np.arange(24).reshape(2, 12))
                    observations, rewards, dones, infos = vec.step(actions)
                    if step == 0:
                        first_observation, first_info = observations, infos[0]
                        observation_copy = observations.copy()
                        temperature_copy = first_info["max_temperature_c"]
                    for i, ref in enumerate(refs):
                        expected, reward, terminated, truncated, info = ref.step(actions[i])
                        self.assertEqual(dones[i], terminated or truncated)
                        self.assertEqual(infos[i]["termination_reason"], info["termination_reason"])
                        self.assertAlmostEqual(infos[i]["time"], info["time"], places=8)
                        self.assertAlmostEqual(infos[i]["copper_energy_j"], info["copper_energy_j"], delta=.002)
                        self.assertAlmostEqual(infos[i]["absolute_mechanical_work_j"],
                                               info["absolute_mechanical_work_j"], delta=.002)
                        max_reward_error = max(max_reward_error, abs(rewards[i]-reward))
                        if dones[i]:
                            np.testing.assert_allclose(infos[i]["terminal_observation"], expected, atol=.003, rtol=.001)
                            self.assertEqual(infos[i]["TimeLimit.truncated"], truncated and not terminated)
                            expected, _ = ref.reset()
                        max_obs_error = max(max_obs_error, float(np.max(np.abs(observations[i]-expected))))
                        np.testing.assert_allclose(observations[i], expected, atol=.003, rtol=.001)
                        np.testing.assert_allclose(vec.bases[i].data.qpos, ref.unwrapped.data.qpos, atol=2e-5)
                    if step == 20:
                        self.assertEqual(infos[1]["termination_reason"], "over_temperature")
                        self.assertEqual(vec.result["counts"][1], 1)
                        self.assertEqual(vec.result["counts"][0], 40)
                    np.testing.assert_allclose(vec.physics.model.geom_friction.numpy(),
                                               [r.unwrapped.model.geom_friction for r in refs], rtol=1e-6)
                    np.testing.assert_allclose(vec.physics.params.numpy()[:, 4:6],
                                               [[r.unwrapped.plant.kp, r.unwrapped.plant.kd] for r in refs])
                self.assertLess(max_reward_error, .0001)
                np.testing.assert_array_equal(first_observation, observation_copy)
                self.assertEqual(first_info["max_temperature_c"], temperature_copy)
                # A failed GPU batch must never be handed to PPO as valid data.
                with wp.ScopedDevice(vec.device):
                    vec.physics.data.overflow.fill_(1)
                with self.assertRaisesRegex(RuntimeError, "overflow"):
                    vec.step(actions)
                print(f"CPU/GPU parity: max normalized observation error={max_obs_error:g}, reward error={max_reward_error:g}")
        finally:
            for ref in refs:
                ref.close()

    def test_checkpoint_resume_and_cpu_policy_playback(self):
        config = WalkConfig(physics_device="gpu", num_envs=2, rollout_steps=64,
                            episode_seconds=.4, evaluation_interval=0)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)/"run"
            train_walk(directory, 64, config)
            train_walk(directory, 64, resume=True)
            policy, saved, _ = load_walk_policy(directory)
            self.assertEqual(policy.num_timesteps, 128)
            self.assertEqual(saved.physics_device, "gpu")
            status = json.loads((directory/"status.json").read_text())
            self.assertEqual(status["compute"]["physics_device"], "gpu")
            self.assertEqual(status["state"], "complete")
            # The existing viewer/evaluator can load GPU-trained checkpoints on CPU.
            with make_walk_env(replace(saved, physics_device="cpu")) as env:
                obs, _ = env.reset(seed=1)
                obs, reward, *_ = env.step(policy.predict(obs, deterministic=True)[0])
                self.assertTrue(np.isfinite(obs).all() and np.isfinite(reward))

    def test_missing_cuda_is_reported_as_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)/"run"
            with patch("sim.gpu_walk.wp.is_cuda_available", return_value=False):
                with self.assertRaisesRegex(RuntimeError, "CUDA-capable"):
                    train_walk(directory, 64, WalkConfig(physics_device="gpu", num_envs=2, rollout_steps=64))
            status = json.loads((directory/"status.json").read_text())
            self.assertEqual(status["state"], "failed")
            self.assertIn("--physics-device=cpu", status["error"])


if __name__ == "__main__":
    unittest.main()
