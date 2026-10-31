"""Subprocess physics equivalence, RPCs, terminal observations, and checkpoint resume."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

from sim.robot_model import SIM_DIR
from sim.training import TrainingConfig, make_env, train, load_policy
from sim.training_compute import training_envs, set_training_scale, default_num_envs
from sim.walk_training import WalkConfig, make_walk_env, train_walk, load_walk_policy


class ComputeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_parallel_matches_single_environments_and_autoreset(self):
        for config, factory, field in (
            (WalkConfig(num_envs=2, rollout_steps=64, episode_seconds=.06), make_walk_env, "command_scale"),
            (TrainingConfig(num_envs=2, rollout_steps=64, episode_seconds=.06), make_env, "goal_scale"),
        ):
            with training_envs(factory, config, SIM_DIR / "motor_model.yaml") as vec:
                processes = vec.processes
                refs = [factory(config, SIM_DIR / "motor_model.yaml") for _ in range(2)]
                try:
                    obs = vec.reset()
                    for i, ref in enumerate(refs):
                        expected, _ = ref.reset(seed=config.seed+i)
                        np.testing.assert_array_equal(obs[i], expected)
                    set_training_scale(vec, field, .25)
                    self.assertEqual(vec.get_attr(field), [.25, .25])
                    for ref in refs:
                        setattr(ref.unwrapped, field, .25)
                    for step in range(3):
                        actions = np.full((2, 12), .001*step, dtype=np.float32)
                        obs, rewards, dones, infos = vec.step(actions)
                        for i, ref in enumerate(refs):
                            expected, reward, terminated, truncated, _ = ref.step(actions[i])
                            self.assertEqual(bool(dones[i]), terminated or truncated)
                            self.assertAlmostEqual(float(rewards[i]), reward, places=5)
                            if dones[i]:
                                self.assertTrue(infos[i]["TimeLimit.truncated"])
                                np.testing.assert_array_equal(infos[i]["terminal_observation"], expected)
                                expected, _ = ref.reset()
                            np.testing.assert_array_equal(obs[i], expected)
                        snapshot = vec.env_method("training_snapshot", indices=0)[0]
                        np.testing.assert_array_equal(snapshot["qpos"], refs[0].unwrapped.data.qpos)
                finally:
                    for ref in refs:
                        ref.close()
            self.assertTrue(all(not process.is_alive() for process in processes))

    def test_both_trainers_resume_parallel_checkpoints(self):
        for config, trainer, loader in (
            (WalkConfig(num_envs=2, rollout_steps=64, episode_seconds=.4, evaluation_interval=0), train_walk, load_walk_policy),
            (TrainingConfig(num_envs=2, rollout_steps=64, episode_seconds=.4, evaluation_interval=0,
                            initialization="random"), train, load_policy),
        ):
            with tempfile.TemporaryDirectory() as tmp:
                run = Path(tmp) / "run"
                trainer(run, 64, config)
                trainer(run, 64, resume=True)
                policy, saved, _ = loader(run)
                self.assertEqual(policy.num_timesteps, 128)
                self.assertEqual(policy.n_steps, 32)
                self.assertEqual(saved.num_envs, 2)
                status = json.loads((run / "status.json").read_text())
                self.assertEqual(status["state"], "complete")
                self.assertEqual(status["compute"]["vector_backend"], "SubprocVecEnv")
                self.assertTrue((run / "rollout.json").exists())

    def test_default_worker_counts(self):
        for available, gui, headless in ((16, 4, 8), (12, 4, 4), (6, 2, 2), (4, 2, 2), (1, 1, 1)):
            with patch("os.sched_getaffinity", return_value=set(range(available))):
                self.assertEqual(default_num_envs(), gui)
                self.assertEqual(default_num_envs(headless=True), headless)

    def test_invalid_worker_counts(self):
        for cls in (TrainingConfig, WalkConfig):
            for count in (0, -1, 3, 512):
                with self.assertRaises(ValueError):
                    cls(num_envs=count).validate()


if __name__ == "__main__":
    unittest.main()
