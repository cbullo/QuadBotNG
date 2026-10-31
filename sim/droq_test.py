"""DroQ update scheduling, terminal replay and checkpoint contracts."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3.common.vec_env import DummyVecEnv

from sim.droq import DroQ
from sim.walk_training import publish_walk


class ShortEpisode(gym.Env):
    observation_space = gym.spaces.Box(-10, 10, (4,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, (2,), dtype=np.float32)

    def __init__(self, timeout=False):
        self.timeout = timeout

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.zeros(4, np.float32), {}

    def step(self, action):
        self.steps += 1
        end = self.steps == 2
        return (np.full(4, self.steps, np.float32), float(1-np.square(action).sum()),
                end and not self.timeout, end and self.timeout, {})


class DroQTest(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1)

    def model(self, env, ratio=3):
        return DroQ(env=env, device="cpu", seed=42, utd_ratio=ratio, learning_starts=0,
                    buffer_size=32, batch_size=4, policy_kwargs=dict(net_arch=[8, 8]))

    def test_vector_utd_actor_delay_dropout_and_terminal_bootstrap(self):
        env = DummyVecEnv([ShortEpisode, lambda: ShortEpisode(timeout=True)])
        self.addCleanup(env.close)
        model = self.model(env)
        initial = {key: value.clone() for key, value in model.actor.state_dict().items()}
        target_modes = []
        hook = model.critic_target.register_forward_pre_hook(
            lambda module, inputs: target_modes.append(module.training))
        self.addCleanup(hook.remove)
        model.learn(4)
        self.assertEqual(model._n_updates, 12)  # 4 transitions, not 2 vector steps.
        self.assertEqual(model.actor_updates, 4)
        self.assertEqual(model.target_entropy, -1.)
        self.assertTrue(target_modes and all(target_modes))
        self.assertTrue(any(not torch.equal(value, model.actor.state_dict()[key])
                            for key, value in initial.items()))
        self.assertEqual(sum(isinstance(m, torch.nn.Dropout) for m in model.critic.modules()), 4)
        self.assertEqual(sum(isinstance(m, torch.nn.LayerNorm) for m in model.critic.modules()), 4)
        self.assertFalse(any(isinstance(m, torch.nn.Dropout) for m in model.actor.modules()))
        replay = model.replay_buffer
        np.testing.assert_array_equal(replay.next_observations[1], np.full((2, 4), 2))
        np.testing.assert_array_equal(replay.timeouts[1], [0, 1])
        with patch("numpy.random.randint", return_value=np.array([0, 1])):
            samples = replay._get_samples(np.array([1, 1]))
        np.testing.assert_array_equal(samples.dones.numpy().ravel(), [1, 0])

    def test_stop_during_optimization_and_resume_paired_replay(self):
        env = DummyVecEnv([ShortEpisode])
        self.addCleanup(env.close)
        model = self.model(env, ratio=2)
        stopped = False

        def progress(completed, total):
            nonlocal stopped
            stopped = model._n_updates >= 1

        model.stop_requested = lambda: stopped
        model.update_progress = progress
        model.learn(1)
        self.assertEqual(model._n_updates, 1)
        self.assertEqual(model.actor_updates, 0)
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            pointer = publish_walk(model, directory)
            restored = DroQ.load(directory / pointer["file"], env=env, device="cpu")
            restored.load_replay_buffer(directory / pointer["replay_file"])
            self.assertEqual(restored.replay_buffer.pos, model.replay_buffer.pos)
            np.testing.assert_array_equal(restored.replay_buffer.observations, model.replay_buffer.observations)
            np.testing.assert_array_equal(restored.predict(np.zeros(4), deterministic=True)[0],
                                          model.predict(np.zeros(4), deterministic=True)[0])
            self.assertFalse(restored.stop_requested())  # No serialized UI/stop closures.
            restored.learn(1, reset_num_timesteps=False)
            self.assertEqual(restored._n_updates, 3)
            self.assertEqual(restored.actor_updates, 1)
            self.assertEqual(restored.replay_buffer.pos, 2)
            updated = publish_walk(restored, directory)
            self.assertFalse((directory / pointer["replay_file"]).exists())
            self.assertTrue((directory / updated["replay_file"]).exists())
            # A failed replay write must not invalidate the published pair.
            restored.num_timesteps += 1
            with patch.object(restored, "save_replay_buffer", side_effect=OSError("disk full")):
                with self.assertRaises(OSError):
                    publish_walk(restored, directory)
            self.assertEqual(json.loads((directory / "latest.json").read_text()), updated)
            self.assertTrue((directory / updated["replay_file"]).exists())

    def test_wrapped_replay_round_trip(self):
        env = DummyVecEnv([ShortEpisode, lambda: ShortEpisode(timeout=True)])
        self.addCleanup(env.close)
        model = self.model(env)
        replay = model.replay_buffer
        for index in range(replay.buffer_size + 3):
            values = np.full((2, 4), index, np.float32)
            replay.add(values, values+1, np.zeros((2, 2), np.float32),
                       np.ones(2), np.ones(2), [{}, {"TimeLimit.truncated": True}])
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "replay.npz"
            model.save_replay_buffer(path)
            restored = self.model(env)
            restored.load_replay_buffer(path)
            self.assertTrue(restored.replay_buffer.full)
            self.assertEqual(restored.replay_buffer.pos, 3)
            for name in ("observations", "next_observations", "actions", "rewards", "dones", "timeouts"):
                np.testing.assert_array_equal(getattr(restored.replay_buffer, name), getattr(replay, name))


if __name__ == "__main__":
    unittest.main()
