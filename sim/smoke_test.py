"""Check native physics, packaged model assets and Gymnasium without a display.

HalfCheetah is an upstream fixture, not the QuadBot model.
"""

import unittest

import gymnasium as gym
import mujoco
import numpy as np


class SimulatorDependenciesTest(unittest.TestCase):
    def test_headless_rollout(self):
        print(f"MuJoCo {mujoco.__version__}, Gymnasium {gym.__version__}")
        with gym.make("HalfCheetah-v5") as env:
            initial, _ = env.reset(seed=42)
            repeated, _ = env.reset(seed=42)
            np.testing.assert_array_equal(initial, repeated)
            self.assertTrue(env.observation_space.contains(initial))
            start_time = env.unwrapped.data.time
            env.action_space.seed(42)
            for _ in range(20):
                observation, reward, terminated, truncated, _ = env.step(
                    env.action_space.sample()
                )
                self.assertTrue(env.observation_space.contains(observation))
                self.assertTrue(np.isfinite(observation).all())
                self.assertTrue(np.isfinite(reward))
                self.assertFalse(terminated or truncated)
            self.assertGreater(env.unwrapped.data.time, start_time)


if __name__ == "__main__":
    unittest.main()
