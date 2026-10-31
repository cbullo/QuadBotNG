"""Walking/thermal contracts and actual PPO checkpoint training on QuadBot."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from gymnasium.utils.env_checker import check_env
import mujoco
import numpy as np
import torch

from sim.gym_motor import VoltageMotorPlant
from sim.robot_model import LEGS, SIM_DIR, active_joints, motor_matrix
from sim.training import read_manifest
from sim.walk_env import QuadBotWalkEnv, energy_motion_score, motion_shortfall, walking_reward
from sim.walk_training import (WalkConfig, evaluate_walk, load_walk_policy, make_walk_env,
                               read_walk_manifest, train_walk, walk_success)


class WalkingTest(unittest.TestCase):
    def test_gym_seeded_physics_and_differential(self):
        with QuadBotWalkEnv(episode_seconds=.2) as env:
            check_env(env, skip_render_check=True)
            traces = []
            for _ in range(2):
                obs, info = env.reset(seed=42, options={"command": [.1, 0, 0]})
                self.assertTrue(env.observation_space.contains(obs))
                self.assertGreaterEqual(info["max_temperature_c"], 20)
                self.assertLess(info["max_temperature_c"], 45)
                trace = []
                for _ in range(5):
                    obs, reward, terminated, truncated, info = env.step(np.zeros(12))
                    self.assertFalse(terminated or truncated, info)
                    self.assertTrue(env.observation_space.contains(obs))
                    trace.append(np.r_[env.data.qpos.copy(), obs["imu"], obs["motor_temperature"], reward])
                traces.append(trace)
            np.testing.assert_array_equal(*traces)
            self.assertGreater(info["velocity_error_m_s"], .09)
            for leg in LEGS:
                ids = [env.model.actuator(f"motor_{leg}{axis}").id for axis in "IOZ"]
                angles = [env.data.qpos[env.model.joint(j).qposadr[0]] for j in active_joints(leg)]
                np.testing.assert_allclose(env.data.actuator_length[ids], motor_matrix(leg) @ angles, atol=1e-9)

    def test_walking_allows_foot_travel_and_preserves_command_reward_order(self):
        with QuadBotWalkEnv(robustness="none", resample_seconds=.04, episode_seconds=.4) as env:
            env.reset(seed=3)
            env.foot_anchors[:, 0] += 1.  # Deliberately not a walking failure.
            changed = False
            for _ in range(8):
                command = env.velocity_command.copy()
                obs, _, terminated, truncated, info = env.step(np.zeros(12))
                self.assertFalse(terminated or truncated, info)
                np.testing.assert_array_equal(info["command"], command)
                np.testing.assert_allclose(obs["command"], env.velocity_command)
                changed |= not np.array_equal(command, env.velocity_command)
            self.assertTrue(changed)
            env.reset(seed=3, options={"command": [.1, 0, 0]})
            for _ in range(8):
                env.step(np.zeros(12))
                np.testing.assert_array_equal(env.velocity_command, [.1, 0, 0])

    def test_energy_heat_and_temperature_are_scored(self):
        plant = VoltageMotorPlant(np.full(12, .2941995))
        np.testing.assert_allclose(plant.copper_loss(.0420285), 1.2, atol=1e-12)
        np.testing.assert_allclose(plant.copper_loss(.0420285/2), .3, atol=1e-12)
        with QuadBotWalkEnv(robustness="none") as env:
            scores = []
            for temperature in (20, 50):
                env.reset(seed=42, options={"command": [0, 0, 0]})
                env.plant.temperature[:] = temperature
                _, reward, terminated, _, info = env.step(np.zeros(12))
                self.assertFalse(terminated)
                self.assertGreater(info["copper_energy_j"], 0)
                self.assertAlmostEqual(info["copper_energy_j"], info["copper_power_w"]*env.dt)
                self.assertGreaterEqual(info["energy_efficiency_score"], 0)
                self.assertLess(info["energy_efficiency_score"], 1)
                self.assertGreaterEqual(info["absolute_mechanical_work_j"], info["positive_mechanical_work_j"])
                scores.append((reward, info))
            self.assertLess(scores[1][0], scores[0][0])
            self.assertAlmostEqual(scores[0][1]["copper_power_w"], scores[1][1]["copper_power_w"])
            env.plant.temperature[:] = 56
            _, reward, terminated, _, info = env.step(np.zeros(12))
            self.assertTrue(terminated)
            self.assertEqual(info["termination_reason"], "over_temperature")
            self.assertEqual(reward, -25.)

    def test_useful_motion_requires_commanded_chassis_progress(self):
        for command, velocity, angular, expected in (
            ([0, 0, 0], [0, 0, 0], [0, 0, 0], 0),
            ([.1, 0, 0], [0, 0, 0], [0, 0, 0], 1),
            ([.1, 0, 0], [-.1, 0, 0], [0, 0, 0], 1),
            ([.1, 0, 0], [0, .1, 0], [0, 0, 0], 1),
            ([.1, 0, 0], [.05, 0, 0], [0, 0, 0], .5),
            ([.1, 0, 0], [.2, 0, 0], [0, 0, 0], 0),
            ([0, -.1, 0], [0, -.1, 0], [0, 0, 0], 0),
            ([0, 0, -.4], [0, 0, 0], [0, 0, -.4], 0),
            ([0, 0, -.4], [0, 0, 0], [0, 0, .4], 1),
            ([.1, 0, .4], [.1, 0, 0], [0, 0, 0], .5),
        ):
            with self.subTest(command=command, velocity=velocity, angular=angular):
                self.assertAlmostEqual(motion_shortfall(command, velocity, angular), expected)

    def test_costs_lower_reward_without_rewarding_early_failure(self):
        previous = float("inf")
        for cost in (0., 1., 10., 100.):
            terms = {"tracking_xy": 1., "copper_heat": -cost}
            running, terminal = walking_reward(terms, .02, False, "walk_energy_v2")
            self.assertGreater(running, 0.)
            self.assertLess(running, previous)
            self.assertEqual(terminal, 0.)
            self.assertEqual(walking_reward(terms, .02, True, "walk_energy_v2"), (0., -25.))
            previous = running

    def test_unproductive_power_and_heat_cost_preserve_physics(self):
        results = {}
        for version in ("walk_energy_v1", "walk_energy_v2"):
            with QuadBotWalkEnv(robustness="none", reward_version=version) as env:
                for temperature in (20, 50):
                    for command in ([0, 0, 0], [.1, 0, 0]):
                        env.reset(seed=42, options={"command": command})
                        env.plant.temperature[:] = temperature
                        _, reward, terminated, _, info = env.step(np.zeros(12))
                        self.assertFalse(terminated)
                        results[version, temperature, command[0]] = (reward, info, env.data.qpos.copy())
        for temperature in (20, 50):
            for command in (0, .1):
                old = results["walk_energy_v1", temperature, command]
                new = results["walk_energy_v2", temperature, command]
                np.testing.assert_array_equal(old[2], new[2])
                self.assertEqual(old[1]["copper_power_w"], new[1]["copper_power_w"])
                for term in ("copper_heat", "temperature"):
                    self.assertEqual(new[1]["reward_terms"][term], 2*old[1]["reward_terms"][term])
            stand = results["walk_energy_v2", temperature, 0][1]["reward_terms"]
            move = results["walk_energy_v2", temperature, .1][1]["reward_terms"]
            self.assertEqual(stand["unproductive_power"], 0.)
            self.assertLess(move["unproductive_power"], 0.)
        cold = results["walk_energy_v2", 20, .1]
        hot = results["walk_energy_v2", 50, .1]
        self.assertLess(hot[0], cold[0])
        self.assertLess(hot[1]["reward_terms"]["unproductive_power"], cold[1]["reward_terms"]["unproductive_power"])

    def test_sideways_fall_penalty_and_normal_time_limit(self):
        for version in ("walk_energy_v1", "walk_energy_v2", "walk_energy_v3"):
            with QuadBotWalkEnv(robustness="none", episode_seconds=.02, reward_version=version) as env:
                env.reset(seed=42, options={"command": [0, 0, 0]})
                _, reward, terminated, truncated, info = env.step(np.zeros(12))
                self.assertFalse(terminated)
                self.assertTrue(truncated)
                self.assertEqual(info["termination_reward"], 0.)
                env.reset(seed=42, options={"command": [0, 0, 0]})
                env.data.qpos[2] = .5
                env.data.qpos[3:7] = [2**-.5, 2**-.5, 0, 0]
                mujoco.mj_forward(env.model, env.data)
                _, reward, terminated, _, info = env.step(np.zeros(12))
                self.assertTrue(terminated)
                self.assertEqual(info["termination_reason"], "fallen")
                if version != "walk_energy_v1":
                    self.assertEqual(reward, -25.)
                    self.assertEqual(info["running_reward"], 0.)
                else:
                    self.assertAlmostEqual(reward, env.dt*sum(info["reward_terms"].values())-2.)

    def test_energy_per_motion_handles_stalls_turns_and_standing(self):
        command = [.1, 0, 0]
        stopped = [0, 0, 0]
        for power in (0., 20., 200.):
            for velocity in (stopped, [-.1, 0, 0], [0, .1, 0]):
                self.assertEqual(energy_motion_score(command, velocity, stopped, power)[0], 0.)
        forward = energy_motion_score(command, command, stopped, 20.)[0]
        self.assertAlmostEqual(forward, np.exp(-1))
        self.assertEqual(forward, energy_motion_score(command, [.2, 0, 0], stopped, 20.)[0])
        self.assertGreater(forward, energy_motion_score(command, [.05, 0, 0], stopped, 20.)[0])
        self.assertGreater(forward, energy_motion_score(command, command, stopped, 40.)[0])
        self.assertEqual(forward, energy_motion_score([0, -.1, 0], [0, -.1, 0], stopped, 20.)[0])
        self.assertEqual(forward, energy_motion_score([0, 0, -.2], stopped, [0, 0, -.2], 20.)[0])
        self.assertEqual(energy_motion_score([0, 0, -.2], stopped, [0, 0, .2], 20.)[0], 0.)
        self.assertEqual(forward, energy_motion_score(stopped, stopped, stopped, 20.)[0])
        self.assertEqual(energy_motion_score(stopped, stopped, stopped, 0.)[0], 1.)

    def test_paper_reward_and_history_contract(self):
        terms = dict(tracking_xy=1., tracking_yaw=.5, energy_efficiency=.4, temperature=-2.)
        running, terminal = walking_reward(terms, .02, False, "walk_energy_v3")
        self.assertAlmostEqual(running, .02*1.9*np.exp(-2))
        self.assertEqual(terminal, 0.)
        self.assertEqual(walking_reward(terms, .02, True, "walk_energy_v3"), (0., -25.))
        for kind, history, expected in (("mlp", None, 30), ("lstm", None, 1),
                                         ("lstm", 30, 30), ("mlp", 4, 4), ("droq", None, 30)):
            config = WalkConfig(policy_kind=kind, history_frames=history, robustness="none")
            self.assertEqual(config.history_frames, expected)
            with make_walk_env(config) as env:
                obs, _ = env.reset(seed=42)
                self.assertEqual(obs.shape, (60*expected,))
                next_obs, _, _, _, info = env.step(np.zeros(12))
                if expected > 1:
                    np.testing.assert_array_equal(next_obs[:-60], obs[60:])
                self.assertNotIn("air_time", info["reward_terms"])
                self.assertNotIn("flight", info["reward_terms"])
                self.assertNotIn("unproductive_power", info["reward_terms"])
                reset_obs, _ = env.reset(seed=42)
                np.testing.assert_array_equal(reset_obs, obs)

    def test_velocity_and_gravity_observations_use_body_frame(self):
        with QuadBotWalkEnv(robustness="none") as env:
            env.reset(seed=1)
            env.data.qpos[3:7] = [2**-.5, 0, 0, 2**-.5]
            env.data.qvel[:] = 0
            env.data.qvel[:3] = [0, .2, 0]
            mujoco.mj_forward(env.model, env.data)
            np.testing.assert_allclose(env._info()["body_velocity_m_s"][:2], [.2, 0], atol=1e-8)
            np.testing.assert_allclose(env._observation()["projected_gravity"], [0, 0, -1], atol=1e-7)

    def test_success_rejects_stationary_hot_and_inefficient_policies(self):
        config = WalkConfig()
        command = np.array([.1, 0, 0])
        good = [[.01, .02]]*5
        self.assertTrue(walk_success(command, good, 10, 10, 10, 45, "", config))
        self.assertFalse(walk_success(command, [[.1, 0]]*5, 10, 10, 1, 20, "", config))
        self.assertFalse(walk_success(command, good, 10, 10, 21, 45, "", config))
        self.assertFalse(walk_success(command, good, 10, 10, 10, 51, "", config))
        self.assertFalse(walk_success(command, good, 1, 10, 10, 45, "fallen", config))
        hold = SimpleNamespace(num_timesteps=0, predict=lambda *a, **k: (np.zeros(12), None))
        progress = []
        report = evaluate_walk(hold, replace(config, robustness="none"), SIM_DIR / "motor_model.yaml", seconds=.04,
                               progress_callback=progress.append)
        self.assertEqual(progress[0]["completed"], 0)
        self.assertEqual(progress[-1]["completed"], 28)
        self.assertTrue(all(p["total"] == 28 for p in progress))
        self.assertEqual([p["completed"] for p in progress], sorted(p["completed"] for p in progress))
        self.assertGreater(progress[-1]["elapsed_seconds"], 0)
        cancelled = evaluate_walk(hold, config, SIM_DIR / "motor_model.yaml", seconds=.04,
                                  stop_requested=lambda: True)
        self.assertIsNone(cancelled)
        self.assertEqual(len(report["goals"]), 14)
        self.assertTrue(all(not r["policy"]["success"] for r in report["goals"] if r["goal"] != "stand"))

    def test_real_training_reload_resume_and_task_separation(self):
        torch.set_num_threads(1)
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "walk"
            config = WalkConfig(rollout_steps=64, episode_seconds=.4, evaluation_interval=0)
            train_walk(directory, 128, config)
            policy, saved, pointer = load_walk_policy(directory)
            self.assertEqual(saved, config)
            self.assertEqual(pointer["steps"], 128)
            self.assertGreater(policy._n_updates, 0)
            from stable_baselines3 import PPO
            initial = PPO.load(directory / "policy-000000000-000000.zip")
            self.assertTrue(any(not torch.equal(value, policy.policy.state_dict()[key])
                                for key, value in initial.policy.state_dict().items()))
            with make_walk_env(config, directory / "motor_model.yaml") as env:
                obs, _ = env.reset(seed=1)
                self.assertEqual(obs.shape, (60*config.history_frames,))
                again, _, _ = load_walk_policy(directory)
                np.testing.assert_array_equal(policy.predict(obs, deterministic=True)[0],
                                              again.predict(obs, deterministic=True)[0])
            with self.assertRaises(ValueError):
                read_manifest(directory)  # Pose training must reject this policy.
            train_walk(directory, 64, resume=True)
            _, _, pointer = load_walk_policy(directory)
            self.assertEqual(pointer["steps"], 192)
            self.assertEqual(json.loads((directory / "status.json").read_text())["state"], "complete")

            # Historical manifests omitted reward_version. Preserve their old
            # reward on resume instead of silently changing the saved task.
            manifest_path = directory / "config.json"
            manifest = json.loads(manifest_path.read_text())
            self.assertEqual(manifest["contract"], "walk_energy_v3")
            manifest["contract"] = "walk_energy_v1"
            with manifest_path.open("w") as stream:
                json.dump(manifest, stream)
            with self.assertRaisesRegex(ValueError, "reward version"):
                read_walk_manifest(directory)
            del manifest["training"]["reward_version"]
            manifest_path.write_text(json.dumps(manifest))
            legacy = read_walk_manifest(directory)
            self.assertEqual(legacy.reward_version, "walk_energy_v1")
            del manifest["training"]["history_frames"]
            manifest_path.write_text(json.dumps(manifest))
            self.assertEqual(read_walk_manifest(directory).history_frames, 4)
            manifest["training"]["history_frames"] = config.history_frames
            manifest_path.write_text(json.dumps(manifest))
            with make_walk_env(legacy) as env:
                self.assertEqual(env.unwrapped.walk_reward_version, "walk_energy_v1")
            train_walk(directory, 64, resume=True)
            _, saved, pointer = load_walk_policy(directory)
            self.assertEqual(saved.reward_version, "walk_energy_v1")
            self.assertEqual(pointer["steps"], 256)

    def test_unlimited_training_stops_saves_and_resumes(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "walk"
            config = WalkConfig(rollout_steps=64, episode_seconds=.4, evaluation_interval=0)

            def reached(steps):
                path = directory / "latest.json"
                return path.exists() and json.loads(path.read_text())["steps"] >= steps

            train_walk(directory, 0, config, stop_requested=lambda: reached(128))
            policy, _, pointer = load_walk_policy(directory)
            self.assertEqual(pointer["steps"], 128)
            self.assertGreater(policy._n_updates, 0)
            self.assertEqual(json.loads((directory / "status.json").read_text())["state"], "stopped")
            train_walk(directory, 0, resume=True, stop_requested=lambda: reached(192))
            _, _, pointer = load_walk_policy(directory)
            self.assertEqual(pointer["steps"], 192)
            self.assertEqual(json.loads((directory / "status.json").read_text())["state"], "stopped")
            with self.assertRaisesRegex(ValueError, "nonnegative"):
                train_walk(directory, -1, resume=True)

    def test_droq_walk_resume_preview_and_qualification(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "droq"
            config = WalkConfig(policy_kind="droq", history_frames=1, rollout_steps=64,
                                episode_seconds=.08, evaluation_interval=0,
                                droq_utd_ratio=1, droq_buffer_size=256, droq_learning_starts=0)
            train_walk(directory, 4, config)
            preview, saved, pointer = load_walk_policy(directory)
            self.assertEqual(saved, config)
            self.assertEqual(preview.replay_buffer.buffer_size, 1)
            self.assertEqual(pointer["steps"], 4)
            learner, _, _ = load_walk_policy(directory, for_training=True)
            self.assertEqual(learner.replay_buffer.size(), 4)
            self.assertEqual(learner._n_updates, 4)
            report = evaluate_walk(preview, config, directory / "motor_model.yaml", seconds=.04)
            self.assertEqual(len(report["goals"]), 14)
            train_walk(directory, 4, resume=True)
            learner, _, _ = load_walk_policy(directory, for_training=True)
            self.assertEqual(learner.num_timesteps, 8)
            self.assertEqual(learner.replay_buffer.size(), 8)
            self.assertEqual(learner.actor_updates, 8)


if __name__ == "__main__":
    unittest.main()
