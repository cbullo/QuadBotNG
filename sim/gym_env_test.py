"""Training API, deterministic physics, sensor observations and task semantics."""

import unittest

import gymnasium as gym
from gymnasium.utils.env_checker import check_env
import mujoco
import numpy as np

from sim.gym_env import QuadBotBodyPoseEnv, command_quaternion
from sim.gym_motor import VoltageMotorPlant
from sim.goals import BODY_POSE_GOALS, POSE_BODY_POSE_GOALS
from sim.imu import read_imu
from sim.robot_model import LEGS, active_joints, motor_matrix


def anchored_pose(env):
    """Offline geometric reference only; never used by training or playback."""
    scratch = mujoco.MjData(env.model)
    scratch.qpos[:] = env.data.qpos
    scratch.qpos[:3] = env.target_position
    scratch.qpos[3:7] = env.target_quaternion
    active = [env.model.joint(joint).qposadr[0] for leg in LEGS for joint in active_joints(leg)]
    passive = [[env.model.joint(f"{role}_{p['cad']}").qposadr[0]
                for role in ("main_to_lower", "crank_to_link")] for p in LEGS.values()]
    def feet(x):
        scratch.qpos[active] = x
        for i, pair in enumerate(passive):
            scratch.qpos[pair] = x[3*i+1] - x[3*i]
        mujoco.mj_forward(env.model, scratch)
        return scratch.geom_xpos[env.foot_ids].ravel().copy()
    x = env.data.qpos[active].copy()
    target = env.foot_anchors.ravel()
    for _ in range(35):
        current = feet(x)
        jac = np.column_stack([(feet(x + np.eye(12)[i]*1e-6) - current)/1e-6 for i in range(12)])
        x += np.clip(np.linalg.lstsq(jac, target-current, rcond=None)[0], -.05, .05)
    np.testing.assert_allclose(feet(x), target, atol=1e-7)
    action = (scratch.actuator_length - env.nominal_motor_position) / env.action_scale
    return scratch, action


class GymEnvironmentTest(unittest.TestCase):
    def test_contact_arrays_match_contact_loop(self):
        with QuadBotBodyPoseEnv() as env:
            env.reset(seed=42)
            for height in (env.data.qpos[2], .5, .04):
                env.data.qpos[2] = height
                mujoco.mj_forward(env.model, env.data)
                feet, body = np.zeros(4, dtype=bool), False
                for contact in env.data.contact:
                    if env.ground not in contact.geom or contact.dist > 0:
                        continue
                    other = contact.geom[1] if contact.geom[0] == env.ground else contact.geom[0]
                    matches = env.foot_ids == other
                    feet |= matches
                    body |= not matches.any()
                actual_feet, actual_body = env._contacts()
                np.testing.assert_array_equal(actual_feet, feet)
                self.assertEqual(actual_body, body)

    def test_gym_contract_and_repeatability(self):
        with gym.make("QuadBot-BodyPose-v0") as env:
            check_env(env.unwrapped, skip_render_check=True)
            action = np.linspace(-.02, .02, 12, dtype=np.float32)
            traces = []
            for _ in range(2):
                initial, info = env.reset(seed=14)
                self.assertTrue(info["foot_contacts"].all())
                self.assertTrue(env.observation_space.contains(initial))
                trace = []
                for _ in range(10):
                    obs, reward, terminated, truncated, info = env.step(action)
                    self.assertTrue(env.observation_space.contains(obs))
                    self.assertFalse(terminated or truncated)
                    trace.append(np.r_[*[obs[key] for key in sorted(obs)], reward,
                                       env.unwrapped.data.qpos.copy()])
                traces.append(trace)
            np.testing.assert_array_equal(*traces)
            self.assertAlmostEqual(info["time"], .2)

    def test_command_reference_and_sensor_observation(self):
        with QuadBotBodyPoseEnv() as env:
            obs, _ = env.reset(seed=1, options={"command": [0]*6})
            anchors = env.foot_anchors.copy()
            reference = env.reference_position.copy()
            command = [.01, -.005, .008, .03, -.04, .05]
            env.set_command(command)
            np.testing.assert_array_equal(env.foot_anchors, anchors)
            np.testing.assert_allclose(env.target_position, reference + env.reference_rotation @ command[:3])
            expected = np.empty(4)
            mujoco.mju_mulQuat(expected, env.reference_quaternion, command_quaternion(command[3:]))
            np.testing.assert_allclose(env.target_quaternion, expected)
            obs, _, _, _, _ = env.step(np.zeros(12))
            np.testing.assert_allclose(obs["command"], command)
            imu = read_imu(env.model, env.data)
            np.testing.assert_allclose(obs["imu"], np.r_[imu.angular_velocity_rad_s, imu.specific_force_m_s2], atol=1e-6)
            # Ground truth appears in scoring only, not in the sensor observation.
            self.assertEqual(set(obs), {"command", "motor_position", "motor_velocity", "imu", "motor_temperature", "previous_action"})
            np.testing.assert_allclose(command_quaternion([np.pi/2, 0, 0]), [2**-.5, 2**-.5, 0, 0])

    def test_nominal_hold_and_differential(self):
        with QuadBotBodyPoseEnv() as env:
            env.reset(seed=1, options={"command": [0]*6})
            for _ in range(50):
                _, reward, terminated, truncated, info = env.step(np.zeros(12))
                self.assertFalse(terminated or truncated, info)
                self.assertGreater(reward, 0)
                self.assertLessEqual(np.max(np.abs(env.data.ctrl)), .2941995)
            self.assertLess(max(info["foot_displacement_m"]), .005)
            self.assertLess(info["max_closure_error_m"], .001)
            for leg in LEGS:
                ids = [env.model.actuator(f"motor_{leg}{axis}").id for axis in "IOZ"]
                angles = [env.data.qpos[env.model.joint(joint).qposadr[0]] for joint in active_joints(leg)]
                np.testing.assert_allclose(env.data.actuator_length[ids], motor_matrix(leg) @ angles, atol=1e-10)

    def test_termination_truncation_and_reset(self):
        with QuadBotBodyPoseEnv(episode_seconds=.04) as env:
            env.reset(seed=1)
            self.assertFalse(env.step(np.zeros(12))[3])
            _, _, terminated, truncated, _ = env.step(np.zeros(12))
            self.assertFalse(terminated)
            self.assertTrue(truncated)
            with self.assertRaises(gym.error.ResetNeeded):
                env.step(np.zeros(12))
            env.reset(seed=1)
            env.plant.temperature[:] = 56
            _, reward, terminated, _, info = env.step(np.zeros(12))
            self.assertTrue(terminated)
            self.assertEqual(info["termination_reason"], "over_temperature")
            self.assertLess(reward, 0)
            env.reset(seed=1)
            self.assertLess(np.max(env.plant.temperature), 30)
            # Simulate a foot losing its original anchor; commands never move it.
            env.foot_anchors[0, 0] += .06
            _, _, terminated, _, info = env.step(np.zeros(12))
            self.assertTrue(terminated)
            self.assertEqual(info["termination_reason"], "feet_displaced")

    def test_vector_instances_and_invalid_actions(self):
        env = gym.vector.SyncVectorEnv([QuadBotBodyPoseEnv, QuadBotBodyPoseEnv])
        try:
            obs, _ = env.reset(seed=12)
            self.assertEqual(obs["imu"].shape, (2, 6))
            self.assertFalse(np.array_equal(obs["command"][0], obs["command"][1]))
            env.step(np.zeros((2, 12), dtype=np.float32))
            self.assertIsNot(env.envs[0].data, env.envs[1].data)
        finally:
            env.close()
        with QuadBotBodyPoseEnv() as env:
            env.reset(seed=1)
            for action in ([0]*3, [np.nan]*12, [np.inf]*12):
                with self.assertRaises(ValueError):
                    env.step(action)
            with self.assertRaises(ValueError):
                env.set_command([np.nan]*6)

    def test_motor_physics_matches_native_reference_cases(self):
        plant = VoltageMotorPlant(np.full(12, .2941995))
        np.testing.assert_allclose(plant.torque(-4, 0), .0420285, atol=1e-12)
        np.testing.assert_allclose(plant.torque(4, 0), -.0420285, atol=1e-12)
        np.testing.assert_allclose(plant.torque(-100, 0), .2941995, atol=1e-12)
        self.assertTrue(np.all(plant.torque(0, 1) < 0))
        plant.heat(np.full(12, .0420285), 30)
        np.testing.assert_allclose(plant.temperature, 20 + 1.2*8*(1-np.exp(-30/160)), atol=1e-12)

    def test_smoothed_targets_and_dense_far_goal_reward(self):
        with QuadBotBodyPoseEnv(action_filter_seconds=.15, target_rate_limit=6,
                                reward_version="dense_v3", goal_scale=.1,
                                goal_mode="workspace") as env:
            obs, _ = env.reset(seed=1)
            self.assertLessEqual(abs(obs["command"][2]), .0151)
            env.reset(options={"command": [0, 0, -.15, 0, 0, 0]})
            obs, _, _, _, info = env.step(np.ones(12))
            # A full-range request must not jump all twelve radians at once.
            np.testing.assert_allclose(obs["previous_action"] * env.action_scale, .12, atol=1e-7)
            previous = obs["previous_action"].copy()
            obs, _, _, _, _ = env.step(-np.ones(12))
            self.assertLessEqual(np.max(np.abs(obs["previous_action"]-previous))*env.action_scale, .120001)
            self.assertGreater(info["reward_terms"]["tracking"], .4)
            # The same pose error was effectively a zero signal in tracking_v2.
            self.assertEqual(float(np.exp(-.5*(.15/.003)**2)), 0)
            obs, _ = env.reset()
            np.testing.assert_array_equal(obs["previous_action"], np.zeros(12))

    def test_randomization_is_seeded_bounded_and_never_compounds(self):
        with QuadBotBodyPoseEnv(robustness="mild_v1") as env:
            traces = []
            nominal = env.model.geom_friction.copy()
            for _ in range(2):
                obs, _ = env.reset(seed=7, options={"command": [0]*6})
                friction = env.model.geom_friction.copy()
                self.assertFalse(np.array_equal(friction[env.foot_ids], nominal[env.foot_ids]))
                ratios = friction[env.foot_ids, 0] / nominal[env.foot_ids, 0]
                self.assertTrue(np.all((ratios >= .8) & (ratios <= 1.2)))
                true_position = env.data.actuator_length - env.nominal_motor_position
                self.assertLessEqual(np.max(np.abs(obs["motor_position"] - true_position)), .010001)
                self.assertGreater(np.max(np.abs(obs["motor_position"] - true_position)), 0)
                np.testing.assert_array_equal(obs["previous_action"], np.zeros(12))
                traces.append(np.r_[friction.ravel(), env.plant.kp, env.plant.kd,
                                    obs["motor_position"], env.step(np.zeros(12))[0]["imu"]])
            np.testing.assert_array_equal(*traces)
            env.robustness = "none"
            obs, _ = env.reset(seed=7)
            np.testing.assert_array_equal(env.model.geom_friction, nominal)
            self.assertEqual((env.plant.kp, env.plant.kd), env._nominal_gains)

    def test_planted_reward_tracks_small_goals_and_coupled_limits(self):
        with QuadBotBodyPoseEnv(reward_version="planted_v4") as env:
            command = [0, 0, -.008, 0, 0, 0]
            env.reset(seed=1, options={"command": command})
            hold = sum(env.step(np.zeros(12))[1] for _ in range(75))
            env.reset(seed=1, options={"command": command})
            _, action = anchored_pose(env)
            track = 0
            for i in range(75):
                _, reward, terminated, _, info = env.step(action * min(1, (i+1)/50))
                self.assertFalse(terminated)
                track += reward
            self.assertGreater(track, hold * 1.5)
            self.assertLess(max(info["foot_displacement_m"]), .005)
            # Test the nonlinear task's coupled mechanical limits directly.
            # A tendon limit must be priced even if individual hinges are legal.
            costs = env._soft_limit_cost()
            limited = np.flatnonzero(env.model.tendon_limited)
            self.assertGreater(len(limited), 0)
            tendon = limited[0]
            env.data.ten_length[tendon] = env.model.tendon_range[tendon, 1] + .1
            self.assertGreater(env._soft_limit_cost(), costs)

    def test_commanded_pose_has_motor_authority(self):
        # Offline kinematic feasibility diagnostic, never a policy/controller:
        # solve the anchored-foot geometry, then apply only its rotor targets
        # through the real voltage plant. No simulation pose edits during step.
        with QuadBotBodyPoseEnv(action_scale=1, min_base_height=.12) as env:
            for name, command in POSE_BODY_POSE_GOALS:
                with self.subTest(goal=name):
                    env.reset(seed=1, options={"command": command})
                    holding_return = sum(env.step(np.zeros(12))[1] for _ in range(75))
                    env.reset(seed=1, options={"command": command})
                    _, action = anchored_pose(env)
                    self.assertLessEqual(np.max(np.abs(action)), 1.0)
                    tracking_return = 0
                    for step in range(75):
                        _, reward, terminated, _, info = env.step(action * min(1, (step+1)/50))
                        tracking_return += reward
                        self.assertFalse(terminated, info)
                    self.assertLess(info["position_error_m"], .003)
                    self.assertLess(info["orientation_error_rad"], .02)
                    self.assertLess(np.max(info["foot_displacement_m"]), .005)
                    self.assertGreater(tracking_return, holding_return * 1.5,
                                       f"{name}: tracking must clearly beat ignoring the goal")

    def test_workspace_goals_respect_geometry_and_physical_limits(self):
        with QuadBotBodyPoseEnv() as env:
            for name, command in BODY_POSE_GOALS:
                with self.subTest(goal=name):
                    env.reset(seed=1, options={"command": command})
                    scratch, action = anchored_pose(env)
                    self.assertLessEqual(np.max(np.abs(action)), 1)
                    for i in range(env.model.njnt):
                        if env.model.jnt_limited[i]:
                            value = scratch.qpos[env.model.jnt_qposadr[i]]
                            low, high = env.model.jnt_range[i]
                            self.assertTrue(low - 1e-6 <= value <= high + 1e-6,
                                            (env.model.joint(i).name, value, low, high))
                    for i in range(env.model.ntendon):
                        if env.model.tendon_limited[i]:
                            low, high = env.model.tendon_range[i]
                            self.assertTrue(low - 1e-6 <= scratch.ten_length[i] <= high + 1e-6,
                                            env.model.tendon(i).name)
                    self.assertGreater(min((c.dist for c in scratch.contact), default=0), -.001)
                    # Check physical travel as well as endpoint geometry.
                    # This reference permits the measured rolling of rounded
                    # feet; it does not claim the 5 mm tracking task is solved.
                    for step in range(150):
                        _, _, terminated, _, info = env.step(action * min(1, (step+1)/100))
                        self.assertFalse(terminated, info)
                        self.assertTrue(info["foot_contacts"].all())
                        self.assertLess(max(info["foot_displacement_m"]), .025)
                    self.assertLess(info["position_error_m"], .025)
                    self.assertLess(info["orientation_error_rad"], .04)

    def test_deep_crouch_and_return_through_voltage_plant(self):
        # This is a reference motor trajectory, not a learned policy. Rounded
        # rigid feet roll during a deep crouch: record/bound that motion rather
        # than pretending the old 5 mm planted-foot objective has been solved.
        with QuadBotBodyPoseEnv() as env:
            env.reset(seed=1, options={"command": (0, 0, -.15, 0, 0, 0)})
            initial_height = env.data.qpos[2]
            anchors = env.foot_anchors.copy()
            _, action = anchored_pose(env)
            self.assertGreater(np.max(np.abs(action)) * env.action_scale, 9)
            for step in range(150):
                _, _, terminated, _, info = env.step(action * min(1, (step+1)/100))
                self.assertFalse(terminated, info)
                self.assertTrue(info["foot_contacts"].all())
                self.assertLess(max(info["foot_displacement_m"]), .025)
            self.assertAlmostEqual(initial_height - env.data.qpos[2], .15, delta=.01)
            self.assertLess(env.data.qpos[2], .09)
            self.assertFalse(info["body_contact"])
            np.testing.assert_array_equal(env.foot_anchors, anchors)
            for step in range(125):
                _, _, terminated, _, info = env.step(action * max(0, 1-(step+1)/100))
                self.assertFalse(terminated, info)
            self.assertAlmostEqual(env.data.qpos[2], initial_height, delta=.003)


if __name__ == "__main__":
    unittest.main()
