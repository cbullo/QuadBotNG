"""Commanded body pose with planted feet, using the QuadBot differential model.

Import this module to register QuadBot-BodyPose-v0. See docs/gym.md for the
action, observation and reward contracts and the ideal sensor limitations.
"""

from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from sim.gym_motor import VoltageMotorPlant
from sim.goals import GOAL_SETS, WORKSPACE_ACTION_SCALE, WORKSPACE_MIN_BASE_HEIGHT
from sim.imu import read_imu
from sim.robot_model import LEGS, closure_errors, load_model, set_pose, stand_angles


def command_quaternion(rpy):
    """Active Rz(yaw) Ry(pitch) Rx(roll), quaternion in MuJoCo wxyz order."""
    r, p, y = np.asarray(rpy) / 2
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


class QuadBotBodyPoseEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, episode_seconds=10.0,
                 command_range=(.01, .01, .01, .06, .06, .06),
                 action_scale=WORKSPACE_ACTION_SCALE, motor_config=None, goal_mode="mixed",
                 reward_version="tracking_v2", min_base_height=WORKSPACE_MIN_BASE_HEIGHT,
                 action_filter_seconds=0.0, target_rate_limit=0.0, goal_scale=1.0,
                 robustness="none", stance_bend=.09):
        if render_mode not in (None, *self.metadata["render_modes"]):
            raise ValueError("unsupported render_mode")
        if not np.isfinite(episode_seconds) or episode_seconds <= 0:
            raise ValueError("episode_seconds must be positive")
        if not np.isfinite(action_scale) or action_scale <= 0:
            raise ValueError("action_scale must be positive")
        if not np.isfinite(min_base_height) or min_base_height <= 0:
            raise ValueError("min_base_height must be positive")
        self.min_base_height = float(min_base_height)
        if (not np.isfinite([action_filter_seconds, target_rate_limit, goal_scale]).all()
                or min(action_filter_seconds, target_rate_limit) < 0 or not 0 < goal_scale <= 1):
            raise ValueError("invalid action filter, target rate or goal scale")
        self.action_filter_seconds = float(action_filter_seconds)
        self.target_rate_limit = float(target_rate_limit)
        self.goal_scale = float(goal_scale)
        self.command_range = self._vector(command_range, "command_range")
        if goal_mode not in (*GOAL_SETS, "mixed"):
            raise ValueError("goal_mode must be workspace, poses, axis or mixed")
        self.goal_mode = goal_mode
        if reward_version not in ("legacy_v1", "tracking_v2", "dense_v3", "planted_v4"):
            raise ValueError("unknown reward_version")
        self.reward_version = reward_version
        if robustness not in ("none", "mild_v1"):
            raise ValueError("unknown robustness profile")
        self.robustness = robustness
        if not np.isfinite(stance_bend) or not 0 <= stance_bend <= .5:
            raise ValueError("stance_bend must be between zero and .5 radians")
        self.stance_bend = float(stance_bend)
        if np.any(self.command_range < 0):
            raise ValueError("command_range must be nonnegative")
        self.render_mode = render_mode
        self.model = load_model()
        self.data = mujoco.MjData(self.model)
        self.frame_skip = 40  # 50 Hz policy, 2 kHz physics, 1 kHz rotor PD.
        self.dt = self.frame_skip * self.model.opt.timestep
        self.max_steps = int(np.ceil(episode_seconds / self.dt))
        self.action_scale = float(action_scale)
        self.motor_names = tuple(self.model.actuator(i).name for i in range(self.model.nu))
        expected = tuple(f"motor_{leg}{axis}" for leg in LEGS for axis in "IOZ")
        if self.motor_names != expected:
            raise ValueError("unexpected differential motor order")
        self.plant = VoltageMotorPlant(self.model.actuator_ctrlrange[:, 1],
                                       Path(motor_config) if motor_config else None)
        self.foot_ids = np.array([self.model.geom(f"arm_lower_{p['cad']}_1_collision_1").id
                                  for p in LEGS.values()])
        self.ground = self.model.geom("ground").id
        self._nominal_friction = self.model.geom_friction.copy()
        self._nominal_gains = (self.plant.kp, self.plant.kd)
        self.action_space = spaces.Box(-1, 1, (12,), dtype=np.float32)
        self.observation_space = spaces.Dict({
            "command": spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32),
            "motor_position": spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32),
            "motor_velocity": spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32),
            "imu": spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32),
            "motor_temperature": spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32),
            "previous_action": spaces.Box(-1, 1, (12,), dtype=np.float32),
        })
        self._renderer = self._viewer = self._viewer_context = None
        self._needs_reset = True

    @staticmethod
    def _vector(value, name):
        result = np.asarray(value, dtype=float)
        if result.shape != (6,) or not np.isfinite(result).all():
            raise ValueError(f"{name} must contain six finite values")
        return result.copy()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        if set(options) - {"command"}:
            raise ValueError("supported reset option: command=[dx,dy,dz,roll,pitch,yaw]")
        if "command" in options:
            command = self._vector(options["command"], "command")
        elif self.goal_mode in GOAL_SETS:
            goals = GOAL_SETS[self.goal_mode]
            command = self.goal_scale * np.array(goals[self.np_random.integers(len(goals))][1])
        else:
            command = self.np_random.uniform(-self.command_range, self.command_range)
        # Always restore nominal values: randomization must not compound across
        # resets. Feet have higher contact priority than the floor in this model.
        self.model.geom_friction[:] = self._nominal_friction
        self.plant.kp, self.plant.kd = self._nominal_gains
        if self.robustness == "mild_v1":
            self.model.geom_friction[np.r_[self.foot_ids, self.ground], 0] *= self.np_random.uniform(.8, 1.2)
            self.plant.kp *= self.np_random.uniform(.9, 1.1)
            self.plant.kd *= self.np_random.uniform(.9, 1.1)
        # Leave extension headroom. Passive angles are reset consistently by
        # set_pose; noise on independent qpos would break the four-bar closure.
        poses = {leg: np.array(stand_angles(leg)) + [self.stance_bend, -self.stance_bend, 0] for leg in LEGS}
        set_pose(self.model, self.data, poses)
        feet = self.data.geom_xpos[self.foot_ids]
        clearance = np.min(feet[:, 2] - self.model.geom_size[self.foot_ids, 0])
        self.data.qpos[2] += .0005 - clearance
        mujoco.mj_forward(self.model, self.data)
        self.nominal_motor_position = self.data.actuator_length.copy()
        self.plant.reset()
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.steps = 0
        self._needs_reset = True
        # A physical settling interval, with no pinning or restored support.
        # Motor heat incurred here remains part of this episode's state.
        for _ in range(25):
            self._advance(self.nominal_motor_position)
        contacts, body_contact = self._contacts()
        if (not contacts.all() or body_contact or np.any(self.data.warning.number)
                or self.data.qpos[2] < .12 or np.max(self.plant.temperature) > 55):
            raise RuntimeError("could not establish a healthy four-foot starting stance")
        self.reference_position = self.data.body("base_link").xpos.copy()
        self.reference_quaternion = self.data.body("base_link").xquat.copy()
        self.reference_rotation = self.data.body("base_link").xmat.reshape(3, 3).copy()
        self.foot_anchors = self.data.geom_xpos[self.foot_ids].copy()
        self.data.time = 0.0  # Episode clock starts after the settling interval.
        self._needs_reset = False
        self.set_command(command)
        if self.render_mode == "human":
            self.render()
        return self._observation(), self._info()

    def set_command(self, command):
        """Change the requested pose, keeping the original stance/foot anchors.

        Translation (metres) is in the reset chassis frame. Rotation (radians)
        is relative to the reset chassis orientation, using Rz Ry Rx.
        """
        if self._needs_reset:
            raise gym.error.ResetNeeded("call reset before setting a command")
        self.command = self._vector(command, "command")
        self.target_position = self.reference_position + self.reference_rotation @ self.command[:3]
        self.target_quaternion = np.empty(4)
        mujoco.mju_mulQuat(self.target_quaternion, self.reference_quaternion,
                           command_quaternion(self.command[3:]))

    def _advance(self, target):
        self.copper_energy_j = self.positive_work_j = self.torque_squared_integral = 0.0
        self.absolute_work_j = 0.0
        for i in range(self.frame_skip):
            # Split stepping refreshes rotor position/velocity before applying
            # voltage and back-EMF. It preserves the generated implicitfast solver.
            mujoco.mj_step1(self.model, self.data)
            if i % 2 == 0:
                self.voltage = self.plant.voltage(target, self.data.actuator_length,
                                                  self.data.actuator_velocity)
            self.data.ctrl[:] = self.plant.torque(self.voltage, self.data.actuator_velocity)
            dt = self.model.opt.timestep
            self.copper_energy_j += float(np.sum(self.plant.copper_loss(self.data.ctrl))) * dt
            self.positive_work_j += float(np.sum(np.maximum(self.data.ctrl * self.data.actuator_velocity, 0))) * dt
            self.absolute_work_j += float(np.sum(np.abs(self.data.ctrl * self.data.actuator_velocity))) * dt
            self.torque_squared_integral += float(np.mean(self.data.ctrl**2)) * dt
            self.plant.heat(self.data.ctrl, self.model.opt.timestep)
            mujoco.mj_step2(self.model, self.data)
            if np.any(self.data.warning.number) or np.max(self.plant.temperature) > 55:
                break
        mujoco.mj_forward(self.model, self.data)

    def _contacts(self):
        # Read the native contact arrays once instead of crossing Python/C for
        # every contact. This matters with hundreds of worlds per policy step.
        geoms = self.data.contact.geom
        touching = (self.data.contact.dist <= 0) & np.any(geoms == self.ground, axis=1)
        geoms = geoms[touching]
        others = np.where(geoms[:, 0] == self.ground, geoms[:, 1], geoms[:, 0])
        matches = others[:, None] == self.foot_ids[None, :]
        return np.any(matches, axis=0), bool(np.any(~np.any(matches, axis=1)))

    def _observation(self):
        imu = read_imu(self.model, self.data)
        result = {key: np.asarray(value, dtype=np.float32).copy() for key, value in {
            "command": self.command,
            "motor_position": self.data.actuator_length - self.nominal_motor_position,
            "motor_velocity": self.data.actuator_velocity,
            "imu": np.concatenate([imu.angular_velocity_rad_s, imu.specific_force_m_s2]),
            "motor_temperature": self.plant.temperature,
            "previous_action": self.previous_action,
        }.items()}
        if self.robustness == "mild_v1":
            # Assumed uniform noise bounds in rotor radians, rad/s and SI IMU
            # units. These are training variations, not measured sensor specs.
            for key, scale in (("motor_position", .01), ("motor_velocity", .1),
                               ("imu", np.array([.01]*3 + [.05]*3))):
                result[key] += self.np_random.uniform(-1, 1, result[key].shape) * scale
        return result

    def _soft_limit_cost(self):
        """Penalize approach to actual joint AND differential tendon limits."""
        costs = []
        for values, ranges, limited in (
            (self.data.qpos[self.model.jnt_qposadr], self.model.jnt_range, self.model.jnt_limited),
            (self.data.ten_length, self.model.tendon_range, self.model.tendon_limited),
        ):
            mask = limited.astype(bool)
            low, high = ranges[mask].T
            half = (high - low) / 2
            excess = np.maximum(np.abs(values[mask] - (high + low)/2) / half - .95, 0)
            costs.extend((excess / .05)**2)
        return float(np.mean(costs)) if costs else 0.0

    def _info(self):
        rotation_error = np.empty(3)
        mujoco.mju_subQuat(rotation_error, self.target_quaternion, self.data.body("base_link").xquat)
        feet, body_contact = self._contacts()
        return {
            "time": float(self.data.time),
            "position_error_m": float(np.linalg.norm(self.target_position - self.data.body("base_link").xpos)),
            "orientation_error_rad": float(np.linalg.norm(rotation_error)),
            "foot_displacement_m": np.linalg.norm(self.data.geom_xpos[self.foot_ids] - self.foot_anchors, axis=1),
            "foot_contacts": feet,
            "body_contact": body_contact,
            "max_temperature_c": float(np.max(self.plant.temperature)),
            "max_closure_error_m": max(closure_errors(self.model, self.data).values()),
        }

    def step(self, action):
        if self._needs_reset:
            raise gym.error.ResetNeeded("call reset before stepping a new episode")
        action = np.asarray(action, dtype=float)
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError("action must contain twelve finite values")
        action = np.clip(action, -1, 1)
        # Filter in target space, with a rate bound in physical rotor rad/s.
        # previous_action exposes the applied target, making filter state visible
        # to the policy. A zero filter/rate preserves historical checkpoints.
        delta = (action - self.previous_action) * self.dt / (self.dt + self.action_filter_seconds)
        if self.target_rate_limit:
            limit = self.target_rate_limit * self.dt / self.action_scale
            delta = np.clip(delta, -limit, limit)
        action = self.previous_action + delta
        start_time = self.data.time
        self._advance(self.nominal_motor_position + self.action_scale * action)
        self.steps += 1
        if (np.any(self.data.warning.number) or not np.isfinite(self.data.qpos).all()
                or not np.isfinite(self.data.qvel).all()):
            self._needs_reset = True
            raise RuntimeError("MuJoCo numerical failure; discard this rollout and reset")
        info = self._info()
        reason = ""
        if info["max_temperature_c"] > 55:
            reason = "over_temperature"
        elif (self.data.qpos[2] < self.min_base_height or self.data.body("base_link").xmat[8] < .5
              or info["body_contact"]):
            reason = "fallen"
        elif np.max(info["foot_displacement_m"]) > .05:
            reason = "feet_displaced"
        elif info["max_closure_error_m"] > .005:
            reason = "linkage_error"
        # Pose tracking counts only while feet remain at their original anchors
        # and in contact. Translation/rotation are not independent foot targets.
        planted = float(np.exp(-np.mean(info["foot_displacement_m"]**2) / .005**2)
                        * np.all(info["foot_contacts"]))
        terms = {
            "position": 2 * np.exp(-(info["position_error_m"] / .02)**2) * planted,
            "orientation": 2 * np.exp(-(info["orientation_error_rad"] / .1)**2) * planted,
            "feet": planted,
            "effort": -.01 * float(np.mean((self.voltage / self.plant.voltage_limit)**2)),
            "action_change": -.02 * float(np.mean((action - self.previous_action)**2)),
            "motion": -.02 * float(np.sum(self.data.qvel[:6]**2)),
        }
        if self.reward_version == "tracking_v2":
            # The previous 20 mm / 0.1 rad tolerances paid 94–98% of the
            # maximum for ignoring our 5–8 mm / 0.03–0.04 rad commands.
            # Couple translation and rotation so an uncommanded, already
            # correct axis cannot hide failure on the commanded axis.
            error2 = (info["position_error_m"] / .003)**2 + (info["orientation_error_rad"] / .01)**2
            terms.pop("position")
            terms.pop("orientation")
            terms["tracking"] = 4.5 * np.exp(-.5 * error2) * planted
            terms["feet"] = .5 * planted
        elif self.reward_version == "dense_v3":
            # A rational distance score has a usable slope far from the goal.
            # Keep pose and contact/anchor scores separate: losing one contact
            # must not erase every indication of progress toward the goal.
            error = info["position_error_m"] / .02 + info["orientation_error_rad"] / .1
            terms.pop("position")
            terms.pop("orientation")
            terms["tracking"] = 4.0 / (1 + error)
            terms["feet"] = .5 * float(np.mean(info["foot_contacts"]))
            terms["foot_drift"] = -2.0 * float(np.tanh(np.mean(info["foot_displacement_m"]) / .005))
        elif self.reward_version == "planted_v4":
            # Normalize by the curriculum's success tolerances, so ignoring
            # small commands cannot earn almost the full tracking reward.
            p = max(.002, .25 * np.linalg.norm(self.command[:3]))
            r = max(.008, .25 * np.linalg.norm(self.command[3:]))
            error2 = (info["position_error_m"] / p)**2 + (info["orientation_error_rad"] / r)**2
            terms = {
                "tracking": 4.0 / (1.0 + error2),
                "feet": .5 * float(np.mean(info["foot_contacts"])),
                "foot_drift": -2.0 * float(np.tanh(np.max(info["foot_displacement_m"]) / .005)),
                "effort": terms["effort"],
                # Use physical rotor target rate, not a scale-dependent action
                # difference. Respect the existing voltage/thermal plant.
                "action_rate": -.02 * float(np.mean(((action-self.previous_action)
                                                      * self.action_scale / self.dt / 6)**2)),
                "motion": terms["motion"],
                "soft_limits": -2.0 * self._soft_limit_cost(),
            }
        reward = float((self.data.time - start_time) * sum(terms.values()) - bool(reason))
        self.previous_action = action.astype(np.float32)
        terminated = bool(reason)
        truncated = self.steps >= self.max_steps
        self._needs_reset = terminated or truncated
        info.update(reward_terms={key: float(value) for key, value in terms.items()},
                    termination_reason=reason)
        if self.render_mode == "human":
            self.render()
        return self._observation(), reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            if self._viewer is None:
                from sim.controller_viewer import joined_passive_viewer
                self._viewer_context = joined_passive_viewer(self.model, self.data)
                self._viewer = self._viewer_context.__enter__()
                self._viewer.cam.distance = .9
                self._viewer.cam.lookat[:] = [0, 0, .15]
            self._viewer.sync()
        elif self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = .9, 135, -25
            camera.lookat[:] = [0, 0, .15]
            self._renderer.update_scene(self.data, camera=camera)
            return self._renderer.render().copy()

    def close(self):
        if self._viewer_context is not None:
            self._viewer_context.__exit__(None, None, None)
            self._viewer_context = self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


gym.register(id="QuadBot-BodyPose-v0", entry_point="sim.gym_env:QuadBotBodyPoseEnv")
