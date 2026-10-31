"""Velocity-command walking through QuadBot's coupled voltage motor plant.

Independent from the planted-foot task: feet can step, but sliding, motor
heating and effort are costly. See docs/walking.md for the exact contract.
"""
import gymnasium as gym
import mujoco
import numpy as np

from sim.gym_env import QuadBotBodyPoseEnv

WALK_REWARD_VERSIONS = ("walk_energy_v1", "walk_energy_v2", "walk_energy_v3")


def energy_motion_score(command, velocity, angular_velocity, power_w):
    """Bounded energy-per-useful-motion score, adapted from Liang et al. (2025).

    The paper uses mechanical power and absolute chassis speeds. Here power
    includes copper loss, and only commanded progress counts (also sideways).
    Cap progress at the target so overspeed cannot buy an efficiency bonus.
    Scales: 200 J/m, 100 J/rad; neutral commands instead use a 20 W budget.
    These are starting reward scales, not measured electrical efficiencies.
    """
    speed = float(np.linalg.norm(command[:2]))
    translation = rotation = 0.
    moving = speed > .03 or abs(command[2]) > .1
    if speed > .03:
        translation = float(np.clip(np.dot(velocity[:2], command[:2])/speed, 0., speed))
    if abs(command[2]) > .1:
        rotation = float(np.clip(angular_velocity[2]*np.sign(command[2]), 0., abs(command[2])))
    scale_w = 200.*translation + 100.*rotation if moving else 20.
    # A stopped robot given a motion command earns no efficiency bonus, even
    # when it consumes zero power. No singular CoT or arbitrary epsilon reward.
    score = float(np.exp(-max(0., power_w)/scale_w)) if scale_w > 0. else 0.
    return score, translation, rotation


def motion_shortfall(command, velocity, angular_velocity):
    """Unfulfilled commanded chassis motion, not rotor work or joint shaking.

    Standing still is appropriate for neutral commands. Match the existing
    moving-command deadbands; combine translation and turning when both apply.
    """
    progress = []
    if np.linalg.norm(command[:2]) > .03:
        progress.append(np.clip(np.dot(velocity[:2], command[:2]) / np.dot(command[:2], command[:2]), 0., 1.))
    if abs(command[2]) > .1:
        progress.append(np.clip(angular_velocity[2] / command[2], 0., 1.))
    return 1.-float(np.mean(progress)) if progress else 0.


def walking_reward(terms, elapsed, terminated, version):
    raw = float(elapsed * sum(terms.values()))
    if version == "walk_energy_v1":
        return raw, -2. if terminated else 0.
    # A positive, continuously cost-sensitive score avoids both early death to
    # escape costs and a flat zero reward when an inefficient policy starts out.
    benefit = sum(max(value, 0.) for value in terms.values())
    cost = -sum(min(value, 0.) for value in terms.values())
    if version == "walk_energy_v3":
        return (0. if terminated else float(elapsed*benefit*np.exp(-cost))), (-25. if terminated else 0.)
    return (0. if terminated else float(elapsed * benefit / (1.+cost))), (-25. if terminated else 0.)


class QuadBotWalkEnv(QuadBotBodyPoseEnv):
    def __init__(self, render_mode=None, episode_seconds=20., motor_config=None,
                 command_scale=.5, robustness="mild_v1", resample_seconds=4.,
                 reward_version="walk_energy_v3", energy_weight=1.):
        if reward_version not in WALK_REWARD_VERSIONS:
            raise ValueError("unknown walking reward version")
        self.walk_reward_version = reward_version
        if not np.isfinite(energy_weight) or energy_weight < 0:
            raise ValueError("energy_weight must be finite and nonnegative")
        self.energy_weight = float(energy_weight)
        if not np.isfinite(command_scale) or not 0 < command_scale <= 1:
            raise ValueError("command_scale must be in (0, 1]")
        if not np.isfinite(resample_seconds) or resample_seconds <= 0:
            raise ValueError("resample_seconds must be positive")
        super().__init__(render_mode=render_mode, episode_seconds=episode_seconds,
                         action_scale=4., motor_config=motor_config, goal_mode="mixed",
                         action_filter_seconds=.04, target_rate_limit=20.,
                         min_base_height=.08, robustness=robustness, stance_bend=.3)
        self.command_scale = command_scale
        self.command_steps = max(1, round(resample_seconds / self.dt))
        self.velocity_command = np.zeros(3)
        self.nominal_height = .2
        self.observation_space["command"] = gym.spaces.Box(-np.inf, np.inf, (3,), np.float32)
        # Ideal attitude estimate, explicitly separate from raw accelerometer.
        # A hardware attitude estimator is required before deploying this actor.
        self.observation_space["projected_gravity"] = gym.spaces.Box(-1, 1, (3,), np.float32)

    def set_velocity_command(self, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("walk command must be [vx m/s, vy m/s, yaw rad/s]")
        self.velocity_command = command.copy()

    def _sample_command(self):
        command = self.np_random.uniform([-.15, -.1, -.6], [.3, .1, .6]) * self.command_scale
        if self.np_random.random() < .2:
            command[:] = 0
        self.set_velocity_command(command)

    def reset(self, *, seed=None, options=None):
        options = options or {}
        if set(options) - {"command", "initial_temperature_c"}:
            raise ValueError("walk reset options: command, initial_temperature_c")
        # Initialize this environment's RNG before choosing warm-start heat.
        gym.Env.reset(self, seed=seed)
        temperature = float(options.get("initial_temperature_c",
                            self.np_random.uniform(20., 40.) if self.robustness != "none" else 20.))
        if not np.isfinite(temperature) or not 20 <= temperature <= 45:
            raise ValueError("initial_temperature_c must be between 20 and 45 C")
        self.plant.initial = temperature
        self.velocity_command = np.zeros(3)
        # Base reset performs closure-consistent placement and physical settling.
        # Its six-component pose command is always zero and is never our task goal.
        super().reset(seed=None, options={"command": np.zeros(6)})
        self.nominal_height = float(self.data.qpos[2])
        self.fixed_command = "command" in options
        if self.fixed_command:
            self.set_velocity_command(options["command"])
        else:
            self._sample_command()
        self.air_time = np.zeros(4)
        self.last_contacts = self._contacts()[0]
        self.total_copper_j = self.total_work_j = 0.0
        self.total_absolute_work_j = 0.0
        self.peak_temperature = float(max(self.plant.temperature))
        return self._observation(), self._info()

    def _observation(self):
        obs = super()._observation()
        obs["command"] = self.velocity_command.astype(np.float32).copy()
        rotation = self.data.body("base_link").xmat.reshape(3, 3)
        obs["projected_gravity"] = (-rotation[2, :]).astype(np.float32).copy()
        return obs

    def _info(self):
        info = super()._info()
        velocity = np.zeros(6)
        # BODY uses the inertial frame, whose principal axes differ from the
        # chassis axes in our CAD model. The IMU site is at the chassis origin.
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE,
                                self.model.site("imu").id, velocity, 1)
        info.update(body_velocity_m_s=velocity[3:].copy(), body_angular_velocity_rad_s=velocity[:3].copy(),
                    command=self.velocity_command.copy(),
                    velocity_error_m_s=float(np.linalg.norm(velocity[3:5]-self.velocity_command[:2])),
                    yaw_error_rad_s=float(abs(velocity[2]-self.velocity_command[2])))
        return info

    def applied_action(self, action):
        """Pure target filtering, shared by scalar and batched physics backends."""
        action = np.asarray(action, dtype=float)
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError("action must contain twelve finite values")
        delta = (np.clip(action, -1, 1)-self.previous_action) * self.dt / (self.dt+self.action_filter_seconds)
        limit = self.target_rate_limit * self.dt / self.action_scale
        return self.previous_action + np.clip(delta, -limit, limit)

    def step(self, action):
        if self._needs_reset:
            raise gym.error.ResetNeeded("call reset before stepping a new episode")
        applied = self.applied_action(action)
        start = self.data.time
        self._advance(self.nominal_motor_position + self.action_scale * applied)
        elapsed = self.data.time-start
        self.steps += 1
        if (np.any(self.data.warning.number) or not np.isfinite(self.data.qpos).all()
                or not np.isfinite(self.data.qvel).all()):
            self._needs_reset = True
            raise RuntimeError("MuJoCo numerical failure; discard rollout and reset")
        info = self._info()
        reason = ""
        if info["max_temperature_c"] > 55:
            reason = "over_temperature"
        elif (self.data.qpos[2] < max(.08, .55*self.nominal_height)
              or self.data.body("base_link").xmat[8] < .5 or info["body_contact"]):
            reason = "fallen"
        elif info["max_closure_error_m"] > .005:
            reason = "linkage_error"
        contacts = info["foot_contacts"]
        self.air_time += elapsed
        # Reward touchdown once, using a one-step contact filter against chatter.
        filtered = contacts | self.last_contacts
        touchdown = (self.air_time > elapsed) & filtered
        moving = np.linalg.norm(self.velocity_command[:2]) > .03 or abs(self.velocity_command[2]) > .1
        slip = 0.0
        for foot, contact in zip(self.foot_ids, contacts):
            if contact:
                v = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_GEOM, int(foot), v, 0)
                slip += float(np.sum(v[3:5]**2))
        copper_power = self.copper_energy_j / elapsed
        work_power = self.positive_work_j / elapsed
        absolute_work_power = self.absolute_work_j / elapsed
        torque_rms = np.sqrt(self.torque_squared_integral / elapsed)
        velocity, angular = info["body_velocity_m_s"], info["body_angular_velocity_rad_s"]
        gravity = -self.data.body("base_link").xmat.reshape(3, 3)[2, :]
        terms = {
            "tracking_xy": 1.5 * np.exp(-(info["velocity_error_m_s"]/.12)**2),
            "tracking_yaw": .5 * np.exp(-(info["yaw_error_rad_s"]/.4)**2),
            "copper_heat": -.5 * copper_power / 20.,
            "mechanical_effort": -.05 * work_power / 10.,
            "torque": -.05 * (torque_rms / float(max(self.plant.limits)))**2,
            "temperature": -float(np.mean((np.maximum(self.plant.temperature-30., 0)/20.)**2)),
            "upright": -2. * float(np.sum(gravity[:2]**2)),
            "height": -.5 * ((self.data.qpos[2]-self.nominal_height)/.05)**2,
            "vertical_motion": -float(velocity[2]**2),
            "roll_pitch_motion": -.05 * float(np.sum(angular[:2]**2)),
            "slip": -.2 * slip / .2**2,
            "air_time": .2 * float(np.sum(np.clip(self.air_time-.1, 0, .3)*touchdown)) * moving,
            "flight": -.5 * float(not np.any(contacts)),
            "action_rate": -.02 * float(np.mean(((applied-self.previous_action)
                                                  * self.action_scale / self.dt / 20.)**2)),
            "soft_limits": -2. * self._soft_limit_cost(),
            "stand_still": -.1 * float(np.mean((self.data.actuator_length-self.nominal_motor_position)**2))
                           * (not moving),
        }
        if self.walk_reward_version == "walk_energy_v2":
            terms["copper_heat"] *= 2.
            terms["temperature"] *= 2.
            # Penalize stalled/sideways/wrong-direction energy use. Do not use
            # positive rotor work as a discount: oscillating the joints would
            # then evade this cost without transporting the chassis.
            warmth = 1.+float(np.mean(np.maximum(self.plant.temperature-30., 0.)/20.))
            terms["unproductive_power"] = (-copper_power/20. * warmth
                * motion_shortfall(self.velocity_command, velocity, angular))
        elif self.walk_reward_version == "walk_energy_v3":
            # Eq. (2): (tracking + alpha * energy) * exp(-auxiliary_cost).
            # Copper and absolute shaft work now enter the bounded energy score
            # together, rather than stacking unbounded running power penalties.
            for name in ("copper_heat", "mechanical_effort", "torque", "air_time", "flight"):
                del terms[name]
            terms["tracking_xy"] /= 1.5
            terms["temperature"] *= 2.
            energy_score, useful_speed, useful_yaw = energy_motion_score(
                self.velocity_command, velocity, angular, copper_power+absolute_work_power)
            terms["energy_efficiency"] = self.energy_weight*energy_score
            info.update(energy_efficiency_score=energy_score, useful_speed_m_s=useful_speed,
                        useful_yaw_rad_s=useful_yaw)
        self.air_time *= ~filtered
        self.last_contacts = contacts.copy()
        self.previous_action = applied.astype(np.float32)
        self.total_copper_j += self.copper_energy_j
        self.total_work_j += self.positive_work_j
        self.total_absolute_work_j += self.absolute_work_j
        self.peak_temperature = max(self.peak_temperature, info["max_temperature_c"])
        terminated, truncated = bool(reason), self.steps >= self.max_steps
        self._needs_reset = terminated or truncated
        info.update(reward_terms={k: float(v) for k, v in terms.items()}, termination_reason=reason,
                    copper_power_w=copper_power, positive_mechanical_power_w=work_power,
                    torque_rms_nm=float(torque_rms), copper_energy_j=self.total_copper_j,
                    positive_mechanical_work_j=self.total_work_j, peak_temperature_c=self.peak_temperature)
        info.update(absolute_mechanical_power_w=absolute_work_power,
                    absolute_mechanical_work_j=self.total_absolute_work_j,
                    energy_proxy_power_w=copper_power+absolute_work_power)
        raw_running_reward = float(elapsed * sum(terms.values()))
        running_reward, termination_reward = walking_reward(terms, elapsed, terminated, self.walk_reward_version)
        reward = running_reward + termination_reward
        info.update(raw_running_reward=raw_running_reward, running_reward=running_reward,
                    termination_reward=termination_reward)
        if not self.fixed_command and not self._needs_reset and self.steps % self.command_steps == 0:
            self._sample_command()
        if self.render_mode == "human":
            self.render()
        return self._observation(), reward, terminated, truncated, info


gym.register(id="QuadBot-Walk-v0", entry_point="sim.walk_env:QuadBotWalkEnv")
