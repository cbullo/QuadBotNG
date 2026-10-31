"""MJWarp walking physics with CPU Gym rewards, observations, resets and PPO.

Imported only for an explicitly requested CUDA backend. The existing Gym step
remains the reward/termination contract; only its 40-substep advance is replaced.
"""
from functools import partial
import os

import mujoco
import mujoco_warp as mjw
import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv
import warp as wp

if os.environ.get("QUADBOT_WARP_CACHE"):
    wp.config.kernel_cache_dir = os.environ["QUADBOT_WARP_CACHE"]


@wp.kernel(enable_backward=False)
def motor_step(length: wp.array2d[float], velocity: wp.array2d[float],
               target: wp.array2d[float], voltage: wp.array2d[float], ctrl: wp.array2d[float],
               temperature: wp.array2d[wp.float64], energy: wp.array3d[wp.float64],
               limits: wp.array[float], params: wp.array2d[wp.float64],
               halted: wp.array[int], substep: int):
    w, a = wp.tid()
    if halted[w] != 0:
        return
    vmax, sign = float(params[w, 0]), float(params[w, 1])
    ke, resistance = float(params[w, 2]), float(params[w, 3])
    if substep % 2 == 0:
        voltage[w, a] = wp.clamp(sign * (float(params[w, 4]) * (target[w, a]-length[w, a])
                                - float(params[w, 5])*velocity[w, a]), -vmax, vmax)
    current = (0.5*sign*voltage[w, a] - ke*velocity[w, a])/resistance
    torque = wp.clamp(1.5*ke*current, -limits[a], limits[a])
    ctrl[w, a] = torque
    torque64 = wp.float64(torque)
    iq = torque64/(wp.float64(1.5)*params[w, 2])
    copper = wp.float64(1.5)*params[w, 3]*iq*iq
    temperature[w, a] += (params[w, 6] + copper*params[w, 7] - temperature[w, a])*params[w, 8]
    energy[w, a, 0] += copper*params[w, 9]
    energy[w, a, 1] += wp.max(torque64*wp.float64(velocity[w, a]), wp.float64(0.0))*params[w, 9]
    energy[w, a, 2] += torque64*torque64*params[w, 9]/wp.float64(12.0)
    energy[w, a, 3] += wp.abs(torque64*wp.float64(velocity[w, a]))*params[w, 9]


@wp.kernel(enable_backward=False)
def thermal_stop(qpos: wp.array2d[float], qvel: wp.array2d[float], warm: wp.array2d[float],
                 saved_qpos: wp.array2d[float], saved_qvel: wp.array2d[float], saved_warm: wp.array2d[float],
                 temperature: wp.array2d[wp.float64], halted: wp.array[int], counts: wp.array[int]):
    w = wp.tid()
    # Stop each world at its first over-temperature substep, exactly like CPU.
    # MJWarp still solves halted worlds, but their integration state is restored
    # and their motor energy/heat no longer advances. They reset after this frame.
    if halted[w] != 0:
        for j in range(qpos.shape[1]):
            qpos[w, j] = saved_qpos[w, j]
        for j in range(qvel.shape[1]):
            qvel[w, j] = saved_qvel[w, j]
            warm[w, j] = saved_warm[w, j]
    else:
        counts[w] += 1
        for j in range(qpos.shape[1]):
            saved_qpos[w, j] = qpos[w, j]
        for j in range(qvel.shape[1]):
            saved_qvel[w, j] = qvel[w, j]
            saved_warm[w, j] = warm[w, j]
        for a in range(12):
            if temperature[w, a] > wp.float64(55.0):
                halted[w] = 1


class BatchedMotorPhysics:
    def __init__(self, bases):
        self.bases = bases
        self.n = len(bases)
        base = bases[0]
        self.model = mjw.put_model(base.model, batch_sizes={"geom_friction": self.n})
        self.data = mjw.put_data(base.model, base.data, nworld=self.n, nconmax=128, njmax=256)
        self.target = wp.zeros((self.n, 12), dtype=float)
        self.voltage = wp.zeros_like(self.target)
        self.temperature = wp.zeros((self.n, 12), dtype=wp.float64)
        self.energy = wp.zeros((self.n, 12, 4), dtype=wp.float64)
        self.params = wp.zeros((self.n, 10), dtype=wp.float64)
        self.limits = wp.array(base.plant.limits, dtype=float)
        self.halted = wp.zeros(self.n, dtype=int)
        self.inactive = wp.zeros(self.n, dtype=int)
        self.counts = wp.zeros(self.n, dtype=int)
        self.saved_qpos = wp.zeros_like(self.data.qpos)
        self.saved_qvel = wp.zeros_like(self.data.qvel)
        self.saved_warm = wp.zeros_like(self.data.qacc_warmstart)
        self.reset_mask = wp.zeros(self.n, dtype=bool)
        self.upload(np.ones(self.n, dtype=bool))
        self.target.assign(np.array([b.nominal_motor_position for b in bases], dtype=np.float32))
        print(f"Compiling GPU physics for {self.n} worlds (first launch can take several minutes)...", flush=True)
        self.advance()  # Compile before capture; these warm-up states are discarded.
        wp.synchronize()
        self.upload(np.ones(self.n, dtype=bool))
        with wp.ScopedCapture() as capture:
            self.advance()
        self.graph = capture.graph
        self.outputs = {name: getattr(self.data, name) for name in ("qpos", "qvel", "qacc_warmstart", "ctrl", "overflow")}
        self.outputs.update(temperature=self.temperature, energy=self.energy, counts=self.counts)
        self.host = {name: wp.empty_like(value, device="cpu", pinned=True) for name, value in self.outputs.items()}
        print("GPU physics ready; rewards and PPO run on CPU.", flush=True)

    def upload(self, mask):
        """Sync settled/reset worlds; preserve all other integration states."""
        self.reset_mask.assign(mask)
        mjw.reset_data(self.model, self.data, self.reset_mask)
        def assign_reset(destination, values):
            # Keep untouched GPU warm starts exactly: a CPU forward solve may
            # produce slightly different accelerations in the CPU mirrors.
            if not mask.all():
                current = destination.numpy()
                current[mask] = values[mask]
                values = current
            destination.assign(values)

        for name in ("qpos", "qvel", "qacc_warmstart", "ctrl"):
            assign_reset(getattr(self.data, name), np.array([getattr(b.data, name) for b in self.bases], dtype=np.float32))
        assign_reset(self.data.time, np.array([b.data.time for b in self.bases], dtype=np.float32))
        assign_reset(self.temperature, np.array([b.plant.temperature for b in self.bases]))
        self.model.geom_friction.assign(np.array([b.model.geom_friction for b in self.bases], dtype=np.float32))
        values = []
        for b in self.bases:
            p, dt = b.plant, b.model.opt.timestep
            values.append([p.voltage_limit, p.sign, p.ke, p.resistance, p.kp, p.kd,
                           p.ambient, p.thermal_resistance, -np.expm1(-dt/p.capacity/p.thermal_resistance), dt])
        self.params.assign(np.array(values, dtype=np.float64))

    def advance(self):
        self.energy.zero_()
        wp.copy(self.halted, self.inactive)
        self.counts.zero_()
        for i in range(self.bases[0].frame_skip):
            mjw.step1(self.model, self.data)
            wp.launch(motor_step, dim=(self.n, 12), inputs=[self.data.actuator_length,
                self.data.actuator_velocity, self.target, self.voltage, self.data.ctrl,
                self.temperature, self.energy, self.limits, self.params, self.halted, i])
            mjw.step2(self.model, self.data)
            wp.launch(thermal_stop, dim=self.n, inputs=[self.data.qpos, self.data.qvel,
                self.data.qacc_warmstart, self.saved_qpos, self.saved_qvel, self.saved_warm,
                self.temperature, self.halted, self.counts])

    def step(self, targets, active=None):
        if active is None:
            self.inactive.zero_()
        else:
            self.inactive.assign(np.asarray(~active, dtype=np.int32))
        self.target.assign(np.asarray(targets, dtype=np.float32))
        wp.capture_launch(self.graph)
        for name, value in self.outputs.items():
            wp.copy(self.host[name], value)
        wp.synchronize_device()
        result = {name: value.numpy() for name, value in self.host.items()}
        if np.any(result["overflow"]) or not all(np.isfinite(v).all() for v in result.values()):
            raise RuntimeError("GPU physics overflow or nonfinite state; discard rollout")
        return result


class GpuWalkVecEnv(DummyVecEnv):
    """SB3 vector API: batched CUDA physics, unchanged per-world Gym semantics."""
    physics_device = "gpu"

    def __init__(self, factories, auto_reset=True):
        wp.init()
        if not wp.is_cuda_available():
            raise RuntimeError("GPU walking requires a CUDA-capable NVIDIA GPU; use --physics-device=cpu")
        self.device = wp.get_device("cuda:0")
        self.physics = None
        self.result = None
        super().__init__(factories)
        self.auto_reset = auto_reset
        self.finished = np.zeros(self.num_envs, dtype=bool)
        self.final_infos = [{} for _ in self.envs]
        self.bases = [env.unwrapped for env in self.envs]
        self.cpu_advances = [base._advance for base in self.bases]
        self.reset_worlds = np.zeros(self.num_envs, dtype=bool)
        for i, base in enumerate(self.bases):
            base._advance = partial(self._advance_world, i, self.cpu_advances[i])

    def _advance_world(self, i, cpu_advance, target):
        b = self.bases[i]
        if b._needs_reset:
            # Keep the existing physical settling, randomization, and seed order.
            self.reset_worlds[i] = True
            return cpu_advance(target)
        r = self.result
        for name in ("qpos", "qvel", "qacc_warmstart", "ctrl"):
            getattr(b.data, name)[:] = r[name][i]
        b.data.time += int(r["counts"][i]) * b.model.opt.timestep
        b.plant.temperature[:] = r["temperature"][i]
        (b.copper_energy_j, b.positive_work_j, b.torque_squared_integral,
         b.absolute_work_j) = r["energy"][i].sum(axis=0)
        # CPU forward supplies the exact existing contact/IMU/reward interface.
        mujoco.mj_forward(b.model, b.data)

    def reset(self):
        self.finished[:] = False
        observations = super().reset()
        with wp.ScopedDevice(self.device):
            if self.physics is None:
                self.physics = BatchedMotorPhysics(self.bases)
            else:
                self.physics.upload(np.ones(self.num_envs, dtype=bool))
        self.reset_worlds[:] = False
        return observations

    def step_wait(self):
        targets = [b.nominal_motor_position + b.action_scale*b.applied_action(a)
                   for b, a in zip(self.bases, self.actions)]
        with wp.ScopedDevice(self.device):
            self.result = self.physics.step(targets, active=~self.finished)
            # This walking environment creates fresh info dictionaries/arrays
            # each step. Preserve those directly rather than recursively copying
            # every reward term for every world as generic DummyVecEnv does.
            infos = []
            for i, env in enumerate(self.envs):
                if self.finished[i]:
                    self.buf_rews[i] = 0.
                    infos.append(self.final_infos[i])
                    continue
                obs, self.buf_rews[i], terminated, truncated, info = env.step(self.actions[i])
                self.buf_dones[i] = terminated or truncated
                info["TimeLimit.truncated"] = truncated and not terminated
                if self.buf_dones[i]:
                    info["terminal_observation"] = obs
                    if self.auto_reset:
                        obs, self.reset_infos[i] = env.reset()
                    else:
                        self.finished[i] = True
                        self.final_infos[i] = info
                self._save_obs(i, obs)
                infos.append(info)
            if self.reset_worlds.any():
                self.physics.upload(self.reset_worlds)
                self.reset_worlds[:] = False
        return self._obs_from_buf(), self.buf_rews.copy(), self.buf_dones.copy(), infos

    def close(self):
        for base, advance in zip(self.bases, self.cpu_advances):
            base._advance = advance
        super().close()
        self.result = self.physics = None
