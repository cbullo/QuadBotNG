"""CPU rollout workers or optional batched GPU walking physics; CPU PPO."""
from contextlib import contextmanager
from functools import partial
import os

import gymnasium as gym
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnv


def default_num_envs(headless=False):
    # Leave CPU capacity for Qt, preview physics, and the desktop.
    cpus = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)
    limit = min(8 if headless else 4, max(1, cpus // 2))
    # Power-of-two counts divide the supported 64-transition rollout multiples.
    return 1 << (limit.bit_length() - 1)


def validate_compute(num_envs, rollout_steps):
    if num_envs < 1 or rollout_steps % num_envs or rollout_steps // num_envs < 2:
        raise ValueError("num_envs must divide rollout_steps and leave at least 2 steps per environment")


class TrainingAccess(gym.Wrapper):
    """Small explicit RPC surface; never transfer MuJoCo model/data over a pipe."""
    def training_snapshot(self):
        base = self.unwrapped
        return dict(time=float(base.data.time), qpos=base.data.qpos.tolist(),
                    qvel=base.data.qvel.tolist(), command=base.command.tolist(),
                    target_position=base.target_position.tolist(),
                    target_quaternion=base.target_quaternion.tolist(),
                    foot_anchors=base.foot_anchors.tolist())

    def set_training_scale(self, name, value):
        if name not in ("goal_scale", "command_scale") or not 0 < value <= 1:
            raise ValueError("invalid training curriculum")
        setattr(self.unwrapped, name, value)


def _worker(factory, config, motor_config):
    return TrainingAccess(factory(config, motor_config))


@contextmanager
def training_envs(factory, config, motor_config):
    validate_compute(config.num_envs, config.rollout_steps)
    # Spawned interpreters import NumPy/Torch: prevent nested thread pools in each.
    keys = ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")
    previous = {key: os.environ.get(key) for key in keys}
    try:
        for key in keys:
            os.environ[key] = "1"
        factories = [partial(_worker, factory, config, motor_config) for _ in range(config.num_envs)]
        if getattr(config, "physics_device", "cpu") == "gpu":
            from sim.gpu_walk import GpuWalkVecEnv
            env = GpuWalkVecEnv(factories)
        else:
            env = (DummyVecEnv(factories) if config.num_envs == 1 else
                   SubprocVecEnv(factories, start_method="spawn"))
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    try:
        env.seed(config.seed)  # SB3 assigns seed + worker index at the next reset.
        yield env
    finally:
        env.close()


def set_training_scale(env, name, value):
    if isinstance(env, VecEnv):
        env.env_method("set_training_scale", name, value)
    else:
        setattr(env.unwrapped, name, value)


def compute_layout(env):
    return dict(policy_device="cpu", physics_device=getattr(env, "physics_device", "cpu"), num_envs=env.num_envs,
                torch_threads=1, vector_backend=type(env).__name__)
