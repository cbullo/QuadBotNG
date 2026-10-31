"""Measure real robot rollouts and PPO separately, excluding warm-up and GUI."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import time

import numpy as np
from stable_baselines3 import PPO
import torch

from sim.robot_model import SIM_DIR
from sim.training import TrainingConfig, make_env
from sim.training_compute import training_envs
from sim.walk_training import WalkConfig, make_walk_env


class TimedPPO(PPO):
    def collect_rollouts(self, *args, **kwargs):
        started = time.perf_counter()
        result = super().collect_rollouts(*args, **kwargs)
        self.rollout_seconds += time.perf_counter()-started
        return result

    def train(self):
        started = time.perf_counter()
        super().train()
        self.update_seconds += time.perf_counter()-started


def benchmark(task, workers, updates, rollout_steps, physics_device="cpu"):
    config = (WalkConfig(evaluation_interval=0) if task == "walk" else
              TrainingConfig(evaluation_interval=0, initialization="random"))
    config = replace(config, num_envs=workers, rollout_steps=rollout_steps)
    if task == "walk":
        config = replace(config, physics_device=physics_device)
    config.validate()
    started = time.perf_counter()
    with training_envs(make_walk_env if task == "walk" else make_env, config,
                       SIM_DIR / "motor_model.yaml") as env:
        model = TimedPPO("MlpPolicy", env, device="cpu", seed=config.seed,
                         n_steps=rollout_steps // workers, batch_size=64, n_epochs=5,
                         learning_rate=3e-4 if task == "walk" else config.learning_rate,
                         target_kl=.01, gamma=.99, gae_lambda=.95,
                         policy_kwargs=dict(net_arch=[128, 128, 128], activation_fn=torch.nn.ELU,
                            log_std_init=np.log(.5/4) if task == "walk" else -2-np.log(config.action_scale)))
        model.rollout_seconds = model.update_seconds = 0.
        model.learn(rollout_steps)  # Reset, optimizer initialization, and one warm-up rollout.
        startup = time.perf_counter()-started
        model.rollout_seconds = model.update_seconds = 0.
        completed_episodes = 0

        def count_episodes(local, _global):
            nonlocal completed_episodes
            completed_episodes += int(np.count_nonzero(local["dones"]))
            return True

        started = time.perf_counter()
        model.learn(rollout_steps*updates, reset_num_timesteps=False, callback=count_episodes)
        elapsed = time.perf_counter()-started
        return dict(task=task, physics_device=physics_device, num_envs=workers, transitions=rollout_steps*updates,
                    steps_per_env=rollout_steps//workers, startup_and_warmup_s=startup,
                    completed_episodes=completed_episodes,
                    elapsed_s=elapsed, rollout_s=model.rollout_seconds, ppo_update_s=model.update_seconds,
                    transitions_per_second=rollout_steps*updates/elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("walk", "pose", "both"), default="both")
    parser.add_argument("--workers", nargs="+", type=int, default=[1, 2, 4, 8])
    parser.add_argument("--updates", type=int, default=5)
    parser.add_argument("--rollout-steps", type=int, default=512)
    parser.add_argument("--physics-device", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.physics_device == "gpu" and args.task != "walk":
        parser.error("GPU physics currently supports --task=walk only")
    if args.updates < 1:
        parser.error("updates must be positive")
    torch.set_num_threads(1)
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    rows = []
    for task in (["walk", "pose"] if args.task == "both" else [args.task]):
        for workers in args.workers:
            row = benchmark(task, workers, args.updates, args.rollout_steps, args.physics_device)
            rows.append(row)
            print(json.dumps(row), flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(dict(torch=str(torch.__version__),
            torch_cuda_available=torch.cuda.is_available(), cpu_count=os.cpu_count(), results=rows), indent=2)+"\n")


if __name__ == "__main__":
    main()
