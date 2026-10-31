"""Measure walking tracking, power and temperature, or watch a saved policy."""
import argparse
from contextlib import redirect_stdout
from dataclasses import replace
import json
import os
from pathlib import Path
import time
import sys

import mujoco
import numpy as np
import torch

from sim.walk_training import WalkPolicyPlayer, evaluate_walk, load_walk_policy, make_walk_env
from sim.training import StopRequest


def watch_policy(directory, config, policy, pointer, seconds, command, temperature, watch):
    """Keep each episode on one checkpoint; refresh only at episode boundaries."""
    with StopRequest() as stop, make_walk_env(replace(config, episode_seconds=seconds),
            directory / "motor_model.yaml", "human") as env:
        episode = 0
        while not stop.requested:
            episode += 1
            obs, _ = env.reset(seed=12345, options={"command": command,
                                                  "initial_temperature_c": temperature})
            player = WalkPolicyPlayer(policy)
            print(f"Viewer episode {episode}: checkpoint {pointer['steps']} steps, command {command}, "
                  f"warm start {temperature:g} C", flush=True)
            for _ in range(env.unwrapped.max_steps):
                started = time.monotonic()
                action = player.predict(obs)
                obs, _, terminated, truncated, info = env.step(action)
                viewer = env.unwrapped._viewer
                with viewer.lock():
                    viewer.cam.lookat[:] = env.unwrapped.data.body("base_link").xpos
                velocity = info["body_velocity_m_s"]
                viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                    f"Saved policy: {pointer['steps']:,} steps | episode {episode}\n"
                    f"Command: {command[0]:+.2f}, {command[1]:+.2f} m/s, {command[2]:+.2f} rad/s\n"
                    f"Speed: {velocity[0]:+.2f}, {velocity[1]:+.2f} m/s\n"
                    f"Motor temperature: {info['max_temperature_c']:.1f} C (peak {info['peak_temperature_c']:.1f})\n"
                    f"Copper heating: {info['copper_power_w']:.1f} W\n"
                    f"Mechanical effort: {info['positive_mechanical_power_w']:.1f} W\n"
                    f"Time: {info['time']:.1f} / {seconds:g} s", None))
                if not viewer.is_running() or stop.requested:
                    return
                if terminated or truncated:
                    break
                time.sleep(max(0, env.unwrapped.dt-(time.monotonic()-started)))
            print(json.dumps({k: info[k] for k in ("time", "velocity_error_m_s", "yaw_error_rad_s",
                              "copper_energy_j", "peak_temperature_c", "termination_reason")}), flush=True)
            if not watch:
                return
            policy, _, pointer = load_walk_policy(directory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--seconds", type=float, default=10.)
    parser.add_argument("--physics-device", choices=("cpu", "gpu"),
                        help="qualification backend; defaults to the run's training backend")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--watch", action="store_true", help="keep viewer open and refresh checkpoints between episodes")
    parser.add_argument("--command", type=float, nargs=3, default=[.1, 0, 0], metavar=("VX", "VY", "YAW"))
    parser.add_argument("--initial-temperature-c", type=float, default=40.)
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be finite and positive")
    if not np.isfinite(args.command).all() or not 20 <= args.initial_temperature_c <= 45:
        parser.error("command must be finite and initial temperature between 20 and 45 C")
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    torch.set_num_threads(1)
    policy, config, pointer = load_walk_policy(args.run)
    if args.physics_device:
        config = replace(config, physics_device=args.physics_device)
    motor_config = args.run / "motor_model.yaml"
    if not (args.viewer or args.watch):
        with redirect_stdout(sys.stderr):
            report = evaluate_walk(policy, config, motor_config, args.seconds)
        print(json.dumps(dict(checkpoint=pointer, **report), indent=2))
        return
    watch_policy(args.run, config, policy, pointer, args.seconds, args.command,
                 args.initial_temperature_c, args.watch)


if __name__ == "__main__":
    main()
