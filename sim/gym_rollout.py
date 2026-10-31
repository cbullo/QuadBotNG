"""Exercise the body-pose Gym environment; zero/random actions are not a policy."""

import argparse
import os
from pathlib import Path
import time

import gymnasium as gym
import numpy as np

import sim.gym_env  # Register QuadBot-BodyPose-v0.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render-output", type=Path, help="save the final RGB frame as PNG")
    parser.add_argument("--random-actions", action="store_true")
    parser.add_argument("--command", nargs=6, type=float, default=[0]*6,
                        metavar=("DX", "DY", "DZ", "ROLL", "PITCH", "YAW"))
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.viewer and args.render_output:
        parser.error("choose --viewer or --render-output")
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    mode = "human" if args.viewer else "rgb_array" if args.render_output else None
    with gym.make("QuadBot-BodyPose-v0", render_mode=mode) as env:
        env.action_space.seed(args.seed)
        _, info = env.reset(seed=args.seed, options={"command": args.command})
        total = 0
        episodes = 0
        started = time.monotonic()
        for step in range(args.steps):
            tick = time.monotonic()
            action = env.action_space.sample() if args.random_actions else np.zeros(12, dtype=np.float32)
            _, reward, terminated, truncated, info = env.step(action)
            total += reward
            if args.viewer:
                time.sleep(max(0, env.unwrapped.dt - (time.monotonic() - tick)))
                if not env.unwrapped._viewer.is_running():
                    break
            if terminated or truncated:
                episodes += 1
                print(f"Episode ended: {info['termination_reason'] or 'time_limit'} at {info['time']:.2f}s")
                if step + 1 < args.steps:
                    env.reset(options={"command": args.command})
        print(f"{step+1} steps in {time.monotonic()-started:.2f}s; ended episodes={episodes}, return={total:.3f}")
        print(f"Position error={info['position_error_m']:.4f} m; rotation error={info['orientation_error_rad']:.4f} rad; "
              f"max foot drift={max(info['foot_displacement_m']):.4f} m; "
              f"temperature={info['max_temperature_c']:.1f} C")
        if args.render_output:
            from PIL import Image
            args.render_output.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(env.render()).save(args.render_output)
            print(f"Saved {args.render_output}")


if __name__ == "__main__":
    main()
