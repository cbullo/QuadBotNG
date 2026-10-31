"""Train PPO and watch its progress locally, or attach to an existing run."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="train without the Qt window")
    parser.add_argument("--run-dir", type=Path, help="new run directory (must not already exist)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--resume", type=Path, help="continue a saved run with its recorded settings")
    mode.add_argument("--watch", type=Path, help="view an existing run without starting a trainer")
    parser.add_argument("--steps", type=int, default=100000, help="additional training transitions")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--rollout-steps", type=int, default=512)
    parser.add_argument("--episode-seconds", type=float, default=10)
    parser.add_argument("--profile", choices=("research", "baseline"), default="research",
                        help="new-run recipe; resume always uses the saved configuration")
    parser.add_argument("--from-scratch", action="store_true", help="skip supervised pose initialization for a new run")
    parser.add_argument("--evaluation-interval", type=int, default=8192,
                        help="training steps between tracking evaluations; 0 disables them")
    parser.add_argument("--no-render", action="store_true", help="show curves without the robot preview")
    parser.add_argument("--exit-after", type=float, help="close the window after this many seconds")
    parser.add_argument("--screenshot-output", type=Path)
    parser.add_argument("--capture-at", type=float, default=10, help="seconds after window opens to capture a screenshot")
    from sim.training_compute import default_num_envs
    parser.add_argument("--num-envs", type=int, default=None,
                        help="parallel CPU physics workers; resume uses the saved count")
    args = parser.parse_args()
    if args.num_envs is None:
        args.num_envs = default_num_envs(headless=args.headless)
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.run_dir and (args.resume or args.watch):
        parser.error("--run-dir cannot be combined with --resume or --watch")
    if args.headless and args.watch:
        parser.error("--watch requires the window")
    if args.exit_after is not None and not 0 < args.exit_after < float("inf"):
        parser.error("--exit-after must be finite and positive")
    if not 0 < args.capture_at < float("inf"):
        parser.error("--capture-at must be finite and positive")
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    directory = args.resume or args.watch or args.run_dir or Path("runs") / datetime.now().strftime("body-pose-%Y%m%d-%H%M%S-%f")
    directory = directory.resolve()
    if not (args.resume or args.watch) and directory.exists():
        parser.error(f"run already exists: {directory}; use --resume or choose a new directory")
    if (args.resume or args.watch) and not (directory / "config.json").is_file():
        parser.error(f"missing run configuration: {directory}")
    # EGL keeps MuJoCo offscreen rendering independent of Qt's display context.
    if not args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from sim.training import StopRequest, TrainingConfig, train
    from dataclasses import replace
    config = TrainingConfig(seed=args.seed, rollout_steps=args.rollout_steps, num_envs=args.num_envs,
                            episode_seconds=args.episode_seconds,
                            initialization="random" if args.from_scratch else "pose_demonstrations_v1",
                            evaluation_interval=args.evaluation_interval)
    if args.profile == "baseline":
        config = replace(config, reward_version="dense_v3", policy_version="small_v1",
                         command_interval=0., robustness="none", evaluation_suite="legacy_v1")
    try:
        config.validate()
    except ValueError as error:
        parser.error(str(error))
    print(f"Training run: {directory}", flush=True)
    if args.headless:
        with StopRequest() as stop:
            train(directory, args.steps, config, resume=bool(args.resume),
                  stop_requested=lambda: stop.requested)
        return 0
    from sim.training_view import run_window
    return run_window(directory, args)


if __name__ == "__main__":
    sys.exit(main())
