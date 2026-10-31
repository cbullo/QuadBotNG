"""Train the separate temperature/effort-aware walking policy on the PC."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--run-dir", type=Path)
    group.add_argument("--resume", type=Path)
    group.add_argument("--watch", type=Path, help="open an existing run without starting or owning its trainer")
    parser.add_argument("--headless", action="store_true", help="train without the live window")
    parser.add_argument("--no-render", action="store_true", help="show metrics without the robot image")
    parser.add_argument("--exit-after", type=float, help="close the window after this many seconds")
    parser.add_argument("--screenshot-output", type=Path)
    parser.add_argument("--capture-at", type=float, default=10.)
    parser.add_argument("--steps", type=int, default=100000,
                        help="additional training transitions (default: 100000); 0 runs until stopped")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--policy", choices=("mlp", "lstm", "droq"), default="mlp",
                        help="new run: PPO history MLP, recurrent PPO LSTM, or DroQ/SAC MLP; resume keeps saved policy")
    parser.add_argument("--history-frames", type=int,
                        help="input frames including current: default 30 for MLP/DroQ, 1 for LSTM; resume keeps saved history")
    parser.add_argument("--rollout-steps", type=int, default=None,
                        help="PPO rollout size; for DroQ, transitions between checkpoints (default 4096)")
    parser.add_argument("--droq-utd-ratio", type=int, default=20,
                        help="DroQ critic updates per new transition (default 20; smaller values reduce CPU work)")
    parser.add_argument("--droq-buffer-size", type=int, default=50000,
                        help="DroQ replay capacity in transitions across all worlds (default 50000)")
    parser.add_argument("--droq-learning-starts", type=int, default=1000,
                        help="DroQ random-action warm-up transitions before learning (default 1000)")
    parser.add_argument("--physics-device", choices=("cpu", "gpu"), default="cpu",
                        help="GPU uses batched MJWarp physics and CPU learning; resume uses saved settings")
    parser.add_argument("--episode-seconds", type=float, default=20.)
    parser.add_argument("--evaluation-interval", type=int, default=8192)
    parser.add_argument("--max-copper-power-w", type=float, default=20.)
    parser.add_argument("--max-temperature-c", type=float, default=50.)
    parser.add_argument("--energy-weight", type=float, default=1.,
                        help="new-run energy-efficiency reward weight (default: 1); resume keeps saved settings")
    from sim.training_compute import default_num_envs
    parser.add_argument("--num-envs", type=int, default=None,
                        help="CPU workers or GPU worlds; resume uses the saved count")
    args = parser.parse_args()
    if args.num_envs is None:
        args.num_envs = ((8 if args.physics_device == "gpu" else 1) if args.policy == "droq" else
                         128 if args.physics_device == "gpu" else default_num_envs(headless=args.headless))
    if args.rollout_steps is None:
        args.rollout_steps = 4096 if args.policy == "droq" else 8192 if args.physics_device == "gpu" else 512
    if args.steps < 0:
        parser.error("steps must be nonnegative; use 0 to run until stopped")
    if args.headless and args.watch:
        parser.error("--watch requires the window")
    if args.exit_after is not None and not 0 < args.exit_after < float("inf"):
        parser.error("--exit-after must be finite and positive")
    if not 0 < args.capture_at < float("inf"):
        parser.error("--capture-at must be finite and positive")
    # Select EGL before importing MuJoCo; Qt owns the actual display context.
    if not args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from sim.training import StopRequest
    from sim.walk_training import WalkConfig, train_walk, read_walk_manifest
    config = WalkConfig(seed=args.seed, policy_kind=args.policy, rollout_steps=args.rollout_steps, num_envs=args.num_envs,
                        physics_device=args.physics_device, episode_seconds=args.episode_seconds,
                        evaluation_interval=args.evaluation_interval, max_copper_power_w=args.max_copper_power_w,
                        max_temperature_c=args.max_temperature_c, energy_weight=args.energy_weight,
                        history_frames=args.history_frames, droq_utd_ratio=args.droq_utd_ratio,
                        droq_buffer_size=args.droq_buffer_size, droq_learning_starts=args.droq_learning_starts)
    try:
        config.validate()
    except ValueError as error:
        parser.error(str(error))
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    directory = args.resume or args.watch or args.run_dir or Path("runs") / datetime.now().strftime("walk-%Y%m%d-%H%M%S-%f")
    directory = directory.resolve()
    if not (args.resume or args.watch) and directory.exists():
        parser.error("run already exists; use --resume or choose a new directory")
    if args.resume or args.watch:
        try:
            read_walk_manifest(directory)
        except (ValueError, OSError) as error:
            parser.error(str(error))
    print(f"Walking training run: {directory.resolve()}", flush=True)
    if args.headless:
        with StopRequest() as stop:
            train_walk(directory, args.steps, config, resume=bool(args.resume), stop_requested=lambda: stop.requested)
        return 0
    from sim.walk_training_view import WalkTrainingWindow
    from sim.training_view import run_window
    options = [f"--policy={args.policy}", f"--history-frames={config.history_frames}", f"--physics-device={args.physics_device}", f"--num-envs={args.num_envs}", f"--seed={args.seed}", f"--rollout-steps={args.rollout_steps}",
               f"--episode-seconds={args.episode_seconds}", f"--evaluation-interval={args.evaluation_interval}",
               f"--max-copper-power-w={args.max_copper_power_w}", f"--max-temperature-c={args.max_temperature_c}",
               f"--energy-weight={args.energy_weight}", f"--droq-utd-ratio={args.droq_utd_ratio}",
               f"--droq-buffer-size={args.droq_buffer_size}", f"--droq-learning-starts={args.droq_learning_starts}"]
    return run_window(directory, args, window_factory=WalkTrainingWindow,
                      worker_name="walk_train_worker", worker_options=options)


if __name__ == "__main__":
    sys.exit(main())
