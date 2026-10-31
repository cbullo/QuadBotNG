"""Compare a saved policy with nominal hold on the same body-pose goals."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path

import numpy as np
import torch

from sim.goals import GOAL_SETS, POSE_BODY_POSE_GOALS
from sim.training import evaluate_tracking, load_policy, make_env


def evaluate(directory, seconds=4, full_workspace=False, nominal=False):
    torch.set_num_threads(1)
    policy, config, pointer = load_policy(directory)
    config = replace(config, goal_scale=1 if full_workspace else config.goal_scale,
                     robustness="none" if nominal else config.robustness)
    results = []
    with make_env(replace(config, episode_seconds=seconds), Path(directory) / "motor_model.yaml") as env:
        for name, command in GOAL_SETS.get(config.goal_mode, POSE_BODY_POSE_GOALS):
            command = np.array(command) * (1 if full_workspace else config.goal_scale)
            row = {"goal": name}
            for kind in ("hold", "policy", "sampled_policy"):
                torch.manual_seed(12345)
                obs, _ = env.reset(seed=12345, options={"command": command})
                initial = env.unwrapped.data.qpos.copy()
                actions, total, peak_drift = [], 0.0, 0.0
                for _ in range(env.unwrapped.max_steps):
                    action = np.zeros(12) if kind == "hold" else policy.predict(
                        obs, deterministic=kind == "policy")[0]
                    obs, reward, terminated, truncated, info = env.step(action)
                    actions.append(env.unwrapped.previous_action.copy())
                    total += reward
                    peak_drift = max(peak_drift, float(max(info["foot_displacement_m"])))
                    if terminated or truncated:
                        break
                row[kind] = {
                    "return": total, "duration_s": info["time"],
                    "position_error_mm": info["position_error_m"] * 1000,
                    "orientation_error_deg": float(np.degrees(info["orientation_error_rad"])),
                    "body_translation_mm": float(np.linalg.norm(env.unwrapped.data.qpos[:3] - initial[:3])*1000),
                    "max_rotor_target_offset_rad": float(np.max(np.abs(actions))) * config.action_scale,
                    "foot_drift_mm": peak_drift*1000,
                    "termination_reason": info["termination_reason"],
                }
            results.append(row)
    return {"checkpoint": pointer, "goal_mode": config.goal_mode,
            "action_scale": config.action_scale,
            "goal_scale": config.goal_scale, "goals": results,
            "qualification": evaluate_tracking(policy, config, Path(directory) / "motor_model.yaml", seconds)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--seconds", type=float, default=4)
    parser.add_argument("--full-workspace", action="store_true", help="test full goals instead of the saved curriculum stage")
    parser.add_argument("--nominal", action="store_true", help="disable saved sensor/gain/friction variation for comparison")
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be finite and positive")
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    print(json.dumps(evaluate(args.run.resolve(), args.seconds, args.full_workspace, args.nominal), indent=2))


if __name__ == "__main__":
    main()
