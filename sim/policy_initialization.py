"""Explicit supervised pose initialization before physics-based PPO training.

This module is used only by the trainer. Evaluation/playback never calls its
geometric solver; the exported policy is an ordinary sensor-input neural network.
"""
from dataclasses import replace
import time

import mujoco
import numpy as np
import torch

from sim.goals import GOAL_SETS
from sim.robot_model import LEGS, active_joints


def reference_action(env):
    """Solve reset-frame body/foot geometry for a supervised rotor-target label."""
    scratch = mujoco.MjData(env.model)
    scratch.qpos[:] = env.data.qpos
    scratch.qpos[:3] = env.target_position
    scratch.qpos[3:7] = env.target_quaternion
    active = [env.model.joint(j).qposadr[0] for leg in LEGS for j in active_joints(leg)]
    passive = [[env.model.joint(f"{role}_{p['cad']}").qposadr[0]
                for role in ("main_to_lower", "crank_to_link")] for p in LEGS.values()]

    def feet(x):
        scratch.qpos[active] = x
        for i, pair in enumerate(passive):
            scratch.qpos[pair] = x[3*i+1] - x[3*i]
        mujoco.mj_forward(env.model, scratch)
        return scratch.geom_xpos[env.foot_ids].ravel().copy()

    x = env.data.qpos[active].copy()
    target = env.foot_anchors.ravel()
    for _ in range(35):
        current = feet(x)
        if np.max(np.abs(target-current)) < 1e-8:
            break
        jac = np.column_stack([(feet(x + np.eye(12)[i]*1e-6)-current)/1e-6 for i in range(12)])
        x += np.clip(np.linalg.lstsq(jac, target-current, rcond=None)[0], -.05, .05)
    if np.max(np.abs(feet(x)-target)) > 1e-6:
        raise ValueError("initialization pose has no anchored-foot solution")
    action = (scratch.actuator_length - env.nominal_motor_position) / env.action_scale
    if np.max(np.abs(action)) > 1:
        raise ValueError("initialization pose exceeds motor target range")
    return action.astype(np.float32)


def initialize_policy(model, config, motor_config, progress, stop_requested, updates=1500):
    from sim.training import make_env

    observations, labels = [], []
    started = time.monotonic()
    # Evaluation uses exactly 1.0 × the stage's goals. Demonstrations bracket
    # that amplitude, so evaluation measures interpolation at unseen amplitudes.
    with make_env(replace(config, episode_seconds=2, robustness="none"), motor_config) as env:
        for factor in (.5, 1.5):
            for _, command in GOAL_SETS[config.goal_mode]:
                if stop_requested():
                    return None
                obs, _ = env.reset(seed=12345, options={
                    "command": np.array(command) * min(1., factor * config.goal_scale)})
                action = reference_action(env.unwrapped)
                for step in range(75):
                    if stop_requested():
                        return None
                    if step % 5 == 0:
                        observations.append(obs.copy())
                        labels.append(action.copy())
                    obs, _, terminated, truncated, _ = env.step(action)
                    if terminated or truncated:
                        break
                progress.status("initializing_policy", initialization_examples=len(labels))
    x = torch.as_tensor(np.array(observations), device=model.device)
    y = torch.as_tensor(np.array(labels), device=model.device)
    actor = model.policy
    parameters = list(actor.mlp_extractor.policy_net.parameters()) + list(actor.action_net.parameters())
    optimizer = torch.optim.Adam(parameters, lr=3e-3)
    actor.set_training_mode(True)
    for step in range(updates):
        if stop_requested():
            return None
        predicted = actor.get_distribution(x).distribution.mean
        loss = ((predicted-y)*config.action_scale).square().mean()
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(parameters, 1.)
        optimizer.step()
        model.supervised_updates = step + 1
        if step % 100 == 0:
            progress.status("initializing_policy", initialization_examples=len(labels),
                            supervised_updates=step+1, initialization_loss=float(loss.detach()))
    actor.set_training_mode(False)
    model.initialization_complete = True
    return {"method": "pose_demonstrations_v1", "complete": True, "examples": len(labels),
            "supervised_updates": updates, "rotor_mse_rad2": float(loss.detach()),
            "goal_scale": config.goal_scale, "elapsed_seconds": time.monotonic()-started}
