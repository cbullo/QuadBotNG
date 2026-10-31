"""PPO training, observation contract and atomic policy publication."""
from collections import deque
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import signal
import time

import gymnasium as gym
import numpy as np
import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
import torch

from sim.gym_env import QuadBotBodyPoseEnv
from sim.goals import GOAL_SETS, WORKSPACE_ACTION_SCALE, WORKSPACE_MIN_BASE_HEIGHT
from sim.training_compute import training_envs, validate_compute, set_training_scale, compute_layout
from sim.robot_model import SIM_DIR, build_xml


@dataclass
class TrainingConfig:
    seed: int = 42
    rollout_steps: int = 512
    num_envs: int = 1
    episode_seconds: float = 10
    history: int = 4
    action_scale: float = WORKSPACE_ACTION_SCALE
    goal_mode: str = "workspace"
    reward_version: str = "planted_v4"
    min_base_height: float = WORKSPACE_MIN_BASE_HEIGHT
    observation_version: str = "workspace_v2"
    action_filter_seconds: float = .15
    target_rate_limit: float = 6.0
    goal_scale: float = .1
    learning_rate: float = 3e-5
    target_kl: float | None = .01
    evaluation_interval: int = 8192
    initialization: str = "pose_demonstrations_v1"
    policy_version: str = "pose_mlp_v2"
    command_interval: float = 4.0
    robustness: str = "mild_v1"
    evaluation_suite: str = "pose_suite_v2"

    def validate(self):
        validate_compute(self.num_envs, self.rollout_steps)
        if self.rollout_steps < 64 or self.rollout_steps % 64:
            raise ValueError("rollout_steps must be a positive multiple of 64")
        if not np.isfinite(self.episode_seconds) or self.episode_seconds <= 0:
            raise ValueError("episode_seconds must be positive")
        if self.history < 1 or not np.isfinite(self.action_scale) or self.action_scale <= 0:
            raise ValueError("history and action_scale must be positive")
        if self.goal_mode not in (*GOAL_SETS, "mixed"):
            raise ValueError("goal_mode must be workspace, poses, axis or mixed")
        if not np.isfinite(self.min_base_height) or self.min_base_height <= 0:
            raise ValueError("min_base_height must be positive")
        if self.observation_version not in ("legacy_v1", "workspace_v2"):
            raise ValueError("unknown observation_version")
        if self.reward_version not in ("legacy_v1", "tracking_v2", "dense_v3", "planted_v4"):
            raise ValueError("unknown reward_version")
        if self.policy_version not in ("small_v1", "pose_mlp_v2"):
            raise ValueError("unknown policy_version")
        if self.robustness not in ("none", "mild_v1"):
            raise ValueError("unknown robustness profile")
        if self.evaluation_suite not in ("legacy_v1", "pose_suite_v2"):
            raise ValueError("unknown evaluation_suite")
        if not np.isfinite(self.command_interval) or self.command_interval < 0:
            raise ValueError("command_interval must be finite and nonnegative")
        if (not np.isfinite([self.action_filter_seconds, self.target_rate_limit,
                             self.goal_scale, self.learning_rate]).all()
                or min(self.action_filter_seconds, self.target_rate_limit) < 0
                or not 0 < self.goal_scale <= 1 or self.learning_rate <= 0
                or self.evaluation_interval < 0):
            raise ValueError("invalid training control settings")
        if self.target_kl is not None and (not np.isfinite(self.target_kl) or self.target_kl <= 0):
            raise ValueError("target_kl must be positive or None")
        if self.initialization not in ("random", "pose_demonstrations_v1"):
            raise ValueError("unknown initialization method")
        if self.initialization != "random" and self.goal_mode not in GOAL_SETS:
            raise ValueError("pose initialization requires a named goal set")


class PolicyObservation(gym.ObservationWrapper):
    """Fixed scaling; the same transformation is stored with every checkpoint.

    Order: command, rotor position/velocity, IMU, temperature, previous action.
    No privileged pose/contact observations or mutable normalization statistics.
    """
    def __init__(self, env, version="workspace_v2"):
        super().__init__(env)
        self.version = version
        self.observation_space = gym.spaces.Box(-10, 10, (60,), np.float32)

    def observation(self, obs):
        command_scale = ([.01, .01, .01, .06, .06, .06] if self.version == "legacy_v1"
                         else [.06, .03, .15, .2, .25, .25])
        rotor_scale = 1 if self.version == "legacy_v1" else WORKSPACE_ACTION_SCALE
        vector = np.concatenate([
            obs["command"] / command_scale,
            obs["motor_position"] / rotor_scale, obs["motor_velocity"] / 20,
            obs["imu"] / [5, 5, 5, 9.81, 9.81, 9.81],
            (obs["motor_temperature"] - 20) / 35, obs["previous_action"],
        ])
        return np.clip(vector, -10, 10).astype(np.float32)


class EpisodeDiagnostics(gym.Wrapper):
    fields = ("position_error_mm", "orientation_error_deg", "foot_drift_mm", "feet_planted_pct")

    def reset(self, **kwargs):
        self.count = 0
        self.position = self.orientation = self.drift = self.planted = 0.0
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.count += 1
        self.position += info["position_error_m"] * 1000
        self.orientation += np.degrees(info["orientation_error_rad"])
        self.drift = max(self.drift, float(np.max(info["foot_displacement_m"])) * 1000)
        self.planted += bool(np.all(info["foot_contacts"]))
        if terminated or truncated:
            info.update(position_error_mm=self.position/self.count,
                        orientation_error_deg=self.orientation/self.count,
                        foot_drift_mm=self.drift, feet_planted_pct=100*self.planted/self.count)
        return obs, reward, terminated, truncated, info


class PoseCommandInput(gym.Wrapper):
    """Supply evaluation goals before observation scaling/history is applied."""
    def __init__(self, env, command_provider):
        super().__init__(env)
        self.command_provider = command_provider

    def reset(self, *, seed=None, options=None):
        options = dict(options or {}, command=self.command_provider())
        return self.env.reset(seed=seed, options=options)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        if not (terminated or truncated):
            # Reward belongs to the goal acted on. The next observation carries
            # the next goal, including the newest frame in the history wrapper.
            self.env.unwrapped.set_command(self.command_provider())
            obs["command"] = self.env.unwrapped.command.astype(np.float32).copy()
        return obs, reward, terminated, truncated, info


class PoseCommandSchedule(gym.Wrapper):
    """Train transitions between poses, retaining the episode's foot anchors.

    Explicit reset commands disable sampling for reproducible fixed-goal tests
    and supervised initialization. Reward always belongs to the command acted on.
    """
    def __init__(self, env, seconds):
        super().__init__(env)
        self.interval = max(1, round(seconds / env.unwrapped.dt))

    def _sample(self, obs):
        base = self.env.unwrapped
        goals = GOAL_SETS.get(base.goal_mode)
        if goals is None:
            command = base.np_random.uniform(-base.command_range, base.command_range) * base.goal_scale
        else:
            command = np.array(goals[base.np_random.integers(len(goals))][1], dtype=float)
            command *= base.goal_scale * base.np_random.uniform(.5, 1.)
        if base.np_random.random() < .2:
            command = np.zeros(6)  # Return to nominal stance is part of the task.
        base.set_command(command)
        obs["command"] = base.command.astype(np.float32).copy()
        return obs

    def reset(self, *, seed=None, options=None):
        self.fixed = "command" in (options or {})
        self.elapsed = 0
        obs, info = self.env.reset(seed=seed, options=options)
        if not self.fixed:
            obs = self._sample(obs)
            info = self.env.unwrapped._info()
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.elapsed += 1
        if not self.fixed and not (terminated or truncated) and self.elapsed % self.interval == 0:
            obs = self._sample(obs)
        return obs, reward, terminated, truncated, info


def make_env(config, motor_config, command_provider=None):
    config.validate()
    env = QuadBotBodyPoseEnv(episode_seconds=config.episode_seconds,
                             action_scale=config.action_scale, motor_config=motor_config,
                             goal_mode=config.goal_mode, reward_version=config.reward_version,
                             min_base_height=config.min_base_height,
                             action_filter_seconds=config.action_filter_seconds,
                             target_rate_limit=config.target_rate_limit, goal_scale=config.goal_scale,
                             robustness=config.robustness)
    if command_provider is not None:
        env = PoseCommandInput(env, command_provider)
    elif config.command_interval:
        env = PoseCommandSchedule(env, config.command_interval)
    env = Monitor(EpisodeDiagnostics(env), info_keywords=EpisodeDiagnostics.fields)
    env = PolicyObservation(env, config.observation_version)
    env = gym.wrappers.FrameStackObservation(env, config.history)
    return gym.wrappers.FlattenObservation(env)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def model_fingerprint():
    return hashlib.sha256(build_xml().encode()).hexdigest()


def read_manifest(run_dir):
    manifest = json.loads((Path(run_dir) / "config.json").read_text())
    if manifest["format"] != 1 or manifest["model_sha256"] != model_fingerprint():
        raise ValueError("checkpoint model/observation contract differs from this checkout")
    settings = dict(manifest["training"])
    settings.setdefault("goal_mode", "mixed")  # Preserve earlier runs on resume.
    settings.setdefault("reward_version", "legacy_v1")
    settings.setdefault("min_base_height", .12)
    settings.setdefault("observation_version", "legacy_v1")
    for key, value in dict(action_filter_seconds=0., target_rate_limit=0., goal_scale=1.,
                           learning_rate=3e-4, target_kl=None, evaluation_interval=0,
                           initialization="random", policy_version="small_v1", command_interval=0.,
                           robustness="none", evaluation_suite="legacy_v1").items():
        settings.setdefault(key, value)
    config = TrainingConfig(**settings)
    config.validate()
    return config


def publish(model, run_dir, suffix=""):
    """Publish a complete zip before updating its pointer; readers never race writes."""
    path = Path(run_dir) / f"policy-{model.num_timesteps:09d}-{model._n_updates:06d}{suffix}.zip"
    temporary = path.with_suffix(".tmp.zip")
    model.save(temporary)
    temporary.replace(path)
    pointer = {"file": path.name, "steps": model.num_timesteps, "updates": model._n_updates}
    atomic_json(Path(run_dir) / "latest.json", pointer)
    return pointer


def load_policy(run_dir):
    run_dir = Path(run_dir)
    config = read_manifest(run_dir)
    pointer = json.loads((run_dir / "latest.json").read_text())
    # The pointer is a basename created by publish, never an arbitrary path.
    if Path(pointer["file"]).name != pointer["file"]:
        raise ValueError("invalid checkpoint path")
    model = PPO.load(run_dir / pointer["file"], device="cpu")
    config = replace(config, goal_scale=getattr(model, "goal_scale", config.goal_scale))
    return model, config, pointer


def tracking_success(command, samples, peak_drift, terminated):
    """Require sustained tracking/contact and no earlier anchor excursion."""
    samples = np.asarray(samples)
    return bool(not terminated and peak_drift <= .005
                and np.all(samples[:, 0] <= max(.002, .25*np.linalg.norm(command[:3])))
                and np.all(samples[:, 1] <= max(.008, .25*np.linalg.norm(command[3:])))
                and np.all(samples[:, 2]))


def evaluate_tracking(model, config, motor_config, seconds=3.0, stop_requested=lambda: False):
    """Measure goal tracking against nominal hold, without exploration noise.

    Compare paired seeds/durations and test intermediate amplitudes too. Require
    the entire final half-second within tolerance, not just its average.
    This world has no effect on training state or its random-number generators.
    """
    config = replace(config, episode_seconds=seconds)
    rows = []
    goals = GOAL_SETS.get(config.goal_mode, GOAL_SETS["workspace"])
    cases = [(name, goal, 1., 12345) for name, goal in goals]
    if config.evaluation_suite == "pose_suite_v2":
        cases += [(name + " (75%)", goal, .75, 54321) for name, goal in goals]
    with make_env(config, motor_config) as env:
        for name, goal, amplitude, seed in cases:
            if stop_requested():
                return None
            command = np.array(goal) * config.goal_scale * amplitude
            row = {"goal": name, "command": command.tolist(), "seed": seed}
            for kind in ("hold", "policy"):
                obs, _ = env.reset(seed=seed, options={"command": command})
                samples = deque(maxlen=max(1, round(.5 / env.unwrapped.dt)))
                total = 0.0
                peak_drift = 0.0
                for _ in range(env.unwrapped.max_steps):
                    if stop_requested():
                        return None
                    action = np.zeros(12) if kind == "hold" else model.predict(obs, deterministic=True)[0]
                    obs, reward, terminated, truncated, info = env.step(action)
                    samples.append((info["position_error_m"], info["orientation_error_rad"],
                                    bool(np.all(info["foot_contacts"]))))
                    peak_drift = max(peak_drift, float(max(info["foot_displacement_m"])))
                    total += reward
                    if terminated or truncated:
                        break
                position, orientation, _ = np.mean(samples, axis=0)
                row[kind] = dict(position_error_mm=float(position*1000),
                                 orientation_error_deg=float(np.degrees(orientation)),
                                 cost=float(position/.02 + orientation/.1),
                                 duration_s=info["time"],
                                 foot_drift_mm=peak_drift*1000,
                                 termination_reason=info["termination_reason"],
                                 episode_return=total)
                if kind == "policy":
                    row["success"] = tracking_success(command, samples, peak_drift, terminated)
            rows.append(row)
    gains = [(r["hold"]["cost"] - r["policy"]["cost"]) / max(r["hold"]["cost"], .01)
             for r in rows if np.linalg.norm(r["command"]) > 0]
    return {"steps": model.num_timesteps, "goal_scale": config.goal_scale,
            "suite": config.evaluation_suite, "robustness": config.robustness,
            "tracking_gain_pct": float(100*np.mean(np.clip(gains, -2, 1))),
            "success_pct": 100*sum(r["success"] for r in rows)/len(rows),
            "survival_pct": 100*sum(not r["policy"]["termination_reason"] for r in rows)/len(rows),
            "mean_duration_s": float(np.mean([r["policy"]["duration_s"] for r in rows])),
            "goals": rows}


def record_evaluation(model, config, run_dir, env, progress):
    scale = getattr(model, "goal_scale", config.goal_scale)
    report = evaluate_tracking(model, replace(config, goal_scale=scale), run_dir / "motor_model.yaml",
                               stop_requested=progress.stop_requested)
    if report is None:
        return
    with (run_dir / "evaluations.jsonl").open("a") as stream:
        stream.write(json.dumps(report, allow_nan=False) + "\n")
    atomic_json(run_dir / "evaluation.json", report)
    # Advance only after repeated success, never just because time has passed.
    model.curriculum_passes = (getattr(model, "curriculum_passes", 0) + 1
                               if report["success_pct"] >= 75 else 0)
    if model.curriculum_passes >= 2 and scale < 1:
        model.goal_scale = min(1., scale * 1.5)
        model.curriculum_passes = 0
        set_training_scale(env, "goal_scale", model.goal_scale)  # Applies at the next reset.
    model.last_evaluation_step = model.num_timesteps
    progress.evaluation = report
    print(f"Evaluation: goals={scale:.0%}, tracking gain={report['tracking_gain_pct']:.1f}%, "
          f"success={report['success_pct']:.1f}%, survival={report['survival_pct']:.1f}%", flush=True)


class TrainingProgress(BaseCallback):
    def __init__(self, run_dir, stop_requested):
        super().__init__()
        self.run_dir = Path(run_dir)
        self.stop_requested = stop_requested
        self.episodes = deque(maxlen=50)
        self.episode_count = 0
        self.last_status = 0
        self.last_preview = 0
        self.started = time.monotonic()
        self.evaluation = None
        self.last_episode_end = None

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.episodes.append(info["episode"])
                self.episode_count += 1
                self.last_episode_end = {"reason": info.get("termination_reason") or "time_limit",
                                         "seconds": info.get("time")}
        if time.monotonic() - self.last_status >= 1:
            self.status("training")
        if time.monotonic() - self.last_preview >= .1:
            self.last_preview = time.monotonic()
            # The vector environment has already autoreset on a terminal transition. Skip
            # that frame rather than pairing the old goal/error with a new pose.
            if not self.locals["dones"][0]:
                snapshot = self.training_env.env_method("training_snapshot", indices=0)[0]
                info = self.locals["infos"][0]
                atomic_json(self.run_dir / "rollout.json", {
                    "steps": self.num_timesteps, "episode": self.episode_count,
                    **snapshot,
                    "position_error_m": info["position_error_m"],
                    "orientation_error_rad": info["orientation_error_rad"],
                    "foot_displacement_m": info["foot_displacement_m"].tolist(),
                    "foot_contacts": info["foot_contacts"].tolist(),
                    "max_temperature_c": info["max_temperature_c"],
                })
        return not self.stop_requested()

    def status(self, state, **extra):
        self.last_status = time.monotonic()
        atomic_json(self.run_dir / "status.json", {
            "state": state, "steps": self.model.num_timesteps,
            "updates": self.model._n_updates, "episodes": self.episode_count,
            "elapsed_seconds": time.monotonic() - self.started, **extra,
            "goal_scale": getattr(self.model, "goal_scale", 1.),
            "last_episode_end": self.last_episode_end, "compute": compute_layout(self.training_env),
        })

    def log_update(self):
        row = {"steps": self.model.num_timesteps, "updates": self.model._n_updates,
               "elapsed_seconds": time.monotonic() - self.started, "episodes": self.episode_count}
        for key in ("r", "l", *EpisodeDiagnostics.fields):
            row[key] = float(np.mean([episode[key] for episode in self.episodes])) if self.episodes else None
        for key in ("train/loss", "train/approx_kl", "train/entropy_loss"):
            value = self.model.logger.name_to_value.get(key)
            row[key] = float(value) if value is not None and np.isfinite(value) else None
        with (self.run_dir / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        self.status("training")


def train(run_dir, steps, config=None, resume=False, stop_requested=lambda: False):
    """Train for at least steps transitions (rounded to PPO rollout boundaries)."""
    if steps <= 0:
        raise ValueError("steps must be positive")
    torch.set_num_threads(1)
    run_dir = Path(run_dir)
    if resume:
        config = read_manifest(run_dir)
    else:
        config = config or TrainingConfig()
        config.validate()
        run_dir.mkdir(parents=True, exist_ok=False)
        (run_dir / "motor_model.yaml").write_bytes((SIM_DIR / "motor_model.yaml").read_bytes())
        atomic_json(run_dir / "config.json", {
            "format": 1, "model_sha256": model_fingerprint(), "training": asdict(config),
            "algorithm": "PPO", "stable_baselines3": stable_baselines3.__version__,
            "torch": torch.__version__,
        })
    # Prevent two writers from interleaving policies and logs in the same run.
    import fcntl
    with (run_dir / "trainer.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with training_envs(make_env, config, run_dir / "motor_model.yaml") as env:
            try:
                if resume:
                    model, _, _ = load_policy(run_dir)
                    model.set_env(env)
                    set_training_scale(env, "goal_scale", getattr(model, "goal_scale", config.goal_scale))
                else:
                    model = PPO("MlpPolicy", env, device="cpu", seed=config.seed,
                                n_steps=config.rollout_steps // config.num_envs, batch_size=64, n_epochs=5,
                                learning_rate=config.learning_rate, target_kl=config.target_kl,
                                gamma=.99, gae_lambda=.95,
                                # Preserve exploration in physical rotor radians
                                # when widening the normalized action mapping.
                                policy_kwargs={"net_arch": ([128, 128, 128] if config.policy_version == "pose_mlp_v2"
                                                            else [64, 64]),
                                               "activation_fn": (torch.nn.ELU if config.policy_version == "pose_mlp_v2"
                                                                 else torch.nn.Tanh),
                                               "log_std_init": -2.0 - np.log(config.action_scale)},
                                verbose=0)
                    model.goal_scale = config.goal_scale
                    model.curriculum_passes = 0
                    model.last_evaluation_step = 0
                    model.initialization_complete = config.initialization == "random"
                progress = TrainingProgress(run_dir, stop_requested)
                progress.init_callback(model)
                start = model.num_timesteps
                publish(model, run_dir)
                progress.status("training")
                if config.initialization != "random" and not getattr(model, "initialization_complete", False):
                    from sim.policy_initialization import initialize_policy
                    report = initialize_policy(model, config, run_dir / "motor_model.yaml", progress, stop_requested)
                    if report is not None:
                        atomic_json(run_dir / "initialization.json", report)
                    publish(model, run_dir, suffix=f"-initialization-{getattr(model, 'supervised_updates', 0)}")
                if not resume and config.evaluation_interval and steps >= config.evaluation_interval:
                    record_evaluation(model, config, run_dir, env, progress)
                while model.num_timesteps < start + steps and not stop_requested():
                    model.learn(config.rollout_steps, reset_num_timesteps=False, callback=progress)
                    if (config.evaluation_interval
                            and model.num_timesteps - getattr(model, "last_evaluation_step", 0)
                            >= config.evaluation_interval):
                        record_evaluation(model, config, run_dir, env, progress)
                    publish(model, run_dir)
                    progress.log_update()
                    print(f"PPO steps={model.num_timesteps}, training epochs={model._n_updates}, "
                          f"completed episodes={progress.episode_count}", flush=True)
                progress.status("stopped" if stop_requested() else "complete")
            except Exception as error:
                atomic_json(run_dir / "status.json", {"state": "failed", "error": str(error)})
                raise



class StopRequest:
    """Signal handlers set a flag; checkpoints are written by the normal loop."""
    def __init__(self):
        self.requested = False

    def __enter__(self):
        self.previous = {sig: signal.signal(sig, self._handle) for sig in (signal.SIGINT, signal.SIGTERM)}
        return self

    def _handle(self, *_):
        self.requested = True

    def __exit__(self, *_):
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)
