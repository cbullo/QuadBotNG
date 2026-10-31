"""PPO/DroQ walking experiments with explicit energy and thermal qualification."""
from collections import deque
from contextlib import ExitStack
from functools import partial
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import time
from uuid import uuid4

import gymnasium as gym
import numpy as np
from sb3_contrib import RecurrentPPO
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
import torch

from sim.training_compute import training_envs, validate_compute, set_training_scale, compute_layout
from sim.robot_model import SIM_DIR
from sim.training import atomic_json, model_fingerprint, publish
from sim.walk_env import QuadBotWalkEnv, WALK_REWARD_VERSIONS
from sim.droq import DroQ


@dataclass
class WalkConfig:
    seed: int = 42
    rollout_steps: int = 512
    num_envs: int = 1
    physics_device: str = "cpu"
    episode_seconds: float = 20.
    command_scale: float = .5
    robustness: str = "mild_v1"
    evaluation_interval: int = 8192
    max_copper_power_w: float = 20.
    max_temperature_c: float = 50.
    reward_version: str = "walk_energy_v3"
    policy_kind: str = "mlp"
    energy_weight: float = 1.
    history_frames: int | None = None
    droq_utd_ratio: int = 20
    droq_buffer_size: int = 50000
    droq_learning_starts: int = 1000

    def __post_init__(self):
        if self.history_frames is None:
            self.history_frames = 1 if self.policy_kind == "lstm" else 30

    def validate(self):
        if type(self.history_frames) is not int or self.history_frames < 1:
            raise ValueError("history_frames must be a positive integer")
        if self.policy_kind not in ("mlp", "lstm", "droq"):
            raise ValueError("walking policy must be mlp, lstm or droq")
        if type(self.droq_utd_ratio) is not int or self.droq_utd_ratio < 1:
            raise ValueError("droq_utd_ratio must be a positive integer")
        if type(self.droq_buffer_size) is not int or self.droq_buffer_size < max(256, self.num_envs):
            raise ValueError("droq_buffer_size must hold at least 256 transitions and one vector step")
        if type(self.droq_learning_starts) is not int or self.droq_learning_starts < 0:
            raise ValueError("droq_learning_starts must be a nonnegative integer")
        if self.reward_version not in WALK_REWARD_VERSIONS:
            raise ValueError("unknown walking reward version")
        if not np.isfinite(self.energy_weight) or self.energy_weight < 0:
            raise ValueError("energy_weight must be finite and nonnegative")
        if self.physics_device not in ("cpu", "gpu"):
            raise ValueError("physics_device must be cpu or gpu")
        validate_compute(self.num_envs, self.rollout_steps)
        if self.rollout_steps < 64 or self.rollout_steps % 64:
            raise ValueError("rollout_steps must be a positive multiple of 64")
        if not np.isfinite([self.episode_seconds, self.command_scale,
                            self.max_copper_power_w, self.max_temperature_c]).all():
            raise ValueError("walking configuration must be finite")
        if self.episode_seconds <= 0 or not 0 < self.command_scale <= 1:
            raise ValueError("invalid episode duration or command scale")
        if self.max_copper_power_w <= 0 or not 40 < self.max_temperature_c < 55:
            raise ValueError("power must be positive and thermal qualification limit must be between 40 and 55 C")
        if self.robustness not in ("none", "mild_v1") or self.evaluation_interval < 0:
            raise ValueError("invalid robustness or evaluation interval")


class WalkObservation(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.observation_space = gym.spaces.Box(-10, 10, (60,), np.float32)

    def observation(self, obs):
        values = np.r_[obs["command"] / [.3, .1, .6], obs["motor_position"] / 4,
                       obs["motor_velocity"] / 20, obs["imu"] / [5, 5, 5, 9.81, 9.81, 9.81],
                       obs["projected_gravity"], (obs["motor_temperature"]-20) / 35,
                       obs["previous_action"]]
        return np.clip(values, -10, 10).astype(np.float32)


class WalkCommandInput(gym.Wrapper):
    def __init__(self, env, command_provider):
        super().__init__(env)
        self.command_provider = command_provider

    def reset(self, *, seed=None, options=None):
        return self.env.reset(seed=seed, options=dict(options or {}, command=self.command_provider()))

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        if not (terminated or truncated):
            self.env.unwrapped.set_velocity_command(self.command_provider())
            obs["command"] = self.env.unwrapped.velocity_command.astype(np.float32).copy()
        return obs, reward, terminated, truncated, info


class WalkEpisodeDiagnostics(gym.Wrapper):
    fields = ("peak_temperature_c", "copper_energy_j", "mean_copper_power_w", "mean_velocity_error_m_s")

    def reset(self, **kwargs):
        self.velocity_error = 0.
        self.count = 0
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.velocity_error += info["velocity_error_m_s"]
        self.count += 1
        if terminated or truncated:
            info["mean_velocity_error_m_s"] = self.velocity_error / self.count
            info["mean_copper_power_w"] = info["copper_energy_j"] / info["time"]
        return obs, reward, terminated, truncated, info


def make_walk_env(config, motor_config=None, render_mode=None, command_provider=None):
    config.validate()
    env = QuadBotWalkEnv(episode_seconds=config.episode_seconds, command_scale=config.command_scale,
                        robustness=config.robustness, motor_config=motor_config, render_mode=render_mode,
                        reward_version=config.reward_version, energy_weight=config.energy_weight)
    if command_provider is not None:
        env = WalkCommandInput(env, command_provider)
    env = Monitor(WalkEpisodeDiagnostics(env), info_keywords=WalkEpisodeDiagnostics.fields)
    env = WalkObservation(env)
    if config.history_frames == 1:
        return env
    return gym.wrappers.FlattenObservation(gym.wrappers.FrameStackObservation(env, config.history_frames))


def read_walk_manifest(directory):
    manifest = json.loads((Path(directory) / "config.json").read_text())
    if (manifest.get("format") != 2 or manifest.get("task") != "QuadBot-Walk-v0"
            or manifest.get("contract") not in WALK_REWARD_VERSIONS
            or manifest.get("model_sha256") != model_fingerprint()):
        raise ValueError("walking checkpoint task/model/observation contract differs from this checkout")
    training = dict(manifest["training"])
    training.setdefault("policy_kind", "mlp")
    # Before explicit history settings, MLP used four frames and LSTM used one.
    training.setdefault("history_frames", 1 if training["policy_kind"] == "lstm" else 4)
    training.setdefault("reward_version", manifest["contract"])
    if training["reward_version"] != manifest["contract"]:
        raise ValueError("walking reward version differs from checkpoint contract")
    config = WalkConfig(**training)
    config.validate()
    expected_algorithm = walk_algorithm(config.policy_kind).__name__
    if manifest.get("algorithm") != expected_algorithm:
        raise ValueError("walking policy architecture differs from checkpoint algorithm")
    return config


def walk_algorithm(kind):
    return {"mlp": PPO, "lstm": RecurrentPPO, "droq": DroQ}[kind]


def load_walk_policy(directory, *, for_training=False):
    directory = Path(directory)
    config = read_walk_manifest(directory)
    pointer = json.loads((directory / "latest.json").read_text())
    if Path(pointer["file"]).name != pointer["file"]:
        raise ValueError("invalid walking checkpoint path")
    algorithm = walk_algorithm(config.policy_kind)
    options = {}
    if algorithm is DroQ:
        # Preview/evaluation need weights only; avoid allocating hundreds of MB
        # of replay data in each viewer or qualification process.
        options["buffer_size"] = config.droq_buffer_size if for_training else 1
        if for_training and (not pointer.get("replay_file") or
                Path(pointer["replay_file"]).name != pointer["replay_file"]):
            raise ValueError("DroQ checkpoint is missing its paired replay buffer")
    policy = algorithm.load(directory / pointer["file"], device="cpu", **options)
    if algorithm is DroQ and for_training:
        policy.load_replay_buffer(directory / pointer["replay_file"])
    return policy, replace(config, command_scale=policy.walk_command_scale), pointer


def publish_walk(model, directory):
    if not isinstance(model, DroQ):
        return publish(model, directory)
    # Publish the pair only after both files are complete. Unique names also
    # protect the previous pair when evaluation saves at identical counters.
    latest = directory / "latest.json"
    previous = json.loads(latest.read_text()) if latest.exists() else {}
    stem = f"policy-{model.num_timesteps:09d}-{model._n_updates:06d}-{uuid4().hex[:12]}"
    checkpoint = directory / f"{stem}.zip"
    temporary = checkpoint.with_suffix(".tmp.zip")
    model.save(temporary)
    temporary.replace(checkpoint)
    unchanged = (previous.get("steps") == model.num_timesteps and
                 previous.get("updates") == model._n_updates and previous.get("replay_file"))
    replay_name = previous["replay_file"] if unchanged else f"{stem}.replay.npz"
    if not unchanged:
        replay = directory / replay_name
        temporary = replay.with_suffix(".tmp")
        model.save_replay_buffer(temporary)
        temporary.replace(replay)
    pointer = dict(file=checkpoint.name, replay_file=replay_name,
                   steps=model.num_timesteps, updates=model._n_updates)
    atomic_json(latest, pointer)
    # Older weights remain available for inspection; only the latest replay is
    # needed for resume. Do not accumulate a full buffer every checkpoint.
    old_replay = previous.get("replay_file")
    if old_replay and old_replay != replay_name and Path(old_replay).name == old_replay:
        (directory / old_replay).unlink(missing_ok=True)
    return pointer


class WalkPolicyPlayer:
    """One independent inference stream, with an LSTM state per environment.

    Training manages its own states in RecurrentPPO. Viewers and qualification
    must preserve theirs between decisions and reset them at episode boundaries.
    PPO's feedforward predict accepts the same state/reset arguments.
    """
    def __init__(self, policy, num_envs=1):
        self.policy = policy
        self.num_envs = num_envs
        self.reset()

    def reset(self):
        self.state = None
        self.episode_starts = np.ones(self.num_envs, dtype=bool)

    def predict(self, observation, deterministic=True):
        action, self.state = self.policy.predict(observation, state=self.state,
            episode_start=self.episode_starts, deterministic=deterministic)
        self.episode_starts = np.zeros(self.num_envs, dtype=bool)
        return action


WALK_GOALS = (
    ("stand", (0, 0, 0)), ("forward", (.2, 0, 0)), ("backward", (-.12, 0, 0)),
    ("left", (0, .1, 0)), ("right", (0, -.1, 0)),
    ("turn_left", (0, 0, .4)), ("forward_turn", (.15, 0, -.4)),
)


def walk_success(command, samples, duration, requested_seconds, power, peak_temperature, reason, config):
    """A cool stationary robot must fail a nonzero motion command."""
    samples = np.asarray(samples)
    return bool(not reason and duration >= requested_seconds - 1e-6
                and np.all(samples[:, 0] <= max(.02, .25*np.linalg.norm(command[:2])))
                and np.all(samples[:, 1] <= max(.06, .25*abs(command[2])))
                and power <= config.max_copper_power_w and peak_temperature <= config.max_temperature_c)


def qualification_case(command, tail, info, total, seconds, config):
    power = info["copper_energy_j"] / info["time"]
    result = dict(duration_s=info["time"], episode_return=total,
        velocity_error_m_s=float(np.mean(tail, axis=0)[0]),
        yaw_error_rad_s=float(np.mean(tail, axis=0)[1]),
        mean_copper_power_w=power, copper_energy_j=info["copper_energy_j"],
        mean_positive_mechanical_power_w=info["positive_mechanical_work_j"]/info["time"],
        mean_absolute_mechanical_power_w=info["absolute_mechanical_work_j"]/info["time"],
        mean_energy_proxy_power_w=(info["copper_energy_j"]+info["absolute_mechanical_work_j"])/info["time"],
        peak_temperature_c=info["peak_temperature_c"], termination_reason=info["termination_reason"])
    result["success"] = walk_success(command, tail, info["time"], seconds, power,
        info["peak_temperature_c"], info["termination_reason"], config)
    return result


def qualification_report(policy, config, rows, device):
    return dict(steps=policy.num_timesteps, command_scale=config.command_scale, physics_device=device,
                success_pct=100*np.mean([r["policy"]["success"] for r in rows]),
                survival_pct=100*np.mean([not r["policy"]["termination_reason"] for r in rows]),
                mean_copper_power_w=float(np.mean([r["policy"]["mean_copper_power_w"] for r in rows])),
                peak_temperature_c=max(r["policy"]["peak_temperature_c"] for r in rows),
                max_copper_power_w=config.max_copper_power_w, max_temperature_c=config.max_temperature_c,
                goals=rows)


class GpuWalkQualification:
    """Reusable 28-world qualification graph; completed cases stay frozen."""
    def __init__(self, config, motor_config, seconds):
        self.config, self.motor_config, self.seconds = config, motor_config, seconds
        self.env = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        if self.env is not None:
            self.env.close()

    def run(self, policy, config, stop_requested, progress_callback):
        if stop_requested():
            return None
        started, last_progress = time.monotonic(), 0.
        rows, cases, options = [], [], []
        for temperature in (20., 40.):
            for name, goal in WALK_GOALS:
                command = np.array(goal)*config.command_scale
                row = dict(goal=name, command=command.tolist(), initial_temperature_c=temperature)
                rows.append(row)
                for kind in ("hold", "policy"):
                    cases.append((row, kind))
                    options.append(dict(command=command, initial_temperature_c=temperature))
        done = np.zeros(len(cases), dtype=bool)

        def notify(sim_time, phase="running", force=False):
            nonlocal last_progress
            now = time.monotonic()
            if progress_callback is not None and (force or now-last_progress >= 1):
                progress_callback(dict(completed=int(done.sum()), total=len(cases),
                    physics_device="gpu", parallel_cases=len(cases), phase=phase,
                    simulated_seconds=sim_time, case_seconds=self.seconds, elapsed_seconds=now-started))
                last_progress = now

        notify(0., "initializing", True)
        if stop_requested():
            return None
        if self.env is None:
            from sim.gpu_walk import GpuWalkVecEnv
            factory = partial(make_walk_env, replace(self.config, episode_seconds=self.seconds), self.motor_config)
            self.env = GpuWalkVecEnv([factory]*len(cases), auto_reset=False)
        # All cases use the same seed, matching the serial CPU qualification;
        # SB3's normal seed+i convention would change paired baseline conditions.
        self.env._seeds = [12345]*len(cases)
        self.env.set_options(options)
        obs = self.env.reset()
        if stop_requested():
            return None
        tails = [deque(maxlen=max(1, round(1/self.env.bases[0].dt))) for _ in cases]
        totals = np.zeros(len(cases))
        policy_cases = np.array([kind == "policy" for _, kind in cases])
        player = WalkPolicyPlayer(policy, num_envs=int(policy_cases.sum()))
        sim_time = 0.
        for _ in range(self.env.bases[0].max_steps):
            if stop_requested():
                return None
            actions = np.zeros((len(cases), 12), dtype=np.float32)
            if np.any(policy_cases & ~done):
                # Keep a fixed batch/world ordering as cases finish; shrinking
                # it would move one world's recurrent memory onto another.
                actions[policy_cases] = player.predict(obs[policy_cases])
                actions[done] = 0.
            obs, rewards, ended, infos = self.env.step(actions)
            previous_count = int(done.sum())
            for i in np.flatnonzero(~done):
                info = infos[i]
                totals[i] += rewards[i]
                tails[i].append((info["velocity_error_m_s"], info["yaw_error_rad_s"]))
                sim_time = max(sim_time, info["time"])
                if ended[i]:
                    row, kind = cases[i]
                    row[kind] = qualification_case(row["command"], tails[i], info, float(totals[i]), self.seconds, config)
                    done[i] = True
            player.episode_starts = done[policy_cases].copy()
            notify(sim_time, force=int(done.sum()) != previous_count)
            if done.all():
                break
        if not done.all():
            raise RuntimeError("GPU qualification ended with incomplete cases")
        notify(sim_time, force=True)
        return qualification_report(policy, config, rows, "gpu")


def evaluate_walk(policy, config, motor_config, seconds=10., stop_requested=lambda: False,
                  progress_callback=None, gpu_evaluator=None):
    if not np.isfinite(seconds) or seconds <= 0:
        raise ValueError("evaluation seconds must be finite and positive")
    if config.physics_device == "gpu":
        if gpu_evaluator is not None:
            return gpu_evaluator.run(policy, config, stop_requested, progress_callback)
        with GpuWalkQualification(config, motor_config, seconds) as evaluator:
            return evaluator.run(policy, config, stop_requested, progress_callback)
    rows = []
    started = time.monotonic()
    last_progress = 0.
    completed = 0

    def notify(name, kind, temperature, simulated_seconds, force=False):
        nonlocal last_progress
        now = time.monotonic()
        if progress_callback is not None and (force or now-last_progress >= 1):
            progress_callback(dict(completed=completed, total=4*len(WALK_GOALS), goal=name,
                kind=kind, initial_temperature_c=temperature, simulated_seconds=simulated_seconds,
                case_seconds=seconds, elapsed_seconds=now-started, physics_device="cpu"))
            last_progress = now

    with make_walk_env(replace(config, episode_seconds=seconds), motor_config) as env:
        for initial_temperature in (20., 40.):
            for name, command in WALK_GOALS:
                command = np.array(command) * config.command_scale
                row = dict(goal=name, command=command.tolist(), initial_temperature_c=initial_temperature)
                for kind in ("hold", "policy"):
                    if stop_requested():
                        return None
                    notify(name, kind, initial_temperature, 0., force=True)
                    obs, _ = env.reset(seed=12345, options={"command": command,
                                                          "initial_temperature_c": initial_temperature})
                    tail = deque(maxlen=max(1, round(1 / env.unwrapped.dt)))
                    player = WalkPolicyPlayer(policy)
                    total = 0.0
                    for _ in range(env.unwrapped.max_steps):
                        if stop_requested():
                            return None
                        action = np.zeros(12) if kind == "hold" else player.predict(obs)
                        obs, reward, terminated, truncated, info = env.step(action)
                        total += reward
                        notify(name, kind, initial_temperature, info["time"])
                        tail.append((info["velocity_error_m_s"], info["yaw_error_rad_s"]))
                        if terminated or truncated:
                            break
                    row[kind] = qualification_case(command, tail, info, total, seconds, config)
                    completed += 1
                    notify(name, kind, initial_temperature, info["time"], force=True)
                rows.append(row)
    return qualification_report(policy, config, rows, "cpu")


class WalkProgress(BaseCallback):
    def __init__(self, directory, stop_requested):
        super().__init__()
        self.directory = directory
        self.stop_requested = stop_requested
        self.started = time.monotonic()
        self.last_status = 0.
        self.last_preview = 0.
        self.last_metrics = 0.
        self.episodes = deque(maxlen=50)

    def status(self, state, **extra):
        droq = isinstance(self.model, DroQ)
        if droq:
            extra.update(actor_updates=self.model.actor_updates, utd_ratio=self.model.utd_ratio,
                         replay_transitions=self.model.replay_buffer.size()*self.model.n_envs,
                         losses=self.model.last_losses)
        atomic_json(self.directory / "status.json", dict(state=state, steps=self.model.num_timesteps,
                    updates=self.model._n_updates, elapsed_seconds=time.monotonic()-self.started,
                    command_scale=self.model.walk_command_scale, compute=compute_layout(self.training_env),
                    policy_kind="droq" if droq else "lstm" if isinstance(self.model, RecurrentPPO) else "mlp",
                    history_frames=self.model.observation_space.shape[0]//60, **extra))
        self.last_status = time.monotonic()

    def optimization_progress(self, completed, total):
        if time.monotonic()-self.last_status > 1:
            if time.monotonic()-self.last_metrics > 5:
                self.log_update()
            self.status("optimizing", optimization=dict(completed=completed, total=total))

    def qualification_progress(self, report):
        self.status("evaluating", qualification=report)
        # Case boundaries go into the log; within-case status updates keep the UI alive.
        if report.get("physics_device") == "gpu":
            if report["simulated_seconds"] == 0 or report["completed"] == report["total"]:
                print(f"GPU qualification: {report['phase']}, {report['completed']}/{report['total']} cases; "
                      f"{report['elapsed_seconds']:.0f} s elapsed", flush=True)
        elif report["simulated_seconds"] == 0 or report["completed"] == report["total"]:
            print(f"Qualification {report['completed']}/{report['total']} complete; "
                  f"{report['goal']} {report['kind']} at {report['initial_temperature_c']:g} C; "
                  f"{report['elapsed_seconds']:.0f} s elapsed (runs at full CPU speed)", flush=True)

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.episodes.append(info["episode"])
        if isinstance(self.model, DroQ) and time.monotonic()-self.last_metrics > 5:
            self.log_update()
        elif time.monotonic()-self.last_status > 1:
            self.status("training")
        if time.monotonic()-self.last_preview >= .1 and not self.locals["dones"][0]:
            self.last_preview = time.monotonic()
            snapshot = self.training_env.env_method("training_snapshot", indices=0)[0]
            info = self.locals["infos"][0]
            atomic_json(self.directory / "rollout.json", dict(
                steps=self.model.num_timesteps, time=snapshot["time"],
                qpos=snapshot["qpos"], qvel=snapshot["qvel"],
                command=info["command"].tolist(), body_velocity_m_s=info["body_velocity_m_s"].tolist(),
                max_temperature_c=info["max_temperature_c"], peak_temperature_c=info["peak_temperature_c"],
                copper_power_w=info["copper_power_w"], positive_mechanical_power_w=info["positive_mechanical_power_w"],
                velocity_error_m_s=info["velocity_error_m_s"], yaw_error_rad_s=info["yaw_error_rad_s"]))
        return not self.stop_requested()

    def log_update(self):
        row = dict(steps=self.model.num_timesteps, updates=self.model._n_updates)
        if isinstance(self.model, DroQ):
            row.update(actor_updates=self.model.actor_updates, **self.model.last_losses)
        for key in ("r", "l", *WalkEpisodeDiagnostics.fields):
            row[key] = float(np.mean([ep[key] for ep in self.episodes])) if self.episodes else None
        with (self.directory / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        self.last_metrics = time.monotonic()
        self.status("training")


def create_walk_policy(env, config):
    if config.policy_kind == "droq":
        return DroQ(env=env, device="cpu", seed=config.seed, utd_ratio=config.droq_utd_ratio,
                    buffer_size=config.droq_buffer_size, learning_starts=config.droq_learning_starts)
    recurrent = config.policy_kind == "lstm"
    policy_kwargs = dict(net_arch=[128, 128] if recurrent else [128, 128, 128],
                         activation_fn=torch.nn.ELU, log_std_init=np.log(.5 / 4.))
    if recurrent:
        policy_kwargs.update(lstm_hidden_size=128, n_lstm_layers=1,
                             shared_lstm=False, enable_critic_lstm=True)
    return walk_algorithm(config.policy_kind)("MlpLstmPolicy" if recurrent else "MlpPolicy", env, device="cpu",
                seed=config.seed, n_steps=config.rollout_steps // config.num_envs,
                batch_size=64, n_epochs=5, learning_rate=3e-4, target_kl=.01,
                gamma=.99, gae_lambda=.95, policy_kwargs=policy_kwargs, verbose=0)


def train_walk(directory, steps, config=None, resume=False, stop_requested=lambda: False):
    if steps < 0:
        raise ValueError("steps must be nonnegative; 0 runs until stopped")
    torch.set_num_threads(1)
    directory = Path(directory)
    if resume:
        config = read_walk_manifest(directory)
        if config.reward_version != "walk_energy_v3":
            penalty = -2 if config.reward_version == "walk_energy_v1" else -25
            print(f"Resuming {config.reward_version} rewards (failure penalty {penalty}). "
                  "Start a new run to use walk_energy_v3 energy-per-motion rewards.", flush=True)
    else:
        config = config or WalkConfig()
        config.validate()
        directory.mkdir(parents=True, exist_ok=False)
        (directory / "motor_model.yaml").write_bytes((SIM_DIR / "motor_model.yaml").read_bytes())
        atomic_json(directory / "config.json", dict(format=2, task="QuadBot-Walk-v0", contract=config.reward_version,
                    model_sha256=model_fingerprint(), training=asdict(config),
                    algorithm=walk_algorithm(config.policy_kind).__name__,
                    policy=("Separate 128-unit actor/critic LSTMs; 128/128 ELU; "
                            if config.policy_kind == "lstm" else
                            "SAC Gaussian actor, two dropout .01/LayerNorm critics; 256/256 ReLU; "
                            if config.policy_kind == "droq" else
                            "128/128/128 ELU; ") + f"{config.history_frames}x60 observations; rotor targets"))
    import fcntl
    with (directory / "trainer.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print(f"Walking policy: {config.policy_kind.upper()}; history: {config.history_frames} frames; "
              f"reward: {config.reward_version}", flush=True)
        if config.policy_kind == "droq":
            replay_mb = config.droq_buffer_size * (2*60*config.history_frames + 15) * 4 / 1e6
            print(f"DroQ: UTD {config.droq_utd_ratio} critic updates per transition; "
                  f"warm-up {config.droq_learning_starts} transitions; replay up to {replay_mb:.0f} MB; "
                  "CPU learning, GPU physics only when selected.", flush=True)
        atomic_json(directory / "status.json", dict(state="initializing", policy_kind=config.policy_kind,
            history_frames=config.history_frames, compute=dict(
            physics_device=config.physics_device, policy_device="cpu", num_envs=config.num_envs)))
        try:
            with training_envs(make_walk_env, config, directory / "motor_model.yaml") as env, ExitStack() as resources:
                if resume:
                    model, config, _ = load_walk_policy(directory, for_training=True)
                    model.set_env(env)
                    set_training_scale(env, "command_scale", config.command_scale)
                else:
                    model = create_walk_policy(env, config)
                    model.walk_command_scale = config.command_scale
                    model.curriculum_passes = model.last_evaluation_step = 0
                gpu_evaluator = (resources.enter_context(GpuWalkQualification(config, directory / "motor_model.yaml", 10.))
                                 if config.physics_device == "gpu" and config.evaluation_interval else None)
                progress = WalkProgress(directory, stop_requested)
                progress.init_callback(model)
                if isinstance(model, DroQ):
                    model.stop_requested = stop_requested
                    model.update_progress = progress.optimization_progress
                start = model.num_timesteps
                publish_walk(model, directory)
                progress.status("initializing" if config.physics_device == "gpu" else "training")
                while (steps == 0 or model.num_timesteps < start+steps) and not stop_requested():
                    chunk = config.rollout_steps if steps == 0 else min(config.rollout_steps, start+steps-model.num_timesteps)
                    model.learn(chunk, reset_num_timesteps=False, callback=progress)
                    if (not stop_requested() and config.evaluation_interval and
                            model.num_timesteps-model.last_evaluation_step >= config.evaluation_interval):
                        progress.status("saving")
                        publish_walk(model, directory)
                        progress.status("evaluating")
                        report = evaluate_walk(model, replace(config, command_scale=model.walk_command_scale),
                                               directory / "motor_model.yaml", seconds=10., stop_requested=stop_requested,
                                               progress_callback=progress.qualification_progress, gpu_evaluator=gpu_evaluator)
                        if report is not None:
                            atomic_json(directory / "evaluation.json", report)
                            with (directory / "evaluations.jsonl").open("a") as stream:
                                stream.write(json.dumps(report, allow_nan=False) + "\n")
                            model.curriculum_passes = model.curriculum_passes+1 if report["success_pct"] >= 75 else 0
                            if model.curriculum_passes >= 2:
                                model.walk_command_scale = min(1., model.walk_command_scale*1.25)
                                set_training_scale(env, "command_scale", model.walk_command_scale)
                                model.curriculum_passes = 0
                            model.last_evaluation_step = model.num_timesteps
                    progress.status("saving")
                    publish_walk(model, directory)
                    progress.log_update()
                    units = "critic updates" if isinstance(model, DroQ) else "epochs"
                    print(f"Walking {type(model).__name__} steps={model.num_timesteps}, "
                          f"{units}={model._n_updates}", flush=True)
                progress.status("stopped" if stop_requested() else "complete")
        except Exception as error:
            atomic_json(directory / "status.json", dict(state="failed", error=str(error)))
            raise
