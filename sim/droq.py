"""DroQ on SB3's rollout/replay infrastructure, following A Walk in the Park.

Reference algorithm/config: https://github.com/ikostrikov/walk_in_the_park
Two dropout + LayerNorm critics; minimum target Q, mean actor Q; delayed actor
and entropy updates. The actor remains SB3's squashed Gaussian MLP.
"""
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from stable_baselines3 import SAC
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.utils import polyak_update
from stable_baselines3.sac.policies import SACPolicy


class DroQPolicy(SACPolicy):
    def make_critic(self, features_extractor=None):
        critic = super().make_critic(features_extractor)
        # Replace the registered Q modules before SACPolicy builds optimizers.
        # Keep independent networks, including independent dropout masks.
        networks = []
        for index, original in enumerate(critic.q_networks):
            layers = []
            linear_layers = [layer for layer in original if isinstance(layer, nn.Linear)]
            for number, linear in enumerate(linear_layers):
                layers.append(linear)
                if number < len(linear_layers)-1:
                    layers.extend((nn.Dropout(.01), nn.LayerNorm(linear.out_features), nn.ReLU()))
            network = nn.Sequential(*layers).to(self.device)
            setattr(critic, f"qf{index}", network)
            networks.append(network)
        critic.q_networks = networks
        return critic


class DroQ(SAC):
    """Each collected transition buys `utd_ratio` critic updates and one actor update.

    SB3 gradient_steps=-1 supplies the number of transitions, including all
    vector worlds. Counting per transition keeps UTD independent of world count.
    Runtime callbacks are deliberately excluded from model checkpoints.
    """
    def __init__(self, policy=DroQPolicy, env=None, *, utd_ratio=20, **kwargs):
        if type(utd_ratio) is not int or utd_ratio < 1:
            raise ValueError("DroQ UTD ratio must be a positive integer")
        self.utd_ratio = utd_ratio
        self.actor_updates = 0
        self.stop_requested = lambda: False
        self.update_progress = lambda completed, total: None
        self.last_losses = {}
        defaults = dict(learning_rate=3e-4, buffer_size=50000, learning_starts=1000,
                        batch_size=256, tau=.005, gamma=.99, train_freq=1,
                        gradient_steps=-1, ent_coef="auto_0.1",
                        policy_kwargs=dict(net_arch=[256, 256]))
        defaults.update(kwargs)
        super().__init__(policy, env, **defaults)

    def _setup_model(self):
        # The reference uses half the usual SAC target entropy.
        if self.target_entropy == "auto":
            self.target_entropy = -float(np.prod(self.action_space.shape)) / 2
        super()._setup_model()

    def _excluded_save_params(self):
        return super()._excluded_save_params() + ["stop_requested", "update_progress"]

    def train(self, gradient_steps, batch_size=256):
        self.policy.set_training_mode(True)
        # DroQ deliberately samples dropout in the target and actor-Q paths too.
        self.critic_target.set_training_mode(True)
        self._update_learning_rate([self.actor.optimizer, self.critic.optimizer, self.ent_coef_optimizer])
        total = gradient_steps * self.utd_ratio
        for index in range(total):
            if self.stop_requested():
                break
            replay = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)
            alpha = self.log_ent_coef.detach().exp()
            with torch.no_grad():
                next_action, next_log_prob = self.actor.action_log_prob(replay.next_observations)
                next_q = torch.cat(self.critic_target(replay.next_observations, next_action), dim=1)
                next_q = next_q.min(dim=1, keepdim=True).values - alpha * next_log_prob.reshape(-1, 1)
                target = replay.rewards + (1-replay.dones) * self.gamma * next_q
            predictions = self.critic(replay.observations, replay.actions)
            critic_loss = sum(F.mse_loss(q, target) for q in predictions) / len(predictions)
            self.critic.optimizer.zero_grad(set_to_none=True)
            critic_loss.backward()
            self.critic.optimizer.step()
            polyak_update(self.critic.parameters(), self.critic_target.parameters(), self.tau)
            self._n_updates += 1
            self.last_losses["critic_loss"] = float(critic_loss.detach())

            if self._n_updates % self.utd_ratio == 0:
                # Do not accumulate critic parameter gradients during the actor update.
                self.critic.requires_grad_(False)
                try:
                    action, log_prob = self.actor.action_log_prob(replay.observations)
                    q = torch.cat(self.critic(replay.observations, action), dim=1).mean(dim=1)
                    actor_loss = (alpha * log_prob - q).mean()
                    self.actor.optimizer.zero_grad(set_to_none=True)
                    actor_loss.backward()
                    self.actor.optimizer.step()
                finally:
                    self.critic.requires_grad_(True)
                # Match the reference's alpha objective, not SB3's log-alpha surrogate.
                temperature_loss = (self.log_ent_coef.exp() *
                                    (-log_prob.detach() - self.target_entropy)).mean()
                self.ent_coef_optimizer.zero_grad(set_to_none=True)
                temperature_loss.backward()
                self.ent_coef_optimizer.step()
                self.actor_updates += 1
                self.last_losses.update(actor_loss=float(actor_loss.detach()),
                                        ent_coef=float(self.log_ent_coef.detach().exp().item()))
            self.update_progress(index+1, total)
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/actor_updates", self.actor_updates)
        for key, value in self.last_losses.items():
            self.logger.record(f"train/{key}", value)

    def save_replay_buffer(self, path):
        """Save only occupied rows, losslessly, without pickling Python objects.

        Full observation histories and terminal next observations are retained.
        The NPZ is uncompressed to keep Stop/save latency bounded by disk I/O.
        """
        replay = self.replay_buffer
        rows = replay.buffer_size if replay.full else replay.pos
        with Path(path).open("wb") as stream:
            np.savez(stream, version=1, capacity=replay.buffer_size, n_envs=replay.n_envs,
                     pos=replay.pos, full=replay.full,
                     **{name: getattr(replay, name)[:rows] for name in
                        ("observations", "next_observations", "actions", "rewards", "dones", "timeouts")})

    def load_replay_buffer(self, path, truncate_last_traj=True):
        # Episodes restart on resume. Each saved transition already has its own
        # next observation, so no trajectory stitching or truncation is needed.
        with np.load(path, allow_pickle=False) as saved:
            capacity, n_envs = int(saved["capacity"]), int(saved["n_envs"])
            if int(saved["version"]) != 1 or n_envs != self.n_envs or capacity != max(1, self.buffer_size // n_envs):
                raise ValueError("DroQ replay configuration differs from the model")
            replay = self.replay_buffer
            if not isinstance(replay, ReplayBuffer) or replay.buffer_size != capacity:
                raise ValueError("DroQ resume requires a full-sized replay buffer")
            pos, full = int(saved["pos"]), bool(saved["full"])
            if not 0 <= pos < capacity:
                raise ValueError("invalid DroQ replay position")
            rows = capacity if full else pos
            for name in ("observations", "next_observations", "actions", "rewards", "dones", "timeouts"):
                values = saved[name]
                target = getattr(replay, name)[:rows]
                if values.shape != target.shape or values.dtype != target.dtype:
                    raise ValueError(f"invalid DroQ replay {name}")
                target[:] = values
            replay.pos, replay.full = pos, full
