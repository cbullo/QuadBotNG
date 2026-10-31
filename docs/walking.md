# Walking training and live window

Start training and its local robot/metrics window with one command:

```sh
bazel run --config=pc //sim:walk_train
```

For batched NVIDIA GPU physics with the same live window:

```sh
bazel run --config=pc //sim:walk_train -- --physics-device=gpu
```

To start a new GPU run with recurrent memory and no step limit:

```sh
bazel run --config=pc //sim:walk_train -- --physics-device=gpu --policy=lstm --steps=0
```

`--policy=lstm` selects SB3-Contrib's `RecurrentPPO`/`MlpLstmPolicy` (pinned
to 2.9.0). By default each observation is one normalized 60-value sensor/command frame.
Separate actor and critic networks each use a single 128-unit LSTM, followed
by 128/128 ELU layers and their action/value output. The actor still produces
twelve rotor targets at 50 Hz. Hidden and cell states persist between decisions
and rollout updates; episode resets clear the affected world's memory. GPU
qualification keeps a fixed batch ordering so finished cases cannot shift
another case's memory. Preview and standalone playback also retain state and
clear it on episode/checkpoint restarts.

The standard GPU rollout contains 64 steps per world (1.28 simulated seconds).
Recurrent PPO trains on sequences with stored initial states; rollout and
minibatch boundaries limit backpropagation length, not how long inference may
carry memory. Remembering useful gait history must still be learned.
See the [SB3-Contrib recurrent PPO documentation](https://sb3-contrib.readthedocs.io/en/master/modules/ppo_recurrent.html).

`--policy=mlp` remains the default and now uses 30 observation frames, matching
the buffer length in the energy paper's
[configuration](https://github.com/Oscar-B-Liang/wtw_new/blob/main/go1_gym/envs/go1/go1_config.py)
and [history wrapper](https://github.com/Oscar-B-Liang/wtw_new/blob/main/go1_gym/envs/wrappers/history_wrapper.py).
This is the current frame plus 29 previous frames: 1,800 inputs spanning 580 ms
at 50 Hz. Our Gym wrapper pads new episodes with the reset observation and never
carries frames across resets. LSTM still defaults to one frame plus recurrent
memory. `--history-frames=N` overrides either architecture for a new run, so
`--policy=lstm --history-frames=30` combines the two, and
`--policy=mlp --history-frames=4` selects the previous history length.

```sh
# Paper-style explicit history with the new energy reward:
bazel run --config=pc //sim:walk_train -- --physics-device=gpu --policy=mlp --history-frames=30 --steps=0
```

## DroQ / SAC

Start a **new DroQ run**, with GPU physics, the live window and no step limit:

```sh
bazel run --config=pc //sim:walk_train -- --physics-device=gpu --policy=droq --steps=0
```

DroQ is a separate off-policy learning algorithm selected through `--policy`.
`mlp` and `lstm` still select PPO and recurrent PPO. This implementation follows
the [A Walk in the Park learner](https://github.com/ikostrikov/walk_in_the_park/blob/main/rl/agents/sac/sac_learner.py)
and [DroQ configuration](https://github.com/ikostrikov/walk_in_the_park/blob/main/configs/droq_config.py):

- A squashed Gaussian MLP actor and two independent Q critics, with 256/256
  ReLU hidden layers. Each critic hidden layer uses dropout (0.01) and LayerNorm.
- Minimum target Q for the critic backup; mean Q for the actor loss. Dropout
  is active in the online critic, target critic and actor's Q evaluation.
- Default 20 critic minibatch updates per new transition, then one actor and
  entropy-temperature update. Minibatches contain 256 replay transitions.
  Target weights receive a soft update after each critic update (tau 0.005).
- Discount 0.99, Adam learning rates 0.0003, initial entropy coefficient 0.1,
  target entropy −6 for twelve actions, and 1,000 random warm-up transitions.

The implementation uses the existing SB3/PyTorch infrastructure instead of the
paper's JAX stack, including SB3's actor initialization and log-standard-deviation
bounds. It retains our 30-frame inputs, coupled rotor actions, task rewards,
fall/overheat penalties and thermal qualification. Rewards are not rescaled.
These robot-specific choices make it an adaptation, not an exact reproduction
of the A1 experiment. Learning quality and wall-clock speed are unmeasured.

DroQ defaults to **8 GPU worlds or 1 CPU world**, and checkpoints every 4,096
transitions (`--rollout-steps`). It learns after every vector step, rather than
waiting for this checkpoint interval. The update ratio counts transitions across
all worlds: eight worlds at ratio 20 produce 160 critic updates and eight actor
updates after each vector step. More worlds therefore do not reduce optimization
work per transition. GPU physics and GPU qualification are supported; **learning
still runs on CPU** with the installed CPU-only Torch wheel. This algorithm aims
to reuse experience, and can take substantially longer per transition than PPO.
`--droq-utd-ratio=1` reduces CPU work but departs from the reference ratio of 20.

Replay defaults to **50,000 transitions** (`--droq-buffer-size`), requiring about
723 MB with 30-frame histories, plus model/optimizer and simulator memory. A
full replay checkpoint has approximately the same size. Only occupied rows are
written during warm-up, and only the latest replay snapshot is retained. While
saving, disk space must accommodate both the old and new snapshot. Model files
are retained independently. Smaller buffers or fewer history frames reduce
memory/disk use; the reference's million-transition replay would need about
14.5 GB for our stacked observations alone and is not the default here.

**Stop training and save** checks for cancellation between critic updates and
then saves the paired model and replay. Saving a full replay still takes disk
I/O time; the window reports it. Resume restores both critics, target networks,
actor, optimizers, learned entropy coefficient, update counters, curriculum and
replay, while beginning fresh simulation episodes:

```sh
bazel run --config=pc //sim:walk_train -- --resume=runs/YOUR_DROQ_RUN --steps=0
```

Preview and evaluation load weights without the full replay allocation. Existing
PPO checkpoints remain PPO; `--resume` preserves the saved algorithm, history,
update ratio, replay capacity and `--droq-learning-starts` warm-up setting.
The live status distinguishes critic updates, actor updates, replay occupancy
and optimization progress. Training losses are also recorded in `metrics.jsonl`.

Regression coverage for vector update ratios, terminal versus timeout handling,
stop during optimization, paired checkpoint publication, wrapped replay, resume
and walking qualification is in `//sim:droq_test` and `//sim:walk_test`. These
tests were added but not executed, respecting the request to stop tests.

## Shared run behavior

The run manifest records the architecture, and resume uses that saved choice. Old
manifests without an architecture field load as MLP; missing history fields
retain four frames for old MLP runs and one for old LSTM runs. Start a new run
to change the architecture or history length;
an existing MLP checkpoint cannot be converted by adding `--policy=lstm` to
`--resume`. Restarted training restores weights/optimizer but begins fresh
simulation episodes and recurrent memory. The window/log identifies the active
policy. LSTM PPO still runs on CPU alongside GPU physics; its learning quality
and speed have not yet been measured here.

This uses **both devices**: MuJoCo Warp runs the 2 kHz physics, 1 kHz voltage
controller and motor heating on the GPU. The CPU computes observations/rewards,
episode resets, policy learning, qualification scoring and preview physics. GPU PPO runs default to 128
worlds and 8,192 transitions per PPO rollout (64 steps per world). CPU PPO defaults
remain unchanged. `--num-envs` and `--rollout-steps` override these defaults for
new runs; resume uses the settings saved in that run, so an existing CPU run
continues on CPU. Policies trained on GPU can be previewed/evaluated on CPU.

The first GPU launch compiles kernels and can take several minutes; compilation
is cached for later runs. The window reports initialization while this happens,
and its training status identifies GPU physics and CPU PPO. A missing CUDA
device or physics overflow produces an explicit failure rather than silently
switching devices. MuJoCo Warp 3.17.0 and Warp 1.18.0 are pinned in the Bazel
dependency lock. PyTorch intentionally remains the CPU build.

The GPU backend retains the coupled differential, friction/gain randomization,
per-motor temperatures, copper/effort rewards, and 55 °C cutoff. Each overheated
world stops at its first violating physics substep and resets independently.
GPU runs also qualify all 28 cold/warm baseline/policy cases in parallel on the
GPU. Completed cases freeze, and the evaluator reuses its compiled graph between
checks. CPU runs retain serial CPU qualification. The window shows completed
cases, parallel simulated time and elapsed time. Stop/save remains available.
Existing trainer processes need one stop/save and resume to load this change.
For a CPU reference comparison, use `//sim:walk_evaluate` with
`--physics-device=cpu`; standalone evaluation otherwise follows the saved backend. See the measured comparison
and limits in [simulator-research.md](simulator-research.md#gpu-implementation-and-measurements).

The native PySide6/PyQtGraph window uses the same worker lifecycle as pose
training. With the default CPU backend, training runs at full speed with parallel
CPU simulation workers and a CPU PPO process. New windowed runs default to up to four physics workers;
headless runs use up to eight, capped at half the available logical CPUs. Closing the
window or pressing **Stop training and save** requests a checkpoint and stops
that worker. After the worker exits, the button confirms **Stopped — checkpoint
saved** (or reports a trainer failure). The window and independent policy preview
stay open; continued preview motion does not mean training is still running.
Completed training also leaves the window open for inspection.
The plain launch command always starts a new run. To continue saved training,
use `--resume=<run-directory>` (the directory is shown at the bottom of the window).
Resume restores policy/optimizer/curriculum/counters, but starts fresh simulation
episodes. Interrupted qualification is retried when due.

The default launch stops after **100,000 additional training transitions** (across
all worlds), rounded up to a PPO rollout boundary or a DroQ vector step. There is no default time limit.
Use `--steps=0` to keep training until you press **Stop training and save**, close
the owning window, or interrupt the headless trainer. Qualification and checkpoint
saving continue during an unlimited run. For example:

```sh
bazel run --config=pc //sim:walk_train -- --resume=runs/YOUR_RUN --steps=0
# Or start a new unlimited GPU run:
bazel run --config=pc //sim:walk_train -- --physics-device=gpu --steps=0
```

An already running trainer keeps its original limit; stop/save and resume with
`--steps=0` to change it. The step budget is chosen for each invocation rather
than saved in the run configuration.

The window includes:

- **Policy preview:** independent simulation paced at 50 Hz, with
  automatic checkpoint refresh between 30-second loops. Falls reset the episode
  using the same checkpoint until the loop finishes. The default displays mean
  policy actions; sampled actions are selectable.
- **Live training rollout:** the actual exploratory physics state, sampled at
  10 Hz. This is a snapshot of accelerated training, not real-time playback.
- Forward/sideways/turn commands, automatic command cycling, and manual restart.
- Instantaneous speed, tracking error, maximum motor temperature, copper
  heating power and positive mechanical power.
- Training curves for return, mean velocity error, peak motor temperature and
  mean copper power, averaged over the last 50 completed episodes. Curves appear
  after episodes complete; the default episode length is 20 simulated seconds.
- Separate qualification progress: completed cases out of 28, current command,
  baseline/policy and temperature, simulated case time and actual elapsed time.
  Qualification uses the saved training backend (28 parallel cases for GPU);
  training counters pause while it runs. The preview's 30-second loop is independent of qualification.
- Cold/warm qualification results when periodic evaluation has completed.

Evaluation playback starts at 40 °C to expose warm-motor behavior. It uses the
same observation preprocessing and physical model as headless policy evaluation.
It does not pause the trainer or inject viewer commands into training. Rendering
speed can limit playback speed but does not throttle the separate training worker.

```sh
# Choose a run directory:
bazel run --config=pc //sim:walk_train -- --run-dir=runs/walk-01 --steps=100000
# Resume and open the window:
bazel run --config=pc //sim:walk_train -- --resume=runs/walk-01
# Attach without starting or owning a trainer:
bazel run --config=pc //sim:walk_train -- --watch=runs/walk-01
# Train without a window:
bazel run --config=pc //sim:walk_train -- --headless --run-dir=runs/walk-02
# Machine-readable comparison against zero-action standing:
bazel run --config=pc //sim:walk_evaluate -- runs/walk-01
# Longer cold/warm tests for accumulated heat:
bazel run --config=pc //sim:walk_evaluate -- runs/walk-01 --seconds=60
```

Use `--no-render` for a metrics-only window. The robot image uses EGL and
respects an explicit `MUJOCO_GL` setting. Runs use timestamped directories under
ignored `runs/` by default. GUI worker output is saved to `<run-directory>.log`.
`config.json`, `motor_model.yaml`, policy ZIPs, `latest.json`, `metrics.jsonl`,
`status.json`, `rollout.json` and evaluation reports remain inspectable locally.

## Task and policy

`QuadBot-Walk-v0` is separate from planted-foot `QuadBot-BodyPose-v0`; their
checkpoints are deliberately incompatible. New MLP walking policies have 1,800
flattened inputs by default; LSTM walking uses 60 per decision by default.
The research basis is described in
[rl-body-pose.md](rl-body-pose.md): PPO, nominal-posture offsets, velocity
commands, action regularization and randomization are adapted from established
quadruped training. The current energy recipe avoids prescribed gait rewards.
No fixed gait or independent serial-joint model is imposed.

- Commands are `[vx, vy, yaw_rate]` in current chassis axes, in m/s and rad/s.
  Full sampling ranges are forward `[-.15,.30]`, sideways `[-.10,.10]`, turning
  `[-.60,.60]`; training begins at half range. Commands resample every four
  seconds and 20% are neutral. Explicit reset commands disable resampling.
- Reset uses a more bent stance (CAD main/crank offsets `+.3/-.3` radians) with
  the original closed linkages, floor contacts and physical settling interval.
  Temperature is sampled from 20–40 °C. Each environment retains the existing
  mild friction, gain and sensor variation.
- Actions are twelve rotor offsets in FR/FL/BR/BL × I/O/Z order, ±4 rotor
  radians, filtered over 40 ms and limited to 20 rotor rad/s target changes.
  These pass through the existing 9:1 reduction, three-motor differential,
  voltage PD and torque/thermal model. They are not independent joint angles.
- Policy/control/physics rates are 50 Hz / 1 kHz / 2 kHz. Hardware calibration,
  electrical zeros, motor resistance and torque limits are unchanged.
- Observations per frame are command (3), rotor position/velocity (24), raw
  IMU (6), projected gravity (3), modeled temperatures (12) and previous applied
  action (12). In MLP mode, 30 frames feed separate 128/128/128 ELU actor/critic
  networks; LSTM mode uses one frame and persistent memory as described above.
  Projected gravity is an **ideal simulation attitude estimate**; hardware needs
  an attitude estimator. Simulator base velocity is used for reward/evaluation,
  not exposed to the actor. This is not a deployed sensor/control pipeline.
- PPO uses 512 total transitions per update (128 steps per environment with
  four workers, 64 with eight), batches of 64, up to five epochs, learning rate
  3e-4, target KL .01, discount .99 and GAE .95. Initial action standard
  deviation is .5 physical rotor radians. Walking starts from random weights;
  body-pose demonstrations are not substituted for a learned walking policy.

## Temperature and effort are explicit objectives

The shared plant now integrates copper heating, positive/absolute shaft work and squared
torque at every 0.5 ms physics step. It uses the **applied saturated motor torque**
and the same copper-loss equation as the existing thermal model:

```text
Iq = shaft_torque / (1.5 * Ke)
copper_power = sum(1.5 * phase_resistance * Iq²)       [W]
positive_mechanical_power = sum(max(torque * rotor_speed, 0)) [W]
absolute_mechanical_power = sum(abs(torque * rotor_speed))   [W]
```

Copper power measures motor winding heat, not total battery power. Positive
shaft work assumes no regeneration credit; inverter/electronics losses are not
modeled. These metrics must not be described as measured electrical efficiency.

New runs use `walk_energy_v3`, adapting equations (2)–(4) of
[Adaptive Energy Regularization for Autonomous Gait Transition and Energy-Efficient Quadruped Locomotion](https://hybrid-robotics.berkeley.edu/publications/ICRA2025_Energy_Efficient_Quadruped_Locomotion.pdf).
The paper balances bounded motion/energy rewards and multiplies them by an
exponential auxiliary-cost factor. We use:

```text
P_proxy = copper_power + absolute_mechanical_power
energy_score = exp(-P_proxy / (200 J/m * useful_speed + 100 J/rad * useful_yaw_rate))
reward = dt * (tracking_xy + tracking_yaw + energy_weight * energy_score) * exp(-aux_cost)
```

The absolute work is integrated at physics substeps on both CPU and GPU,
including braking work; it is not approximated from the last torque sample.
The sum with copper power is an effort/heat proxy, not measured battery power.
The paper's shaft-power term alone would miss heat from holding stalled motors.

For a translation command, useful speed is actual planar velocity projected
along that command, clamped from zero to the commanded speed. The same rule
applies to signed yaw commands. This includes backwards and sideways walking;
uncommanded sideways drift, reverse motion or excess speed cannot improve the
energy denominator. If a moving command has zero useful progress, its energy
score is zero, even at zero power. For neutral commands the score is instead
`exp(-P_proxy / 20 W)` so standing still is appropriate. Existing moving-command
deadbands apply (0.03 m/s translation, 0.1 rad/s turning).

The 200 J/m and 100 J/rad scales correspond to 20 W at 0.1 m/s or 0.2 rad/s,
chosen as initial scales for QuadBot's lower commanded speeds. They are not
calibrated optimal values. `--energy-weight=1` is the default; zero disables the
energy bonus for comparison. Larger weights do not necessarily improve walking:
the paper reports loss of tracking and immobility at excessive weighting.
Temperature remains an auxiliary cost, independent of this weight.

The component rates/costs are:

| Term | Rate contribution |
| --- | --- |
| XY tracking | `exp(-(velocity_error / .12 m/s)²)` |
| Turning tracking | `.5 * exp(-(yaw_error / .4 rad/s)²)` |
| Efficiency | `energy_weight * energy_score`, bounded from zero to the weight |
| Temperature | `-2 * mean((max(T - 30°C, 0) / 20°C)²)` |
| Motion quality | Costs for tilt, height error, vertical/roll/pitch motion, contact-foot sliding, abrupt rotor targets and soft hinge/differential limits; nominal-pose regularization at rest. |

Air-time and flight terms are removed, allowing contact timing to emerge.
Separate copper/work/torque and stalled-power penalties are replaced by the
bounded efficiency term; they are not stacked on top of it. Auxiliary cost is
the magnitude of the remaining negative terms. Running rewards stay nonnegative.
Falling, linkage failure or a motor above 55 °C instead receives **exactly -25** and terminates
the episode. A normal episode time limit receives no failure penalty.
`reward_terms` contains the component rates; `raw_running_reward` records their
signed integral for diagnostics, while `running_reward` and `termination_reward`
sum to the actual returned reward. Feet may move away from their starting
positions: the planted-foot displacement termination does not apply to walking.

This is a QuadBot adaptation, not an exact reproduction: we retain coupled rotor
actions, the thermal model and failure penalties, count copper loss, use commanded
progress, handle neutral commands explicitly, and use our own speed/energy scales.
LSTM remains optional instead of requiring the paper's explicit history policy.

Existing checkpoints retain their saved reward version. `walk_energy_v1` keeps
its signed-sum reward and -2 failure penalty. `walk_energy_v2` keeps its positive
benefit/cost ratio, doubled copper/temperature costs, unproductive-power cost,
air-time/flight terms and -25 failure penalty. Missing reward-version fields are
inferred from the saved contract. **Start a new run to use v3**; resuming an old
run keeps its recipe, history length and learned policy.

The inspected `walk-20261026-201827-457588` run ended at 6,116,352 transitions
with mean episode duration about 0.35 s and copper loss about 194 W. Earlier,
longer episodes consumed roughly 140–170 W and had more negative total returns
than the later rapid falls. That suggests an early-termination incentive, though
undiscounted episode totals alone do not establish the cause of PPO collapse.
The new recipe addresses that incentive and wasteful power use; short software
tests do not establish that a useful gait will converge.

Periodic qualification tests seven commands from both 20 °C and 40 °C, pairing
policy and zero-action hold with the same seed. It requires surviving the full
test, sustained tracking over the last second, mean total copper power ≤20 W
and peak temperature ≤50 °C. Tracking tolerances are the larger of 0.02 m/s or
25% commanded translation speed, and 0.06 rad/s or 25% commanded turning rate.
A cool stationary robot fails nonzero motion commands. Two consecutive reports
with ≥75% successful cases expand the command range by 1.25, up to full range.

The 20 W/50 °C qualification budgets are explicit starting targets, not measured
safe continuous operating limits. They are configurable for **new runs** with
`--max-copper-power-w` and `--max-temperature-c` (thermal budget must lie strictly
between 40 and 55 °C). They gate curriculum/evaluation; the reward weights above
remain fixed by the saved reward version. Resume uses the run's recorded settings. The
55 °C physical episode cutoff remains independent of these qualification budgets.

## Validation and limitations

```sh
bazel test --config=pc //sim:walk_test //sim:walk_training_view_test
```

Tests cover seeded Gymnasium behavior, the actual differential mapping,
chassis-frame velocity, independent stepping, energy/temperature costs,
thermal/fall penalties versus normal time limits, wasteful-power costs,
legacy reward compatibility, rejection of stationary/hot/inefficient policies, actual
PPO weight updates, checkpoint reload/resume, GUI/headless trajectory agreement,
live snapshot replay, checkpoint refresh boundaries and owned-worker shutdown.

A short training run checks the software path; it does not establish a useful
gait. Validate repeated command transitions, disturbances, longer warm runs,
energy and thermal behavior across seeds before declaring success. The thermal
constants, friction and actuator approximations still need physical validation.
Training and playback are simulation-only; no robot execution or deployment is
performed. The existing pose trainer remains available as `//sim:train`.

## CPU/GPU allocation and measured throughput

The following CPU-backend measurements predate GPU physics and DroQ. On the
inspected Ryzen 7 5800H (8 cores / 16 threads) with RTX 3050 Laptop GPU (4 GiB),
the allocation for those measurements was:

| Work | Device / concurrency |
| --- | --- |
| MuJoCo stepping, differential/voltage control, heat, rewards, observations | CPU, independent spawned worker processes |
| Small MLP action inference, PPO gradients, pose initialization | CPU, one Torch thread; inference batches all workers |
| Qualification tests and preview physics | CPU, separate single environments |
| Robot image | OpenGL/EGL; verified NVIDIA RTX 3050 renderer on this machine |
| Qt, charts, checkpoint/log I/O | CPU |

[Stable-Baselines3 recommends CPU PPO and parallel environments for MLP policies](https://stable-baselines3.readthedocs.io/en/v2.7.0/modules/ppo.html).
[MuJoCo's GPU guidance](https://mujoco.readthedocs.io/en/stable/mjwarp/index.html)
distinguishes single-world latency from batched throughput and notes that
CPU/GPU transfers can bottleneck RL. The current locked PyTorch wheel is CPU-only.
The measurements below cover CPU Gym environments and PPO. The GPU backend
described above now ports the motor/thermal physics to MJWarp and runs batched
qualification. GPU rendering alone does not imply GPU physics. Other machines
may select another GPU or software OpenGL.

Measured on 2026-10-26, 2,560 transitions per case after one warm-up update,
512 transitions per PPO update, no window, checkpoint writes or qualification:

| Workers | Walking transitions/s | Pose transitions/s |
| --- | ---: | ---: |
| 1 | 180 | 193 |
| 2 | 320 | 351 |
| 4 | 601 | 650 |
| 8 | 766 | 1,074 |

Four workers improved measured walking throughput by 3.3×; eight by 4.3×.
PPO optimization accounted for 0.2–2.4% of measured training time across these
cases; rollout collection (physics, resets, IPC, batched inference) dominated.
These are short throughput measurements, not learning-quality comparisons or a
CPU-versus-CUDA benchmark. Different sampled trajectories/reset rates and laptop
load affect results. Four workers are the interactive compromise, while eight
were the fastest measured headless setting. The raw measurements are in
[the benchmark record](../sim/benchmarks/cpu_training_20261026.json).

```sh
# Same one-command UI; automatic worker count:
bazel run --config=pc //sim:walk_train
# Explicit count for a new run (also supported by //sim:train):
bazel run --config=pc //sim:walk_train -- --num-envs=2
# Repeat the comparison on this machine:
bazel run --config=pc //sim:benchmark_training -- --output=/tmp/training-compute.json
```

The window and `status.json` show the active CPU worker count and policy device.
`--rollout-steps` remains the total number of transitions per update. The worker
count must divide it and leave at least two steps per worker. Changing the count
changes each trajectory's GAE horizon, so compare qualification results before
claiming better learning. Seeds are `seed + worker index`. Curriculum changes
reach every worker; the live replay samples worker zero. One-worker mode uses
`DummyVecEnv`; multiple workers use `SubprocVecEnv` with spawn, and close with
the trainer. Workers do not render or start nested BLAS thread pools.

Resume preserves the saved worker count and rollout settings. Older manifests
without a worker count remain single-environment runs, preserving their PPO
buffer configuration. Start a new run to use the new defaults. Motor calibration,
physical coupling, energy penalties and temperature limits are unchanged.
