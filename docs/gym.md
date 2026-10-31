# Body-pose tracking with planted feet

`QuadBot-BodyPose-v0` is the initial Gymnasium task for QuadBotNG: follow a
requested chassis translation and roll/pitch/yaw offset while keeping all four
feet at their reset ground positions. It uses the existing floating MuJoCo
model, closed linkages, three-motor differential, collision geometry and motor
limits. Feet remain ordinary frictional contacts; they are not welded to the
floor. A local PPO trainer and live evaluation window are available; task
performance still needs a longer learning run and evaluation.

New runs use the research-based planted-foot recipe described in
[rl-body-pose.md](rl-body-pose.md): an ELU PPO policy, changing pose commands,
modest seeded dynamics/sensor variation and sustained tracking evaluation.
That document compares established quadruped RL projects and explains the
robot-specific adaptations. `--profile=baseline` retains the preceding recipe
for comparison; resumed runs always use their saved settings.

The foot spheres use the [rubber sock contact approximation](physical-model.md)
configured in `sim/robot_parameters.json`. Start a new training run after changing
contact parameters: the saved model fingerprint deliberately rejects checkpoints
from different physics settings. Existing running processes retain their loaded
model until restarted.

## Watch training

```sh
bazel run --config=pc //sim:train
```

This opens one native Qt window and starts a separate CPU PPO trainer. The left
side defaults to **Latest policy evaluation (real time, 30 s loops)**. It runs
an independent simulation using sampled actions from one checkpoint for
30 simulated seconds, then resets and loads the newest available checkpoint.
If no newer policy exists, it repeats with the same one. The right side plots
actual training episode return, body position/orientation
error, and maximum foot drift. The curves average the last 50 completed training
episodes. They appear after episodes finish and a PPO rollout is processed.
The trainer continues at full speed and samples exploratory actions. Evaluation
targets one 20 ms physics/control advance per 20 ms UI tick and renders at about
25 FPS; slow rendering or physics can reduce playback speed. Training is never
throttled to the preview. Select **Live training rollout (exploration)** to see
the actual accelerated training state, sampled at 10 Hz, instead.

The action selector defaults to **Sample policy actions (with exploration, like
training)**. This draws from the saved PPO policy's action distribution, as the
trainer does. It is a new rollout, not an exact replay of the trainer's random
samples. Choose **Mean policy actions (deterministic, no exploration)** to assess
the learned mean behavior. Early policies can move during sampled playback yet
barely move in deterministic mode: exploration contributes motion even when the
mean action remains close to nominal hold. Both modes show the maximum motor
target offset and use the same observation preprocessing and motor simulation.
Changing the action selector restarts the evaluation loop.

The green box marks the requested chassis pose and green spheres mark the foot
anchors. The goal label reports XYZ offsets in mm and roll/pitch/yaw in degrees.
Evaluation changes between crouch, lean, roll, shift and turn goals every four
simulated seconds within each 30-second loop. Goal changes retain the physical
state and original foot anchors; the policy's observation history receives the
new command. If a fall, displaced foot or overheating ends an episode, the
preview resets that episode and continues with the same policy for the remainder
of the loop. The loop counter and early-reset count are shown above the image.
Training's episode duration and saved configuration are unaffected.
Adjust the translation/orientation controls and click
**Restart evaluation with this command** to disable cycling and test a fixed
command. These controls affect evaluation only. A manual restart starts a new
30-second loop and checks for a newer checkpoint. An untrained mean
policy may barely move; live exploration is not evidence of learned tracking.

New training runs sample directions from the same 16 workspace goals used in the preview,
at 50–100% of the current curriculum stage, with 20% neutral commands. Commands
change every four seconds without resetting the feet. The
preview begins with a deep crouch followed by standing tall, then cycles through
different heights, translations, rotations and combined poses every four seconds.
The goals use:

| Transform | Goal magnitude |
| --- | --- |
| Forward/backward shifts | ±60 mm |
| Sideways shifts | ±30 mm |
| Yaw | ±12.6° |
| Roll | ±9.7° |
| Pitch | ±12.6° |
| Crouch | 50, 100 and 150 mm down |
| Combined poses | 20/10 mm diagonal shifts with roll, pitch and yaw |

The translation/rotation goals lower the body by 50 mm to leave leg extension
headroom. All targets pass an anchored-foot geometric check against the existing
joint/tendon limits and collision geometry. A physical reference trajectory
descends 150 mm and returns to standing using the voltage motor plant, with all
four feet in contact and without body-ground contact. The belly reaches roughly
2 cm above the ground. The rigid rounded feet roll about 2 cm during that reference
descent: this does **not** demonstrate the stricter 5 mm planted-foot reward or
a learned policy solving these goals. The reference solver runs only in tests;
training and playback have no inverse-kinematics assistance.

**Start a new training run for the expanded workspace.** Earlier saved runs
retain their action scale, observation scaling, height cutoff and goal sampler
when resumed. Their automatic preview uses the smaller targets and displays a
compatibility message. Reinterpreting their ±1-radian outputs as ±12 radians
would change the meaning of their learned actions.
New runs use `reward_version="planted_v4"`, which scores translation and
orientation together against stage-dependent tolerances. Earlier unversioned runs keep
their original reward when resumed; start a new run to change the objective.

**Stop training and save** requests a clean stop. Closing the window or Ctrl+C
also stops its trainer and saves the current policy. After the requested number
of training steps completes, the window stays open for evaluation until closed.
Default training length is 100,000 transitions, rounded up to complete 512-step
PPO rollouts. This is an initial experiment budget, not a guaranteed convergence
time. No hardware, account, cloud service or CUDA installation is used.

```sh
# Choose a new output directory and budget:
bazel run --config=pc //sim:train -- --run-dir=runs/balance-01 --steps=100000

# Continue that run for additional steps, using its recorded settings:
bazel run --config=pc //sim:train -- --resume=runs/balance-01 --steps=100000

# Train without a window:
bazel run --config=pc //sim:train -- --headless --run-dir=runs/balance-02

# Attach a viewer to a running or completed experiment:
bazel run --config=pc //sim:train -- --watch=runs/balance-02

# Compare nominal hold, deterministic policy and sampled policy on the saved curriculum:
bazel run --config=pc //sim:evaluate_policy -- runs/balance-02
```

New runs default to a timestamped directory under ignored `runs/`. A directory
is never silently overwritten. `--watch` starts no trainer and cannot stop an
independently launched one. `--no-render` shows the curves without requiring
MuJoCo's offscreen rendering. The standard window uses EGL for its robot image;
it respects an explicit `MUJOCO_GL` override.

Each run stores `config.json`, a snapshot of `motor_model.yaml`, immutable policy
ZIPs, the atomic `latest.json` pointer, `metrics.jsonl`, `status.json`, and the
latest sampled training state in `rollout.json`. Older runs without rollout
frames can still be viewed in evaluation mode. GUI
trainer output goes to `<run-directory>.log`. Every PPO update publishes a
checkpoint, including the optimizer state, so the viewer cannot read an
incomplete write. Resume restores the policy/optimizer and training step count
but starts fresh episodes; it is not a bit-for-bit continuation of simulator or
RNG state. Only one writer may use a run. Model fingerprints reject loading a
run after incompatible generated geometry/transmission changes.

The implementation uses [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
2.9.0 and the official PyTorch 2.8.0 CPU wheel, pinned in the existing dependency
lock. The research recipe uses one CPU environment, one PyTorch thread, separate
128×128×128 ELU actor/critic MLPs, 512-step rollouts, batches of 64, up to five
PPO epochs, a 0.00003 learning rate and target KL 0.01. Initial
action standard deviation is `exp(-2)` physical rotor radians: normalized standard
deviation is divided by the saved action scale so expanding the range does not
multiply exploration noise. Four observation
frames (240 values) supply short sensor history. Fixed feature scaling is shared
by training and evaluation; there are no separate running normalization files
to accidentally mismatch. This is a baseline configuration, not tuned evidence
of successful body-pose tracking.

## Run and verify

The existing Bazel-managed Python and locked dependencies are sufficient:

```sh
bazel test --config=pc //sim:imu_test //sim:gym_env_test

# Headless nominal rotor hold, zero body-pose command:
bazel run --config=pc //sim:gym_rollout -- --steps=250

# Preview with a requested 5 mm X shift and 0.03 rad roll:
bazel run --config=pc //sim:gym_rollout -- \
  --viewer --command 0.005 0 0 0.03 0 0

# Exercise resets and episode endings with random actions:
bazel run --config=pc //sim:gym_rollout -- --random-actions --steps=250

# Optional offscreen rendering on a host with EGL drivers:
MUJOCO_GL=egl bazel run --config=pc //sim:gym_rollout -- \
  --steps=50 --render-output=/tmp/quadbot-gym.png
```

The rollout executable uses zero motor-target offsets unless `--random-actions`
is given. It demonstrates the environment and nominal stance, **not a trained
body-pose tracker**: changing the command changes the policy input and reward,
not an existing inverse-kinematics controller. Viewer mode paces simulation;
headless mode runs as fast as the CPU permits.

## Python interface

Add `//sim:gym_env` to a Python Bazel target's dependencies:

```python
import gymnasium as gym
import sim.gym_env  # Registers QuadBot-BodyPose-v0.

env = gym.make("QuadBot-BodyPose-v0", episode_seconds=10)
try:
    obs, info = env.reset(seed=42, options={
        "command": [0.005, 0, -0.005, 0.03, 0, 0],
    })
    for _ in range(100):
        action = env.action_space.sample()  # Replace with your policy.
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            obs, info = env.reset()
finally:
    env.close()
```

Each instance owns its model, physics state and motor temperatures. CPU
`gym.vector.SyncVectorEnv` and process-based Gymnasium vector environments can
construct separate instances. Keep rendering disabled during training.
Use `gym.wrappers.FlattenObservation` for algorithms that require a flat vector;
the native observation is a dictionary. The provided PPO runner applies its own
fixed scaling and frame history. This is native CPU MuJoCo, not an MJX-batched
implementation. Both PPO launchers now use parallel CPU workers by default
(up to four with the GUI, eight headless), with CPU MLP training and OpenGL
rendering. See the [compute split and measured benchmark](walking.md#cpugpu-allocation-and-measured-throughput).
Use `--num-envs=1` for a single training environment. Resume uses saved settings;
older runs remain single-environment. `--rollout-steps` counts total transitions
across workers, so the per-environment horizon decreases as the count increases.

## Commands and reset

Commands are six numbers: `[dx, dy, dz, roll, pitch, yaw]`. Translation is in
metres along the **reset chassis axes**, and rotation is in radians relative
to the reset chassis orientation. Rotation uses `Rz(yaw) Ry(pitch) Rx(roll)`;
MuJoCo quaternions use WXYZ order. The body origin is `base_link`, not its COM.

For the base environment's default `goal_mode="mixed"`, seeded reset samples uniformly within
`command_range=(0.01, 0.01, 0.01, 0.06, 0.06, 0.06)`. These small starting
ranges can contain combinations outside the policy's rotor action limits. The
trainer uses `goal_mode="workspace"` and the tested goals in `sim/goals.py`.
`goal_mode="axis"` and `goal_mode="poses"` retain earlier small curricula. Set
`command_range=(0, 0, 0, 0, 0, 0)` for initial balance training, then widen
after learning to hold stance. Explicit commands may exceed the sampling range;
their feasibility is the caller's responsibility.

Reset places the robot in the extended `stand` pose with CAD main/crank offsets
of +0.09/-0.09 rad, leaving extension headroom. Passive linkage coordinates
remain consistent. It places the foot spheres just above the floor, then runs
0.5 seconds of physical rotor hold to settle. It requires four foot contacts,
no other floor contacts, healthy temperature and no MuJoCo warnings. It captures
the settled chassis pose and foot sphere centres as fixed episode references.
There is no startup suspension in this environment. Episode time starts at zero
after settling; heat generated during settling is retained.

`env.unwrapped.set_command([...])` changes the requested offset during an
episode without changing the reference pose or foot anchors. Fixed per-episode
commands are the default; a trainer can schedule smooth command trajectories.
Calling `step` after an episode ends requires `reset`. The research trainer adds
a four-second command sampler; explicit reset commands disable that sampler.

## Actions and dynamics

Actions are a float32 Box of shape `(12,)` in `[-1, 1]`, ordered
**FR I/O/Z, FL I/O/Z, BR I/O/Z, BL I/O/Z**. They specify rotor target offsets
from the reset's nominal motor targets; they are not independent leg-joint
positions. Default `action_scale=12.0` means ±12 rotor radians, before the 9:1
reduction and differential coupling. A 150 mm crouch needs approximately 9.7
rotor radians, leaving control headroom. The model's configured joint limits,
voltage limits and torque limits still apply. Targets are absolute
offsets, not integrated increments. Finite actions outside the Box are clipped;
wrong shapes and NaN/Inf are rejected.

The policy runs at 50 Hz. A 1 kHz rotor PD converts targets to voltage using
the `stance` gains in `sim/motor_model.yaml`; physics runs at 2 kHz. The motor
plant applies that file's voltage sign, AVR command/2 scaling, measured winding
resistance, back-EMF approximation, shaft torque saturation and lumped thermal
equations. The existing MuJoCo tendon transmissions apply the differential's
coupling and per-leg signs. Zero action retains the nominal rotor hold.

This training path runs Python directly against the physical model. It does not
run the production C++ controller, board startup, serial transport, encoder
quantization, watchdog or firmware timing. Temperature above 55 C ends an
episode instead of emulating board resets. Use the native controller simulator
for those integration checks. Motor-plant tests cover the same numeric reference
cases as `native/motor_plant_test.cc`. Hardware calibration is unchanged.

## Observations and the IMU

All dictionary values are independent float32 arrays (60 numbers total):

| Key | Size | Meaning |
| --- | ---: | --- |
| `command` | 6 | Requested translation and RPY offset. |
| `motor_position` | 12 | Ideal rotor angle minus nominal target, radians. |
| `motor_velocity` | 12 | Ideal rotor velocity, rad/s. |
| `imu` | 6 | Chassis gyro XYZ, then specific force XYZ; rad/s and m/s². |
| `motor_temperature` | 12 | Continuous modeled motor temperature, °C. |
| `previous_action` | 12 | Last clipped normalized action; zero after reset. |

The [simulated IMU](physical-model.md#simulated-imu) uses native MuJoCo gyro
and accelerometer sensors. A supported upright accelerometer reads roughly
`[0, 0, 9.81]`, while free fall reads zero. Its output includes motion-induced
acceleration; it cannot simply be treated as the gravity direction during motion.
The base environment's default sensor is ideal. The research trainer adds the
small seeded uniform sensor noise documented in [rl-body-pose.md](rl-body-pose.md).
Bias, drift and latency are not modeled.

Absolute chassis pose, foot positions and contact states are used for scoring
and diagnostics only, not fed to the policy. An IMU does not directly measure
absolute position or yaw. This is therefore a sensor-based, partially observable
task: history/recurrent policies or a later explicit state estimator may be
needed, particularly for changing commands and detecting slip. Hardware encoder
reference establishment and IMU fusion remain separate future work.

New policies use `observation_version="workspace_v2"`: command normalization
scales are `[0.06, 0.03, 0.15, 0.2, 0.25, 0.25]` (metres/radians), and rotor
positions are divided by 12. This keeps deep-crouch commands and full-range
rotor positions distinct instead of clipping them. Saved older policies retain
their original observation scaling.

## Rewards and episode endings

Each step reports reward components in `info["reward_terms"]`. The positive
terms of the older `tracking_v2` reward are gated by all four foot contacts and an exponential penalty on mean
squared foot displacement from the fixed anchors (5 mm scale):

- Pose tracking: `4.5 * exp(-0.5 * ((position_error / 0.003 m)^2 + (rotation_error / 0.01 rad)^2)) * planted`.
- Planted feet: `0.5 * planted`.
- Penalties for normalized squared voltage (0.01), action change (0.02), and
  squared chassis translational/angular velocity (0.02).

The sum is multiplied by elapsed simulated seconds. A failure adds a -1 penalty.
These initial shaping weights require evaluation during learning; they do not
establish that a policy will solve the task. Foot displacement uses sphere
centres, not a model of deformation or microscopic contact-patch slip.

The new default trainer uses `planted_v4`; its exact terms and motivation are
documented in [rl-body-pose.md](rl-body-pose.md). It uses a rational coupled
tracking score, separate contact/maximum-drift terms, physical rotor target-rate
cost and soft penalties near both hinge and differential tendon limits.
The previous trainer recipe, selectable with `--profile=baseline`, uses
`dense_v3`: `4/(1 + position_error/.02 + rotation_error/.1)`, a half-point
contact fraction and a bounded mean-foot-drift cost. Earlier saved reward
versions remain available for checkpoint compatibility.

The original `legacy_v1` reward scored position and orientation independently
with 20 mm and 0.1 rad tolerances, plus a planted-feet reward of 1. Ignoring the
small curriculum commands earned about 94–98% of the theoretical maximum.
`tracking_v2` removes that near-perfect reward for holding still. The physical
small-pose feasibility test also checks that following each earlier goal earns at least
1.5 times the return of nominal hold over the same interval. This validates the
incentive, not PPO convergence.

`terminated=True` means temperature above 55 C, falling (base below the configured
`min_base_height`, now 0.06 m for new runs instead of 0.12 m,
tilt over 60 degrees, or a non-foot floor contact), any foot drifting over 5 cm,
or linkage closure error over 5 mm. `truncated=True` means the configured time
limit, rounded up to a policy step (10 s by default). Numerical solver warnings
or non-finite physics raise an error so invalid rollouts are not silently learned
from. Time-limit truncation and physical failure remain distinct for RL bootstrapping,
following the [Gymnasium environment contract](https://gymnasium.farama.org/introduction/create_custom_env/).

Diagnostics include position/orientation error, each foot's displacement and
contact flag, maximum temperature, linkage error and termination reason. The
existing CAD alignment, collision and motor/thermal approximations still need
hardware validation. A successful rollout is software validation, not evidence
of sim-to-real transfer.

## Validation

Verified with the repository's locked MuJoCo/Gymnasium versions:

- `gym_env_test`: Gymnasium checker, seeded trajectories, copied observations,
  command frames, IMU observations, differential coordinates, nominal hold,
  vector instances, thermal failure, foot displacement and time limits.
- `imu_test`, `robot_model_test`, `native_runner_test`, `controller_viewer_test`,
  `dashboard_test` and `native_test` pass after the sensor integration.
- A 500-step nominal-hold rollout reaches the 10-second time limit with reported
  position, orientation and foot errors below the CLI's four-decimal precision.
- A 250-step seeded random-action rollout exercises ten foot-displacement
  episode endings and resets without a solver failure with the original ±1 rad
  action range; it is not a full-workspace tracking benchmark.
- EGL rendering produces a 640×480 RGB preview.

`//sim:training_test` additionally checks real PPO weight updates, live training
snapshot replay, 30-second evaluation loops, goal changes without moving foot
anchors, checkpoint updates only at loop boundaries, early-failure resets, policy reload
equivalence, optimizer/step resumption, Qt checkpoint evaluation, trainer child
cleanup, SIGTERM checkpoint saving, and refusal to overwrite an existing run.
It compares Qt preview observations and physical trajectories with identical
headless policy rollouts in both sampled and deterministic modes, and checks
that older checkpoints retain their original reward, action, observation and
height-limit configuration.
Short training tests verify the pipeline, not convergence to a solved task.
No physical robot execution or hardware deployment was performed.
