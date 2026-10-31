# Research-based planted-foot pose training

Researched 2026-10-26. The requested task is chassis translation and rotation
with four planted feet. The following established quadruped projects supply
training patterns, not pretrained weights compatible with QuadBotNG.

## Established approaches

| Project | Policy and objective | Useful part for QuadBotNG |
| --- | --- | --- |
| ETH / RSL, *Learning to Walk in Minutes* (ANYmal) | PPO; the public base configuration uses separate 512/256/128 ELU actor/critic networks. Commands specify horizontal velocity and turning/heading. Tracking rewards accompany effort, action-rate, collision and foot-motion terms; terrain difficulty can increase through a curriculum. | Command-conditioned PPO, small-to-large goals, explicit tracking and regularization terms. Its walking rewards and independent joint actions do not match this task. |
| Google DeepMind / collaborators, *MuJoCo Playground* (Go1) | Joystick locomotion, balance and recovery tasks. The current Go1 PPO configuration uses 512/256/128 actor/critic networks and privileged critic observations. The joystick environment runs a 50 Hz policy, applies offsets around a home pose, changes commands during episodes, and includes sensor/dynamics variation. | Nominal-pose offsets, command changes, paired evaluations and robustness variation. Keep our voltage plant and coupled rotor coordinates. |
| Berkeley / CMU, *Rapid Motor Adaptation* (A1) | A base policy plus a history-based adaptation module responds to changing terrain and dynamics. Training occurs in simulation and was transferred to the authors' A1. | Sensor history and variation are useful here; a dedicated adaptation module is a later experiment, not something already implemented by stacking four frames. |

Primary sources (rolling repository files were inspected on the research date):

- [Rudin et al., paper and reported ANYmal results](https://proceedings.mlr.press/v164/rudin22a.html),
  [legged_gym base task/PPO configuration](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py).
- [MuJoCo Playground project](https://playground.mujoco.org/),
  [Go1 task, actions and rewards](https://github.com/google-deepmind/mujoco_playground/blob/main/mujoco_playground/_src/locomotion/go1/joystick.py),
  [PPO configuration](https://github.com/google-deepmind/mujoco_playground/blob/main/mujoco_playground/config/locomotion_params.py),
  [dynamics randomization](https://github.com/google-deepmind/mujoco_playground/blob/main/mujoco_playground/_src/locomotion/go1/randomize.py).
- [Kumar et al., RMA project and paper](https://ashish-kmr.github.io/rma-legged-robots/).

The published hardware results apply to those robots. Network sizes, gains,
speed ranges and training times are not validated parameters for this robot.
This implementation independently adapts the patterns; it does not import
upstream environment code, models or weights.

## Implemented recipe

`//sim:train -- --profile=research` is the default for new runs. The existing
CPU MuJoCo / Gymnasium / Stable-Baselines3 stack and dependency lock suffice.

| Component | QuadBot adaptation |
| --- | --- |
| Goal | `[dx,dy,dz,roll,pitch,yaw]` relative to the settled starting chassis, with the same four foot anchors for the whole episode. |
| Actor and critic | Separate 128/128/128 ELU MLPs, chosen as a smaller CPU starting point. Both receive four frames of the existing 60 sensor/command values. No privileged pose input is added. |
| Actions | 12 normalized rotor offsets, FR/FL/BR/BL × I/O/Z; ±12 rotor radians through the existing 9:1 reduction and three-motor differential. The existing 150 ms filter and 6 rotor rad/s target slew limit remain. |
| Control | 50 Hz policy, 1 kHz voltage PD, 2 kHz physics, existing torque/voltage/thermal bounds. Electrical zero offsets and measured calibration are unchanged. |
| Learning | PPO, 512-step rollouts, batches of 64, up to five epochs, learning rate 3e-5, target KL .01, discount .99, GAE .95. These retain the conservative pose trainer settings rather than copying a massively parallel locomotion optimizer. |
| Initialization | Existing explicitly supervised geometric pose demonstrations, then PPO in physics. `--from-scratch` disables demonstrations. Playback uses the neural policy only. Demonstration fitting uses nominal dynamics; PPO introduces variation. |
| Commands | Sample existing feasible workspace directions at 50–100% of the current stage, with a 20% neutral-command probability. Resample every four seconds without resetting the body or anchors. Explicit reset commands and viewer-supplied commands bypass this sampler. |
| Curriculum | Start at 10% workspace; multiply by 1.5 after two consecutive evaluations with at least 75% success, capped at full size. New stage goals apply at the next reset or scheduled command. |

The workspace includes crouching, returning upright, forward/back/side shifts,
roll, pitch, yaw and combined poses. The full limits and their geometric checks
are described in [gym.md](gym.md). Interpolated goals and transitions still need
physical evaluation: endpoint feasibility alone is not proof of a reliable path.

`planted_v4` scores a **rate**, integrated over elapsed simulation time:

```text
p_tol = max(0.002 m, 0.25 * norm(command translation))
r_tol = max(0.008 rad, 0.25 * norm(command rotation))
e2 = (position_error / p_tol)^2 + (rotation_error / r_tol)^2
tracking = 4 / (1 + e2)
feet = 0.5 * fraction_of_feet_in_contact
anchor penalty = -2 * tanh(maximum_foot_displacement / 0.005 m)
effort = -0.01 * mean((voltage / voltage_limit)^2)
target rate = -0.02 * mean((rotor_target_rate / 6 rad/s)^2)
body motion = -0.02 * sum(base_generalized_velocity^2)
soft limits = -2 * mean(squared normalized intrusion into outer 5% of limit ranges)
```

Soft limits include both limited hinges and differential tendons (including
coupled coordinates). Failure still incurs a separate -1 penalty. The rational
tracking score has a nonzero tail and uses stage-dependent tolerances: ignoring
a small command is no longer almost optimal. Foot contact does not erase the
tracking signal. These weights are engineering choices to test, not published
weights validated on QuadBot. No air-time, swing-clearance, gait-phase or forward
velocity reward is included because those objectives encourage moving feet.

The `mild_v1` robustness profile varies foot **and floor** sliding friction by a
common factor U(0.8,1.2), and PD gains independently by U(0.9,1.1) at reset.
Foot variation matters because foot contact priority overrides floor properties
in this model. Each reset restores the original values before sampling; the
YAML/XML files are never changed. Observations add independent uniform noise
bounded by 0.01 rotor rad, 0.1 rotor rad/s, 0.01 gyro rad/s and 0.05 m/s² specific
force. Commands, previous applied targets and temperatures remain uncorrupted.
These are modest assumed training ranges, not measured hardware uncertainty.
Mass/inertia, transmission signs and geometry are not randomized.

## Evaluation and use

```sh
# Native training window, research recipe:
bazel run --config=pc //sim:train -- --run-dir=runs/pose-research --steps=100000

# Previous small-network/dense-v3 recipe for comparison:
bazel run --config=pc //sim:train -- --headless --profile=baseline \
  --run-dir=runs/pose-baseline --steps=100000

# Deterministic qualification plus hold / deterministic / sampled diagnostics:
bazel run --config=pc //sim:evaluate_policy -- runs/pose-research
# Separately measure nominal dynamics and full-workspace extrapolation:
bazel run --config=pc //sim:evaluate_policy -- runs/pose-research --nominal
bazel run --config=pc //sim:evaluate_policy -- runs/pose-research --full-workspace
```

`pose_suite_v2` compares deterministic policy actions with zero-action hold on
all 16 stage goals and 16 intermediate-amplitude cases (75%). It uses two fixed
seeds, paired between hold and policy. Success requires every sample in the
final half-second to meet position/orientation tolerances and have four contacts,
no physical termination, and maximum drift **anywhere in the episode** ≤5 mm.
The report includes tracking gain over hold, survival, duration, per-goal errors,
peak drift and failure reason. Very short diagnostic episodes use all available
samples; use at least three seconds for actual qualification. Intermediate
amplitudes test interpolation, not unseen task directions. This suite measures
fixed-goal settling; changing-command behavior is exercised in training and
the viewer but is not certified by the fixed-goal success percentage.

`config.json` records the reward, network, sampler, randomization and evaluation
versions. Resume restores them; it does not silently upgrade existing policies.
Older manifests get explicit original defaults. The CLI profile only applies
to new runs, and the GUI forwards it to its worker. The baseline still benefits
from the stricter success bookkeeping.

## Limits of the result

This is an implemented training recipe, not a claim of solved pose control or
hardware transfer. Compare multiple seeds and sustained evaluation before
concluding that a recipe improves learned performance. Supervised initialization
and PPO improvement must be reported separately. A stationary policy's survival
or a high aggregate return is not sufficient evidence of following commands.

Absolute translation/yaw and slip are not directly observable from the existing
IMU. Four-frame history does not solve that observability problem. Future work
can compare recurrent policies or an explicit estimator; a privileged critic
would also require a separate actor/critic observation contract. RMA itself is
not implemented here. Deep crouches remain difficult under the strict 5 mm
foot-centre displacement objective because rounded rigid feet can roll; the
criterion is deliberately not relaxed to manufacture success. Mechanical stops,
encoder references, voltage response and IMU mounting/fusion still need hardware
validation. No serial control, firmware flashing or robot deployment is added.

The bounded validation run `runs/pose-research-validation-20261026` used seed 42.
At 10% workspace, supervised initialization alone passed 31/32 three-second
fixed-goal evaluations (77.4% mean tracking-cost improvement over nominal hold).
After 2,048 PPO transitions, the curriculum reached 15% workspace and passed
32/32 cases (80.9% tracking-cost improvement over hold). These different goal
scales are not an apples-to-apples PPO improvement measurement. They demonstrate
the pipeline and small-goal behavior, not full-workspace success or hardware
transfer. A separate [walking task and live window](walking.md) now implement
velocity commands with explicit temperature and effort objectives.
