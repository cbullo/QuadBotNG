# Simulator selection for QuadBotNG

Research date: 2026-10-25. Decision: **use MuJoCo as the initial physics engine**,
with MJCF as the simulation model, a native C++ integration for controller
validation, and a Gymnasium interface for initial learning experiments. Plan
for MJX / MuJoCo Playground when GPU training becomes useful.

This is a source-based engineering selection, not a benchmark result. No engine
was installed and no robot model was validated during this research. Sources
below are official project documentation/repositories. Rolling `stable`,
`latest`, `main` and `develop` pages can change; pin mutually compatible releases
when implementing the prototype, and verify the features on those releases.

## Requirements that drive the choice

The initial learning use case is assumed to be locomotion/control from onboard
state observations. Vision and photorealistic scenes are possible future needs,
but have not been requested as the first milestone.

- The user confirms a **three-motor differential per leg**, twelve motors total.
  Motor coordinates must remain distinct from leg and linkage coordinates.
- The existing model contains a closed linkage for each leg: five tree joints
  and an additional closure connection per leg, rather than three independent
  serial joints. CAD joint count is not actuator count.
- The controller commands voltage, scaled by 512 on the wire, and receives
  encoder feedback. An ideal joint-position motor would conceal important
  actuator dynamics and coupling.
- Existing C++ control code should run on PC against a simulated board layer.
  AVR register, PWM and I2C code need behavioral modeling, not x86 compilation.
- We need repeatable resets, explicit simulation time, contacts, joint stops,
  diagnostics, parameter variation, and an eventual path to many environments.
- Model correctness and integration effort take priority over advertised peak
  FPS. No published generic throughput number predicts this constrained model.

Local `lspci` reports an NVIDIA RTX 3050 Mobile and AMD integrated graphics.
GPU memory and working CUDA access were not established: `nvidia-smi` failed in
the execution environment. CPU development must therefore be a viable starting
point. This does not preclude training on another workstation or rented GPU.

## Comparing the candidates

The suitability judgments in the last column are specific to this repository.

| Candidate | Relevant capabilities / limitations | Assessment |
| --- | --- | --- |
| **MuJoCo + MJX / Playground** | Equality constraints for linkage closure, tendon transmissions, native API and Python bindings; CPU workflow plus accelerator implementations and locomotion environments. | **Selected.** Best combination of mechanical expressiveness, C++ reuse, small initial setup and a training path. |
| **Isaac Lab / Isaac Sim** | Extensive GPU robot-learning, actuator, sensor and scene infrastructure. PhysX closed loops are possible through joints excluded from the articulation tree. Full Isaac Sim has substantial GPU requirements. Isaac Lab also has newer backend integrations. | Strong alternative for large-scale sensor/vision work; more infrastructure and asset integration than needed first. |
| **Genesis** | Robot-learning-oriented simulator; current rigid solver documents connect, weld and joint equality constraints, with batched model/state support. | Serious alternative, not dismissed as incapable of loops. For this project, I prefer MuJoCo's direct native integration and explicit tendon machinery. |
| **Newton** | Warp-based framework with several solver backends, including MuJoCo Warp; accepts MJCF, URDF and USD. Its FAQ describes the Isaac Lab integration as experimental. | Worth monitoring; a useful later integration route. Starting with MuJoCo avoids another framework before it provides a concrete benefit. |
| **Gazebo Sim** | General robotics system simulation, plugins, sensors, transport and interchangeable physics engines. Existing repository fragments use Gazebo conventions. | Prefer for later ROS/system/sensor integration. The unfinished local adapter provides little reason to make it the primary learning engine. |
| **PyBullet / Bullet** | Native C++ engine and Python robotics/RL interface; established examples and straightforward programmatic control. | Viable baseline. MuJoCo offers a clearer documented path from the chosen transmission model to batched training. No speed superiority is claimed without measurement. |
| **Webots** | General robot simulation with controller/supervisor workflows and learning examples. A legacy PROTO export exists here. | Useful for interactive robot-system work; not my first choice for this project's later batched learning architecture. The export is not a validated simulator. |
| **RaiSim** | Robotics simulator with an activation-key licensing process. | Not selected: no identified requirement justifies an additional licensing dependency over the open alternatives. |

Sources for the comparison:

- MuJoCo: [modeling](https://mujoco.readthedocs.io/en/stable/modeling.html),
  [native programming](https://mujoco.readthedocs.io/en/stable/programming/index.html),
  [MJX](https://mujoco.readthedocs.io/en/stable/mjx.html),
  [Playground](https://github.com/google-deepmind/mujoco_playground).
- Isaac: [Lab overview](https://developer.nvidia.com/isaac/lab),
  [explicit actuator models](https://isaac-sim.github.io/IsaacLab/develop/source/concepts/actuators.html),
  [closed-loop rigging](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/rig_closed_loop_structures.html),
  [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
- Genesis: [rigid solver](https://genesis-world.readthedocs.io/en/latest/user_guide/theory/rigid_solver/index.html),
  [constraints](https://genesis-world.readthedocs.io/en/v1.4.0/user_guide/theory/rigid_solver/constraints.html),
  [solver options](https://genesis-world.readthedocs.io/en/v1.4.0/api_reference/engine/solvers/rigid_solver.html).
- Newton: [official FAQ](https://newton-physics.github.io/newton/stable/faq.html).
- Gazebo: [Sim overview](https://gazebosim.org/libs/sim/),
  [physics abstraction](https://gazebosim.org/libs/physics/).
- Bullet: [official repository](https://github.com/bulletphysics/bullet3).
- Webots: [official examples documentation](https://github.com/cyberbotics/webots/blob/master/docs/guide/samples-howto.md).
- RaiSim: [licensing](https://raisim.com/sections/License.html).

Full Isaac Sim's currently published workstation table lists an RTX 4080 and
16 GB VRAM as the minimum GPU configuration. The local RTX 3050 Mobile falls
below that listed GPU model. This strengthens the CPU-first choice; it is not
a claim that every headless Isaac Lab/Newton workflow requires RTX rendering.
[Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

## Physics, environment API and trainer are separate choices

Gymnasium defines the environment contract (`reset`, `step`, observations,
actions, termination and truncation); it is not itself a physics solver.
Stable-Baselines3 can train against custom Gymnasium environments. Use this
combination for an understandable first baseline, likely PPO, after mechanics
are validated. [Gymnasium API](https://gymnasium.farama.org/api/env/),
[SB3 custom environments](https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html)

For accelerated locomotion training, use MuJoCo Playground's task/training
patterns rather than expecting a Python Gymnasium wrapper to become a GPU
simulation automatically. Playground provides locomotion tasks and both JAX
and Warp implementation paths. A custom robot, actuator model, observations
and reward still need implementation.
[Playground repository](https://github.com/google-deepmind/mujoco_playground)

Avoid starting from legacy Isaac Gym: NVIDIA has deprecated it in favor of
Isaac Lab. Also avoid choosing old Brax physics pipelines as the foundation:
its current README directs new simulation work to MJX/MuJoCo Warp and new tasks
to Playground, while retaining `brax/training`.
[Isaac Gym notice](https://forums.developer.nvidia.com/t/isaac-gym-deprecation-transition-to-isaac-lab/322978),
[Brax maintenance notice](https://github.com/google/brax/blob/main/README.md)

## Why MuJoCo fits the differential and closed linkage

MuJoCo can close a kinematic tree using equality constraints. These are solver
constraints with compliance/error, not exact geometric elimination; closure
drift and convergence must be measured. Use ordinary tree joints wherever
possible and closure constraints only where the mechanism requires them.
[Constraint formulation](https://mujoco.readthedocs.io/en/stable/computation/index.html#equality)

Fixed tendons define linear combinations of scalar joint coordinates, making
them candidates for representing a geared differential transmission. They are
abstract transmissions and need not represent a physical cable. If the actual
mapping is nonlinear, use the correct kinematic mapping and force Jacobian
instead of forcing it into a linear tendon model.
[Fixed tendons](https://mujoco.readthedocs.io/en/stable/XMLreference.html#tendon-fixed)

For motor coordinates `m = f(q)` and motor torques `tau_m`, preserve power via
`tau_q = (df/dq)^T tau_m`. Check the existing theta/gamma/z equations and signs
against physical geometry; do not copy the differing control-mixing formulas
as unquestioned mechanics. A single scalar joint gear ratio does not capture
the three-motor coupling.

Start with an explicit linkage model. A reduced-coordinate training model may
be useful later, but must reproduce the measured kinematics, work/energy and
effective inertia. Do not delete passive links just to obtain twelve joints.

Use MJCF as the canonical simulation description and reuse the existing STL
assets and measured configuration where valid. An automatic URDF import is a
starting aid, not evidence that Gazebo-only closures and transmissions survived.
Validate inertia tensors, mesh scales, joint axes and stops, and use appropriate
simple/convex collision shapes. Preserve CAD visuals separately from collisions.

## Actuator and controller integration

The proposed development path is:

```text
Existing C++ controller (PC build)
    -> six simulated two-motor board interfaces
    -> twelve voltage-driven motor models
    -> differential transmission and linkage dynamics in MuJoCo
    -> encoder/board feedback to the same controller
```

Model aggregate FOC behavior initially: voltage saturation, torque/speed
response, damping/friction, reflected inertia, sampling, command latency and
encoder quantization/wrap. Add current dynamics, backlash, thermal behavior and
faults when identified parameters or test requirements justify them. A simple
motor equation can be a baseline approximation, but it needs calibration for
these BLDC motors; it is not automatically a validated FOC model.

Use simulation time instead of host sleeps and wall clocks. The source uses
nominal 5 ms controller periods; evaluate smaller physics substeps and time-step
convergence rather than assuming a fixed step is accurate. Keep electrical,
control and policy rates explicit.

The learning path should batch the numerical plant/control model on the
accelerator, not launch thousands of the current threaded C++ controllers.
Share parameters and mapping definitions, then compare traces against the
native controller simulation. An arbitrary C++ callback or firmware routine
will not automatically execute inside a compiled JAX/Warp batch.

Choose the eventual policy action space deliberately: direct motor voltage
or targets/residuals around a validated low-level controller. Do not train on
ideal joint torques and silently deploy the output as voltage.

Deployment observations must be available or estimable on the real robot.
Inventory actual sensors before using base velocity/orientation, contact or
terrain information as policy inputs. Simulator ground truth may be used for
rewards or a privileged training critic without exposing it to the deployed
actor. Train on the PC/GPU; the Odroid needs only the chosen inference runtime
and control interfaces, not the training simulator.

## GPU compatibility and decision gates

MJX's documented feature table includes the equality constraints and tendon
transmissions needed for the proposed model. It does not imply identical CPU
and GPU trajectories or throughput. Test the exact model on pinned versions.
MJX-Warp targets NVIDIA GPUs and does not provide automatic differentiation;
MJX-JAX provides a differentiable path with its own feature limits. Ordinary
model-free PPO does not require differentiating through physics.
[MJX feature table](https://mujoco.readthedocs.io/en/stable/mjx.html#feature-parity)

Before building a complete training system:

1. **One-leg prototype:** verify dimensions, linkage closure, motor-coordinate
   mapping and signs against known physical poses. Drive each motor separately.
2. **Mechanical checks:** check limits, static load response, work/energy
   consistency and closure residuals across step sizes and solver tolerances.
3. **Full robot:** validate free fall/contact, stance and perturbation response;
   confirm exactly twelve actuators and the correct board/channel assignment.
4. **Controller integration:** run existing initialization/stand-up through the
   simulated boards with repeatable state resets and deterministic scheduling.
5. **Small learning baseline:** validate the Gymnasium environment and train a
   limited task before adding gait rewards, terrain curricula and randomization.
6. **GPU feasibility:** load the same mechanism in MJX, compare control/state
   traces within agreed tolerances, then measure throughput, constraint error
   and VRAM for several batch sizes. Select Warp/JAX and training hardware from
   those measurements, not generic FPS claims.

If a selected release cannot model the required mechanism reliably, revisit
the solver/model choice before simplifying away robot behavior. Isaac Lab is
the main alternative if complex rendered perception becomes the dominant
requirement; Genesis/Newton deserve a focused prototype if measurements expose
a concrete advantage. None of those possibilities blocks starting with MuJoCo.

## Deferred work

Odroid libc-compatible cross-toolchain restoration is recorded in
[TODO.md](../TODO.md). It is independent of PC simulator development and should
not be resumed implicitly while implementing the simulator.


## GPU port investigation — 2026-10-26

**Initial source audit (superseded by the measured implementation below):**
a batched MuJoCo Warp implementation is a promising candidate
for this RTX 3050 Laptop GPU, but a speedup over the current parallel CPU trainer
has not been measured. This investigation used source inspection and small
standard-library calculations only: no Bazel calls, package installation, GPU
allocation, physics benchmark or changes to the training physics/dependencies.
The user's training could continue throughout.

Earlier device uncertainty was resolved during the CPU parallelization work:
the machine has a Ryzen 7 5800H (8 cores / 16 threads), RTX 3050 Laptop GPU with
4,096 MiB VRAM, driver 580.178.04, and working NVIDIA EGL rendering. That verifies
graphics access, not a tested Warp/CUDA installation. Warp's documented driver
minimum is 525 for CUDA 12 builds and 580 for CUDA 13 builds; the observed driver
meets those version minima. Pin and test a compatible package set separately.
[Warp installation requirements](https://nvidia.github.io/warp/v1.17/user_guide/installation.html)

### Compatibility with the actual current robot

This audit reads `sim/robot_model.py`, generated `sim/mujoco/quadbot.xml`,
`gym_env.py`, `gym_motor.py`, `walk_env.py`, and the installed SB3 collector.
The current linkage representation is **eight scalar tendon equalities**, not
four point-connect constraints; preserve that parallelogram branch selection.
The generated model has 20 hinges plus a floating base (`nq=27`, `nv=26`),
24 fixed tendons, 12 actuators, 110 geoms (84 primitive collision geoms including
the floor), and gyro/accelerometer/actuator-position/velocity sensors.

| Model feature | Source-level GPU assessment |
| --- | --- |
| Three-motor differential, 9:1 reduction, per-leg signs | Preserve the same fixed-tendon coefficients and actuator ordering. Tendon transmissions/equalities are supported by MJWarp. |
| Passive linkage, joint/tendon limits, friction loss | Covered by the documented feature categories; physical trace agreement still requires testing. |
| Twelve tendon armatures and tendon damping | Explicit implementations exist in Warp's smooth/passive dynamics. Do not replace coupled rotor inertia with independent hinge armatures. |
| `implicitfast`, Newton, dense Jacobian, elliptic friction | Supported categories; the model does not request the unsupported implicitfast midpoint feature. |
| Plane, box, cylinder, capsule and sphere collisions | Supported by Warp. Pure MJX-JAX has collision-pair restrictions involving cylinders, which need separate auditing for this model. |
| IMU and actuator sensors | Supported sensor categories; preserve the final forward pass and chassis-site frame conventions. |

Sources: [feature table](https://mujoco.readthedocs.io/en/stable/mjx.html#feature-parity),
[Warp compatibility exceptions](https://github.com/google-deepmind/mujoco_warp#mujoco-api-compatibility),
[tendon armature implementation](https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/smooth.py),
[tendon damping implementation](https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/passive.py).
These are rolling upstream sources inspected on the date above, not an installed
or validated release. Resolve a compatible release/commit before implementation.

### What must be ported

A policy transition currently executes 40 physics substeps at 0.5 ms each.
Within those substeps, rotor PD updates voltage every second substep (1 kHz);
back-EMF, torque saturation, copper heat, positive work and temperature update
every substep (2 kHz). Policy observations/actions run at 50 Hz.

Port that entire numerical loop to batched Warp kernels around `step1`/`step2`,
then capture it in a CUDA graph. Upstream provides the same two-phase stepping
structure with implicitfast integration. Keep controller state, held voltage,
per-motor temperatures and energy accumulators on the GPU between calls.
[Warp split-step implementation](https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/forward.py)

Also port action filtering/rate limits, observations/frame stacks, rewards,
contacts/slip/air-time, command sampling, domain randomization and reset masks.
Reset currently includes 1,000 physical settling substeps; handling different
worlds' resets without advancing unaffected worlds or returning each reset to
Python needs explicit design. Preserve early thermal termination within a
policy transition and use actual elapsed time in the reward. Cache IDs and
static configuration on CPU; do not call named MuJoCo accessors inside kernels.

The installed SB3 collector stores NumPy rollout arrays and converts action,
value and log-probability tensors back to CPU. Merely selecting `device="cuda"`
is therefore not a fully GPU-resident training pipeline. A staged prototype can
retain CPU PPO and exchange one observation/action batch per policy transition,
then measure whether transfers matter. The intended final arrangement is Warp
physics plus CUDA PyTorch inference/rollout storage/PPO, with shared tensor views;
CPU handles UI, logging, checkpoint files and occasional preview snapshots.
The currently pinned Torch wheel is CPU-only, so that final arrangement also
requires a separate dependency profile. [Warp/PyTorch sharing and stream rules](https://nvidia.github.io/warp/latest/user_guide/interoperability/pytorch.html)

### Numerical and memory gates

Warp's model loader clamps solver tolerance to at least `1e-6` for its default
float32 computation, whereas this model requests `1e-10`. This is a convergence
criterion, not a bound of one micrometre on linkage error. Test closure, foot
contacts, rotor forces and stability under load rather than assuming identical
trajectories or silently weakening the CPU reference.
[Conversion code](https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/io.py)

The thermal time constant here is `20 J/K * 8 K/W = 160 s`. Its exact update
fraction at 0.5 ms is `-expm1(-0.0005/160) = 3.124995e-6`. A standard-library
float32 rounding check shows that an update from 40 °C toward 40.1 °C becomes
40.0000003125 in float64 but rounds back to 40.0 in float32. Computing
`1-exp(-dt/tau)` naively in float32 also changes the fraction by about -0.82%.
Preserve stable exponential evaluation and use float64 thermal accumulation,
or validate compensated accumulation, without requiring all physics to use
float64. Compare heating/cooling and energy over long warm trajectories; this
matters directly to the user's low-temperature objective.

Four GiB is a batch-size constraint, not proof of infeasibility. Contacts and
constraint workspaces, shared assets, CUDA graph allocations, PPO buffers and
the desktop/preview must all fit. Start with 16/32/64/128 worlds, trying 256
only after measuring memory headroom. These are proposed test sizes, not proven
capacities. Measure worst-case falling/self-contact states and detect buffer
overflow; do not truncate contacts to obtain higher speed. Keep visuals out of
the batched render path since this policy uses no image observations.

### Evidence required to claim a speedup on this laptop

The existing [CPU benchmark](../sim/benchmarks/cpu_training_20261026.json) measured
walking at 601 transitions/s with four workers and 766 with eight. At eight,
99.03% of measured time was rollout collection, including physics, resets,
Python/IPC and inference. It was not an isolated native-solver profile.
Eliminating PPO update time alone would improve that case by less than 1%.
GPU work should target the environment loop first.

1. In an isolated dependency environment, load the unchanged robot and compare
   single-world controlled short traces: free motion, stand, individual motor
   excitation, limits, contacts, warm motors and thermal termination. Then use
   longer statistical/evaluation comparisons; chaotic contact trajectories
   need not remain numerically identical.
2. Measure warmed batched physics plus the real plant at each proposed batch
   size, retaining timestep, rates and solver settings as far as supported.
   Report compilation/capture cost, VRAM and overflow separately. Warp recommends
   graph capture and explicit contact/constraint buffer sizing.
3. Measure complete policy transitions including resets, observations, transfer
   costs, PPO and the user's live preview. Compare equal simulated work with
   both CPU baselines; raw physics FPS is not training throughput. The eight-CPU
   result represents at least 30,654 world-substeps/s before counting settling
   and final forward passes.
4. As an engineering acceptance target, seek at least 2× the eight-worker CPU
   throughput (about 1,530 walking transitions/s) with adequate VRAM headroom
   and unchanged qualification results. This is a proposed threshold, not a
   prediction. Larger environment batches require revisiting PPO rollout length:
   keeping 512 total transitions with 128 worlds gives only four steps per world.
   Benchmark throughput separately from learning quality and time to qualification.

The recommended first candidate is direct **MuJoCo Warp + PyTorch**, preserving
the present model and avoiding an additional JAX layer. MJX-Warp/Playground is
an alternative if adopting its JAX training stack is intentional. Warp targets
batched throughput, whereas native MuJoCo remains appropriate for a single
interactive preview. [Backend tradeoffs and tuning](https://mujoco.readthedocs.io/en/stable/mjwarp/index.html)

The source audit alone did not establish a speedup. The following implementation
and benchmarks were performed subsequently with no user trainer running.

## GPU implementation and measurements

The optional `--physics-device=gpu` walking backend now uses direct MuJoCo Warp
3.17.0 / Warp 1.18.0 on the RTX 3050 Laptop GPU. Its CUDA graph advances 40
physics substeps per policy action, including the coupled rotor voltage/torque
plant and double-precision temperature/energy integration. Each world's
over-temperature state is frozen at the first violating substep. Overflows and
nonfinite batches abort training. Actions still pass through the same filter
and rotor-rate limit. Hardware calibration and model parameters are unchanged.

The first integration keeps the existing CPU Gym reward, observation, settling,
randomization and termination code. A CPU forward solve at each policy boundary
provides its contacts/IMU interface; small state batches cross the PCIe bus once
per policy step. PPO remains CPU-based. Reset worlds retain independent seeds,
friction, gains and temperatures; other worlds keep their GPU integration state.
The Qt preview and cold/warm qualification continue using native CPU MuJoCo.
This uses CPU and GPU for different stages; it does not add CPU rollout workers
to the GPU batch or overlap PPO updates with collection of another rollout.

The isolated motor/physics benchmark (three repeats, median throughput) measured:

| Backend | Parallel worlds/workers | Transitions/s |
| --- | ---: | ---: |
| Native CPU | 1 | 273 |
| Native CPU | 4 | 1,005 |
| Native CPU | 8 | 1,806 |
| GPU | 1 | 46 |
| GPU | 32 | 1,314 |
| GPU | 128 | 4,268 |
| GPU | 256 | 6,129 |

These figures exclude PPO, rewards, observations, resets and per-step worker IPC.
The nominal hold used about 302 MiB total device memory with 128 worlds and
430 MiB with 256. A separate 128-world trace used differing rotor targets,
20–40 °C starting temperatures and a 5 cm drop/landing. Against CPU, the first
world's maximum foot-position error over two simulated seconds was below
1 micrometre and temperature error below 0.000022 °C, with no buffer overflow.
This checks numerical agreement over short traces, not physical robot validity.

Complete PPO timing gave much smaller gains:

| Backend | Worlds/workers | Timed transitions | Transitions/s |
| --- | ---: | ---: | ---: |
| CPU, initial comparison | 8 | 32,768 | 962 |
| CPU, after shared contact optimization | 8 | 32,768 | 811 |
| GPU, initial integration | 128 | 32,768 | 929 |
| GPU, optimized, short run | 128 | 32,768 | 1,095 |
| GPU, optimized, short run | 256 | 65,536 | 1,198 |
| GPU, optimized, full episode resets | 128 | 131,072 | 971 |

The longer GPU run completed 128 episodes and includes their physical resets;
the short GPU runs do not span a full 20-second episode. CPU comparisons use
8,192-transition batches, so their much longer per-world trajectories include
resets. Desktop/editor load remained active, and the CPU measurements varied
enough that these are observed rates, not a precise speedup estimate. Cached
startup plus one warm-up rollout took about 30 seconds for 128 GPU worlds and
50 seconds for 256. Cold kernel compilation takes longer.

**Decision:** keep CPU as the default and expose GPU explicitly. GPU physics
works, but sustained complete training is currently around 970 transitions/s,
versus the observed 810–960 CPU range. CPU reward/observation work and physical
resets limit the gain. The 256-world short result is not evidence of a sustained
large speedup. No simultaneous CPU-physics/GPU-physics worker pool is enabled.
The GPU default of 128 worlds avoids doubling CPU mirror/state allocation for
the relatively small observed 256-world gain.

Raw measurements are retained in
[physics results](../sim/benchmarks/gpu_physics_20261026.json) and
[PPO results](../sim/benchmarks/gpu_training_20261026.json). The isolated
physics script remains in `sim/experiments/gpu_physics_benchmark.py`;
`sim/benchmark_training.py` measures the integrated trainer.

The full integration tests additionally compare normalized observations, rewards,
energy, individual resets, per-world randomization, thermal interruption and
terminal observations against CPU. Checkpoint resume and CPU playback are tested.
Run the explicit GPU suite on a CUDA machine with:

```sh
bazel test --config=pc //sim:gpu_walk_test --test_output=errors
```

Reproduce complete rollout/PPO timing (includes resets when episodes end, excludes
initialization/warm-up, GUI, qualification and checkpoint writes):

```sh
bazel run --config=pc //sim:benchmark_training -- --task=walk --physics-device=gpu --workers=128 --rollout-steps=8192 --updates=4
bazel run --config=pc //sim:benchmark_training -- --task=walk --workers=8 --rollout-steps=8192 --updates=4
```

The GPU default has a larger PPO batch than the original CPU default. Its faster
transition rate does not establish faster learning or shorter time to qualify:
per-world horizon, state diversity and update frequency differ. The current CPU
reward/reset interface limits GPU gains; single-world preview belongs on CPU.

### Batched GPU qualification follow-up

GPU training now uses 28 parallel qualification worlds: seven commands, two
starting temperatures, and baseline/policy pairs. Every case retains seed 12345,
the same 10-second duration and scoring. Completed cases freeze instead of
autoresetting; the evaluator and CUDA graph are reused between qualifications.
CPU training retains serial CPU qualification. `walk_evaluate --physics-device=cpu`
provides an explicit reference comparison regardless of the saved backend.

On the saved 16,384-step checkpoint from `walk-20261026-201827-457588`, full
qualification took **64.33 seconds on CPU and 28.02 seconds on GPU**, including
GPU evaluator construction with cached kernels (about 2.3× faster). All 28
pass/fail decisions and termination reasons agreed. Maximum differences were
0.0108 °C peak temperature, 0.1671 W mean copper power, 0.00000675 m/s tracking
error, and 0.00000517 rad/s yaw error. This is one checkpoint comparison;
small numerical differences can affect cases near a qualification threshold.
[Full comparison](../sim/benchmarks/gpu_qualification_20261026.json).

Progress now reports the GPU batch, completed cases, simulated time and elapsed
time. Cancellation discards incomplete qualification and preserves the training
checkpoint. Existing trainer processes must stop/save and resume once to load
the new evaluator.
