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
