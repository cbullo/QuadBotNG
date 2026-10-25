# Candidate QuadBotNG physical model

The MuJoCo model reuses the expanded `sim/urdf/quadbot.urdf` and STL assets. It
preserves all five hinges per leg and closes each four-bar linkage with a point
constraint. Four independent planar closures remove eight of the twenty leg
coordinates, leaving twelve leg degrees of freedom. A floating chassis adds six.

The linkage uses the URDF's 120 mm main/link lengths and 35 mm crank/closure lever.
The closure points are `[0, 0, -0.12]` in each link and `[0.035, 0, 0]` in its
lower arm. `set_pose` selects the uncrossed parallelogram branch analytically.

## Differential

The production encoder equations, for reduced motor angles `ai`, `ao`, `az`, are:

```text
z     = az - offset_z
theta = -(ao + ai)/2 - offset_theta
gamma = ao - ai - s*(4/3)*z - offset_gamma
```

Here `s` is the YAML `z_gamma_dir`. For increments about a reference pose:

```text
dai = -dtheta - dgamma/2 - s*(2/3)*dz
dao = -dtheta + dgamma/2 + s*(2/3)*dz
daz = dz
```

`robot_model.py` combines this inverse with an explicit candidate CAD alignment,
then multiplies each motor coordinate by its 9:1 gear ratio to obtain rotor
radians. Each rotor is represented by a fixed tendon, with one torque actuator.
MuJoCo applies the transpose of that mapping to forces. The model therefore
preserves motor/joint mechanical power and coupled reflected rotor inertia.

The CAD alignment is configurable in `sim/robot_parameters.json`:

```text
theta = theta_sign * (main + crank)/2
gamma = gamma_sign * (crank - main)
z     = z_sign * shoulder
```

The controller gives the differential algebra and per-leg coupling signs, but
does not prove the CAD frame alignment or absolute encoder zeroes. These signs
and zero references remain a candidate interpretation. Gamma zero here describes
the CAD right-angle linkage reference, not a fully extended leg. The initial
demo stance rotates both main and crank by pi/4; it is not a homing calibration.
The controller's `stand` keyframe is generated from the extension-side gamma
bound in `src/odroid_controller/controller/config/robot_config.yaml`: +1.0472 rad
on FR/BR and -1.0472 rad on FL/BL. With the candidate alignment above, this gives
approximately main = 15 degrees, crank = 75 degrees, shoulder = 0, and both
passive angles = 60 degrees. The leg opening is about 150 degrees. Under controller
load it bends slightly and the chassis settles at approximately 0.237 m.

## Configured angle limits

The generator reads the controller YAML directly using locked PyYAML; there is
no separate standing-angle setting or copied table of limits in the simulator.
Changing the YAML requires regenerating both MJCF assets. A runtime controller
`--config` override does not regenerate an existing `--model`; generate a matching
model when changing the physical ranges.

| Legs | Gamma minimum (rad) | Gamma maximum (rad) | Z range (rad) |
| --- | --- | --- | --- |
| FR, BR | -1.4835 | 1.0472 | [-0.2, 1.4] |
| FL, BL | -1.0472 | 1.4835 | [-0.2, 1.4] |

Gamma is limited through a fixed tendon `gamma_sign * (crank - main)`, preserving
the three-motor differential. Z bounds are transformed by `z_sign` into shoulder
hinge bounds. These are compliant simulated stops (2 ms solver time constant),
not independent main/crank clamps or new hardware control limits. Tests drive
past both ends of both coordinates and check settling within 0.002 rad of each
bound. The existing controller uses these values as homing references, without
general motion clamping. Simulation homing remains disabled because the absolute
encoder/CAD alignment and physical stops are still not validated.

The modeled gamma interval corresponds to about 5–150 degrees of leg opening.
Exact 180-degree extension remains outside it and would place the four-bar at
a singularity. These individual coordinate intervals are not a collision-free
Cartesian workspace: combining the folded gamma endpoint with maximum Z at the
default swing angle can hit the chassis. Chassis collisions remain enabled.

## Physical assumptions

The source masses total 5.049489 kg. Other than the crank correction below, the
URDF inertial tensors and centers of mass are retained. STL units are scaled
from millimeters to meters. Absolute source mesh paths are replaced by paths
relative to the generated MJCF.

| Parameter | Current interpretation |
| --- | --- |
| Crank Ixx | Source export is zero; replaced with estimated 4e-7 kg·m². |
| Rotor inertia | Estimated 1e-6 kg·m² per rotor, applied through the coupled tendon. |
| Rotor damping | Estimated 2e-5 N·m·s/rad. |
| Joint damping | Estimated 0.002 N·m·s/rad. |
| Joint dry friction | Main 0.071, crank 0.029, shoulder 0.073 N·m at the joint outputs; calibrated to unloaded raised-leg self-weight. |
| Motor torque ceiling | User-selected 3,000 gf·cm = 0.2941995 N·m at each shaft. |
| Contacts | Outer CAD envelopes plus smaller internal linkage proxies; intended bearing interfaces excluded. |
| Limits | YAML gamma/Z bounds mapped into compliant stops; physical alignment and homing remain unvalidated. |
| Solver | 0.5 ms implicitfast integration, explicit compliant four-bar closures. |

The model's inertias, damping, friction and torque ceiling are not measured robot
identification. No battery/sensor masses were added from unrelated older model
variants. Collision envelopes and contact material parameters remain approximations.

## Joint friction

The real robot can hold a raised, unloaded leg against gravity with little spare
holding capacity. The model represents this using MuJoCo joint `frictionloss`,
which resists motion at rest up to a finite torque and continues opposing sliding.
Viscous damping alone cannot hold a stationary leg. See
[MuJoCo friction loss](https://mujoco.readthedocs.io/en/stable/computation/index.html#friction-loss).

`joint_frictionloss_nm` in [robot_parameters.json](../sim/robot_parameters.json)
contains estimated output-side torques, not motor-shaft torques. Because the
observation does not identify losses in individual bearings and gears, the losses
are lumped at the three independent hinges. Passive lower/link hinge dry friction
is zero to avoid counting the same resistance again around the closed linkage.
The differential transmission and its 9:1 gearing still apply to motor torque.

The calibration reference is an upside-down, fixed chassis with a raised bent
leg: main = crank = pi/4, shoulder = 0. This is an explicit assumption about the
observed pose, not a measured breakaway test. The leg has no supporting contacts
and the motors apply zero torque. The CAD gravitational loads projected onto the
main, crank and shoulder coordinates have magnitudes 0.06398, 0.02596 and
0.06622 N·m. Since both passive angles equal `crank - main`, their gravity loads
subtract from the main coordinate and add to the crank coordinate. The selected
friction torques are about 10% above these loads, rounded upwards.

Regression tests check all four legs at 500 us and 250 us: less than 0.001 rad
drift over five seconds without contacts or actuation, and more than 0.1 rad of
motion within one second when gravity is increased by 25% or dry friction is
removed. Motor pulses also verify that the actuators overcome friction through
the differential. Friction uses a 2 ms solver time constant and constant 0.9999
impedance to limit numerical creep; it does not lock the joints or the chassis.
The same generated models are used by the demo and the production-controller
simulation, including when boards are stopped or reset.

These values need refinement with measured release angles or applied torques.
They do not assert that every pose holds: gravitational leverage changes with
pose. Sliding friction currently uses the same threshold as breakaway, without
a separate Stribeck curve. Friction losses are dissipated mechanically; their heat
is not added to the motor copper-temperature approximation.

## Collision representation

Detailed STL meshes are used for rendering only. Collision geometry is defined
in [collision_geometry.json](../sim/collision_geometry.json), in each URDF body's
local frame, in metres. Right-leg templates are mirrored in Y for LF/LB.
These shapes do not change the CAD mass, center of mass, inertia, differential,
or 3,000 gf·cm actuator limits (`inertiafromgeom="false"`).

Outer envelopes used for ground, chassis and other-leg contacts:

| Part | Collision shape | Basis |
| --- | --- | --- |
| Chassis | Three boxes for the lower frame, middle housing and front raised section | Separate CAD envelopes preserve space beside the chassis. |
| Twelve fixed drive motors | Cylinders, radius 27.5 mm, axial length 48.1 mm | Measured directly from the drive STL's axial faces. |
| Shoulder | Cylinder plus bracket box | Housing and bracket bounds from CAD. |
| Main arm | Capsule plus bearing cylinder | 120 mm linkage and its bearing envelope. |
| Crank / parallel link | Capsules | 35 mm / 120 mm linkage; CAD cross-section bounds. |
| Lower arm / foot | Shaft capsule plus 19 mm-radius sphere | Narrow shaft and rounded toe represented separately. |

This is preferable here to using each complete visual mesh as a collider:
MuJoCo collides mesh convex hulls, which fill cavities and gaps. In the neutral
stance, using every visual hull produced about 20 mm of artificial penetration
at the main/crank and lower/link bearings. Primitive compounds make these
envelopes explicit and easy to adjust. Convex mesh decomposition remains an
option for a part whose shape cannot be represented adequately by primitives;
it is unnecessary for the current rods, cylindrical motors and frame sections.
See [MuJoCo collision detection](https://mujoco.readthedocs.io/en/latest/computation/#collision-detection).

Within a leg, the solid outer envelopes fill the linkage gaps and produce false
contact well before the configured endpoints. `within_leg` in the same JSON
defines smaller proxies for those interactions: an offset 4 mm main-arm spine,
two separated bearing cheeks, a 3 mm offset link spine and crank, a 4 mm lower-arm
shaft, and a 6 mm internal toe sphere. The shoulder retains its cylinder/box.
These proxies are an explicit clearance approximation, not a new CAD fit: the
exported solids themselves overlap in the deeply folded endpoint under the
current coordinate alignment. The outer 19 mm foot sphere and limb envelopes
remain unchanged for floor/chassis/other-leg contacts. Internal proxies must be
revisited when the linkage clearances and coordinate references are measured.

Collision masks select the appropriate representation. Floor uses bit 1,
chassis bit 2, each leg's outer envelope one of 4/8/16/32, and each leg's internal
proxies one of 64/128/256/512. Outer shapes collide with floor, chassis and other
legs; internal shapes collide only within their own leg. Tests also deliberately
overlap the internal main/link proxies beyond the configured range to verify
their contact remains active. Visual geoms have both masks zero.
MuJoCo's usual adjacent-body filtering remains
active. Explicit body-pair exclusions cover only base/shoulder, drives/shoulder,
coaxial main/crank, and lower/link closure bearings, independently for each leg.
The base exclusions also keep the suspended variant consistent with the floating
one. Other leg/chassis and leg/leg pairs remain eligible to collide.

Self-contact uses a 2 ms time constant with critical damping. The higher-priority
floor keeps its 6 ms contact time constant; six-dimensional floor contacts enable
the existing torsional/rolling friction values for rounded feet and capsules.
Sliding/torsional/rolling friction remain candidate values `0.8, 0.005, 0.0001`.
Priority selects the floor contact parameters as described in
[MuJoCo contact parameters](https://mujoco.readthedocs.io/en/latest/modeling.html#contact-parameters).

Tests sweep 101 gamma positions, including both configured endpoints, for every
leg. The full sweep is contact-free at Z=0; repeating at both Z endpoints produces
no internal leg contact, while preserving possible chassis interference. Each
Z endpoint is also contact-free at gamma=0. Tests check no false contacts in both
standing keyframes, coverage
of every physical body, opposite-leg contact, and force-limited inward folding
of all four legs. Chassis contact blocks folding at about 0.82–0.89 rad rather
than the commanded 1.5 rad; disabling contact permits the same motion. Peak
transient contact penetration stays below 3 mm in this aggressive test.
This is a compliant contact approximation, not an exact CAD interference check.
Forty-second thermal-collapse tests use a 5 J/K heat capacity to trigger the fault
early enough to observe settling, then check zero motor torque and passive
settling at 500 us and 250 us physics steps. Default motor heat capacity stays
20 J/K; the straighter stance draws less current and heats more slowly.

To inspect only the collision shapes in the controller viewer:

```sh
bazel run --config=pc //sim:controller_viewer -- --scenario=stand --show-collisions --duration=8
```

Blue shapes show outer envelopes; orange shapes show within-leg proxies. Blue
envelopes can overlap within the same leg because orange proxies determine those
contacts. Both are visible to make the approximation inspectable.

## Generation and inspection

```sh
bazel run --config=pc //sim:generate_model
bazel run --config=pc //sim:generate_model -- \
  --fixed-base --output sim/mujoco/quadbot_suspended.xml
bazel test --config=pc //sim:robot_model_test
bazel run --config=pc //sim:robot_demo -- --duration=30
```

`quadbot.xml` has a free chassis, a bent `stance` keyframe, and an extended `stand`
keyframe. The suspended variant
fixes the chassis at 0.4 m. Regenerate both after changing geometry or parameters;
tests compare checked-in assets to generator output.

The independent demo installs finite-gain, torque-limited rotor position servos
for inspection. The generated XML contains torque actuators. Tests check
structure/mass/inertias, closure across poses, linkage mobility, encoder equations,
virtual work, reflected inertia, each motor, timestep sensitivity, passive free
fall and loaded contact. These numerical checks do not validate hardware signs
or homing. For the actual C++ controller integration, use
[simulated-boards.md](simulated-boards.md).
