# Repository reconnaissance

**Historical baseline:** the subsequent Bzlmod migration and current build
status are documented in [build.md](build.md). The old firmware programs are
likely deprecated per the user, and the web server is now disabled pending
rework. The build failures and unverified status below describe the initial
survey, before that migration.

Inspected on 2026-10-25 at `eadb124` (2024-07-19, `Conf`). The user describes
an Odroid host, six two-motor 8-bit power controllers, and a 12-DOF quadruped,
and explicitly confirms a three-motor differential for each leg.

This is an initial source survey. No robot connection, deployment, firmware
flashing, dependency upgrade, or simulator implementation was performed.

## Source map

| Location | Purpose |
| --- | --- |
| `src/odroid_controller/controller/main.cc` | Loads YAML, constructs boards/motors/legs, connects hardware, configures joystick/behaviors/web server, runs the update loop. |
| `src/odroid_controller/controller.{h,cc}` | Event distribution, stop/e-stop behaviors, calibration/leg testing, initialization and stand-up sequence under `WalkBehavior`. |
| `src/odroid_controller/leg.h`, `leg_controls.*`, `pid_utils.h` | Motor-to-leg coordinate conversion, PID control, calibration and homing. |
| `src/odroid_controller/motor.*` | Encoder wrap accumulation, gear ratios, calibration configuration, telemetry. |
| `src/odroid_controller/bldc_driver_board.*` | Linux serial transport, communication threads, synchronization, command queues and feedback. |
| `src/libs/communication/` | Shared binary command and response definitions. |
| `src/mcu_firmware/controller/main_loop.cc` | Main two-motor firmware, state machine and FOC updates. |
| `src/mcu_firmware/base/` | Common AVR main loop, binary commander, analog temperature readings and watchdog. |
| `src/mcu_firmware/base/simple_foc/` | Custom in-tree FOC, PWM and magnetic sensor implementation. |
| `src/mcu_firmware/{calibration,encoder,init_foc,open_loop}/` | Alternative firmware programs for bring-up/calibration. |
| `src/odroid_controller/web_server/` and `controller/public/` | libwebsockets HTTP/SSE telemetry server and browser assets. Server uses port 8080. |
| `src/utils/encoder_linearization/` | Python calibration processing and recorded motor measurements. Imports NumPy, SciPy, Matplotlib and `pwlf`; no dependency manifest found. |
| `src/utils/sin_approx/`, `misc/conversion_test/` | Numerical experiments, including some checked-in generated/binary artifacts. |
| `sim/` | Meshes, Xacro/URDF, a Webots PROTO export and an example SDF world. |
| `toolchain/`, root Bazel files | AVR and AArch64 build definitions and dependencies. |

## Hardware and control path

The configured host is Odroid-C4/AArch64 Linux. `RCTimer` targets ATmega328P at
16 MHz. Firmware configures two AS5600 12-bit magnetic sensors and two BLDC
motors with seven pole pairs per board. Host/firmware serial speed is 115200.

The main YAML contains six boards, twelve motors and four legs:

| Board | Device | Channel 0 | Channel 1 |
| --- | --- | --- | --- |
| `controller_FR` | `/dev/ardu2` | FRI | FRO |
| `controller_FL` | `/dev/ardu3` | FLI | FLO |
| `controller_FZ` | `/dev/ardu1` | FRZ | FLZ |
| `controller_BZ` | `/dev/ardu4` | BLZ | BRZ |
| `controller_BR` | `/dev/ardu5` | BRO | BRI |
| `controller_BL` | `/dev/ardu6` | BLI | BLO |

Motor names in the file have the `motor_` prefix. Each leg uses I/O/Z motors;
Z motors share a board across a pair of legs. All configured `gear_ratio_inv`
values are 9. Calibration includes electrical zero, encoder direction,
linearization offset and 16 coefficients, and wire swapping.

Control flow is joystick/events → behavior → leg control → voltage commands →
board firmware FOC. Feedback travels through serial telemetry → `Motor` → `Leg`.
`WalkBehavior` currently sequences initialization and stand-up; its name should
not be taken as evidence of a complete walking gait implementation.

The host update loop uses elapsed wall time without an explicit sleep. Leg
controllers use a nominal 0.005 s update interval. `Controller::Update` gates
leg updates and board ticks on serial command backpressure and accumulates dt.
Initialization also contains blocking one-second sleeps. These details matter
for future deterministic simulation and timing tests.

## Differential coordinates and units

The **three-motor differential per leg is a user-confirmed physical property**.
Do not model the twelve motors as twelve independently driven model joints.
The exact physical transmission still needs reconciliation with the model.

`Motor::GetAccumulatedAngle()` converts encoder counts with pi/2048 radians per
count (4096 counts/revolution). `Leg` applies the configured gear ratio:

```text
a_i = accumulated_motor_i_angle / gear_ratio_inv_i
a_o = accumulated_motor_o_angle / gear_ratio_inv_o
z = accumulated_motor_z_angle / gear_ratio_inv_z - zero_offset_z
theta = -0.5 * (a_o + a_i) - zero_offset_theta
gamma = (a_o - a_i) - z_gamma_dir * (4/3) * z - zero_offset_gamma
```

These are the current source equations, not independently verified mechanics.
`z` here is an angular coordinate, not Cartesian height. Per-leg direction and
reference values live in YAML. Host leg-index order in `Legs::GetLeg` is
FR, FL, BL, BR; startup separately relies on sorted leg names BL, BR, FL, FR.

`DriveTo` maps control outputs with `o = 0.5*tau_theta - tau_gamma`,
`i = 0.5*tau_theta + tau_gamma`, and
`z = tau_z + z_gamma_dir*(4/3)*tau_gamma`. The separate
`ThetaGammaZLegControl` uses different mixing (half gamma and no Z compensation).
Reconcile these paths before treating either as the definitive actuator model.

Despite `tau` variable names, transmitted commands are signed 16-bit voltage
values scaled by 512, with configured direction applied. They are not calibrated
joint torques. A physical simulator needs an explicit voltage-to-motor-force
model, or a deliberately defined approximation. Motor constants, friction,
back-EMF and drivetrain losses have not been established by this survey.

## Binary interface

`binary_commands.h` defines the command ID in the low four bits, motor channel
in bit `0x40`, and GET in `0x80`. Commands include voltage, phase, board state,
sensor configuration, linearization and synchronization. `binary_stream.h`
defines board-to-host message headers.

Board states in `base/binary_commander.h` are PRE_INIT=0, INIT=1, STOPPED=2,
RUNNING=3 and ERROR=4. The host sends calibration and availability after sync,
then INIT. Telemetry alternates angle and temperature frames, each with a header
and two uint16 values. Native-memory serialization relies on the deployed
machines' byte order and should be specified explicitly for another backend.

Firmware has a watchdog and temperature-triggered reset. Host communication
timeout error handling is commented out in `BLDCDriverBoard::CheckTimeout`.
Simulation of stop/fault behavior should follow a reviewed contract rather
than assuming that all existing paths already enforce it.

## Existing simulation material

- `gazebo_driver.{h,cc}` is an unfinished sketch: placeholder expressions,
  incomplete function bodies and missing class qualifications. It includes
  `gazebo-11/gazebo/transport/Node.hh`.
- `sim/world.sdf` instead references `gz-sim-*` system plugins. It contains a
  ground plane and placeholder vehicle, not the quadruped. It specifies 1 ms
  physics steps. There is no established end-to-end simulator here.
- `quadbot.xacro` defines a repeated leg macro; `quadbot.urdf` is its apparent
  expanded counterpart. The latter has 22 links, 20 continuous joints and one
  fixed joint, plus four Gazebo ball-joint extensions closing the linkages.
  These extra passive/linkage joints must not become extra motor actuators.
- `sim/meshes/` includes visual and simplified collision STL files. The URDF
  scales meshes by 0.001. All active URDF mesh paths resolved in this checkout,
  but they are absolute `/home/tomasz/dev/QuadBotNG/...` paths.
- Inertial data exists; URDF link masses sum to about 5.04949 kg. This is a
  file-derived total, not a confirmed robot mass. Some inertias contain zero
  diagonal values and require physical validation before simulation.
- Transmission sections repeat `crank_trans_*` names and lack joint bindings.
  Continuous joints do not provide physical joint stops. Homing behavior seeks
  limits, so physical constraints and software reference offsets need attention.
- `URDF.xacro` is a separate older-looking model; `URDF.trans` and `URDF.gazebo`
  contain additional fragments. Do not silently mix model variants.
- `URDF.proto` is a Webots R2022b-format export with absolute mesh paths and a
  default `void` controller. This is not evidence that Webots was selected.
- Model leg names RF/LF/RB/LB correspond to controller names FR/FL/BR/BL.

Potential integration points are the existing `ControllerType` board adapter
selection, or a future explicit motor/board interface. A serial-protocol emulator
is another option if retaining host serial behavior is a requirement. No engine,
interface, fidelity level, or timing model has been chosen; await the user's
simulator requirements and verify differential mechanics first.

## Build and maintenance findings

- C++17/Bazel project with WORKSPACE dependencies. Recorded versions include
  Arduino AVR core 1.8.5, fmt 8.1.1, libwebsockets 4.3.2,
  rules_foreign_cc 0.9.0 and EmbeddedSystemsBuildScripts 1.0.2. yaml-cpp and
  external SimpleFOC use moving `master` archives. This survey did not compare
  them with current upstream releases.
- No `.bazelversion` was found. `MODULE.bazel` contains only migration comments;
  it and `MODULE.bazel.lock` were already untracked at inspection start and
  were left untouched. They do not constitute a completed dependency migration.
- AArch64 tools/sysroot are hardcoded under `/home/tomasz/x-tools/`; the specified
  compiler does not exist here. `.bazelrc` also hardcodes a ccache writable path.
- README host build instructions point at nonexistent
  `//src/odroid_controller:controller`; the binary is in the nested
  `//src/odroid_controller/controller` package.
- Both host packages define independent simulator flags. Simulator `defines`
  branches are dictionaries rather than string lists, and the executable's
  simulator branch references a local `:gazebo_sim_lib` that is actually defined
  in the parent package. The Gazebo dependency itself is not wired up.
- Configuration is loaded relative to the process working directory as
  `config/robot_config.yaml`; web assets are served from `../public`. Launch and
  runfiles handling need review for local execution.
- Deployment scripts use a hardcoded SSH robot address and stop the remote
  service. Some calibration/deployment paths and the short calibration document
  refer to older layouts (`calibration.ino`, an `encoders` YAML section).
- Only one test target was found: `//src/odroid_controller/tests:angle_tests`,
  with nine cases. It tests a copied `TestMotor`, not the production `Motor`.
  Nonzero initial encoder handling deserves review: the production first-update
  branch mixes radians with count storage. No fix was attempted.
- Serial lifecycle/queue handling also deserves focused tests: `Disconnect`
  does not invalidate the descriptor and shadows the sync state; `SendCommand`
  does not advance its byte pointer after partial writes; reply processing
  appears not to consume/clear queues. These are source-review findings, not
  reproduced hardware failures.

## Verification performed

Read source/build/configuration files; parsed the principal XML models and YAML;
checked model mesh references, joint counts, duplicate transmission names and
the configured board/motor mapping. Locally available tools reported Bazel
9.3.1, host GCC 13.3.0 and AVR GCC 7.3.0. `gz`, `gazebo`, `xacro` and
`aarch64-linux-gnu-g++` were not found on PATH.

An offline target query was attempted with:

```sh
bazel --batch --output_user_root=/tmp/quadbotng-recon-bazel query //src/odroid_controller/tests:angle_tests --nofetch --lockfile_mode=off
```

It exited 37 with an internal Java `NullPointerException` in
`NetworkMetricsCollector.collectSystemNetworkUsages` (`loopbackInterfaceNames`
was null). This establishes a local Bazel execution failure, not a compiler or
test result. Compilation and tests remain unverified. The temporary output root
keeps this probe separate from the usual Bazel cache.
