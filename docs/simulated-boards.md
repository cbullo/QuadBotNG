# Running the controller with simulated boards

Implemented on 2026-10-25. This runs the production C++ `Controller`, `Leg`,
`Motor`, and leg-control code through the same board client used for serial
hardware. Six two-channel board emulators exchange the existing binary protocol
with that client. Native MuJoCo advances one shared robot model.

This is controller integration and a bring-up motor approximation. The `stand`
scenario demonstrates a known-pose ground stance through host voltage control;
it does not validate hardware equivalence, homing, walking or standing from rest.
The earlier `//sim:robot_demo` uses its own position servos and remains a separate
geometry/load demonstration.

## Launch

For controllers, simulation, plots and sensed 3D inspection in one command:

```sh
bazel run --config=pc //sim:inspector
```

The free native inspector opens a Qt window and supports acknowledged commands over TCP.
See [the inspector guide](inspector.md).

```sh
# Ground stance matching the direct demo's bent-leg pose (desktop viewer):
bazel run --config=pc //sim:controller_viewer -- --scenario=stand --duration=8

# Inspect the actual collision shapes, including self-collision:
bazel run --config=pc //sim:controller_viewer -- --scenario=stand --show-collisions

# Headless full-robot startup and passive dynamics:
bazel run --config=pc //src/odroid_controller/controller:controller -- \
  --backend=sim --scenario=idle --duration=5

# Existing leg PID/mixing holds the sensed pose after board initialization:
bazel run --config=pc //src/odroid_controller/controller:controller -- \
  --backend=sim --scenario=hold --suspended --duration=10

# Render snapshots from that native controller process:
bazel run --config=pc //sim:controller_viewer -- \
  --scenario=hold --suspended --duration=30

# Deterministic trace for inspection or comparison:
bazel run --config=pc //src/odroid_controller/controller:controller -- \
  --backend=sim --scenario=stop --suspended --duration=5 --trace=/tmp/quadbot.csv
```

The native runner is headless. The viewer starts it as a child with real-time
pacing and renders its snapshots; it does not run a second physics simulation.
Closing the viewer, terminal Ctrl+C, or SIGTERM stops and reaps the simulated
child. The viewer waits for MuJoCo's background renderer to finish before Python
and GLFW shut down; merely requesting window closure is asynchronous and could
previously cause an exit-time segmentation fault. Repeated interrupts do not
interrupt cleanup. Viewer pause controls do not pause the native simulation clock.
Simulation never opens real serial devices
or a joystick, and failures do not fall back to hardware.

Assets resolve through Bazel runfiles, including from another working directory.
Override them with `--config=PATH`, `--model=PATH`, or `--motor-config=PATH`.
The suspended asset fixes the chassis at 0.4 m, clear of the ground. Legs still
respond to gravity during the approximately three-second board boot/sync/setup.
Pose hold targets the delivered encoder pose after setup, not an externally
imposed standing pose. Expect to see the difference from the independent demo.

`stand` is the viewer default. It uses the extended `stand` keyframe, with a
roughly 150-degree leg opening rather than the demo's 90-degree bend. The generator
reads `min_gamma`, `max_gamma`, `min_z` and `max_z` directly from the controller's
`robot_config.yaml`; regenerate both MJCF assets after changing those values.
The extension target is +1.0472 rad gamma for FR/BR and -1.0472 rad for FL/BL,
with Z=0. Gamma and Z also have compliant stops in the physical model. Dedicated
internal collision proxies allow both configured gamma endpoints while keeping
outer foot, chassis and other-leg contact envelopes. See the coordinate and
clearance assumptions in [physical-model.md](physical-model.md).
The `bob` scenario starts from the same extended keyframe. After startup and a
one-second settling interval, it cycles through a gentle crouch and rise every
four seconds. A cosine trajectory moves the CAD main/crank targets by up to
+0.09/-0.09 rad respectively, keeping shoulder and mean leg angle constant.
The existing model differential converts these target offsets into rotor angles;
the controller still uses only decoded encoder feedback. No physical pose is
reset during the movement. Inspector commands override the script, and pose and
thermal faults retain their normal stop behavior. Run it with
`bazel run --config=pc //sim:inspector -- --scenario=bob --duration=20`.
The demo and other scenarios retain the bent `stance` keyframe.

The runner places the extended pose just above the ground and holds all
coordinates during board boot/synchronization. Once
the boards are ready, `RobotSystem::HoldRotorPose` captures the delivered encoder
targets and attaches a new `RotorPoseLegControl` to each leg. Support is released
when all six boards reach RUNNING; after that there is no support, pose reset or
MuJoCo position servo. The controller sends voltage commands through the same
client and board emulators, with the differential and 3,000 gf·cm shaft ceiling
unchanged. This is a known-pose initialization fixture, not a homing sequence.

The rotor PD uses 76.2 V/rad and 0.428 V/(rad/s), configured under `stance` in
`sim/motor_model.yaml`, with a 1 ms host control period. This faster cadence is
used by `stand`, `bob`, `init` and `standup`; the other scenarios and hardware entry point retain
5 ms updates. The modeled 115200-baud transport and quantized feedback remain
active. Velocity comes from successive decoded rotor angles, not MuJoCo state.
The old `ThetaGammaZLegControl` gains and measured hardware calibration are
unchanged. This new policy has not been validated on hardware.

An eight-second default preview includes about three seconds of setup and five
seconds of unsupported ground stance. With the current unmeasured thermal
constants, the loaded rear motors eventually exceed 55 C and reset their boards,
ending the stance. Timing depends on mechanical load, including joint friction.
Longer runs retain this thermal fault; no motor is artificially cooled and startup
support is never restored. Use `--motor-config=PATH` for measured parameters.
The regression tests shorten heat capacity in their own motor configuration to
exercise thermal shutdown without a long run.

Both pose controllers now latch a host fault if any board reports a temperature
above 55 C, loses fresh encoder feedback for 100 ms, or reports a transport error.
The shared controller detaches all leg controls and sends STOP to every board;
healthy motors no longer fight the fallen pose after a rear-board reset. Pings
and temperature packets do not count as fresh encoder feedback. A stalled board
can only consume STOP after it resumes; its independent AVR watchdog remains
responsible for a stalled MCU reset. This adds host behavior, without changing
the firmware protocol or watchdog.

After a latched pose fault, the runner keeps stepping passive physics and pumping
healthy boards even if the failed board later reaches its communication timeout.
Pose control cannot restart within that session. The reason is printed, and the
CSV/summary `pose_fault` field becomes 1. Longer runs let you watch the subsequent
fall and settling with the new self-collision shapes. Primitive contact geometry
and exclusions are described in [physical-model.md](physical-model.md).

| Scenario | Behavior |
| --- | --- |
| `idle` | Complete setup, stay STOPPED, zero motor torque. |
| `pulse` | RUNNING, apply a 0.1 V pulse to each mapped channel for 200 ms, then command zero voltage. This tests wiring/protocol; it is not a gait. |
| `hold` | RUNNING, attach existing `ThetaGammaZLegControl` to each leg using its current sensed pose as target. |
| `stand` | Initialize at the YAML extension-side gamma bounds (about 150-degree opening), complete board startup under support, then release into host rotor-PD voltage control. |
| `bob` | Start as `stand`, settle for one second, then gently crouch and rise on a four-second cycle through the same rotor controllers. |
| `stop` | Start pose hold, send a production stop event after one second of control, disable leg control and board outputs. |
| `standup` | Run init, then the staged rise toward stand; abort on homing failure or an unreached waypoint. |
| `init` | Start belly-down with legs raised, let physics run during startup, then run production Start/Confirm and repeated diagonal-pair reference probing. Experimental, not validated homing. |
| `homing` | Rejected before execution: mechanical stops and absolute references remain unvalidated. |

The PC binary selects `--backend=serial` or `--backend=sim` at runtime. Serial is
still the default for existing deployment scripts, runs indefinitely by default,
and uses the joystick unless `--no-joystick` is supplied. An explicit
`--duration` bounds a serial run and sends STOP before normal exit. Simulation
runs for ten seconds by default. Do not run the serial backend against real
robot device paths as ordinary simulator verification.

## Protocol and transport

`BLDCDriverBoard` is a shared, single-thread-owned protocol client. `Tick` pumps
nonblocking communication and updates motor feedback; `Controller::Update`
pumps it before checking transmission backpressure. The runner also pumps it
at physics substeps. No communication worker or raw-memory callback storage is
needed. Hardware uses `SerialTransport`; deterministic simulation uses bounded
`MemoryTransport` byte queues with the same codec and a supplied monotonic clock.

Byte values use explicit little-endian serialization. Signed voltages use
1/512 V; telemetry contains two unsigned 16-bit angle or temperature values.
Encoder values are rotor counts modulo 4096. GET replies carry the command nibble,
its payload, and the active firmware's trailing zero sequence byte. Replies are
matched and removed exactly once; unsupported GET operations are rejected.

Client readiness requires INIT acknowledgement and the first encoder packet for
each configured motor. The supported setup path sends channel availability, linearization settings,
electrical offsets and INIT. The emulator stores those settings and follows
PRE_INIT → INIT → STOPPED, then RUNNING ↔ STOPPED. It preserves last commanded
voltage across stop/run, as the firmware does; disabled outputs produce zero
applied shaft torque. Motion type is represented; direct-phase motor excitation
is explicitly unsupported by the physical approximation.

Important deliberate boundaries:

- `CMD_LIMITS / SCMD_LIM_VOLT` now supports SET and GET using signed 16-bit
  little-endian values in 1/512 V. SET is four bytes (header, subcommand, value);
  GET is two bytes; its reply is four bytes (header, value, zero sequence).
  The shared header's former one-byte payload declaration is corrected for both
  the host and active firmware. Old firmware must be rebuilt/reflashed before
  using this command; its old framing is incompatible with the corrected SET.
  Current/velocity limit subcommands remain unsupported by the active firmware.
  The emulator mirrors the voltage-limit register, including its INIT default
  of `12*512-1`. The active `setPhaseVoltageSin2` FOC path still uses a fixed
  28 V cap rather than this register; fixing framing does not change that motor
  behavior. Unsupported unsafe calibration GET paths are not invented.
- Ideal telemetry represents corrected encoders. Measured linearization and wire
  configuration are stored for protocol fidelity, not reapplied to an ideal
  sensor or used as mechanical homing offsets. Calibration data is unchanged.
- The emulator's INIT delay is an explicit 1 ms approximation. Telemetry is
  offered at 500 us intervals when output capacity permits; this is a modeled
  scheduling rate, not a measured firmware loop rate.
- The execution watchdog follows active `base/main.cc`: `WDTO_4S`, fed only
  after a completed critical-loop iteration. Expiry is checked before execution
  resumes, including skipped updates or an expired `StallUntil` fault. The 2 s
  sync delay blocks feeding. Ordinary command inactivity retains the last
  voltage while the critical loop continues to run.
- A sensor reading above 55 C selects `WDTO_15MS` and blocks command handling
  and watchdog feeding until reset. Reset clears both channels' configuration
  and outputs, but preserves physical encoder/temperature samples. A hot motor
  can trigger another reset until it cools. The emulator still approximates
  temperature acquisition: the firmware's alternating 100 ms ADC checks,
  thermistor lookup table and median filter are not reproduced.
- The shared client now treats ten seconds without any valid incoming frame as
  a communication failure (the old timeout handler was disabled), and GETs have
  a one-second reply deadline. Failure disconnects and reports an error, with
  best-effort reset; this is not proof of a physical robot stop guarantee.
- Hardware RTS reset is nonblocking. PTYs cannot implement physical RTS reset;
  an unsupported reset is reported explicitly. Reconnecting a PTY client or
  restarting its server provides a fresh emulator state.

Production encoder accumulation now initializes in counts, including nonzero
initial readings, and clears feedback on reset/reconnect. Serial writes advance
correctly after short writes; descriptor lifecycle and fragmented reads are
covered by tests. The two blocking homing startup sleeps were replaced with
elapsed-time states. Stop/e-stop detach active leg controls; ordinary stop sends
STOPPED to the boards. These are changes to shared host code, not simulator-only
patches. No firmware was flashed or run.

## Motor approximation and timing

`sim/motor_model.yaml` is separate from hardware calibration. The motor model
uses the user's measured **10 ohm line-to-line resistance**. Its balanced
star-equivalent phase resistance is 5 ohm. The active AVR `setPhaseVoltageSin2`
scales sinusoidal duty from zero to the command magnitude, so the phase-neutral
voltage amplitude is approximately **half the wire command**. The old independent
0.03 Nm/V gain ignored this scaling and did not use resistance to calculate torque.

The `sinusoidal_voltage` model uses peak-phase current and voltage conventions:

```text
Rphase = Rline / 2
Uq = sign * clamp(command, +/-28 V) / 2
Iq = (Uq - Ke * rotor_speed) / Rphase
shaft torque = clamp(1.5 * Ke * Iq, +/-shaft_torque_limit)
```

The phase back-EMF constant `Ke = 0.0700475 V/(rad/s)` is inferred from the
user-selected **3,000 gf·cm shaft stall torque at a 28 V command** and the measured
10 ohm line-to-line resistance. It is not a measured Kv. With the AVR half-command
scaling, `Ke = 0.2941995 * 5 / (1.5 * 14)`. Rotor speed is mechanical rad/s.
The 1.5 factor follows the balanced sinusoidal peak-phase convention, as described
in the [PMSM equations](https://www.mathworks.com/help/simscape-electrical/ref/pmsm.html).
At rest, a 4 V command produces 0.0420285 Nm, 14 V produces 0.14709975 Nm,
and 28 V produces 0.2941995 Nm. At speed, back EMF reduces current and torque.
The 9:1 differential transmission is applied exactly once by MuJoCo.
Hardware calibration is unchanged. Homing voltage gains are reduced for this
stronger motor approximation; this tuning remains unvalidated on hardware.

A zero command on an enabled bridge includes back-EMF braking; a disabled bridge
coasts with the mechanical plant's friction. The model omits winding inductance,
current-loop dynamics, PWM ripple, and identified drivetrain losses. The shaft
ceiling is an ideal torque/current limit, not a measured saturation curve.
Output-side dry friction is documented in [physical-model.md](physical-model.md#joint-friction).

The lumped thermal model uses the same electrical model and the saturated torque:

```text
Iq = applied_shaft_torque / (1.5 * Ke)
loss = 1.5 * Rphase * Iq^2
C * dT/dt = loss - (T - ambient) / Rthermal
```

It integrates constant loss exactly over a physics step. Disabled bridges have
zero modeled copper loss. Ambient/initial temperature (20 C), heat capacity
(20 J/K), thermal resistance (8 K/W), and sensor lag remain unmeasured assumptions.
The measured winding resistance affects both torque and heat; there is no
independent `thermal.effective_resistance_ohm` or `shaft_torque_per_volt` knob.
Old `linear_voltage_back_emf` configurations are rejected with a migration hint;
use the current `sim/motor_model.yaml` schema.

The runner quantizes the temperature to whole degrees C, saturates it to the
firmware conversion's 0–255 range, and supplies all twelve board channels using
the existing robot mapping. The shared uint16 telemetry then updates
`Motor::GetTemperature()` through either memory or serial/PTY transport. No
temperature bypasses the protocol. Board/watchdog resets leave the plant hot;
a new simulation run restores `initial_c`. Old custom configurations using
`fixed_temperature_c` must migrate to the `thermal` section.

CSV columns `<motor>_temperature_c` show the controller's received values;
`<motor>_plant_temperature_c` show the continuous physical approximation for
comparison. The run summary includes total `watchdog_resets` and the subset
`thermal_resets`. Use `--motor-config=path/to/motor.yaml` to vary thermal parameters
without changing hardware calibration.

The shaft torque ceiling comes from the MJCF actuators: currently 3,000 gf·cm,
or 0.2941995 N·m, set in `sim/robot_parameters.json`. MuJoCo applies the differential
and 9:1 transmission once. The board adapter neither multiplies torque by nine
again nor substitutes independent leg-joint actuators. The simulated Z encoders
reproduce each YAML `ref_z` reading at the positive
Z stop; they still transmit only single-turn counts, not true joint positions
or known rotor revolutions. I/O absolute reference alignment remains provisional.
All twelve actuator names must map uniquely to the six boards in the original YAML.

The runner uses integer microseconds, 500 us physics steps and 5 ms controller
updates (1 ms for `stand`, `bob`, `init`, and `standup`). `--step-us=250` provides a smaller-step comparison;
supported steps must divide the scenario's control period.
The link models 115200 baud as ten bits per byte rounded upward to
whole microseconds, 200 us one-way latency, and bounded queues. Baud zero is an
explicit ideal-link option in a separate motor configuration, not the default.

Each tick samples plant rotor positions into the board sensor registers, pumps
client/emulator communication, runs due controller updates, applies board outputs,
and advances physics. Newly generated control commands enter the transport at
its next pump. Incoming sensor samples must cross the decoder before control
uses them. Control PID timing accounts for elapsed time when backpressure delays
updates. Rendering is independent; `--realtime` only paces the runner.

Fresh runs reconstruct the clock, links, emulators, controller/behavior/PID state
and MuJoCo state. Repeated identical runs on the same build produce identical
CSV traces. This is the complete-reset mechanism; there is not yet an interactive
in-process reset command for the entire runner. Board reset/reconnect is tested
independently and does not teleport the physical model.

## Six virtual serial ports

The same board emulator core can serve real terminal file descriptors:

```sh
# Terminal 1: starts six PTYs and writes a temporary config with their addresses.
bazel run --config=pc //src/odroid_controller/controller:controller -- \
  --backend=sim --pty-server --suspended --duration=60 --trace=/tmp/quadbot-pty.yaml

# Terminal 2: uses the actual serial transport against those virtual ports.
bazel run --config=pc //src/odroid_controller/controller:controller -- \
  --backend=serial --no-joystick --config=/tmp/quadbot-pty.yaml --duration=10
```

In PTY-server mode `--trace` names the device-configuration overlay, not a CSV.
The original calibration file is never rewritten. The server paces physics in
real time and accepts commands from the external client; it does not run its own
production controller concurrently. The serial example verifies setup and stop,
not a gait. OS scheduling prevents PTY tests from promising deterministic timing.

## Current initialization and stand-up validation

The default `init` start completes at 25.05 s including board startup. FR/BL
then FL/BR run as diagonal pairs. The controller leaves theta free during the
folded gamma/Z searches, actively holds waiting legs, backs off and repeats each
probe, checks the Z encoder phase, and commits references only after all four
legs finish. A 26.5 s deadline after board startup prevents endless recovery.
This is not a guarantee from arbitrary configurations: three additional
collision-free initial pose samples with the earlier weak motor model encountered trapped legs or inconsistent
contact and stopped. One also contacted another leg; diagonal pairing alone
does not prove collision-free motion from arbitrary starts.

Homing now uses damped, rate-limited position commands with bounded integrals
and scales the coupled motor command vector together. A 40 ms velocity filter
combined with excessive derivative gain produced oscillation; the filter is
now 4 ms. With the stronger 3,000 gf·cm / 28 V motor model, proportional and
derivative gains are one quarter of the prior homing gains, while integral gains
remain unchanged. The per-motor command ceiling remains 24 V. The generalized
theta floor-probe cap is 2 V (1 V per I/O motor), preserving a light touch; the
folded-leg unloading command is also reduced. Hardware calibration and contact
geometry are unchanged.

`standup` runs `init`, then raises and folds the legs, swings them along the
upper arc into the standing orientation, and extends toward the existing stand
keyframe. A separate scratch model computes the static floor-to-stand frame
conversion; running plant positions never feed the controller. The final driven
hinges differ from the stand keyframe by one full turn, avoiding the lower arc
through the floor. Encoder feedback gates waypoint progression and timeout.
There is no startup support fixture or state reset. With the 5.05 kg CAD mass
and 3,000 gf·cm / 28 V motor approximation, the default headless check finishes
init at 25.05 s and stands at 38.38 s (25.13 s / 38.45 s with a 250 us step).
It holds four-foot contact through 50 s at the default 500 us step,
with no inter-leg contacts or controller faults. Chassis height while holding is
0.236–0.239 m; during init it remains on its belly at 0.0575 m. This is one default
starting pose, not a guarantee of recovery from arbitrary configurations.
Use `--scenario=standup` (default 50 s) to inspect it.

The parallelogram constraints preserve the intended assembly branch; the runner
rejects branch error above 0.05 rad. Summaries report branch error, floor
penetration, inter-leg contact count, homing completion time and stand-up result.
Small point closure error alone cannot establish mechanical correctness.

The historical stand/bob validation below used the stronger motor approximation.
The existing tight pose/trajectory assertions do not pass with the corrected
weaker motors; those scenarios require further adjustment. Their independent
rotor hold gains were rescaled for the changed voltage-to-torque gain, but that
does not restore the old force capability or validate standing on hardware.

## Build and checks

The C++ MuJoCo target uses headers and `libmujoco.so.3.17.0` from the already
hashed, pinned x86-64 wheel via `sim/deps/mujoco_native.json`. A platform selection
adds this dependency only to the PC executable. Hardware-only Odroid builds keep
MuJoCo and the emulator out of their link graph. The obsolete Gazebo flag remains
as a compatibility label but no longer selects a separate controller type.

```sh
bazel test --config=pc \
  //src/odroid_controller/tests:board_tests \
  //src/odroid_controller/tests:angle_tests \
  //sim:motor_plant_test //sim:native_runner_test //sim:controller_viewer_test \
  //sim:robot_model_test //sim:smoke_test
bazel build --config=pc //sim:controller_viewer
bazel build --config=pc //src/mcu_firmware/controller:mcu_controller
```

Tests cover wire units/framing, nonzero encoder startup and wraps, fragmented
one-byte transport, both motor channels, calibration setup, GET lifetime,
stop/reset/reconnect, watchdog versus command loss, timeouts, PTY serial I/O,
shaft force limits, encoder and temperature feedback, analytic heating/cooling,
thermal parameter validation, watchdog deadlines and thermal reset persistence,
repeated native traces, control stop,
timestep comparison, and model load/closure/contact checks. The six-port PTY
integration test starts only virtual devices.

Verified on 2026-10-25: all six test targets above pass; the controller viewer
and transitioned active AVR firmware build; Odroid controller analysis succeeds
with `--nobuild`. A 30-second suspended native `hold` run completed without
MuJoCo warnings (peak closure error about 0.161 mm). The viewer target and CLI
were checked, but its desktop window was not opened during this implementation.
No physical controller was executed, no deployment was performed, and the
hardware calibration YAML was left unchanged. A subsequent `CMD_LIMITS` fix
changed the shared firmware/host payload declaration; it was built but not flashed.

Temperature/watchdog follow-up: `motor_plant_test`, `board_tests` and
`native_runner_test` pass with thermal dynamics, decoded temperature feedback,
short/expired stalls, thermal resets and PTY coverage. Thermal parameters still
require measurement; passing tests establish software behavior, not hardware
temperature accuracy.

Standing follow-up: `native_runner_test`, `board_tests`, `angle_tests` and
`motor_plant_test` pass. The eight-second `stand` trace repeats exactly and stays
upright after support release at both 500 us and 250 us physics steps, with all
four feet contacting the floor and chassis height about 0.237 m in the current
YAML-derived extended pose. Tests check pose error, torque limits and thermal resets without
restoring support. This validates the short known-pose scenario, not sustained
thermal performance or the existing homing/stand-up behavior.

Viewer shutdown follow-up: `controller_viewer_test` passes with asynchronous
renderer teardown, real SIGINT/SIGTERM delivery, repeated interruption and a
child that ignores termination. Desktop checks of the actual MuJoCo viewer
also pass: the eight-second standing run exits with code 0, Ctrl+C with 130,
and SIGTERM with 143, without an exit-time segmentation fault.

Remaining validation gates: measured motor response, absolute CAD/encoder
alignment, physical stops and real homing, sustained thermal performance,
and eventual comparison against real hardware. Those cannot be established by
successful protocol tests or the independent demo servos.
