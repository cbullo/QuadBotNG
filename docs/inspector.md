# QuadBot native inspector

The inspector uses PySide6 Essentials (community Qt) and PyQtGraph. It runs
locally without a browser, HTTP server, WebSocket bridge, ROS, account or paid
service. Dependencies are pinned in `sim/deps/requirements.in` and the hashed
lock. The former Lichtblick UI, bridge, layout and release download were removed.

## One command

```sh
bazel run --config=pc //sim:inspector -- --scenario=bob --duration=20
```

This starts the production C++ controller, six simulated boards, native MuJoCo
physics, its existing viewer and the native inspector. Both windows observe the
same simulation. Closing either window, Ctrl+C, or reaching the duration reaps
the launched children. `//sim:native_inspector` is an alias for the same app.

`stand` is the default scenario, with a default duration of 30 seconds. `bob`
gently crouches and rises every four seconds after startup. The existing motor,
thermal and fault models remain active; long runs can overheat and stop. The
thermal model and CAD alignment remain bring-up approximations.

`init` replays the production reference-pose sequence from an upright chassis
resting on its belly, with all four bent legs pointing up:

```sh
bazel run --config=pc //sim:inspector -- --scenario=init
```

It has no startup support fixture and cannot be combined with `--suspended`.
The status line reports each active leg's initialization stage; the MuJoCo viewer
shows actual simulated geometry. The encoder-only 3D panel remains unreferenced
unless supplied an explicit reference. CSV traces add `<leg>_init_stage` (`-1`
when no initialization control is attached, otherwise the enum in
`InitializationLegControl`). STOP cancels the sequence, including during startup.
`standup` runs initialization and then a staged rise toward the existing stand
keyframe, with a 50-second default:

```sh
bazel run --config=pc //sim:inspector -- --scenario=standup
```

It raises the legs, folds them, swings them over the upper arc, and extends.
Each preparation waypoint must be reached before proceeding. Failed homing,
STOP, or a stand-up waypoint timeout prevents further movement. There is no
startup fixture or pose reset. With the 5.05 kg CAD mass and 3,000 gf·cm at 28 V
motor approximation, the default check finishes init at 25.1 s and stands at
38.4 s, holding all four feet on the floor through 50 s without inter-leg contact.
Hardware mass, motor dynamics, and arbitrary starting configurations still need validation.

The init default is 30 seconds. FR/BL probe together, then FL/BR; the inactive
pair is actively held. Homing retains free theta during the folded gamma/Z
searches so contact can redirect a leg. Probes back off and repeat; time, travel
and encoder-phase failures stop the sequence. Completion holds the parked pose
and does not automatically stand. This remains experimental hardware homing.

```sh
# Inspector alone, still running the same native physics/controller:
bazel run --config=pc //sim:inspector -- --scenario=bob --duration=20 --no-viewer

# Observation and Stop only:
bazel run --config=pc //sim:inspector -- --read-only
```

Other simulation options: `--suspended`, `--motor-config=PATH`, and
`--show-collisions` (for the separate MuJoCo viewer). Use `--no-3d` when OpenGL
is unavailable. The inspector connects only to `127.0.0.1`; its single TCP port
is 9876, configurable with `--controller-port=PORT`. A conflicting listener is
reported before starting the simulation.

## Telemetry and plots

The controller emits JSON-line snapshots at approximately 50 Hz, independently
of its control-loop frequency. The inspector retains every received snapshot
in a 3,000-sample ring buffer (about 60 seconds). Plots and sensed geometry target
a 30 Hz display cadence; the current-value table refreshes at 5 Hz. Actual redraw
rate depends on window size, graphics drivers and competing viewers. These are
sampled diagnostics, not a lossless trace of the high-frequency control loop.

Three plot selectors offer voltage commands (V), accumulated encoder angles
(rotor rad), temperatures (°C), raw encoder counts and estimated rotor velocity
(rad/s). All twelve motors are initially shown. Uncheck motors in the table to
hide their curves. Disable **Follow live** to pan/zoom over buffered history;
data collection continues. Plot decimation affects drawing only, not storage.

Voltage is the last fully transmitted voltage command, not measured bus voltage.
Velocity is the finite difference between snapshots and is unavailable when
encoder feedback is stale. The table shows encoder age in ms; negative age
means no valid sample. Values before initialization are not valid measurements.

The IMU readout shows chassis-frame XYZ angular velocity (rad/s) and specific
force (m/s²). It is currently an ideal simulated gyro/accelerometer, with no
attitude estimate. It reports unavailable on hardware and stale connections,
and waits while the artificial startup support fixture is active. TCP snapshots
include an `imu` object with `time`, `valid`, `frame: "base_link"`, and XYZ
objects `angular_velocity_rad_s` and `specific_force_m_s2`; absent sensors are
`null`. See [sensor conventions and limitations](physical-model.md#simulated-imu).

The 3D view uses persistent OpenGL meshes and updates their transforms/colors.
It reconstructs the linkage from decoded leg coordinates, preserving the
three-motor differential and FR/FL/BR/BL sign conventions. MuJoCo evaluates
kinematics only in this view; it does not run another physics simulation.
Colors indicate temperature. The chassis is anchored because world position
is not sensed. Geometry uses simplified collision shapes, not detailed CAD
meshes. Camera orbit/zoom is handled by PyQtGraph's 3D widget.

## Commands

1. **STOP** overrides scripted motion and asks all boards to stop.
2. Once controls have stopped, **Hold current pose** captures the sensed pose.
3. Select a leg, adjust theta/gamma/z in radians and press **Send leg target**.
   `z` is an angular coordinate, not body height. Values populate from feedback
   when Hold is acquired or the selected leg changes.

The controller enforces fresh feedback, configured gamma/Z bounds and a ±0.15
rad envelope around the captured pose. It rejects motion unless enabled by
`--dashboard-control`. Each request has a monotonic sequence and an explicit
acceptance/rejection message. Acceptance means accepted by the host, not that
an MCU has completed the move.

The GUI thread renews a 500 ms host lease every 100 ms while it owns Hold and
receives fresh telemetry. A frozen GUI cannot renew it. Closing/disconnecting
stops inspector-owned control; commands are never replayed after reconnect.
The app requires a restart after connection loss to establish a new pose
reference. Host faults and MCU watchdogs retain their existing behavior.

## Real robot connection

The native inspector uses exactly the same controller TCP endpoint for simulated
and physical boards. It does not open serial ports itself. On an already
configured robot, the controller supports `--backend=serial --no-joystick
--dashboard-port=9876` and optionally `--dashboard-control` for motion.

Forward the endpoint and attach from the PC:

```sh
ssh -N -L 9876:127.0.0.1:9876 robot@ROBOT_ADDRESS
bazel run --config=pc //sim:inspector -- --connect --read-only
```

Remove `--read-only` when motion is intended and enabled on the controller.
`--connect` launches neither simulation nor the separate MuJoCo viewer and has
no automatic duration limit. The Odroid needs no Python/Qt/MuJoCo UI packages.
The endpoint admits one client: do not attach two inspectors to the same port.
Hardware deployment/execution was not used to verify this integration.

Encoder accumulation does not establish absolute mechanical references. Numeric
telemetry and plots work without a reference; sensed 3D requires a known pose:

```sh
bazel run --config=pc //sim:inspector -- --connect --reference=/absolute/path/reference.json
```

Reference JSON has all four host leg names, each with `sensed` values
`[theta,gamma,z]` and the corresponding CAD `[main,crank,shoulder]` in radians:

```json
{"FR":{"sensed":[0,0,0],"cad":[0,0,0]},
 "FL":{"sensed":[0,0,0],"cad":[0,0,0]},
 "BR":{"sensed":[0,0,0],"cad":[0,0,0]},
 "BL":{"sensed":[0,0,0],"cad":[0,0,0]}}
```

Zeros illustrate the format, not calibration. References belong to the current
controller session and are invalidated by missing/stale feedback. The launcher
knows the startup reference only for its own `stand`/`bob` scenarios. No hardware
calibration values have been changed.

## Validation and profiling

```sh
bazel test --config=pc //dashboard:native_test //dashboard:dashboard_test //sim:controller_viewer_test
bazel run --config=pc //sim:inspector -- --scenario=bob --duration=20 --no-viewer \
  --profile-output=/tmp/native-profile.json --screenshot-output=/tmp/native.png
```

Tests exercise the real simulated controller, native TCP framing/commands,
acknowledgements, target limits, lease expiration, sample retention, differential
kinematics, reference loss, Stop/startup ordering and subprocess cleanup.
The JSON profile reports sample counts, UI timing, process CPU and OpenGL renderer.

Free upstream components:

- [PyQtGraph (MIT)](https://www.pyqtgraph.org/)
- [PySide6 community edition (LGPL/GPL)](https://doc.qt.io/qtforpython-6/index.html)
- [PyOpenGL (BSD)](https://github.com/mcfletch/pyopengl)
