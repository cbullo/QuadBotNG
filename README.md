# QuadBotNG

Control software for a 12-DOF quadruped with a three-motor differential per leg,
an Odroid-C4 Linux host and six two-motor ATmega328P power controllers.

Simulator research selects **MuJoCo**, with Gymnasium initially and a later
MJX / MuJoCo Playground training path. See the
[simulator comparison and implementation gates](docs/simulator-research.md).
Dependency tooling is available through Bazel; the robot simulator itself is
not implemented yet. Odroid cross-toolchain restoration is
[deferred](TODO.md).

## Build setup

Dependencies and toolchain registration use **Bzlmod** (`MODULE.bazel` and
`MODULE.bazel.lock`). Bazel **8.4.2** is pinned in `.bazelversion`; use Bazelisk
or an installation providing that version. `WORKSPACE` is only a marker.

| Configuration | Platform target | Compiler / purpose |
| --- | --- | --- |
| `--config=pc` | `//:PC` | Local C++ compiler, x86-64 Linux; host controller and tests. |
| `--config=odroid` | `//:Odroid-C4` | AArch64 Linux cross compiler; robot host. |
| `--config=rctimer` | `//:RCTimer` | AVR GCC, ATmega328P at 16 MHz, hardware UART; power-controller firmware. |

The original platform labels remain usable with `--platforms`. The `rctimer`
configuration also disables the AVR rules' default `gnu99` feature for C++.
C++17 is configured in `.bazelrc`.

Install a host C++ compiler and the AVR GCC/binutils/libc tools. The retained
AVR repository generator discovers its tools during repository setup, including
for PC builds; it does not compile AVR firmware unless a firmware target is
requested. See [build notes](docs/build.md) for migration details and limitations.

## PC controller and tests

```sh
bazel build --config=pc //src/odroid_controller/controller:controller
bazel test --config=pc //src/odroid_controller/tests:angle_tests
bazel build --config=pc //:example_compdb
```

The compilation database is `bazel-bin/compile_commands.json`.

The PC binary currently uses the hardware board backend; it is not yet a
simulator. Running it attempts to open devices from `config/robot_config.yaml`
and the joystick. Simulation will reuse the host control code with a mock or
simulated board backend instead of compiling AVR register/PWM/I2C code for PC.
The existing Gazebo backend is unfinished. Its simulator flag remains a
placeholder, not a working launch mode.

**The web server is disabled pending rework:** the controller does not link or
start it, and deployment bundles omit its browser assets. Its source and manual
build targets remain available.

## Simulator tooling (PC)

Bazel downloads Python 3.12 and the locked MuJoCo 3.17.0 / Gymnasium 1.4.0
dependencies. No virtual environment or separate MuJoCo installation is needed.

```sh
# Headless physics and Gymnasium integration check:
bazel test --config=pc //sim:smoke_test

# Desktop viewer (opens empty; drag in a model):
bazel run --config=pc //sim:viewer
# Or load a model, with paths relative to your current directory:
bazel run --config=pc //sim:viewer -- --mjcf=path/to/model.xml
```

The viewer needs a desktop session and working system OpenGL drivers. CPU
physics and the headless check need no CUDA installation or display. These
targets support x86-64 Linux (`//:PC`); they do not use the Odroid sysroot.
The check uses Gymnasium's packaged HalfCheetah example, not our quadruped.
The existing URDF still needs conversion and validation of its differential
and closed linkage before it can serve as the robot simulation model.

Direct dependencies live in `sim/deps/requirements.in`; all resolved versions
and wheel hashes are recorded in `sim/deps/requirements.txt`. After editing
the input, regenerate the lock on the PC and rerun the smoke test:

```sh
bazel run --config=pc //sim/deps:requirements.update
```

The updater needs network access. Ordinary builds use the committed lock and
download dependencies on first use. To refresh transitive versions too, append
`-- --upgrade` to the updater command.

## Active AVR firmware

```sh
# Explicit AVR configuration, producing controller_hex.hex:
bazel build --config=rctimer //src/mcu_firmware/controller:controller_hex

# Wrapper switches to RCTimer even when the outer build targets PC or Odroid:
bazel build --config=pc //src/mcu_firmware/controller:mcu_controller
```

The calibration, encoder, init_foc and open_loop firmware programs are likely
deprecated. They are retained as references and are not maintained by this
migration. Build the named active targets instead of `//...`.

## Odroid and deployment

The cross toolchain retains the crosstool-NG layout:

```text
<root>/bin/aarch64-linux-gnu-{gcc,ar,ld,...}
<root>/aarch64-linux-gnu/sysroot
```

Its default root remains `/home/tomasz/x-tools/aarch64-linux-gnu`. To override it,
create an ignored `.bazelrc.local` containing, for example:

```text
build:odroid --define=AARCH64_TOOLCHAIN_ROOT=/opt/x-tools/aarch64-linux-gnu
```

```sh
bazel build --config=odroid //src/odroid_controller/controller:controller
# Build the mixed AArch64 host + AVR firmware bundle without deploying:
bazel build --config=odroid //src/odroid_controller/controller:deploy_controller
```

Only when deployment is intended:

```sh
bazel run --config=odroid //src/odroid_controller/controller:deploy_controller -- all
```

The existing script targets `robot@192.168.0.12`, stops the robot service and
copies the bundle. Firmware flashing and service restart are currently commented
out in that script; `all` does not enable them. Its runfiles path now uses
Bzlmod's `_main` directory.
