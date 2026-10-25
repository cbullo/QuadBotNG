# Build architecture and Bzlmod migration

Updated 2026-10-25. Subsequent controller/simulator integration is documented in
[simulated-boards.md](simulated-boards.md); the earlier migration findings below
describe their validation stage. [README](../README.md) contains the supported commands.

Odroid cross-toolchain restoration is now deferred in [TODO.md](../TODO.md).
The user clarified that crosstool-NG was chosen to target the Odroid's libc,
independently of the development PC. Preserve that compatibility requirement;
installing the current PC distribution's ARM64 cross libraries is not sufficient
evidence of compatibility. Simulator work is the current priority.

## Platform boundaries

The host entry point is `//src/odroid_controller/controller:controller`, using
`//src/odroid_controller:controller_lib` and the shared binary protocol library.
The same C++ sources build for PC and Odroid by selecting a target platform.
`PC` explicitly means x86-64 Linux, not a portable Windows/macOS target.

`//:Odroid-C4` selects the existing AArch64 toolchain, which runs on x86-64 Linux.
Its compiler and sysroot retain the original layout; `AARCH64_TOOLCHAIN_ROOT`
can now be overridden with `--define` instead of editing the toolchain source.

`//:RCTimer` inherits the AVR/bare-metal constraints from `@AvrToolchain` and adds
ATmega328P, 16 MHz and hardware UART. `controller_hex` converts the firmware ELF
to Intel HEX. `mcu_controller` wraps that target with the transition defined in
`src/mcu_firmware/defs.bzl`, forcing the AVR platform and disabling `gnu99` while
preserving other requested features. The wrapper forwards both output files
and runfiles. This allows `deploy_controller` to include AVR firmware even when
its outer platform is Odroid (or PC for local packaging checks).

Platform selection and simulation backend selection are separate concerns.
Building with `--config=pc` changes the compiler, not the runtime device backend.
The parent host package owns the `simulator` setting; the executable package's
old flag spelling is an alias. The unfinished Gazebo implementation is retained
as a manual target. No mock backend or physics engine was implemented here.

The planned mock boundary is the two-motor board command/feedback interface
currently supplied by `BLDCDriverBoard` / `ControllerType`. Preserve the shared
protocol, voltage units, encoder feedback, board states and the three-motor
leg differential. AVR FOC, PWM registers, sensor I2C and watchdog code remain
firmware; a PC simulation needs a model of their externally observable behavior,
not a native compilation of those hardware routines.

## Dependency migration

`MODULE.bazel` is authoritative and `.bazelrc` disables WORKSPACE resolution.
`.bazelversion` pins the tested Bazel 8.4.2 baseline. Bazel 9 compatibility was
not pursued as part of this migration.

Registry modules provide skylib, platforms, rules_cc, rules_shell,
rules_foreign_cc, fmt, yaml-cpp and GoogleTest. Non-module archives use
`use_repo_rule(http_archive)` with fixed revisions and SHA-256 hashes. Existing
repository names are preserved. The former moving SimpleFOC fork is pinned to
`23cc4d23faf0b56e907b6365da610e69246c3d9a`; yaml-cpp is pinned to 0.8.0.
Arduino AVR core stays at 1.8.5, fmt at 8.1.1 and libwebsockets at 4.3.2.
The generated module lockfile records the resolved dependency graph.

`toolchain/extensions.bzl` calls the existing AVR repository generator without
its WORKSPACE-only toolchain-registration macro. `MODULE.bazel` registers the
ATmega328P toolchain and the local AArch64 toolchain explicitly. The small
`embedded_bzlmod.patch` replaces the removed label-resolution helper, updates
CPU/OS constraint labels, and adds an explicit generated C++ toolchain load.
The upstream generator still discovers AVR tools even for PC builds; separating
that discovery from platform metadata is a possible future build improvement.

Local C++/shell BUILD rules have explicit loads. The Arduino EEPROM library is
header-only and no longer declares an empty C++ glob. The global `-pie` compile
flag and machine-specific ccache writable path were removed from `.bazelrc`;
local settings belong in the ignored `.bazelrc.local`.

The compilation database keeps the existing `//:example_compdb` target with its
corrected host-binary label. Deployment runfiles now use `_main` rather than
`__main__`. The deployment script was not executed against a robot.

Reference: the [Bazel Bzlmod migration guide](https://bazel.build/versions/8.0.0/external/migration)
explains registry dependencies, repository rules, extensions and module-level
toolchain registration. The retained AVR generator comes from
[EmbeddedSystemsBuildScripts](https://github.com/es-ude/EmbeddedSystemsBuildScripts/tree/v1.0.2).

## Python simulator dependencies

`rules_python` 1.2.0 is now an explicit dependency (the same version already
selected transitively by the existing build). Its toolchain supplies Python
3.12, currently mapped to 3.12.8 by that pinned rules release. The `sim_deps`
pip extension installs wheels from the hashed `sim/deps/requirements.txt` lock;
MuJoCo's wheel includes the native physics library. System Python packages and
a separate virtual environment are not used.

The `//sim:viewer` and `//sim:smoke_test` targets require x86-64 Linux. The
headless smoke test exercises Gymnasium's packaged HalfCheetah model, seeded
reset and a finite rollout with advancing physics time. It does not validate
our robot geometry, differential, controller, or motor-board emulation.
The viewer requires desktop OpenGL support; it forwards MuJoCo's usual flags.
Lock regeneration is exposed as `//sim/deps:requirements.update`, with the
network-dependent lock consistency test marked manual. See the README for
commands. This Python integration does not yet expose MuJoCo as a native C++
library to the Odroid controller.

## Disabled and legacy components

The user confirmed that the old firmware programs are likely deprecated and
should not be revived. Their sources and target names are retained. A diagnostic
build found missing SimpleFOC headers in `encoder` and `init_foc`, obsolete APIs
in `calibration`, and conflicting upstream/custom FOC definitions in `open_loop`.
These are outside the active build baseline; avoid blanket `//...` builds.

The user also confirmed that the web server is incomplete and needs rework.
Its include, global instance and startup call have been removed from the host
entry point, along with its controller link dependency and deployment assets.
The server library and libwebsockets target are manual targets retained for
future work. Default controller builds do not compile or link them.

## Validation

Verified with Bazel 8.4.2, host GCC 13.3.0 and AVR GCC 7.3.0:

- PC controller compilation and linking; ELF architecture is x86-64.
- All nine existing angle tests pass. These use a copied `TestMotor`, so they
  are not comprehensive coverage of production motor/transport behavior.
- MuJoCo 3.17.0 / Gymnasium 1.4.0 headless smoke test passes with the managed
  Python toolchain. The Bazel viewer also launches on the local COSMIC/Wayland
  desktop: a screenshot confirms HalfCheetah, the textured ground, shadows,
  reflections and viewer controls render with Run selected. No startup or
  OpenGL errors appeared; the upstream model emits a `settotalmass`
  deprecation warning. PC controller and transitioned AVR builds still pass
  with the Python dependency extension enabled.
- Active AVR controller builds both directly with `--config=rctimer` and via
  `mcu_controller` from an outer PC build. ELF architecture is AVR; reported
  flash usage is 9,740 bytes and RAM usage is 1,024 bytes.
- Local PC/AVR deployment bundle builds and contains the firmware HEX under
  `_main/src/mcu_firmware/controller/`.
- Compilation database generation produces valid JSON and existing execution
  directories.
- Odroid deployment target analysis succeeds with `--nobuild`. Full AArch64
  compilation remains unverified because the configured compiler is absent.

Existing source warnings, including missing returns, remain for later review.
No controller was run, no firmware was flashed, and no deployment was performed.
