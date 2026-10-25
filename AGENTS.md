# Repository context

QuadBotNG controls a 12-DOF quadruped: an Odroid-C4 Linux host and six
ATmega328P boards, each driving two brushless motors. The user confirms that
each leg uses a **three-motor differential**. Motor angles are not independent
leg joint angles; preserve the coupling and per-leg sign conventions.

Read `docs/repository-notes.md` for the architecture, protocol, simulator assets,
and initial build findings. These notes describe the inspected code, not a
validated physical model or working simulator.

## Working hints

- The host entry point is `src/odroid_controller/controller/main.cc`; its Bazel
  target is `//src/odroid_controller/controller:controller`.
- Hardware mapping and calibration live in
  `src/odroid_controller/controller/config/robot_config.yaml`. Preserve measured
  calibration values when changing infrastructure or introducing simulation.
- The active firmware uses the in-tree `base/simple_foc` implementation, not
  simply the upstream SimpleFOC archive declared in `MODULE.bazel`.
- The existing Gazebo adapter and simulator build branches are unfinished.
  `sim/` contains useful geometry and linkage references, not a validated setup.
- The host uses FR/FL/BR/BL names; model assets use RF/LF/RB/LB. Check mappings
  explicitly. Board channel ordering also varies between legs.
- Deployment scripts stop the remote robot service, copy files, and provide
  flashing/start operations. Do not use deployment or hardware execution as
  ordinary build verification; use them only when requested.
- Build dependencies now use Bzlmod with Bazel 8.4.2 pinned. Read `README.md`
  and `docs/build.md` for current platform configurations and validation;
  the build findings in `docs/repository-notes.md` describe the earlier baseline.
- Preserve `//:RCTimer` and `//:Odroid-C4`; `//:PC` builds the host code for
  x86-64 Linux. Firmware wrappers must continue to force the AVR platform.
- The user says calibration/encoder/init_foc/open_loop firmware is likely
  deprecated: do not revive it as part of build or simulation work.
- The web server is incomplete and disabled pending rework. Keep its startup
  and link dependency out of the active controller until rework is requested.
- Odroid toolchain restoration is deferred in `TODO.md`. Its target libc must
  remain compatible with the Odroid, independently of the PC's libraries;
  preserve the crosstool-NG approach until that work resumes.
- Simulator research selects MuJoCo as the initial physics engine, with a
  Gymnasium interface and a later MJX / MuJoCo Playground training path. Read
  `docs/simulator-research.md` for the comparison and validation gates.
  `//sim:viewer` and `//sim:smoke_test` use Bazel-managed Python and locked
  dependencies in `sim/deps/`; update them with
  `bazel run --config=pc //sim/deps:requirements.update`. The smoke test uses
  upstream HalfCheetah, not a validated quadruped model. The PC controller
  still uses hardware transport; no mock backend exists yet.
