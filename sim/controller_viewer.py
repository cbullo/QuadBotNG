"""View snapshots from the production C++ controller's native simulation.

The native process owns simulation time and physics. This window only renders
its snapshots; closing it terminates that child process. Viewer pause controls
are not a simulation-clock control.
"""
import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import signal
import tempfile
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
from python.runfiles import runfiles


class ShutdownSignals:
    """Request shutdown without raising an exception inside native GUI code."""

    def __init__(self):
        self.signum = 0

    def __enter__(self):
        self.previous = {sig: signal.signal(sig, self.request)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
        return self

    def request(self, signum, _frame):
        self.signum = self.signum or signum

    def __exit__(self, *_):
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)


@contextmanager
def joined_passive_viewer(model, data):
    # In the pinned MuJoCo, launch_passive starts a daemon render thread and
    # Handle.close only requests exit. is_running becomes false BEFORE native
    # destruction finishes. Letting Python/GLFW exit then can segfault.
    # This single-window application starts no other Python threads during
    # launch, so capture the threads the public launcher creates and join them.
    previous = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data)
    threads = set(threading.enumerate()) - previous
    try:
        yield viewer
    finally:
        viewer.close()
        for thread in threads:
            thread.join()


def stop_child(child, timeout=2):
    """Reap our controller even if it ignores a termination request."""
    if child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
    else:
        child.wait()


def run(args, shutdown):
    assets = runfiles.Create()
    binary = assets.Rlocation("_main/src/odroid_controller/controller/controller")
    model_name = "quadbot_suspended.xml" if args.suspended else "quadbot.xml"
    model = mujoco.MjModel.from_xml_path(assets.Rlocation(f"_main/sim/mujoco/{model_name}"))
    data = mujoco.MjData(model)
    pose = "init" if args.scenario in ("init", "standup") else "stand" if args.scenario in ("stand", "bob") else "stance"
    mujoco.mj_resetDataKeyframe(model, data, model.key(pose).id)
    mujoco.mj_forward(model, data)
    if shutdown.signum:
        return
    if getattr(args, "attach_trace", None):
        # The inspector owns the controller; only render its snapshots here.
        with joined_passive_viewer(model, data) as viewer:
            display(viewer, model, data, Path(args.attach_trace), None, args, shutdown)
        return
    with tempfile.TemporaryDirectory(prefix="quadbot-controller-") as tmp:
        trace = Path(tmp) / "state.csv"
        command = [binary, "--backend=sim", "--realtime", f"--scenario={args.scenario}",
                   f"--duration={args.duration}", f"--trace={trace}"]
        if args.suspended:
            command.append("--suspended")
        if args.motor_config:
            command.append(f"--motor-config={Path(args.motor_config).resolve()}")
        environment = dict(os.environ)
        environment.update(assets.EnvVars())
        # The parent owns shutdown. A terminal Ctrl+C must not race a native
        # child exit against the viewer's signal handling and look like a crash.
        child = subprocess.Popen(command, env=environment, start_new_session=True)
        try:
            with joined_passive_viewer(model, data) as viewer:
                try:
                    display(viewer, model, data, trace, child, args, shutdown)
                finally:
                    # Reap the child before waiting for renderer teardown.
                    stop_child(child)
        finally:
            # Also covers a failure to launch the viewer.
            stop_child(child)


def display(viewer, model, data, trace, child, args, shutdown):
    reader = None
    pending = ""
    try:
        viewer.cam.distance = 0.9
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [0, 0, 0.25 if args.suspended else 0.15]
        if args.show_collisions:
            viewer.opt.geomgroup[2] = 0
            viewer.opt.geomgroup[3] = 1
        while not shutdown.signum and viewer.is_running():
            if reader is None and trace.exists():
                reader = trace.open()
            if reader is not None:
                pending += reader.read()
                records = pending.split("\n")
                pending = records.pop()
                for record in reversed(records):
                    if not record or record.startswith("time,"):
                        continue
                    values = np.fromstring(record, sep=",")
                    if len(values) < model.nq + 1:
                        raise RuntimeError("incomplete native snapshot")
                    with viewer.lock():
                        data.time = values[0]
                        data.qpos[:] = values[1:1+model.nq]
                        mujoco.mj_forward(model, data)
                    break
            viewer.sync()
            if child is not None and child.poll() is not None:
                if child.returncode and not shutdown.signum:
                    raise RuntimeError(f"native controller exited with {child.returncode}")
                break
            time.sleep(1 / 60)
    finally:
        if reader is not None:
            reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("idle", "hold", "stand", "bob", "init", "standup", "pulse", "stop"), default="stand")
    parser.add_argument("--duration", type=float, help="seconds (standup: 50; init: 30; others: 8)")
    parser.add_argument("--motor-config", help="override simulation motor/thermal/stance parameters")
    parser.add_argument("--attach-trace", help="render an inspector-owned trace without starting a controller")
    parser.add_argument("--suspended", action="store_true")
    parser.add_argument("--show-collisions", action="store_true",
                        help="show collision primitives instead of detailed visual meshes")
    args = parser.parse_args()
    if args.duration is None:
        args.duration = 50 if args.scenario == "standup" else 30 if args.scenario == "init" else 8
    with ShutdownSignals() as shutdown:
        run(args, shutdown)
        return 128 + shutdown.signum if shutdown.signum else 0


if __name__ == "__main__":
    raise SystemExit(main())
