"""Native Qt inspector: simulation launcher or direct TCP robot connection."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

from PySide6 import QtCore, QtGui, QtWidgets
from python.runfiles import runfiles

from dashboard.native_client import ControllerClient
from dashboard.native_view import InspectorWindow
from sim.controller_viewer import stop_child


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="attach to an existing controller/SSH forward instead of launching simulation")
    parser.add_argument("--controller-port", type=int, default=9876)
    parser.add_argument("--scenario", choices=("stand", "bob", "init", "standup", "idle", "hold", "pulse", "stop"), default="stand")
    parser.add_argument("--duration", type=float, help="simulation duration, seconds (standup: 50; init: 30; others: 30)")
    parser.add_argument("--reference", help="known-pose reference JSON for an existing controller")
    parser.add_argument("--read-only", action="store_true", help="disable motion in the launched simulation; Stop remains available")
    parser.add_argument("--no-viewer", action="store_true", help="omit the separate MuJoCo simulation window")
    parser.add_argument("--no-3d", action="store_true", help="omit sensed 3D, useful without OpenGL")
    parser.add_argument("--suspended", action="store_true")
    parser.add_argument("--show-collisions", action="store_true")
    parser.add_argument("--motor-config")
    parser.add_argument("--profile-output", help="write UI timing/sample counts to JSON on exit")
    parser.add_argument("--screenshot-output", help="save the inspector window during the run")
    parser.add_argument("--capture-at", type=float, default=10, help="seconds after UI startup to take screenshot")
    args = parser.parse_args()
    if args.duration is None:
        args.duration = 50 if args.scenario == "standup" else 30
    if not 0 < args.controller_port < 65536:
        parser.error("controller port must be in 1..65535")
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    if not math.isfinite(args.capture_at) or args.capture_at <= 0:
        parser.error("capture-at must be finite and positive")
    if args.reference and not args.connect:
        parser.error("simulation supplies its own pose reference; --reference requires --connect")
    if args.connect and (args.motor_config or args.suspended or args.scenario != "stand"):
        parser.error("simulation options cannot be used with --connect")
    reference = json.loads(Path(args.reference).read_text()) if args.reference else None
    if not args.connect:
        try:
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(("127.0.0.1", args.controller_port))
        except OSError:
            parser.exit(1, f"Controller port {args.controller_port} is in use; stop the previous instance or choose --controller-port=PORT.\n")

    # The GUI timer limits redraws. Avoid a separate vsync wait for each of
    # the four OpenGL widgets, especially alongside the MuJoCo viewer.
    surface = QtGui.QSurfaceFormat.defaultFormat()
    surface.setSwapInterval(0)
    QtGui.QSurfaceFormat.setDefaultFormat(surface)
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setStyle("Fusion")
    client = ControllerClient(args.controller_port, allow_motion=not args.read_only)
    window = InspectorWindow(client, reference, startup_stand=not args.connect and args.scenario in ("stand", "bob"),
                             enable_3d=not args.no_3d)
    children = []
    trace_dir = None
    exit_code = 0
    started = time.monotonic()
    cpu_started = time.process_time()
    monitor = QtCore.QTimer()
    monitor.setInterval(100)

    def check_children():
        nonlocal exit_code
        for child in children:
            code = child.poll()
            if code is not None:
                if code:
                    print(f"Inspector child exited with code {code}", file=sys.stderr)
                    exit_code = 1
                window.close()
                app.quit()
                return

    monitor.timeout.connect(check_children)
    previous_handlers = {}

    def shutdown(signum, _):
        nonlocal exit_code
        exit_code = 128 + signum
        window.close()
        app.quit()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, shutdown)
        if not args.connect:
            rf = runfiles.Create()
            env = dict(os.environ, **rf.EnvVars())
            command = [rf.Rlocation("_main/src/odroid_controller/controller/controller"), "--backend=sim", "--realtime",
                       f"--scenario={args.scenario}", f"--duration={args.duration}", f"--dashboard-port={args.controller_port}"]
            if not args.read_only:
                command.append("--dashboard-control")
            if args.suspended:
                command.append("--suspended")
            if args.motor_config:
                command.append(f"--motor-config={Path(args.motor_config).resolve()}")
            if not args.no_viewer:
                trace_dir = tempfile.TemporaryDirectory(prefix="quadbot-native-")
                trace = Path(trace_dir.name) / "state.csv"
                command.append(f"--trace={trace}")
            children.append(subprocess.Popen(command, env=env, start_new_session=True))
            if not args.no_viewer:
                viewer = [rf.Rlocation("_main/sim/controller_viewer"), f"--attach-trace={trace}", f"--scenario={args.scenario}"]
                if args.suspended:
                    viewer.append("--suspended")
                if args.show_collisions:
                    viewer.append("--show-collisions")
                children.append(subprocess.Popen(viewer, env=env, start_new_session=True))
        window.show()
        client.start()
        monitor.start()
        if args.screenshot_output:
            QtCore.QTimer.singleShot(int(args.capture_at*1000), lambda: window.save_screenshot(args.screenshot_output))
        print(f"Native inspector: TCP 127.0.0.1:{args.controller_port}; close the window or Ctrl+C to stop.", flush=True)
        app.exec()
    finally:
        monitor.stop()
        client.close()
        for child in children:
            stop_child(child)
        if trace_dir:
            trace_dir.cleanup()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        if args.profile_output:
            values = sorted(window.refresh_times)
            profile = {"elapsed_s": time.monotonic()-started, "cpu_s": time.process_time()-cpu_started,
                       "samples": window.samples, "history_samples": window.history.count,
                       "refreshes": window.refreshes, "scene_paints": window.robot_view.paints if window.robot_view else 0,
                       "mean_refresh_ms": 1000*sum(values)/len(values) if values else None,
                       "p95_refresh_ms": 1000*values[int(.95*len(values))] if values else None}
            if window.robot_view:
                paint_times = window.robot_view.paint_times
                profile.update(renderer=window.robot_view.renderer,
                               mean_scene_paint_ms=1000*sum(paint_times)/len(paint_times) if paint_times else None)
            Path(args.profile_output).write_text(json.dumps(profile, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
