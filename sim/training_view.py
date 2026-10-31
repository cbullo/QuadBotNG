"""Local training curves and a separate latest-policy evaluation world."""
import json
from dataclasses import replace
from pathlib import Path
import os
import signal
import subprocess
import sys
import time

import mujoco
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from python.runfiles import runfiles
import torch

from sim.controller_viewer import stop_child
from sim.training import StopRequest, load_policy, make_env
from sim.goals import (AXIS_BODY_POSE_GOALS, BODY_POSE_GOALS, POSE_BODY_POSE_GOALS,
                       WORKSPACE_ACTION_SCALE, WORKSPACE_MIN_BASE_HEIGHT)


# Training samples these cases; evaluation cycles through them for comparison.
EVALUATION_GOALS = BODY_POSE_GOALS
EVALUATION_SECONDS = 30.0


class TrainingWindow(QtWidgets.QWidget):
    def __init__(self, directory, child=None, render=True):
        super().__init__()
        self.directory, self.child, self.render_enabled = Path(directory), child, render
        self.env = self.policy = self.renderer = None
        self.pointer = None
        self.failed = False
        self.ticks = self.loaded_checkpoints = 0
        self.rows = []
        self.goal_index = -1
        self.last_rollout_step = None
        self.next_rollout_poll = 0
        self.loop_elapsed = 0.0
        self.loop_count = 0
        self.loop_resets = 0
        self.evaluation_goal_name = ""
        self.evaluation_goals = EVALUATION_GOALS
        self.goal_scale = 1.0
        self.last_reset = ""
        self.setWindowTitle("QuadBot — PPO training")
        self.resize(1280, 900)
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel("Starting trainer…")
        layout.addWidget(self.status)
        self.preview_status = QtWidgets.QLabel("Waiting for the initial policy checkpoint")
        layout.addWidget(self.preview_status)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["Live training rollout (exploration)", "Latest policy evaluation (real time, 30 s loops)"])
        self.mode.setCurrentIndex(1)
        layout.addWidget(self.mode)
        self.action_mode = QtWidgets.QComboBox()
        self.action_mode.addItems([
            "Sample policy actions (with exploration, like training)",
            "Mean policy actions (deterministic, no exploration)",
        ])
        layout.addWidget(self.action_mode)
        self.policy_limits = QtWidgets.QLabel()
        self.policy_limits.setWordWrap(True)
        layout.addWidget(self.policy_limits)
        self.learning_status = QtWidgets.QLabel("Waiting for a measured policy evaluation…")
        self.learning_status.setWordWrap(True)
        layout.addWidget(self.learning_status)
        self.goal_status = QtWidgets.QLabel("Waiting for the trainer's current goal")
        layout.addWidget(self.goal_status)
        split = QtWidgets.QHBoxLayout()
        layout.addLayout(split, 1)
        left = QtWidgets.QVBoxLayout()
        split.addLayout(left, 1)
        self.image = QtWidgets.QLabel("Loading policy…" if render else "Robot rendering disabled")
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setMinimumSize(480, 360)
        left.addWidget(self.image, 1)
        self.evaluation = QtWidgets.QLabel("Evaluation is independent of training.")
        self.evaluation.setWordWrap(True)
        left.addWidget(self.evaluation)
        command_row = QtWidgets.QHBoxLayout()
        self.command_inputs = []
        for name, unit, bounds, value in zip(("X", "Y", "Z", "Roll", "Pitch", "Yaw"),
                                           ("mm",)*3 + ("°",)*3,
                                           ((-60, 60), (-30, 30), (-150, 10),
                                            (-10, 10), (-13, 13), (-13, 13)),
                                           (0, 0, -100, 0, 0, 0)):
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(*bounds)
            box.setDecimals(1)
            box.setSingleStep(.5)
            box.setPrefix(name + " ")
            box.setSuffix(" " + unit)
            box.setValue(value)
            self.command_inputs.append(box)
            command_row.addWidget(box)
        left.addLayout(command_row)
        self.cycle_goals = QtWidgets.QCheckBox("Change goal every 4 seconds within each 30-second loop")
        self.cycle_goals.setChecked(True)
        left.addWidget(self.cycle_goals)
        self.restart_button = QtWidgets.QPushButton("Restart evaluation with this command")
        self.restart_button.clicked.connect(self.manual_goal)
        left.addWidget(self.restart_button)
        self.stop = QtWidgets.QPushButton("Stop training and save")
        self.stop.setEnabled(child is not None)
        self.stop.clicked.connect(self.stop_training)
        left.addWidget(self.stop)
        note = QtWidgets.QLabel("Green box: requested chassis pose. Green spheres: fixed foot anchors.\n"
                               "Live mode shows actual sampled training actions, downsampled to 10 Hz.\n"
                               "Sampled playback includes exploration; movement alone does not prove tracking.\n"
                               "Playback runs at real-time pace; newer policies load between 30-second loops.")
        note.setWordWrap(True)
        left.addWidget(note)
        graphs = QtWidgets.QVBoxLayout()
        split.addLayout(graphs, 1)
        self.curves = {}
        for key, title, unit in (("r", "Training episode return", "return"),
                                 ("position_error_mm", "Body position error", "mm"),
                                 ("orientation_error_deg", "Body orientation error", "degrees"),
                                 ("foot_drift_mm", "Maximum foot displacement", "mm")):
            plot = pg.PlotWidget(title=title + " — last 50 completed episodes")
            plot.setLabel("bottom", "Training transitions")
            plot.setLabel("left", unit)
            plot.showGrid(x=True, y=True, alpha=.2)
            self.curves[key] = plot.plot(pen=pg.mkPen("#41b6c4", width=2))
            graphs.addWidget(plot)
        plot = pg.PlotWidget(title="Deterministic tracking gain over nominal hold — current curriculum")
        plot.setLabel("bottom", "Training transitions")
        plot.setLabel("left", "Tracking gain", "%")
        plot.showGrid(x=True, y=True, alpha=.2)
        self.evaluation_curve = plot.plot(pen=pg.mkPen("#e3b341", width=2))
        graphs.addWidget(plot)
        location = QtWidgets.QLabel(f"Run: {directory}")
        location.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(location)
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.setInterval(500)
        self.poll_timer.timeout.connect(self.poll)
        self.poll_timer.start()
        self.step_timer = QtCore.QTimer(self)
        self.step_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.step_timer.setInterval(20)
        self.step_timer.timeout.connect(self.tick)
        self.step_timer.start()
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.action_mode.currentIndexChanged.connect(self.action_mode_changed)
        self.mode_changed()

    def action_mode_changed(self, *_):
        if self.mode.currentIndex() == 1:
            self.restart()

    def mode_changed(self, *_):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.env is not None:
            self.env.close()
            self.env = None
        self.pointer = None
        evaluating = self.mode.currentIndex() == 1
        self.action_mode.setEnabled(evaluating)
        self.cycle_goals.setEnabled(evaluating)
        self.restart_button.setEnabled(evaluating)
        for box in self.command_inputs:
            box.setEnabled(evaluating)
        if evaluating:
            self.restart()
        else:
            self.last_rollout_step = None

    def manual_goal(self):
        self.cycle_goals.setChecked(False)
        self.restart()

    def evaluation_command(self):
        if self.cycle_goals.isChecked():
            index = (self.goal_index + int(self.loop_elapsed / 4)) % len(self.evaluation_goals)
            self.evaluation_goal_name, command = self.evaluation_goals[index]
            self.evaluation_goal_name += f" ({self.goal_scale:.0%} curriculum)"
            return np.array(command) * self.goal_scale
        self.evaluation_goal_name = "Manual evaluation"
        return self.command()

    def update_preview_status(self):
        actions = "sampled with exploration" if self.action_mode.currentIndex() == 0 else "deterministic mean"
        self.preview_status.setText(
            f"Evaluation checkpoint: {self.pointer['steps']:,} training steps, "
            f"{self.pointer['updates']} PPO epochs. Loop {self.loop_count}: "
            f"{self.loop_elapsed:.1f}/{EVALUATION_SECONDS:.0f}s; early resets: {self.loop_resets}. "
            f"Actions: {actions}. Newer policies load at the next loop. "
            + (f"Last reset: {self.last_reset}." if self.last_reset else ""))

    def describe_goal(self, name, command):
        xyz = ", ".join(f"{v*1000:+.1f}" for v in command[:3])
        rpy = ", ".join(f"{np.degrees(v):+.1f}" for v in command[3:])
        self.goal_status.setText(f"Goal: {name}   |   XYZ [{xyz}] mm   |   roll/pitch/yaw [{rpy}]°")

    def command(self):
        values = np.array([box.value() for box in self.command_inputs])
        values[:3] /= 1000
        values[3:] = np.radians(values[3:])
        return values

    def poll(self):
        try:
            state = {}
            status_path = self.directory / "status.json"
            if status_path.exists():
                state = json.loads(status_path.read_text())
                self.status.setText(f"Training: {state['state']}   |   steps: {state.get('steps', 0):,}   |   "
                                    f"PPO epochs: {state.get('updates', 0)}   |   "
                                    f"episodes this session: {state.get('episodes', 0)}   |   "
                                    f"curriculum: {state.get('goal_scale', 1):.0%}" +
                                    f" | CPU physics ×{state.get('compute', {}).get('num_envs', 1)}; CPU PPO" +
                                    (f"   |   {state['error']}" if "error" in state else ""))
                if state["state"] == "failed":
                    self.failed = True
            if self.child is not None and self.child.poll() is not None:
                self.stop.setEnabled(False)
                if self.child.returncode:
                    self.stop.setText("Trainer failed — check log")
                elif state.get("state") in ("stopped", "complete"):
                    self.stop.setText("Stopped — checkpoint saved" if state["state"] == "stopped"
                                      else "Complete — checkpoint saved")
                    self.stop.setToolTip(f"Saved run: {self.directory}\nResume with --resume={self.directory}")
                else:
                    self.stop.setText("Trainer exited — checking saved status")
                if self.child.returncode:
                    self.failed = True
                    self.status.setText(f"Trainer failed (exit {self.child.returncode}); see {self.directory}.log")
            metrics = self.directory / "metrics.jsonl"
            if metrics.exists():
                # A concurrently appended final line may be incomplete.
                lines = metrics.read_text().splitlines(keepends=True)
                rows = [json.loads(line) for line in lines if line.endswith("\n")]
                if len(rows) != len(self.rows):
                    self.rows = rows
                    for key, curve in self.curves.items():
                        selected = [row for row in rows if row.get(key) is not None]
                        curve.setData([row["steps"] for row in selected], [row[key] for row in selected])
            evaluations = self.directory / "evaluations.jsonl"
            if evaluations.exists():
                records = [json.loads(line) for line in evaluations.read_text().splitlines(keepends=True)
                           if line.endswith("\n")]
                if records:
                    self.evaluation_curve.setData([r["steps"] for r in records],
                                                  [r["tracking_gain_pct"] for r in records])
                    latest = records[-1]
                    self.learning_status.setText(
                        f"Measured at {latest['steps']:,} steps, {latest['goal_scale']:.0%} goals: "
                        f"tracking gain {latest['tracking_gain_pct']:+.1f}% versus holding still; "
                        f"success {latest['success_pct']:.0f}%; survival {latest['survival_pct']:.0f}%. "
                        "Goal size increases only after two evaluations with ≥75% success.")
            elif self.pointer is not None:
                self.learning_status.setText("This run has no recorded tracking evaluations yet.")
            if (self.mode.currentIndex() == 1 and self.env is None
                    and (self.directory / "latest.json").exists() and not self.failed):
                self.restart()
        except Exception as error:
            self.fail(error)

    def fail(self, error):
        self.failed = True
        self.step_timer.stop()
        self.preview_status.setText(f"Preview error: {error}. Training can be stopped below.")

    def restart(self):
        if not (self.directory / "latest.json").exists():
            return
        try:
            pointer = json.loads((self.directory / "latest.json").read_text())
            if pointer != self.pointer:
                self.policy, config, self.pointer = load_policy(self.directory)
                self.loaded_checkpoints += 1
                self.goal_scale = config.goal_scale
                wide = (config.action_scale >= WORKSPACE_ACTION_SCALE
                        and config.min_base_height <= WORKSPACE_MIN_BASE_HEIGHT)
                self.evaluation_goals = EVALUATION_GOALS if wide else POSE_BODY_POSE_GOALS
                self.policy_limits.setText("" if wide else
                    f"This checkpoint uses ±{config.action_scale:g} rotor radian targets and the older stance limits. "
                    "Automatic playback uses the smaller goals; start a new training run for deep crouches and larger poses.")
                if self.env is None:
                    self.env = make_env(replace(config, episode_seconds=EVALUATION_SECONDS),
                                        self.directory / "motor_model.yaml", self.evaluation_command)
            self.loop_elapsed = 0.0
            self.loop_count += 1
            self.loop_resets = 0
            self.last_reset = ""
            self.goal_index = (self.goal_index + 1) % len(self.evaluation_goals)
            self.observation, _ = self.env.reset(seed=12345)
            self.describe_goal(self.evaluation_goal_name, self.env.unwrapped.command)
            self.update_preview_status()
        except Exception as error:
            self.fail(error)

    def tick(self):
        if self.failed:
            return
        try:
            if self.mode.currentIndex() == 0:
                self.show_training_rollout()
                return
            if self.env is None:
                return
            action, _ = self.policy.predict(
                self.observation, deterministic=self.action_mode.currentIndex() == 1)
            self.loop_elapsed += self.env.unwrapped.dt
            self.observation, _, terminated, truncated, info = self.env.step(action)
            self.ticks += 1
            if self.ticks % 2 == 0:
                self.update_preview_status()
                self.describe_goal(self.evaluation_goal_name, self.env.unwrapped.command)
                self.evaluation.setText(
                    f"Evaluation t={info['time']:.2f}s   |   body error: {info['position_error_m']*1000:.1f} mm, "
                    f"{np.degrees(info['orientation_error_rad']):.2f}°\n"
                    f"Foot drift: {max(info['foot_displacement_m'])*1000:.1f} mm   |   "
                    f"contacts: {int(sum(info['foot_contacts']))}/4   |   "
                    f"temperature: {info['max_temperature_c']:.1f} °C\n"
                    f"Applied motor target offset (max): "
                    f"{np.max(np.abs(self.env.unwrapped.previous_action))*self.env.unwrapped.action_scale:.3f} rad")
                if self.render_enabled:
                    self.render_frame()
            if self.loop_elapsed >= EVALUATION_SECONDS - 1e-9:
                self.restart()
            elif terminated or truncated:
                # Preserve this loop's policy even when it falls. Resets may
                # interrupt motion, but cannot trigger premature checkpoint swaps.
                self.loop_resets += 1
                self.last_reset = f"{info['termination_reason'] or 'time_limit'} at {info['time']:.2f}s"
                self.observation, _ = self.env.reset(seed=12345)
        except Exception as error:
            self.fail(error)

    def show_training_rollout(self):
        if time.monotonic() < self.next_rollout_poll:
            return
        self.next_rollout_poll = time.monotonic() + .1
        path = self.directory / "rollout.json"
        if not path.exists():
            self.preview_status.setText("Waiting for live training frames; use evaluation for older saved runs.")
            return
        sample = json.loads(path.read_text())
        if sample["steps"] == self.last_rollout_step:
            return
        if self.env is None:
            from sim.training import read_manifest
            config = read_manifest(self.directory)
            self.env = make_env(config, self.directory / "motor_model.yaml")
        base = self.env.unwrapped
        base.data.qpos[:] = sample["qpos"]
        base.data.qvel[:] = sample["qvel"]
        base.data.time = sample["time"]
        mujoco.mj_forward(base.model, base.data)
        base.target_position = np.array(sample["target_position"])
        base.target_quaternion = np.array(sample["target_quaternion"])
        base.foot_anchors = np.array(sample["foot_anchors"])
        self.last_rollout_step = sample["steps"]
        self.ticks += 1
        self.preview_status.setText(f"Actual training rollout: step {sample['steps']:,}, "
                                    f"episode {sample['episode'] + 1}. Accelerated physics; sampled frames.")
        name = next((name for name, command in (*BODY_POSE_GOALS, *POSE_BODY_POSE_GOALS, *AXIS_BODY_POSE_GOALS)
                     if np.allclose(command, sample["command"])), "Mixed pose")
        self.describe_goal(f"Training episode {sample['episode'] + 1}: {name}", sample["command"])
        self.evaluation.setText(
            f"Episode t={sample['time']:.2f}s   |   body error: {sample['position_error_m']*1000:.1f} mm, "
            f"{np.degrees(sample['orientation_error_rad']):.2f}°\n"
            f"Foot drift: {max(sample['foot_displacement_m'])*1000:.1f} mm   |   "
            f"contacts: {sum(sample['foot_contacts'])}/4   |   temperature: {sample['max_temperature_c']:.1f} °C")
        if self.render_enabled:
            self.render_frame()

    def render_frame(self):
        base = self.env.unwrapped
        if self.renderer is None:
            self.renderer = mujoco.Renderer(base.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = .9, 135, -25
        camera.lookat[:] = [0, 0, .15]
        self.renderer.update_scene(base.data, camera=camera)
        scene = self.renderer.scene
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, base.target_quaternion)
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_BOX,
                           [.19, .045, .03], base.target_position, rotation, [.1, .9, .4, .2])
        scene.ngeom += 1
        for anchor in base.foot_anchors:
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                               [.022, 0, 0], anchor, np.eye(3).ravel(), [.1, .9, .4, .3])
            scene.ngeom += 1
        pixels = self.renderer.render()
        image = QtGui.QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0],
                             QtGui.QImage.Format_RGB888).copy()
        self.image.setPixmap(QtGui.QPixmap.fromImage(image).scaled(
            self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def stop_training(self):
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            self.stop.setEnabled(False)
            self.stop.setText("Saving checkpoint…")

    def closeEvent(self, event):
        self.poll_timer.stop()
        self.step_timer.stop()
        self.stop_training()
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.env is not None:
            self.env.close()
        event.accept()


def run_window(directory, args, window_factory=TrainingWindow, worker_name="train_worker", worker_options=None):
    torch.set_num_threads(1)
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setStyle("Fusion")
    child = log = None
    if not args.watch:
        directory.parent.mkdir(parents=True, exist_ok=True)
        log = Path(str(directory) + ".log").open("a")
        rf = runfiles.Create()
        command = [rf.Rlocation(f"_main/sim/{worker_name}"), "--headless", f"--steps={args.steps}"]
        if args.resume:
            command += [f"--resume={directory}"]
        elif worker_options is not None:
            command += [f"--run-dir={directory}", *worker_options]
        else:
            command += [f"--run-dir={directory}", f"--seed={args.seed}",
                        f"--num-envs={args.num_envs}", f"--rollout-steps={args.rollout_steps}", f"--episode-seconds={args.episode_seconds}",
                        f"--evaluation-interval={args.evaluation_interval}", f"--profile={args.profile}"]
            if args.from_scratch:
                command += ["--from-scratch"]
        child = subprocess.Popen(command, env=dict(os.environ, **rf.EnvVars()),
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    window = window_factory(directory, child, render=not args.no_render)
    try:
        with StopRequest() as stop:
            timer = QtCore.QTimer()
            timer.setInterval(100)
            timer.timeout.connect(lambda: window.close() if stop.requested else None)
            timer.start()
            if args.exit_after:
                QtCore.QTimer.singleShot(round(1000*args.exit_after), window.close)
            if args.screenshot_output:
                destination = args.screenshot_output.resolve()
                destination.parent.mkdir(parents=True, exist_ok=True)
                QtCore.QTimer.singleShot(round(args.capture_at*1000), lambda: window.grab().save(str(destination)))
            window.show()
            app.exec()
            timer.stop()
    finally:
        window.close()
        if child is not None:
            # Signal handler requests a checkpoint; allow time before force-stop.
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                stop_child(child)
        if log is not None:
            log.close()
    return 1 if window.failed or (child is not None and child.returncode) else 0
