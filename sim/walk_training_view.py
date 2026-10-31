"""Native walking-training window: actual rollouts and separate policy replay."""
from dataclasses import replace
import json
from pathlib import Path
import signal

import mujoco
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from sim.walk_training import WALK_GOALS, WalkPolicyPlayer, load_walk_policy, make_walk_env, read_walk_manifest


PREVIEW_SECONDS = 30.


class WalkTrainingWindow(QtWidgets.QWidget):
    def __init__(self, directory, child=None, render=True):
        super().__init__()
        self.directory, self.child, self.render_enabled = Path(directory), child, render
        self.env = self.policy = self.renderer = self.pointer = None
        self.failed = False
        self.rows = []
        self.ticks = self.loaded_checkpoints = self.loop_count = self.loop_resets = 0
        self.loop_elapsed = 0.
        self.command_scale = .5
        self.last_rollout_step = None
        self.setWindowTitle("QuadBot — walking training")
        self.resize(1280, 900)
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel("Starting walking trainer…")
        layout.addWidget(self.status)
        self.qualification_status = QtWidgets.QLabel("Qualification runs separately from the preview, at full compute speed.")
        self.qualification_status.setWordWrap(True)
        layout.addWidget(self.qualification_status)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["Policy preview (real-time playback, 30 s loops)", "Live training rollout (exploration)"])
        layout.addWidget(self.mode)
        self.action_mode = QtWidgets.QComboBox()
        self.action_mode.addItems(["Mean policy actions (deterministic)", "Sample policy actions (with exploration)"])
        layout.addWidget(self.action_mode)
        self.preview_status = QtWidgets.QLabel("Waiting for the initial walking checkpoint…")
        layout.addWidget(self.preview_status)
        self.learning_status = QtWidgets.QLabel("Waiting for cold/warm tracking and power qualification…")
        self.learning_status.setWordWrap(True)
        layout.addWidget(self.learning_status)
        split = QtWidgets.QHBoxLayout()
        layout.addLayout(split, 1)
        left = QtWidgets.QVBoxLayout()
        split.addLayout(left, 1)
        self.image = QtWidgets.QLabel("Loading robot…" if render else "Robot rendering disabled")
        self.image.setMinimumSize(480, 280)
        self.image.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        left.addWidget(self.image, 1)
        self.telemetry = QtWidgets.QLabel("Waiting for motor temperature and effort measurements…")
        self.telemetry.setWordWrap(True)
        self.telemetry.setMinimumHeight(self.telemetry.fontMetrics().lineSpacing()*5)
        left.addWidget(self.telemetry)
        command_row = QtWidgets.QHBoxLayout()
        self.command_inputs = []
        for name, bounds, value, unit in (("Forward", (-.15, .3), .1, "m/s"),
                                        ("Sideways", (-.1, .1), 0, "m/s"),
                                        ("Turn", (-.6, .6), 0, "rad/s")):
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(*bounds)
            box.setDecimals(2)
            box.setSingleStep(.02)
            box.setValue(value)
            box.setPrefix(name + " ")
            box.setSuffix(" " + unit)
            self.command_inputs.append(box)
            command_row.addWidget(box)
        left.addLayout(command_row)
        self.cycle_goals = QtWidgets.QCheckBox("Cycle walking commands every 4 seconds")
        self.cycle_goals.setChecked(True)
        left.addWidget(self.cycle_goals)
        self.restart_button = QtWidgets.QPushButton("Restart evaluation with this command")
        self.restart_button.clicked.connect(self.manual_command)
        left.addWidget(self.restart_button)
        self.stop = QtWidgets.QPushButton("Stop training and save")
        self.stop.setEnabled(child is not None)
        self.stop.clicked.connect(self.stop_training)
        left.addWidget(self.stop)
        note = QtWidgets.QLabel(
            "Evaluation uses a saved policy in a separate simulation, starting at 40 °C.\n"
            "New checkpoints load between 30-second loops. Early falls reset the same policy.\n"
            "Live training shows actual exploratory rollouts, sampled at 10 Hz.\n"
            "Low power alone is not success: the robot must follow the motion command.")
        note.setWordWrap(True)
        left.addWidget(note)
        graphs = QtWidgets.QVBoxLayout()
        split.addLayout(graphs, 1)
        self.curves = {}
        for key, title, unit in (("r", "Training return", "return"),
                                 ("mean_velocity_error_m_s", "Mean velocity error", "m/s"),
                                 ("peak_temperature_c", "Peak motor temperature", "°C"),
                                 ("mean_copper_power_w", "Mean copper heating power", "W")):
            plot = pg.PlotWidget(title=title + " — last 50 completed training episodes")
            plot.setLabel("bottom", "Training transitions")
            plot.setLabel("left", unit)
            plot.getAxis("left").enableAutoSIPrefix(False)
            plot.showGrid(x=True, y=True, alpha=.2)
            self.curves[key] = plot.plot(pen=pg.mkPen("#41b6c4", width=2), symbol="o", symbolSize=3)
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
        self.action_mode.currentIndexChanged.connect(self.restart)
        self.poll()

    def command(self):
        if self.cycle_goals.isChecked():
            # Start with forward motion; neutral standing remains in the cycle.
            index = (1 + int(self.loop_elapsed / 4)) % len(WALK_GOALS)
            return np.array(WALK_GOALS[index][1]) * self.command_scale
        return np.array([box.value() for box in self.command_inputs])

    def release_preview(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.env is not None:
            self.env.close()
            self.env = None

    def mode_changed(self, *_):
        self.release_preview()
        self.last_rollout_step = None
        evaluating = self.mode.currentIndex() == 0
        for widget in (self.action_mode, self.cycle_goals, self.restart_button, *self.command_inputs):
            widget.setEnabled(evaluating)
        if evaluating:
            self.restart()

    def manual_command(self):
        self.cycle_goals.setChecked(False)
        self.restart()

    def restart(self, *_):
        if not (self.directory / "latest.json").exists() or self.mode.currentIndex() != 0:
            return
        try:
            self.release_preview()
            pointer = json.loads((self.directory / "latest.json").read_text())
            if pointer != self.pointer:
                self.policy, self.config, self.pointer = load_walk_policy(self.directory)
                self.loaded_checkpoints += 1
            self.command_scale = self.config.command_scale
            self.loop_elapsed = 0.
            self.loop_resets = 0
            self.loop_count += 1
            self.env = make_walk_env(replace(self.config, episode_seconds=PREVIEW_SECONDS),
                                    self.directory / "motor_model.yaml", command_provider=self.command)
            self.observation, _ = self.env.reset(seed=12345, options={"initial_temperature_c": 40.})
            self.player = WalkPolicyPlayer(self.policy)
            self.draw()
        except Exception as error:
            self.fail(error)

    def poll(self):
        try:
            state = {}
            status = self.directory / "status.json"
            if status.exists():
                state = json.loads(status.read_text())
                droq = state.get("policy_kind") == "droq"
                update_label = "DroQ critic updates" if droq else "PPO epochs"
                learner = "DroQ" if droq else "PPO"
                self.status.setText(f"Walking: {state['state']} | steps: {state.get('steps', 0):,} | "
                                    f"{update_label}: {state.get('updates', 0)} | "
                                    f"command range: {state.get('command_scale', .5):.0%}" +
                                    f" | {state.get('compute', {}).get('physics_device', 'cpu').upper()} physics"
                                    f" ×{state.get('compute', {}).get('num_envs', 1)}; CPU {learner}" +
                                    f" | {state.get('policy_kind', 'mlp').upper()} policy" +
                                    (f" | {state['history_frames']} history frames" if "history_frames" in state else "") +
                                    (f" | replay: {state.get('replay_transitions', 0):,}; "
                                     f"actor updates: {state.get('actor_updates', 0):,}" if droq else "") +
                                    (f" | {state['error']}" if "error" in state else ""))
                qualification = state.get("qualification")
                if state["state"] == "evaluating" and qualification:
                    if qualification.get("physics_device") == "gpu":
                        self.qualification_status.setText(
                            f"GPU qualification: {qualification['completed']}/{qualification['total']} cases complete | "
                            f"{qualification.get('phase', 'running')} {qualification['parallel_cases']} cases in parallel | "
                            f"{qualification['simulated_seconds']:.1f}/{qualification['case_seconds']:g} simulated s | "
                            f"{qualification['elapsed_seconds']:.0f} s elapsed. Training resumes afterward.")
                    else:
                        self.qualification_status.setText(
                            f"Qualification: {qualification['completed']}/{qualification['total']} cases complete | "
                            f"{qualification['goal']} / {qualification['kind']} / "
                            f"{qualification['initial_temperature_c']:g} °C | "
                            f"case: {qualification['simulated_seconds']:.1f}/{qualification['case_seconds']:g} simulated s | "
                            f"{qualification['elapsed_seconds']:.0f} s elapsed. Full CPU speed; training resumes afterward.")
                elif state["state"] == "evaluating":
                    self.qualification_status.setText("Qualification is running at full compute speed; training resumes afterward.")
                elif state["state"] == "optimizing" and state.get("optimization"):
                    update = state["optimization"]
                    self.qualification_status.setText(
                        f"DroQ learning: {update['completed']}/{update['total']} critic updates for the latest transitions. "
                        "Stop/save can interrupt learning.")
                elif state["state"] == "saving" and droq:
                    self.qualification_status.setText("Saving DroQ weights, optimizers and replay buffer…")
                else:
                    self.qualification_status.setText("Qualification runs separately from the preview, at full compute speed.")
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
                    self.fail(f"Trainer exited with {self.child.returncode}; see {self.directory}.log")
            metrics = self.directory / "metrics.jsonl"
            if metrics.exists():
                self.rows = [json.loads(line) for line in metrics.read_text().splitlines(keepends=True)
                             if line.endswith("\n")]
                for key, curve in self.curves.items():
                    rows = [row for row in self.rows if row.get(key) is not None]
                    curve.setData([row["steps"] for row in rows], [row[key] for row in rows])
            evaluation = self.directory / "evaluation.json"
            if evaluation.exists():
                report = json.loads(evaluation.read_text())
                self.learning_status.setText(
                    f"Qualified goals: {report['success_pct']:.0f}% | survived: {report['survival_pct']:.0f}% | "
                    f"mean copper power: {report['mean_copper_power_w']:.1f} W | "
                    f"peak temperature: {report['peak_temperature_c']:.1f} °C. "
                    f"Measured at {report['steps']:,} steps, {report['command_scale']:.0%} command range.")
            if self.env is None and self.mode.currentIndex() == 0 and not self.failed:
                self.restart()
        except Exception as error:
            self.fail(error)

    def show_telemetry(self, info):
        command, velocity = info["command"], info["body_velocity_m_s"]
        self.telemetry.setText(
            f"Command: {command[0]:+.2f}, {command[1]:+.2f} m/s; turn {command[2]:+.2f} rad/s\n"
            f"Speed: {velocity[0]:+.2f}, {velocity[1]:+.2f} m/s; error {info['velocity_error_m_s']:.3f} m/s\n"
            f"Motor temperature: {info['max_temperature_c']:.1f} °C (peak {info['peak_temperature_c']:.1f})\n"
            f"Copper heating: {info['copper_power_w']:.1f} W | "
            f"positive mechanical power: {info['positive_mechanical_power_w']:.1f} W")

    def tick(self):
        if self.failed:
            return
        try:
            if self.mode.currentIndex() == 1:
                self.training_frame()
                return
            if self.env is None:
                return
            before = self.env.unwrapped.data.time
            action = self.player.predict(self.observation, deterministic=self.action_mode.currentIndex() == 0)
            self.observation, _, terminated, truncated, info = self.env.step(action)
            self.loop_elapsed += info["time"]-before
            self.ticks += 1
            self.show_telemetry(info)
            self.preview_status.setText(
                f"Preview checkpoint: {self.pointer['steps']:,} steps | loop {self.loop_count}: "
                f"{self.loop_elapsed:.1f}/{PREVIEW_SECONDS:g} s | early resets: {self.loop_resets}")
            if self.ticks % 2 == 0:
                self.draw()
            if self.loop_elapsed >= PREVIEW_SECONDS-1e-8:
                self.restart()
            elif terminated or truncated:
                self.loop_resets += 1
                self.observation, _ = self.env.reset(seed=12345, options={"initial_temperature_c": 40.})
                self.player.reset()
        except Exception as error:
            self.fail(error)

    def training_frame(self):
        path = self.directory / "rollout.json"
        if not path.exists():
            self.preview_status.setText("Waiting for live training samples…")
            return
        frame = json.loads(path.read_text())
        if frame["steps"] == self.last_rollout_step:
            return
        if self.env is None:
            config = read_walk_manifest(self.directory)
            self.env = make_walk_env(config, self.directory / "motor_model.yaml")
            self.env.reset(seed=12345)
        base = self.env.unwrapped
        base.data.qpos[:] = frame["qpos"]
        base.data.qvel[:] = frame["qvel"]
        mujoco.mj_forward(base.model, base.data)
        self.last_rollout_step = frame["steps"]
        self.ticks += 1
        self.preview_status.setText(f"Live training rollout: {frame['steps']:,} steps | episode time {frame['time']:.2f} s")
        self.show_telemetry(frame)
        self.draw()

    def draw(self):
        if not self.render_enabled or self.env is None:
            return
        base = self.env.unwrapped
        if self.renderer is None:
            self.renderer = mujoco.Renderer(base.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = .9, 135, -25
        camera.lookat[:] = base.data.body("base_link").xpos
        self.renderer.update_scene(base.data, camera=camera)
        pixels = self.renderer.render()
        image = QtGui.QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0],
                             QtGui.QImage.Format_RGB888).copy()
        self.image.setPixmap(QtGui.QPixmap.fromImage(image).scaled(
            self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def fail(self, error):
        self.failed = True
        self.preview_status.setText(str(error))
        self.step_timer.stop()

    def stop_training(self):
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            self.stop.setEnabled(False)
            self.stop.setText("Saving checkpoint…")

    def closeEvent(self, event):
        self.poll_timer.stop()
        self.step_timer.stop()
        self.stop_training()
        self.release_preview()
        event.accept()
