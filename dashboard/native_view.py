"""Native plots and persistent OpenGL geometry for decoded robot telemetry."""
import time

import mujoco
import numpy as np
from PySide6 import QtCore, QtGui, QtNetwork, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from OpenGL import GL

from dashboard.native_client import FIELDS, History
from dashboard.telemetry import MOTORS, LEGS
from dashboard.scene import SensedScene
from sim.robot_model import LEGS as LEG_PARAMETERS


PLOTS = {"voltage_command_v": ("Voltage command", "V"), "rotor_rad": ("Encoder angle", "rad"),
         "temperature_c": ("Temperature", "°C"), "encoder_count": ("Encoder count", "counts"),
         "velocity_rad_s": ("Rotor velocity", "rad/s")}
COLORS = ("#55aaff", "#ff9955", "#ffe066", "#55ddaa", "#66ddff", "#bb99ff",
          "#ccdd55", "#ee77aa", "#ff6655", "#88bbee", "#bbbbbb", "#bb8877")


def box_mesh():
    vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return gl.MeshData(vertexes=vertices, faces=faces)


class RobotView(gl.GLViewWidget):
    def __init__(self, scene):
        super().__init__()
        self.scene = scene
        self.setBackgroundColor("#151a22")
        self.setCameraPosition(pos=QtGui.QVector3D(0, 0, .26), distance=.9, elevation=22, azimuth=135)
        grid = gl.GLGridItem()
        grid.setSize(.8, .8)
        grid.setSpacing(.1, .1)
        self.addItem(grid)
        self.items_by_geom = []
        self.valid = False
        self.paints = 0
        self.paint_times = []
        self.renderer = None
        meshes = {"box": box_mesh(), "sphere": gl.MeshData.sphere(rows=8, cols=12),
                  "cylinder": gl.MeshData.cylinder(rows=1, cols=16, radius=[1, 1], length=2)}
        model = scene.model
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            if "_collision_" not in name or "_self_collision_" in name:
                continue
            size = model.geom_size[i]
            kind = int(model.geom_type[i])
            shapes = []
            if kind == mujoco.mjtGeom.mjGEOM_BOX:
                shapes = [("box", size, np.zeros(3))]
            elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                shapes = [("sphere", [size[0]]*3, np.zeros(3))]
            elif kind in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
                shapes = [("cylinder", [size[0], size[0], size[1]], np.array([0, 0, -size[1]]))]
                if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                    shapes += [("sphere", [size[0]]*3, np.array([0, 0, sign*size[1]])) for sign in (-1, 1)]
            leg = next((leg for leg, params in LEG_PARAMETERS.items() if f"_{params['cad']}_" in name), None)
            for shape, scale, offset in shapes:
                item = gl.GLMeshItem(meshdata=meshes[shape], smooth=False, shader="shaded",
                                     color=(.35, .65, .85, 1))
                item.setVisible(False)
                self.addItem(item)
                self.items_by_geom.append((i, item, np.asarray(scale), offset, leg))

    def show_state(self, state, valid):
        self.valid = valid
        for i, item, scale, offset, leg in self.items_by_geom:
            item.setVisible(valid)
            if not valid:
                continue
            rotation = self.scene.data.geom_xmat[i].reshape(3, 3)
            matrix = np.eye(4)
            matrix[:3, :3] = rotation @ np.diag(scale)
            matrix[:3, 3] = self.scene.data.geom_xpos[i] + rotation @ offset
            item.setTransform(QtGui.QMatrix4x4(*matrix.ravel().tolist()))
            if leg:
                heat = max(state["motors"][f"motor_{leg}{axis}"]["temperature_c"] for axis in "IOZ")
                fraction = max(0, min(1, (heat-25)/30))
                item.setColor((.2+.8*fraction, .8*(1-fraction), .5*(1-fraction), 1))
        self.update()

    def paintGL(self, *args, **kwargs):
        start = time.perf_counter()
        if self.renderer is None:
            self.renderer = GL.glGetString(GL.GL_RENDERER).decode()
        super().paintGL(*args, **kwargs)
        self.paints += 1
        self.paint_times.append(time.perf_counter()-start)
        self.paint_times = self.paint_times[-3000:]


class InspectorWindow(QtWidgets.QMainWindow):
    def __init__(self, client, reference=None, startup_stand=False, enable_3d=True):
        super().__init__()
        self.setWindowTitle("QuadBot inspector — native")
        self.resize(1440, 1000)
        self.client = client
        self.history = History()
        self.scene = SensedScene(reference, startup_stand=startup_stand)
        self.latest = None
        self.samples = self.refreshes = 0
        self.refresh_times = []
        self.received_hold = False
        self.next_table = 0
        self.next_axes = 0
        self.follow = QtWidgets.QCheckBox("Follow live (15 s)")
        self.follow.setChecked(True)
        self.status = QtWidgets.QLabel("Connecting…")
        self.pose_status = QtWidgets.QLabel("Waiting for referenced encoder feedback")
        self.pose_status.setWordWrap(True)
        self.imu_status = QtWidgets.QLabel("IMU unavailable")
        self.imu_status.setWordWrap(True)
        self.ack = QtWidgets.QLabel("Commands are acknowledged by the controller.")
        self.ack.setWordWrap(True)
        self.robot_view = RobotView(self.scene) if enable_3d else None

        left = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(left)
        column.addWidget(self.status)
        if self.robot_view:
            column.addWidget(self.robot_view, 3)
        column.addWidget(self.pose_status)
        column.addWidget(self.imu_status)
        self.table = QtWidgets.QTableWidget(len(MOTORS), 6)
        self.table.setHorizontalHeaderLabels(["Motor / plot", "Encoder", "Angle (rad)", "Voltage (V)", "Temp (°C)", "Age (ms)"])
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        for i, name in enumerate(MOTORS):
            for j in range(6):
                item = QtWidgets.QTableWidgetItem(name.removeprefix("motor_") if j == 0 else "—")
                if j == 0:
                    item.setCheckState(QtCore.Qt.Checked)
                    item.setForeground(QtGui.QColor(COLORS[i]))
                self.table.setItem(i, j, item)
        column.addWidget(self.table, 2)
        row = QtWidgets.QHBoxLayout()
        self.stop = QtWidgets.QPushButton("STOP")
        self.stop.setStyleSheet("background:#b52929;color:white;font-weight:bold;padding:9px")
        self.hold = QtWidgets.QPushButton("Hold current pose")
        self.hold.setToolTip("Stop the scripted control first, then capture the current pose.")
        self.stop.clicked.connect(lambda: client.send("stop"))
        self.hold.clicked.connect(lambda: client.send("hold"))
        row.addWidget(self.stop)
        row.addWidget(self.hold)
        column.addLayout(row)
        target = QtWidgets.QHBoxLayout()
        self.leg = QtWidgets.QComboBox()
        self.leg.addItems(LEGS)
        self.leg.currentTextChanged.connect(self.capture_target)
        target.addWidget(self.leg)
        self.targets = {}
        for axis in ("theta", "gamma", "z"):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(4)
            spin.setSingleStep(.01)
            spin.setRange(-1000, 1000)
            spin.setPrefix(f"{axis} ")
            spin.setSuffix(" rad")
            self.targets[axis] = spin
            target.addWidget(spin)
        column.addLayout(target)
        self.send_target = QtWidgets.QPushButton("Send leg target")
        self.send_target.clicked.connect(self.command_target)
        column.addWidget(self.send_target)
        column.addWidget(self.ack)

        right = QtWidgets.QWidget()
        plots_column = QtWidgets.QVBoxLayout(right)
        plots_column.addWidget(self.follow)
        self.plots = []
        pg.setConfigOptions(antialias=False, useOpenGL=enable_3d,
                            background="#151a22", foreground="#d5dce7")
        for field in FIELDS[:3]:
            selector = QtWidgets.QComboBox()
            for key, (label, unit) in PLOTS.items():
                selector.addItem(f"{label} ({unit})", key)
            selector.setCurrentIndex(list(PLOTS).index(field))
            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=.15)
            plot.setLabel("bottom", "Controller time", units="s")
            # Avoid scanning all curves and relaying out axes after every
            # individual setData(). Refresh shared bounds at the table rate.
            plot.disableAutoRange()
            plot.setLabel("left", *PLOTS[field])
            selector.currentIndexChanged.connect(lambda _, s=selector, p=plot: p.setLabel("left", *PLOTS[s.currentData()]))
            plot.addLegend(offset=(8, 8), colCount=6)
            curves = [plot.plot(pen=pg.mkPen(color, width=1), name=name.removeprefix("motor_"))
                      for name, color in zip(MOTORS, COLORS)]
            for curve in curves:
                curve.setClipToView(True)
                curve.setDownsampling(auto=True, method="peak")
            plots_column.addWidget(selector)
            plots_column.addWidget(plot, 1)
            self.plots.append((selector, plot, curves))
        splitter = QtWidgets.QSplitter()
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([650, 790])
        self.setCentralWidget(splitter)
        client.state_received.connect(self.receive_state)
        client.ack_received.connect(self.receive_ack)
        client.status_changed.connect(self.status.setText)
        client.connection_lost.connect(self.disconnected)
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        for button in (self.stop, self.hold, self.send_target):
            button.setEnabled(False)

    def receive_state(self, state):
        self.latest = state
        self.history.append(state)
        self.samples += 1
        # Observe every sample so brief stale/reset events cannot be skipped
        # between display frames. This performs kinematics, never physics.
        self.scene.update_pose(state)
        if state["remote_hold"] and not self.received_hold:
            self.capture_target()
        self.received_hold = state["remote_hold"]

    def disconnected(self):
        self.scene.invalidated = True
        if self.robot_view:
            self.robot_view.show_state(self.latest, False)

    def receive_ack(self, message):
        result = "Accepted" if message["accepted"] else "Rejected"
        self.ack.setText(f"{result}: {message.get('command', '')} — {message['message']}")

    def capture_target(self, *_):
        if self.latest:
            pose = self.latest["legs"][f"leg_{self.leg.currentText()}"]
            for axis, spin in self.targets.items():
                spin.setValue(pose[axis])

    def command_target(self):
        self.client.send("leg_target", leg=f"leg_{self.leg.currentText()}",
                         **{axis: spin.value() for axis, spin in self.targets.items()})

    def refresh(self):
        start = time.perf_counter()
        fresh = self.client.fresh()
        state = self.latest
        self.stop.setEnabled(self.client.socket.state() == QtNetwork.QAbstractSocket.ConnectedState)
        self.hold.setEnabled(bool(fresh and self.client.allow_motion and state["ready"] and state["motion_enabled"] and not state["pose_fault"] and
                                  all(leg["mode"] == "stopped" for leg in state["legs"].values())))
        self.send_target.setEnabled(bool(fresh and self.client.allow_motion and self.client.owns_hold and state["remote_hold"]))
        if state is None:
            return
        if not fresh:
            self.imu_status.setText("IMU unavailable — telemetry stale")
            self.scene.invalidated = True
            self.pose_status.setText("Telemetry stale/disconnected — pose hidden; restart to re-reference")
            if self.robot_view:
                self.robot_view.show_state(state, False)
            return
        if self.robot_view:
            self.robot_view.show_state(state, self.scene.was_ready and self.scene.reference is not None and not self.scene.invalidated)
        self.pose_status.setText(self.scene.status)
        imu = state.get("imu")
        if imu and imu["valid"]:
            gyro = ", ".join(f"{imu['angular_velocity_rad_s'][axis]:+.3f}" for axis in "xyz")
            accel = ", ".join(f"{imu['specific_force_m_s2'][axis]:+.3f}" for axis in "xyz")
            self.imu_status.setText(f"IMU (body XYZ): gyro [{gyro}] rad/s\nSpecific force [{accel}] m/s²")
        else:
            self.imu_status.setText("IMU unavailable" if not imu else "IMU waiting for startup support release")
        times, values = self.history.arrays()
        now = time.monotonic()
        update_axes = now >= self.next_axes
        if update_axes:
            self.next_axes = now + .2
        visible_motors = [self.table.item(i, 0).checkState() == QtCore.Qt.Checked for i in range(len(MOTORS))]
        for selector, plot, curves in self.plots:
            field = selector.currentData()
            index = FIELDS.index(field)
            for i, curve in enumerate(curves):
                visible = visible_motors[i]
                curve.setVisible(visible)
                if visible:
                    curve.setData(times, values[:, i, index])
            if self.follow.isChecked() and len(times):
                plot.setXRange(max(0, times[-1]-15), max(15, times[-1]), padding=0)
                if update_axes and any(visible_motors):
                    shown = values[times >= times[-1]-15][:, visible_motors, index]
                    finite = shown[np.isfinite(shown)]
                    if finite.size:
                        low, high = float(finite.min()), float(finite.max())
                        margin = max((high-low)*.08, .01 if field != "temperature_c" else .5)
                        plot.setYRange(low-margin, high+margin, padding=0)
        if now >= self.next_table:
            self.next_table = now + .2
            self.status.setText(f"{state['backend']}  •  {state['time']:.2f} s  •  " +
                                (f"FAULT: {state['pose_fault']}" if state['pose_fault'] else
                                 state['dashboard_reason'] or ("Ready" if state['ready'] else "Initializing boards")))
            stages = [f"{name.removeprefix('leg_')}: {leg['init_stage']}"
                      for name, leg in state["legs"].items() if "init_stage" in leg]
            if stages:
                self.status.setText(self.status.text() + "  •  " + " | ".join(stages))
            for i, name in enumerate(MOTORS):
                m = state["motors"][name]
                for j, text in enumerate((str(m['encoder_count']), f"{m['rotor_rad']:.4f}", f"{m['voltage_command_v']:.3f}",
                                          f"{m['temperature_c']:.0f}", f"{m['encoder_age_ms']:.1f}"), 1):
                    if self.table.item(i, j).text() != text:
                        self.table.item(i, j).setText(text)
        self.refreshes += 1
        self.refresh_times.append(time.perf_counter()-start)
        self.refresh_times = self.refresh_times[-3000:]

    def closeEvent(self, event):
        self.timer.stop()
        self.client.close()
        super().closeEvent(event)

    def save_screenshot(self, path):
        # QWidget.grab() can omit QGraphicsView's OpenGL viewport on Wayland.
        # Render the plot scenes into the export using a raster QPainter.
        capture = self.grab()
        painter = QtGui.QPainter(capture)
        for _, plot, _ in self.plots:
            viewport = plot.viewport()
            rectangle = QtCore.QRectF(viewport.mapTo(self, QtCore.QPoint()), viewport.size())
            painter.fillRect(rectangle, QtGui.QColor("#151a22"))
            plot.render(painter, rectangle, viewport.rect())
        painter.end()
        return capture.save(path)
