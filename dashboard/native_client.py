"""Qt client for the shared loopback TCP JSON-lines controller endpoint."""
import json
import time

import numpy as np
from PySide6 import QtCore, QtNetwork

from dashboard.telemetry import MOTORS


FIELDS = ("voltage_command_v", "rotor_rad", "temperature_c", "encoder_count", "velocity_rad_s")


class History:
    """Bounded sample storage, independent of the display refresh cadence."""
    def __init__(self, capacity=3000):
        self.capacity = capacity
        self.times = np.empty(capacity)
        self.values = np.full((capacity, len(MOTORS), len(FIELDS)), np.nan)
        self.count = self.cursor = 0

    def append(self, state):
        self.times[self.cursor] = state["time"]
        for i, name in enumerate(MOTORS):
            motor = state["motors"][name]
            self.values[self.cursor, i] = [motor.get(field, np.nan) for field in FIELDS]
        self.cursor = (self.cursor + 1) % self.capacity
        self.count = min(self.count + 1, self.capacity)

    def arrays(self):
        if self.count < self.capacity:
            return self.times[:self.count], self.values[:self.count]
        return (np.concatenate((self.times[self.cursor:], self.times[:self.cursor])),
                np.concatenate((self.values[self.cursor:], self.values[:self.cursor])))


class ControllerClient(QtCore.QObject):
    state_received = QtCore.Signal(dict)
    ack_received = QtCore.Signal(dict)
    status_changed = QtCore.Signal(str)
    connection_lost = QtCore.Signal()

    def __init__(self, port, parent=None, allow_motion=True):
        super().__init__(parent)
        self.port = port
        self.allow_motion = allow_motion
        self.socket = QtNetwork.QTcpSocket(self)
        self.socket.setReadBufferSize(262144)
        self.socket.connected.connect(self._connected)
        self.socket.readyRead.connect(self._read)
        self.socket.disconnected.connect(self._disconnected)
        self.socket.errorOccurred.connect(self._error)
        self.retry = QtCore.QTimer(self)
        self.retry.setInterval(500)
        self.retry.timeout.connect(self._connect)
        self.heartbeat = QtCore.QTimer(self)
        self.heartbeat.setInterval(100)
        self.heartbeat.timeout.connect(self._heartbeat)
        self.buffer = bytearray()
        self.seq = 0
        self.pending = {}
        self.latest = None
        self.last_received = 0
        self.ever_connected = self.closed = self.owns_hold = False

    def start(self):
        self.retry.start()
        self.heartbeat.start()
        self._connect()

    def _connect(self):
        if not self.closed and self.socket.state() == QtNetwork.QAbstractSocket.UnconnectedState:
            self.socket.connectToHost("127.0.0.1", self.port)

    def _connected(self):
        self.ever_connected = True
        self.retry.stop()
        self.status_changed.emit("Connected; waiting for telemetry")

    def _error(self, _):
        self.status_changed.emit(self.socket.errorString())

    def _disconnected(self):
        self.owns_hold = False
        self.pending.clear()
        self.buffer.clear()
        self.connection_lost.emit()
        self.status_changed.emit("Disconnected — restart inspector to establish a new pose reference")

    def fresh(self):
        return (self.socket.state() == QtNetwork.QAbstractSocket.ConnectedState and
                self.latest is not None and time.monotonic() - self.last_received < 0.2)

    def send(self, operation, **fields):
        if not self.allow_motion and operation not in ("stop", "heartbeat"):
            self.status_changed.emit("Motion commands disabled in this inspector")
            return False
        if self.closed or self.socket.state() != QtNetwork.QAbstractSocket.ConnectedState:
            self.status_changed.emit("Command not sent: controller disconnected")
            return False
        if self.socket.bytesToWrite() > 8192 or (operation != "stop" and len(self.pending) >= 16):
            self.status_changed.emit("Command queue full; disconnecting")
            self.socket.abort()  # The host stops inspector-owned control on disconnect.
            return False
        self.seq += 1
        message = {**fields, "seq": self.seq, "command": operation}
        data = (json.dumps(message, allow_nan=False, separators=(",", ":")) + "\n").encode()
        if self.socket.write(data) < 0:
            return False
        if operation != "heartbeat":
            self.pending[self.seq] = operation
        if operation == "stop":
            self.owns_hold = False
        return True

    def _heartbeat(self):
        # Runs on the GUI thread: a stalled UI cannot renew its control lease.
        if self.owns_hold and self.fresh():
            self.send("heartbeat")

    def _read(self):
        self.buffer.extend(bytes(self.socket.readAll()))
        if len(self.buffer) > 262144:
            self.status_changed.emit("Telemetry buffer exceeded; disconnecting")
            self.socket.abort()
            return
        try:
            while b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                message = json.loads(line)
                if message["type"] == "state":
                    previous = self.latest
                    for name, motor in message["motors"].items():
                        if (previous is not None and message["time"] > previous["time"] and
                                0 <= motor["encoder_age_ms"] < 100 and
                                0 <= previous["motors"][name]["encoder_age_ms"] < 100):
                            motor["velocity_rad_s"] = (motor["rotor_rad"] - previous["motors"][name]["rotor_rad"]) / (message["time"] - previous["time"])
                    self.latest = message
                    self.last_received = time.monotonic()
                    if not message["remote_hold"]:
                        self.owns_hold = False
                    self.state_received.emit(message)
                elif message["type"] == "ack":
                    operation = self.pending.pop(message.get("seq"), None)
                    if operation == "hold" and message["accepted"]:
                        self.owns_hold = True
                    self.ack_received.emit(message)
        except (ValueError, KeyError, TypeError) as error:
            self.status_changed.emit(f"Invalid telemetry: {error}")
            self.socket.abort()

    def close(self):
        if self.closed:
            return
        if self.owns_hold:
            self.send("stop")
            self.socket.flush()
            if self.socket.bytesToWrite():
                self.socket.waitForBytesWritten(100)
        self.closed = True
        self.retry.stop()
        self.heartbeat.stop()
        self.socket.abort()
