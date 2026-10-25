"""Real native controller + TCP integration, exclusively in simulation."""
import asyncio
import contextlib
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

import numpy as np

from dashboard.scene import SensedScene
from sim.robot_model import LEGS, active_joints, closure_errors, leg_matrix, stand_angles


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class ControllerTestCase(unittest.IsolatedAsyncioTestCase):
    scenario = "stand"
    duration = 20
    realtime = True

    async def asyncSetUp(self):
        self.root = Path(os.environ["TEST_SRCDIR"]) / "_main"
        self.port = free_port()
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen([
            str(self.root / "src/odroid_controller/controller/controller"),
            "--backend=sim", f"--scenario={self.scenario}", f"--duration={self.duration}",
            *(["--realtime"] if self.realtime else []),
            f"--dashboard-port={self.port}", "--dashboard-control"],
            env=dict(os.environ, RUNFILES_DIR=str(self.root.parent)), stdout=self.log, stderr=self.log)
        self.connections = []

    async def asyncTearDown(self):
        for writer in self.connections:
            writer.close()
            with contextlib.suppress(OSError):
                await writer.wait_closed()
        if self.process.poll() is None:
            self.process.terminate()
        await asyncio.to_thread(self.process.wait, 3)
        self.log.close()

    async def tcp(self):
        for _ in range(100):
            try:
                r, w = await asyncio.open_connection("127.0.0.1", self.port)
                self.connections.append(w)
                return r, w
            except OSError:
                await asyncio.sleep(0.02)
        self.fail("native controller never opened dashboard")

    async def until(self, reader, predicate, timeout=6):
        async def read():
            while line := await reader.readline():
                state = json.loads(line)
                if predicate(state):
                    return state
            self.fail("controller disconnected")
        return await asyncio.wait_for(read(), timeout)

class IntegrationTest(ControllerTestCase):
    async def test_native_feedback_stop_targets_sequences_and_lease(self):
        reader, writer = await self.tcp()
        ready = await self.until(reader, lambda s: s.get("ready") and s["time"] > 3.2)
        self.assertEqual(len(ready["motors"]), 12)
        self.assertEqual(len(ready["boards"]), 6)
        self.assertEqual(ready["motors"]["motor_BRI"]["channel"], 1)
        self.assertEqual(ready["motors"]["motor_BRO"]["channel"], 0)
        self.assertTrue(all(0 <= m["encoder_count"] < 4096 for m in ready["motors"].values()))
        self.assertTrue(all(0 <= m["encoder_age_ms"] < 100 for m in ready["motors"].values()))
        self.assertTrue(any(abs(m["voltage_command_v"]) > 0.01 for m in ready["motors"].values()))

        async def command(seq, op, **fields):
            packet = (json.dumps({"seq": seq, "command": op, **fields}) + "\n").encode()
            # Real TCP fragmentation exercises incremental framing.
            writer.write(packet[:7]); await writer.drain(); await asyncio.sleep(0.002)
            writer.write(packet[7:]); await writer.drain()
            return await self.until(reader, lambda s: s.get("type") == "ack" and s.get("seq") == seq)

        self.assertTrue((await command(1, "stop"))["accepted"])
        await self.until(reader, lambda s: s.get("type") == "state" and all(
            b["state_command"] == 2 for b in s["boards"].values()))
        self.assertFalse((await command(1, "hold"))["accepted"])
        self.assertTrue((await command(2, "hold"))["accepted"])
        held = await self.until(reader, lambda s: s.get("remote_hold"))
        self.assertTrue(all(l["mode"] == "leg_hold" for l in held["legs"].values()))
        self.assertFalse((await command(3, "leg_target", leg="leg_FR", theta=1000, gamma=0, z=0))["accepted"])
        self.assertFalse((await command(4, "leg_target", leg="leg_FR", theta="nan", gamma=0, z=0))["accepted"])
        pose = held["legs"]["leg_FR"]
        self.assertTrue((await command(5, "leg_target", leg="leg_FR", theta=pose["theta"]+0.01,
                                      gamma=pose["gamma"], z=pose["z"]))["accepted"])
        targeted = await self.until(reader, lambda s: s.get("remote_hold"))
        self.assertAlmostEqual(targeted["legs"]["leg_FR"]["target_theta"], pose["theta"]+0.01, places=5)
        await self.until(reader, lambda s: s.get("type") == "state" and
                         not s["remote_hold"] and "lease" in s["dashboard_reason"])
        # Stop is queued through the controller event loop. A snapshot can
        # report the expired lease before that event has been processed.
        await self.until(reader, lambda s: s.get("type") == "state" and all(
            l["mode"] == "stopped" for l in s["legs"].values()) and all(
            b["state_command"] == 2 for b in s["boards"].values()), timeout=0.5)

    async def test_early_stop_cannot_be_undone_by_scripted_startup(self):
        reader, writer = await self.tcp()
        writer.write(b'{"command":"stop","seq":1}\n')
        await writer.drain()
        await self.until(reader, lambda s: s.get("type") == "ack")
        state = await self.until(reader, lambda s: s.get("time", 0) > 3.3)
        self.assertTrue(all(l["mode"] == "stopped" for l in state["legs"].values()))
        self.assertTrue(all(b["state_command"] != 3 for b in state["boards"].values()))


class InitIntegrationTest(ControllerTestCase):
    scenario = "init"

    async def test_init_stage_telemetry_and_stop(self):
        reader, writer = await self.tcp()
        state = await self.until(reader, lambda s: s.get("type") == "state" and
                                 s["legs"]["leg_FR"].get("init_stage") == "seek_gamma_stop")
        self.assertEqual(state["legs"]["leg_BL"]["mode"], "init")
        self.assertEqual(state["legs"]["leg_FL"]["init_stage"], "waiting")
        writer.write(b'{"command":"stop","seq":1}\n')
        await writer.drain()
        self.assertTrue((await self.until(reader, lambda s: s.get("type") == "ack"))["accepted"])
        # Observe past startup delay; no later scripted reactivation.
        stopped = await self.until(reader, lambda s: s.get("time", 0) > state["time"] + 1.5)
        self.assertTrue(all(l["mode"] == "stopped" and "init_stage" not in l for l in stopped["legs"].values()))
        self.assertTrue(all(b["state_command"] == 2 for b in stopped["boards"].values()))

    test_early_stop_cannot_be_undone_by_scripted_startup = IntegrationTest.test_early_stop_cannot_be_undone_by_scripted_startup


class StandupIntegrationTest(ControllerTestCase):
    scenario = "standup"
    duration = 45
    realtime = False

    async def test_stop_during_rise_cancels_remaining_waypoints(self):
        reader, writer = await self.tcp()
        rising = await self.until(reader, lambda s: s.get("type") == "state" and
                                  s["legs"]["leg_FR"].get("init_stage") == "stand", timeout=30)
        writer.write(b'{"command":"stop","seq":1}\n')
        await writer.drain()
        self.assertTrue((await self.until(reader, lambda s: s.get("type") == "ack"))["accepted"])
        stopped = await self.until(reader, lambda s: s.get("time", 0) > rising["time"] + 2)
        self.assertTrue(all(l["mode"] == "stopped" for l in stopped["legs"].values()))
        self.assertTrue(all(b["state_command"] == 2 for b in stopped["boards"].values()))


class SceneTest(unittest.TestCase):
    def test_differential_signs_passive_closure_and_missing_reference(self):
        reference = {leg: {"sensed": [0, 0, 0], "cad": list(stand_angles(leg))} for leg in LEGS}
        scene = SensedScene(reference)
        state = {"ready": True, "backend": "serial", "time": 1,
                 "motors": {f"motor_{leg}{axis}": {"encoder_age_ms": 0, "temperature_c": 30}
                            for leg in LEGS for axis in "IOZ"},
                 "legs": {f"leg_{leg}": {"theta": 0.03, "gamma": -0.02, "z": 0.01} for leg in LEGS}}
        self.assertTrue(scene.update_pose(state))
        for leg in LEGS:
            expected = np.asarray(stand_angles(leg)) + np.linalg.solve(leg_matrix(leg), [0.03, -0.02, 0.01])
            actual = [scene.data.qpos[scene.model.joint(joint).qposadr[0]] for joint in active_joints(leg)]
            np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertLess(max(closure_errors(scene.model, scene.data).values()), 0.0001)
        unreferenced = SensedScene()
        self.assertFalse(unreferenced.update_pose(state))
        state["motors"]["motor_FRZ"]["encoder_age_ms"] = 101
        self.assertFalse(scene.update_pose(state))
        state["motors"]["motor_FRZ"]["encoder_age_ms"] = 0
        self.assertFalse(scene.update_pose(state), "reference must not silently survive feedback loss")



if __name__ == "__main__":
    unittest.main()
