"""End-to-end tests of the actual C++ controller and native plant."""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import time
import re
import unittest

import numpy as np


class NativeRunnerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(os.environ["TEST_SRCDIR"]) / "_main"
        cls.binary = cls.root / "src/odroid_controller/controller/controller"
        cls.env = dict(os.environ, RUNFILES_DIR=str(cls.root.parent))

    def run_scenario(self, scenario, dt=500, suspended=True, thermal=None, duration=5):
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.csv"
            args = [str(self.binary), "--backend=sim", f"--scenario={scenario}",
                    f"--duration={duration}", f"--step-us={dt}", f"--trace={trace}"]
            if suspended:
                args.append("--suspended")
            if thermal:
                config = (self.root / "sim/motor_model.yaml").read_text()
                for key, value in thermal.items():
                    config, count = re.subn(rf"(?m)^  {key}:.*$", f"  {key}: {value}", config)
                    self.assertEqual(count, 1)
                motor_config = Path(tmp) / "motor.yaml"
                motor_config.write_text(config)
                args.append(f"--motor-config={motor_config}")
            result = subprocess.run(args, cwd=tmp, env=self.env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("initialized=1", result.stdout)
            self.last_stdout = result.stdout
            text = trace.read_text()
            rows = list(csv.DictReader(text.splitlines()))
            self.assertTrue(rows)
            return text, rows

    def test_repeatability_and_real_encoder_feedback(self):
        first, rows = self.run_scenario("pulse")
        repeated, _ = self.run_scenario("pulse")
        self.assertEqual(first, repeated)
        torques = np.array([[float(v) for k, v in r.items() if k.endswith("_torque")] for r in rows])
        encoders = np.array([[int(v) for k, v in r.items() if k.endswith("_encoder")] for r in rows])
        self.assertEqual(torques.shape[1], 12)
        self.assertTrue(np.all(np.max(np.abs(torques), axis=0) > 0))
        self.assertLessEqual(np.max(np.abs(torques)), 0.2941995)
        self.assertTrue(np.all((encoders >= 0) & (encoders < 4096)))
        self.assertTrue(np.all(np.ptp(encoders, axis=0) > 0))

    def test_production_leg_control_and_stop(self):
        _, rows = self.run_scenario("stop")
        forces = [[float(v) for k, v in row.items() if k.endswith("_torque")] for row in rows]
        self.assertGreater(np.max(np.abs(forces)), 0)
        for row, force in zip(rows, forces):
            if float(row["time"]) > 4.3:
                np.testing.assert_array_equal(force, np.zeros(12))

    def test_temperature_heating_cooling_and_feedback(self):
        # A loaded stance provides meaningful copper heating; suspended limbs
        # can now rest against the chassis and require almost no holding torque.
        first, rows = self.run_scenario("stand", suspended=False, duration=8)
        repeated, _ = self.run_scenario("stand", suspended=False, duration=8)
        self.assertEqual(first, repeated)
        names = [key.removesuffix("_plant_temperature_c") for key in rows[0]
                 if key.endswith("_plant_temperature_c")]
        self.assertEqual(len(names), 12)
        self.assertTrue(any(max(float(row[name + "_plant_temperature_c"]) for row in rows) > 21
                            for name in names))
        feedback = [row for row in rows if float(row["time"]) > 4.3]
        for name in names:
            # Controller values must be decoded board telemetry, with bounded
            # quantization/delivery lag, and matched by motor name (not index).
            for row in feedback:
                self.assertAlmostEqual(int(row[name + "_temperature_c"]),
                                       float(row[name + "_plant_temperature_c"]), delta=1)
        # Idle motors also cool; all twelve channels deliver non-default samples.
        _, cooling = self.run_scenario("idle", thermal={"initial_c": 40,
                                                       "heat_capacity_j_per_k": 1,
                                                       "thermal_resistance_k_per_w": 5})
        for name in names:
            self.assertGreater(int(cooling[-1][name + "_temperature_c"]), 20)
            self.assertLess(float(cooling[-1][name + "_plant_temperature_c"]), 40)
            self.assertAlmostEqual(int(cooling[-1][name + "_temperature_c"]),
                                   float(cooling[-1][name + "_plant_temperature_c"]), delta=1)

    def test_modeled_heat_triggers_board_watchdog_without_cooling_reset(self):
        _, rows = self.run_scenario("stand", suspended=False,
                                    thermal={"initial_c": 54,"heat_capacity_j_per_k": 2})
        resets = re.search(r"thermal_resets=(\d+)", self.last_stdout)
        self.assertIsNotNone(resets)
        self.assertGreater(int(resets.group(1)), 0)
        temperatures = [[float(value) for key, value in row.items()
                         if key.endswith("_plant_temperature_c")] for row in rows]
        self.assertGreater(np.max(temperatures), 55)
        # Even after MCU resets, physical temperatures retain the initial heat.
        self.assertGreater(np.min(temperatures), 40)

    def test_invalid_thermal_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = (self.root / "sim/motor_model.yaml").read_text().replace(
                "heat_capacity_j_per_k: 20.0", "heat_capacity_j_per_k: 0")
            path = Path(tmp) / "invalid.yaml"
            path.write_text(config)
            result = subprocess.run([str(self.binary), "--backend=sim", "--duration=5",
                                     f"--motor-config={path}"], env=self.env,
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("invalid thermal parameters", result.stderr)

    def test_timestep_convergence(self):
        _, coarse = self.run_scenario("pulse", dt=500)
        _, fine = self.run_scenario("pulse", dt=250)
        # Compare matching final times (trace samples follow a physics step).
        for rows in (coarse, fine):
            self.assertGreater(float(rows[-1]["time"]), 4.99)
        q0 = [float(v) for k, v in coarse[-1].items() if k.startswith("q")]
        q1 = [float(v) for k, v in fine[-1].items() if k.startswith("q")]
        np.testing.assert_allclose(q0, q1, atol=0.02, rtol=0)

    def test_ground_contact_scenario(self):
        self.run_scenario("idle", suspended=False)

    def test_standing_pose_after_startup_support_is_released(self):
        first, rows = self.run_scenario("stand", suspended=False, duration=8)
        self.assertIn("feet_on_ground=4", self.last_stdout)
        self.assertIn("watchdog_resets=0 thermal_resets=0", self.last_stdout)
        release = float(re.search(r"startup_support_released_at=([0-9.]+)", self.last_stdout).group(1))
        self.assertGreater(release, 3)
        self.assertLess(release, 3.2)
        repeated, _ = self.run_scenario("stand", suspended=False, duration=8)
        self.assertEqual(first, repeated)
        _, fine = self.run_scenario("stand", suspended=False, duration=8, dt=250)
        self.assertIn("feet_on_ground=4", self.last_stdout)
        self.assertIn("watchdog_resets=0 thermal_resets=0", self.last_stdout)
        for trace in (rows, fine):
            for offset in range(7, 27, 5):
                opening = 90 + np.rad2deg(float(trace[0][f"q{offset + 3}"]) -
                                         float(trace[0][f"q{offset + 1}"]))
                self.assertAlmostEqual(opening, 90 + np.rad2deg(1.0472), places=6)
            for row in trace:
                if float(row["time"]) < 3:
                    self.assertEqual(row["startup_support"], "1")
                if float(row["time"]) < 4:
                    continue
                self.assertEqual(row["startup_support"], "0")
                self.assertGreater(float(row["q2"]), 0.23)
                self.assertLess(float(row["q2"]), 0.25)
                self.assertLess(abs(float(row["q0"])), 0.02)
                self.assertLess(abs(float(row["q1"])), 0.02)
                self.assertGreater(abs(float(row["q3"])), 0.999)
                # All linkage coordinates stay near the extended stance, including
                # passive closures. No independent-joint servos or fixed base.
                for i in range(7, 27):
                    self.assertAlmostEqual(float(row[f"q{i}"]), float(trace[0][f"q{i}"]), delta=0.08)
                for key, value in row.items():
                    if key.endswith("_torque"):
                        self.assertLessEqual(abs(float(value)), 0.2941995)
        np.testing.assert_allclose([float(rows[-1][f"q{i}"]) for i in range(27)],
                                   [float(fine[-1][f"q{i}"]) for i in range(27)], atol=0.005, rtol=0)

    def test_standing_keeps_thermal_watchdog_active(self):
        _, rows = self.run_scenario("stand", suspended=False, duration=8,
                                    thermal={"heat_capacity_j_per_k": 2})
        resets = re.search(r"thermal_resets=(\d+)", self.last_stdout)
        self.assertGreater(int(resets.group(1)), 0)
        self.assertTrue(all(row["startup_support"] == "0" for row in rows if float(row["time"]) > 3.2))

    def test_thermal_collapse_stops_all_motors_and_settles(self):
        # Continue beyond the old 10 s board timeout after the rear MCU reset.
        # Healthy motors must stop fighting the floor. Self-collision can leave
        # the fallen robot temporarily resting on its own links before rolling
        # onto the chassis, so let the passive physical fall finish.
        for dt in (500, 250):
            with self.subTest(step_us=dt):
                # Straighter legs draw less current. Accelerate heating so the
                # fault occurs early enough to observe passive settling, instead
                # of requiring the old bent-stance thermal failure time.
                _, rows = self.run_scenario("stand", dt=dt, suspended=False, duration=40,
                                            thermal={"heat_capacity_j_per_k": 5})
                self.assertIn("Pose control stopped:", self.last_stdout)
                self.assertIn("controller_BZ:", self.last_stdout)
                self.assertGreater(int(re.search(r"thermal_resets=(\d+)", self.last_stdout).group(1)), 0)
                fault = next(float(row["time"]) for row in rows if row["pose_fault"] == "1")
                first_hot = next(float(row["time"]) for row in rows
                                 if any(float(value) >= 55.5 for key, value in row.items()
                                        if key.endswith("_plant_temperature_c")))
                self.assertGreaterEqual(fault, first_hot - 0.005)
                self.assertLess(fault, first_hot + 0.12)
                after = [row for row in rows if float(row["time"]) > fault + 0.05]
                for row in after:
                    self.assertEqual(row["startup_support"], "0")
                    self.assertEqual(row["pose_fault"], "1")
                    for key, value in row.items():
                        if key.endswith("_torque"):
                            self.assertEqual(float(value), 0)
                for key in rows[0]:
                    if key.endswith("_plant_temperature_c"):
                        temperatures = [float(row[key]) for row in after]
                        self.assertTrue(all(a >= b >= 20 for a, b in zip(temperatures, temperatures[1:])))
                settled = [row for row in rows if float(row["time"]) > 35]
                positions = np.array([[float(row[f"q{i}"]) for i in range(27)] for row in settled])
                self.assertTrue(np.isfinite(positions).all())
                self.assertLess(np.max(np.ptp(positions[:, :3], axis=0)), 0.001)
                velocities = np.diff(positions[:, 7:], axis=0) / 0.005
                self.assertLess(np.sqrt(np.mean(velocities**2)), 0.02)

    def test_six_pty_boards_with_serial_backend(self):
        with tempfile.TemporaryDirectory() as tmp:
            overlay = Path(tmp) / "pty.yaml"
            server = subprocess.Popen([str(self.binary), "--backend=sim", "--pty-server",
                                       "--suspended", "--duration=7", f"--trace={overlay}"],
                                      cwd=tmp, env=self.env, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 5
                while not overlay.exists() and server.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(overlay.exists())
                self.assertEqual(len(re.findall(r"/dev/pts/\d+", overlay.read_text())), 6)
                client = subprocess.run([str(self.binary), "--backend=serial", "--no-joystick",
                                         "--duration=4", f"--config={overlay}"],
                                        cwd=tmp, env=self.env, capture_output=True, text=True, timeout=10)
                self.assertEqual(client.returncode, 0, client.stdout + client.stderr)
                out, err = server.communicate(timeout=10)
                self.assertEqual(server.returncode, 0, out + err)
                count = re.search(r"commands=(\d+)", out)
                self.assertIsNotNone(count)
                self.assertGreater(int(count.group(1)), 200)
            finally:
                if server.poll() is None:
                    server.terminate()
                    server.communicate(timeout=5)

    def test_homing_gate(self):
        result = subprocess.run([str(self.binary), "--backend=sim", "--scenario=homing"],
                                env=self.env, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("mechanical stops", result.stderr)


if __name__ == "__main__":
    unittest.main()
