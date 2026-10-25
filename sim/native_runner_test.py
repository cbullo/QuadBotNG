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
            result = subprocess.run(args, cwd=tmp, env=self.env, capture_output=True, text=True, timeout=max(30, duration))
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
        self.assertTrue(any(max(float(row[name + "_plant_temperature_c"]) for row in rows) > 20.1
                            for name in names))
        # Equivalent standing legs must not create a rear-only Z heat load.
        for leg in ("FR", "FL", "BR", "BL"):
            self.assertLess(float(rows[-1][f"motor_{leg}Z_plant_temperature_c"]), 20.1)
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
                                    thermal={"initial_c": 54, "ambient_c": 54, "heat_capacity_j_per_k": 0.25})
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

    def test_init_probing_preserves_linkage_branch(self):
        first, rows = self.run_scenario("init", suspended=False, duration=8)
        repeated, _ = self.run_scenario("init", suspended=False, duration=8)
        self.assertEqual(first, repeated)
        self.assertTrue(all(r["startup_support"] == "0" for r in rows))
        self.assertAlmostEqual(float(rows[0]["q2"]), 0.0575, delta=1e-5)
        for name in ("FR", "BL"):
            stages = {int(r[f"leg_{name}_init_stage"]) for r in rows}
            self.assertTrue({1, 2, 3, 4}.issubset(stages))
        # The diagonal pair probes while the other pair actively holds.
        for name in ("FL", "BR"):
            self.assertTrue(all(r[f"leg_{name}_init_stage"] in ("-1", "0") for r in rows))
        self.assertEqual(rows[-1]["pose_fault"], "0")
        for main, crank, lower, link in ((8, 10, 9, 11), (13, 15, 14, 16),
                                         (18, 20, 19, 21), (23, 25, 24, 26)):
            for row in rows:
                delta = float(row[f"q{crank}"]) - float(row[f"q{main}"])
                for passive in (lower, link):
                    # Point closure alone allowed a 0.725 rad branch snap
                    # without a MuJoCo warning or a large spatial closure error.
                    self.assertLess(abs(float(row[f"q{passive}"]) - delta), 0.005)
        folding = [r for r in rows if float(r["time"]) < 6]
        for previous, current in zip(folding, folding[1:]):
            dt = float(current["time"]) - float(previous["time"])
            for q in (23, 25):
                # A 5 ms control period caused BL to oscillate at >90 rad/s.
                self.assertLess(abs(float(current[f"q{q}"]) - float(previous[f"q{q}"])) / dt, 10)

    def test_init_requires_belly_on_ground(self):
        result = subprocess.run([str(self.binary), "--backend=sim", "--scenario=init", "--suspended"],
                                env=self.env, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("omit --suspended", result.stderr)

    def test_init_completes_all_legs_and_holds_without_lifting_chassis(self):
        _, rows = self.run_scenario("init", suspended=False, duration=30)
        self.assertIn("homing_complete=1", self.last_stdout)
        self.assertIn("interleg_contacts=0", self.last_stdout)
        completed = float(re.search(r"homing_complete_at=([0-9.e+-]+)", self.last_stdout).group(1))
        self.assertLess(completed, 30)
        self.assertTrue(all(r["pose_fault"] == "0" for r in rows))
        for name in ("FR", "FL", "BL", "BR"):
            stages = {int(r[f"leg_{name}_init_stage"]) for r in rows}
            self.assertTrue(set(range(14)).issubset(stages), (name, stages))
            self.assertEqual(rows[-1][f"leg_{name}_init_stage"], "13")
        # Encoder signs differ with the rear mounting; the physical Z search
        # must still go outward on every leg, not downward into the floor.
        for name, qindex, direction in (("FR", 7, -1), ("FL", 12, 1),
                                         ("BR", 17, -1), ("BL", 22, 1)):
            seeking = [r for r in rows if r[f"leg_{name}_init_stage"] == "4"]
            travel = float(seeking[-1][f"q{qindex}"]) - float(seeking[0][f"q{qindex}"])
            self.assertGreater(direction * travel, 0.2, name)
        # True physics is used only for validation, never by the controller.
        heights = [float(r["q2"]) for r in rows]
        self.assertLess(max(heights) - min(heights), 0.001)
        self.assertLess(float(re.search(r"max_ground_penetration_m=([0-9.e+-]+)", self.last_stdout).group(1)), 0.001)
        self.assertLess(float(re.search(r"max_branch_error_rad=([0-9.e+-]+)", self.last_stdout).group(1)), 0.005)

    def test_standup_runs_init_then_lifts_and_holds_on_four_feet(self):
        _, rows = self.run_scenario("standup", suspended=False, duration=50)
        self.assertIn("homing_complete=1", self.last_stdout)
        self.assertIn("interleg_contacts=0", self.last_stdout)
        self.assertIn("feet_on_ground=4", self.last_stdout)
        completed = float(re.search(r"stand_complete_at=([0-9.e+-]+)", self.last_stdout).group(1))
        self.assertGreater(completed, 0)
        self.assertLess(completed, 45)
        self.assertTrue(all(r["startup_support"] == "0" for r in rows))
        homed = float(re.search(r"homing_complete_at=([0-9.e+-]+)", self.last_stdout).group(1))
        self.assertLess(homed, 30)
        for name in ("FR", "FL", "BL", "BR"):
            standing = [r for r in rows if r[f"leg_{name}_init_stage"] == "15"]
            self.assertTrue(standing, name)
            self.assertGreaterEqual(float(standing[0]["time"]), homed)
        self.assertTrue(all(r["pose_fault"] == "0" for r in rows))
        # No teleport to a standing keyframe; the chassis rises through contact.
        for a, b in zip(rows, rows[1:]):
            self.assertLess(max(abs(float(a[f"q{i}"]) - float(b[f"q{i}"])) for i in range(27)), 0.1)
        holding = [r for r in rows if float(r["time"]) > completed + 2]
        self.assertTrue(holding)
        heights = [float(r["q2"]) for r in holding]
        self.assertGreater(min(heights), 0.20)
        self.assertLess(max(heights) - min(heights), 0.005)
        self.assertTrue(all(abs(float(r["q3"])) > 0.995 for r in holding))

    def test_standup_does_not_start_after_homing_fault(self):
        _, rows = self.run_scenario("standup", suspended=False, duration=8,
                                    thermal={"initial_c": 54, "ambient_c": 54,
                                             "heat_capacity_j_per_k": 0.1})
        self.assertIn("pose_fault=1", self.last_stdout)
        self.assertIn("homing_complete=0", self.last_stdout)
        self.assertIn("standup_started_at=-1", self.last_stdout)
        self.assertIn("stand_complete_at=-1", self.last_stdout)
        for name in ("FR", "FL", "BL", "BR"):
            self.assertTrue(all(r[f"leg_{name}_init_stage"] != "15" for r in rows))
        for row in rows[-100:]:
            self.assertEqual(row["pose_fault"], "1")
            self.assertTrue(all(float(v) == 0 for k, v in row.items() if k.endswith("_torque")))

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
                                    thermal={"initial_c": 54, "ambient_c": 54, "heat_capacity_j_per_k": 0.25})
        resets = re.search(r"thermal_resets=(\d+)", self.last_stdout)
        self.assertGreater(int(resets.group(1)), 0)
        self.assertTrue(all(row["startup_support"] == "0" for row in rows if float(row["time"]) > 3.2))

    def test_bobbing_stance_moves_without_support_or_pose_fault(self):
        _, rows = self.run_scenario("bob", suspended=False, duration=13)
        self.assertIn("feet_on_ground=4", self.last_stdout)
        self.assertIn("watchdog_resets=0 thermal_resets=0 pose_fault=0", self.last_stdout)
        moving = [r for r in rows if float(r["time"]) > 5]
        self.assertTrue(all(r["startup_support"] == "0" for r in moving))
        heights = np.array([float(r["q2"]) for r in moving])
        self.assertGreater(np.ptp(heights), 0.004)
        self.assertLess(np.ptp(heights), 0.025)
        self.assertGreater(heights.min(), 0.21)
        self.assertTrue(all(abs(float(r["q3"])) > 0.995 for r in moving))
        # Two complete cycles should agree in height; all legs must bend and
        # return, rather than merely drift or collapse after startup.
        cycle = lambda start: np.array([float(r["q2"]) for r in rows
                                       if start <= float(r["time"]) < start+4])
        np.testing.assert_allclose(cycle(5), cycle(9), atol=0.002, rtol=0)
        for offset in range(7, 27, 5):
            main = np.array([float(r[f"q{offset+1}"]) for r in moving])
            crank = np.array([float(r[f"q{offset+3}"]) for r in moving])
            self.assertGreater(np.ptp(main), 0.05)
            self.assertGreater(np.ptp(crank), 0.05)

    def test_thermal_collapse_stops_all_motors_and_settles(self):
        # Continue beyond the old 10 s board timeout after a thermal MCU reset.
        # Healthy motors must stop fighting the floor. Self-collision can leave
        # the fallen robot temporarily resting on its own links before rolling
        # onto the chassis, so let the passive physical fall finish.
        for dt in (500, 250):
            with self.subTest(step_us=dt):
                # Straighter legs draw less current. Accelerate heating so the
                # fault occurs early enough to observe passive settling, instead
                # of requiring the old bent-stance thermal failure time.
                _, rows = self.run_scenario("stand", dt=dt, suspended=False, duration=40,
                                            thermal={"initial_c": 54, "ambient_c": 54, "heat_capacity_j_per_k": 0.5})
                self.assertIn("Pose control stopped:", self.last_stdout)
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
