"""Mechanical regression checks, independent of hardware and rendering."""

import re
import unittest

import mujoco
import numpy as np

from sim.robot_model import (LEGS, LEG_LIMITS, PARAMETERS, SIM_DIR, active_joints, build_xml,
                             closure_errors, configure_demo_servos, leg_matrix, load_model, motor_matrix,
                             set_pose)


class RobotModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_model()
        cls.fixed = load_model(fixed_base=True)

    def test_structure_inertia_and_generated_asset(self):
        m = self.model
        self.assertEqual((m.nu, m.neq, m.nv), (12, 4, 26))
        np.testing.assert_array_equal(m.dof_frictionloss[:6], 0)  # Free chassis.
        self.assertAlmostEqual(m.body_mass.sum(), 5.049489443143543, places=8)
        self.assertTrue(np.all(m.body_inertia[1:] > 0))
        for inertia in m.body_inertia[1:]:
            self.assertLessEqual(max(inertia), sum(inertia) - max(inertia) + 1e-12)
        xml = SIM_DIR / "mujoco/quadbot.xml"
        self.assertEqual(xml.read_text(), build_xml())
        standalone = mujoco.MjModel.from_xml_path(str(xml))
        self.assertEqual(standalone.nu, 12)
        self.assertEqual((SIM_DIR / "mujoco/quadbot_suspended.xml").read_text(), build_xml(fixed_base=True))

    def test_hardware_mapping_snapshot(self):
        # Read only the scalar fields we mirror, not the electrical calibration.
        # Deliberately fail if the YAML structure changes; this isn't a YAML loader.
        source = (SIM_DIR.parent / "src/odroid_controller/controller/config/robot_config.yaml").read_text()

        def block(key):
            return re.search(r"^  " + key + r":\n(.*?)(?=^  \w|\Z)",
                             source, re.M | re.S).group(1)

        def scalar(text, key):
            return re.search(r"^    " + key + r":\s*([^\n#]+)", text, re.M).group(1).strip().strip('"')

        for name, p in LEGS.items():
            leg = block(f"leg_{name}")
            self.assertEqual(float(scalar(leg, "z_gamma_dir")), p["z_gamma_dir"])
            for i, axis in enumerate("IOZ"):
                motor = f"motor_{name}{axis}"
                self.assertEqual(scalar(leg, f"motor_{axis}"), motor)
                config = block(motor)
                self.assertEqual(float(scalar(config, "gear_ratio_inv")), p["gear_ratio"][i])
                self.assertEqual(scalar(config, "controller"), p["boards"][i])
                self.assertEqual(int(scalar(config, "index")), p["channels"][i])

    def test_extended_stand_keyframe_is_closed_and_clear_of_self_contact(self):
        for fixed in (False, True):
            m = load_model(fixed_base=fixed)
            d = mujoco.MjData(m)
            mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
            mujoco.mj_forward(m, d)
            self.assertEqual(d.ncon, 0)
            self.assertLess(max(closure_errors(m, d).values()), 1e-10)
            for leg in LEGS:
                cad = LEGS[leg]["cad"]
                # Measure the physical bend between thigh and lower-arm axes.
                toward_hip = d.body(f"arm_main_{cad}_1").xmat.reshape(3, 3)[:, 2]
                toward_foot = d.body(f"arm_lower_{cad}_1").xmat.reshape(3, 3)[:, 0]
                opening = np.rad2deg(np.arccos(np.clip(toward_hip @ toward_foot, -1, 1)))
                p, limits = LEGS[leg], LEG_LIMITS[leg]
                gamma_target = limits["max_gamma" if p["gamma_sign"] > 0 else "min_gamma"]
                self.assertAlmostEqual(opening, 90 + np.rad2deg(gamma_target / p["gamma_sign"]), places=8)
                self.assertGreater(opening, 149)
                self.assertLess(opening, 151)
                self.assertAlmostEqual(d.ten_length[m.tendon(f"gamma_{leg}").id], gamma_target, places=10)

    def test_configured_extrema_and_linkage_sweep_are_clear(self):
        for leg, p in LEGS.items():
            m = load_model(fixed_base=True, leg=leg)
            d = mujoco.MjData(m)
            limits = LEG_LIMITS[leg]
            gamma = m.tendon(f"gamma_{leg}")
            self.assertTrue(m.tendon_limited[gamma.id])
            np.testing.assert_allclose(m.tendon_range[gamma.id], [limits["min_gamma"], limits["max_gamma"]])
            shoulder = m.joint(active_joints(leg)[2])
            self.assertTrue(m.jnt_limited[shoulder.id])
            np.testing.assert_allclose(m.jnt_range[shoulder.id], sorted(limits[k] / p["z_sign"] for k in ("min_z", "max_z")))
            for gamma_value in np.linspace(limits["min_gamma"], limits["max_gamma"], 101):
                delta = gamma_value / p["gamma_sign"]
                main, crank = np.pi / 4 - delta / 2, np.pi / 4 + delta / 2
                for z in (limits["min_z"], 0, limits["max_z"]):
                    set_pose(m, d, {leg: (main, crank, z / p["z_sign"])})
                    self.assertLess(max(closure_errors(m, d).values()), 1e-12)
                    pairs = [{m.body(m.geom_bodyid[int(g)]).name for g in c.geom} for c in d.contact]
                    # Joint intervals are not a collision-free Cartesian
                    # workspace. Fold + extreme shoulder roll can hit the
                    # chassis; preserve that real external envelope contact.
                    for pair in pairs:
                        self.assertTrue(pair & {"base_link", "drives_1"}, (leg, gamma_value, z, pair))
                    if z == 0:
                        self.assertEqual(d.ncon, 0, (leg, gamma_value))
            # Z endpoints are also clear at the nominal gamma=0 bend.
            for z in (limits["min_z"], limits["max_z"]):
                set_pose(m, d, {leg: (np.pi / 4, np.pi / 4, z / p["z_sign"])})
                self.assertEqual(d.ncon, 0)

    def test_internal_proxies_still_collide_and_outer_envelopes_remain_active(self):
        m = load_model(fixed_base=True)
        d = mujoco.MjData(m)
        for leg, p in LEGS.items():
            cad = p["cad"]
            delta = np.deg2rad(110)  # Deliberate overlap outside configured range.
            set_pose(m, d, {leg: (np.pi / 4 - delta / 2, np.pi / 4 + delta / 2, 0)})
            pairs = [{m.geom(int(g)).name for g in c.geom} for c in d.contact]
            self.assertIn({f"arm_main_{cad}_1_self_collision_0", f"arm_link_{cad}_1_self_collision_0"}, pairs)
            foot = m.geom(f"arm_lower_{cad}_1_collision_1").id
            core = m.geom(f"arm_lower_{cad}_1_self_collision_1").id
            self.assertAlmostEqual(m.geom_size[foot, 0], 0.019)

            def eligible(a, b):
                return bool((m.geom_contype[a] & m.geom_conaffinity[b]) or
                            (m.geom_contype[b] & m.geom_conaffinity[a]))

            for name in ("ground", "base_link_collision_0"):
                self.assertTrue(eligible(foot, m.geom(name).id))
                self.assertFalse(eligible(core, m.geom(name).id))
            for other, op in LEGS.items():
                if other != leg:
                    self.assertTrue(eligible(foot, m.geom(f"arm_lower_{op['cad']}_1_collision_1").id))

    def test_configured_limits_stop_motor_driven_motion(self):
        for leg, p in LEGS.items():
            for axis in ("gamma", "z"):
                for bound, sign in (("min", -1), ("max", 1)):
                    with self.subTest(leg=leg, axis=axis, bound=bound):
                        m = load_model(fixed_base=True, leg=leg)
                        m.opt.gravity[:] = 0
                        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
                        configure_demo_servos(m)
                        d = mujoco.MjData(m)
                        limit = LEG_LIMITS[leg][f"{bound}_{axis}"]
                        requested = limit + sign * 0.15
                        delta = requested / p["gamma_sign"] if axis == "gamma" else 0
                        target = (np.pi / 4 - delta / 2, np.pi / 4 + delta / 2,
                                  requested / p["z_sign"] if axis == "z" else 0)
                        set_pose(m, d)
                        d.ctrl[:] = motor_matrix(leg) @ target
                        for _ in range(round(2 / m.opt.timestep)):
                            mujoco.mj_step(m, d)
                        q = [d.joint(j).qpos[0] for j in active_joints(leg)]
                        value = (leg_matrix(leg) @ q)[1 if axis == "gamma" else 2]
                        self.assertAlmostEqual(value, limit, delta=0.002)
                        self.assertFalse(np.any(d.warning.number))
                        self.assertLess(max(closure_errors(m, d).values()), 0.001)

    def test_primitive_collisions_cover_bodies_without_false_stance_contacts(self):
        for fixed in (False, True):
            model = load_model(fixed_base=fixed)
            data = mujoco.MjData(model)
            set_pose(model, data)
            self.assertEqual(data.ncon, 0)
            allowed = {int(kind) for kind in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                            mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE)}
            for body in range(1, model.nbody):
                geoms = np.where((model.geom_bodyid == body) & (model.geom_group == 3))[0]
                self.assertGreater(len(geoms), 0, model.body(body).name)
                for geom in geoms:
                    self.assertIn(model.geom_type[geom], allowed)
                    self.assertNotEqual(model.geom_conaffinity[geom], 0)
            visual = np.where(model.geom_group == 2)[0]
            self.assertTrue(np.all(model.geom_contype[visual] == 0))
            self.assertTrue(np.all(model.geom_conaffinity[visual] == 0))

    def test_folded_legs_are_blocked_by_chassis_contacts(self):
        for leg in LEGS:
            with self.subTest(leg=leg):
                model = load_model(fixed_base=True, leg=leg)
                # This collision stress test intentionally folds inward beyond
                # the controller's Z range. Isolate contact response from stops.
                model.jnt_limited[:] = False
                model.tendon_limited[:] = False
                configure_demo_servos(model)
                data = mujoco.MjData(model)
                set_pose(model, data)
                target_shoulder = 1.5 if leg in ("FR", "BR") else -1.5
                target = motor_matrix(leg) @ [np.pi / 4, np.pi / 4, target_shoulder]
                ids = [model.actuator(f"motor_{leg}{axis}").id for axis in "IOZ"]
                touched = False
                peak_penetration = 0
                for _ in range(round(3 / model.opt.timestep)):
                    data.ctrl[ids] = target
                    mujoco.mj_step(model, data)
                    for contact in data.contact:
                        bodies = {model.body(model.geom_bodyid[int(g)]).name for g in contact.geom}
                        if bodies & {"base_link", "drives_1"}:
                            touched = True
                            peak_penetration = max(peak_penetration, -contact.dist)
                self.assertTrue(touched)
                # The abrupt target step saturates motors into compliant
                # contact; allow 3 mm transient overlap, not persistent pass-through.
                self.assertLess(peak_penetration, 0.003)
                shoulder = data.joint(active_joints(leg)[2]).qpos[0]
                self.assertGreater(abs(shoulder), 0.5)
                self.assertLess(abs(shoulder), 1.0)
                self.assertLess(max(closure_errors(model, data).values()), 0.0001)
                self.assertFalse(np.any(data.warning.number))
                # The limit above is caused by collision response, not a hidden
                # joint limit or changes to the differential/torque ceiling.
                model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
                set_pose(model, data)
                for _ in range(round(3 / model.opt.timestep)):
                    data.ctrl[ids] = target
                    mujoco.mj_step(model, data)
                self.assertAlmostEqual(data.joint(active_joints(leg)[2]).qpos[0], target_shoulder, delta=0.05)

    def test_opposite_legs_can_collide(self):
        model = load_model(fixed_base=True)
        data = mujoco.MjData(model)
        set_pose(model, data, {"FR": (np.pi/4, np.pi/4, 0.6),
                               "FL": (np.pi/4, np.pi/4, -0.6)})
        pairs = [{model.body(model.geom_bodyid[int(g)]).name for g in contact.geom}
                 for contact in data.contact]
        self.assertIn({"arm_lower_RF_1", "arm_lower_LF_1"}, pairs)

    def test_closure_over_workspace_and_mobility(self):
        m = self.fixed
        d = mujoco.MjData(m)
        rng = np.random.default_rng(42)
        for _ in range(30):
            poses = {name: rng.uniform([0.2, 0.2, -0.35], [1.2, 1.2, 0.35]) for name in LEGS}
            set_pose(m, d, poses)
            self.assertLess(max(closure_errors(m, d).values()), 1e-12)
            # Each planar ball closure removes two independent coordinates;
            # its third equation is redundant because the bearing axes align.
            equality_rows = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
            rank = np.linalg.matrix_rank(d.efc_J.reshape(d.nefc, m.nv)[equality_rows], tol=1e-8)
            self.assertEqual(m.nv - rank, 12)

    def test_encoder_equations_and_virtual_work(self):
        m = self.fixed
        d = mujoco.MjData(m)
        rng = np.random.default_rng(12)
        poses = {name: rng.uniform(0.2, 0.8, 3) for name in LEGS}
        set_pose(m, d, poses)
        d.qvel[:] = rng.normal(size=m.nv)
        d.ctrl[:] = rng.uniform(-0.05, 0.05, m.nu)
        mujoco.mj_forward(m, d)
        expected_force = np.zeros(m.nv)
        for name, p in LEGS.items():
            ids = [m.actuator(f"motor_{name}{axis}").id for axis in "IOZ"]
            dofs = [m.joint(j).dofadr[0] for j in active_joints(name)]
            ai, ao, z = d.actuator_length[ids] / p["gear_ratio"]
            actual_leg = [-0.5 * (ai + ao), ao - ai - p["z_gamma_dir"] * 4 / 3 * z, z]
            np.testing.assert_allclose(actual_leg, leg_matrix(name) @ poses[name], atol=1e-12)
            expected_force[dofs] = motor_matrix(name).T @ d.ctrl[ids]
            # Independent numerical derivative of MuJoCo's actual transmission.
            for column, joint in enumerate(active_joints(name)):
                address = m.joint(joint).qposadr[0]
                old = d.qpos[address]
                before = d.actuator_length[ids].copy()
                d.qpos[address] += 1e-7
                mujoco.mj_forward(m, d)
                derivative = (d.actuator_length[ids] - before) / 1e-7
                np.testing.assert_allclose(derivative, motor_matrix(name)[:, column], atol=2e-8)
                d.qpos[address] = old
                mujoco.mj_forward(m, d)
        np.testing.assert_allclose(d.qfrc_actuator, expected_force, atol=1e-12)
        self.assertAlmostEqual(d.qvel @ d.qfrc_actuator,
                               d.actuator_velocity @ d.actuator_force, places=12)

    def test_each_motor_and_timestep_convergence(self):
        # Independent motor torques exercise reaction on the shoulder and both
        # branches. Pulses exceed dry friction but stay far below the shaft
        # torque ceiling; their short duration avoids four-bar singularities.
        for name in LEGS:
            m = load_model(fixed_base=True, leg=name)
            m.opt.gravity[:] = 0
            for motor in range(3):
                endpoints = []
                for dt in (0.001, 0.0005):
                    m.opt.timestep = dt
                    d = mujoco.MjData(m)
                    set_pose(m, d)
                    start = d.qpos.copy()
                    d.ctrl[motor] = 0.02
                    for _ in range(round(0.04 / dt)):
                        mujoco.mj_step(m, d)
                    mujoco.mj_forward(m, d)
                    self.assertTrue(np.isfinite(d.qpos).all())
                    self.assertGreater(np.linalg.norm(d.qpos - start), 1e-4)
                    self.assertLess(max(closure_errors(m, d).values()), 2e-5)
                    self.assertFalse(np.any(d.warning.number))
                    endpoints.append(d.qpos.copy())
                np.testing.assert_allclose(*endpoints, atol=0.003, rtol=0)

    def test_raised_leg_dry_friction_holds_but_releases_under_extra_load(self):
        for leg in LEGS:
            for dt in (0.0005, 0.00025):
                with self.subTest(leg=leg, timestep=dt):
                    m = load_model(fixed_base=True, leg=leg)
                    m.opt.timestep = dt
                    # Gravity reversal is equivalent to turning the suspended
                    # chassis upside down: the bent leg now points upwards.
                    # This avoids starting exactly at an unstable vertical
                    # equilibrium, where zero friction would appear to hold.
                    m.opt.gravity[2] = 9.81
                    d = mujoco.MjData(m)
                    set_pose(m, d)
                    self.assertEqual(d.ncon, 0)
                    cad = LEGS[leg]["cad"]
                    dofs = [m.joint(j).dofadr[0] for j in active_joints(leg)]
                    passive = [m.joint(j).dofadr[0] for j in
                               (f"main_to_lower_{cad}", f"crank_to_link_{cad}")]
                    # Both passive angles = crank-main. Project gravity onto
                    # the three independent coordinates by virtual work.
                    gravity = d.qfrc_bias[dofs].copy()
                    gravity[:2] += [-sum(d.qfrc_bias[passive]), sum(d.qfrc_bias[passive])]
                    ratio = m.dof_frictionloss[dofs] / abs(gravity)
                    self.assertTrue(np.all((ratio > 1.05) & (ratio < 1.15)))
                    np.testing.assert_array_equal(m.dof_frictionloss[passive], 0)
                    start = d.qpos.copy()
                    for _ in range(round(5 / dt)):
                        mujoco.mj_step(m, d)
                        self.assertEqual(d.ncon, 0)  # No chassis/floor support.
                    self.assertLess(np.max(abs(d.qpos - start)), 0.001)
                    np.testing.assert_array_equal(d.actuator_force, 0)
                    self.assertFalse(np.any(d.warning.number))

                    # Only 25% more gravitational load must overcome friction.
                    # The model must also fall with dry friction removed.
                    for gravity_scale, friction_scale in ((1.25, 1), (1, 0)):
                        m.opt.gravity[2] = 9.81 * gravity_scale
                        m.dof_frictionloss[:] *= friction_scale
                        set_pose(m, d)
                        for _ in range(round(1 / dt)):
                            mujoco.mj_step(m, d)
                        self.assertGreater(np.max(abs(d.qpos - start)), 0.1)
                        self.assertFalse(np.any(d.warning.number))

    def test_reflected_rotor_inertia(self):
        m = load_model(fixed_base=True, leg="FR")
        d = mujoco.MjData(m)
        set_pose(m, d)
        with_rotors = np.empty((m.nv, m.nv))
        without_rotors = np.empty_like(with_rotors)
        mujoco.mj_fullM(m, d, with_rotors)
        m.tendon_armature[:] = 0
        mujoco.mj_forward(m, d)
        mujoco.mj_fullM(m, d, without_rotors)
        expected = np.zeros_like(with_rotors)
        dofs = [m.joint(j).dofadr[0] for j in active_joints("FR")]
        b = motor_matrix("FR")
        expected[np.ix_(dofs, dofs)] = PARAMETERS["rotor_inertia_kg_m2"] * b.T @ b
        np.testing.assert_allclose(with_rotors - without_rotors, expected, atol=1e-14)

    def test_unactuated_free_fall(self):
        m = self.model
        d = mujoco.MjData(m)
        set_pose(m, d)
        start = d.subtree_com[1].copy()
        for _ in range(round(0.05 / m.opt.timestep)):
            mujoco.mj_step(m, d)
            self.assertEqual(d.ncon, 0)
        mujoco.mj_forward(m, d)
        expected = start + 0.5 * m.opt.gravity * d.time**2
        np.testing.assert_allclose(d.subtree_com[1], expected, atol=0.0003)

    def test_falling_contact_and_static_hold(self):
        m = load_model()
        configure_demo_servos(m)
        d = mujoco.MjData(m)
        set_pose(m, d)
        target = d.actuator_length.copy()
        touched = False
        max_error = 0
        for _ in range(round(2 / m.opt.timestep)):
            # Rotor-space PD for a numerical load test, not the hardware PID.
            d.ctrl[:] = target
            mujoco.mj_step(m, d)
            touched |= d.ncon > 0
            max_error = max(max_error, max(closure_errors(m, d).values()))
        self.assertTrue(touched)
        self.assertFalse(np.any(d.warning.number))
        self.assertTrue(np.isfinite(d.qpos).all())
        self.assertGreater(d.qpos[2], 0.10)
        self.assertLess(max_error, 0.001)
        self.assertLess(np.linalg.norm(d.qvel[:6]), 0.2)
        mujoco.mj_forward(m, d)
        contact_bodies = {m.body(m.geom_bodyid[g]).name for c in d.contact
                          for g in (c.geom1, c.geom2)}
        for p in LEGS.values():
            self.assertIn(f"arm_lower_{p['cad']}_1", contact_bodies)
        print(f"2 s loaded contact: peak closure error {max_error:.3g} m; base z {d.qpos[2]:.3f} m")


if __name__ == "__main__":
    unittest.main()
