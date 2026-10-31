"""Check actual robot IMU axes, gravity convention, motion and sample ownership."""
import unittest

import mujoco
import numpy as np

from sim.imu import read_imu
from sim.robot_model import load_model, set_pose


class ImuTest(unittest.TestCase):
    def test_supported_and_tilted_gravity(self):
        model = load_model(fixed_base=True)
        data = mujoco.MjData(model)
        set_pose(model, data)
        sample = read_imu(model, data)
        np.testing.assert_allclose(sample.angular_velocity_rad_s, 0, atol=1e-12)
        np.testing.assert_allclose(sample.specific_force_m_s2, [0, 0, 9.81], atol=1e-12)
        # Rotate the chassis +90 degrees about X. Local +Y now points up.
        model.body_quat[model.body("base_link").id] = [2**-.5, 2**-.5, 0, 0]
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(read_imu(model, data).specific_force_m_s2, [0, 9.81, 0], atol=1e-12)
        np.testing.assert_array_equal(sample.specific_force_m_s2, [0, 0, 9.81])

    def test_freefall_and_body_frame_rotation(self):
        model = load_model()
        data = mujoco.MjData(model)
        set_pose(model, data)
        data.qpos[2] = 2
        mujoco.mj_forward(model, data)
        self.assertEqual(data.ncon, 0)
        np.testing.assert_allclose(read_imu(model, data).specific_force_m_s2, 0, atol=1e-9)
        # Support the floating robot with forces, verifying the native sensor
        # rather than the suspended-fixture correction above.
        data.qfrc_applied[:] = data.qfrc_bias
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(read_imu(model, data).specific_force_m_s2, [0, 0, 9.81], atol=1e-9)
        data.qfrc_applied[:] = 0
        # Free-joint angular velocity is local. Tilt makes world/local differ.
        data.qpos[3:7] = [2**-.5, 2**-.5, 0, 0]
        data.qvel[3:6] = [.2, -.3, .4]
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(read_imu(model, data).angular_velocity_rad_s, [.2, -.3, .4], atol=1e-12)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        self.assertEqual(read_imu(model, data).time, model.opt.timestep)


if __name__ == "__main__":
    unittest.main()
