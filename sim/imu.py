"""Ideal chassis IMU. Samples require current MuJoCo forward dynamics.

Axes match base_link. Gyro is rad/s; acceleration is specific force in m/s²
(+9.81 on Z when upright and supported, approximately zero in free fall).
No orientation, position, sensor fusion, noise, bias or transport is inferred.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    time: float
    angular_velocity_rad_s: np.ndarray
    specific_force_m_s2: np.ndarray


def read_imu(model, data):
    """Copy a sample so subsequent physics steps cannot mutate the result.

Call mj_forward after stepping if the sample must match the new data.time;
MuJoCo's mj_step sensor values otherwise describe its pre-integration state.
"""
    acceleration = data.sensor("imu_accelerometer").data.copy()
    site = model.site("imu").id
    if model.body_weldid[model.site_bodyid[site]] == 0:
        # MuJoCo returns zero acceleration for a world-welded body. The
        # suspended fixture's physical accelerometer still senses support.
        acceleration = -data.site_xmat[site].reshape(3, 3).T @ model.opt.gravity
    return ImuSample(float(data.time), data.sensor("imu_gyro").data.copy(), acceleration)
