"""Sensed linkage geometry, using board feedback rather than simulator qpos.

Only kinematics run here. The chassis is anchored; world pose isn't sensed.
An explicit known-pose reference is necessary because the encoders are single
turn and host accumulation doesn't establish absolute mechanical references.
"""
import math

import mujoco
import numpy as np

from sim.robot_model import LEGS, leg_matrix, load_model, set_pose, stand_angles


class SensedScene:
    def __init__(self, reference=None, startup_stand=False):
        self.model = load_model(fixed_base=True)
        self.data = mujoco.MjData(self.model)
        self.reference = reference
        self.startup_stand = startup_stand
        self.invalidated = False
        self.was_ready = False
        self.status = "unreferenced; supply a known-pose reference"
        self.inverse = {leg: np.linalg.inv(leg_matrix(leg)) for leg in LEGS}
        if reference is not None:
            self.validate(reference)

    @staticmethod
    def validate(reference):
        if set(reference) != set(LEGS):
            raise ValueError("reference must contain FR, FL, BR, BL")
        for leg in LEGS:
            for key in ("sensed", "cad"):
                value = reference[leg][key]
                if len(value) != 3 or not all(math.isfinite(x) for x in value):
                    raise ValueError("reference sensed/cad values must contain three finite radians")

    def update_pose(self, state):
        """Update referenced kinematics without allocating transport geometry."""
        ready = state["ready"] and all(0 <= m["encoder_age_ms"] < 100 for m in state["motors"].values())
        if self.was_ready and not ready:
            self.invalidated = True
        self.was_ready = ready
        # Only the launcher-owned stand scenario has a known startup fixture.
        # Never recapture a reference after a board reset, fall or reconnect.
        if self.startup_stand and self.reference is None and ready and not self.invalidated:
            if state["backend"] != "sim" or state["time"] > 3.3 or any(
                    state["legs"][f"leg_{leg}"]["mode"] != "rotor_hold" for leg in LEGS):
                self.invalidated = True
            else:
                self.reference = {leg: {
                    "sensed": [state["legs"][f"leg_{leg}"][axis] for axis in ("theta", "gamma", "z")],
                    "cad": list(stand_angles(leg))} for leg in LEGS}
        if self.invalidated or self.reference is None or not ready:
            self.status = "reference lost; restart/re-reference" if self.invalidated else "waiting for referenced encoder feedback"
            return False
        self.status = "referenced leg pose; chassis anchored; CAD alignment provisional"
        poses = {}
        for leg in LEGS:
            sensed = np.array([state["legs"][f"leg_{leg}"][axis] for axis in ("theta", "gamma", "z")])
            ref = self.reference[leg]
            poses[leg] = np.asarray(ref["cad"]) + self.inverse[leg] @ (sensed - ref["sensed"])
        set_pose(self.model, self.data, poses)
        return True

