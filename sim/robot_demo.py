"""Inspect the robot under rotor-space torque control (not voltage/firmware)."""

import argparse
import os
from pathlib import Path
import time

import mujoco
import numpy as np

from sim.robot_model import LEGS, closure_errors, configure_demo_servos, load_model, set_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixed-base", action="store_true", help="suspend the chassis")
    parser.add_argument("--leg", choices=LEGS, help="include only one leg")
    parser.add_argument("--motor", help="oscillate one rotor, e.g. motor_FRI; hold the others")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--render", type=Path, help="save the final view as a PNG")
    parser.add_argument("--kp", type=float, default=0.8)
    parser.add_argument("--kd", type=float, default=0.005)
    parser.add_argument("--timestep", type=float, default=0.0005)
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("duration must be nonnegative")
    if args.timestep <= 0:
        parser.error("timestep must be positive")
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    model = load_model(fixed_base=args.fixed_base, leg=args.leg)
    model.opt.timestep = args.timestep
    configure_demo_servos(model, args.kp, args.kd)
    data = mujoco.MjData(model)
    set_pose(model, data)
    target = data.actuator_length.copy()
    motor_id = None
    if args.motor:
        motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, args.motor)
        if motor_id < 0:
            parser.error(f"motor {args.motor!r} isn't in this model")
    max_closure = 0
    peak_motor_torque = 0
    contact_seen = False

    def step():
        nonlocal max_closure, peak_motor_torque, contact_seen
        desired = target.copy()
        if motor_id is not None:
            desired[motor_id] += 0.6 * np.sin(2 * np.pi * 0.5 * data.time)
        data.ctrl[:] = desired
        mujoco.mj_step(model, data)
        peak_motor_torque = max(peak_motor_torque, float(np.max(np.abs(data.actuator_force))))
        max_closure = max(max_closure, max(closure_errors(model, data).values()))
        contact_seen |= data.ncon > 0

    if args.headless:
        for _ in range(round(args.duration / model.opt.timestep)):
            step()
    else:
        from mujoco import viewer as viewer_api
        with viewer_api.launch_passive(model, data) as viewer:
            viewer.cam.distance = 0.85
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.lookat[:] = [0, 0, 0.15]
            while viewer.is_running() and data.time < args.duration:
                start = time.monotonic()
                with viewer.lock():
                    step()
                viewer.sync()
                time.sleep(max(0, model.opt.timestep - (time.monotonic() - start)))
    mujoco.mj_forward(model, data)
    if args.render:
        from PIL import Image
        camera = mujoco.MjvCamera()
        camera.distance = 0.85
        camera.azimuth = 135
        camera.elevation = -25
        camera.lookat[:] = [0, 0, 0.15]
        with mujoco.Renderer(model, height=900, width=1200) as renderer:
            renderer.update_scene(data, camera=camera)
            args.render.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(renderer.render()).save(args.render)
        print(f"Saved {args.render}")
    print(f"t={data.time:.3f}s, motors={model.nu}, contact={contact_seen}, "
          f"peak closure error={max_closure:.6g} m, warnings={data.warning.number.sum()}")
    print(f"base position={data.body('base_link').xpos}, max motor error="
          f"{np.max(np.abs(target - data.actuator_length)):.4g} rad")
    print(f"base velocity norm={np.linalg.norm(data.body('base_link').cvel):.4g}, "
          f"contacts={data.ncon}")
    print(f"peak motor shaft torque={peak_motor_torque:.8g} N m "
          f"({peak_motor_torque / 0.0000980665:.4g} gf cm)")
    if np.any(data.warning.number) or not np.isfinite(data.qpos).all():
        raise RuntimeError("Simulation produced warnings or non-finite state")


if __name__ == "__main__":
    main()
