"""Isolated MJWarp feasibility experiment, not a training backend.

Run with a separate Python environment from the repository root:
  PYTHONPATH=. python sim/experiments/gpu_physics_benchmark.py --worlds=1
Dependencies are pinned separately; no Bazel or project dependency changes.
Measures the actual 40-substep motor/physics/heat loop, NOT full PPO throughput.
"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import time

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from sim.walk_env import QuadBotWalkEnv
from sim.robot_model import SIM_DIR, build_xml, closure_errors

wp.config.kernel_cache_dir = os.environ.get("QUADBOT_WARP_CACHE", "/tmp/quadbot-warp-cache")


@wp.kernel
def plant_step(length: wp.array2d[float], velocity: wp.array2d[float],
               target: wp.array2d[float], voltage: wp.array2d[float], ctrl: wp.array2d[float],
               temperature: wp.array2d[wp.float64], energy: wp.array3d[wp.float64],
               limits: wp.array[float], params: wp.array[wp.float64], substep: int):
    w, a = wp.tid()
    # Same sign, phase-voltage factor, back-EMF and saturation as VoltageMotorPlant.
    vmax, sign = float(params[0]), float(params[1])
    ke, resistance = float(params[2]), float(params[3])
    if substep % 2 == 0:
        voltage[w, a] = wp.clamp(sign * (float(params[4]) * (target[w, a]-length[w, a])
                                - float(params[5])*velocity[w, a]), -vmax, vmax)
    current = (0.5*sign*voltage[w, a] - ke*velocity[w, a])/resistance
    torque = wp.clamp(1.5*ke*current, -limits[a], limits[a])
    ctrl[w, a] = torque
    torque64 = wp.float64(torque)
    iq = torque64/(wp.float64(1.5)*params[2])
    copper = wp.float64(1.5)*params[3]*iq*iq
    temperature[w, a] += (params[6] + copper*params[7] - temperature[w, a])*params[8]
    if substep == 0:
        energy[w, a, 0] = wp.float64(0.0)
        energy[w, a, 1] = wp.float64(0.0)
        energy[w, a, 2] = wp.float64(0.0)
    energy[w, a, 0] += copper*params[9]
    energy[w, a, 1] += wp.max(torque64*wp.float64(velocity[w, a]), wp.float64(0.0))*params[9]
    energy[w, a, 2] += torque64*torque64*params[9]/wp.float64(12.0)


class GpuPlant:
    def __init__(self, env, worlds, nconmax, njmax):
        self.env, self.worlds = env, worlds
        self.closure_link_ids = [env.model.site(f"closure_link_{leg}").id for leg in ("FR","FL","BR","BL")]
        self.closure_lower_ids = [env.model.site(f"closure_lower_{leg}").id for leg in ("FR","FL","BR","BL")]
        self.m = mjw.put_model(env.model)
        self.nconmax, self.njmax = nconmax, njmax
        p = env.plant
        self.params = wp.array(np.array([p.voltage_limit, p.sign, p.ke, p.resistance, p.kp, p.kd,
            p.ambient, p.thermal_resistance, -np.expm1(-env.model.opt.timestep/p.capacity/p.thermal_resistance),
            env.model.opt.timestep]), dtype=wp.float64)
        self.limits = wp.array(p.limits, dtype=float)
        self.restore()

    def restore(self):
        self.d = mjw.put_data(self.env.model, self.env.data, nworld=self.worlds,
                             nconmax=self.nconmax, njmax=self.njmax)
        self.target = wp.array(np.tile(self.env.nominal_motor_position, (self.worlds, 1)), dtype=float)
        self.voltage = wp.zeros((self.worlds, 12), dtype=float)
        self.temperature = wp.array(np.tile(self.env.plant.temperature, (self.worlds, 1)), dtype=wp.float64)
        self.energy = wp.zeros((self.worlds, 12, 3), dtype=wp.float64)

    def advance(self):
        for i in range(self.env.frame_skip):
            mjw.step1(self.m, self.d)
            wp.launch(plant_step, dim=(self.worlds, 12), inputs=[self.d.actuator_length,
                self.d.actuator_velocity, self.target, self.voltage, self.d.ctrl,
                self.temperature, self.energy, self.limits, self.params, i])
            mjw.step2(self.m, self.d)
        mjw.forward(self.m, self.d)  # Current-state IMU/kinematics, as in CPU _advance.

    def check(self):
        overflow = self.d.overflow.numpy()
        qpos, qvel, temp = self.d.qpos.numpy(), self.d.qvel.numpy(), self.temperature.numpy()
        if np.any(overflow) or not all(np.isfinite(v).all() for v in (qpos, qvel, temp)):
            raise RuntimeError(f"GPU physics failed: overflow={np.unique(overflow)}, nonfinite state")
        # This scoped benchmark lacks masked early termination/reset. Reject such traces.
        if np.max(temp) > 55:
            raise RuntimeError("Benchmark exceeded thermal cutoff; result is not a valid training transition rate")
        sites = self.d.site_xpos.numpy()
        closure = np.linalg.norm(sites[:, self.closure_link_ids]-sites[:, self.closure_lower_ids], axis=2)
        if closure.max() > .005:
            raise RuntimeError('GPU linkage closure exceeds the training cutoff')
        return dict(max_temperature_c=float(temp.max()), min_base_height_m=float(qpos[:, 2].min()),
                    max_closure_error_m=float(closure.max()),
                    overflow_bits=int(np.bitwise_or.reduce(overflow)))


def run(args):
    wp.init()
    result = dict(versions={p: importlib.metadata.version(p) for p in
                          ('mujoco', 'mujoco-warp', 'warp-lang', 'numpy', 'gymnasium')},
                  model_sha256=hashlib.sha256(build_xml().encode()).hexdigest(),
                  motor_sha256=hashlib.sha256((SIM_DIR/'motor_model.yaml').read_bytes()).hexdigest(),
                  scope="40 physics substeps + voltage PD + torque + float64 heat/energy + forward; no PPO/reward/reset",
                  worlds=args.worlds, nconmax=args.nconmax, njmax=args.njmax)
    with wp.ScopedDevice('cuda:0'), QuadBotWalkEnv(robustness='none') as env:
        env.reset(seed=42, options=dict(command=[0, 0, 0], initial_temperature_c=40.))
        env.data.qpos[2] += args.drop_height
        mujoco.mj_forward(env.model, env.data)
        result['initial_drop_height_m'] = args.drop_height
        result['diverse_worlds'] = args.diverse
        result['initial_contacts'] = env.data.ncon
        result['initial_constraints'] = env.data.nefc
        started = time.perf_counter()
        gpu = GpuPlant(env, args.worlds, args.nconmax, args.njmax)
        print('Model uploaded; compiling kernels...', flush=True)
        gpu.advance()
        wp.synchronize()
        result['compile_upload_warmup_s'] = time.perf_counter()-started
        gpu.restore()
        if args.diverse:
            gpu.temperature.assign(np.tile(env.plant.temperature, (args.worlds, 1))
                                   - np.linspace(0, 20, args.worlds)[:, None])
        started = time.perf_counter()
        with wp.ScopedCapture() as capture:
            gpu.advance()
        result['capture_s'] = time.perf_counter()-started
        graph = capture.graph
        result['cpu_solver_tolerance'] = env.model.opt.tolerance
        result['gpu_solver_tolerance'] = float(gpu.m.opt.tolerance.numpy()[0])
        result['pre_graph_memory_used_mib'] = (wp.get_device().total_memory-wp.get_device().free_memory)/2**20
        print('Graph ready; checking CPU/GPU trajectories...', flush=True)
        cpu_wall, gpu_wall, max_qpos, max_qvel, max_temp = 0., 0., 0., 0., 0.
        max_copper_relative, max_gpu_closure = 0., 0.
        max_base_position_error, max_foot_position_error = 0., 0.
        sample = mujoco.MjData(env.model)
        # Controlled rotor offsets exercise the actual differential, not ideal joint servos.
        for i in range(args.verify_steps):
            target = env.nominal_motor_position + args.amplitude*np.sin(i*.1 + np.arange(12))
            targets = np.tile(target, (args.worlds, 1))
            if args.diverse:
                targets = env.nominal_motor_position + args.amplitude*np.sin(
                    i*.1 + np.arange(12)[None, :] + .37*np.arange(args.worlds)[:, None])
            gpu.target.assign(targets.astype(np.float32))
            wp.synchronize()
            started = time.perf_counter()
            wp.capture_launch(graph)
            wp.synchronize()
            gpu_wall += time.perf_counter()-started
            started = time.perf_counter()
            env._advance(target)
            cpu_wall += time.perf_counter()-started
            if np.max(env.plant.temperature)>55 or np.any(env.data.warning.number):
                raise RuntimeError('CPU verification trace failed')
            gpu.check()
            qpos = gpu.d.qpos.numpy()[0]
            qvel = gpu.d.qvel.numpy()[0]
            temp = gpu.temperature.numpy()[0]
            max_qpos = max(max_qpos, float(np.max(np.abs(qpos-env.data.qpos))))
            max_qvel = max(max_qvel, float(np.max(np.abs(qvel-env.data.qvel))))
            max_temp = max(max_temp, float(np.max(np.abs(temp-env.plant.temperature))))
            copper = gpu.energy.numpy()[0, :, 0].sum()
            max_copper_relative = max(max_copper_relative, abs(copper-env.copper_energy_j)/max(env.copper_energy_j,1e-10))
            sample.qpos[:] = qpos
            mujoco.mj_forward(env.model, sample)
            max_gpu_closure = max(max_gpu_closure, max(closure_errors(env.model, sample).values()))
            max_base_position_error = max(max_base_position_error, float(np.linalg.norm(sample.qpos[:3]-env.data.qpos[:3])))
            max_foot_position_error = max(max_foot_position_error,
                float(np.linalg.norm(sample.geom_xpos[env.foot_ids]-env.data.geom_xpos[env.foot_ids],axis=1).max()))
        result['verification'] = dict(steps=args.verify_steps, rotor_target_amplitude_rad=args.amplitude,
            cpu_seconds=cpu_wall, gpu_seconds=gpu_wall, max_qpos_abs_error=max_qpos,
            max_qvel_abs_error=max_qvel, max_temperature_error_c=max_temp,
            max_copper_relative_error=max_copper_relative, max_gpu_closure_m=max_gpu_closure, max_base_position_error_m=max_base_position_error,
            max_foot_position_error_m=max_foot_position_error)
        # Timed steady hold uses the compiled graph, without per-step host synchronizations.
        gpu.target.assign(np.tile(env.nominal_motor_position, (args.worlds, 1)).astype(np.float32))
        timings=[]
        for _ in range(args.repeats):
            wp.synchronize()
            started = time.perf_counter()
            for _ in range(args.steps):
                wp.capture_launch(graph)
            wp.synchronize()
            timings.append(time.perf_counter()-started)
            gpu.check()
        result['gpu_memory_used_mib'] = (wp.get_device().total_memory-wp.get_device().free_memory)/2**20
        result['gpu_pool_peak_mib'] = wp.get_mempool_used_mem_high()/2**20
        result['gpu_pool_used_mib'] = wp.get_mempool_used_mem_current()/2**20
        result['timing_s'] = timings
        result['transitions_per_second'] = args.worlds*args.steps/float(np.median(timings))
        result['single_cpu_advance_per_second'] = args.verify_steps/cpu_wall
        result['final_state'] = gpu.check()
        result['timed_steps_per_repeat'] = args.steps
        result['physics_substeps_per_transition'] = env.frame_skip
        print(json.dumps(result,indent=2),flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2)+'\n')


def cpu_worker(connection, steps, repeats):
    try:
        with QuadBotWalkEnv(robustness='none') as env:
            env.reset(seed=42, options=dict(command=[0, 0, 0], initial_temperature_c=40.))
            env._advance(env.nominal_motor_position)
            connection.send('ready')
            for _ in range(repeats):
                connection.recv()
                started = time.perf_counter()
                for _ in range(steps):
                    env._advance(env.nominal_motor_position)
                elapsed = time.perf_counter()-started
                if np.any(env.data.warning.number) or np.max(env.plant.temperature)>55:
                    raise RuntimeError('CPU benchmark trace failed')
                connection.send(dict(elapsed_s=elapsed, max_temperature_c=float(env.plant.temperature.max())))
    except Exception as error:
        connection.send(dict(error=str(error)))
        raise
    finally:
        connection.close()


def run_cpu(args):
    import multiprocessing as mp
    ctx = mp.get_context('spawn')
    workers, pipes = [], []
    try:
        for _ in range(args.worlds):
            parent, child = ctx.Pipe()
            process = ctx.Process(target=cpu_worker, args=(child, args.steps, args.repeats))
            process.start()
            child.close()
            workers.append(process)
            pipes.append(parent)
        for pipe in pipes:
            ready = pipe.recv()
            if ready != 'ready':
                raise RuntimeError(ready)
        times, rows = [], []
        for _ in range(args.repeats):
            started = time.perf_counter()
            for pipe in pipes:
                pipe.send('start')
            batch = [pipe.recv() for pipe in pipes]
            times.append(time.perf_counter()-started)
            if any('error' in row for row in batch):
                raise RuntimeError(batch)
            rows.append(batch)
        result = dict(backend='native_cpu', workers=args.worlds, timed_steps_per_repeat=args.steps,
                      timing_s=times, transitions_per_second=args.worlds*args.steps/float(np.median(times)),
                      scope='40 substeps + native NumPy voltage/thermal/energy plant + forward; no PPO/reward/reset/step IPC',
                      versions={p: importlib.metadata.version(p) for p in ('mujoco','numpy','gymnasium')},
                      worker_results=rows)
        print(json.dumps(result,indent=2),flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2)+'\n')
    finally:
        for pipe in pipes:
            pipe.close()
        for process in workers:
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', choices=('gpu','cpu'), default='gpu')
    p.add_argument('--worlds',type=int,default=1)
    p.add_argument('--nconmax',type=int,default=128)
    p.add_argument('--njmax',type=int,default=256)
    p.add_argument('--verify-steps',type=int,default=50)
    p.add_argument('--amplitude',type=float,default=.05)
    p.add_argument('--drop-height',type=float,default=0.)
    p.add_argument('--diverse',action='store_true')
    p.add_argument('--steps',type=int,default=100)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if min(args.worlds,args.nconmax,args.njmax,args.verify_steps,args.steps,args.repeats)<1:
        p.error('counts must be positive')
    (run if args.backend == "gpu" else run_cpu)(args)


if __name__=='__main__':
    main()
