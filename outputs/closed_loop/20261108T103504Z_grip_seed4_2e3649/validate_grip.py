"""Recorded feedback-controlled jaw and bottle-grip validation, simulation only.

Uses the real grip_until_contact and RobotIO interface. Bottle ground truth scores
the trial; motion control uses encoder-derived FK and a known bench target.
"""
import argparse
from dataclasses import asdict
import json
import math

import numpy as np

from pillbot.control.grasp import grip_until_contact
from pillbot.sim.backends import make_sim
from pillbot.sim.recording import RunRecorder
from pillbot.utils.config import load_yaml


def move_feedback(world, io, goal, timeout=14):
    """Rate-limited Cartesian reference plus bounded integral correction for servo sag."""
    goal = np.asarray(goal, float)
    start = world.time
    reference = world.fk(io.to_sim(io.read()))[0]
    correction = np.zeros(3)
    stable = 0
    while world.time - start < timeout:
        delta = goal - reference
        reference += delta * min(1., .001 / max(np.linalg.norm(delta), 1e-9))
        actual = world.fk(io.to_sim(io.read()))[0]
        error = reference - actual
        correction = np.clip(correction + .06 * error, -.015, .015)
        q, residual = world.ik(reference + correction, seeds=[io.to_sim(io.read())], fixed={'wrist_roll': 90})
        if residual > .003:
            raise RuntimeError(f'Unreachable Cartesian reference: {residual*1000:.1f} mm')
        io.step_toward(io.to_real(q | {'wrist_roll': 90}))
        world.tick()
        distance = np.linalg.norm(world.fk(io.to_sim(io.read()))[0] - goal)
        stable = stable + 1 if distance < .0015 else 0
        if stable >= 8:
            return float(distance)
    raise RuntimeError(f'Cartesian feedback timeout: {distance*1000:.1f} mm')


def trial(seed=0, mode='grip', hold_seconds=3.):
    sim = load_yaml('configs/sim.yaml')
    robot = load_yaml('configs/robot.yaml')
    # Simulated pad threshold: 100 summed kPa ~= 1.6 N on 4 mm taxels.
    # Hardware requires measured tactile calibration before adopting these values.
    robot['tactile']['contact_threshold'] = 100
    rng = np.random.default_rng(seed)
    # Corrected pivot spacing needs a ~19 cm reach for a vertical cap grasp.
    nominal_xy = np.array([.18, .065])
    actual_xy = nominal_xy + rng.uniform(-.0015, .0015, 2)
    sim['fixture']['pick_xy'] = actual_xy.tolist()
    sim['fixture']['bottle_mass'] = float(rng.uniform(.025, .045))
    world, io, tactile = make_sim(seed=seed, sim_cfg=sim, robot_cfg=robot, start='zone', pick_xy=actual_xy)
    rec = RunRecorder(world, mode, seed)
    result = {'success': False, 'mode': mode, 'hold_seconds': hold_seconds,
              'initial_xy_m': actual_xy.tolist(), 'benchmark': 'known bench location; vision not validated'}
    try:
        if mode == 'jaw':
            positions = []
            for value in [15, 85, 15, 85]:
                rec.phase = f'jaw_{value}'
                if not io.move_to({'gripper': value}, gripper_settle_s=.7):
                    raise RuntimeError('Jaw command timed out')
                positions.append(io.read()['gripper'])
            result['measured_jaw_units'] = positions
            result['measured_travel_deg'] = (max(positions)-min(positions)) * 1.1
            result['success'] = result['measured_travel_deg'] > 50 and all(abs(p-v)<5 for p,v in zip(positions,[15,85,15,85]))
        else:
            target = np.r_[nominal_xy + rng.uniform(-.0006, .0006, 2), .066]
            q_grasp, _ = world.ik(target, fixed={'wrist_roll':90})
            def arc_waypoint(dz):
                lo, hi = 0., 30.
                for _ in range(30):
                    delta = (lo + hi) / 2
                    q = q_grasp | {'shoulder_lift':q_grasp['shoulder_lift']-delta,
                                   'elbow_flex':q_grasp['elbow_flex']+delta, 'wrist_roll':90}
                    point, _ = world.fk(q)
                    if point[2] < target[2] + dz: lo = delta
                    else: hi = delta
                return point
            hover, lifted = arc_waypoint(.045), arc_waypoint(.025)
            rec.phase = 'open_jaw'
            io.set_gripper(75)
            tactile.tare()
            rec.phase = 'approach'
            result['hover_error_mm'] = move_feedback(world, io, hover) * 1000
            rec.phase = 'descend'
            result['grasp_error_mm'] = move_feedback(world, io, target) * 1000
            rec.phase = 'close_until_contact'
            gcfg = robot['gripper'] | {'open':75, 'target_pressure':150, 'squeeze':1}
            rec.metadata['effective_gripper_config'] = gcfg
            rec.save_manifest()
            grasp = grip_until_contact(io, tactile, gcfg)
            result['grasp'] = asdict(grasp)
            if not grasp.success:
                raise RuntimeError(grasp.reason)
            rec.phase = 'lift'
            result['lift_error_mm'] = move_feedback(world, io, lifted) * 1000
            rec.phase = 'hold_feedback'
            heights, pressures, tilts, relative = [], [], [], []
            for _ in range(round(hold_seconds * world.hz)):
                snap = tactile.snapshot()
                if snap.pressure < 150 and io.target['gripper'] > 0:
                    io.step_toward({'gripper': max(0, io.target['gripper'] - .3)})
                else:
                    io._send_target()
                world.tick()
                heights.append(float(world.bottle_pos[2]*1000))
                pressures.append(snap.pressure)
                tilts.append(world.bottle_upright())
                relative.append(world.bottle_pos - world.grasp_pos())
            drift = np.max(np.linalg.norm(np.asarray(relative)-relative[0],axis=1))*1000
            result.update(min_hold_height_mm=min(heights), max_hold_tilt_deg=max(tilts),
                          contact_fraction=float(np.mean(np.array(pressures)>tactile.contact_threshold)),
                          max_relative_drift_mm=float(drift))
            held = min(heights)>15 and max(tilts)<15 and result['contact_fraction']>=.9 and drift<5
            rec.phase = 'lower'
            move_feedback(world, io, target)
            rec.phase = 'release'
            io.set_gripper(75)
            rec.phase = 'retreat'
            move_feedback(world, io, hover)
            world.step_seconds(.5)
            result['final_bottle_height_mm'] = float(world.bottle_pos[2]*1000)
            result['final_bottle_tilt_deg'] = world.bottle_upright()
            result['success'] = bool(held and abs(result['final_bottle_height_mm'])<4 and result['final_bottle_tilt_deg']<15)
        rec.phase = 'pass' if result['success'] else 'fail'
        world.step_seconds(.4)
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        rec.phase = 'failed'
    finally:
        path = rec.finish(result)
        world.close()
    print(json.dumps({'path':str(path), **result}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['jaw','grip'], default='grip')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--episodes', type=int, default=1)
    args = parser.parse_args()
    results = [trial(args.seed + i, args.mode) for i in range(args.episodes)]
    if not all(r['success'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
