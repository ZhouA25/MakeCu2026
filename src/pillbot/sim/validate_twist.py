"""Record a press-and-twist trial with a bottle rigidly attached to ground."""
import argparse
from dataclasses import asdict
import json

import numpy as np

from pillbot.control.grasp import grip_until_contact
from pillbot.sim.backends import make_sim
from pillbot.sim.recording import RunRecorder
from pillbot.sim.validate_grip import move_feedback
from pillbot.utils.config import load_yaml


def trial(seed=0, twist_degrees=90., pusher=False):
    sim, robot = load_yaml('configs/sim.yaml'), load_yaml('configs/robot.yaml')
    sim['fixture']['fixed_to_ground'] = True
    sim['fixture']['clip_xy'] = [.18, .065]
    sim['arm']['palm_pusher'] = pusher
    robot['tactile']['contact_threshold'] = 100
    world, io, tactile = make_sim(seed=seed, sim_cfg=sim, robot_cfg=robot, start='clip')
    # No clamp is required under the user's rigid-ground assumption.
    world.set_clamp(False)
    recorder = RunRecorder(world, 'fixed_push_twist', seed)
    recorder.metadata['controller_inputs'] = 'joint encoders, tactile, simulated cap-compression feedback; known fixed target'
    result = {'mode':'fixed_push_twist', 'success':False, 'seed':seed, 'pusher':pusher,
              'commanded_twist_deg':twist_degrees, 'fixed_body_dofs':int(world.model.body_dofnum[world.bottle_body])}
    initial = world.bottle_pos.copy()
    try:
        target = world.cap_center.copy()
        q0, _ = world.ik(target, fixed={'wrist_roll':90})
        hover_q = q0 | {'shoulder_lift':q0['shoulder_lift']-18,
                       'elbow_flex':q0['elbow_flex']+18,'wrist_roll':90}
        hover, _ = world.fk(hover_q)
        recorder.phase = 'approach_open'
        io.set_gripper(75)
        tactile.tare()
        move_feedback(world, io, hover)
        recorder.phase = 'descend_to_cap'
        move_feedback(world, io, target)
        recorder.phase = 'grip_cap'
        gcfg = robot['gripper'] | {'open':75,'target_pressure':350,'squeeze':2}
        recorder.metadata['effective_gripper_config'] = gcfg
        grasp = grip_until_contact(io, tactile, gcfg)
        result['grasp'] = asdict(grasp)
        if not grasp.success:
            raise RuntimeError(grasp.reason)
        recorder.phase = 'push_down'
        # Instrumented simulation: regulate measured compression, not commanded Z.
        desired_press = min(2.7, world.unlock_mm + .35)
        depth = .004
        lateral = np.zeros(2)
        def press_tick(roll):
            nonlocal depth, lateral
            compression = world.cap_state()['pressed_mm']
            depth = float(np.clip(depth + .00006*(desired_press-compression), 0, .022))
            actual = world.fk(io.to_sim(io.read()))[0]
            lateral = np.clip(lateral + .06*(target[:2]-actual[:2]), -.015, .015)
            commanded = target + [lateral[0],lateral[1],-depth]
            q, residual = world.ik(commanded, seeds=[io.to_sim(io.read())],fixed={'wrist_roll':roll})
            io.step_toward(io.to_real(q | {'wrist_roll':roll}))
            world.tick()
        stable = 0
        for _ in range(240):
            press_tick(90.)
            stable = stable+1 if world.cap_state()['pressed_mm'] >= world.unlock_mm else 0
            if stable>=15: break
        result['cap_after_press'] = world.cap_state()
        if stable<15:
            raise RuntimeError('Cap could not be kept unlocked within the existing actuator limits')
        recorder.phase = 'push_and_twist'
        cap_start = world.cap_state()['turned_deg']
        samples=[]
        for roll in np.linspace(90.,90.-twist_degrees,int(twist_degrees*2)+1):
            press_tick(float(roll))
            samples.append(world.cap_state())
        for _ in range(15): press_tick(90.-twist_degrees)
        result['cap_after_twist'] = world.cap_state()
        result['actual_wrist_deg'] = world.read()['wrist_roll']
        result['actual_cap_twist_deg'] = world.cap_state()['turned_deg']-cap_start
        result['unlocked_fraction_during_twist'] = float(np.mean([s['pressed_mm']>=world.unlock_mm for s in samples]))
        result['max_cap_press_mm'] = max(s['pressed_mm'] for s in samples)
        result['max_bottle_displacement_mm'] = float(np.linalg.norm(world.bottle_pos-initial)*1000)
        result['success'] = bool(result['actual_cap_twist_deg']>=.7*twist_degrees and result['unlocked_fraction_during_twist']>=.8)
        recorder.phase = 'release_cap'
        io.set_gripper(75)
        recorder.phase = 'retreat'
        move_feedback(world, io, hover)
        world.step_seconds(.4)
    except Exception as error:
        result['success'] = False
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        recorder.phase = 'pass' if result['success'] else 'failed'
        result['final_cap_state'] = world.cap_state()
        result['bottle_displacement_mm'] = float(np.linalg.norm(world.bottle_pos-initial)*1000)
        try:
            path=recorder.finish(result)
        finally:
            world.close()
    print(json.dumps({'path':str(path),**result}),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--twist-degrees',type=float,default=90)
    parser.add_argument('--pusher',action='store_true')
    args=parser.parse_args()
    if not 0 < args.twist_degrees <= 180:
        parser.error('--twist-degrees must be greater than 0 and at most 180 for a single stroke')
    if not trial(args.seed,args.twist_degrees,args.pusher)['success']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
