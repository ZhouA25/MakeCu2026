"""The whole task in simulation, stage by stage, with the REAL control code.

  python -m pillbot.sim.demo                          one run, prints what happens at each stage
  python -m pillbot.sim.demo --video outputs/sim.gif  ... and record it (overview camera)
  python -m pillbot.sim.demo --episodes 20            success rate over 20 randomized bottles/caps
  python -m pillbot.sim.demo --approach ik            approach with inverse kinematics instead of the
                                                      trained policy (also used if no model is trained)

Stages:
  1. reach     the trained policy moves the grasp point to random points
  2. find      wrist camera scan -> cap position (sim/perception.py), checked against the truth
  3. approach  the policy flies to just above the detected cap
  4. open      control/open_sequence.open_cap (the code the prototype runs): grip by feel with the
               SensX pressure, push down, twist with the wrist, regrip, lift, verify, drop in the tray

The bottle position, cap tightness, child-proof unlock force and pad friction are randomized every
episode, so a high success rate means the method is robust, not tuned to one case.
"""
import argparse
import logging
import time
from pathlib import Path

import numpy as np

from pillbot.control.open_sequence import OpenFailed, open_cap
from pillbot.sim.backends import make_sim
from pillbot.sim.envs import sample_reachable_goal
from pillbot.sim.kinematics import orbit_twist
from pillbot.sim.perception import locate_cap
from pillbot.sim.train import DEFAULT_MODEL, ReachPolicy

log = logging.getLogger("pillbot.sim.demo")
APPROACH_ABOVE = 0.045       # m above the cap centre: fingertips clear the cap top


def cap_poses(world, io, cap_center) -> dict:
    """The open sequence's poses, computed from the DETECTED cap (instead of teaching them by hand)."""
    press = world.sim.get("press_depth_mm", 6) / 1000

    def pose(p):
        sol, err = world.ik(p)
        if err > 0.002:
            raise OpenFailed(f"cap at {np.round(p, 3)} is out of reach ({err * 1000:.0f} mm short)")
        return io.to_real(sol) | {"wrist_roll": 0}

    above = None
    for h in (APPROACH_ABOVE, 0.040, 0.035):          # near the base the elbow limit caps the height
        if world.ik(cap_center + [0, 0, h])[1] <= 0.002:
            above = pose(cap_center + [0, 0, h])
            break
    if above is None:
        raise OpenFailed(f"no room above the cap at {np.round(cap_center, 3)} — move the clip")
    return {"cap_above": above, "cap_grip": pose(cap_center), "cap_press": pose(cap_center - [0, 0, press]),
            "cap_drop": above | {"shoulder_pan": above["shoulder_pan"] + 70}}


def move_policy_or_ik(world, io, policy, goal) -> float:
    """Move the grasp point to `goal`; returns the remaining error (m)."""
    if policy is not None:
        err = policy.drive(world, goal)
        io._target = io.read() | io.to_real(world.targets())     # keep RobotIO's command in sync
        return err
    sol, _ = world.ik(goal)
    io.move_to(io.to_real(sol), timeout=10)
    return float(np.linalg.norm(world.grasp_pos() - goal))


def run_episode(seed: int, bottle: str, policy, grip_pressure: float | None, recorder=None,
                n_reach: int = 3, sim_cfg: dict | None = None, robot_cfg: dict | None = None,
                orbit: bool = True) -> dict:
    rng = np.random.default_rng(seed)
    world, io, tactile = make_sim(bottle, seed=seed, sim_cfg=sim_cfg, robot_cfg=robot_cfg)
    nominal = np.asarray(world.sim["fixture"]["clip_xy"], float)       # where the clip is supposed to be
    world.reset(clip_xy=nominal + rng.uniform(-0.015, 0.015, 2))      # ...and where it really is
    io.connect()
    world.on_tick = recorder
    out = {"seed": seed}

    # 1. reach random points
    errs = []
    for _ in range(n_reach):
        goal = sample_reachable_goal(world, rng)
        errs.append(move_policy_or_ik(world, io, policy, goal) * 1000)
    out["reach_mm"] = errs
    log.info("1 reach     %d random points, errors %s mm", n_reach, [round(e, 1) for e in errs])

    # 2. find the cap with the wrist camera
    found = locate_cap(world, io, nominal_xy=nominal)
    if found is None:
        out["result"] = "cap not seen"
        return out
    cap = found["cap_center"]
    out["find_mm"] = float(np.linalg.norm(cap - world.cap_center) * 1000)
    log.info("2 find      cap at %s, true error %.1f mm", np.round(cap, 3), out["find_mm"])

    # 3. approach: above the detected cap, jaws open
    io.move_to({"gripper": world.sim.get("gripper_open", 95)})
    above = cap + [0, 0, APPROACH_ABOVE]
    if world.ik(above)[1] > 0.002:
        above = cap + [0, 0, 0.035]
    out["approach_mm"] = move_policy_or_ik(world, io, policy, above) * 1000
    log.info("3 approach  above the cap, error %.1f mm", out["approach_mm"])

    # 4. open with the real control code
    gcfg = dict(world.robot_cfg["gripper"]) | {"open": world.sim.get("gripper_open", 95)}
    if grip_pressure:
        gcfg["target_pressure"] = grip_pressure
    tactile.tare()
    try:
        poses = cap_poses(world, io, cap)
        stats = open_cap(io, tactile, world.bottle, poses, gcfg, ocfg=world.robot_cfg.get("open"),
                         twist=orbit_twist(world, io) if orbit else None)
        removed = world.cap_state()["moved_mm"] > 30
        out.update(stats, result="opened" if removed else "cap not removed")
    except OpenFailed as e:
        out["result"] = f"failed: {e}"
    cs = world.cap_state()
    log.info("4 open      %s — cap turned %.0f deg, carried %.0f mm away (thread %.2f N·m, unlock %.1f mm)",
             out["result"], cs["turned_deg"], cs["moved_mm"], world.thread_torque, world.unlock_mm)
    world.close()
    return out


class GifRecorder:
    def __init__(self, every: int = 4, size=(360, 270)):
        self.frames, self.every, self.size, self.n = [], every, size, 0

    def __call__(self, world):
        self.n += 1
        if self.n % self.every == 0:
            self.frames.append(world.render("overview", *self.size))

    def save(self, path: Path, fps: int = 15):
        import imageio.v3 as iio
        path.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(path, np.stack(self.frames), duration=1000 / fps, loop=0)
        print(f"saved {path} ({len(self.frames)} frames)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--bottle", default="standard_amber_40dr")
    ap.add_argument("--approach", choices=["policy", "ik"], default="policy")
    ap.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    ap.add_argument("--grip-pressure", type=float, default=500,
                    help="gripper.target_pressure for the sim run (0 = contact + squeeze only)")
    ap.add_argument("--pusher", action="store_true",
                    help="give the hand the palm pusher (the design change the sim recommends)")
    ap.add_argument("--strong-gripper", action="store_true",
                    help="STS3215-class gripper servo (~3x an XL330's torque; what the SO-101 jaw is built for)")
    ap.add_argument("--wrist-only", action="store_true",
                    help="twist with wrist_roll alone (no arm compensation) to see why it fails")
    ap.add_argument("--video", type=Path, help="save a GIF of the (first) episode")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO if args.episodes == 1 else logging.WARNING,
                        format="%(message)s")

    policy = None
    if args.approach == "policy":
        if args.model.exists():
            policy = ReachPolicy(args.model)
        else:
            print(f"no trained policy at {args.model} — using IK. Train one: python -m pillbot.sim.train reach")

    sim_cfg = None
    if args.pusher or args.strong_gripper:
        from pillbot.utils.config import load_yaml
        sim_cfg = load_yaml("configs/sim.yaml")
        if args.pusher:
            sim_cfg["arm"]["palm_pusher"] = True
        if args.strong_gripper:
            sim_cfg["arm"]["grip_torque_per_ma"] = 0.00089

    results, t0 = [], time.time()
    for ep in range(args.episodes):
        rec = GifRecorder() if args.video and ep == 0 else None
        r = run_episode(args.seed + ep, args.bottle, policy, args.grip_pressure or None, rec, sim_cfg=sim_cfg,
                        orbit=not args.wrist_only)
        results.append(r)
        if rec:
            rec.save(args.video)
        if args.episodes > 1:
            print(f"episode {ep:3d}: {r['result']:<16s} find {r.get('find_mm', np.nan):4.1f} mm  "
                  f"approach {r.get('approach_mm', np.nan):4.1f} mm  twists {r.get('cycles', '-')}")
    ok = sum(r["result"] == "opened" for r in results)
    reach = np.concatenate([r["reach_mm"] for r in results])
    print(f"\n{ok}/{len(results)} opened  |  reach median {np.median(reach):.1f} mm  |  "
          f"find median {np.nanmedian([r.get('find_mm', np.nan) for r in results]):.1f} mm  |  "
          f"{time.time() - t0:.0f} s wall")


if __name__ == "__main__":
    main()
