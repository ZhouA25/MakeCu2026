"""The whole task in simulation, stage by stage, with the REAL control code.

  python -m pillbot.sim.demo                          one run, prints what happens at each stage
  python -m pillbot.sim.demo --video outputs/sim.gif  ... and record it (overview camera)
  python -m pillbot.sim.demo --episodes 20            success rate over 20 randomized bottles/caps
  python -m pillbot.sim.demo --approach ik            approach with inverse kinematics instead of the
                                                      trained policy (also used if no model is trained)

Stages (the whole task; --start clip skips 2-4 with the bottle already clamped):
  1. reach     the trained policy moves the grasp point to random points
  2. find      wrist camera scan of the pick spot -> where the bottle is (sim/perception.py)
  3. approach  the policy flies above the bottle
  4. pick+place  control/pick_place.pick_bottle / place_in_clip and control/fixture.Fixture.clamp —
               the code the prototype runs — grip the cap by feel, then (place.mode slide, default)
               lift a few mm and slide it into the open clamp on the flush clip, or (lift) carry it
               over and lower it in; clamp
  5. find      wrist camera -> cap position in the clip
  6. open      control/open_sequence.open_cap: grip by feel, push down, twist (wrist + arm together),
               regrip, lift, verify, drop the cap in the tray

The bottle position, cap tightness, child-proof unlock force and pad friction are randomized every
episode, so a high success rate means the method is robust, not tuned to one case.
"""
import argparse
import logging
import time
from pathlib import Path

import numpy as np

from pillbot.control.open_sequence import OpenFailed, open_cap, open_with_retry
from pillbot.sim.backends import make_sim
from pillbot.sim.envs import sample_reachable_goal
from pillbot.sim.kinematics import orbit_twist
from pillbot.sim.perception import locate_cap
from pillbot.sim.train import DEFAULT_MODEL, ReachPolicy

log = logging.getLogger("pillbot.sim.demo")
APPROACH_ABOVE = 0.045       # m above the cap centre: fingertips clear the cap top


def tool_roll(world) -> float:
    """Wrist roll (sim deg) for work at the bottle: the SO-101's hinged jaw opens sideways instead of
    toward the arm's base (where it would hit a riser / the base) — like a person would teach it."""
    return float(world.sim.get("tool_roll_deg", 90))


def cap_poses(world, io, cap_center) -> dict:
    """The open sequence's poses, computed from the DETECTED cap (instead of teaching them by hand)."""
    press = world.sim.get("press_depth_mm", 6) / 1000
    above = _highest(world, io, "grasp", cap_center, (APPROACH_ABOVE, 0.040, 0.035), "cap")
    return {"cap_above": above, "cap_grip": _pose_at(world, io, "grasp", cap_center, "cap"),
            "cap_press": _pose_at(world, io, "grasp", cap_center - [0, 0, press], "cap"),
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


def _pose_at(world, io, site: str, p, what: str) -> dict:
    roll = tool_roll(world)
    sol, err = world.ik(p, site=site, fixed={"wrist_roll": roll})
    if err > 0.002:
        raise OpenFailed(f"{what} at {np.round(p, 3)} is out of reach ({err * 1000:.0f} mm short)")
    return io.to_real(sol | {"wrist_roll": roll})


def _highest(world, io, site: str, p, heights, what: str) -> dict:
    """Pose with `site` the highest reachable of `heights` above `p` (the elbow limit caps it)."""
    for h in heights:
        if world.ik(np.asarray(p) + [0, 0, h], site=site, fixed={"wrist_roll": tool_roll(world)})[1] <= 0.002:
            return _pose_at(world, io, site, np.asarray(p) + [0, 0, h], what)
    raise OpenFailed(f"no room above the {what} at {np.round(p, 3)}")


def pick_place_poses(world, io, bottle_xy) -> dict:
    """The pick/place poses pick_place.py expects, from the DETECTED bottle and the known clip.

    fixture.pick_grip (sim.yaml): "cap" = carry the bottle by its cap (it's locked on until unscrewed;
    the only option with a palm pusher, which would land on the cap in a body grip), or "body" =
    `body_grip_below_top` under the body's top edge. The real code interpolates zone_* corners, so
    all four corners get the same pose here."""
    fx, b = world.sim["fixture"], world.bottle
    if fx.get("pick_grip", "cap") == "cap":
        # 3 mm above the cap's middle: the pusher must not press a free-standing bottle while the
        # jaws close (it tilts it, and the tilt is then locked into the grip); the pads still cover the cap
        site, grip_up = "grasp", b["height_mm"] / 1000 - fx["cap_height"] / 2 + 0.003
    else:
        site = "grasp_body"
        grip_up = b["height_mm"] / 1000 - fx["cap_height"] - fx.get("body_grip_below_top", 0.012)
    lift = (0.06, 0.055, 0.05, 0.045, 0.04)
    grasp = _pose_at(world, io, site, [bottle_xy[0], bottle_xy[1], grip_up], "bottle")
    hover = _highest(world, io, site, [bottle_xy[0], bottle_xy[1], grip_up], lift, "bottle")
    # Placing aims the bottle's BASE (it hangs in line with the fingers, which tilt more the higher the
    # arm reaches), so it comes down square over the clip instead of landing on a clamp jaw.
    base = "held_base_cap" if site == "grasp" else "held_base_body"
    cx, cy = world.clip_xy
    floor = fx["clip_floor"]
    jaws_top = floor + 0.044                                          # clamp jaws top + margin
    release_up = 0.004        # let go 4 mm up (1 mm was worse for slide: 5/10 not seated vs 2/10 —
                              # the IK error then pushes the base into the floor and shoves the bottle)
    poses = {f"zone_{kind}_{k}": pose for kind, pose in (("hover", hover), ("grasp", grasp))
             for k in ("00", "10", "11", "01")}
    poses |= {"clip_above": _highest(world, io, base, [cx, cy, jaws_top], (0.01, 0.005, 0.0), "clip"),
              "clip_insert": _pose_at(world, io, base, [cx, cy, floor + release_up], "clip")}
    return poses, site


def run_episode(seed: int, bottle: str, policy, grip_pressure: float | None, recorder=None,
                n_reach: int = 3, sim_cfg: dict | None = None, robot_cfg: dict | None = None,
                orbit: bool = True, start: str = "zone") -> dict:
    """start="zone": pick the bottle from the pick spot, put it in the clip, clamp, open (the whole
    task). start="clip": the bottle is already clamped in the clip (opening only)."""
    from pillbot.control.fixture import Fixture
    from pillbot.control.pick_place import clamp_and_check, pick_bottle, place_in_clip
    from pillbot.sim.backends import SimHub
    rng = np.random.default_rng(seed)
    world, io, tactile = make_sim(bottle, seed=seed, sim_cfg=sim_cfg, robot_cfg=robot_cfg)
    fx = world.sim["fixture"]
    clip = np.asarray(fx["clip_xy"], float)                       # the clip is bolted down: known
    pick = np.asarray(fx.get("pick_xy", (0.055, 0.095)), float)   # where the bottle should be set down
    world.reset(clip_xy=clip + rng.uniform(-0.003, 0.003, 2), start=start,
                pick_xy=pick + rng.uniform(-0.01, 0.01, 2))      # ...placed by hand: ±1 cm
    io.connect()
    world.on_tick = recorder
    gcfg = dict(world.robot_cfg["gripper"]) | {"open": world.sim.get("gripper_open", 95)}
    if grip_pressure:
        gcfg["target_pressure"] = grip_pressure
    retry = world.robot_cfg.get("retry", {})
    hub = SimHub(world)
    fixture = Fixture(hub, world.robot_cfg["fixture"], dry_run=True)   # the real clamp code
    out = {"seed": seed}

    def done(result):
        out["result"] = result
        cs = world.cap_state()
        log.info("  -> %s (cap turned %.0f deg, carried %.0f mm away; thread %.2f N·m, unlock %.1f mm)",
                 result, cs["turned_deg"], cs["moved_mm"], world.thread_torque, world.unlock_mm)
        world.close()
        return out

    # 1. reach random points
    errs = []
    for _ in range(n_reach):
        goal = sample_reachable_goal(world, rng)
        errs.append(move_policy_or_ik(world, io, policy, goal) * 1000)
    out["reach_mm"] = errs
    log.info("1 reach     %d random points, errors %s mm", n_reach, [round(e, 1) for e in errs])

    try:
        if start == "zone":
            # 2. find the bottle at the pick spot (wrist camera looks at its cap)
            fixture.unclamp()
            seen = locate_cap(world, io, nominal_xy=pick, surface_z=0.0)
            if seen is None:
                return done("bottle not seen")
            bxy = seen["cap_center"][:2]
            out["find_bottle_mm"] = float(np.linalg.norm(bxy - world.bottle_pos[:2]) * 1000)
            log.info("2 find      bottle at %s, true error %.1f mm", np.round(bxy, 3), out["find_bottle_mm"])
            # 3. approach with the policy, 4. pick + place + clamp with the real code
            poses, site = pick_place_poses(world, io, bxy)
            io.move_to({"gripper": gcfg["open"]})
            hover_pt = world.site_pose(io.to_sim(poses["zone_hover_00"]), site)[0]
            out["approach_mm"] = move_policy_or_ik(world, io, policy, hover_pt) * 1000   # policy flies the grasp site
            log.info("3 approach  above the bottle, error %.1f mm", out["approach_mm"])
            tactile.tare()
            if not pick_bottle(io, tactile, poses, gcfg, 0.5, 0.5, place=world.robot_cfg.get("place")):
                return done("failed: missed the bottle")
            place_in_clip(io, poses, gcfg, place=world.robot_cfg.get("place"))
            seated = clamp_and_check(io, tactile, fixture, poses, gcfg, fixture.bottle_seated,
                                     place=world.robot_cfg.get("place"), retries=int(retry.get("seat", 1)))
            out["seat_mm"] = float(np.linalg.norm(world.bottle_pos[:2] - world.clip_xy) * 1000)
            log.info("4 place     seated %s, %.1f mm off centre, tilt %.1f°, clamped",
                     fixture.bottle_seated(), out["seat_mm"], world.bottle_upright())
            if not seated:
                return done("failed: bottle not seated in the clip")

        # 5. find the cap in the clip (again before each open retry: the cap may have moved)
        def find_cap_poses():
            found = locate_cap(world, io, nominal_xy=world.clip_xy)
            if found is None:
                raise OpenFailed("cap not seen")
            cap = found["cap_center"]
            out["find_mm"] = float(np.linalg.norm(cap - world.cap_center) * 1000)
            log.info("5 find      cap at %s, true error %.1f mm", np.round(cap, 3), out["find_mm"])
            return cap_poses(world, io, cap)

        # 6. open with the real control code (retry a missed grip / slipping cap once)
        tactile.tare()
        twist = orbit_twist(world, io) if orbit else None
        stats = open_with_retry(
            lambda p: open_cap(io, tactile, world.bottle, p, gcfg, ocfg=world.robot_cfg.get("open"), twist=twist),
            io, gcfg, find_cap_poses, retries=int(retry.get("open", 1)))
        out.update(stats)
        removed = world.cap_state()["moved_mm"] > 30
        log.info("6 open      %s", "cap off and in the tray" if removed else "cap not removed")
        return done("opened" if removed else "cap not removed")
    except OpenFailed as e:
        return done(f"failed: {e}")


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
    ap.add_argument("--riser", type=float, help="block under the arm base, m (e.g. 0.08)")
    ap.add_argument("--recommended", action="store_true",
                    help="all of the sim's hardware recommendations: --pusher --strong-gripper --riser 0.08")
    ap.add_argument("--strong-gripper", action="store_true",
                    help="STS3215-class gripper servo (~3x an XL330's torque; what the SO-101 jaw is built for)")
    ap.add_argument("--wrist-only", action="store_true",
                    help="twist with wrist_roll alone (no arm compensation) to see why it fails")
    ap.add_argument("--start", choices=["zone", "clip"], default="zone",
                    help="zone: pick from the pick spot first (default); clip: bottle already clamped")
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

    robot_cfg = None
    if args.recommended:
        args.pusher, args.strong_gripper, args.riser = True, True, args.riser or 0.08
        from pillbot.utils.config import load_yaml
        robot_cfg = load_yaml("configs/robot.yaml")
        robot_cfg["tactile"]["slip_ratio"] = 0.3        # hinged jaw: pressure shifts a lot (docs/simulation.md)
    sim_cfg = None
    if args.pusher or args.strong_gripper or args.riser is not None:
        from pillbot.utils.config import load_yaml
        sim_cfg = load_yaml("configs/sim.yaml")
        if args.pusher:
            sim_cfg["arm"]["palm_pusher"] = True
        if args.strong_gripper:
            sim_cfg["arm"]["grip_torque_per_ma"] = 0.00089
        if args.riser is not None:
            sim_cfg["arm"]["base_riser"] = args.riser

    results, t0 = [], time.time()
    for ep in range(args.episodes):
        rec = GifRecorder() if args.video and ep == 0 else None
        r = run_episode(args.seed + ep, args.bottle, policy, args.grip_pressure or None, rec, sim_cfg=sim_cfg,
                        robot_cfg=robot_cfg,
                        orbit=not args.wrist_only, start=args.start)
        results.append(r)
        if rec:
            rec.save(args.video)
        if args.episodes > 1:
            print(f"episode {ep:3d}: {r['result'][:60]:<60s} seat {r.get('seat_mm', np.nan):4.1f} mm  "
                  f"cap find {r.get('find_mm', np.nan):4.1f} mm  twists {r.get('cycles', '-')}")
    ok = sum(r["result"] == "opened" for r in results)
    reach = np.concatenate([r["reach_mm"] for r in results])
    print(f"\n{ok}/{len(results)} opened  |  reach median {np.median(reach):.1f} mm  |  "
          f"find median {np.nanmedian([r.get('find_mm', np.nan) for r in results]):.1f} mm  |  "
          f"{time.time() - t0:.0f} s wall")


if __name__ == "__main__":
    main()
