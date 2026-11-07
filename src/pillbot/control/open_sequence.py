"""Scripted cap opening, bottle already in the clip:
grip cap -> (press down) -> twist CCW -> release -> rotate wrist back -> repeat -> lift cap off.

Tactile (if available) is used to grip by feel and to detect the cap slipping in the jaws;
a slipped twist doesn't count and the next grip squeezes harder.

Run alone:  python -m pillbot.control.open_sequence --bottle standard_amber_40dr --dry-run
"""
import argparse
import logging

from pillbot.control.grasp import grip_until_contact, release
from pillbot.control.poses import load_poses
from pillbot.control.robot_io import RobotIO
from pillbot.sensors.tactile import make_tactile
from pillbot.utils.config import get_bottle, load_yaml

log = logging.getLogger(__name__)

TWIST_PER_GRIP = 90.0   # deg of wrist_roll per grip cycle; raise if your wrist range allows
MAX_SLIPS = 3


class OpenFailed(RuntimeError):
    pass


def open_cap(io, tactile, bottle: dict, poses: dict, gcfg: dict, monitor=None) -> dict:
    """Returns stats. `monitor` (optional) is a BottleSpinMonitor from perception."""
    push = bottle["cap_type"] == "push_and_turn"
    io.move_to(poses["cap_above"] | {"gripper": gcfg["open"]})
    io.move_to(poses["cap_grip"])

    turned, slips, cycles, extra = 0.0, 0, 0, 0.0
    while turned < bottle["twist_deg"]:
        cycles += 1
        g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
        if not g.success:
            raise OpenFailed(f"could not grip cap: {g.reason}")
        if push:
            io.move_to(poses["cap_press"])                # push down while twisting
        step = min(TWIST_PER_GRIP, bottle["twist_deg"] - turned)
        roll0 = io.target["wrist_roll"]
        io.move_to({"wrist_roll": roll0 - step})          # CCW = loosen (flip sign if your wrist is mirrored)

        slipped = tactile.is_slipping(g.pressure)
        if monitor is not None and monitor.bottle_spun():
            raise OpenFailed("bottle is spinning in the clip — tighten/line the clip")

        release(io, gcfg)
        io.move_to(poses["cap_grip"] | {"wrist_roll": roll0})   # back up + unwind the wrist
        if slipped:
            slips += 1
            extra += gcfg["squeeze"]
            log.warning("open: cap slipped (%d/%d), squeezing harder", slips, MAX_SLIPS)
            if slips >= MAX_SLIPS:
                raise OpenFailed("cap keeps slipping — check pads / grip force")
            continue
        turned += step
        log.info("open: cycle %d, turned %.0f/%.0f deg", cycles, turned, bottle["twist_deg"])

    # Cap should be free: grip, lift off, drop in tray
    g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
    io.move_to(poses["cap_above"])
    io.move_to(poses["cap_drop"])
    release(io, gcfg)
    return {"cycles": cycles, "slips": slips, "turned_deg": turned, "cap_lifted": g.success}


def open_cap_turntable(io, tactile, bottle: dict, poses: dict, gcfg: dict, fixture) -> dict:
    """Arm grips (+ presses) the cap and holds still; the turntable spins the bottle to unscrew it.

    Each pass: pre-wind turntable (cap NOT held) -> grip cap -> press -> loosen up to ~300° -> release.
    """
    push = bottle["cap_type"] == "push_and_turn"
    fixture.clamp()
    io.move_to(poses["cap_above"] | {"gripper": gcfg["open"]})

    remaining, passes, slips, extra = float(bottle["twist_deg"]), 0, 0, 0.0
    while remaining > 0:
        passes += 1
        fixture.prewind()                                   # only while the cap is free
        io.move_to(poses["cap_grip"])
        g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
        if not g.success:
            raise OpenFailed(f"could not grip cap: {g.reason}")
        if push:
            io.move_to(poses["cap_press"])
        stroke = min(remaining, fixture.loosen_stroke())
        fixture.loosen(stroke)
        slipped = tactile.is_slipping(g.pressure)
        release(io, gcfg)
        io.move_to(poses["cap_above"])
        if slipped:
            slips += 1
            extra += gcfg["squeeze"]
            log.warning("open: cap slipped in jaws (%d/%d)", slips, MAX_SLIPS)
            if slips >= MAX_SLIPS:
                raise OpenFailed("cap keeps slipping — check pads / grip force")
            continue
        remaining -= stroke
        log.info("open: pass %d, %.0f deg left", passes, remaining)

    io.move_to(poses["cap_grip"])
    g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
    io.move_to(poses["cap_above"])
    io.move_to(poses["cap_drop"])
    release(io, gcfg)
    fixture.home()
    fixture.unclamp()
    return {"passes": passes, "slips": slips, "turned_deg": float(bottle["twist_deg"]) - remaining,
            "cap_lifted": g.success}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="configs/robot.yaml")
    p.add_argument("--bottle", default="standard_amber_40dr")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    cfg = load_yaml(args.config)
    io = RobotIO.from_config(cfg, dry_run=args.dry_run)
    if args.dry_run:
        io.sim_gripper_block = 15.0
    tactile = make_tactile(cfg, io, dry_run=args.dry_run)
    io.connect()
    tactile.connect()
    try:
        poses = load_poses()
        print(open_cap(io, tactile, get_bottle(args.bottle), poses, cfg["gripper"]))
        io.move_to(poses["home"])
    finally:
        tactile.close()
        io.disconnect()


if __name__ == "__main__":
    main()
