"""Scripted cap opening, bottle clamped in the clip:
grip cap -> push down (child-proof caps) -> twist with wrist_roll -> release -> unwind wrist -> repeat
-> lift the cap off -> drop it in the tray.

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

DEFAULT_OPEN = {
    "twist_per_grip_deg": 90.0,   # wrist_roll per grip cycle; more = fewer regrips, but watch wrist cables
    "loosen_sign": -1,            # -1: decreasing wrist_roll unscrews (CCW seen from above). Flip if wrong
    "max_slips": 3,
    "max_extra_twists": 2,        # cap still threaded after twist_deg (small slips add up)? twist more
}


class OpenFailed(RuntimeError):
    pass


def open_cap(io, tactile, bottle: dict, poses: dict, gcfg: dict, monitor=None, ocfg: dict | None = None,
             twist=None) -> dict:
    """Returns stats. `monitor` (optional) is a BottleSpinMonitor from perception.
    `ocfg` = configs/robot.yaml `open:` section (twist_per_grip_deg, loosen_sign, max_slips).
    `twist(roll_deg)` (optional) turns the wrist to `roll_deg` while gripping. Default: wrist_roll only,
    which turns the cap about the WRIST axis. With the SO-101 gripper the cap's centre sits ~12 mm off
    that axis, so pass a coordinated twist that also moves the arm (sim/kinematics.py, docs/simulation.md)."""
    o = {**DEFAULT_OPEN, **(ocfg or {})}
    twist = twist or (lambda roll: io.move_to({"wrist_roll": roll}))
    per_grip, sign, max_slips = float(o["twist_per_grip_deg"]), (-1 if o["loosen_sign"] < 0 else 1), int(o["max_slips"])
    push = bottle["cap_type"] == "push_and_turn"
    io.move_to(poses["cap_above"] | {"gripper": gcfg["open"]})
    io.move_to(poses["cap_grip"])

    def grip_push_twist(step: float) -> bool:
        """One cycle: grip, push (child-proof), twist `step` degrees, release, unwind. True = slipped."""
        g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
        if not g.success:
            raise OpenFailed(f"could not grip cap: {g.reason}")
        if push:
            io.move_to(poses["cap_press"])                # push down while twisting
        roll0 = io.target["wrist_roll"]
        twist(roll0 + sign * step)                        # twist to loosen
        slipped = tactile.is_slipping(g.pressure)
        if monitor is not None and monitor.bottle_spun():
            raise OpenFailed("bottle is spinning in the clamp — tighten clamp_closed_us / add grippy liner")
        release(io, gcfg)
        io.move_to(poses["cap_grip"] | {"wrist_roll": roll0})   # back up + unwind the wrist
        return slipped

    turned, slips, cycles, extra = 0.0, 0, 0, 0.0
    while turned < bottle["twist_deg"]:
        cycles += 1
        step = min(per_grip, bottle["twist_deg"] - turned)
        slipped = grip_push_twist(step)
        if slipped:
            slips += 1
            extra += gcfg["squeeze"]
            log.warning("open: cap slipped (%d/%d), squeezing harder", slips, max_slips)
            if slips >= max_slips:
                raise OpenFailed("cap keeps slipping — check pads / grip force")
            continue
        turned += step
        log.info("open: cycle %d, turned %.0f/%.0f deg", cycles, turned, bottle["twist_deg"])

    # Cap should be free: grip and lift. `turned` counts what the WRIST did; small slips the tactile
    # check misses make the cap turn less, so confirm it really came off: the arm reached cap_above
    # (a still-threaded cap holds the arm down) and the pad still feels the cap (else the jaws slid off).
    extra_twists = 0
    while True:
        g = grip_until_contact(io, tactile, gcfg, extra_squeeze=extra)
        reached = io.move_to(poses["cap_above"])
        lifted = g.success and reached is not False and (not tactile.available or tactile.in_contact())
        if lifted or extra_twists >= int(o["max_extra_twists"]):
            break
        extra_twists += 1
        log.warning("open: cap still threaded after %.0f deg, extra twist %d/%d",
                    turned, extra_twists, o["max_extra_twists"])
        release(io, gcfg)
        io.move_to(poses["cap_grip"])
        grip_push_twist(per_grip)
        turned += per_grip
    if not lifted:
        release(io, gcfg)
        raise OpenFailed(f"cap still on after {turned:.0f} deg of twisting — raise twist_deg or grip harder")
    io.move_to(poses["cap_drop"])
    release(io, gcfg)
    return {"cycles": cycles + extra_twists, "slips": slips, "turned_deg": turned, "cap_lifted": lifted,
            "extra_twists": extra_twists}


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
        print(open_cap(io, tactile, get_bottle(args.bottle), poses, cfg["gripper"], ocfg=cfg.get("open")))
        io.move_to(poses["home"])
    finally:
        tactile.close()
        io.disconnect()


if __name__ == "__main__":
    main()
