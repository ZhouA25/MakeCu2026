"""Quick motion demo: no poses, camera or hub needed. Starts from wherever the arm is now,
lifts ONCE, then stays up and repeats until Ctrl-C:

  (once) lift up  ->  [ claw open / shut  ->  twist wrist one way, back, the other way, back ]  x forever

It never lowers: past a certain angle the shoulder/elbow run out of torque, so the arm stays up.

  python -m pillbot.tools.demo_moves                  repeat until Ctrl-C
  python -m pillbot.tools.demo_moves --once           one cycle
  python -m pillbot.tools.demo_moves --lift 0         don't lift, just claw + twist where it is
  python -m pillbot.tools.demo_moves --lift -60       lift the other way (if + went down)
  python -m pillbot.tools.demo_moves --twist 60 --speed 0.3
  python -m pillbot.tools.demo_moves --dry-run        no hardware

Angles are servo degrees (the shoulder has a 4:1 gearbox: --lift 60 is ~15 degrees of arm).
The claw goes to 95 (open) and 5 (shut), short of its hard stops so it never sits pushing on one at
full current (that trips the servo's overload protection). Ctrl-C stops; the arm holds where it is
until you support it and press Enter.
"""
import argparse
import logging
import sys
import time

from pillbot.control.robot_io import RobotIO
from pillbot.utils.config import load_yaml

log = logging.getLogger("demo")


def lift_step(start: dict, lift: float) -> tuple[str, dict] | None:
    if not lift or "shoulder_lift" not in start:
        return None
    return ("up", {"shoulder_lift": start["shoulder_lift"] + lift})


def demo_steps(start: dict, twist: float, claw_open: float = 95.0, claw_shut: float = 5.0,
               claws: int = 2) -> list[tuple[str, dict]]:
    """One repeating cycle at the top: `claws` x (open, shut), then twist +/- and back. No lowering."""
    steps = []
    for _ in range(claws):
        steps += [("claw OPEN", {"gripper": claw_open}), ("claw SHUT", {"gripper": claw_shut})]
    if twist and "wrist_roll" in start:
        r = start["wrist_roll"]
        steps += [("twist +", {"wrist_roll": r + twist}), ("twist back", {"wrist_roll": r}),
                  ("twist -", {"wrist_roll": r - twist}), ("twist back", {"wrist_roll": r})]
    return steps


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lift", type=float, default=60.0, help="shoulder_lift change, servo deg (default 60)")
    ap.add_argument("--claws", type=int, default=2, help="open/shut cycles per loop (default 2)")
    ap.add_argument("--twist", type=float, default=45.0, help="wrist twist each way, deg (default 45; 0 = none)")
    ap.add_argument("--speed", type=float, default=0.5, help="fraction of normal speed (default 0.5)")
    ap.add_argument("--pause", type=float, default=0.5, help="seconds between moves")
    ap.add_argument("--once", action="store_true", help="one cycle instead of repeating")
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    cfg = load_yaml(args.config)
    io = RobotIO.from_config(cfg, dry_run=args.dry_run)
    io.max_step *= max(0.05, min(args.speed, 1.0))
    log.info("connecting to the arm on %s ...", cfg["follower"].get("port"))
    io.connect()
    start = io.read()
    log.info("start: %s", {j: round(v, 1) for j, v in start.items()})
    up = lift_step(start, args.lift)
    steps = demo_steps(start, args.twist, claws=args.claws)
    cycle = 0
    def go(label, goal):
        log.info(label)
        far = max((abs(v - io.target[j]) for j, v in goal.items() if j != "gripper"), default=0.0)
        if io.move_to(goal, timeout=6.0 + 1.5 * far / max(io.max_step * io.hz, 1e-6)) is False:
            log.warning("  %s: didn't fully get there (blocked or too weak?)", label)
        if not args.dry_run:
            time.sleep(args.pause)

    try:
        if up:
            go(*up)                                  # once; the arm stays up from here on
        while True:
            cycle += 1
            for label, goal in steps:
                go(f"[{cycle}] {label}", goal)
            if args.once or args.dry_run:
                break
        log.info("done")
        if not args.dry_run and sys.stdin.isatty():
            input("Holding position. Support the arm, then press Enter to release torque...")
    except KeyboardInterrupt:
        log.warning("stopped")
        if not args.dry_run:
            input("Holding position. Support the arm, then press Enter to release torque...")
    finally:
        io.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
