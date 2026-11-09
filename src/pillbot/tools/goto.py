"""Move the arm to taught poses, no camera, hub or tray needed. Good for checking poses and for demos.

  python -m pillbot.tools.goto --list                          what's taught
  python -m pillbot.tools.goto cap_grip                        go to one pose and hold it
  python -m pillbot.tools.goto home cap_above cap_grip close cap_press twist-90 open cap_above home
  python -m pillbot.tools.goto --step home cap_above cap_grip  press Enter before each move
  python -m pillbot.tools.goto --slow ...                      half speed (or --speed 0.25)

Steps are pose names from configs/poses.yaml, or these gripper / wrist commands:
  open | close         gripper open / closed (configs/robot.yaml gripper.open / .closed)
  grip=40              gripper to 40 (0 = closed, 100 = open)
  twist-90 | twist+45  turn wrist_roll by that many degrees from where it is
  wait=1.5             pause (seconds)
Ctrl-C stops; the arm then holds where it is until you press Enter (support it first).
"""
import argparse
import logging
import sys
import time

from pillbot.control.poses import is_taught, load_poses
from pillbot.control.robot_io import RobotIO
from pillbot.utils.config import load_yaml

log = logging.getLogger(__name__)


def parse_step(step: str, poses: dict) -> tuple[str, object]:
    """('pose', name) | ('grip', value|'open'|'close') | ('twist', deg) | ('wait', s). ValueError if unknown."""
    if step in ("open", "close"):
        return "grip", step
    if step.startswith("grip="):
        return "grip", float(step[5:])
    if step.startswith("twist") and step[5:6] in "+-" and step[5:]:
        return "twist", float(step[5:])
    if step.startswith("wait="):
        return "wait", float(step[5:])
    if step in poses:
        if not is_taught(poses[step]):
            raise ValueError(f"pose '{step}' isn't taught yet: python -m pillbot.control.teach --name {step}")
        return "pose", step
    raise ValueError(f"unknown step '{step}' (not a pose in configs/poses.yaml, nor open/close/grip=/twist±/wait=)")


def _timeout(io, goal: dict) -> float:
    """Long enough for the ramp at the current speed (the shoulder's 4:1 gearing means big angles)."""
    far = max((abs(v - io.target.get(j, v)) for j, v in goal.items() if j in io.joints), default=0.0)
    return 6.0 + 1.5 * far / max(io.max_step * io.hz, 1e-6)


def run_steps(io, steps: list[tuple[str, object]], poses: dict, gcfg: dict, step_mode: bool = False):
    for kind, arg in steps:
        label = f"{kind} {arg}"
        if step_mode:
            input(f"next: {label}  [Enter]")
        log.info(label)
        if kind == "pose":
            if io.move_to(poses[arg], timeout=_timeout(io, poses[arg])) is False:
                log.warning("  didn't fully reach %s (blocked, or a joint too weak?)", arg)
        elif kind == "grip":
            io.set_gripper(gcfg["closed" if arg == "close" else "open"] if isinstance(arg, str) else arg)
        elif kind == "twist":
            io.move_to({"wrist_roll": io.target["wrist_roll"] + arg})
        elif kind == "wait":
            time.sleep(arg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("steps", nargs="*", help="pose names and gripper/wrist commands, in order")
    ap.add_argument("--list", action="store_true", help="list poses (taught or not) and exit")
    ap.add_argument("--step", action="store_true", help="press Enter before each step")
    ap.add_argument("--slow", action="store_true", help="half speed")
    ap.add_argument("--speed", type=float, help="speed factor vs safety.max_joint_step_deg (e.g. 0.25)")
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    poses = load_poses()
    if args.list or not args.steps:
        for name, p in poses.items():
            vals = ", ".join(f"{j} {v:g}" for j, v in p.items()) if is_taught(p) else "-- not taught --"
            print(f"  {name:15s} {vals}")
        if not args.steps:
            print("\nGive pose names to move, e.g.:  python -m pillbot.tools.goto home cap_above")
        return 0
    try:
        steps = [parse_step(s, poses) for s in args.steps]
    except ValueError as e:
        sys.exit(str(e))

    cfg = load_yaml(args.config)
    io = RobotIO.from_config(cfg, dry_run=args.dry_run)
    speed = args.speed or (0.5 if args.slow else 1.0)
    io.max_step *= max(0.05, min(speed, 1.0))
    io.connect()
    try:
        run_steps(io, steps, poses, cfg["gripper"], args.step)
        if not args.dry_run and sys.stdin.isatty():
            input("Done — holding position. Support the arm, then press Enter to release torque...")
    except KeyboardInterrupt:
        log.warning("stopped")
        if not args.dry_run:
            input("Holding position. Support the arm, then press Enter to release torque...")
    finally:
        io.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
