"""Teach poses by hand: torque goes off, you move the arm, press Enter to save.

  python -m pillbot.control.teach --all             # guided walk through every required pose
  python -m pillbot.control.teach --name cap_press  # (re)teach one pose
  python -m pillbot.control.teach --missing         # only poses still at the placeholder
  python -m pillbot.control.teach --bottle otc_vitamin --bottle-poses
                                                    # height-dependent poses for ONE bottle type
  python -m pillbot.control.teach --bottle otc_vitamin --name cap_grip
"""
import argparse

import yaml

from pillbot.control.poses import BOTTLE_POSES, REQUIRED, load_poses, untaught
from pillbot.control.robot_io import RobotIO
from pillbot.utils.config import ROOT, load_yaml

HINTS = {
    "home": "safe resting pose, clear of everything",
    "scan": "wrist camera ~25-35 cm above the start zone, looking STRAIGHT DOWN, whole zone in view "
            "(check with: detect_bottle --camera wrist)",
    "zone_hover_00": "start-zone corner 1, jaws open ~5 cm ABOVE the cap (forearm vertical)",
    "zone_hover_10": "start-zone corner 2 (hover)",
    "zone_hover_11": "start-zone corner 3 (hover)",
    "zone_hover_01": "start-zone corner 4 (hover)",
    "zone_grasp_00": "corner 1, jaws open around the CAP, top-down (grasp height)",
    "zone_grasp_10": "corner 2 (grasp)",
    "zone_grasp_11": "corner 3 (grasp)",
    "zone_grasp_01": "corner 4 (grasp)",
    "clip_above": "holding a bottle by its cap, its base just above the funnel clip",
    "clip_insert": "bottle fully seated in the clip",
    "cap_above": "jaws open, ~3 cm above the cap of a clipped bottle",
    "cap_grip": "jaws around the cap",
    "cap_press": "same as cap_grip but pushed a few mm DOWN (this sets the push force)",
    "cap_drop": "above the cap tray",
}


def save_pose(name: str, pose: dict):
    path = ROOT / "configs/poses.yaml"
    data = yaml.safe_load(path.read_text())
    data["poses"][name] = pose
    path.write_text(yaml.safe_dump(data, sort_keys=False))


def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--name")
    g.add_argument("--all", action="store_true")
    g.add_argument("--missing", action="store_true")
    g.add_argument("--bottle-poses", action="store_true", help="the height-dependent poses (needs --bottle)")
    p.add_argument("--bottle", help="save as '<pose>@<bottle>' (bottle name from bottles.yaml)")
    p.add_argument("--config", default="configs/robot.yaml")
    args = p.parse_args()

    if args.bottle_poses and not args.bottle:
        p.error("--bottle-poses needs --bottle <name>")
    if args.bottle:
        from pillbot.utils.config import get_bottle
        get_bottle(args.bottle)                      # fail early on a typo
    names = ([args.name] if args.name else BOTTLE_POSES if args.bottle_poses
             else REQUIRED if args.all else untaught(load_poses()))
    suffix = f"@{args.bottle}" if args.bottle else ""
    if not names:
        print("All poses taught.")
        return

    io = RobotIO.from_config(load_yaml(args.config))
    io.connect()
    io.torque_off()
    try:
        for n in names:
            input(f"\n[{n}{suffix}] {HINTS.get(n, '')}\n  Move the arm there, then press Enter...")
            pose = {k: round(v, 1) for k, v in io.read().items() if k != "gripper"}
            save_pose(n + suffix, pose)
            print(f"  saved {n + suffix}: {pose}")
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()
