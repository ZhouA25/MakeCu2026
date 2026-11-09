"""Fix the gripper's calibrated range without recalibrating the whole arm (other joints untouched,
so taught poses don't shift). Nothing to unplug.

  python -m pillbot.tools.gripper_range          claw goes limp: open it FULLY and close it FULLY by hand,
                                                 then press Enter -> saved to the calibration file + servo
  python -m pillbot.tools.gripper_range --show   just print the current range, change nothing

The gripper's 0..100 (closed..open) is scaled across this range, and the servo's position limits are
set to it, so a range of a few steps means a claw that can't move.
"""
import argparse
import sys
import threading
import time

from pillbot.control.robot_io import make_robot
from pillbot.utils.config import load_yaml

MIN_SPAN = 200


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--show", action="store_true", help="print the saved range and the live position only")
    ap.add_argument("--config", default="configs/robot.yaml")
    args = ap.parse_args(argv)

    cfg = load_yaml(args.config)
    robot = make_robot(cfg["follower"])
    robot.connect(calibrate=False)                 # don't trigger LeRobot's full calibration prompt
    bus = robot.bus
    try:
        cal = robot.calibration.get("gripper")
        if cal is None:
            sys.exit("No gripper calibration yet — run the full calibration once (scripts/calibrate.sh).")
        print(f"saved gripper range: {cal.range_min}..{cal.range_max} ({cal.range_max - cal.range_min} steps)"
              f"   file: {robot.calibration_fpath}")
        if args.show:
            raw = bus.read("Present_Position", "gripper", normalize=False)
            print(f"live raw position: {raw}")
            return 0

        bus.disable_torque("gripper")
        print("\nGripper is limp. Open the claw FULLY and close it FULLY a couple of times,")
        print("END WITH IT CLOSED, then press Enter.\n")
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        lo = hi = int(bus.read("Present_Position", "gripper", normalize=False))
        while not done.is_set():
            raw = int(bus.read("Present_Position", "gripper", normalize=False))
            lo, hi = min(lo, raw), max(hi, raw)
            print(f"\r  now {raw:5d}   min {lo:5d}   max {hi:5d}   span {hi - lo:5d}   ", end="", flush=True)
            time.sleep(0.05)
        print()
        if hi - lo < MIN_SPAN:
            print(f"Span {hi - lo} is too small (need {MIN_SPAN}+): the claw didn't move. Nothing saved.")
            return 1
        closed_end = "low" if abs(raw - lo) < abs(raw - hi) else "high"
        cal.range_min, cal.range_max = lo, hi
        bus.write("Min_Position_Limit", "gripper", lo, normalize=False)   # EEPROM: torque is off
        bus.write("Max_Position_Limit", "gripper", hi, normalize=False)
        robot._save_calibration()
        print(f"saved gripper range {lo}..{hi} to the servo and {robot.calibration_fpath}")
        if closed_end == "high":
            print("NOTE: closing the claw RAISES its position, so the software's 'closed' (0) will open it.\n"
                  "      Tell Claude / check grasp direction before relying on close/open.")
        print("Test it:  python -m pillbot.tools.goto close open close")
        return 0
    finally:
        if not args.show and sys.stdin.isatty():
            input("Support the arm (torque turns off when this exits), then press Enter...")
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
