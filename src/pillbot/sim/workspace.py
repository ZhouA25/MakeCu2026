"""Where can the arm grab a cap top-down? Checks configs/sim.yaml's arm against each bottle.

  python -m pillbot.sim.workspace                    every bottle, current base_riser
  python -m pillbot.sim.workspace --risers 0 0.06 0.12

The arm has no wrist pitch, so the fingers point wherever the forearm points. Grabbing a cap from
above needs the forearm near vertical, and that only happens at certain distances from the base for a
given cap height. This prints, per bottle, the clip distances where the cap can be gripped AND
approached from above, and how far from vertical the fingers are there.
"""
import argparse
import copy

import numpy as np

from pillbot.sim.world import SimWorld
from pillbot.utils.config import load_yaml

APPROACH_ABOVE = 0.035      # minimum hover above the cap centre for the fingertips to clear it


def check(sim: dict, bottle: dict, radii) -> list[tuple[float, float | None]]:
    """[(radius, tilt_deg or None if unreachable)] for a cap at each clip distance."""
    w = SimWorld(bottle, sim_cfg=sim)
    out = []
    for r in radii:
        w.reset(clip_xy=[r, 0.0], randomize=False)
        sol, err = w.ik(w.cap_center)
        above_err = w.ik(w.cap_center + [0, 0, APPROACH_ABOVE])[1]
        if err > 0.002 or above_err > 0.002:
            out.append((r, None))
            continue
        _, axis = w.fk(sol)
        out.append((r, float(np.degrees(np.arccos(np.clip(-axis[2], -1, 1))))))
    w.close()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--risers", type=float, nargs="*", help="base_riser heights to compare (m)")
    args = ap.parse_args()
    base = load_yaml("configs/sim.yaml")
    bottles = load_yaml("configs/bottles.yaml")["bottles"]
    radii = np.round(np.arange(0.06, 0.205, 0.02), 3)
    for riser in args.risers or [base["arm"]["base_riser"]]:
        sim = copy.deepcopy(base)
        sim["arm"]["base_riser"] = riser
        print(f"\nbase_riser = {riser * 100:.0f} cm   (cells: finger tilt from vertical at that clip distance, "
              f"'--' = can't grip from above)")
        print(f"{'bottle':22s}" + "".join(f"{r * 100:6.0f}cm" for r in radii))
        for b in bottles:
            row = check(sim, b, radii)
            print(f"{b['name']:22s}" + "".join(f"{t:7.0f}°" if t is not None else "     --" for _, t in row))
    print("\nAim for < ~15° tilt. Put the clip at that distance from the arm's base axis "
          "(configs/sim.yaml fixture.clip_xy), and measure the real arm into configs/sim.yaml first.")


if __name__ == "__main__":
    main()
