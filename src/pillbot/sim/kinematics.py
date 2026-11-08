"""Arm kinematics from the MuJoCo model — usable on the REAL robot too (mujoco runs on the Pi).

The SO-101 gripper holds a cap ~12 mm off the wrist_roll axis, so turning only the wrist swings the
cap around in a circle instead of unscrewing it. `orbit_twist` turns the wrist in small steps and, at
each step, re-solves shoulder_pan / shoulder_lift / elbow_flex so the grasp point stays exactly where it
was: the cap turns about its OWN axis.

    from pillbot.sim.kinematics import orbit_twist
    open_cap(io, tactile, bottle, poses, gcfg, twist=orbit_twist(world, io))

`world` only provides the kinematic model here (FK/IK). On the real arm, build a SimWorld for the
same bottle and pass the real RobotIO: joint angles go through configs/sim.yaml `joint_map`, so that
mapping must be calibrated first (docs/simulation.md checklist).
"""
import math

from pillbot.sim.world import REACH


def orbit_twist(world, io, step_deg: float = 3.0, timeout: float = 2.0):
    def twist(roll_to: float):
        start = io.to_sim(io.target)                    # commanded pose (keeps the push-down depth)
        hold, _ = world.fk(start)                       # grasp point to keep fixed
        roll_from = io.target["wrist_roll"]
        n = max(1, math.ceil(abs(roll_to - roll_from) / step_deg))
        q = {j: start[j] for j in REACH}
        for k in range(1, n + 1):
            roll = roll_from + (roll_to - roll_from) * k / n
            q, _ = world.ik(hold, seeds=[q], fixed={"wrist_roll": io.to_sim({"wrist_roll": roll})["wrist_roll"]})
            io.move_to(io.to_real(q) | {"wrist_roll": roll}, timeout=timeout)
    return twist
