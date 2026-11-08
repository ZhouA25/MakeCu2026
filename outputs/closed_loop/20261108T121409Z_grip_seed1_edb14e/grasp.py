"""Close the gripper until we feel the object, and detect missed grasps.

With a tactile sensor: close in small steps until pressure crosses the contact threshold,
then squeeze a little more. Without one: close fully and check whether the gripper stalled
early (= something is between the jaws).
"""
import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass
class GraspResult:
    success: bool
    gripper: float        # commanded gripper value at the end
    pressure: float       # tactile pressure (0 if no sensor)
    reason: str = ""


def grip_until_contact(io, tactile, gcfg: dict, extra_squeeze: float = 0.0) -> GraspResult:
    closed, step = gcfg["closed"], gcfg["step"]
    squeeze = gcfg["squeeze"] + extra_squeeze

    if not tactile.available:
        io.set_gripper(closed)
        actual = io.read()["gripper"]
        held = actual > closed + gcfg["stall_margin"]
        return GraspResult(held, closed, 0.0,
                           "stalled on object" if held else "closed fully: nothing in hand (missed)")

    target = io.target["gripper"]
    while target > closed:
        target = max(closed, target - step)
        io.set_gripper(target, settle_s=0.05)
        if tactile.in_contact():
            target = max(closed, target - squeeze)
            io.set_gripper(target, settle_s=0.2)
            # Optional: keep tightening until the pad reads a set grip pressure (gripper.target_pressure)
            want = gcfg.get("target_pressure")
            p = tactile.pressure()
            while want and p < want and target > closed:
                target = max(closed, target - step / 2)
                io.set_gripper(target, settle_s=0.1)
                p = tactile.pressure()
            log.info("grasp: contact at gripper=%.1f pressure=%.0f", target, p)
            threshold = max(float(getattr(tactile, "contact_threshold", 0)), float(want or 0))
            if p < threshold:
                return GraspResult(False, target, p, "contact lost or grip pressure not reached after squeezing")
            return GraspResult(True, target, p, "contact maintained after squeezing")
    return GraspResult(False, target, tactile.pressure(), "closed fully without contact (missed)")


def release(io, gcfg: dict):
    io.set_gripper(gcfg["open"])
