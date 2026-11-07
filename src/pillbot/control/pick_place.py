"""Scripted pick (from the start zone) and place (into the funnel clip)."""
import logging

from pillbot.control.grasp import grip_until_contact, release
from pillbot.control.poses import zone_pose

log = logging.getLogger(__name__)


def pick_bottle(io, tactile, poses: dict, gcfg: dict, s: float, t: float, retries: int = 1) -> bool:
    """Grasp the bottle body at zone position (s, t). Returns False if every attempt missed."""
    hover = zone_pose(poses, "hover", s, t)
    grasp = zone_pose(poses, "grasp", s, t)
    for attempt in range(retries + 1):
        io.move_to(hover | {"gripper": gcfg["open"]})
        io.move_to(grasp)
        res = grip_until_contact(io, tactile, gcfg)
        if res.success:
            io.move_to(hover)                     # lift (gripper target is kept)
            log.info("pick: got bottle (%s)", res.reason)
            return True
        log.warning("pick: attempt %d missed: %s", attempt + 1, res.reason)
        release(io, gcfg)
        io.move_to(hover)
    return False


def place_in_clip(io, poses: dict, gcfg: dict):
    """Lower the held bottle into the funnel clip and let go."""
    io.move_to(poses["clip_above"])
    io.move_to(poses["clip_insert"])
    release(io, gcfg)
    io.move_to(poses["clip_above"])
