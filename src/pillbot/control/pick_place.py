"""Scripted pick (from the start zone) and place (into the clamp clip).

Two ways to get the bottle into the clip (configs/robot.yaml `place.mode`):
  slide (default)  the clip floor is flush with the table: grip the bottle, lift it only a few mm
                   ("skim"), slide it across into the OPEN clamp, set it down, let go. No carrying high,
                   so no swinging and little load on shoulder_lift; the self-centring rack-and-pinion
                   clamp then centres it. Put the zone and the clip the same distance from the arm's
                   base axis and the slide is mostly a shoulder_pan sweep.
  lift             lift to zone_hover, move to clip_above, lower to clip_insert, let go.

No extra poses to teach for slide: the skim pose is `place.skim` (default 0.06 = 6%) of the way from
the grasp pose toward the hover pose (zone_grasp -> zone_hover, clip_insert -> clip_above). The hover
poses sit straight above the grasp poses, so that's a lift of ~6% of the hover height (~3 mm at 5 cm).
"""
import logging

from pillbot.control.grasp import grip_until_contact, release
from pillbot.control.poses import zone_pose

log = logging.getLogger(__name__)

DEFAULT_PLACE = {"mode": "slide", "skim": 0.06}


def let_go(io, gcfg: dict, up_pose: dict):
    """Set the bottle free without flinging it: open the jaws only a crack (`gripper.release_crack`
    units), back straight up to `up_pose`, then open fully. Opening fully at once, the grippy pad on the
    SO-101's hinged jaw drags the cap sideways as it swings away (the sim lost ~3 cm this way)."""
    crack = float(gcfg.get("release_crack", 12))
    io.move_to({"gripper": min(gcfg["open"], io.target["gripper"] + crack)})
    io.move_to(up_pose)
    release(io, gcfg)


def place_cfg(cfg: dict | None) -> dict:
    p = {**DEFAULT_PLACE, **(cfg or {})}
    if p["mode"] not in ("slide", "lift"):
        raise ValueError(f"place.mode must be slide | lift, not {p['mode']!r}")
    if not 0 <= float(p["skim"]) < 1:
        raise ValueError("place.skim is a fraction of the way from grasp to hover (0 .. <1)")
    return p


def blend(lo: dict, hi: dict, frac: float) -> dict:
    """Arm joints `frac` of the way from pose `lo` to pose `hi` (the gripper is left alone)."""
    return {j: lo[j] + frac * (hi[j] - lo[j]) for j in lo if j in hi and j != "gripper"}


def pick_bottle(io, tactile, poses: dict, gcfg: dict, s: float, t: float, retries: int = 1,
                place: dict | None = None) -> bool:
    """Grasp the bottle body at zone position (s, t). Returns False if every attempt missed.
    Lifts to the hover pose (place.mode lift) or only to the skim height (slide)."""
    p = place_cfg(place)
    hover = zone_pose(poses, "hover", s, t)
    grasp = zone_pose(poses, "grasp", s, t)
    for attempt in range(retries + 1):
        io.move_to(hover | {"gripper": gcfg["open"]})
        io.move_to(grasp)
        res = grip_until_contact(io, tactile, gcfg)
        if res.success:
            io.move_to(blend(grasp, hover, float(p["skim"])) if p["mode"] == "slide" else hover)
            log.info("pick: got bottle (%s)", res.reason)
            return True
        log.warning("pick: attempt %d missed: %s", attempt + 1, res.reason)
        release(io, gcfg)
        io.move_to(hover)
    return False


def place_in_clip(io, poses: dict, gcfg: dict, place: dict | None = None):
    """Put the held bottle in the clip and let go. The clamp must be OPEN (Fixture.unclamp)."""
    p = place_cfg(place)
    if p["mode"] == "slide":
        skim = blend(poses["clip_insert"], poses["clip_above"], float(p["skim"]))
        # 1) still at the pick angle, move in/out to the clip's distance from the base (everything but
        #    shoulder_pan from the clip pose), 2) then only shoulder_pan sweeps it in. A straight blend
        #    from an off-radius pick spot comes in at an angle and catches a clamp jaw.
        io.move_to(skim | {"shoulder_pan": io.target["shoulder_pan"]})
        io.move_to(skim, timeout=10)
    else:
        io.move_to(poses["clip_above"])
    io.move_to(poses["clip_insert"])
    let_go(io, gcfg, poses["clip_above"])


def reseat(io, tactile, poses: dict, gcfg: dict, place: dict | None = None) -> bool:
    """The bottle didn't seat: grip it again where it stands in the clip (the clamp must be OPEN), lift
    it to skim height and set it down again. False if the gripper found nothing to hold."""
    p = place_cfg(place)
    io.move_to(poses["clip_above"] | {"gripper": gcfg["open"]})
    io.move_to(poses["clip_insert"])
    g = grip_until_contact(io, tactile, gcfg)
    if not g.success:
        log.warning("reseat: nothing to grip at the clip (%s)", g.reason)
        release(io, gcfg)
        io.move_to(poses["clip_above"])
        return False
    io.move_to(blend(poses["clip_insert"], poses["clip_above"], float(p["skim"])))
    io.move_to(poses["clip_insert"])
    let_go(io, gcfg, poses["clip_above"])
    return True


def clamp_and_check(io, tactile, fixture, poses: dict, gcfg: dict, seated, place: dict | None = None,
                    retries: int = 1) -> bool:
    """Clamp, then check the bottle is seated (`seated()` -> True / False / None = can't tell, accept).
    Not seated: unclamp, reseat, clamp again, up to `retries` times. Returns the final check."""
    for attempt in range(retries + 1):
        fixture.clamp()
        ok = seated()
        if ok is None or ok:
            return True
        if attempt == retries:
            break
        log.warning("place: bottle not seated, reseating (%d/%d)", attempt + 1, retries)
        fixture.unclamp()
        reseat(io, tactile, poses, gcfg, place)
    return False
