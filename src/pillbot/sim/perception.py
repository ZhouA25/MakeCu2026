"""Find the cap with the WRIST camera, the same way the real robot can.

1. Move to a scan pose: the camera ~11 cm above where the clip should be, looking down.
2. Compare the image with a background shot of the empty clip (perception.method: background on
   the real robot) and keep the bright, changed pixels: the cap's top.
3. Turn the blob's center pixel into a 3D point. The camera's position is known from the joint
   angles (forward kinematics), and the cap top's height is known from the bottle profile, so the
   pixel's viewing ray hits exactly one point at that height.

Only the joint angles, the camera image and the bottle height are used: nothing that only the
simulator knows. So this code carries over to the real robot once the camera is calibrated.
"""
import cv2
import numpy as np



def scan_pose(world, io, nominal_xy, top_z: float | None = None) -> dict:
    """Real-arm joint angles that AIM the wrist camera at the cap. With no wrist pitch the camera
    looks along the forearm, so "look at the cap from d cm" = "put the point d cm in front of the
    lens on the cap" — a plain IK problem. Tries the farthest view the arm can reach."""
    from pillbot.sim.scene import LOOK_CM
    target = [nominal_xy[0], nominal_xy[1], world.cap_top_z if top_z is None else top_z]
    for d in LOOK_CM:
        sol, err = world.ik(target, site=f"cam_look_{d}")
        if err < 0.002:
            return io.to_real(sol) | {"wrist_roll": 0}
    raise RuntimeError(f"wrist camera can't see a cap at {np.round(nominal_xy, 3)} — "
                       "see python -m pillbot.sim.workspace")


def find_cap_pixel(frame: np.ndarray, background: np.ndarray, diff_threshold: int = 30,
                   bright: int = 170, min_area: int = 150):
    """(u, v, mask) of the cap-top blob, or None. Cap = changed vs background AND bright/white."""
    diff = cv2.absdiff(frame, background).max(axis=2) > diff_threshold
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    white = (hsv[..., 2] > bright) & (hsv[..., 1] < 60)
    mask = (diff & white).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # close gaps (a printed label / stripe / glare line splits the cap top into pieces)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < min_area:
        return None
    return float(cents[k][0]), float(cents[k][1]), (labels == k)


def locate_cap(world, io, nominal_xy=None, looks: int = 2, surface_z: float | None = None) -> dict | None:
    """Scan with the wrist camera and return {"cap_top": xyz, "cap_center": xyz (where to grip),
    "pixel": (u, v), "image": rgb, "mask": bool image}, or None if no cap is seen.

    The 2nd look is centred on the 1st estimate: near the image edge the jaws can hide part of
    the cap and bias the blob's centre. `surface_z`: what the bottle stands on (default: the clip's
    floor; 0 = the table, i.e. the pick spot). With the bottle's height it gives the cap-top height."""
    surface = world.sim["fixture"]["clip_floor"] if surface_z is None else surface_z
    top_z = surface + world.bottle["height_mm"] / 1000
    xy = world.sim["fixture"]["clip_xy"] if nominal_xy is None else nominal_xy
    found = None
    for _ in range(looks):
        found = _look(world, io, xy, top_z) or found
        if found is None:
            return None
        xy = found["cap_top"][:2]
    return found


def _look(world, io, nominal_xy, top_z: float) -> dict | None:
    io.move_to(scan_pose(world, io, nominal_xy, top_z) | {"gripper": world.sim.get("gripper_open", 95)}, timeout=10)
    world.step_seconds(0.5)                               # let the arm settle (real: scan_settle_s)

    world.show_bottle(False)                              # real robot: background taken once, clip empty
    background = world.render("wrist")
    world.show_bottle(True)
    frame = world.render("wrist")
    hit = find_cap_pixel(frame, background)
    if hit is None:
        return None
    u, v, mask = hit
    origin, ray = world.camera_ray(u, v)
    z = top_z                                             # known: bottle profile + what it stands on
    if abs(ray[2]) < 1e-6:
        return None
    top = origin + ray * (z - origin[2]) / ray[2]
    center = top - [0, 0, world.sim["fixture"]["cap_height"] / 2]
    return {"cap_top": top, "cap_center": center, "pixel": (u, v), "image": frame, "mask": mask}
