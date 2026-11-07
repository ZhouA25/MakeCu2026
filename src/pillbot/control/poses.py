"""Pose helpers: load taught poses and interpolate inside the start zone (no IK required).

The zone is small (~20 cm), so bilinear interpolation of joint angles between the 4 taught
corner poses is a decent approximation. Shrink the zone if grasps drift in the middle.
"""
from pillbot.utils.config import load_yaml

REQUIRED = [
    "home", "scan",
    "zone_hover_00", "zone_hover_10", "zone_hover_11", "zone_hover_01",
    "zone_grasp_00", "zone_grasp_10", "zone_grasp_11", "zone_grasp_01",
    "clip_above", "clip_insert", "cap_above", "cap_grip", "cap_press", "cap_drop",
]


# Poses that depend on bottle height (cap higher on a taller bottle). Each can be taught per bottle
# as "<pose>@<bottle>", e.g. cap_grip@otc_vitamin; the plain name is the fallback for every bottle.
BOTTLE_POSES = [
    "zone_grasp_00", "zone_grasp_10", "zone_grasp_11", "zone_grasp_01",
    "clip_insert", "cap_above", "cap_grip", "cap_press",
]


def load_poses(path: str = "configs/poses.yaml", bottle: str | None = None) -> dict:
    """All poses; with `bottle`, any '<pose>@<bottle>' entries replace the generic ones."""
    raw = load_yaml(path)["poses"]
    poses = {k: v for k, v in raw.items() if "@" not in k}
    if bottle:
        for k, v in raw.items():
            if k.endswith(f"@{bottle}") and v and any(v.values()):
                poses[k.split("@")[0]] = v
    return poses


def bottle_specific(path: str = "configs/poses.yaml") -> dict[str, list[str]]:
    """{bottle: [poses taught specifically for it]}"""
    out: dict[str, list[str]] = {}
    for k in load_yaml(path)["poses"]:
        if "@" in k:
            pose, b = k.split("@", 1)
            out.setdefault(b, []).append(pose)
    return out


def lerp(a: dict, b: dict, t: float) -> dict:
    return {j: a[j] + (b[j] - a[j]) * t for j in a}


def zone_pose(poses: dict, kind: str, s: float, t: float) -> dict:
    """kind = 'hover' | 'grasp'; (s, t) in [0, 1]^2 = position inside the start zone."""
    s, t = min(max(s, 0.0), 1.0), min(max(t, 0.0), 1.0)
    p = {k: poses[f"zone_{kind}_{k}"] for k in ("00", "10", "11", "01")}
    near = lerp(p["00"], p["10"], s)   # edge t = 0
    far = lerp(p["01"], p["11"], s)    # edge t = 1
    return lerp(near, far, t)


def untaught(poses: dict) -> list[str]:
    """Poses still at the all-zero placeholder."""
    return [n for n in REQUIRED if n not in poses or not any(poses[n].values())]
