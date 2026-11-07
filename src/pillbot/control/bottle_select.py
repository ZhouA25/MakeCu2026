"""Pick a bottle profile from configs/bottles.yaml — by name, or automatically from the measured width."""
from pillbot.utils.config import load_yaml


def bottles() -> list[dict]:
    return load_yaml("configs/bottles.yaml")["bottles"]


def match_bottle(width_mm: float | None, tolerance_mm: float = 12.0) -> dict | None:
    """Closest profile by body diameter, if within tolerance."""
    if not width_mm:
        return None
    best = min(bottles(), key=lambda b: abs(b["body_diameter_mm"] - width_mm))
    return best if abs(best["body_diameter_mm"] - width_mm) <= tolerance_mm else None


def check_fits(bottle: dict, gcfg: dict) -> str | None:
    """Warning text if the cap is wider than the gripper can open."""
    mx = gcfg.get("max_cap_mm")
    if mx and bottle.get("cap_diameter_mm", 0) > mx:
        return (f"{bottle['name']}: cap {bottle['cap_diameter_mm']} mm is wider than the gripper's "
                f"max_cap_mm {mx} — print wider jaws or skip this bottle")
    return None
