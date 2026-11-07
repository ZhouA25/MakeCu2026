"""Find a pill bottle in a camera frame by color (HSV).

Simple HSV color detection works well for amber bottles on a plain, contrasting mat.
Thresholds live in configs/robot.yaml -> perception: (tune them with --tune).
Upgrade to YOLO (`pip install -e .[vision]`) if you need many bottle types.

  python -m pillbot.perception.detect_bottle                    live preview (perception_camera)
  python -m pillbot.perception.detect_bottle --camera wrist     any camera from robot.yaml
  python -m pillbot.perception.detect_bottle --tune             sliders + mask view; press s to save
  python -m pillbot.perception.detect_bottle --image photo.jpg  test on a saved photo (e.g. from /capture)

Keys: q quit · s save thresholds (tune mode) · p save a snapshot to outputs/
"""
import argparse
import re
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from pillbot.utils.config import ROOT, load_yaml

# Defaults for amber pharmacy bottles — override in robot.yaml `perception:`
DEFAULTS = {"hsv_low": [5, 80, 60], "hsv_high": [25, 255, 255], "min_area": 1500}


@dataclass
class Detection:
    x: int
    y: int
    w: int
    h: int
    area: float = 0.0
    label: str = "bottle"

    @property
    def center_px(self) -> tuple[float, float]:
        return self.x + self.w / 2, self.y + self.h / 2

    @property
    def base_px(self) -> tuple[float, float]:
        """Bottom-center of the box ≈ where the bottle touches the table.
        With a ~45° camera the box center is too high, so use the base for zone mapping."""
        return self.x + self.w / 2, self.y + self.h


def thresholds(cfg: dict | None = None) -> dict:
    p = ((cfg or {}).get("perception") or {})
    return {k: p.get(k, v) for k, v in DEFAULTS.items()}


def color_mask(frame_bgr, low, high, open_k: int = 5):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low), np.array(high))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))


def detect_bottle(frame_bgr, th: dict | None = None) -> Detection | None:
    th = th or DEFAULTS
    mask = color_mask(frame_bgr, th["hsv_low"], th["hsv_high"])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= th["min_area"]]
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    return Detection(*cv2.boundingRect(best), area=float(cv2.contourArea(best)))


def detect_amber_bottle(frame_bgr, min_area: int = 1500) -> Detection | None:   # backwards compat
    return detect_bottle(frame_bgr, {**DEFAULTS, "min_area": min_area})


def locate_in_zone(frame_bgr, mapper, th: dict | None = None) -> tuple[float, float] | None:
    """Bottle position as zone coords (s, t), or None if not found / outside the zone."""
    det = detect_bottle(frame_bgr, th)
    if det is None:
        return None
    s, t = mapper.to_zone(*det.base_px)
    return (s, t) if mapper.inside(s, t) else None


def save_thresholds(th: dict, path: Path | None = None) -> Path:
    """Write/replace the `perception:` block in the active robot config, keeping everything else."""
    if path is None:
        local = ROOT / "configs/robot.local.yaml"
        path = local if local.exists() else ROOT / "configs/robot.yaml"
    text = path.read_text()
    block = ("perception:                # bottle color detection (tune: detect_bottle --tune)\n"
             f"  hsv_low: [{', '.join(str(int(v)) for v in th['hsv_low'])}]\n"
             f"  hsv_high: [{', '.join(str(int(v)) for v in th['hsv_high'])}]\n"
             f"  min_area: {int(th['min_area'])}\n")
    pat = re.compile(r"^perception:.*\n(?:[ \t]+.*\n|[ \t]*\n(?=[ \t]))*", re.M)
    text = pat.sub(block, text, count=1) if pat.search(text) else text.rstrip("\n") + "\n\n" + block
    path.write_text(text)
    return path


def annotate(frame, det: Detection | None, mapper=None, fps: float | None = None):
    if mapper:
        mapper.draw(frame)
    if det:
        cv2.rectangle(frame, (det.x, det.y), (det.x + det.w, det.y + det.h), (0, 255, 0), 2)
        bx, by = map(int, det.base_px)
        cv2.circle(frame, (bx, by), 6, (0, 255, 0), -1)
        txt = f"bottle {det.w}x{det.h}px"
        if mapper:
            s, t = mapper.to_zone(*det.base_px)
            txt += f"  s={s:.2f} t={t:.2f}" + ("" if mapper.inside(s, t) else "  (outside zone)")
        cv2.putText(frame, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "no bottle", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if fps:
        cv2.putText(frame, f"{fps:.0f} fps", (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--camera", help="camera name from robot.yaml (default: perception_camera)")
    ap.add_argument("--image", help="test on an image file instead of a camera")
    ap.add_argument("--tune", action="store_true", help="HSV sliders + mask window; s saves")
    args = ap.parse_args(argv)

    cfg = load_yaml(args.config)
    th = thresholds(cfg)
    try:
        from pillbot.perception.zone import ZoneMapper
        mapper = ZoneMapper.load()
    except FileNotFoundError:
        mapper = None

    cap, still = None, None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit(f"can't read {args.image}")
    else:
        from pillbot.perception.cameras import open_camera
        name = args.camera or cfg.get("perception_camera", "phone")
        cap = open_camera(cfg["cameras"][name])
        print(f"camera: {name}")

    if args.tune:
        cv2.namedWindow("tune")
        for i, ch in enumerate("HSV"):
            cv2.createTrackbar(f"{ch} low", "tune", int(th["hsv_low"][i]), 179 if ch == "H" else 255, lambda _: None)
            cv2.createTrackbar(f"{ch} high", "tune", int(th["hsv_high"][i]), 179 if ch == "H" else 255, lambda _: None)
        cv2.createTrackbar("min area", "tune", int(th["min_area"]), 20000, lambda _: None)
        print("Tune until ONLY the bottle is white in the mask. s = save, q = quit")

    t_prev, fps = time.time(), None
    while True:
        if still is not None:
            frame = still.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                print("camera read failed (stream busy? another viewer open?)")
                time.sleep(0.5)
                continue
            now = time.time()
            fps = 0.9 * (fps or 0) + 0.1 * (1 / max(now - t_prev, 1e-3)) if fps else 1 / max(now - t_prev, 1e-3)
            t_prev = now
        if args.tune:
            lo = [cv2.getTrackbarPos(f"{c} low", "tune") for c in "HSV"]
            hi = [cv2.getTrackbarPos(f"{c} high", "tune") for c in "HSV"]
            th = {"hsv_low": lo, "hsv_high": hi, "min_area": max(1, cv2.getTrackbarPos("min area", "tune"))}
            cv2.imshow("mask", color_mask(frame, lo, hi))
        det = detect_bottle(frame, th)
        cv2.imshow("bottle", annotate(frame, det, mapper, fps))
        key = cv2.waitKey(30 if still is not None else 1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("p"):
            out = ROOT / "outputs" / f"snapshot_{int(time.time())}.jpg"
            cv2.imwrite(str(out), frame)
            print(f"saved {out}")
        if key == ord("s") and args.tune:
            print(f"saved thresholds to {save_thresholds(th)}: {th}")
    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
