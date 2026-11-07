"""Find a bottle — any bottle — in a camera frame.

Methods (configs/robot.yaml -> perception.method):
  background  (default) Snapshot the EMPTY start zone once; anything that appears afterwards is the
              bottle. Works for amber, white, clear-ish, any shape. Needs a FIXED camera and steady
              light. Re-take the background whenever the camera, tape or lighting changes (key b).
  yolo        Pretrained YOLO "bottle" class (pip install -e ".[vision]"). No background needed;
              slower, and pill bottles are not always recognised — check on yours.
  color       HSV color range (amber by default). Fallback for one known bottle color.

  python -m pillbot.perception.detect_bottle                    live preview (perception_camera)
  python -m pillbot.perception.detect_bottle --method yolo      try another method
  python -m pillbot.perception.detect_bottle --camera wrist     any camera from robot.yaml
  python -m pillbot.perception.detect_bottle --tune             color/threshold sliders; s saves
  python -m pillbot.perception.detect_bottle --image photo.jpg  test on a saved photo
  python -m pillbot.perception.detect_bottle --goto-scan        arm moves to `scan` first (wrist cam)

Keys: q quit · b capture background (clear the zone first!) · s save settings (--tune) · p snapshot
"""
import argparse
import logging
import re
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from pillbot.utils.config import ROOT, load_yaml

BACKGROUND_PATH = ROOT / "configs/background.local.png"
DEFAULTS = {
    "method": "background",
    "min_area": 400,              # px at QVGA (320x240); ~1500 at VGA
    "diff_threshold": 30,         # background: how different a pixel must be to count as "object"
    "hsv_low": [5, 80, 60],       # color: amber pharmacy bottle
    "hsv_high": [25, 255, 255],
    "yolo_model": "yolo11n.pt",
    "yolo_conf": 0.25,
    "zone_size_mm": 200,          # side of the taped start-zone square -> bottle width in mm
    "align": True,                # background: undo small camera shifts (arm backlash) first
    "camera_on_arm": False,
    "anchor": "auto",             # auto (center if camera_on_arm else base) | center | base
    "scan_settle_s": 0.8,
}
COCO_BOTTLE = 39
log = logging.getLogger(__name__)


@dataclass
class Detection:
    x: int
    y: int
    w: int
    h: int
    area: float = 0.0
    label: str = "bottle"
    score: float = 1.0
    cx: float | None = None        # blob centroid (more accurate than the box centre for odd shapes)
    cy: float | None = None

    @property
    def center_px(self) -> tuple[float, float]:
        if self.cx is not None:
            return self.cx, self.cy
        return self.x + self.w / 2, self.y + self.h / 2

    def anchor_px(self, mode: str = "center") -> tuple[float, float]:
        """Where the bottle 'is' in the image.
        center: seen from above (wrist camera at the scan pose) -> the middle of the blob = the cap.
        base:   seen at an angle (fixed camera ~45°) -> bottom of the box = where it meets the table."""
        return self.base_px if mode == "base" else self.center_px

    @property
    def base_px(self) -> tuple[float, float]:
        """Bottom-center of the box ≈ where the bottle touches the table (use for zone mapping)."""
        return self.x + self.w / 2, self.y + self.h

    @property
    def base_edges_px(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Bottom-left / bottom-right corners — mapped through the zone they give the width in mm."""
        return (self.x, self.y + self.h), (self.x + self.w, self.y + self.h)


def settings(cfg: dict | None = None) -> dict:
    p = ((cfg or {}).get("perception") or {})
    st = {k: p.get(k, v) for k, v in DEFAULTS.items()}
    if st["anchor"] == "auto":
        st["anchor"] = "center" if st["camera_on_arm"] else "base"
    return st


thresholds = settings   # old name


def _largest_blob(mask, min_area, label) -> Detection | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    m = cv2.moments(best)
    cx, cy = (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] else (None, None)
    return Detection(*cv2.boundingRect(best), area=float(cv2.contourArea(best)), label=label, cx=cx, cy=cy)


# ------------------------------------------------------------------ color
def color_mask(frame_bgr, low, high, open_k: int = 5):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low), np.array(high))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))


class ColorDetector:
    name = "color"

    def __init__(self, st: dict):
        self.st = st

    def mask(self, frame):
        return color_mask(frame, self.st["hsv_low"], self.st["hsv_high"])

    def __call__(self, frame) -> Detection | None:
        return _largest_blob(self.mask(frame), self.st["min_area"], "bottle (color)")


# ------------------------------------------------------------------ background
class BackgroundDetector:
    name = "background"

    def __init__(self, st: dict, background=None, roi_mask=None):
        self.st = st
        self.background = background
        self.roi_mask = roi_mask           # optional: only look inside the start zone (+ margin)

    @classmethod
    def from_disk(cls, st, mapper=None):
        bg = cv2.imread(str(BACKGROUND_PATH)) if BACKGROUND_PATH.exists() else None
        return cls(st, bg, None)

    def capture(self, frames):
        """Median of several frames of the EMPTY scene (robust to noise / flicker)."""
        stack = np.stack([cv2.GaussianBlur(f, (5, 5), 0) for f in frames]).astype(np.float32)
        self.background = np.median(stack, axis=0).astype(np.uint8)
        return self.background

    def save(self):
        BACKGROUND_PATH.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(BACKGROUND_PATH), self.background)

    def aligned_background(self, frame):
        """Shift the background to line up with `frame` (the scan pose never repeats to the pixel)."""
        if not self.st.get("align", True):
            return self.background, (0.0, 0.0)
        g1 = cv2.cvtColor(self.background, cv2.COLOR_BGR2GRAY).astype(np.float32)
        g2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        (dx, dy), _ = cv2.phaseCorrelate(g1, g2)
        if abs(dx) > 40 or abs(dy) > 40:          # implausible: don't make things worse
            return self.background, (0.0, 0.0)
        m = np.float32([[1, 0, dx], [0, 1, dy]])
        bg = cv2.warpAffine(self.background, m, (frame.shape[1], frame.shape[0]),
                            borderMode=cv2.BORDER_REPLICATE)
        return bg, (dx, dy)

    def mask(self, frame):
        if self.background is None or self.background.shape != frame.shape:
            return np.zeros(frame.shape[:2], np.uint8)
        bg, _ = self.aligned_background(cv2.GaussianBlur(frame, (5, 5), 0))
        diff = cv2.absdiff(cv2.GaussianBlur(frame, (5, 5), 0), bg)
        d = diff.max(axis=2)                                   # biggest change in any channel
        _, m = cv2.threshold(d, int(self.st["diff_threshold"]), 255, cv2.THRESH_BINARY)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))   # fill a white bottle's holes
        if self.roi_mask is not None:
            m = cv2.bitwise_and(m, self.roi_mask)
        return m

    def __call__(self, frame) -> Detection | None:
        if self.background is None:
            return None
        if self.background.shape != frame.shape:
            log.warning("background is %s but frame is %s — re-take it (b)", self.background.shape[:2],
                        frame.shape[:2])
            return None
        return _largest_blob(self.mask(frame), self.st["min_area"], "bottle")


# ------------------------------------------------------------------ yolo
class YoloDetector:
    name = "yolo"

    def __init__(self, st: dict):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit('YOLO needs ultralytics: pip install -e ".[vision]"') from e
        self.st = st
        self.model = YOLO(st["yolo_model"])           # downloads the small model on first use

    def mask(self, frame):
        return None

    def __call__(self, frame) -> Detection | None:
        r = self.model(frame, classes=[COCO_BOTTLE], conf=float(self.st["yolo_conf"]), verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return None
        i = int(r.boxes.conf.argmax())
        x1, y1, x2, y2 = r.boxes.xyxy[i].tolist()
        return Detection(int(x1), int(y1), int(x2 - x1), int(y2 - y1), area=(x2 - x1) * (y2 - y1),
                         label="bottle (yolo)", score=float(r.boxes.conf[i]))


def make_detector(cfg: dict | None = None, method: str | None = None, mapper=None):
    st = settings(cfg)
    m = method or st["method"]
    if m == "background":
        return BackgroundDetector.from_disk(st, mapper)
    if m == "yolo":
        return YoloDetector(st)
    if m == "color":
        return ColorDetector(st)
    raise ValueError(f"unknown perception.method '{m}' (background | yolo | color)")


# ------------------------------------------------------------------ helpers used by run.py
def detect_bottle(frame_bgr, th: dict | None = None) -> Detection | None:
    """Color detection (kept for compatibility)."""
    return ColorDetector({**DEFAULTS, **(th or {})})(frame_bgr)


def detect_amber_bottle(frame_bgr, min_area: int = 1500) -> Detection | None:
    return detect_bottle(frame_bgr, {"min_area": min_area})


def locate_in_zone(frame_bgr, mapper, detector=None, anchor: str | None = None) -> tuple[float, float] | None:
    """Bottle position as zone coords (s, t), or None if not found / outside the zone."""
    if detector is None or isinstance(detector, dict):        # old call style: dict of thresholds
        detector = ColorDetector({**DEFAULTS, **(detector or {})})
    det = detector(frame_bgr)
    if det is None:
        return None
    anchor = anchor or getattr(detector, "st", {}).get("anchor", "base")
    s, t = mapper.to_zone(*det.anchor_px("base" if anchor == "auto" else anchor))
    return (s, t) if mapper.inside(s, t) else None


def width_mm(det: Detection, mapper, zone_size_mm: float, anchor: str = "base") -> float | None:
    """Bottle width from the zone calibration (zone corners = known square), measured across the
    blob at the anchor row (base row for an angled camera, centre row for a top-down one)."""
    if mapper is None or not zone_size_mm:
        return None
    if anchor == "base":
        (ax, ay), (bx, by) = det.base_edges_px
    else:
        yc = det.center_px[1]
        (ax, ay), (bx, by) = (det.x, yc), (det.x + det.w, yc)
    s1, t1 = mapper.to_zone(ax, ay)
    s2, t2 = mapper.to_zone(bx, by)
    return float(np.hypot(s2 - s1, t2 - t1) * zone_size_mm)


def save_settings(values: dict, path: Path | None = None) -> Path:
    """Write/replace the `perception:` block in the active robot config, keeping everything else."""
    if path is None:
        local = ROOT / "configs/robot.local.yaml"
        path = local if local.exists() else ROOT / "configs/robot.yaml"
    text = path.read_text()
    cur = (load_yaml(path).get("perception") or {}) if path.exists() else {}
    merged = {**DEFAULTS, **cur, **values}
    fmt = lambda v: f"[{', '.join(str(int(x)) for x in v)}]" if isinstance(v, (list, tuple)) else v
    block = "perception:                # bottle detection (tune: detect_bottle --tune)\n" + "".join(
        f"  {k}: {fmt(merged[k])}\n" for k in DEFAULTS)
    pat = re.compile(r"^perception:.*\n(?:[ \t]+.*\n)*", re.M)
    text = pat.sub(block, text, count=1) if pat.search(text) else text.rstrip("\n") + "\n\n" + block
    path.write_text(text)
    return path


save_thresholds = save_settings   # old name


def annotate(frame, det: Detection | None, mapper=None, fps=None, info: str = "", anchor: str = "base"):
    if mapper:
        mapper.draw(frame)
    if det:
        cv2.rectangle(frame, (det.x, det.y), (det.x + det.w, det.y + det.h), (0, 255, 0), 2)
        ax, ay = map(int, det.anchor_px(anchor))
        cv2.circle(frame, (ax, ay), 6, (0, 255, 0), -1)
        cv2.drawMarker(frame, (ax, ay), (0, 0, 0), cv2.MARKER_CROSS, 14, 1)
        txt = det.label + (f" {det.score:.2f}" if det.score < 1 else "")
        if mapper:
            s, t = mapper.to_zone(*det.anchor_px(anchor))
            txt += f"  s={s:.2f} t={t:.2f}" + ("" if mapper.inside(s, t) else " (outside zone)")
        cv2.putText(frame, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "no bottle", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    if info:
        cv2.putText(frame, info, (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    if fps:
        cv2.putText(frame, f"{fps:.0f} fps", (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--method", choices=["background", "yolo", "color"])
    ap.add_argument("--camera", help="camera name from robot.yaml (default: perception_camera)")
    ap.add_argument("--image", help="test on an image file instead of a camera")
    ap.add_argument("--tune", action="store_true", help="sliders for the current method; s saves")
    ap.add_argument("--goto-scan", action="store_true", help="move the arm to the `scan` pose and hold")
    args = ap.parse_args(argv)

    cfg = load_yaml(args.config)
    arm = None
    if args.goto_scan:
        from pillbot.perception.scan import connect_arm_at_scan
        arm = connect_arm_at_scan(cfg)
    st = settings(cfg)
    try:
        from pillbot.perception.zone import ZoneMapper
        mapper = ZoneMapper.load()
    except FileNotFoundError:
        mapper = None
    det_fn = make_detector(cfg, args.method, mapper)
    zone_mm = st["zone_size_mm"]
    from pillbot.control.bottle_select import match_bottle

    cap, still = None, None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit(f"can't read {args.image}")
    else:
        from pillbot.perception.cameras import open_camera
        name = args.camera or cfg.get("perception_camera", "phone")
        cap = open_camera(cfg["cameras"][name])
        print(f"camera: {name}   method: {det_fn.name}")
    if det_fn.name == "background" and det_fn.background is None:
        print("No background yet: clear the zone and press b.")

    if args.tune:
        cv2.namedWindow("tune")
        cv2.createTrackbar("min area", "tune", int(st["min_area"]), 20000, lambda _: None)
        if det_fn.name == "background":
            cv2.createTrackbar("diff thresh", "tune", int(st["diff_threshold"]), 120, lambda _: None)
        elif det_fn.name == "color":
            for i, ch in enumerate("HSV"):
                mx = 179 if ch == "H" else 255
                cv2.createTrackbar(f"{ch} low", "tune", int(st["hsv_low"][i]), mx, lambda _: None)
                cv2.createTrackbar(f"{ch} high", "tune", int(st["hsv_high"][i]), mx, lambda _: None)
        print("Tune until ONLY the bottle is white in the mask. s = save, q = quit")

    def read():
        if still is not None:
            return still.copy()
        ok, f = cap.read()
        return f if ok else None

    t_prev, fps = time.time(), None
    while True:
        frame = read()
        if frame is None:
            print("camera read failed (stream busy? another viewer open?)")
            time.sleep(0.5)
            continue
        now = time.time()
        inst = 1 / max(now - t_prev, 1e-3)
        fps, t_prev = (inst if fps is None else 0.9 * fps + 0.1 * inst), now
        if args.tune:
            det_fn.st["min_area"] = max(1, cv2.getTrackbarPos("min area", "tune"))
            if det_fn.name == "background":
                det_fn.st["diff_threshold"] = cv2.getTrackbarPos("diff thresh", "tune")
            elif det_fn.name == "color":
                det_fn.st["hsv_low"] = [cv2.getTrackbarPos(f"{c} low", "tune") for c in "HSV"]
                det_fn.st["hsv_high"] = [cv2.getTrackbarPos(f"{c} high", "tune") for c in "HSV"]
            m = det_fn.mask(frame)
            if m is not None:
                cv2.imshow("mask", m)
        det = det_fn(frame)
        info = ""
        if det and mapper and zone_mm:
            wmm = width_mm(det, mapper, zone_mm, det_fn.st.get("anchor", "base"))
            b = match_bottle(wmm) if wmm else None
            info = f"~{wmm:.0f} mm wide" + (f" -> {b['name']}" if b else "")
        cv2.imshow("bottle", annotate(frame, det, mapper, None if still is not None else fps, info,
                                      det_fn.st.get("anchor", "base")))
        key = cv2.waitKey(30 if still is not None else 1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("b") and det_fn.name == "background":
            frames = [f for f in (read() for _ in range(15)) if f is not None]
            det_fn.capture(frames)
            det_fn.save()
            print(f"background captured ({len(frames)} frames) -> {BACKGROUND_PATH.relative_to(ROOT)}")
        if key == ord("p"):
            out = ROOT / "outputs" / f"snapshot_{int(time.time())}.jpg"
            cv2.imwrite(str(out), frame)
            print(f"saved {out}")
        if key == ord("s") and args.tune:
            keep = {k: det_fn.st[k] for k in ("min_area", "diff_threshold", "hsv_low", "hsv_high")}
            print(f"saved to {save_settings({**keep, 'method': det_fn.name})}")
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    if arm:
        arm.disconnect()


if __name__ == "__main__":
    main()
