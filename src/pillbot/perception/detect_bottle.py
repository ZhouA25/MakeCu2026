"""Find a pill bottle in a fixed-camera frame.

Simple HSV color detection works well for amber bottles on a plain, contrasting mat.
Upgrade to YOLO (`pip install -e .[vision]`) if you need many bottle types.

Live preview (with zone overlay if calibrated):  python -m pillbot.perception.detect_bottle
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    x: int
    y: int
    w: int
    h: int
    label: str = "bottle"

    @property
    def base_px(self) -> tuple[float, float]:
        """Bottom-center of the box ≈ where the bottle touches the table.
        With a ~45° camera the box center is too high, so use the base for zone mapping."""
        return self.x + self.w / 2, self.y + self.h


# HSV range for amber pharmacy bottles — tune under your demo lighting.
AMBER_LOW = np.array([5, 80, 60])
AMBER_HIGH = np.array([25, 255, 255])


def color_mask(frame_bgr, low, high, open_k: int = 5):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))


def detect_amber_bottle(frame_bgr, min_area: int = 1500) -> Detection | None:
    mask = color_mask(frame_bgr, AMBER_LOW, AMBER_HIGH)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None
    return Detection(*cv2.boundingRect(max(contours, key=cv2.contourArea)))


def locate_in_zone(frame_bgr, mapper) -> tuple[float, float] | None:
    """Bottle position as zone coords (s, t), or None if not found / outside the zone."""
    det = detect_amber_bottle(frame_bgr)
    if det is None:
        return None
    s, t = mapper.to_zone(*det.base_px)
    return (s, t) if mapper.inside(s, t) else None


if __name__ == "__main__":
    from pillbot.perception.cameras import grab, perception_camera
    from pillbot.perception.zone import ZoneMapper
    from pillbot.utils.config import load_yaml

    cfg = load_yaml("configs/robot.yaml")
    try:
        mapper = ZoneMapper.load()
    except FileNotFoundError:
        mapper = None
    cap = perception_camera(cfg)
    while True:
        frame = grab(cap)
        det = detect_amber_bottle(frame)
        if mapper:
            mapper.draw(frame)
        if det:
            cv2.rectangle(frame, (det.x, det.y), (det.x + det.w, det.y + det.h), (0, 255, 0), 2)
            bx, by = map(int, det.base_px)
            cv2.circle(frame, (bx, by), 6, (0, 255, 0), -1)
            if mapper:
                s, t = mapper.to_zone(*det.base_px)
                cv2.putText(frame, f"s={s:.2f} t={t:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("bottle", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
