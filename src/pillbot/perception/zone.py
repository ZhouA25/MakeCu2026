"""Map camera pixels -> position (s, t) in [0,1]^2 inside the taped start zone.

Calibrate once per camera placement:  python -m pillbot.perception.zone --calibrate
Click the 4 tape corners in order: 1 (s0,t0) -> 2 (s1,t0) -> 3 (s1,t1) -> 4 (s0,t1),
matching the corner order you used when teaching zone_* poses.
"""
import argparse

import cv2
import numpy as np
import yaml

from pillbot.utils.config import ROOT, load_yaml

CALIB_PATH = ROOT / "configs/zone_calib.local.yaml"
UNIT_SQUARE = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])


class ZoneMapper:
    def __init__(self, corners_px):
        self.corners_px = np.float32(corners_px)
        self.H = cv2.getPerspectiveTransform(self.corners_px, UNIT_SQUARE)

    @classmethod
    def load(cls, path=CALIB_PATH) -> "ZoneMapper":
        if not path.exists():
            raise FileNotFoundError("No zone calibration — run `python -m pillbot.perception.zone --calibrate`")
        return cls(yaml.safe_load(path.read_text())["corners_px"])

    def save(self, path=CALIB_PATH):
        path.write_text(yaml.safe_dump({"corners_px": self.corners_px.tolist()}))

    def to_zone(self, u: float, v: float) -> tuple[float, float]:
        pt = cv2.perspectiveTransform(np.float32([[[u, v]]]), self.H)[0, 0]
        return float(pt[0]), float(pt[1])

    def inside(self, s: float, t: float, margin: float = 0.05) -> bool:
        return -margin <= s <= 1 + margin and -margin <= t <= 1 + margin

    def draw(self, frame):
        cv2.polylines(frame, [self.corners_px.astype(np.int32)], True, (255, 0, 255), 2)
        for i, (x, y) in enumerate(self.corners_px):
            cv2.putText(frame, str(i + 1), (int(x) + 5, int(y) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        return frame


def calibrate(cfg):
    from pillbot.perception.cameras import grab, perception_camera
    cap = perception_camera(cfg)
    frame = grab(cap)
    cap.release()
    clicks = []

    def on_click(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))

    cv2.namedWindow("zone")
    cv2.setMouseCallback("zone", on_click)
    while len(clicks) < 4:
        vis = frame.copy()
        for i, c in enumerate(clicks):
            cv2.circle(vis, c, 5, (255, 0, 255), -1)
        cv2.putText(vis, f"click corner {len(clicks) + 1}/4", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.imshow("zone", vis)
        if cv2.waitKey(20) & 0xFF == ord("q"):
            return
    cv2.destroyAllWindows()
    ZoneMapper(clicks).save()
    print(f"Saved {CALIB_PATH}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true", required=True)
    ap.add_argument("--config", default="configs/robot.yaml")
    calibrate(load_yaml(ap.parse_args().config))
