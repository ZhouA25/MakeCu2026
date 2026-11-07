"""Detect the bottle spinning in the clip (instead of the cap turning).

Put a strip of colored tape (default: blue) on the SIDE of the bottle body, facing the camera.
If that marker moves sideways while we twist, the clip isn't holding the bottle.
Optionally a different color (e.g. green) on the cap side lets you see the cap turning.
"""
import cv2
import numpy as np

from pillbot.perception.detect_bottle import color_mask

BLUE = (np.array([100, 120, 60]), np.array([130, 255, 255]))
GREEN = (np.array([45, 100, 60]), np.array([80, 255, 255]))


def marker_x(frame_bgr, hsv_range, roi=None, min_area: int = 80) -> float | None:
    """Horizontal centroid (px) of the largest marker blob, optionally inside roi=(x,y,w,h)."""
    x0 = y0 = 0
    if roi:
        x0, y0, w, h = roi
        frame_bgr = frame_bgr[y0:y0 + h, x0:x0 + w]
    mask = color_mask(frame_bgr, *hsv_range, open_k=3)
    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] < min_area:
        return None
    return x0 + m["m10"] / m["m00"]


class BottleSpinMonitor:
    def __init__(self, cap, roi=None, threshold_px: float = 12.0, color=BLUE):
        self.cap, self.roi, self.threshold, self.color = cap, roi, threshold_px, color
        self.start = None

    def arm(self):
        """Call once the bottle is in the clip, before twisting."""
        self.start = self._read()

    def _read(self):
        ok, frame = self.cap.read()
        return marker_x(frame, self.color, self.roi) if ok else None

    def bottle_spun(self) -> bool:
        if self.start is None:
            return False
        x = self._read()
        # marker gone (rotated out of view) or moved sideways => body rotated
        return x is None or abs(x - self.start) > self.threshold
