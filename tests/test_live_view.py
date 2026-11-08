import json
import time
import urllib.request

import numpy as np

from pillbot.perception.detect_bottle import BackgroundDetector, settings
from pillbot.perception.shared_camera import SharedCamera
from pillbot.tools.live_view import LiveView
from pillbot.utils.config import load_yaml

CFG = load_yaml("configs/robot.yaml")


class FakeCap:
    """Stands in for the ESP32-CAM stream: grey table, bottle appears after a few frames."""

    def __init__(self):
        self.n = 0

    def read(self):
        time.sleep(0.01)
        self.n += 1
        img = np.full((240, 320, 3), 90, np.uint8)
        if self.n > 20:
            import cv2
            cv2.circle(img, (160, 120), 22, (235, 235, 235), -1)    # bottle seen from above
        return True, img

    def release(self):
        pass


def get(url, data=None):
    req = urllib.request.Request(url, data=data, method="POST" if data is not None else "GET")
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.headers.get("Content-Type"), r.read()


def test_shared_camera_gives_fresh_frames():
    cam = SharedCamera(FakeCap, "fake")
    try:
        t = time.time()
        f = cam.wait_new(t, timeout=2)
        assert f is not None and f.shape == (240, 320, 3) and cam.stamp > t
    finally:
        cam.close()


def test_live_view_pages_stream_and_background():
    cam = SharedCamera(FakeCap, "fake")
    det = BackgroundDetector(settings(CFG))
    det.save = lambda: None                                  # don't write the real background file
    view = LiveView(CFG, cam, port=0, host="127.0.0.1", detector=det, mapper=None).start()
    base = f"http://127.0.0.1:{view.port}"
    try:
        ctype, body = get(base + "/")
        assert "text/html" in ctype and b"PillBot live" in body
        ctype, body = get(base + "/background", data=b"")    # zone is still empty (first frames)
        assert "background captured" in json.loads(body)["msg"]
        time.sleep(0.4)                                       # bottle appears in the fake stream
        ctype, body = get(base + "/snapshot.jpg")
        assert ctype == "image/jpeg" and body[:2] == b"\xff\xd8"
        status = json.loads(get(base + "/status.json")[1])
        assert status["camera"] == "fake" and status["method"] == "background"
        assert status["bottle"] != "not found"
        ctype, body = get(base + "/stream?frames=2&view=mask")
        assert "multipart/x-mixed-replace" in ctype and body.count(b"--frame") == 2
    finally:
        view.stop()
        cam.close()
