import shutil

import cv2
import numpy as np
import yaml

from pillbot.perception.detect_bottle import DEFAULTS, detect_bottle, save_thresholds, thresholds
from pillbot.utils.config import ROOT, load_yaml


def synthetic_scene(x=200, y=150, w=60, h=110):
    """Grey table with an amber 'bottle' rectangle."""
    img = np.full((480, 640, 3), 90, np.uint8)
    amber_bgr = cv2.cvtColor(np.uint8([[[15, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    img[y:y + h, x:x + w] = amber_bgr
    return img


def test_detects_amber_bottle_and_base_point():
    det = detect_bottle(synthetic_scene(), thresholds(load_yaml("configs/robot.yaml")))
    assert det is not None
    assert abs(det.x - 200) <= 3 and abs(det.w - 60) <= 4
    bx, by = det.base_px
    assert abs(bx - 230) <= 3 and abs(by - 260) <= 3          # bottom-center = table contact


def test_no_bottle_on_empty_table():
    assert detect_bottle(np.full((480, 640, 3), 90, np.uint8), DEFAULTS) is None


def test_small_blobs_ignored():
    assert detect_bottle(synthetic_scene(w=10, h=10), DEFAULTS) is None


def test_save_thresholds_round_trip(tmp_path):
    p = tmp_path / "robot.yaml"
    shutil.copy(ROOT / "configs/robot.yaml", p)
    th = {"hsv_low": [8, 90, 70], "hsv_high": [22, 250, 240], "min_area": 900}
    save_thresholds(th, p)
    save_thresholds(th, p)                                    # idempotent, no duplicate block
    cfg = yaml.safe_load(p.read_text())
    assert cfg["perception"] == th
    assert p.read_text().count("perception:") == 1
    assert cfg["fixture"]["clamp_channel"] == 0 and cfg["cameras"]["wrist"]["type"] == "pillbot_ipcam"
